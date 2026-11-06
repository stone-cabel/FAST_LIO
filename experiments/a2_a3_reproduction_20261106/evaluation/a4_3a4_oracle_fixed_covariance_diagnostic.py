#!/usr/bin/env python3
"""A4-3a-4: new-run-only, oracle-fixed FAST-LIO covariance scale diagnostic.

Strictly OFFLINE. Uses PhysX GT at each replay timestamp (GT never fed to LIO).
Anchors each independent FAST-LIO trajectory at the first sample using a PhysX
pose and treats that transformation as EXACT for conditional diagnostics.
The 3x3 position covariance is rotated into the PhysX world frame.
Important: reference point (FAST-LIO IMU vs Falcon base_link_inertia), initial
anchor uncertainty, bias, and full cross-agent covariance remain unmodeled.
No calibrated NEES or MoCap-free claim is made.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT=Path.home()/"research/isaac_a1_diagnostics"
NAMES=[f"Falcon{i}_base_link_inertia" for i in (1,2,3)]

def rms_norm(v):
    return float(np.sqrt(np.mean(np.sum(np.asarray(v)**2,axis=-1))))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--root",type=Path,default=ROOT)
    ap.add_argument("--out-dir",type=Path,default=None)
    ap.add_argument("--allow-nonmatching-reference",action="store_true",
                    help="Only if IMU/base_link reference-point equivalence was independently verified")
    args=ap.parse_args()
    root=args.root.expanduser()
    out=args.out_dir.expanduser() if args.out_dir else root/"A4_3A/oracle_fixed_covariance_diagnostic"
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Refusing to overwrite existing results: {out}")
    truth=root/"a1_47_mappo_eskf_500steps_300hz.npz"
    with np.load(truth,allow_pickle=False) as d:
        tg=np.asarray(d["time_s"],float)
        names=[str(n) for n in d["body_names"]]
        poses=np.asarray(d["body_link_pose_w_xyzw_300hz"],float)
    missing=[x for x in NAMES if x not in names]
    if missing:
        raise ValueError(f"PhysX body not found: {missing}; present: {names}")
    if poses.shape!=(len(tg),len(names),7) or not np.all(np.diff(tg)>0):
        raise ValueError("Bad PhysX GT shape/timestamps")
    print("A4-3a-4 NEW-RUN ORACLE-FIXED COVARIANCE DIAGNOSTIC")
    print("GT bodies:",NAMES)
    print("Note: per-UAV GT first-frame pose used for anchor; NOT autonomous alignment.")
    if not args.allow_nonmatching_reference:
        print("REFERENCE POINT ALERT: FAST-LIO state.pos is nominal IMU origin; PhysX body is base_link_inertia.")
        print("No IMU<->base_link_inertia extrinsic/lever-arm equivalence has yet been verified.")
        print("Run the orientation-free, translation-invariant increment diagnostic below;")
        print("do NOT interpret position residual vs GT as true sensor consistency without matching reference points.")

    rows=[]; arrays={}
    for drone,name in enumerate(NAMES,start=1):
        path=root/f"A4_3A/fastlio_covariance_replay/drone_{drone}_odom_cov.npz"
        with np.load(path,allow_pickle=False) as d:
            od=np.asarray(d["odom_time_pose_xyzw"],float)
            cov=np.asarray(d["odom_pose_covariance_6x6"],float)
        if od.shape[1]!=8 or cov.shape!=(len(od),6,6) or len(od)<5:
            raise ValueError(f"UAV{drone}: bad Odom/cov shape")
        if not np.isfinite(od).all() or not np.isfinite(cov).all() or not np.all(np.diff(od[:,0])>0):
            raise ValueError(f"UAV{drone}: missing/invalid data")
        t=od[:,0]
        if t[0]<tg[0] or t[-1]>tg[-1]:
            raise ValueError(f"UAV{drone}: GT does not cover odometry timestamps")
        body=poses[:,names.index(name),:]
        pg=np.column_stack([np.interp(t,tg,body[:,j]) for j in range(3)])
        Rg=Slerp(tg,Rotation.from_quat(body[:,3:7]))(t)
        pl=od[:,1:4]; Rl=Rotation.from_quat(od[:,4:8])
        Ralign=(Rg[0]*Rl[0].inv()).as_matrix()
        talign=pg[0]-Ralign@pl[0]
        pa=(Ralign@pl.T).T+talign
        C=np.einsum("ij,njk,lk->nil",Ralign,cov[:,:3,:3],Ralign)
        C=0.5*(C+np.swapaxes(C,1,2))
        emin=np.linalg.eigvalsh(C).min()
        if emin<=0:
            raise ValueError(f"UAV{drone}: transformed 3x3 cov non-PD")
        e=pa-pg
        mahal=np.einsum("ni,ni->n",e,np.linalg.solve(C,e[:,:,None])[:,:,0])
        attitude_err=(Rg.inv()*Rotation.from_matrix(Ralign)*Rl).magnitude()
        # Relative position error increments: anchor translation cancels.
        p_inc=np.diff(pa,axis=0)-np.diff(pg,axis=0)
        print(f"UAV{drone}: N={len(t)} t={t[0]:.6f}..{t[-1]:.6f}s, "
              f"GT-anchor conditional position RMS={rms_norm(e):.6f}m, "
              f"mean Mahalanobis²={np.mean(mahal):.3f}, "
              f"median={np.median(mahal):.3f}, max={np.max(mahal):.3f}")
        print(f"          orientation RMS={np.degrees(np.sqrt(np.mean(attitude_err**2))):.4f}deg, "
              f"position increment residual RMS={rms_norm(p_inc):.6f}m")
        row={
            "drone":drone,
            "physx_body":name,
            "samples":len(t),
            "timestamp_first_s":float(t[0]),
            "timestamp_last_s":float(t[-1]),
            "conditional_oracle_anchor_position_RMSE_m":rms_norm(e),
            "orientation_RMS_deg":float(np.degrees(np.sqrt(np.mean(attitude_err**2)))),
            "mean_conditional_position_mahalanobis2":float(np.mean(mahal)),
            "median_conditional_position_mahalanobis2":float(np.median(mahal)),
            "max_conditional_position_mahalanobis2":float(np.max(mahal)),
            "position_increment_residual_RMSE_m":rms_norm(p_inc),
            "max_cov_asymmetry":float(np.max(np.abs(C-np.swapaxes(C,1,2)))),
            "min_position_cov_eigenvalue":float(emin),
            "anchor_R_lio_to_physx":Ralign.tolist(),
            "anchor_t_lio_to_physx_m":talign.tolist(),
        }
        rows.append(row)
        arrays[f"uav{drone}_time_s"]=t
        arrays[f"uav{drone}_position_error_conditional_W_m"]=e
        arrays[f"uav{drone}_position_cov_conditional_W_m2"]=C
        arrays[f"uav{drone}_position_mahalanobis2_conditional"]=mahal
        arrays[f"uav{drone}_orientation_error_rad"]=attitude_err
    out.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(out/"a4_3a4_oracle_fixed_covariance_arrays.npz",**arrays)
    report={
        "status":"OFFLINE DIAGNOSTIC ONLY; NOT CALIBRATED ESTIMATOR CONSISTENCY",
        "method":"Use matched new replay LIO pose/cov, interpolate same PhysX sequence GT to LIO timestamps; fix single rigid transform per UAV from FIRST GT and LIO pose.",
        "reference_point_caveat":"LIO IMU origin versus PhysX Falcon*_base_link_inertia pose must be checked; any IMU-to-body lever arm creates orientation-dependent position errors.",
        "frame_caveat":"Only 3x3 world-frame position covariance rotated; 6x6 rotation tangent/cross-covariance remains unvalidated.",
        "alignment_caveat":"GT-assisted first-frame transformation is treated as fixed exactly; no anchor uncertainty propagated; statistics not a valid strict NEES test.",
        "statistics_caveat":"47 temporally correlated frames and one 5s trial; no chi-square confidence intervals or multi-seed calibration conclusions.",
        "rows":rows,
    }
    (out/"a4_3a4_oracle_fixed_covariance_summary.json").write_text(
        json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("Saved:",out)
    print("Do NOT report conditional Mahalanobis2 as a statistically validated NEES.")

if __name__=="__main__":
    main()
