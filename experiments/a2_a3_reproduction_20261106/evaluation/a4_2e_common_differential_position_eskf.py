#!/usr/bin/env python3
"""A4-2e split three-UAV position errors into common displacement vs shape change.

Four controlled modes (UAV orientations held at original GT across modes):
B0: GT positions
C: GT positions + mean across-UAV LIO position error (common shift)
D: GT positions + each UAV's differential error (relative geometry only)
P: LIO positions = C + D by identity

All use exactly the SAME IMU, thrust, timeline, pose_stride and ESKF settings.
The PhysX GT is used for controlled offline counterfactuals and held-out
evaluation; the output is NOT a self-contained MoCap-free estimator.
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np

HOME=Path.home()/"research"


def load_a4_2c(script):
    spec=importlib.util.spec_from_file_location("a4_2c_shared",script)
    module=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    return module


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--root",type=Path,default=HOME/"isaac_a1_diagnostics")
    p.add_argument("--sun-root",type=Path,default=HOME/"payload_ekf_sun_E1")
    p.add_argument("--pose-stride",type=int,default=1)
    p.add_argument("--out-dir",type=Path,default=None)
    args=p.parse_args()
    if args.pose_stride < 1: p.error("--pose-stride must be >=1")
    root=args.root.expanduser()
    sun=args.sun_root.expanduser()
    out=(args.out_dir.expanduser() if args.out_dir else
         root/f"A4_2E/common_differential_stride{args.pose_stride}")
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Refusing to overwrite output: {out}")
    helper=load_a4_2c(Path(__file__).with_name("a4_2c_run_paired_10hz_eskf.py"))
    sys.path.insert(0,str(sun/"src"))
    from payload_ekf_sun.streaming_zeng_eskf import StreamingZengESKF

    inputs=root/"A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz"
    with np.load(inputs,allow_pickle=False) as z:
        t=np.asarray(z["time_s"],float)
        pg=np.asarray(z["uav_position_gt_W_m"],float)
        pl=np.asarray(z["uav_position_lio_W_m"],float)
        qg=np.asarray(z["uav_quat_gt_WB_xyzw"],float)
        sf=np.asarray(z["imu_specific_force_B_mps2"],float)
        gyro=np.asarray(z["gyro_B_radps"],float)
        thrust=np.asarray(z["rotor_collective_thrust_N"],float)
    N=len(t)
    if N!=46 or pg.shape!=(N,3,3) or pl.shape!=(N,3,3) or qg.shape!=(N,3,4):
        raise ValueError("Expected A4-2B 46 samples and three UAV poses")
    if sf.shape!=(N,3,3) or gyro.shape!=(N,3,3) or thrust.shape!=(N,3):
        raise ValueError("Invalid physics array shapes")
    if not all(np.isfinite(x).all() for x in (t,pg,pl,qg,sf,gyro,thrust)):
        raise ValueError("Nonfinite inputs")
    if not np.allclose(np.diff(t),.1,atol=1e-8):
        raise ValueError("Expected uniform10Hz sampling")

    delta=pl-pg
    common=delta.mean(axis=1,keepdims=True)  # N x 1 x 3
    diff=delta-common
    if np.max(np.abs(diff.sum(axis=1)))>1e-10:
        raise AssertionError("Differential errors must sum to zero across three UAVs")
    p_c=pg+common
    p_d=pg+diff
    if np.max(np.abs(p_c+diff-pl))>1e-10:
        raise AssertionError("Exact common/differential decomposition failed")

    modes=[
        ("B0_GT",pg),("C_COMMON",p_c),
        ("D_RELATIVE",p_d),("P_ALL_LIO_POSITION",pl)
    ]
    estimates={}
    print("A4-2e common-mode vs relative-configuration UAV position error ablation",flush=True)
    print(f"N={N} time={t[0]:.3f}..{t[-1]:.3f} s, pose_stride={args.pose_stride}",flush=True)
    for name,pos in modes:
        result=helper.run_streaming(name,t,pos,qg,sf,gyro,thrust,
                                    args.pose_stride,StreamingZengESKF)
        estimates[name]=result
        print("FILTER_OK",name,"(payload truth not loaded)",flush=True)
    # Compare accepted prior A4-2d independent run (B0/P arms) bit-for-bit
    benchmark=root/f"A4_2D/pose_rotation_2x2_stride{args.pose_stride}/a4_2d_2x2_eskf_outputs.npz"
    parity={}
    if not benchmark.exists():
        raise FileNotFoundError(f"Required earlier A4-2d parity result missing: {benchmark}")
    with np.load(benchmark,allow_pickle=False) as z:
        for old,new in (("B0_GT_P_GT_R","B0_GT"),("P_LIO_P_GT_R","P_ALL_LIO_POSITION")):
            for key,label in (("p","load_position_W_m"),("R","load_rotation_L_to_W"),
                              ("v","load_velocity_W_mps"),("w","load_omega_L_radps")):
                b=np.asarray(z[f"{old}_{label}"],float)
                err=float(np.max(np.abs(b-estimates[new][key])))
                parity[f"{old}_{label}"]=err
                if err>1e-8:
                    raise ValueError(f"A4-2d previous run parity failed for {old}_{label}: {err}")
    print("Independent A4-2d B0/P parity: PASS")

    # Payload truth only after all 4 filter calculations, as A4-2c.
    truth_path=sun/"results/E7_1_E/seed42_smooth-x_clean_truth_v1.json"
    truth=helper.load_truth_after_filters(truth_path,t)
    scores={name:helper.evaluate(est,*truth) for name,est in estimates.items()}
    common_norm=np.linalg.norm(common[:,0,:],axis=-1)
    diff_norm=np.linalg.norm(diff,axis=-1)
    rms=lambda a:float(np.sqrt(np.mean(np.square(a))))
    print("="*85)
    print(f"Common translation RMS={rms(common_norm):.6f}m, "
          f"per-UAV relative residual RMS={rms(diff_norm):.6f}m")
    for name,_ in modes:
        s=scores[name]
        print(f"{name:20s} load position RMSE={s['pos_rmse_m']:.6f}m, "
              f"attitude RMSE={s['rot_rmse_deg']:.6f}deg, "
              f"velocity RMSE={s['vel_rmse_mps']:.6f}m/s, "
              f"bodyrate RMSE={s['bodyrate_rmse_radps']:.6f}rad/s, "
              f"Kabsch-fit={s['kabsch_fit_mean_m']:.6f}m")
    print("NOTE: This separates position error geometry only, not root cause, not full observability proof.")
    out.mkdir(parents=True,exist_ok=True)
    out_npz=out/"a4_2e_common_differential_eskf_outputs.npz"
    out_json=out/"a4_2e_common_differential_summary.json"
    arrays={"time_s":t,"uav_error_common_W_m":common[:,0,:],
            "uav_error_differential_W_m":diff,
            "uav_error_common_norm_m":common_norm,
            "uav_error_differential_norm_m":diff_norm,
            "truth_load_position_W_m":truth[0],
            "truth_load_quaternion_W_xyzw":truth[1],
            "truth_load_velocity_W_mps":truth[2],
            "truth_load_omega_W_radps":truth[3]}
    for name,_ in modes:
        e=estimates[name]; sc=scores[name]
        arrays[f"{name}_load_p_W_m"]=e["p"]
        arrays[f"{name}_load_R_L_W"]=e["R"]
        arrays[f"{name}_load_v_W_mps"]=e["v"]
        arrays[f"{name}_load_omega_L_radps"]=e["w"]
        arrays[f"{name}_load_position_error_m"]=sc["errors"]["pos_m"]
        arrays[f"{name}_load_rot_error_deg"]=sc["errors"]["rot_deg"]
        arrays[f"{name}_kabsch_fit_m"]=e["kabsch_fit_m"]
    np.savez_compressed(out_npz,**arrays)
    jsonout={
        "experiment":"A4-2e common translation vs differential 3-UAV position error",
        "times_s":[float(t[0]),float(t[-1])],"n":N,
        "pose_stride":args.pose_stride,
        "common_displacement_rms_m":rms(common_norm),
        "differential_per_uav_error_rms_m":rms(diff_norm),
        "prior_A4_2d_B0_P_parity_max_abs":parity,
        "scores":{k:{kk:vv for kk,vv in v.items() if kk!="errors"} for k,v in scores.items()},
        "limitations":[
            "GT UAV positions/attitudes are used in counterfactual interventions, and initial LIO frame uses oracle GT.",
            "All 4 modes share same GT UAV attitude and same pose-derived virtual IMU/real-recorded collective thrust.",
            "Common translation should leave Kabsch attitude invariant under consistent geometry, but ESKF state can still have numerical effects.",
            "This is a fixed-trajectory 10Hz offline test, not independently aligned multi-UAV estimation or physical IMU.",
            "Do not claim statistical observability or exact Sun authors' EKF."
        ]}
    out_json.write_text(json.dumps(jsonout,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print("Saved",out_npz)
    print("Saved",out_json)

if __name__=="__main__":
    main()
