#!/usr/bin/env python3
"""A4-2g: describe which 3-UAV relative position-error directions matter to Kabsch.

Uses A4-2f measured 3x9 finite-difference Jacobians. Not a fresh estimation
run and not an observability proof. No GT payload pose is accessed.
"""
import argparse
import json
from pathlib import Path
import numpy as np

HOME=Path.home()/"research"

def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x,dtype=float)**2)))

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--root",type=Path,default=HOME/"isaac_a1_diagnostics")
    p.add_argument("--out-dir",type=Path,default=None)
    a=p.parse_args()
    root=a.root.expanduser()
    out=a.out_dir.expanduser() if a.out_dir else root/"A4_2G/directional_position_sensitivity"
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Output directory already contains files, refuse overwrite: {out}")

    inp=root/"A4_2F/kabsch_local_jacobian/a4_2f_kabsch_sensitivity.npz"
    positions=root/"A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz"
    with np.load(inp,allow_pickle=False) as z:
        J=np.asarray(z["Jacobian_load_body_rad_per_m"],float)
        t=np.asarray(z["time_s"],float)
        exact=np.asarray(z["rotation_change_exact_body_rad"],float)
        predicted_old=np.asarray(z["rotation_change_predicted_body_rad"],float)
    with np.load(positions,allow_pickle=False) as z:
        gt=np.asarray(z["uav_position_gt_W_m"],float)
        lio=np.asarray(z["uav_position_lio_W_m"],float)
    n=len(t)
    if J.shape!=(n,3,9) or gt.shape!=(n,3,3) or lio.shape!=gt.shape or exact.shape!=(n,3) or predicted_old.shape!=(n,3):
        raise ValueError("Input array shape mismatch")
    if not all(np.isfinite(v).all() for v in (J,t,gt,lio,exact,predicted_old)):
        raise ValueError("Nonfinite inputs")
    delta=lio-gt
    diff=delta-delta.mean(axis=1,keepdims=True)
    d=diff.reshape(n,9)
    full_norm=np.linalg.norm(d,axis=1)

    row_norm=[]; null_norm=[]; singular=[]; projection_check=[]
    gains=[]; bounds=[]; singular_values=[]
    for i in range(n):
        U,S,Vt=np.linalg.svd(J[i],full_matrices=True)
        # The row space of a rank-3 Jacobian is 3D in R^9.
        if S[-1] < 1e-9:
            raise ValueError(f"Jacobian not rank-3 at sample {i}: {S}")
        vrow=Vt[:3,:]
        drow=vrow.T@(vrow@d[i])
        dnull=d[i]-drow
        row_norm.append(np.linalg.norm(drow))
        null_norm.append(np.linalg.norm(dnull))
        proj=J[i]@drow
        naive=J[i]@d[i]
        projection_check.append(np.max(np.abs(proj-naive)))
        old=np.max(np.abs(naive-predicted_old[i]))
        if old > 1e-8:
            raise AssertionError(f"A4-2f predicted Jacobian output mismatch at {i}: {old}")
        singular_values.append(S)
        gain=float(np.linalg.norm(naive)/max(np.linalg.norm(d[i]),1e-12))
        gains.append(gain)
        bounds.append(S[0]*np.linalg.norm(d[i]))
    row_norm=np.asarray(row_norm);null_norm=np.asarray(null_norm)
    gains=np.asarray(gains);bounds=np.asarray(bounds)
    S=np.asarray(singular_values)
    exact_mag=np.linalg.norm(exact,axis=1)
    pred_mag=np.linalg.norm(predicted_old,axis=1)
    print("A4-2g POSITION ERROR DIRECTION PROJECTION — NO NEW ESKF RUN")
    print(f"n={n}, t={t[0]:.3f}..{t[-1]:.3f}s")
    print(f"relative delta 9D norm RMS={rms(full_norm):.6f}m")
    print(f"sensitive row-space delta norm RMS={rms(row_norm):.6f}m")
    print(f"first-order null-space delta norm RMS={rms(null_norm):.6f}m")
    print(f"row-space energy fraction={float(np.sum(row_norm**2)/np.sum(full_norm**2)):.6f}")
    print(f"effective actual-direction gain RMS={rms(gains):.6f}rad/m; "
          f"mean={float(np.mean(gains)):.6f}rad/m")
    print(f"max singular value mean={float(np.mean(S[:,0])):.6f}rad/m "
          f"(worst direction local bound)")
    print(f"predicted attitude change RMS={np.degrees(rms(pred_mag)):.6f}deg")
    print(f"worst-case per-frame bound RMS={np.degrees(rms(bounds)):.6f}deg")
    print(f"observed exact Kabsch change RMS={np.degrees(rms(exact_mag)):.6f}deg")
    print(f"row projection identity max |J delta - J delta_row|={max(projection_check):.3e}rad")
    print("CAVEAT: J row-space and null-space are LOCAL first-order decompositions;"
          " null component can change finite-perturbation Kabsch solution at second order.")

    report={
        "n":n,"start_s":float(t[0]),"end_s":float(t[-1]),
        "relative_position_9d_norm_rms_m":rms(full_norm),
        "jacobian_rowspace_error_norm_rms_m":rms(row_norm),
        "jacobian_nullspace_error_norm_rms_m":rms(null_norm),
        "rowspace_fraction_of_squared_relative_error":float(np.sum(row_norm**2)/np.sum(full_norm**2)),
        "actual_direction_local_gain_rms_rad_per_m":rms(gains),
        "actual_direction_local_gain_mean_rad_per_m":float(np.mean(gains)),
        "max_singular_value_mean_rad_per_m":float(np.mean(S[:,0])),
        "linear_predicted_rotation_rms_deg":float(np.degrees(rms(pred_mag))),
        "worst_case_linear_bound_rms_deg":float(np.degrees(rms(bounds))),
        "exact_kabsch_rotation_rms_deg":float(np.degrees(rms(exact_mag))),
        "max_rowprojection_identity_residual_rad":float(np.max(projection_check)),
        "limits":[
            "Local 3x9 Kabsch Jacobian analysis, not dynamic EKF observability.",
            "Row-space projection and nullspace only first-order; finite perturbations may have second-order effects.",
            "No control intervention, performance improvement, real sensor or multi-seed statistical conclusion.",
            "A4-1 alignment uses GT oracle; position errors are 46 temporally-correlated measurements."
        ],
    }
    out.mkdir(parents=True,exist_ok=True)
    js=out/"a4_2g_projection_summary.json"
    npz=out/"a4_2g_projection_arrays.npz"
    np.savez_compressed(npz,time_s=t,singular_values_rad_per_m=S,
        relative_delta_9d_norm_m=full_norm,
        rowspace_delta_9d_norm_m=row_norm,
        nullspace_delta_9d_norm_m=null_norm,
        effective_local_gain_rad_per_m=gains,
        worstcase_firstorder_rotation_bound_rad=bounds,
        predicted_rotation_change_rad=pred_mag,
        exact_rotation_change_rad=exact_mag)
    js.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("Saved",npz)
    print("Saved",js)

if __name__=="__main__":
    main()
