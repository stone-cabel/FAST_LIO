#!/usr/bin/env python3
"""A4-2f: local Kabsch attitude Jacobian vs 3-UAV position errors.

Read-only experiment using *exact* kabsch_initializer and imu_cable_observer
from existing Sun-inspired project. No load GT is fed to the estimator.
All comparisons use frozen UAV attitude, IMU specific force and collective
rotor thrust; only UAV positions are perturbed. This is geometry-only,
NOT an ESKF dynamics or full-system observability proof.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path.home() / "research"


def rotation_log_between(R_from, R_to):
    return Rotation.from_matrix(R_from.T @ R_to).as_rotvec()


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x, dtype=float)**2)))


def coefficient(x, y):
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    if np.std(x)<1e-12 or np.std(y)<1e-12:
        return None
    return float(np.corrcoef(x,y)[0,1])


def jacobian_kabsch(rho, contacts, directions, rope_length, epsilon):
    from payload_ekf_sun.kabsch_initializer import estimate_payload_pose_kabsch
    _, R0, fit = estimate_payload_pose_kabsch(rho, contacts, directions, rope_length)
    J = np.empty((3,9),dtype=float)
    for k in range(9):
        dp=np.zeros((3,3),dtype=float)
        dp.flat[k]=epsilon
        _, Rp, _ = estimate_payload_pose_kabsch(rho, contacts+dp, directions, rope_length)
        _, Rm, _ = estimate_payload_pose_kabsch(rho, contacts-dp, directions, rope_length)
        # Local right perturbation about R0 in the LOAD frame.
        J[:,k]=(rotation_log_between(R0,Rp)-rotation_log_between(R0,Rm))/(2*epsilon)
    return R0,fit,J


def main():
    a=argparse.ArgumentParser()
    a.add_argument("--root",type=Path,default=ROOT/"isaac_a1_diagnostics")
    a.add_argument("--sun-root",type=Path,default=ROOT/"payload_ekf_sun_E1")
    a.add_argument("--epsilon-m",type=float,default=1e-5)
    a.add_argument("--out-dir",type=Path,default=None)
    args=a.parse_args()
    if not 1e-7 <= args.epsilon_m <= 1e-3:
        a.error("epsilon-m must be in [1e-7,1e-3]")
    root=args.root.expanduser()
    sun=args.sun_root.expanduser()
    dest=args.out_dir.expanduser() if args.out_dir else root/"A4_2F/kabsch_local_jacobian"
    if dest.exists() and any(dest.iterdir()):
        raise FileExistsError(f"Refusing overwrite: {dest}")
    sys.path.insert(0,str(sun/"src"))
    from payload_ekf_sun.streaming_zeng_eskf import (
        RHO_FROM_LOAD_COM, UAV_ATTACHMENT_FROM_BASE_COM,
        quat_xyzw_to_rot, StreamingZengESKF,
    )
    from payload_ekf_sun.imu_cable_observer import invert_cable_force
    from payload_ekf_sun.kabsch_initializer import estimate_payload_pose_kabsch

    file=root/"A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz"
    with np.load(file,allow_pickle=False) as d:
        t=np.asarray(d["time_s"],float)
        p_gt=np.asarray(d["uav_position_gt_W_m"],float)
        p_lio=np.asarray(d["uav_position_lio_W_m"],float)
        q_gt=np.asarray(d["uav_quat_gt_WB_xyzw"],float)
        sf=np.asarray(d["imu_specific_force_B_mps2"],float)
        thrust=np.asarray(d["rotor_collective_thrust_N"],float)
    N=len(t)
    if (N!=46 or p_gt.shape!=(N,3,3) or p_lio.shape!=p_gt.shape
        or q_gt.shape!=(N,3,4) or sf.shape!=(N,3,3) or thrust.shape!=(N,3)):
        raise ValueError("Invalid A4-2B archive geometry")
    if not all(np.isfinite(x).all() for x in (t,p_gt,p_lio,q_gt,sf,thrust)):
        raise ValueError("Nonfinite data")
    if not np.allclose(np.diff(t),.1,atol=1e-8):
        raise ValueError("Expected A4-2B shared10Hz timestamps")

    cfg=StreamingZengESKF()
    rho=RHO_FROM_LOAD_COM
    lever=UAV_ATTACHMENT_FROM_BASE_COM
    length=cfg.rope_length
    J_all=[]; singular=[]; rot_exact=[]; rot_linear=[]; rel_norm=[]
    exact_norm=[]; linear_norm=[]; residual=[]; fits_gt=[]; fits_p=[]
    jac_eps_consistency=[]; common_null=[]
    for k in range(N):
        # Use GT UAV orientation in BOTH modes, as prior A4-2d P arm.
        Rb=quat_xyzw_to_rot(q_gt[k])
        _,direction,_=invert_cable_force(sf[k],Rb,thrust[k],cfg.mass_uav)
        contacts=p_gt[k]+np.einsum("nij,j->ni",Rb,lever)
        R0,fit0,J=jacobian_kabsch(rho,contacts,direction,length,args.epsilon_m)
        _,_,J_half=jacobian_kabsch(rho,contacts,direction,length,args.epsilon_m/2)
        eps_diff=float(np.max(np.abs(J-J_half)))
        jac_eps_consistency.append(eps_diff)
        shift=p_lio[k]-p_gt[k]
        common=shift.mean(axis=0)
        relative=shift-common[None,:]
        # Test the 3D common-translation nullspace in the actual code.
        C=np.kron(np.ones((3,1)),np.eye(3))  # 9 x 3
        null=float(np.linalg.norm(J@C,ord=2))
        common_null.append(null)
        J_all.append(J)
        singular.append(np.linalg.svd(J,compute_uv=False))
        _,RP,fitP=estimate_payload_pose_kabsch(
            rho,contacts+relative,direction,length
        )
        actual=rotation_log_between(R0,RP)
        predicted=J@relative.reshape(9)
        actual_all=rotation_log_between(R0,
            estimate_payload_pose_kabsch(rho,contacts+shift,direction,length)[1])
        if np.linalg.norm(actual-actual_all)>1e-6:
            raise AssertionError("Kabsch R differs after common translation")
        rot_exact.append(actual)
        rot_linear.append(predicted)
        rel_norm.append(float(np.linalg.norm(relative)))
        exact_norm.append(float(np.linalg.norm(actual)))
        linear_norm.append(float(np.linalg.norm(predicted)))
        residual.append(float(np.linalg.norm(actual-predicted)))
        fits_gt.append(fit0)
        fits_p.append(fitP)
    J_all=np.asarray(J_all)
    singular=np.asarray(singular)
    rel_norm=np.asarray(rel_norm)
    exact_norm=np.asarray(exact_norm)
    linear_norm=np.asarray(linear_norm)
    residual=np.asarray(residual)
    common_null=np.asarray(common_null)
    jac_eps_consistency=np.asarray(jac_eps_consistency)

    # Verify that our exact Kabsch invocation is the same as the already
    # accepted StreamingZengESKF study, using its stored fit residuals.
    comparison=root/"A4_2D/pose_rotation_2x2_stride1/a4_2d_2x2_eskf_outputs.npz"
    prior_fit={}
    if not comparison.is_file():
        raise FileNotFoundError(f"Need previous A4-2d for implementation parity: {comparison}")
    with np.load(comparison,allow_pickle=False) as d:
        for key,computed in (("B0_GT_P_GT_R_kabsch_fit_m",np.asarray(fits_gt)),
                             ("P_LIO_P_GT_R_kabsch_fit_m",np.asarray(fits_p))):
            former=np.asarray(d[key],float)
            delta=float(np.max(np.abs(former-computed)))
            prior_fit[key]=delta
            if delta>1e-8:
                raise ValueError(f"Parity check mismatch for Kabsch fits {key}: {delta}")

    print("A4-2f TRUE IMPLEMENTATION KABSCH JACOBIAN AUDIT")
    print(f"samples={N}, time={t[0]:.3f}..{t[-1]:.3f}s, finite difference eps={args.epsilon_m:g}m")
    print(f"A4-2d Kabsch fit parity MAX={max(prior_fit.values()):.9g}m")
    print(f"max shared-translation nullspace residual ||J C||2={np.max(common_null):.9g}rad/m")
    print(f"max relative error in J(eps) vs J(eps/2)={np.max(jac_eps_consistency):.9g}rad/m (absolute)")
    print(f"Relative UAV pose position perturbation, 9D RMS-norm={rms(rel_norm):.6f}m")
    print(f"Jacobian largest singular value median={np.median(singular[:,0]):.6f}rad/m,"
          f" max={np.max(singular[:,0]):.6f}rad/m")
    print(f"GT->relative-only P Kabsch attitude change RMS={np.degrees(rms(exact_norm)):.6f}deg")
    print(f"Linearized attitude change RMS={np.degrees(rms(linear_norm)):.6f}deg")
    print(f"Linear approximation residual RMS={np.degrees(rms(residual)):.6f}deg")
    print(f"Correlation (exact vs linear angle norms): {coefficient(exact_norm,linear_norm)}")
    print("NOTE: Jacobian is 3x9 rad/m; rank/SVD is local pose-fit sensitivity, not full-system observability.")

    dest.mkdir(parents=True,exist_ok=True)
    out_npz=dest/"a4_2f_kabsch_sensitivity.npz"
    out_json=dest/"a4_2f_kabsch_summary.json"
    np.savez_compressed(out_npz,time_s=t,Jacobian_load_body_rad_per_m=J_all,
        singular_values_rad_per_m=singular,rotation_change_exact_body_rad=np.asarray(rot_exact),
        rotation_change_predicted_body_rad=np.asarray(rot_linear),
        relative_position_delta_norm_m=rel_norm,
        exact_rotation_delta_norm_rad=exact_norm,
        predicted_rotation_delta_norm_rad=linear_norm,
        linearization_residual_norm_rad=residual,
        common_translation_nullspace_norm_rad_per_m=common_null,
        jacobian_epsilon_half_max_abs_diff_rad_per_m=jac_eps_consistency,
        kabsch_fit_B0_m=np.asarray(fits_gt),kabsch_fit_P_m=np.asarray(fits_p))
    outcome={
        "experiment":"A4-2f exact Kabsch geometry local Jacobian and differential UAV pose error",
        "samples":N,"time_range_s":[float(t[0]),float(t[-1])],
        "epsilon_m":args.epsilon_m,
        "a4_2d_kabsch_parity_max_abs_m":prior_fit,
        "max_common_translation_nullspace_norm_rad_per_m":float(np.max(common_null)),
        "max_jacobian_eps_vs_half_abs_rad_per_m":float(np.max(jac_eps_consistency)),
        "relative_9d_perturbation_norm_rms_m":rms(rel_norm),
        "jacobian_sigma_max_median_rad_per_m":float(np.median(singular[:,0])),
        "jacobian_sigma_max_peak_rad_per_m":float(np.max(singular[:,0])),
        "exact_Kabsch_rotation_change_rms_deg":float(np.degrees(rms(exact_norm))),
        "linearized_rotation_change_rms_deg":float(np.degrees(rms(linear_norm))),
        "linearization_residual_rms_deg":float(np.degrees(rms(residual))),
        "exact_vs_linear_rotation_magnitude_correlation":coefficient(exact_norm,linear_norm),
        "caveats":[
            "UAV orientations, ideal synthetic IMU and paired thrust frozen to GT orientation-mode; only UAV relative position perturbed.",
            "Uses exact existing cable inversion and Kabsch code; no ESKF state recursion in this experiment.",
            "Only local finite-difference Jacobian; finite-position errors can violate first-order approximation.",
            "SVD of H for three planar load attachment points is rank2; do not treat H rank2 as failure, nor call J singular values global observability.",
            "A4-1 common frame was initially aligned with PhysX truth; no MoCap-free multi-UAV frame alignment."
        ]
    }
    out_json.write_text(json.dumps(outcome,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("Saved:",out_npz)
    print("Saved:",out_json)


if __name__=="__main__":
    main()
