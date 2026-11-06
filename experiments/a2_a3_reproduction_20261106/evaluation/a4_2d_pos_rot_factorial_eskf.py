#!/usr/bin/env python3
"""A4-2d 2x2 UAV position/attitude source ablation into existing Load ESKF.

B0=(GT pos,GT rot); P=(LIO pos,GT rot); R=(GT pos,LIO rot);
B1=(LIO pos,LIO rot). All four use the SAME timestamps, pose_stride,
synthetic body specific force and verified paired collective thrust.
Held-out load truth is loaded after filter runs. Do not overwrite A4-2c.
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

HOME = Path.home() / "research"


def load_base_module(path):
    spec = importlib.util.spec_from_file_location("a4_2c_original", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def stat_diff(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    return float(np.max(np.abs(a - b)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=HOME / "isaac_a1_diagnostics")
    parser.add_argument("--sun-root", type=Path, default=HOME / "payload_ekf_sun_E1")
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--pose-stride", type=int, default=1)
    args = parser.parse_args()
    if args.pose_stride < 1:
        parser.error("pose-stride must be >= 1")
    root = args.root.expanduser()
    sun = args.sun_root.expanduser()
    out = args.out_dir.expanduser() if args.out_dir else root / f"A4_2D/pose_rotation_2x2_stride{args.pose_stride}"
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Output directory not empty; refusing overwrite: {out}")

    old_script = Path(__file__).with_name("a4_2c_run_paired_10hz_eskf.py")
    previous = load_base_module(old_script)
    sys.path.insert(0, str(sun / "src"))
    from payload_ekf_sun.streaming_zeng_eskf import StreamingZengESKF

    p = root / "A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz"
    with np.load(p, allow_pickle=False) as data:
        t = np.asarray(data["time_s"],dtype=float)
        p_gt = np.asarray(data["uav_position_gt_W_m"],dtype=float)
        q_gt = np.asarray(data["uav_quat_gt_WB_xyzw"],dtype=float)
        p_lio = np.asarray(data["uav_position_lio_W_m"],dtype=float)
        q_lio = np.asarray(data["uav_quat_lio_WB_xyzw"],dtype=float)
        sf = np.asarray(data["imu_specific_force_B_mps2"],dtype=float)
        gyro = np.asarray(data["gyro_B_radps"],dtype=float)
        thrust = np.asarray(data["rotor_collective_thrust_N"],dtype=float)
    if (t.shape != (46,) or p_gt.shape != (46,3,3) or p_lio.shape != p_gt.shape
        or q_gt.shape != (46,3,4) or q_lio.shape != q_gt.shape
        or sf.shape != p_gt.shape or gyro.shape != p_gt.shape
        or thrust.shape != (46,3)):
        raise ValueError("Unexpected A4-2B archive shapes")
    if not all(np.isfinite(x).all() for x in (t,p_gt,p_lio,q_gt,q_lio,sf,gyro,thrust)):
        raise ValueError("Nonfinite archive data")
    if not np.allclose(np.diff(t), .1, atol=1e-8):
        raise ValueError("10Hz shared timeline required")
    if np.max(np.abs(np.linalg.norm(q_gt,axis=-1)-1)) > 1e-3:
        raise ValueError("GT quaternion norm invalid")
    if np.max(np.abs(np.linalg.norm(q_lio,axis=-1)-1)) > 1e-3:
        raise ValueError("LIO quaternion norm invalid")

    modes = (
        ("B0_GT_P_GT_R",p_gt,q_gt),
        ("P_LIO_P_GT_R",p_lio,q_gt),
        ("R_GT_P_LIO_R",p_gt,q_lio),
        ("B1_LIO_P_LIO_R",p_lio,q_lio),
    )
    estimates={}
    print(f"A4-2d 2x2 causal input ablation: samples={len(t)}, t={t[0]:.3f}..{t[-1]:.3f}, stride={args.pose_stride}",flush=True)
    for name,pos,quat in modes:
        est=previous.run_streaming(name,t,pos,quat,sf,gyro,thrust,
                                   args.pose_stride, StreamingZengESKF)
        estimates[name]=est
        print("FINISHED",name,"(payload GT not loaded)",flush=True)

    # Payload truth can only be read AFTER every ESKF mode is complete.
    truth_json = sun/"results/E7_1_E/seed42_smooth-x_clean_truth_v1.json"
    p_true,q_true,v_true,w_true = previous.load_truth_after_filters(truth_json,t)
    scores = {name:previous.evaluate(estimates[name],p_true,q_true,v_true,w_true)
              for name,_,_ in modes}

    # Compare B0 and B1 with separately executed accepted A4-2c archive.
    parity = {}
    original_path = root / f"A4_2C/paired_10hz_stride{args.pose_stride}/a4_2c_b0_b1_load_eskf_outputs.npz"
    if original_path.is_file():
        with np.load(original_path, allow_pickle=False) as data:
            for label, current in (("b0",estimates["B0_GT_P_GT_R"]),("b1",estimates["B1_LIO_P_LIO_R"])):
                for suffix, key in (("load_position_W_m","p"),("load_velocity_W_mps","v"),
                                    ("load_rotation_L_to_W","R"),("load_omega_L_radps","w")):
                    ref = np.asarray(data[f"{label}_{suffix}"],float)
                    delta = stat_diff(ref,current[key])
                    parity[f"{label}_{suffix}"] = delta
                    if delta > 1e-8:
                        raise ValueError(f"A4-2d vs A4-2c parity failure {label} {suffix}: {delta}")
        print("A4-2c B0/B1 exact-run parity: PASS")
    else:
        print("NOTICE: A4-2c baseline not found; independent parity test unavailable")

    metric_keys = ("pos_rmse_m","rot_rmse_deg","vel_rmse_mps","bodyrate_rmse_radps")
    print("="*84)
    for name,_,_ in modes:
        m=scores[name]
        print(f"{name:18s} pos={m['pos_rmse_m']:.6f} m, rot={m['rot_rmse_deg']:.6f} deg, "
              f"vel={m['vel_rmse_mps']:.6f} m/s, bodyrate={m['bodyrate_rmse_radps']:.6f} rad/s, "
              f"Kabsch fit={m['kabsch_fit_mean_m']:.6f}m")
    print("Difference from B0 (RMSE deltas, NOT per-sample absolute errors):")
    baseline=scores["B0_GT_P_GT_R"]
    for name in ("P_LIO_P_GT_R","R_GT_P_LIO_R","B1_LIO_P_LIO_R"):
        print(name, {key:round(scores[name][key]-baseline[key],6) for key in metric_keys})

    # Full factorial interaction on scalar aggregate RMSE is descriptive,
    # and NOT an additive physical causal decomposition of each sample.
    interaction={
        k:float(scores["B1_LIO_P_LIO_R"][k]
             - scores["P_LIO_P_GT_R"][k]
             - scores["R_GT_P_LIO_R"][k]
             + baseline[k]) for k in metric_keys
    }
    print("2x2 RMSE interaction (B1 - P - R + B0):",interaction)
    out.mkdir(parents=True,exist_ok=True)
    archive = out/"a4_2d_2x2_eskf_outputs.npz"
    report = out/"a4_2d_2x2_summary.json"
    array_kwargs = {"time_s":t,"truth_payload_position_W_m":p_true,"truth_payload_quat_xyzw":q_true,
                    "truth_payload_velocity_W_mps":v_true,"truth_payload_omega_W_radps":w_true}
    for name,_,_ in modes:
        e=estimates[name]
        array_kwargs[f"{name}_load_position_W_m"] = e["p"]
        array_kwargs[f"{name}_load_rotation_L_to_W"] = e["R"]
        array_kwargs[f"{name}_load_velocity_W_mps"] = e["v"]
        array_kwargs[f"{name}_load_omega_L_radps"] = e["w"]
        array_kwargs[f"{name}_kabsch_fit_m"] = e["kabsch_fit_m"]
        array_kwargs[f"{name}_position_error_m"] = scores[name]["errors"]["pos_m"]
        array_kwargs[f"{name}_rotation_error_deg"] = scores[name]["errors"]["rot_deg"]
    np.savez_compressed(archive,**array_kwargs)
    report_payload={
        "experiment":"A4-2d 2x2 fixed-timeline 10Hz UAV position/orientation LIO-source ablation",
        "time_start_s":float(t[0]),"time_end_s":float(t[-1]),"samples":len(t),
        "pose_stride":args.pose_stride,"same_virtual_specific_force_and_collective_thrust":True,
        "A4_2c_parity_max_abs":parity,
        "scores":{name:{k:v for k,v in scores[name].items() if k!="errors"} for name,_,_ in modes},
        "RMSE_interaction_B1_minus_P_minus_R_plus_B0":interaction,
        "limits":[
            "2x2 intervention changes 3 UAV positions and/or attitudes as a group; does not isolate individual UAV or physical error mechanism.",
            "RMSE effects and interactions are not guaranteed additive physical explanations.",
            "GT UAV poses in B0/P/R and PhysX oracle initial frame in LIO path are experimental privileged information, not online real robot capability.",
            "All modes use pose-derived ideal IMU and matched collective thrust; no measured hardware sensors.",
            "10Hz pose_stride=1 differs from original 100Hz ESKF schedule.",
            "Original load ESKF is a Sun-inspired independent estimator, not verbatim Sun paper code."
        ]
    }
    report.write_text(json.dumps(report_payload,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print("Saved:",archive)
    print("Saved:",report)


if __name__=="__main__":
    main()
