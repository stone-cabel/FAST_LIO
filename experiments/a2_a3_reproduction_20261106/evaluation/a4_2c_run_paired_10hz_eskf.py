#!/usr/bin/env python3
"""A4-2c: paired, 10-Hz, offline PhysX-pose vs LIO-pose -> streaming LoadESKF.

Same measured virtual specific force, three collective rotor thrusts, timestamps,
physics parameters and filter configuration in B0/B1. Only UAV pose switches.
Held-out payload truth is accessed only AFTER both filters finish. This does
NOT test real MoCap-free frame alignment or 100Hz ESKF equivalence.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HOME = Path.home() / "research"


def exact_sample(t, v, query, tolerance=1e-6):
    k = np.searchsorted(t, query)
    k = np.clip(k, 0, len(t)-1)
    km1 = np.maximum(k-1, 0)
    k = np.where(abs(t[km1]-query) <= abs(t[k]-query), km1, k)
    err = np.max(abs(t[k]-query))
    if err > tolerance or len(np.unique(k)) != len(query):
        raise ValueError(f"Could not exactly match timestamps; max discrepancy {err:.9g} s")
    return v[k]


def load_truth_after_filters(path, grid):
    """Only for final reporting; NEVER part of ESKF input generation."""
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    records = None
    for key in ("per_step", "per_step_records", "records", "samples", "steps", "per_step_metrics"):
        val = data.get(key)
        if isinstance(val, list) and val and isinstance(val[0], dict) and "load_true_position_w_m" in val[0]:
            records = val
            break
    if records is None:
        raise ValueError("E7 paired JSON missing per-step load truth")
    ts = []; pp = []; qq = []; vv = []; ww = []
    for row in records:
        keys = ("time_s", "load_true_position_w_m", "load_true_quaternion_xyzw",
                "load_true_velocity_w_mps", "load_true_angular_velocity_w_radps")
        if not all(k in row for k in keys):
            raise ValueError("Missing held-out payload GT field")
        ts.append(float(row["time_s"]))
        pp.append(row["load_true_position_w_m"])
        qq.append(row["load_true_quaternion_xyzw"])
        vv.append(row["load_true_velocity_w_mps"])
        ww.append(row["load_true_angular_velocity_w_radps"])
    ts = np.asarray(ts,float)
    if not np.all(np.diff(ts)>0):
        raise ValueError("JSON truth timestamps not increasing")
    p = exact_sample(ts,np.asarray(pp,float),grid)
    q = exact_sample(ts,np.asarray(qq,float),grid)
    v = exact_sample(ts,np.asarray(vv,float),grid)
    w = exact_sample(ts,np.asarray(ww,float),grid)
    if p.shape != (len(grid),3) or q.shape != (len(grid),4):
        raise ValueError("Invalid load ground truth shape")
    if not all(np.isfinite(a).all() for a in (p,q,v,w)):
        raise ValueError("Nonfinite held-out load truth")
    return p,q,v,w


def run_streaming(mode, time_s, position, quat, specific_force, gyro_body, thrust, stride,
                  StreamingZengESKF):
    from scipy.spatial.transform import Rotation
    n = len(time_s)
    filt = StreamingZengESKF(pose_stride=stride)
    # The current ESKF step uses only position, body->world orientation,
    # specific_force_B and total thrust. It does not use the velocity or
    # body_acc_w fields when explicit specific_force_B is supplied.
    # Populate the unused state slots explicitly, but do NOT claim they are
    # measured real LIO velocity or angular velocity.
    v_diag = np.gradient(position,time_s,axis=0,edge_order=2)
    R = Rotation.from_quat(quat.reshape(-1,4)).as_matrix().reshape(n,3,3,3)
    # Convert the same ideal body gyro to world angular rates for valid
    # body_com_state_w schema; this field is not consumed by this ESKF path.
    omega_world = np.einsum("tnij,tnj->tni",R,gyro_body)
    x = np.concatenate((position,quat,v_diag,omega_world),axis=-1)
    if x.shape != (n,3,13):
        raise ValueError("Wrong UAV state adapter shape")
    if not all(np.isfinite(a).all() for a in (x,specific_force,thrust)):
        raise ValueError("Invalid input arrays")
    # ESKF only sums each rotor group. This is a collective-only
    # compatibility adapter, not an invented reconstruction of rotor forces.
    rotor_equal_placeholder = np.repeat((thrust/4.0)[:,:,None],4,axis=2)
    if not np.allclose(rotor_equal_placeholder.sum(axis=2),thrust,atol=1e-10,rtol=0):
        raise AssertionError("Total thrust not conserved in compatibility adapter")
    outputs = []
    for k,t in enumerate(time_s):
        # Body acceleration placeholder is not used when specific_force_B is
        # supplied. Do not represent it as an observed sensor.
        unused_body_acc = np.zeros((3,6),dtype=float)
        try:
            result = filt.step(float(t),x[k],unused_body_acc,rotor_equal_placeholder[k],
                               specific_force_B=specific_force[k])
        except Exception as e:
            raise RuntimeError(f"{mode} failed at k={k}, t={t:.6f}s: {type(e).__name__}: {e}") from e
        outputs.append(result)
    return {
        "p":np.stack([o["p_W"] for o in outputs]),
        "v":np.stack([o["v_W"] for o in outputs]),
        "R":np.stack([o["R_L_W"] for o in outputs]),
        "w":np.stack([o["omega_L"] for o in outputs]),
        "kabsch_fit_m":np.array([o["kabsch_fit_m"] for o in outputs]),
        "P_trace":np.array([o["P_trace"] for o in outputs]),
    }


def evaluate(est, p_true, q_true, v_true, w_world_true):
    rg = Rotation.from_quat(q_true)
    r_e = Rotation.from_matrix(est["R"])
    w_body_true = np.einsum("tji,tj->ti",rg.as_matrix(),w_world_true)
    ep = np.linalg.norm(est["p"]-p_true,axis=1)
    er = np.degrees((r_e.inv()*rg).magnitude())
    ev = np.linalg.norm(est["v"]-v_true,axis=1)
    ew = np.linalg.norm(est["w"]-w_body_true,axis=1)
    return {
        "pos_rmse_m":float(np.sqrt(np.mean(ep**2))),
        "rot_rmse_deg":float(np.sqrt(np.mean(er**2))),
        "vel_rmse_mps":float(np.sqrt(np.mean(ev**2))),
        "bodyrate_rmse_radps":float(np.sqrt(np.mean(ew**2))),
        "pos_final_error_m":float(ep[-1]),
        "rot_final_error_deg":float(er[-1]),
        "kabsch_fit_mean_m":float(np.mean(est["kabsch_fit_m"])),
        "P_trace_final":float(est["P_trace"][-1]),
        "errors":{"pos_m":ep,"rot_deg":er,"vel_mps":ev,"bodyrate_radps":ew},
    }


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--root",type=Path,default=HOME/"isaac_a1_diagnostics")
    ap.add_argument("--sun-root",type=Path,default=HOME/"payload_ekf_sun_E1")
    ap.add_argument("--pose-stride",type=int,default=1,
                    help="Update Kabsch pose every N samples (10Hz samples; default 1=10Hz).")
    ap.add_argument("--out-dir",type=Path,default=None)
    args=ap.parse_args()
    if args.pose_stride < 1:
        ap.error("--pose-stride must >= 1")
    root=args.root.expanduser()
    src=args.sun_root.expanduser()
    sys.path.insert(0,str(src/"src"))
    from payload_ekf_sun.streaming_zeng_eskf import StreamingZengESKF

    path=root/"A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz"
    with np.load(path,allow_pickle=False) as z:
        t=np.asarray(z["time_s"],float)
        pl=np.asarray(z["uav_position_lio_W_m"],float)
        ql=np.asarray(z["uav_quat_lio_WB_xyzw"],float)
        pg=np.asarray(z["uav_position_gt_W_m"],float)
        qg=np.asarray(z["uav_quat_gt_WB_xyzw"],float)
        sf=np.asarray(z["imu_specific_force_B_mps2"],float)
        gyro=np.asarray(z["gyro_B_radps"],float)
        thrust=np.asarray(z["rotor_collective_thrust_N"],float)
    n=len(t)
    shapes={"pl":(n,3,3),"pg":(n,3,3),"ql":(n,3,4),"qg":(n,3,4),
            "sf":(n,3,3),"gyro":(n,3,3),"thrust":(n,3)}
    for k,a in (("pl",pl),("pg",pg),("ql",ql),("qg",qg),("sf",sf),("gyro",gyro),("thrust",thrust)):
        if a.shape != shapes[k] or not np.isfinite(a).all():
            raise ValueError(f"Invalid {k}: shape={a.shape}; expected {shapes[k]}")
    if len(t)<3 or not np.all(np.diff(t)>0) or not np.allclose(np.diff(t),0.1,atol=1e-8):
        raise ValueError("Expected regularly sampled 10Hz input")
    for k,qq in (("B0",qg),("B1",ql)):
        qnorm=np.linalg.norm(qq,axis=-1)
        if np.max(np.abs(qnorm-1.0))>1e-3:
            raise ValueError(f"{k} non-unit quaternion")

    outdir=(args.out_dir.expanduser() if args.out_dir else
            root/f"A4_2C/paired_10hz_stride{args.pose_stride}")
    if outdir.exists() and any(outdir.iterdir()):
        raise FileExistsError(f"Refusing to overwrite nonempty folder: {outdir}")

    # Both filters are run BEFORE loading payload truth to keep separation audit simple.
    print(f"A4-2c 10Hz paired offline ESKF: {n} frames, t={t[0]:.3f}..{t[-1]:.3f}s",
          flush=True)
    print(f"pose_stride={args.pose_stride}, update_period={args.pose_stride*0.1:.3f}s",
          flush=True)
    print("Collective thrust compatibility: repeated quarter thrust is only a sum-preserving adapter.")
    B0=run_streaming("B0_GT_UAV_POSE",t,pg,qg,sf,gyro,thrust,
                     args.pose_stride,StreamingZengESKF)
    print("B0 filter complete (payload truth not loaded).",flush=True)
    B1=run_streaming("B1_LIO_UAV_POSE",t,pl,ql,sf,gyro,thrust,
                     args.pose_stride,StreamingZengESKF)
    print("B1 filter complete (payload truth not loaded).",flush=True)

    truth_json=src/"results/E7_1_E/seed42_smooth-x_clean_truth_v1.json"
    p_true,q_true,v_true,w_true=load_truth_after_filters(truth_json,t)
    m0=evaluate(B0,p_true,q_true,v_true,w_true)
    m1=evaluate(B1,p_true,q_true,v_true,w_true)
    print("="*74)
    for name,m in (("B0 PhysX UAV pose",m0),("B1 LIO UAV pose",m1)):
        print(f"{name}: load pos RMSE={m['pos_rmse_m']:.6f}m, "
              f"rot RMSE={m['rot_rmse_deg']:.6f}deg, "
              f"vel RMSE={m['vel_rmse_mps']:.6f}m/s, "
              f"body rate RMSE={m['bodyrate_rmse_radps']:.6f}rad/s, "
              f"Kabsch fit mean={m['kabsch_fit_mean_m']:.6f}m")
    print("B1-B0 RMSE DELTAS (positive = worse):")
    for key in ("pos_rmse_m","rot_rmse_deg","vel_rmse_mps","bodyrate_rmse_radps"):
        print(f" {key}: {m1[key]-m0[key]:+.6f}")
    outdir.mkdir(parents=True,exist_ok=True)
    npz=outdir/"a4_2c_b0_b1_load_eskf_outputs.npz"
    js=outdir/"a4_2c_b0_b1_load_eskf_summary.json"
    np.savez_compressed(npz,time_s=t,
        b0_load_position_W_m=B0["p"],b0_load_velocity_W_mps=B0["v"],
        b0_load_rotation_L_to_W=B0["R"],b0_load_omega_L_radps=B0["w"],
        b1_load_position_W_m=B1["p"],b1_load_velocity_W_mps=B1["v"],
        b1_load_rotation_L_to_W=B1["R"],b1_load_omega_L_radps=B1["w"],
        truth_load_position_W_m=p_true,truth_load_quat_LW_xyzw=q_true,
        truth_load_velocity_W_mps=v_true,truth_load_omega_W_radps=w_true,
        b0_position_error_m=m0["errors"]["pos_m"],b1_position_error_m=m1["errors"]["pos_m"],
        b0_rotation_error_deg=m0["errors"]["rot_deg"],b1_rotation_error_deg=m1["errors"]["rot_deg"],
        b0_kabsch_fit_m=B0["kabsch_fit_m"],b1_kabsch_fit_m=B1["kabsch_fit_m"],
        b0_cov_trace=B0["P_trace"],b1_cov_trace=B1["P_trace"])
    summary={
        "experiment":"A4-2c B0 PhysX pose vs B1 A4-1 LIO pose (10Hz paired offline)",
        "status":"LOCAL_RUN_REQUIRED",
        "source_input":str(path),"payload_truth_only_after_both_filter_runs":str(truth_json),
        "n":n,"t_start":float(t[0]),"t_end":float(t[-1]),"pose_stride":args.pose_stride,
        "same_imu_thrust_and_filter_parameters":True,
        "B0":{k:v for k,v in m0.items() if k!="errors"},
        "B1":{k:v for k,v in m1.items() if k!="errors"},
        "delta_B1_minus_B0":{k:m1[k]-m0[k] for k in ("pos_rmse_m","rot_rmse_deg",
                                                      "vel_rmse_mps","bodyrate_rmse_radps")},
        "caveats":[
            "B0 is privileged PhysX UAV pose; B1 uses PhysX oracle-aligned LIO pose.",
            "Both use synthetic 300Hz pose-derived ideal IMU sampled at 10Hz.",
            "Rotor group is artificial fourfold split of TRUE collective thrust; valid only because ESKF sums rotor channels.",
            "uav_com_acc_w input is a zero PLACEHOLDER ignored by the ESKF when specific_force_B is provided.",
            "UAV world velocity / world gyro fields are populated for compatibility but not used by current ESKF.step.",
            "Existing streaming Zeng ESKF uses lumped massless straight-cable approximation; results cannot establish original Sun EKF replication.",
            "10Hz temporal discretization and pose_stride differ from original 100Hz case; both B0 and B1 have identical schedule.",
            "Not real-world LiDAR/IMU, full autonomy or MoCap-free online navigation."
        ],
    }
    js.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("Saved",npz)
    print("Saved",js)


if __name__=="__main__":
    main()
