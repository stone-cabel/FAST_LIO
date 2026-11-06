#!/usr/bin/env python3
"""A4-2b STRICT 10Hz paired-input export, no simulation and no overwrites.

Caveat: sample rows store collective thrust per UAV, not rotor-resolved force.
Only StreamingZengESKF's rotor.sum(axis=1) can be reproduced from these data.
No ESKF is executed in this script. Any PhysX GT is evaluation/oracle baseline,
not MoCap-free estimation.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def records_from(obj, path):
    # Historical E7 JSON schema, not arbitrary file-name matching.
    if not isinstance(obj, dict):
        raise ValueError(f"{path}: expected JSON object")
    for key in ("per_step", "per_step_records", "samples", "records", "steps", "per_step_metrics"):
        rows = obj.get(key)
        if isinstance(rows, list) and len(rows) > 0 and isinstance(rows[0], dict):
            if all(k in rows[0] for k in ("time_s", "uav_positions_w_m", "rotor_collective_thrust_N")):
                return rows, key
    raise ValueError(f"{path}: no E7 per-step record list with time, three UAV positions, collective thrust")


def load_candidate(path):
    with path.open(encoding="utf-8") as f:
        document = json.load(f)
    rows, row_key = records_from(document, path)
    t, p, thrust = [], [], []
    for n, r in enumerate(rows):
        if not all(k in r for k in ("time_s", "uav_positions_w_m", "rotor_collective_thrust_N")):
            raise ValueError(f"{path}: missing fields at row {n}")
        ts = float(r["time_s"])
        pos = np.asarray(r["uav_positions_w_m"], dtype=float)
        force = np.asarray(r["rotor_collective_thrust_N"], dtype=float)
        if pos.shape != (3, 3) or force.shape != (3,):
            raise ValueError(f"{path}: invalid row shape at {n}: {pos.shape}, {force.shape}")
        if not (np.isfinite(ts) and np.isfinite(pos).all() and np.isfinite(force).all()):
            raise ValueError(f"{path}: nonfinite data at row {n}")
        if (force < 0).any():
            raise ValueError(f"{path}: negative collective thrust at row {n}")
        t.append(ts); p.append(pos); thrust.append(force)
    t, p, thrust = np.asarray(t), np.asarray(p), np.asarray(thrust)
    if not np.all(np.diff(t) > 0):
        raise ValueError(f"{path}: nonmonotonic times")
    return t, p, thrust, {"path": str(path), "row_key": row_key,
                            "seed": document.get("seed"),
                            "goal_profile": document.get("goal_profile"),
                            "applied_pose_mode": document.get("applied_pose_mode"),
                            "completed_steps": document.get("completed_steps"),
                            "experiment": document.get("experiment")}


def take_exact(times, values, grid, tol=1e-6):
    # Nearest row lookup, but NEVER interpolate a controller's motor thrust.
    ids = np.searchsorted(times, grid)
    ids = np.clip(ids, 0, len(times)-1)
    previous = np.clip(ids-1, 0, len(times)-1)
    ids = np.where(np.abs(times[previous]-grid) <= np.abs(times[ids]-grid), previous, ids)
    if np.max(np.abs(times[ids]-grid)) > tol or len(np.unique(ids)) != len(grid):
        raise ValueError(f"Cannot extract exact shared timestamps, max dt={np.max(np.abs(times[ids]-grid)):.8g}s")
    return values[ids]


def main():
    parser = argparse.ArgumentParser()
    home = Path.home() / "research"
    parser.add_argument("--root", type=Path, default=home/"isaac_a1_diagnostics")
    parser.add_argument("--eskf-root", type=Path, default=home/"payload_ekf_sun_E1")
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--position-tolerance-m", type=float, default=1e-4)
    parser.add_argument("--force-tolerance-N", type=float, default=1e-5)
    args = parser.parse_args()
    root = args.root.expanduser()
    eskf = args.eskf_root.expanduser()
    outdir = args.out_dir.expanduser() if args.out_dir else root/"A4_2B/paired_collective_thrust_A3_B2"

    source1 = eskf/"results/E7_1/seed42_smooth-x_clean_v1.json"
    source2 = eskf/"results/E7_1_E/seed42_smooth-x_clean_truth_v1.json"
    t1,p1,f1,meta1 = load_candidate(source1)
    t2,p2,f2,meta2 = load_candidate(source2)

    with np.load(root/"a1_47_mappo_eskf_500steps_300hz.npz", allow_pickle=False) as z:
        t_gt = np.asarray(z["time_s"], dtype=float)
        names = list(map(str, z["body_names"]))
        all_pose = np.asarray(z["body_link_pose_w_xyzw_300hz"], dtype=float)
    indexes = [names.index(f"Falcon{i}_base_link_inertia") for i in (1,2,3)]
    world_p_gt = all_pose[:,indexes,:3]

    with np.load(root/"A4_1/oracle_alignment_B2/a4_1_three_uav_common_frame.npz", allow_pickle=False) as z:
        grid = np.asarray(z["time_s"],dtype=float)
        p_lio = np.asarray(z["position_W_m"],dtype=float)
        q_lio = np.asarray(z["quaternion_WB_xyzw"],dtype=float)
        p_gt_10 = np.asarray(z["gt_position_W_m"],dtype=float)
        q_gt_10 = np.asarray(z["gt_quaternion_WB_xyzw"],dtype=float)
    with np.load(root/"A3_1/a3_1_imu_5s_300hz.npz", allow_pickle=False) as z:
        imu_t = np.asarray(z["time_s"],dtype=float)
        specific_force = np.asarray(z["accel_body_mps2"],dtype=float)
        gyro_B = np.asarray(z["gyro_body_radps"],dtype=float)
    if grid.shape != (46,) or not np.allclose(grid, np.arange(4,50)/10,atol=1e-10):
        raise ValueError("Expected the original A4-1 46 exact 10Hz timestamps 0.4..4.9")
    if p_lio.shape!=(46,3,3) or q_lio.shape!=(46,3,4):
        raise ValueError("Invalid A4-1 LIO pose layout")
    if specific_force.shape!=(len(imu_t),3,3) or gyro_B.shape != specific_force.shape:
        raise ValueError("Invalid A3 IMU shape")

    # On BOTH candidate JSON traces, validate EVERY controller time sample in the
    # full A4-1 0.4..4.9s overlap against the 300Hz A3 PhysX reference, not just 46.
    assessments = []
    for t,p,f,m in ((t1,p1,f1,meta1),(t2,p2,f2,meta2)):
        mask = (t >= grid[0]-1e-9) & (t <= grid[-1]+1e-9)
        tt,pp = t[mask],p[mask]
        if len(tt) < 450:
            raise ValueError(f"Insufficient candidate control samples: {len(tt)} {m['path']}")
        true = np.stack([np.column_stack([np.interp(tt,t_gt,world_p_gt[:,i,j])
                                           for j in range(3)]) for i in range(3)],axis=1)
        diff=np.linalg.norm(pp-true,axis=-1)
        largest=float(np.max(diff))
        rms=float(np.sqrt(np.mean(diff**2)))
        assessments.append({"path":m["path"],"control_rows":len(tt),
                            "uav_position_rmse_m":rms,"max_uav_position_error_m":largest})
        if largest >= args.position_tolerance_m:
            raise ValueError(f"Candidate fails A3 pose match tolerance: max {largest:.9g} m; {m['path']}")
    f1_grid=take_exact(t1,f1,grid)
    f2_grid=take_exact(t2,f2,grid)
    p1_grid=take_exact(t1,p1,grid)
    p2_grid=take_exact(t2,p2,grid)
    max_force_diff=float(np.max(np.abs(f1_grid-f2_grid)))
    if max_force_diff >= args.force_tolerance_N:
        raise ValueError(f"Two candidate files differ in collective thrust at 10Hz: max={max_force_diff} N")
    max_poses_diff=float(np.max(np.abs(p1_grid-p2_grid)))
    if max_poses_diff >= args.position_tolerance_m:
        raise ValueError(f"Two candidates disagree in 10Hz UAV positions: max={max_poses_diff}m")

    # Confirm the A4-1 'GT' side is numerically the exact same A3 source.
    true_grid = np.stack([np.column_stack([np.interp(grid,t_gt,world_p_gt[:,i,j])
                                            for j in range(3)]) for i in range(3)],axis=1)
    if np.max(np.abs(true_grid-p_gt_10)) > 1e-5:
        raise ValueError("A4-1 GT positions not from same A3 file")

    imu_sample = take_exact(imu_t, specific_force, grid, tol=1e-6)
    gyro_sample = take_exact(imu_t, gyro_B, grid, tol=1e-6)
    output = outdir/"a4_2b_paired_10hz_inputs.npz"
    report = outdir/"a4_2b_paired_input_audit.json"
    if outdir.exists() and any(outdir.iterdir()):
        raise FileExistsError(f"Output dir already contains data; not overwriting: {outdir}")
    outdir.mkdir(parents=True,exist_ok=True)
    # The *_gt arrays are for paired B0 baseline / offline truth evaluation ONLY.
    np.savez_compressed(output, time_s=grid,
        uav_position_lio_W_m=p_lio, uav_quat_lio_WB_xyzw=q_lio,
        uav_position_gt_W_m=p_gt_10, uav_quat_gt_WB_xyzw=q_gt_10,
        imu_specific_force_B_mps2=imu_sample, gyro_B_radps=gyro_sample,
        rotor_collective_thrust_N=f1_grid,
        pairing_verdict=np.asarray("TWO_E7_JSON_TRACES_MATCH_A3_PHYSX_POSE_AND_COLLECTIVE_THRUST"),
        inference_caveat=np.asarray("10Hz offline oracle frame alignment; virtual pose-derived IMU; not real hardware"))
    report_obj = {
        "verdict":"PASS: A3 300Hz PhysX and both E7 JSON UAV position trajectories match; 10Hz collective thrust agrees across two JSON copies",
        "files":[meta1,meta2],"assessments":assessments,
        "common_samples_10hz":len(grid),"common_start_s":float(grid[0]),
        "common_end_s":float(grid[-1]),
        "max_pair_JSON_collective_thrust_difference_N":max_force_diff,
        "max_pair_JSON_position_difference_m":max_poses_diff,
        "thrust_range_N":[float(f1_grid.min()),float(f1_grid.max())],
        "limits":[
            "Same 3-UAV world positions at controller timestamps strongly supports same trajectory; source process provenances and rotor allocation not audited.",
            "Only collective thrust per UAV (3 numbers/sample) is in JSON, not individual four motors; current StreamingZengESKF uses only sum(rotor_N,axis=1).",
            "PhysX data is valid only as B0 oracle UAV pose and held-out evaluation; LIO is initially aligned using PhysX oracle.",
            "A3 pose-derived IMU is ideal and not independent real hardware.",
            "Do not call this a complete ESKF run: no estimator update or payload estimation happens in this script.",
        ],
        "output":str(output),
    }
    report.write_text(json.dumps(report_obj,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print("A4-2b VERIFIED PAIRED INPUT PREPARATION")
    for row in assessments:
        print(f"rows={row['control_rows']} UAV position RMSE={row['uav_position_rmse_m']:.9g}m "
              f"MAX={row['max_uav_position_error_m']:.9g}m {row['path']}")
    print(f"JSON force agreement max: {max_force_diff:.9g} N")
    print(f"Collective thrust range at 10Hz: {f1_grid.min():.6f} .. {f1_grid.max():.6f} N")
    print("Saved",output)
    print("Saved",report)
    print("NOTE: This is data pairing/preparation only, not ESKF / MoCap-free validation.")

if __name__=="__main__":
    main()
