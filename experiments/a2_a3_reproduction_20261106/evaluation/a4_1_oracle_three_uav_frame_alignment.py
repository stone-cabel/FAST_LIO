#!/usr/bin/env python3
"""A4-1 oracle initial multi-UAV frame alignment and relative-geometry audit.

Consumes ONLY the three validated A3-B2 odometry files and held-out PhysX GT.
Uses GT at ONE common initialization time to construct 3 fixed LIO->W
transforms. GT is NOT used afterward to correct any estimated trajectory;
it is used solely to evaluate errors. This is NOT autonomous frame alignment.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def read_odom(path):
    if not path.is_file():
        raise FileNotFoundError(path)
    with np.load(path, allow_pickle=False) as data:
        odom = np.asarray(data["odom_time_pose_xyzw"], dtype=np.float64)
    if odom.ndim != 2 or odom.shape[1] != 8 or len(odom) < 2:
        raise ValueError(f"Expected >=2 Nx8 odom rows: {path}")
    if not np.all(np.isfinite(odom)) or not np.all(np.diff(odom[:, 0]) > 0):
        raise ValueError(f"Invalid odometry timestamps or nonfinite values: {path}")
    return odom


def sample_pose(times, positions, quaternions, query):
    if query[0] < times[0] - 1e-9 or query[-1] > times[-1] + 1e-9:
        raise ValueError("Query outside trajectory time interval: no extrapolation allowed")
    pos = np.column_stack([np.interp(query, times, positions[:, axis]) for axis in range(3)])
    rot = Slerp(times, Rotation.from_quat(quaternions))(query)
    return pos, rot


def rmse(array):
    return float(np.sqrt(np.mean(np.square(array))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path.home() / "research/isaac_a1_diagnostics")
    ap.add_argument("--output-dir", type=Path, default=None)
    ap.add_argument("--rate-hz", type=float, default=10.0)
    args = ap.parse_args()
    if not (0 < args.rate_hz <= 300):
        ap.error("--rate-hz must be within (0,300]")
    root = args.root.expanduser()
    output = (args.output_dir.expanduser() if args.output_dir else root / "A4_1/oracle_alignment_B2")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output directory not empty; no historical overwrite: {output}")

    file_gt = root / "a1_47_mappo_eskf_500steps_300hz.npz"
    with np.load(file_gt, allow_pickle=False) as data:
        t_gt = np.asarray(data["time_s"], dtype=np.float64)
        gt_pose = np.asarray(data["body_link_pose_w_xyzw_300hz"], dtype=np.float64)
        names = [str(x) for x in data["body_names"]]
    if gt_pose.ndim != 3 or gt_pose.shape[-1] != 7 or len(t_gt) != len(gt_pose):
        raise ValueError("Unexpected GT layout; expected T x B x 7")
    if not np.all(np.diff(t_gt) > 0):
        raise ValueError("Non-increasing GT time")

    measured = {}
    for i in (1, 2, 3):
        path = root / "A3_1/A3_B2_gyro_zero" / f"a3_b2_drone_{i}_odom.npz"
        odo = read_odom(path)
        name = f"Falcon{i}_base_link_inertia"
        if name not in names:
            raise ValueError(f"Missing body in GT: {name}")
        measured[i] = (odo, gt_pose[:, names.index(name)], str(path))

    t_init = max(float(v[0][0,0]) for v in measured.values())
    t_end = min(float(v[0][-1,0]) for v in measured.values())
    if t_init >= t_end or t_init < t_gt[0] or t_end > t_gt[-1]:
        raise ValueError("No valid 3-UAV + GT shared time overlap")
    # Regular common timestamps: ceil(first shared timestamp) through floor(last).
    step = 1.0 / args.rate_hz
    first_index = int(np.ceil((t_init - 1e-9)/step))
    last_index = int(np.floor((t_end + 1e-9)/step))
    grid = step * np.arange(first_index, last_index + 1, dtype=np.float64)
    if len(grid) < 3:
        raise ValueError("Too few shared timestamps")

    aligned_pos, aligned_quat, truth_pos, truth_quat = [], [], [], []
    transformations = {}
    for i in (1, 2, 3):
        odo, gt_i, path = measured[i]
        p0, R0 = sample_pose(odo[:,0], odo[:,1:4], odo[:,4:8], np.array([t_init]))
        pg0, Rg0 = sample_pose(t_gt, gt_i[:,:3], gt_i[:,3:7], np.array([t_init]))
        # Fixed rigid world<-local LIO map, constructed using oracle GT ONCE at t_init.
        RA = Rg0[0] * R0[0].inv()
        tA = pg0[0] - RA.apply(p0[0])
        transformations[str(i)] = {
            "t_W_L_m": tA.tolist(),
            "q_W_L_xyzw": RA.as_quat().tolist(),
            "oracle_calibration_time_s": t_init,
            "source": path,
        }
        p, R = sample_pose(odo[:,0], odo[:,1:4], odo[:,4:8], grid)
        pg, Rg = sample_pose(t_gt, gt_i[:,:3], gt_i[:,3:7], grid)
        aligned_pos.append(RA.apply(p) + tA)
        aligned_quat.append((RA * R).as_quat())
        truth_pos.append(pg)
        truth_quat.append(Rg.as_quat())

    # Shapes: T x 3 x xyz / quaternion, indexed by drone (1,2,3)
    p_hat = np.stack(aligned_pos, axis=1)
    q_hat = np.stack(aligned_quat, axis=1)
    p_gt = np.stack(truth_pos, axis=1)
    q_gt = np.stack(truth_quat, axis=1)
    ep = p_hat - p_gt
    ep_norm = np.linalg.norm(ep, axis=-1)
    rot_err = np.empty((len(grid), 3), dtype=np.float64)
    for k in range(3):
        rot_err[:,k] = np.rad2deg(
            (Rotation.from_quat(q_hat[:,k]).inv() *
             Rotation.from_quat(q_gt[:,k])).magnitude()
        )

    pair_details = {}
    pair_vector_errors = []
    pair_distance_errors = []
    for i,j in ((1,2),(1,3),(2,3)):
        dv_hat = p_hat[:,j-1,:] - p_hat[:,i-1,:]
        dv_gt  = p_gt[:,j-1,:] - p_gt[:,i-1,:]
        ev = dv_hat - dv_gt
        vector_error = np.linalg.norm(ev,axis=-1)
        distance_error = np.linalg.norm(dv_hat,axis=-1) - np.linalg.norm(dv_gt,axis=-1)
        pair_vector_errors.append(vector_error)
        pair_distance_errors.append(distance_error)
        pair_details[f"{i}-{j}"] = {
            "relative_vector_rmse_m": rmse(vector_error),
            "relative_distance_rmse_m": rmse(distance_error),
            "relative_vector_final_error_m":float(vector_error[-1]),
            "relative_distance_final_abs_error_m":float(abs(distance_error[-1])),
            "relative_vector_max_error_m":float(np.max(vector_error)),
        }

    # Velocity is a finite-difference DIAGNOSTIC only, not original FAST-LIO velocity.
    v_hat = np.gradient(p_hat, grid, axis=0, edge_order=2)
    summary = {
        "name":"A4-1 oracle initial SE3 alignment of A3-B2 three-UAV odometry",
        "WARNING":"PhysX oracle initializes EACH UAV's world transform. Not a MoCap-Free or Swarm-LIO2 frame-alignment method.",
        "oracle_used_only_at_t_init_s":t_init,
        "common_time_start_s":float(grid[0]),
        "common_time_end_s":float(grid[-1]),
        "common_samples":len(grid),
        "output_rate_hz":args.rate_hz,
        "trajectory_frame":"W (PhysX world) via per-UAV oracle initialization",
        "body_frame":"Falcon{i}_base_link_inertia as in GT & original A3 evaluation",
        "velocity_note":"numerical derivative of common-grid aligned positions; NOT ESKF velocity state, do not feed to EKF uncritically",
        "individual_position_rmse_m":{str(i):rmse(ep_norm[:,i-1]) for i in (1,2,3)},
        "individual_rotation_rmse_deg":{str(i):rmse(rot_err[:,i-1]) for i in (1,2,3)},
        "pairs":pair_details,
        "fixed_initial_alignment":transformations,
        "gt_source":str(file_gt)
    }
    # Protect all outputs, including empty directories from prior failed runs.
    output.mkdir(parents=True,exist_ok=True)
    outfile = output / "a4_1_three_uav_common_frame.npz"
    manifest = output / "a4_1_summary.json"
    if outfile.exists() or manifest.exists():
        raise FileExistsError("Refusing to overwrite A4-1 archive")
    np.savez_compressed(outfile, time_s=grid,
        position_W_m=p_hat, quaternion_WB_xyzw=q_hat,
        velocity_W_finite_diff_mps=v_hat,
        gt_position_W_m=p_gt, gt_quaternion_WB_xyzw=q_gt,
        position_error_W_m=ep, position_error_norm_m=ep_norm,
        rotation_error_deg=rot_err,
        pair_12_vector_error_m=pair_vector_errors[0],
        pair_13_vector_error_m=pair_vector_errors[1],
        pair_23_vector_error_m=pair_vector_errors[2],
        pair_12_distance_error_m=pair_distance_errors[0],
        pair_13_distance_error_m=pair_distance_errors[1],
        pair_23_distance_error_m=pair_distance_errors[2],
        drone_ids=np.array([1,2,3],dtype=np.int64),
        alignment_method=np.asarray("PHYSX_ORACLE_PER_UAV_INITIAL_SE3_ONLY_NOT_AUTONOMOUS"))
    manifest.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print("="*72)
    print("A4-1 ORACLE INITIAL FRAME ALIGNMENT — NOT AUTONOMOUS")
    print(f"t_init={t_init:.6f}s, common={grid[0]:.3f}..{grid[-1]:.3f}s, n={len(grid)}")
    for i in (1,2,3):
        print(f"Falcon {i}: pos RMSE={summary['individual_position_rmse_m'][str(i)]:.6f}m; "
              f"rot RMSE={summary['individual_rotation_rmse_deg'][str(i)]:.6f}deg")
    for key,value in pair_details.items():
        print(f"Pair {key}: relative vector RMSE={value['relative_vector_rmse_m']:.6f}m; "
              f"distance RMSE={value['relative_distance_rmse_m']:.6f}m; "
              f"max vector error={value['relative_vector_max_error_m']:.6f}m")
    print("Saved:", outfile)
    print("Saved:", manifest)
    print("NEXT: inspect per-pair errors before supplying states to Sun EKF.")

if __name__=="__main__":
    main()
