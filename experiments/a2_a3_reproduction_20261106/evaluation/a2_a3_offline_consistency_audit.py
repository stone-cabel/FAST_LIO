#!/usr/bin/env python3
"""Offline A2 provenance, A3 five-point early-window and drift audit.

Inputs: existing original NPZ data on user's Ubuntu host.
Does not alter FAST-LIO2, original NPZ files, or cached trajectories.
No ground-truth data are supplied to the estimator.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

A2_GT = "a2_4a_7_init_7p2_to_8p0_300hz.npz"
A2_IMU = "a2_4a_8_imu_7p2_to_8p0_300hz.npz"
A2_ODOM = {
    "B0": "a2_4b_0_single_odom.npz",
    "B1": "a2_4b_1_single_odom.npz",
    "B3": "a2_4b_3_retry_odom.npz",
}
A3_GT = "a1_47_mappo_eskf_500steps_300hz.npz"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def info(path: Path) -> dict:
    if not path.exists():
        return {"exists": False, "path": str(path)}
    return {
        "exists": True, "path": str(path),
        "bytes": path.stat().st_size, "sha256": sha256(path),
    }


def normed_rot(q):
    q = np.asarray(q, dtype=float)
    return Rotation.from_quat(q / np.linalg.norm(q, axis=-1, keepdims=True))


def interp_pose(t, pose, query):
    t = np.asarray(t, dtype=float)
    query = np.asarray(query, dtype=float)
    if not np.all(np.diff(t) > 0):
        raise ValueError("Truth/odom time must be strictly increasing")
    if query[0] < t[0] - 1e-7 or query[-1] > t[-1] + 1e-7:
        raise ValueError(f"Query {query[0]}..{query[-1]} outside {t[0]}..{t[-1]}")
    query = np.clip(query, t[0], t[-1])
    p = np.column_stack([np.interp(query, t, pose[:, i]) for i in range(3)])
    rot = Slerp(t, normed_rot(pose[:, 3:7]))(query)
    return p, rot


def read_truth(path: Path, drone: int):
    with np.load(path, allow_pickle=False) as z:
        names = [str(x) for x in z["body_names"]]
        i = names.index(f"Falcon{drone}_base_link_inertia")
        t = np.asarray(z["time_s"], dtype=float).copy()
        poses = np.asarray(z["body_link_pose_w_xyzw_300hz"][:, i], dtype=float).copy()
    return t, poses


def read_odom(path: Path):
    with np.load(path, allow_pickle=False) as z:
        data = np.asarray(z["odom_time_pose_xyzw"], dtype=float).copy()
    if data.ndim != 2 or data.shape[1] != 8 or len(data) < 2:
        raise ValueError(f"Unexpected odometry array {path}: {data.shape}")
    if not np.isfinite(data).all() or not np.all(np.diff(data[:, 0]) > 0):
        raise ValueError(f"Invalid/non-monotonic odometry: {path}")
    return data[:, 0], data[:, 1:]


def metrics_on_query(truth_t, truth_pose, odom_t, odom_pose, query):
    gt_p, gt_R = interp_pose(truth_t, truth_pose, query)
    od_p, od_R = interp_pose(odom_t, odom_pose, query)
    R_align = gt_R[0] * od_R[0].inv()
    delta_est = R_align.apply(od_p - od_p[0])
    delta_gt = gt_p - gt_p[0]
    pos_errors = np.linalg.norm(delta_est - delta_gt, axis=1)
    rot_errors = np.degrees(
        (gt_R * (R_align * od_R).inv()).magnitude()
    )
    return {
        "position_rmse_m": float(np.sqrt(np.mean(pos_errors**2))),
        "orientation_rmse_deg": float(np.sqrt(np.mean(rot_errors**2))),
        "final_position_error_m": float(pos_errors[-1]),
        "error_samples_m": [float(x) for x in pos_errors],
        "orientation_samples_deg": [float(x) for x in rot_errors],
    }


def a2_audit(root: Path):
    print("\n========== PRIORITY 1: A2 ORIGINAL DATA PROVENANCE ==========")
    result = {"files": {}, "scans": {}, "imu": {}, "odometry": {}}
    gt_file, imu_file = root / A2_GT, root / A2_IMU
    result["files"]["gt"] = info(gt_file)
    result["files"]["imu"] = info(imu_file)
    gt_t = gt_pose = None
    if gt_file.exists():
        gt_t, gt_pose = read_truth(gt_file, 1)
        print(f"A2 GT: {len(gt_t)} samples, [{gt_t[0]:.6f},{gt_t[-1]:.6f}] s")

    if imu_file.exists():
        with np.load(imu_file, allow_pickle=False) as z:
            imu_keys = list(z.files)
            t = np.asarray(z["time_s"], dtype=float).copy()
            acc = np.asarray(z["drone_1_accel_body_mps2"], dtype=float).copy()
            gyro = np.asarray(z["drone_1_gyro_body_radps"], dtype=float).copy()
        ok = (
            t.shape == (241,) and acc.shape == (241, 3) and gyro.shape == (241, 3)
            and np.all(np.diff(t) > 0)
            and np.isfinite(acc).all() and np.isfinite(gyro).all()
        )
        result["imu"] = {
            "keys": imu_keys, "valid_replay_shape": bool(ok),
            "sample_count": len(t), "time_span_s": [float(t[0]), float(t[-1])],
            "mean_gyro_first_30_radps": np.mean(gyro[:30], axis=0).tolist(),
        }
        print(f"A2 IMU: {len(t)} samples, [{t[0]:.6f},{t[-1]:.6f}] s, shape valid={ok}")

        # Differentiated GT orientation vs measured gyro: an independent numeric
        # consistency test, not a bias/accuracy certification. Try both the
        # intrinsic angular velocity and end/midpoint timestamp conventions.
        if gt_t is not None and len(gt_t) > 2:
            gt_local = gt_t - gt_t[0]
            if (
                abs(gt_local[-1] - t[-1]) < 0.02 and
                abs(np.median(np.diff(gt_local)) - np.median(np.diff(t))) < 1e-4
            ):
                gt_R = normed_rot(gt_pose[:, 3:7])
                step = (gt_R[:-1].inv() * gt_R[1:]).as_rotvec()
                gyro_derived = step / np.diff(gt_local)[:, None]
                # aligned to each derived step's midpoint
                mids = (gt_local[1:] + gt_local[:-1]) / 2
                gyro_at_mid = np.column_stack([
                    np.interp(mids, t, gyro[:, j]) for j in range(3)
                ])
                residual = np.linalg.norm(gyro_derived - gyro_at_mid, axis=1)
                result["imu"]["gyro_vs_gt_orientation"] = {
                    "median_norm_radps": float(np.median(residual)),
                    "p95_norm_radps": float(np.percentile(residual, 95)),
                    "note": "GT finite-difference body angular velocity vs midpoint IMU interpolation; diagnostic only",
                }
                print("A2 IMU vs truth gyro |diff|: "
                      f"median={np.median(residual):.6g} rad/s, "
                      f"p95={np.percentile(residual,95):.6g} rad/s")
            else:
                print("A2 IMU vs GT gyro: time grids do not align; not verified")
        else:
            print("A2 IMU vs GT gyro: GT unavailable")

    for variant in ("0", "1", "3"):
        scans = []
        for w in range(8):
            path = root / f"a2_4b_{variant}_mid360_window_{w:02d}_2000rays.npz"
            if not path.exists():
                scans.append({"window": w, "exists": False})
                continue
            with np.load(path, allow_pickle=False) as z:
                rec = {"window": w, "exists": True, "file": path.name}
                for key in ("pose_cache_source", "scan_control_mode",
                            "scan_experiment", "scan_start_time_s"):
                    if key in z:
                        v = z[key]
                        rec[key] = v.item() if v.ndim == 0 else np.asarray(v).tolist()
                if gt_t is not None and all(k in z for k in
                    ("body_poses_300hz_xyzw", "body_names", "body_pose_times_s")):
                    names = [str(x) for x in z["body_names"]]
                    idx = names.index("Falcon1_base_link_inertia")
                    pose_scan = np.asarray(z["body_poses_300hz_xyzw"][:, idx], dtype=float)
                    local_t = np.asarray(z["body_pose_times_s"], dtype=float)
                    # A2 GT can use either cropped [0,.8] or uncropped [7.2,8.0].
                    scan_time = float(rec.get("scan_start_time_s", w*.1)) + local_t
                    ref_time = gt_t[0] + scan_time
                    try:
                        gt_p, gt_R = interp_pose(gt_t, gt_pose, ref_time)
                        pd = np.linalg.norm(pose_scan[:, :3] - gt_p, axis=1)
                        ad = np.degrees(
                            (normed_rot(pose_scan[:, 3:7]).inv() * gt_R).magnitude()
                        )
                        rec["scan_pose_vs_A2_gt"] = {
                            "position_max_m": float(pd.max()),
                            "position_rmse_m": float(np.sqrt(np.mean(pd**2))),
                            "rotation_max_deg": float(ad.max()),
                            "matching_origin_assumption": "gt_t[0] + scan_start + local_frame_time",
                        }
                    except Exception as exc:
                        rec["scan_pose_vs_A2_gt_error"] = str(exc)
                scans.append(rec)
        result["scans"][f"B{variant}"] = scans
        found = [x for x in scans if x["exists"]]
        print(f"A2 B{variant}: {len(found)}/8 scans")
        if found:
            print("  first pose_cache_source:",found[0].get("pose_cache_source","<missing>"))
            errs = [x["scan_pose_vs_A2_gt"]["position_max_m"] for x in found
                    if "scan_pose_vs_A2_gt" in x]
            if errs:
                print("  max sensor geometry pose vs GT (Falcon1):",max(errs),"m")
            elif gt_t is None:
                print("  GT absent: pose consistency not computed")

        opath = root / A2_ODOM[f"B{variant}"]
        if opath.exists():
            ot, _ = read_odom(opath)
            result["odometry"][f"B{variant}"] = {
                "samples":len(ot), "start":float(ot[0]), "end":float(ot[-1])
            }
            print(f"  odom: {len(ot)} samples, [{ot[0]:.6f},{ot[-1]:.6f}] s")

    print("NOTICE: NPZ pose_cache_source records the invoked generator cache path;")
    print("numerical pose-vs-GT residual is the stronger cross-check.")
    return result


def a3_audit(root: Path):
    print("\n========== PRIORITY 2: A3 5-POINT EARLY-WINDOW ==========")
    gt_path = root / A3_GT
    if not gt_path.exists():
        raise FileNotFoundError(gt_path)
    records = {}
    for drone in (1, 2, 3):
        truth_t, truth_pose = read_truth(gt_path, drone)
        for mode, path in (
            ("B0", root / "A3_1" / f"a3_1_drone_{drone}_odom.npz"),
            ("B1", root / "A3_1/A3_B1_three_plane" / f"a3_b1_drone_{drone}_odom.npz")
        ):
            if not path.exists():
                raise FileNotFoundError(path)
            odom_t, odom_pose = read_odom(path)
            records[(drone, mode)] = (truth_t, truth_pose, odom_t, odom_pose)

    # Same five query timestamps across all six runs: avoid cherry-picking.
    t0 = max(0.4, *(r[2][0] for r in records.values()))
    t1 = min(0.8, *(r[2][-1] for r in records.values()))
    if t1 <= t0 + 1e-4:
        raise RuntimeError(f"No common [0.4,0.8] interval: [{t0},{t1}]")
    query = np.linspace(t0, t1, 5)
    print("Shared 5-point grid (s):", query)
    result = {"window_s":[float(t0),float(t1)],
              "evaluation_times_s":query.tolist(),"five_point":{}, "drift":{}}

    for drone in (1,2,3):
        entry = {}
        for mode in ("B0","B1"):
            tr_t, tr_pose, o_t, o_pose = records[(drone, mode)]
            entry[mode] = metrics_on_query(tr_t,tr_pose,o_t,o_pose,query)
        improvement = 100*(1-entry["B1"]["position_rmse_m"]/entry["B0"]["position_rmse_m"])
        entry["position_rmse_change_percent"] = float(improvement)
        result["five_point"][f"Falcon{drone}"] = entry
        print(f"Falcon{drone}: B0={entry['B0']['position_rmse_m']:.6f} m; "
              f"B1={entry['B1']['position_rmse_m']:.6f} m; "
              f"improvement={improvement:+.2f}%")
        print(f"        orientation: B0={entry['B0']['orientation_rmse_deg']:.3f} deg; "
              f"B1={entry['B1']['orientation_rmse_deg']:.3f} deg")

    print("\n========== PRIORITY 3: A3 DRIFT / FIRST THRESHOLD ==========")
    for drone in (1,2,3):
        entry={}
        for mode, evalfile in (
            ("B0",root/"A3_1"/f"a3_1_drone_{drone}_evaluation.npz"),
            ("B1",root/"A3_1/A3_B1_three_plane"/f"a3_b1_drone_{drone}_evaluation.npz")
        ):
            # Existing full-length evaluation has first-pose alignment and
            # must be used AS-IS to diagnose long-run accumulated error.
            if evalfile.exists():
                with np.load(evalfile,allow_pickle=False) as z:
                    t=np.asarray(z["time_s"],dtype=float)
                    e=np.asarray(z["position_error_m"],dtype=float)
                    ang=np.asarray(z["rotation_error_deg"],dtype=float)
            else:
                tr_t,tr_pose,odom_t,odom_pose=records[(drone,mode)]
                t=odom_t
                # Full-run first-pose alignment, using all odom timestamps
                m=metrics_on_query(tr_t,tr_pose,odom_t,odom_pose,t)
                e=np.asarray(m["error_samples_m"])
                ang=np.asarray(m["orientation_samples_deg"])
            thresholds = {}
            for bound in (0.1,0.5,1.0,5.0,10.0):
                hit=np.flatnonzero(e>bound)
                thresholds[str(bound)] = float(t[hit[0]]) if len(hit) else None
            snapshots={}
            for tt in (0.5,0.8,1.,2.,3.,4.,4.9):
                k=int(np.argmin(abs(t-tt)))
                snapshots[str(tt)]={
                    "sample_time_s":float(t[k]),
                    "position_error_m":float(e[k]),
                    "orientation_error_deg":float(ang[k]),
                }
            entry[mode]={
                "samples":len(t),
                "first_crossing_time_s":thresholds,
                "error_at_nearest_time":snapshots,
                "full_run_rmse_m":float(np.sqrt(np.mean(e**2))),
                "full_run_attitude_rmse_deg":float(np.sqrt(np.mean(ang**2))),
            }
            print(f"Falcon{drone} {mode}: RMSE={entry[mode]['full_run_rmse_m']:.6f} m "
                  f"first crossing (s)={thresholds}")
        result["drift"][f"Falcon{drone}"]=entry
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--root",type=Path,default=Path.home()/"research/isaac_a1_diagnostics")
    parser.add_argument("--out",type=Path,default=None)
    parser.add_argument("--only",choices=("all","a2","a3"),default="all")
    args=parser.parse_args()
    root=args.root.expanduser().resolve()
    if not root.is_dir():
        parser.error(f"Missing data directory: {root}")
    report={"method":"offline A2/A3 data provenance and five-point/drift audit",
            "truth_used_for_evaluation_only":True}
    if args.only in ("a2","all"):
        report["A2"]=a2_audit(root)
    if args.only in ("a3","all"):
        report["A3"]=a3_audit(root)
    # Keep A2 and A3 audit outputs separate; never overwrite previous runs.
    suffix = args.only
    default_out = root / "A3_1/A3_B1_three_plane" / f"a2_a3_20261106_offline_audit_{suffix}.json"
    out=(args.out or default_out).expanduser()
    out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite existing audit: {out}")
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    print("\nOFFLINE AUDIT COMPLETE:",out)


if __name__=="__main__":
    main()
