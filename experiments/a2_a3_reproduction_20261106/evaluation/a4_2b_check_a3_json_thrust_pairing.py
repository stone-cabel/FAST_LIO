#!/usr/bin/env python3
"""A4-2b read-only audit: are candidate controller JSON thrust records paired
with the exact A3 300Hz PhysX pose trace? Never joins data unless validated.

JSON can have top-level 'per_step', nested 'per_step', etc. The audit finds
a list of records with time_s, uav_positions_w_m, rotor_collective_thrust_N.
Only compares World-frame UAV positions at synchronized timestamps; full
provenance, time interpretation, frames and forces still need checking.
"""
import argparse
import json
from pathlib import Path
import numpy as np

KEYS = {"time_s", "uav_positions_w_m", "rotor_collective_thrust_N"}


def find_records(obj, depth=0):
    if depth > 3:
        return []
    if isinstance(obj, list):
        if obj and isinstance(obj[0], dict) and KEYS.issubset(obj[0]):
            return [obj]
        return []
    if isinstance(obj, dict):
        found = []
        for k, v in obj.items():
            if k in ("per_step", "per_step_records", "samples", "records", "steps", "per_step_metrics", "trajectory"):
                found.extend(find_records(v, depth + 1))
        return found
    return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path.home() / "research/isaac_a1_diagnostics")
    ap.add_argument("--json-root", type=Path, default=Path.home() / "research/payload_ekf_sun_E1/results")
    ap.add_argument("--max-files", type=int, default=80)
    args = ap.parse_args()
    gt_file = args.root.expanduser() / "a1_47_mappo_eskf_500steps_300hz.npz"
    with np.load(gt_file, allow_pickle=False) as d:
        time_gt = np.asarray(d["time_s"], dtype=float)
        names = list(map(str, d["body_names"]))
        pose = np.asarray(d["body_link_pose_w_xyzw_300hz"], dtype=float)
    if not np.all(np.diff(time_gt) > 0):
        raise ValueError("GT times not increasing")
    expected_names = [f"Falcon{i}_base_link_inertia" for i in (1,2,3)]
    idx = [names.index(n) for n in expected_names]
    p_gt = pose[:, idx, :3]

    candidates = []
    for p in sorted(args.json_root.expanduser().rglob("*.json")):
        if any(part not in ("E7_1", "E7_1_E", "E7_1_F", "E6_6") for part in p.parts if part.startswith(("E7_", "E6_"))):
            continue
        if any(tag in str(p) for tag in ("E7_1/", "E7_1_E/", "E7_1_F/", "E6_6/")):
            candidates.append(p)
    candidates = candidates[:args.max_files]
    print("Reference:", gt_file, "GT sample count:", len(time_gt))
    print("Reference UAVs:", expected_names)
    print("JSON files found:", len(candidates))
    findings = []
    for path in candidates:
        try:
            with path.open(encoding="utf-8") as f:
                obj = json.load(f)
            sets = find_records(obj)
            if not sets:
                print("NO_ROW_SCHEMA", path)
                if isinstance(obj, dict):
                    print("  top-level keys:", sorted(obj.keys())[:25])
                continue
            for records in sets[:1]:
                ts=[]; ps=[]; thrust=[]
                for row in records:
                    try:
                        t=float(row["time_s"])
                        pp=np.asarray(row["uav_positions_w_m"],float)
                        f=np.asarray(row["rotor_collective_thrust_N"],float)
                        if pp.shape == (3,3) and f.shape == (3,) and np.isfinite(t) and np.isfinite(pp).all() and np.isfinite(f).all():
                            ts.append(t); ps.append(pp); thrust.append(f)
                    except (KeyError, ValueError, TypeError):
                        continue
                if not ts:
                    print("NO_VALID_ROWS",path)
                    continue
                ts=np.asarray(ts)
                ps=np.asarray(ps)
                thrust=np.asarray(thrust)
                mask=(ts >= time_gt[0]) & (ts <= time_gt[-1]) & (ts >= 0.4) & (ts <= 4.9)
                if not np.any(mask):
                    print("NO_OVERLAP",path, "JSON time range:",float(ts.min()),float(ts.max()))
                    continue
                tt=ts[mask]; p=ps[mask]
                pg=np.stack([np.column_stack([np.interp(tt,time_gt,p_gt[:,i,j]) for j in range(3)]) for i in range(3)],axis=1)
                err=np.linalg.norm(p-pg,axis=-1)
                rms=float(np.sqrt(np.mean(err**2)))
                maxerr=float(np.max(err))
                info=(rms,maxerr,len(tt),str(path),float(tt[0]),float(tt[-1]))
                findings.append(info)
                print(f"RMSE={rms:.6f} m MAX={maxerr:.6f} m rows={len(tt):3d} "
                      f"time=[{tt[0]:.3f},{tt[-1]:.3f}] thrust_min={thrust.min():.5g} "
                      f"thrust_max={thrust.max():.5g} {path}")
        except Exception as ex:
            print("SKIP_ERROR",type(ex).__name__,str(ex),path)
    print("\n========== CLOSEST FILES (LOWER POSITION RMSE FIRST) ==========")
    for rms,mx,n,p,t0,t1 in sorted(findings)[:10]:
        print(f"RMSE={rms:.6f}m MAX={mx:.6f}m n={n} [{t0:.2f},{t1:.2f}] {p}")
    print("\nThis only checks position/time agreement. It does not prove run identity or valid motor-force correspondence.")
    print("Only if a candidate matches to high accuracy should its metadata, timestamps, frames,")
    print("controls, and simulator provenance be audited before extracting thrust.")
if __name__=="__main__":
    main()
