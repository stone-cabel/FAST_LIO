#!/usr/bin/env python3
"""Prepare isolated A3-B2 diagnostic copies, without changing original files.

Run on Ubuntu AFTER creating a standalone FAST_LIO_ROS2 source copy.
Changes exactly one algorithm statement:
    init_state.bg  = mean_gyr;  -> init_state.bg  = V3D::Zero();
The rest of initialization, especially gravity, is unchanged.
This is a simulation/oracle diagnostic, NOT a deployment algorithm.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import hashlib
import json
import shutil


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def change_one(source: Path, dest: Path, original: str, replacement: str) -> dict:
    if not source.is_file():
        raise FileNotFoundError(source)
    if source.resolve() == dest.resolve():
        raise ValueError("Refusing to edit source in place")
    data = source.read_text(encoding="utf-8")
    count = data.count(original)
    if count != 1:
        raise ValueError(f"Guard: expected exactly one change target, found {count} in {source}")
    if dest.exists():
        raise FileExistsError(f"Refusing to overwrite {dest}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(data.replace(original, replacement, 1), encoding="utf-8")
    return {"source": str(source), "destination": str(dest),
            "source_sha256": digest(source), "destination_sha256": digest(dest),
            "original":original.strip(), "replacement":replacement.strip()}


def main():
    pa = argparse.ArgumentParser()
    pa.add_argument("--original-lio", type=Path,
                    default=Path.home()/"research/fastlio2_ws/src/FAST_LIO_ROS2")
    pa.add_argument("--b2-lio", type=Path,
                    default=Path.home()/"research/fastlio2_b2_ws/src/FAST_LIO_ROS2")
    pa.add_argument("--original-replay", type=Path,
                    default=Path.home()/"research/fastlio2_ws/a2_bridge/a3_b1_replay_three_uav.py")
    pa.add_argument("--b2-replay", type=Path,
                    default=Path.home()/"research/fastlio2_b2_ws/a3_b2_replay_three_uav.py")
    args = pa.parse_args()
    original_lio = args.original_lio.expanduser().resolve()
    b2_lio = args.b2_lio.expanduser()
    if not (original_lio/"src/IMU_Processing.hpp").is_file():
        raise FileNotFoundError(original_lio/"src/IMU_Processing.hpp")
    if b2_lio.exists():
        raise FileExistsError("B2 source already exists; refusing to overwrite " + str(b2_lio))
    if args.b2_replay.expanduser().exists():
        raise FileExistsError("B2 replay already exists; refusing to overwrite")
    b2_lio.parent.mkdir(parents=True,exist_ok=True)
    # Copy source tree into an entirely separate workspace, not the historical B1.
    shutil.copytree(original_lio, b2_lio, symlinks=False,
                    ignore=shutil.ignore_patterns(".git","build","install","log","__pycache__"))
    src_h = original_lio/"src/IMU_Processing.hpp"
    dst_h = b2_lio/"src/IMU_Processing.hpp"
    original = "  init_state.bg  = mean_gyr;"
    replacement = "  init_state.bg  = V3D::Zero(); // A3-B2 oracle bias diagnostic only"
    header = dst_h.read_text(encoding="utf-8")
    if header.count(original) != 1:
        raise ValueError(f"B2 guard: expected exactly 1 original init assignment, got {header.count(original)}")
    dst_h.write_text(header.replace(original, replacement, 1),encoding="utf-8")
    src_replay = args.original_replay.expanduser().resolve()
    dst_replay = args.b2_replay.expanduser()
    replay = change_one(src_replay, dst_replay,
                        'output_dir = root / "A3_1/A3_B1_three_plane"',
                        'output_dir = root / "A3_1/A3_B2_gyro_zero"')
    # Also name output files and node explicitly as B2, leaving B1 scan data unchanged
    d = dst_replay.read_text(encoding="utf-8")
    for old,new in [
        ('f"a3_b1_drone_{args.drone}_odom.npz"',
         'f"a3_b2_drone_{args.drone}_odom.npz"'),
        ('Node(f"a3_1_replay_drone_{args.drone}")',
         'Node(f"a3_b2_replay_drone_{args.drone}")'),
    ]:
        if d.count(old)!=1:
            raise ValueError(f"B2 replay guard failed: {old}")
        d=d.replace(old,new,1)
    dst_replay.write_text(d,encoding="utf-8")
    if "a3_b1_mid360_window_" not in d:
        raise ValueError("B2 must consume exactly the original B1 pointcloud scans")
    summary={
        "experiment":"A3-B2 diagnostic oracle gyro bias=0",
        "only_algorithm_change":"init_state.bg = V3D::Zero()",
        "gravity_initialization":"unchanged",
        "lidar_scans":"original A3-B1 three-plane NPZ",
        "imu_input":"original A3_1/a3_1_imu_5s_300hz.npz",
        "header_original_sha256":digest(src_h),
        "header_b2_sha256":digest(dst_h),
        "replay":replay,
        "b2_replay_sha256":digest(dst_replay)
    }
    manifest=b2_lio.parent.parent/"A3_B2_PREPARED_MANIFEST.json"
    if manifest.exists():
        raise FileExistsError(manifest)
    manifest.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    print("B2 source and replay prepared. No build/run performed.")


if __name__=="__main__":
    main()
