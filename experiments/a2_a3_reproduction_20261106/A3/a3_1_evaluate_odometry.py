#!/usr/bin/env python3
"""Evaluate three FAST-LIO2 trajectories against held-out PhysX truth.

Initial-pose alignment is for OFFLINE EVALUATION ONLY.
"""

from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path.home() / "research/isaac_a1_diagnostics"
OUT = ROOT / "A3_1"

with np.load(
    ROOT / "a1_47_mappo_eskf_500steps_300hz.npz",
    allow_pickle=False
) as d:
    truth_t = np.asarray(d["time_s"], dtype=float)
    poses = np.asarray(
        d["body_link_pose_w_xyzw_300hz"], dtype=float
    )
    names = [str(x) for x in d["body_names"]]

print("=" * 70)
print("A3-1 FAST-LIO2 ODOMETRY EVALUATION")
print("=" * 70)

for drone in range(1, 4):
    name = f"Falcon{drone}_base_link_inertia"
    idx = names.index(name)

    file = OUT / f"a3_1_drone_{drone}_odom.npz"

    print(f"\n========== FALCON {drone} ==========")

    if not file.exists():
        print("MISSING:", file)
        continue

    with np.load(file, allow_pickle=False) as d:
        odom = np.asarray(d["odom_time_pose_xyzw"], dtype=float)

    if len(odom) < 2:
        print("Insufficient odometry:", len(odom))
        continue

    t = odom[:, 0]
    est_p = odom[:, 1:4]
    est_q = odom[:, 4:8]

    assert np.isfinite(odom).all()
    assert np.all(np.diff(t) > 0)
    assert t[0] >= truth_t[0]
    assert t[-1] <= truth_t[-1]

    # Time-interpolate ground truth.
    true_p = np.column_stack([
        np.interp(t, truth_t, poses[:, idx, axis])
        for axis in range(3)
    ])

    true_R = Slerp(
        truth_t,
        Rotation.from_quat(poses[:, idx, 3:7])
    )(t)

    est_R = Rotation.from_quat(est_q)

    # Align only at the FIRST available odometry timestamp.
    # No full-trajectory fitting is performed.
    R_align = true_R[0] * est_R[0].inv()

    aligned_p = (
        true_p[0]
        + R_align.apply(est_p - est_p[0])
    )

    aligned_R = R_align * est_R

    position_error = np.linalg.norm(
        aligned_p - true_p, axis=1
    )

    rotation_error = (
        aligned_R.inv() * true_R
    ).magnitude() * 180.0 / np.pi

    estimated_displacement = np.linalg.norm(
        est_p[-1] - est_p[0]
    )

    true_displacement = np.linalg.norm(
        true_p[-1] - true_p[0]
    )

    position_rmse = np.sqrt(
        np.mean(position_error**2)
    )

    rotation_rmse = np.sqrt(
        np.mean(rotation_error**2)
    )

    print("Samples:", len(t))
    print("Time interval:", float(t[0]), "to", float(t[-1]))
    print("Estimated displacement:",
          round(float(estimated_displacement), 4), "m")
    print("True displacement:",
          round(float(true_displacement), 4), "m")

    print("Position RMSE:",
          round(float(position_rmse), 4), "m")
    print("Position final error:",
          round(float(position_error[-1]), 4), "m")
    print("Position max error:",
          round(float(position_error.max()), 4), "m")

    print("Orientation RMSE:",
          round(float(rotation_rmse), 4), "deg")
    print("Orientation final error:",
          round(float(rotation_error[-1]), 4), "deg")

    result = OUT / f"a3_1_drone_{drone}_evaluation.npz"

    np.savez_compressed(
        result,
        time_s=t,
        estimated_position_aligned=aligned_p,
        true_position_world=true_p,
        position_error_m=position_error,
        rotation_error_deg=rotation_error,
        alignment_method=np.asarray(
            "First-odometry-pose SE3 alignment; offline evaluation only"
        )
    )

    print("Saved:", result)

print("\nA3-1 EVALUATION COMPLETE")
