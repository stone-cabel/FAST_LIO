#!/usr/bin/env python3

from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path.home() / "research/isaac_a1_diagnostics"

GT_FILE = ROOT / "a2_4a_7_init_7p2_to_8p0_300hz.npz"

ODOM_FILES = {
    "B0_two_planes": ROOT / "a2_4b_0_single_odom.npz",
    "B1_three_planes": ROOT / "a2_4b_1_single_odom.npz",
}


def load_gt():
    with np.load(GT_FILE, allow_pickle=False) as d:
        t = np.asarray(d["time_s"], dtype=np.float64)
        names = [str(x) for x in d["body_names"]]

        idx = names.index("Falcon1_base_link_inertia")

        pose = np.asarray(
            d["body_link_pose_w_xyzw_300hz"][:, idx, :],
            dtype=np.float64
        )

    return t, pose


def load_odom(path):
    with np.load(path, allow_pickle=False) as d:
        pose = np.asarray(
            d["odom_time_pose_xyzw"],
            dtype=np.float64
        )

    return pose[:, 0], pose[:, 1:8]


def interpolate_pose(t, pose, query):
    query = np.asarray(query, dtype=np.float64)

    assert np.all(np.diff(t) > 0)
    assert query.min() >= t[0] - 1e-9
    assert query.max() <= t[-1] + 1e-9

    p = np.column_stack([
        np.interp(query, t, pose[:, axis])
        for axis in range(3)
    ])

    q = pose[:, 3:7].copy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)

    rotations = Rotation.from_quat(q)
    q_interp = Slerp(t, rotations)(query)

    return p, q_interp


print("========== A2-4B-2 RELATIVE POSE AUDIT ==========")

t_gt, gt_pose = load_gt()

results = {
    label: load_odom(path)
    for label, path in ODOM_FILES.items()
}

# 所有轨迹共同覆盖的时间区间。
t0 = max(
    t_gt[0],
    *(t[0] for t, _ in results.values())
)

t1 = min(
    t_gt[-1],
    *(t[-1] for t, _ in results.values())
)

assert t1 > t0

# 采用固定的5个共同评价时刻。
# 不将高频插值结果误当作新增独立观测。
query = np.linspace(t0, t1, 5)

p_gt, r_gt = interpolate_pose(
    t_gt, gt_pose, query
)

delta_gt = p_gt - p_gt[0]

print(f"\nCommon interval: {t0:.9f} -> {t1:.9f} s")
print("Evaluation timestamps:", query)
print("GT displacement:", delta_gt[-1])

summary = {}

for label, (t, pose) in results.items():

    p_est, r_est = interpolate_pose(
        t, pose, query
    )

    # 首帧姿态对齐：LIO -> PhysX 世界坐标方向。
    R_align = (
        r_gt[0] * r_est[0].inv()
    )

    delta_est = p_est - p_est[0]
    delta_est_world = R_align.apply(delta_est)

    # 每个时刻的三维相对位置误差。
    error_vectors = delta_est_world - delta_gt

    errors = np.linalg.norm(
        error_vectors,
        axis=1
    )

    rmse = np.sqrt(np.mean(errors ** 2))
    endpoint_error = errors[-1]
    max_error = np.max(errors)

    # 姿态诊断：计算初始对齐后相对姿态的差异。
    r_est_aligned = R_align * r_est
    rotation_error = (
        r_gt * r_est_aligned.inv()
    )

    angle_errors_deg = np.degrees(
        rotation_error.magnitude()
    )

    summary[label] = {
        "rmse": rmse,
        "endpoint": endpoint_error,
        "max": max_error,
        "angle_rmse": np.sqrt(
            np.mean(angle_errors_deg ** 2)
        ),
    }

    print(f"\n========== {label} ==========")

    print("Initial alignment rotation (xyzw):")
    print(R_align.as_quat())

    print("\nTime | Position error (m) | Angle error (deg)")

    for ti, ep, ea in zip(
        query, errors, angle_errors_deg
    ):
        print(
            f"{ti:.9f} | "
            f"{ep:.9f} | "
            f"{ea:.6f}"
        )

    print("\nEstimated relative displacement in GT frame:")
    print(delta_est_world[-1])

    print("\nGT relative displacement:")
    print(delta_gt[-1])

    print(f"\nRelative position RMSE: {rmse:.9f} m")
    print(f"Endpoint vector error: {endpoint_error:.9f} m")
    print(f"Maximum position error: {max_error:.9f} m")
    print(
        "Relative orientation RMSE:",
        f"{summary[label]['angle_rmse']:.6f} deg"
    )


b0 = summary["B0_two_planes"]
b1 = summary["B1_three_planes"]

print("\n========== IMPROVEMENT ==========")

rmse_reduction = (
    1 - b1["rmse"] / b0["rmse"]
) * 100

endpoint_reduction = (
    1 - b1["endpoint"] / b0["endpoint"]
) * 100

print(
    f"Relative position RMSE reduction: "
    f"{rmse_reduction:.3f}%"
)

print(
    f"Endpoint vector error reduction: "
    f"{endpoint_reduction:.3f}%"
)

print("\nLIMITATIONS:")
print("- Initial attitude alignment, not optimized SE(3) alignment.")
print("- GT body frame and LIO body frame equivalence assumed.")
print("- 5 evaluation times, with trajectory interpolation.")
print("- Short low-motion diagnostic window.")
print("- Results do not establish long-term localization accuracy.")

print("\nA2-4B-2 RELATIVE POSE AUDIT: PASS")
