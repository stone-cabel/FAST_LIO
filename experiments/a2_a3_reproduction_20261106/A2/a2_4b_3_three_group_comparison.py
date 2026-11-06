#!/usr/bin/env python3

from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path.home() / "research/isaac_a1_diagnostics"

GT_FILE = ROOT / "a2_4a_7_init_7p2_to_8p0_300hz.npz"

ODOM_FILES = {
    "B0_two_planes":
        ROOT / "a2_4b_0_single_odom.npz",

    "B1_three_planes":
        ROOT / "a2_4b_1_single_odom.npz",

    "B3_equal_points":
        ROOT / "a2_4b_3_retry_odom.npz",
}


def load_gt():
    with np.load(GT_FILE, allow_pickle=False) as d:
        t = np.asarray(d["time_s"], dtype=np.float64)

        names = [str(x) for x in d["body_names"]]
        idx = names.index("Falcon1_base_link_inertia")

        pose = np.asarray(
            d["body_link_pose_w_xyzw_300hz"][:, idx],
            dtype=np.float64
        )

    return t, pose


def load_odom(path):
    assert path.is_file(), path

    with np.load(path, allow_pickle=False) as d:
        data = np.asarray(
            d["odom_time_pose_xyzw"],
            dtype=np.float64
        )

    assert data.ndim == 2
    assert data.shape[1] == 8
    assert len(data) >= 2
    assert np.isfinite(data).all()

    return data[:, 0], data[:, 1:8]


def interpolate_pose(t, pose, query):
    t = np.asarray(t, dtype=np.float64)
    query = np.asarray(query, dtype=np.float64)

    assert np.all(np.diff(t) > 0)
    assert query[0] >= t[0] - 1e-9
    assert query[-1] <= t[-1] + 1e-9

    p = np.column_stack([
        np.interp(query, t, pose[:, i])
        for i in range(3)
    ])

    q = pose[:, 3:7].copy()
    norms = np.linalg.norm(q, axis=1)

    assert np.all(np.abs(norms - 1.0) < 1e-3)

    q /= norms[:, None]

    r = Slerp(
        t,
        Rotation.from_quat(q)
    )(query)

    return p, r


print("========== A2-4B-3 THREE-GROUP COMPARISON ==========")

t_gt, pose_gt = load_gt()

groups = {
    name: load_odom(path)
    for name, path in ODOM_FILES.items()
}

print("\n========== RAW ODOM ==========")

for name, (t, pose) in groups.items():
    assert np.all(np.diff(t) > 0), (
        f"{name}: duplicate/non-increasing timestamps"
    )

    print(
        f"{name}: samples={len(t)} "
        f"start={t[0]:.9f} "
        f"end={t[-1]:.9f}"
    )

t0 = max(
    t_gt[0],
    *(t[0] for t, _ in groups.values())
)

t1 = min(
    t_gt[-1],
    *(t[-1] for t, _ in groups.values())
)

assert t1 > t0, "No common time interval"

query = np.linspace(t0, t1, 5)

p_gt, r_gt = interpolate_pose(
    t_gt, pose_gt, query
)

delta_gt = p_gt - p_gt[0]

print("\n========== COMMON INTERVAL ==========")
print(f"Start: {t0:.9f} s")
print(f"End:   {t1:.9f} s")
print(f"Duration: {t1-t0:.9f} s")
print("PhysX GT delta:", delta_gt[-1])
print(
    "PhysX GT displacement:",
    np.linalg.norm(delta_gt[-1]), "m"
)

summary = {}

for name, (t, pose) in groups.items():

    p_est, r_est = interpolate_pose(
        t, pose, query
    )

    R_align = r_gt[0] * r_est[0].inv()

    delta_est = R_align.apply(
        p_est - p_est[0]
    )

    error_vec = delta_est - delta_gt
    errors = np.linalg.norm(error_vec, axis=1)

    rmse = float(
        np.sqrt(np.mean(errors ** 2))
    )

    endpoint = float(errors[-1])

    angle_error = np.degrees(
        (r_gt * (R_align * r_est).inv()).magnitude()
    )

    angle_rmse = float(
        np.sqrt(np.mean(angle_error ** 2))
    )

    summary[name] = {
        "rmse": rmse,
        "endpoint": endpoint,
        "angle_rmse": angle_rmse,
    }

    print(f"\n========== {name} ==========")

    print("Initial alignment quaternion xyzw:")
    print(R_align.as_quat())

    print("Estimated delta in GT frame:")
    print(delta_est[-1])

    print("Endpoint error vector:")
    print(error_vec[-1])

    print("\nTime | Position error (m) | Angle error (deg)")

    for ti, ep, ea in zip(query, errors, angle_error):
        print(f"{ti:.9f} | {ep:.9f} | {ea:.6f}")

    print(f"\nPosition RMSE: {rmse:.9f} m")
    print(f"Endpoint vector error: {endpoint:.9f} m")
    print(f"Orientation RMSE: {angle_rmse:.6f} deg")


print("\n========== FINAL COMPARISON ==========")

print(
    f"{'Group':22s} "
    f"{'Pos RMSE(m)':>13s} "
    f"{'End Err(m)':>13s} "
    f"{'Rot RMSE(deg)':>15s}"
)

for name, result in summary.items():
    print(
        f"{name:22s} "
        f"{result['rmse']:13.9f} "
        f"{result['endpoint']:13.9f} "
        f"{result['angle_rmse']:15.6f}"
    )


b0 = summary["B0_two_planes"]
b1 = summary["B1_three_planes"]
b3 = summary["B3_equal_points"]

print("\n========== RELATIVE IMPROVEMENT ==========")

for label, result in [
    ("B1 vs B0", b1),
    ("B3 vs B0", b3),
]:
    improvement = (
        1.0 - result["rmse"] / b0["rmse"]
    ) * 100

    print(
        f"{label} position RMSE reduction: "
        f"{improvement:.3f}%"
    )

print(
    "B3/B1 position RMSE ratio:",
    b3["rmse"] / b1["rmse"]
)

print("\nLIMITATIONS:")
print("- Common interval across B0, B1, B3.")
print("- Five interpolated evaluation timestamps.")
print("- Initial attitude alignment, not global SE(3) fitting.")
print("- GT/LIO body-frame correspondence assumed.")
print("- Single replay per successful configuration.")
print("- Short trajectory; not long-term localization accuracy.")

print("\nA2-4B-3 THREE-GROUP COMPARISON: PASS")
