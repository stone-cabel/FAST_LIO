#!/usr/bin/env python3

from pathlib import Path
import numpy as np

ROOT = Path.home() / "research/isaac_a1_diagnostics"

files = {
    "B0_two_planes": ROOT / "a2_4b_0_single_odom.npz",
    "B1_three_planes": ROOT / "a2_4b_1_single_odom.npz",
}

results = {}

print("========== A2-4B GEOMETRY COMPARISON ==========")

for label, path in files.items():

    assert path.is_file(), path

    with np.load(path, allow_pickle=False) as d:
        odom = np.asarray(
            d["odom_time_pose_xyzw"],
            dtype=np.float64
        )

    assert odom.ndim == 2
    assert odom.shape[1] == 8
    assert len(odom) >= 2
    assert np.isfinite(odom).all()

    t = odom[:, 0]
    p = odom[:, 1:4]

    assert np.all(np.diff(t) > 0), (
        f"{label}: timestamps not strictly increasing"
    )

    results[label] = (t, p)

    print(f"\n{label}")
    print("ODOM samples:", len(t))
    print("Time range:", t[0], t[-1])

    for ti, pi in zip(t, p):
        print(
            f"t={ti:.9f} "
            f"p=[{pi[0]:+.6f}, "
            f"{pi[1]:+.6f}, "
            f"{pi[2]:+.6f}]"
        )

# 两组共同覆盖的时间范围。
t0 = max(
    results[name][0][0]
    for name in results
)

t1 = min(
    results[name][0][-1]
    for name in results
)

assert t1 > t0, "No common time interval"

def interpolate(t, p, query):
    return np.array([
        np.interp(query, t, p[:, i])
        for i in range(3)
    ])

print("\n========== COMMON INTERVAL ==========")
print(f"Start: {t0:.9f} s")
print(f"End:   {t1:.9f} s")

for label, (t, p) in results.items():

    p0 = interpolate(t, p, t0)
    p1 = interpolate(t, p, t1)

    delta = p1 - p0

    print(f"\n{label}")
    print("Displacement:", delta)
    print("Displacement norm:", np.linalg.norm(delta), "m")
    print("X displacement:", delta[0], "m")

print("\nNOTE:")
print("Results are displacement magnitudes, not aligned ATE.")
print("Time interpolation is used only within observed ranges.")

print("\nA2-4B GEOMETRY COMPARISON: PASS")
