#!/usr/bin/env python3

from pathlib import Path
import numpy as np

ROOT = Path.home() / "research/isaac_a1_diagnostics"

path = ROOT / "a2_4b_1_mid360_window_00_2000rays.npz"

assert path.is_file(), path

with np.load(path, allow_pickle=False) as d:
    names = [str(x) for x in d["scene_names"]]
    ids = np.asarray(d["object_id_moving"][0])
    xyz = np.asarray(
        d["xyz_world_at_fire"][0],
        dtype=np.float64
    )
    ranges = np.asarray(d["ranges_moving_m"][0])

valid = (
    np.isfinite(ranges)
    & (ranges > 0.5)
    & np.isfinite(xyz).all(axis=1)
)

objects = ["ground", "test_wall", "test_wall_x"]

normals = []
counts = []

print("========== A2-4B-1 OBSERVABILITY AUDIT ==========")

for name in objects:

    obj_id = names.index(name)

    points = xyz[valid & (ids == obj_id)]

    assert len(points) >= 20, (
        f"Insufficient points: {name}, count={len(points)}"
    )

    center = points.mean(axis=0)
    centered = points - center

    _, _, vh = np.linalg.svd(
        centered,
        full_matrices=False
    )

    normal = vh[-1]
    normal /= np.linalg.norm(normal)

    rms = np.sqrt(
        np.mean((centered @ normal) ** 2)
    )

    normals.append(normal)
    counts.append(len(points))

    print(f"\n{name}")
    print("Points:", len(points))
    print("Normal:", normal)
    print("Plane RMS:", rms, "m")

H = np.zeros((3, 3), dtype=np.float64)

for normal, count in zip(normals, counts):
    H += count * np.outer(normal, normal)

eigenvalues, eigenvectors = np.linalg.eigh(H)

threshold = 1e-8 * max(eigenvalues[-1], 1.0)
rank = int(np.sum(eigenvalues > threshold))

print("\n========== TRANSLATION INFORMATION ==========")

print("H_t:")
print(H)

print("\nEigenvalues:")
print(eigenvalues)

print("\nEstimated rank:", rank)

print("\nSmallest-eigenvalue direction:")
print(eigenvectors[:, 0])

print("\nMinimum eigenvalue:", eigenvalues[0])

print(
    "\nCondition number:",
    eigenvalues[-1] / max(eigenvalues[0], 1e-15)
)

if rank == 3:
    print("\nA2-4B-1 TRANSLATION RANK: PASS")
else:
    print("\nA2-4B-1 TRANSLATION RANK: FAIL")

print(
    "\nNOTE: Plane-level translation information only; "
    "not the complete FAST-LIO2 IEKF."
)
