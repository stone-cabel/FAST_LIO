#!/usr/bin/env python3

from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path.home() / "research/isaac_a1_diagnostics"

SOURCE = ROOT / "a1_47_mappo_eskf_500steps_300hz.npz"
OUTPUT = ROOT / "a2_2a_imu_300hz_diagnostic.npz"

# Isaac Lab world frame: Z up.
GRAVITY_W = np.array([0.0, 0.0, -9.81])

with np.load(SOURCE, allow_pickle=False) as f:
    t = np.asarray(f["time_s"], dtype=np.float64)
    poses = np.asarray(
        f["body_link_pose_w_xyzw_300hz"],
        dtype=np.float64
    )
    names = [str(x) for x in f["body_names"]]

assert poses.shape == (1501, 19, 7)
assert t.shape == (1501,)
assert np.all(np.diff(t) > 0)

dt = np.diff(t)

print("Time range:", t[0], t[-1])
print("dt min/max:", dt.min(), dt.max())

data = {
    "time_s": t,
    "gravity_world_mps2": GRAVITY_W,
    "source": str(SOURCE),
    "imu_model": "POSE_DERIVATIVE_DIAGNOSTIC_ONLY",
}

for drone in range(1, 4):

    name = f"Falcon{drone}_base_link_inertia"

    if name not in names:
        raise KeyError(f"Missing body: {name}")

    idx = names.index(name)

    p = poses[:, idx, :3]
    q = poses[:, idx, 3:7]

    qnorm = np.linalg.norm(q, axis=1)

    if not np.all(np.isfinite(qnorm)):
        raise ValueError(f"Nonfinite quaternion: {name}")

    if np.max(np.abs(qnorm - 1.0)) > 1e-3:
        raise ValueError(f"Nonunit quaternion: {name}")

    R = Rotation.from_quat(q)

    # World-frame numerical velocity and acceleration.
    v_world = np.gradient(
        p, t, axis=0, edge_order=2
    )

    a_world = np.gradient(
        v_world, t, axis=0, edge_order=2
    )

    # Accelerometer measures specific force in its own frame.
    # Diagnostic assumption: IMU origin and axes coincide
    # with this body's reference frame.
    specific_force_body = R.inv().apply(
        a_world - GRAVITY_W
    )

    # Relative rotations give body-frame angular increments.
    relative_rot = R[:-1].inv() * R[1:]

    omega_interval = (
        relative_rot.as_rotvec() / dt[:, None]
    )

    # Approximate angular velocity at each sample.
    omega_body = np.zeros_like(p)

    omega_body[0] = omega_interval[0]
    omega_body[-1] = omega_interval[-1]

    omega_body[1:-1] = (
        omega_interval[:-1] + omega_interval[1:]
    ) / 2.0

    if not np.all(np.isfinite(specific_force_body)):
        raise ValueError(f"Invalid acceleration: {name}")

    if not np.all(np.isfinite(omega_body)):
        raise ValueError(f"Invalid angular velocity: {name}")

    prefix = f"drone_{drone}"

    data[f"{prefix}_accel_body_mps2"] = (
        specific_force_body.astype(np.float32)
    )

    data[f"{prefix}_gyro_body_radps"] = (
        omega_body.astype(np.float32)
    )

    data[f"{prefix}_orientation_xyzw"] = (
        q.astype(np.float32)
    )

    print(f"\n{name}")

    print(
        "Specific force mean:",
        np.mean(specific_force_body, axis=0)
    )

    print(
        "Specific force norm p50/p95:",
        np.percentile(
            np.linalg.norm(specific_force_body, axis=1),
            [50, 95]
        )
    )

    print(
        "Gyro norm p50/p95:",
        np.percentile(
            np.linalg.norm(omega_body, axis=1),
            [50, 95]
        )
    )

np.savez_compressed(OUTPUT, **data)

print("\nOUTPUT:", OUTPUT)
print("A2-2A DIAGNOSTIC IMU DATA: GENERATED")
