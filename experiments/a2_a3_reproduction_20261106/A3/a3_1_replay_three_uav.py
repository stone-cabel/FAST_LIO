#!/usr/bin/env python3
"""A3-1: Replay one UAV from the synchronized 3-UAV LiDAR-IMU dataset.

Uses simulated raw moving-scan ranges and pose-derived diagnostic IMU.
No PhysX ground-truth pose is published as odometry.
Run separately for UAV 1, 2 and 3, restarting FAST-LIO2 each time.
"""

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from livox_ros_driver2.msg import CustomMsg, CustomPoint


ROOT = Path.home() / "research/isaac_a1_diagnostics"


def set_stamp(field, seconds):
    ns = int(round(float(seconds) * 1e9))
    field.sec = ns // 1_000_000_000
    field.nanosec = ns % 1_000_000_000


def load_scan(root, k, drone_id, min_range):
    path = root / f"a1_48_mid360_window_{k:02d}_2000rays.npz"

    with np.load(path, allow_pickle=False) as d:
        start = float(d["scan_start_time_s"])
        offsets = np.asarray(d["time_offset_ns"], dtype=np.int64)
        directions = np.asarray(d["directions_sensor"], dtype=float)
        ranges = np.asarray(
            d["ranges_moving_m"][drone_id - 1], dtype=float
        )

        assert int(d["scan_window_index"]) == k

        assert np.allclose(
            d["absolute_beam_time_s"],
            start + offsets * 1e-9,
            atol=1e-8,
            rtol=0,
        )

    assert offsets.shape == (2000,)
    assert directions.shape == (2000, 3)
    assert ranges.shape == (2000,)
    assert np.all(np.diff(offsets) > 0)

    valid = (
        np.isfinite(ranges)
        & (ranges > min_range)
        & np.isfinite(directions).all(axis=1)
    )

    indices = np.flatnonzero(valid)

    if len(indices) < 2:
        raise RuntimeError(
            f"UAV {drone_id} scan {k}: insufficient valid returns"
        )

    xyz = ranges[indices, None] * directions[indices]

    msg = CustomMsg()
    set_stamp(msg.header.stamp, start)
    msg.header.frame_id = f"drone_{drone_id}/livox"
    msg.timebase = int(round(start * 1e9))
    msg.lidar_id = drone_id
    msg.rsvd = [0, 0, 0]

    points = []

    for index, coordinates in zip(indices, xyz):
        p = CustomPoint()
        p.offset_time = int(offsets[index])
        p.x, p.y, p.z = (float(v) for v in coordinates)
        p.reflectivity = 0
        p.tag = 0
        p.line = 0
        points.append(p)

    msg.points = points
    msg.point_num = len(points)

    return start, msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()

    if not 0.05 <= args.rate <= 2.0:
        parser.error("--rate must be between 0.05 and 2.0")

    root = args.root.expanduser()
    output_dir = root / "A3_1"
    output_dir.mkdir(exist_ok=True)

    imu_path = output_dir / "a3_1_imu_5s_300hz.npz"

    with np.load(imu_path, allow_pickle=False) as d:
        imu_times = np.asarray(d["time_s"], dtype=float)
        accel = np.asarray(
            d["accel_body_mps2"][:, args.drone - 1, :],
            dtype=float,
        )
        gyro = np.asarray(
            d["gyro_body_radps"][:, args.drone - 1, :],
            dtype=float,
        )

    assert imu_times.shape == (1501,)
    assert accel.shape == (1501, 3)
    assert gyro.shape == (1501, 3)
    assert np.isfinite(accel).all()
    assert np.isfinite(gyro).all()
    assert np.all(np.diff(imu_times) > 0)

    scans = [
        load_scan(root, k, args.drone, args.min_range)
        for k in range(50)
    ]

    scan_times = np.array([item[0] for item in scans])

    assert np.allclose(
        scan_times, np.arange(50) * 0.1, atol=1e-8
    )

    events = [
        (float(t), 0, k)
        for k, t in enumerate(imu_times)
    ]

    events += [
        (float(t), 1, k)
        for k, t in enumerate(scan_times)
    ]

    # IMU first at equal timestamps.
    events.sort(key=lambda x: (x[0], x[1]))

    rclpy.init()
    node = Node(f"a3_1_replay_drone_{args.drone}")

    lidar_topic = f"/drone_{args.drone}/livox/lidar"
    imu_topic = f"/drone_{args.drone}/imu"
    odom_topic = f"/drone_{args.drone}/odometry"

    lidar_pub = node.create_publisher(CustomMsg, lidar_topic, 20)
    imu_pub = node.create_publisher(Imu, imu_topic, 50)

    clock_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    clock_pub = node.create_publisher(Clock, "/clock", clock_qos)

    received = []

    def on_odom(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        t = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )

        received.append(
            (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        )

    sub = node.create_subscription(
        Odometry, odom_topic, on_odom, 20
    )

    def publish_clock(t):
        msg = Clock()
        set_stamp(msg.clock, t)
        clock_pub.publish(msg)

    def drain(count=5):
        for _ in range(count):
            rclpy.spin_once(node, timeout_sec=0.005)

    print("=" * 60, flush=True)
    print(f"A3-1 REPLAY UAV {args.drone}", flush=True)
    print("LiDAR:", lidar_topic, flush=True)
    print("IMU:  ", imu_topic, flush=True)
    print("Odom: ", odom_topic, flush=True)
    print("Scans:", len(scans), flush=True)
    print("IMU:", len(imu_times), flush=True)
    print("Rate:", args.rate, flush=True)
    print("=" * 60, flush=True)

    try:
        # Allow DDS discovery and simulated-time initialization.
        time.sleep(2.0)

        for _ in range(10):
            publish_clock(0.0)
            drain(2)
            time.sleep(0.05)

        start_wall = time.monotonic()

        sent_imu = 0
        sent_lidar = 0

        for t, kind, index in events:
            target_wall = start_wall + t / args.rate
            remaining = target_wall - time.monotonic()

            if remaining > 0:
                time.sleep(remaining)

            publish_clock(t)

            if kind == 0:
                msg = Imu()
                set_stamp(msg.header.stamp, t)
                msg.header.frame_id = f"drone_{args.drone}/imu"

                # Orientation deliberately unavailable.
                msg.orientation_covariance[0] = -1.0

                msg.angular_velocity.x = float(gyro[index, 0])
                msg.angular_velocity.y = float(gyro[index, 1])
                msg.angular_velocity.z = float(gyro[index, 2])

                msg.linear_acceleration.x = float(accel[index, 0])
                msg.linear_acceleration.y = float(accel[index, 1])
                msg.linear_acceleration.z = float(accel[index, 2])

                imu_pub.publish(msg)
                sent_imu += 1

            else:
                scan_msg = scans[index][1]
                lidar_pub.publish(scan_msg)
                sent_lidar += 1

                if index % 10 == 0 or index == 49:
                    print(
                        f"scan={index:02d} "
                        f"t={t:.3f} "
                        f"points={scan_msg.point_num} "
                        f"IMU={sent_imu} "
                        f"ODOM={len(received)}",
                        flush=True,
                    )

            drain(2)

        # Continue simulated clock so FAST-LIO2 can
        # finish processing the final scan.
        for k in range(60):
            publish_clock(5.0 + (k + 1) * 0.01)
            drain(5)
            time.sleep(0.02)

        output = output_dir / (
            f"a3_1_drone_{args.drone}_odom.npz"
        )

        if output.exists():
            raise FileExistsError(
                f"Refusing to overwrite existing output: {output}"
            )

        odom = np.asarray(
            received, dtype=np.float64
        ).reshape(-1, 8)

        np.savez_compressed(
            output,
            odom_time_pose_xyzw=odom,
            drone_id=np.int64(args.drone),
            imu_samples_sent=np.int64(sent_imu),
            lidar_scans_sent=np.int64(sent_lidar),
            note=np.asarray(
                "Unaligned FAST-LIO2 odometry; "
                "pose-derived diagnostic IMU; "
                "offline replay only"
            ),
        )

        print("\n========== RESULT ==========", flush=True)
        print("IMU sent:", sent_imu, flush=True)
        print("LiDAR sent:", sent_lidar, flush=True)
        print("Odometry received:", len(received), flush=True)
        print("Saved:", output, flush=True)

        if len(received) == 0:
            print(
                "WARNING: No odometry. Inspect FAST-LIO2 logs.",
                flush=True,
            )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
