#!/usr/bin/env python3

from pathlib import Path
import time

import numpy as np
import rclpy

from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg, CustomPoint


ROOT = Path.home() / "research/isaac_a1_diagnostics"

TOPIC = "/drone_1/livox/lidar"
FRAME = "drone_1/livox"
DRONE_INDEX = 0

MIN_RANGE = 0.5


def load_scan(window):
    path = ROOT / (
        f"a1_48_mid360_window_{window:02d}_2000rays.npz"
    )

    with np.load(path, allow_pickle=False) as data:
        start = float(data["scan_start_time_s"])

        ranges = np.array(
            data["ranges_moving_m"][DRONE_INDEX],
            dtype=np.float64,
        )

        directions = np.array(
            data["directions_sensor"],
            dtype=np.float64,
        )

        offsets = np.array(
            data["time_offset_ns"],
            dtype=np.int64,
        )

    if ranges.shape != (2000,):
        raise ValueError(f"Unexpected ranges shape: {ranges.shape}")

    if directions.shape != (2000, 3):
        raise ValueError("Unexpected directions shape")

    if offsets.shape != (2000,):
        raise ValueError("Unexpected offsets shape")

    if np.any(np.diff(offsets) <= 0):
        raise ValueError("Non-monotonic ray timestamps")

    valid = (
        np.isfinite(ranges)
        & (ranges > MIN_RANGE)
        & np.isfinite(directions).all(axis=1)
    )

    indices = np.flatnonzero(valid)

    xyz = ranges[indices, None] * directions[indices]

    msg = CustomMsg()

    # Scan start time, in nanoseconds.
    base_ns = int(round(start * 1_000_000_000))

    msg.header.stamp.sec = base_ns // 1_000_000_000
    msg.header.stamp.nanosec = base_ns % 1_000_000_000
    msg.header.frame_id = FRAME

    msg.timebase = base_ns

    msg.lidar_id = 1
    msg.rsvd = [0, 0, 0]

    points = []

    for idx, p in zip(indices, xyz):
        point = CustomPoint()

        point.offset_time = int(offsets[idx])

        point.x = float(p[0])
        point.y = float(p[1])
        point.z = float(p[2])

        # Synthetic metadata for geometry-only testing.
        point.reflectivity = 0
        point.tag = 0
        point.line = 0

        points.append(point)

    msg.points = points
    msg.point_num = len(points)

    return msg


def main():
    rclpy.init()

    node = Node("a2_mid360_npz_publisher")

    pub = node.create_publisher(CustomMsg, TOPIC, 20)

    print("A2-1B Mid-360 publisher")
    print("Topic:", TOPIC)
    print("Windows: 50")
    print("Drone: Falcon 1")
    print("Metadata: synthetic tag/line/reflectivity")
    print("No IMU; no odometry validation")
    print(flush=True)

    # Allow ROS discovery.
    time.sleep(1.0)

    total = 0

    try:
        next_time = time.monotonic()

        for window in range(50):
            msg = load_scan(window)

            pub.publish(msg)

            total += msg.point_num

            if window % 10 == 0 or window == 49:
                print(
                    f"window={window:02d} "
                    f"stamp={msg.header.stamp.sec}."
                    f"{msg.header.stamp.nanosec:09d} "
                    f"points={msg.point_num}",
                    flush=True,
                )

            next_time += 0.1

            delay = next_time - time.monotonic()

            if delay > 0:
                time.sleep(delay)

        print(
            f"A2-1B PUBLISHED: 50 scans, {total} points",
            flush=True,
        )

        # Allow delivery to complete.
        time.sleep(1.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
