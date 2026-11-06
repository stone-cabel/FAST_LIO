#!/usr/bin/env python3
"""A2-2C: replay synthetic Falcon-1 Livox CustomMsg + pose-derived IMU.

Diagnostic-only: IMU is differentiated from PhysX pose, not a sensor model.
Publishes /clock for a FAST-LIO2 node launched with use_sim_time:=true.
Starts a fresh timeline for each run; restart FAST-LIO2 between runs.
Also records received FAST-LIO2 /Odometry messages to an NPZ.

Requires system Python with ROS 2 Jazzy, livox_ros_driver2, numpy.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg, CustomPoint


def stamp(seconds: float):
    ns = int(round(float(seconds) * 1_000_000_000))
    return ns // 1_000_000_000, ns % 1_000_000_000


def set_stamp(field, seconds: float):
    field.sec, field.nanosec = stamp(seconds)


def build_scan(root: Path, window: int, min_range: float, static_only: bool) -> tuple[float, CustomMsg, int]:
    file = root / f'a1_48_mid360_window_{window:02d}_2000rays.npz'
    with np.load(file, allow_pickle=False) as z:
        start = float(z['scan_start_time_s'])
        offsets = np.asarray(z['time_offset_ns'], dtype=np.int64)
        directions = np.asarray(z['directions_sensor'], dtype=np.float64)
        ranges = np.asarray(z['ranges_moving_m'][0], dtype=np.float64)
        objids = np.asarray(z['object_id_moving'][0], dtype=np.int32)
        names = [str(x) for x in z['scene_names']]
        abs_times = np.asarray(z['absolute_beam_time_s'], dtype=np.float64)
        assert int(z['scan_window_index']) == window
    if not (offsets.shape == (2000,) and directions.shape == (2000, 3) and ranges.shape == (2000,)):
        raise ValueError(f'Unexpected scan arrays in {file}')
    if not (np.diff(offsets) > 0).all():
        raise ValueError(f'Offsets nonmonotonic in {file}')
    if not np.allclose(abs_times, start + offsets * 1e-9, atol=1e-8, rtol=0):
        raise ValueError(f'Absolute beam timestamps disagree with offsets: {file}')
    valid = np.isfinite(ranges) & (ranges > min_range) & np.isfinite(directions).all(axis=1)
    if static_only:
        # The 19 visual meshes are dynamic, ground and wall are last two.
        # IDs must be verified against scene_names before enabling this diagnostic filter.
        static_ids = [i for i, name in enumerate(names) if name.lower() in ('ground', 'wall')]
        if len(static_ids) != 2:
            raise RuntimeError(f'Static semantic IDs not verifiable from scene_names: {names}')
        valid &= np.isin(objids, static_ids)
    ids = np.flatnonzero(valid)
    if len(ids) < 2:
        raise ValueError(f'Too few valid returns for window {window}: {len(ids)}')
    xyz = ranges[ids, None] * directions[ids]
    msg = CustomMsg()
    set_stamp(msg.header.stamp, start)
    msg.header.frame_id = 'drone_1/livox'
    msg.timebase = int(round(start * 1e9))
    msg.lidar_id = 1
    msg.rsvd = [0, 0, 0]
    result = []
    for index, point_xyz in zip(ids, xyz):
        p = CustomPoint()
        p.offset_time = int(offsets[index])
        p.x, p.y, p.z = (float(v) for v in point_xyz)
        p.reflectivity = 0  # synthetic, not calibrated physical reflectivity
        p.tag = 0           # passes FAST-LIO AVIA CustomMsg check
        p.line = 0          # one synthetic channel for geometry-only test
        result.append(p)
    msg.points = result
    msg.point_num = len(result)
    return start, msg, len(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home() / 'research' / 'isaac_a1_diagnostics')
    parser.add_argument('--rate', type=float, default=0.5, help='sim seconds per wall second; default slow 0.5x')
    parser.add_argument('--min-range', type=float, default=0.5)
    parser.add_argument('--static-only', action='store_true', help='diagnostic GT semantic filtering; requires ground/wall name verification')
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()
    if not (0.05 <= args.rate <= 2.0):
        parser.error('--rate must be between 0.05 and 2')
    root = args.root.expanduser()
    imu_file = root / 'a2_2a_imu_300hz_diagnostic.npz'
    with np.load(imu_file, allow_pickle=False) as z:
        imu_t = np.asarray(z['time_s'], dtype=np.float64)
        acc = np.asarray(z['drone_1_accel_body_mps2'], dtype=np.float64)
        gyr = np.asarray(z['drone_1_gyro_body_radps'], dtype=np.float64)
    if not (imu_t.shape == (1501,) and acc.shape == (1501, 3) and gyr.shape == (1501, 3)):
        raise ValueError('Unexpected IMU diagnostic data shape')
    if not ((np.diff(imu_t) > 0).all() and abs(imu_t[-1] - 5.) < 1e-8):
        raise ValueError('Unexpected IMU diagnostic timebase')
    if not (np.isfinite(acc).all() and np.isfinite(gyr).all()):
        raise ValueError('Non-finite IMU data')

    scans = [build_scan(root, k, args.min_range, args.static_only) for k in range(50)]
    if any(abs(t - 0.1 * k) > 1e-8 for k, (t, _, _) in enumerate(scans)):
        raise ValueError('Scan-window global time mismatch')

    # Sort by sampling time; IMU comes first at equal timestamps.
    events = [(float(t), 0, 'imu', k) for k, t in enumerate(imu_t)]
    events += [(float(t), 1, 'scan', k) for k, (t, _, _) in enumerate(scans)]
    events.sort(key=lambda e: (e[0], e[1]))

    rclpy.init()
    node = Node('a2_2c_unified_replay')
    scan_pub = node.create_publisher(CustomMsg, '/drone_1/livox/lidar', 20)
    imu_pub = node.create_publisher(Imu, '/drone_1/imu', 50)
    clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
    clock_pub = node.create_publisher(Clock, '/clock', clock_qos)
    received = []

    def on_odom(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        received.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    node.create_subscription(Odometry, '/Odometry', on_odom, 20)

    def spin_available(limit=3):
        for _ in range(limit):
            rclpy.spin_once(node, timeout_sec=0.0)

    print(f'A2-2C: IMU={len(imu_t)} scans={len(scans)} total_return_points={sum(s[2] for s in scans)}', flush=True)
    print(f'RATE={args.rate}x; static_only={args.static_only}; /clock simulated 0..5 s', flush=True)
    print('CAUTION: diagnostic pose-derived IMU; no accuracy claim; restart FAST-LIO2 per run', flush=True)
    time.sleep(1.0)  # DDS discovery and /clock subscription matching

    initial_clock = Clock()
    set_stamp(initial_clock.clock, 0.0)
    for _ in range(5):
        clock_pub.publish(initial_clock)
        time.sleep(.05)
    begin = time.monotonic()
    try:
        last_event_time = -1.0
        published_imu = 0
        published_scans = 0
        for t, _, kind, index in events:
            target = begin + t / args.rate
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            c = Clock()
            set_stamp(c.clock, t)
            clock_pub.publish(c)
            if kind == 'imu':
                msg = Imu()
                set_stamp(msg.header.stamp, t)
                msg.header.frame_id = 'drone_1/imu'
                msg.orientation_covariance[0] = -1.0  # orientation unavailable to estimator
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in gyr[index])
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in acc[index])
                imu_pub.publish(msg)
                published_imu += 1
            else:
                msg = scans[index][1]
                scan_pub.publish(msg)
                published_scans += 1
                if index % 10 == 0 or index == 49:
                    print(f'  t={t:.3f} scan={index:02d} points={msg.point_num} '
                          f'IMU={published_imu} ODOM_received={len(received)}', flush=True)
            last_event_time = t
            spin_available(2)
        # Advance /clock past final IMU so FAST-LIO2 100 Hz clock timer can run.
        for k in range(20):
            c = Clock()
            set_stamp(c.clock, 5.0 + (k + 1) * .01)
            clock_pub.publish(c)
            spin_available(5)
            time.sleep(.04)
        print(f'A2-2C SENT: scans={published_scans} IMU={published_imu} '
              f'ODOM_received={len(received)}', flush=True)
        out = (args.out or (root / 'a2_2c_odom_diagnostic.npz')).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        od = np.asarray(received, dtype=np.float64).reshape(-1, 8)
        np.savez_compressed(out, odom_time_pose_xyzw=od,
                            note='Unaligned FAST-LIO2 odometry; diagnostic IMU; not accuracy certified')
        print(f'ODOM_FILE: {out} shape={od.shape}', flush=True)
        if not len(received):
            print('NOTE: zero odometry; inspect FAST-LIO2 initialization and map logs.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()