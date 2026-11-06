# A4-2b 输入数据核查：A3 5秒轨迹旋翼推力缺口（2026-11-06）

本报告根据用户 Ubuntu 终端实际展示的 NPZ keys/shapes 和项目 grep 输出整理；尚未直接读取其本机数据。

## 已确认

1. `a1_47_mappo_eskf_500steps_300hz.npz`: 1501×19×7 的 `body_link_pose_w_xyzw_300hz`，含 `time_s` 等；未列出推力或电机输入。
2. `A3_1/a3_1_imu_5s_300hz.npz`: 1501×3×3 的 `accel_body_mps2` 和 `gyro_body_radps`；由仿真姿态数值导出的理想 IMU；未列出 `rotor_thrust_N`。
3. `A4_1/oracle_alignment_B2/a4_1_three_uav_common_frame.npz`: 共46点、10Hz 的三机对齐位置姿态及位置微分速度；没有 rotor thrust；对齐用了 PhysX 在共同初始时刻的真值，因此为 oracle 初始化实验。
4. 在 Sun ESKF 项目里，`scripts/12_collect_zeng_isaac.py` 和 `scripts/17_collect_zeng_mappo_dynamic.py` 确有采集 `core._forces[...,2]` 并保存 `rotor_thrust_N` 的实现；**但尚无证据证明这些推力与本次 A3 五秒轨迹来自同一次运行**。
5. `scripts/47_a1_eskf_mappo_capture_300hz.py` 中出现 `core._forces[...,2]` 及 `body_com_state_w`，应首先核查该脚本是否已保存同步推力至其他文件。未核查前不应宣布这段数据完全不可恢复。
6. `StreamingZengESKF.step` 对 `uav_com_state_w_xyzw (3,13)`、`uav_com_acc_w (3,6)`、`rotor_thrust_N (3,4)`、可选 `specific_force_B (3,3)` 有严格输入结构；其绳索力反演直接使用 `rotor.sum(axis=1)`。不能用随意选取的悬停推力或另一条轨迹推力替代真实推力，除非作为明示的非配对诊断。
7. 设计 A4-2 10Hz 初步离线对照时 B0 和 B1 必须使用同一时间戳、同一推力/IMU/滤波器参数、相同 pose_stride；仅切换 UAV pose 输入。注意 ESKF中的 UAV COM 与 A3 `base_link_inertia` 参考点要进一步核对。

## 下一步核查（只读）

```bash
cd ~/research/payload_ekf_sun_E1
sed -n '220,275p' scripts/47_a1_eskf_mappo_capture_300hz.py
sed -n '580,660p' scripts/47_a1_eskf_mappo_capture_300hz.py
sed -n '740,815p' scripts/47_a1_eskf_mappo_capture_300hz.py
rg -n 'savez|\.npz|_forces|rotor|action|capture|out|metadata' scripts/47_a1_eskf_mappo_capture_300hz.py
find ~/research/isaac_a1_diagnostics -type f \( -iname '*force*.npz' -o -iname '*rotor*.npz' -o -iname '*motor*.npz' -o -iname '*action*.npz' -o -iname '*uav_sensor_inputs.npz' \) | sort
```

**状态：A4-2b 动力学输入审计进行中；A4-2c LIO→ESKF 替换尚未启动。**
