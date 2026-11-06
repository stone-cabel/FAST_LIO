# A3-B2 Falcon2 成功回放记录（2026-11-06）

用户在 Ubuntu 的真实实验终端输出：

- ROS2 节点 `/laser_mapping_drone_2` 订阅 `/drone_2/livox/lidar` (CustomMsg), `/drone_2/imu` (Imu), `/clock`；发布 `/drone_2/odometry` (Odometry)。
- 回放脚本使用原始 A3-B1 三平面扫描与 300 Hz 合成 IMU，50 scans、1501 IMU；
- 成功收到 **47 帧 Odometry**，保存本地 `~/research/isaac_a1_diagnostics/A3_1/A3_B2_gyro_zero/a3_b2_drone_2_odom.npz`。
- 此前同样的 B2 算法只因发布 `/Odometry` 与订阅 `/drone_2/odometry` 不符而收到0帧，重映射后接收恢复。
- 先前 B2 日志已显示 `Initialized gyro bias: 0 0 0`，且45+次有效 IEKF 诊断，故重映射未改变算法控制变量。
- 本记录**仅确认数据采集成功**；尚未收到 B2 位置/姿态 RMSE，不能声称已改善全局定位精度。
- 为独立 Falcon2 诊断，新增 `evaluation/a3_b1_vs_b2_single_drone.py`：分别按各自首帧对齐，计算全程位姿 RMSE 和公共0.4~0.8s五点近似对比；JSON新路径防覆盖。五点误差采用插值后的误差向量和角度，可能与历史把轨迹先插值再重算的五点RMSE在数值上有小差异，报告需注明。
