# A3-B2 Falcon2 首轮回放：无 Odometry 的 ROS2 话题问题（2026-11-06）

用户原始运行：B2 已成功编译，运行日志 `IMU Initial Done`、`Initialized gyro bias: 0 0 0`、完整 `scan_begin=0.3~4.9` 的 IEKF 诊断记录。回放发送 1501 IMU 和50帧 LiDAR，但 `Odometry received: 0`，保存了**空结果文件** `A3_1/A3_B2_gyro_zero/a3_b2_drone_2_odom.npz`。不可拿该文件计算 RMSE。

根据归档 `FAST_LIO/laserMapping.cpp` 源码，里程计实际发布在绝对话题 `/Odometry`；回放 `a3_b2_replay_three_uav.py` 订阅 `/drone_2/odometry`。原启动命令缺少 `-r /Odometry:=/drone_2/odometry`，致使 subscriber 为零。修复应该是重新启动 B2 mapping 节点，增加 ROS2 remapping，而不是修改算法或回放源码。重跑前保留空NPZ，另存带 `_zero_received` 后缀，以免 B2 回放原有拒绝覆盖防护触发。

已测 Falcon2 对照：
| 指标 | A3-B1 | A3-B2 |
| --- | ---: | ---: |
| gyro bias 初始值 rad/s | (-2.02169,-0.299259,0.0439556) | (0,0,0) |
| scan 0.3 mean_abs_residual m | 0.0341874 | 0.0056351 |
| scan 0.3 eig_min | 0.268968 | 1.41895 |
| scan 0.6 mean_abs_residual m | 0.0392574 | 0.00654544 |
| scan 0.6 eig_min | 0.819653 | 15.384 |

这些是点云匹配变化；**尚不能**代替 GT 位置或姿态RMSE。需要匹配话题重新回放并导出约47帧 odom。

建议重新启动 mapping 时在原启动参数后追加：
`-r /Odometry:=/drone_2/odometry`，对新进程用 `ros2 node info /laser_mapping_drone_2` 检查发布话题；使用 B2 独立日志文件名保留首次记录。
