# A2/A3 FAST-LIO2 原始实验归档（待本机源代码同步）

本分支专门保存 2026-11-06 的 A2/A3 历史实验原始代码，**不会覆盖 main**。

## 已核实成果

- A2-4B-B0 单墙：位置 RMSE 约 0.3597 m（历史短时实验）。
- A2-4B-B1 三平面：约 0.0110 m；B3 等点数对照：约 0.0346 m；历史结果对应的评估时长仍需审计。
- A3-B0（原始 5s）Falcon 1/2/3 位置 RMSE：19.5023 / 36.6551 / 4.4190 m。
- A3-B1（固定全局墙面锚点 + 增加正交 X 墙，5s）位置 RMSE：5.7875 / 31.2954 / 1.4216 m；姿态 RMSE：33.2991 / 102.5666 / 10.1320 deg。
- 已保留本地 FAST-LIO2 IEKF 点云安全修复以及匹配可观测性诊断。请以本机 Git log 和原始文件为准。

## 原始文件位置（源代码在用户 Ubuntu 主机）

- `~/research/isaac_a1_diagnostics/a2_4b_1_three_plane_mid360.py`
- `~/research/isaac_a1_diagnostics/48_a1_mappo_mid360_window.py`
- `~/research/isaac_a1_diagnostics/A3_1/A3_B1_three_plane/a3_b1_three_plane_mid360.py`
- `~/research/isaac_a1_diagnostics/A3_1/a3_1_evaluate_odometry.py`
- `~/research/isaac_a1_diagnostics/A3_1/A3_B1_three_plane/a3_b1_evaluate_odometry.py`
- `~/research/fastlio2_ws/a2_bridge/a2_4b_{0,1,3}_replay_lidar_imu.py`
- `~/research/fastlio2_ws/a2_bridge/a3_{1,b1}_replay_three_uav.py`
- `~/research/fastlio2_ws/src/FAST_LIO_ROS2/src/{laserMapping.cpp,IMU_Processing.hpp}`

请使用 ChatGPT 对话中提供的 `archive_to_github.py` 在原始 Ubuntu 主机执行归档。脚本将原样复制存在的文件，记录 SHA256 和 Git 历史，跳过不存在的候选路径，不编造缺失代码，随后推送到本分支的 `experiments/a2_a3_reproduction_20261106/`。

## 归档说明

只从本机原始源码同步；不得把运行日志里的片段作为完整代码替代。默认排除大型 NPZ、私钥、Token、缓存和原始数据集，若有第三方源码需保留许可证。**这个分支当前仅建立归档入口，不代表 A2/A3 原始源码已经上传。**
