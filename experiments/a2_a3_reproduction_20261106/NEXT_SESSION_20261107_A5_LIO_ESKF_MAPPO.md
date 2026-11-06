# 2026-11-07 — 下次继续：LIO → 既有 ESKF–MAPPO

记录于 2026-11-06。用户明确：A5接入明天再做，今晚先看现有仿真效果。**本记录不表示 A5-0 已运行。**

## 已完成（独立验证）
- E5–E6：已有 Sun-inspired StreamingZengESKF → Zeng MAPPO Actor → PhysX 控制闭环；E5.7 两种 seed，500 步，其中 497 步使用估计载荷观测；E6.6 已有8组消融闭环审计。该闭环依然依赖 PhysX UAV 状态/加速度，并存在最初3步 Oracle 启动及特权 critic。
- A3/A4：三机模拟 Mid-360 扫描与独立 FAST-LIO2，A4-1 只完成 PhysX 真值辅助多机坐标系对齐；尚无自主 Swarm-LIO2 公共坐标对齐。
- A4-2c–g：完成 LIO 误差对载荷 ESKF 及 Kabsch 几何敏感性的离线受控分析。
- A4-3a：FAST-LIO2 6×6 位姿协方差出版顺序与索引修复并编译，三机各47帧协方差记录数值检验通过；UAV1重复运行位置与协方差逐元素相同；A4-3a-4 GT辅助诊断发现数量级失配，但 IMU/PhysX参考点、外参和对齐不确定性仍待核实。

## 明日 A5 优先顺序
1. **A5-0**：运行已提交脚本 `evaluation/a5_0_lio_eskf_paired_replay.py`，检查三机 A4-3a LIO 位姿 → 原有 StreamingZengESKF 的 10Hz、因果、同轨迹离线输入桥接。源脚本不改现有估计器、不运行 MAPPO，不代表完成闭环。命令：
   ```bash
   conda activate zeng-isaac
   cd ~/research/fastlio-a2-a3-archive-upload
   git pull --ff-only origin archive/a2-a3-20261106
   python experiments/a2_a3_reproduction_20261106/evaluation/a5_0_lio_eskf_paired_replay.py
   ```
2. **A5-1**：同一个 Isaac Sim/PhysX 运行过程中产生动态 Mid-360 扫描、传送 ROS2、实时 FAST-LIO2、IMU 预测/时间管理与三机共系（先将 GT 初始化作为受控 oracle 版本明确标注）；禁止将先前录制的固定 5s LIO 文件当作新动作产生的实时定位结果。
3. **A5-2**：接入原有 E5/E6 MAPPO Actor（135D×3时间帧，ACCBR 5D/机），同时核查每机 ego 观测 [15:33] 是否仍为 PhysX 真值；将估计状态与真实仿真状态严格隔离，保留原指标并做 seed 对照。不能将特权 critic、3步 Oracle 启动的结果称为完全板载 MoCap-free。
4. 后续：自主多机坐标系对齐、A4-3b/c covariance 模型、LiDAR 3D map/MARTS 避障。

## 今晚只看现有仿真
- 先查找 `~/research/isaac_a1_diagnostics` 下已有 MP4；使用 `ffplay -autoexit -loop 0 <file.mp4>` 或系统视频播放器。
- `a1_mid360_three_uav_demo.mp4` 是早期 100 ms 数据慢速点云可视化，不可称连续5秒或在线闭环。
- A1-49 的离线 Warp raycasting 同步示意视频也不等于 Isaac Sim 实时相机录像；按文件名/帧数核实来源。
