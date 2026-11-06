# A4-1 三机公共坐标系对齐：执行及验收说明（2026-11-06）

## 目的

从已完成 A3-B2 的三架独立 FAST-LIO2 里程计获得统一坐标系下的同步位置、姿态，评估三机间相对位移向量误差及相对距离误差，为后续 Sun EKF 数据接口设计做准备。**第一步仅使用仿真真值做公共坐标系初始锚定，不可宣称自主对齐已解决。**

## 运行

```bash
conda activate zeng-isaac
cd ~/research/fastlio-a2-a3-archive-upload
git pull --ff-only origin archive/a2-a3-20261106
python -m py_compile experiments/a2_a3_reproduction_20261106/evaluation/a4_1_oracle_three_uav_frame_alignment.py
python experiments/a2_a3_reproduction_20261106/evaluation/a4_1_oracle_three_uav_frame_alignment.py
```

脚本要求：
- `~/research/isaac_a1_diagnostics/A3_1/A3_B2_gyro_zero/a3_b2_drone_{1,2,3}_odom.npz`
- `~/research/isaac_a1_diagnostics/a1_47_mappo_eskf_500steps_300hz.npz`

输出（首跑才创建，已有非空输出目录时拒绝覆盖）：
- `A4_1/oracle_alignment_B2/a4_1_three_uav_common_frame.npz`
- `A4_1/oracle_alignment_B2/a4_1_summary.json`

建议查看：
```bash
cat ~/research/isaac_a1_diagnostics/A4_1/oracle_alignment_B2/a4_1_summary.json
```

## 计算定义

令 `t_init = max(t_first1,t_first2,t_first3)`，用各机 LIO 位姿插值和同一时刻 PhysX 位姿，求固定 `T^W_Li`；**仅在该初始共同时间使用 GT**。三机此后的轨迹由各自 LIO 经过固定初始变换独立提供，使用共同10Hz时间戳（默认0.4–4.9s），绝不逐帧用 GT 修正。

\[
\\hat p_i^W(t)=R_{WL_i}p_i^{L_i}(t)+t_{WL_i},
\quad e_{ij}(t)=
(\\hat p_j^W-\\hat p_i^W)-(p_{j,GT}^W-p_{i,GT}^W)
\]

评估：
1. 各机统一栅格上位置/姿态 RMSE；
2. 每对(1,2),(1,3),(2,3)相对位置向量 RMSE；
3. 每对相对距离标量 RMSE；
4. 最大相对向量误差与最终误差。

NPZ 输出包括 `time_s`、`position_W_m`、`quaternion_WB_xyzw`，形状分别为 `[T]`、`[T,3,3]`、`[T,3,4]`。有数值微分速度 `velocity_W_finite_diff_mps`，仅供离线接口探索，**不是 FAST-LIO2 内部速度状态，也不应直接作为高可信 EKF 输入**。

本实验不含真实 LiDAR 多机地图配准或 Swarm-LIO2，无外部真值的初始公共系估计将在后续阶段单独解决。三机间距误差较小也不意味着多机坐标系自主对齐已完成。

## 验收

- 严格读取三个 B2 NPZ，拒绝读取空轨迹/时间不递增数据；
- 计算三机共有时间范围，时间同步插值不外推；
- 三对相对误差全部有限且能够生成摘要；
- 与 A3-B2 的厘米级单机 RMSE处于合理数量级（不应强行要求完全相同，因为对齐锚点和采样栅格不同）；
- 保留首帧 oracle 明确标注，不能把此数据直接作为真实机载系统运行证据。

下一步：检查结构化输出及原始 Sun EKF 的输入状态维度/坐标约定，建立清晰的 `LIO -> EKF` interface contract。
