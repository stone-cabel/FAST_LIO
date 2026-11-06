# A4-2g 相对定位误差方向投影：正式验收（2026-11-06）

## 验收

**PASS（原有 46 帧数据的只读子空间投影和数值一致性验证）**。本结果来自用户在本机运行 `evaluation/a4_2g_error_direction_projection.py` 的真实终端输出。没有新的 ESKF 运行，没有上传本机 NPZ。

```text
n=46, t=0.400..4.900s
relative delta 9D norm RMS=0.092392m
sensitive row-space delta norm RMS=0.079250m
first-order null-space delta norm RMS=0.047496m
row-space energy fraction=0.735738
effective actual-direction gain RMS=2.500558rad/m; mean=2.488030rad/m
max singular value mean=3.191385rad/m (worst direction local bound)
predicted attitude change RMS=13.156860deg
worst-case per-frame bound RMS=16.903532deg
observed exact Kabsch change RMS=13.743749deg
row projection identity max |J delta - J delta_row|=2.220e-16rad
```

本地数据：
`~/research/isaac_a1_diagnostics/A4_2G/directional_position_sensitivity/a4_2g_projection_arrays.npz`
`~/research/isaac_a1_diagnostics/A4_2G/directional_position_sensitivity/a4_2g_projection_summary.json`

## 解释与边界

设 `J_geo in R^{3x9}`，对每时刻三个UAV差分位置误差向量 `delta_p in R9` 做 SVD，分解为行空间和零空间投影。根据 `J delta_p = J delta_p_row`，零空间分量一阶不影响 Kabsch 姿态。在46帧时间相关样本中，行空间误差平方能量占比为73.5738%，这**不是载荷姿态误差解释方差或百分比**。实际误差方向有效增益均值2.488、RMS2.501 rad/m，低于所报告平均最大奇异值3.191rad/m；这不表明构型接近奇异。预测姿态变化13.15686°，实际完整Kabsch变化13.743749°，一阶预测对该轨迹相当接近，仍有有限扰动残差。

`16.903532°` 是逐样本 `sigma_max(J)*||delta_p||` 的一阶上界RMS，**不是非线性 Kabsch 姿态误差的严格全局上界**。

A4-1对齐使用 PhysX 初始真值，三机相对位置误差与雅可比都来自受控仿真；输入IMU为位姿生成的理想合成IMU。研究结果是**几何敏感性诊断**，不是自主多机帧对齐、正式传感器协方差标定、完整ESKF局部可观测性或新算法性能提升。不能仅从 `73.6%` 推出存在某种可部署的按敏感方向纠偏技术。

## 建议下一阶段

不要无限追加误差拆分；开始一个独立、有可证伪标准的受控改善实验。建议先检查真实可获得的三机相对约束/配准来源（多机点云重叠、已知吊载几何、通信可用性）以及 LIO 差分误差的统计结构；仅在提供传感器可用相对约束时构建修正器。用仅作实验上限的 oracle 子空间纠偏与无真值约束分开报告，不得把 oracle 干预报告为 MoCap-free 算法。下一阶段重点：多种运行条件、不同噪声及轨迹复现，评估 ESKF 位置姿态和后续 MAPPO 闭环真正是否改善。
