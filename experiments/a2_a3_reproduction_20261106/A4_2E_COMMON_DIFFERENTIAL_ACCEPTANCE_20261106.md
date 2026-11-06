# A4-2e 三机共同漂移 vs 相对构型误差：正式验收（2026-11-06）

## 实际结果（用户 Ubuntu 终端执行，非后台重跑）

```text
A4-2e common-mode vs relative-configuration UAV position error ablation
N=46 time=0.400..4.900 s, pose_stride=1
FILTER_OK B0_GT (payload truth not loaded)
FILTER_OK C_COMMON (payload truth not loaded)
FILTER_OK D_RELATIVE (payload truth not loaded)
FILTER_OK P_ALL_LIO_POSITION (payload truth not loaded)
Independent A4-2d B0/P parity: PASS
Common translation RMS=0.066892m, per-UAV relative residual RMS=0.053343m
B0_GT                load position RMSE=0.007693m, attitude RMSE=0.658196deg, velocity RMSE=0.131729m/s, bodyrate RMSE=0.278787rad/s, Kabsch-fit=0.002157m
C_COMMON             load position RMSE=0.065924m, attitude RMSE=0.658196deg, velocity RMSE=0.158332m/s, bodyrate RMSE=0.278787rad/s, Kabsch-fit=0.002157m
D_RELATIVE           load position RMSE=0.023485m, attitude RMSE=12.362075deg, velocity RMSE=0.137065m/s, bodyrate RMSE=1.104941rad/s, Kabsch-fit=0.022787m
P_ALL_LIO_POSITION   load position RMSE=0.061855m, attitude RMSE=12.362075deg, velocity RMSE=0.159063m/s, bodyrate RMSE=1.104941rad/s, Kabsch-fit=0.022787m
```

**数据位置**
`~/research/isaac_a1_diagnostics/A4_2E/common_differential_stride1/a4_2e_common_differential_eskf_outputs.npz`
`~/research/isaac_a1_diagnostics/A4_2E/common_differential_stride1/a4_2e_common_differential_summary.json`

## 验收判断

- **PASS（代码运行、两项端点复现和类别消融）**。
- 固定无人机姿态GT、理想合成比力、相同真实配对三机总推力、10Hz/pose_stride1条件下：三机共同平移误差对载荷位置误差有明显贡献，但不改变载荷姿态和角速度结果（本轮显示精度内）。
- 相对构型误差会显著提高载荷姿态RMSE（B0的0.658196°→D的12.362075°）以及角速度RMSE（0.278787→1.104941rad/s）；D 与P姿态和角速度显示数值一致。
- 常量全系统平移不影响 Kabsch 的姿态解；本实验共同平移 `δp_common(t)` 随时间变化，预测角速度在固定IMU和推力输入的结构下也保持一致，但不等于证明任何实际滤波器在动态平移下都完全不变。
- 位置误差分解为 `δp_i(t)=δp_common(t)+[δp_i(t)-δp_common(t)]`；共同RMSE=0.066892m，相对每UAV残差RMSE=0.053343m。两者的RMS定义不同，不可简单相加得到单机误差。
- 此轮不能证明几何病态、局部观测性矩阵退化、或优化的构型能降低误差。这些需要 Jacobian/SVD及扰动实验进一步验证。
- 三机LIO轨迹曾在共同初始时刻使用PhysX GT作oracle对齐；B0、C、D均有真值辅助干预。10Hz、46点、单一5s轨迹，不能声明真实MoCap-Free多机部署已通过。
- 核心 LoadESKF 为 Sun-inspired 独立复现，不是原Sun作者逐行源码。

## 下一步 A4-2f

分析三机位置误差到 Kabsch 载荷姿态的局部敏感性，具体应通过 `estimate_payload_pose_kabsch` 的实际实现和 `invert_cable_force` 实际输出建立有限差分Jacobian，而非只计算三机GT点云自身的协方差条件数。检验平移零空间、差分扰动敏感方向、异常高敏感时刻和与载荷姿态误差的相关性，记录单位 m→rad / m→deg。固定方向/张力与姿态、推力，先区分纯几何机制，再单独分析 ESKF的动态传播。
