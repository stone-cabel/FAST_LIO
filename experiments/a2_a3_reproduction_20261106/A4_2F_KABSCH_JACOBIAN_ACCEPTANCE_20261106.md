# A4-2f Kabsch 载荷姿态几何雅可比：正式验收（2026-11-06）

## 状态与数据边界

**PASS：源码一致性、共同平移零空间、有限差分稳定性、局部雅可比线性预测均在既有46帧轨迹上通过。** 用户在本机实际执行 `evaluation/a4_2f_kabsch_geometry_sensitivity.py`，输出如下。GitHub 未重新运行 Isaac Sim/ESKF，也没有上传本机生成的 NPZ。

- 46帧，t=0.4–4.9s；有限差分步长 `epsilon=1e-5m`。
- 与 A4-2d 的 Kabsch 实现校验：最大拟合残差差异 `1.94289029e-16m`。
- 共享平移零空间 `max ||J C||_2 = 3.65353997e-10 rad/m`。
- `J(eps)` 与 `J(eps/2)` 最大绝对差 `3.29005712e-10 rad/m`。
- 三机相对位置扰动（9维范数）RMS `0.092392m`。
- `sigma_max(J)` 中位数 `3.191449 rad/m`，最大 `3.227316 rad/m`。
- 实际 Kabsch 姿态旋转向量变化 RMS `13.743749deg`。
- 一阶 `J delta_p_relative` 姿态变化 RMS `13.156860deg`。
- 一阶预测旋转向量残差 RMS `1.429133deg`。
- 实际与预测姿态旋转量幅值相关系数 `0.9841675754743676`。

结果路径：
`~/research/isaac_a1_diagnostics/A4_2F/kabsch_local_jacobian/a4_2f_kabsch_sensitivity.npz`
`~/research/isaac_a1_diagnostics/A4_2F/kabsch_local_jacobian/a4_2f_kabsch_summary.json`

## 推导与可支持的主张

观测模型：
`a_i^W = p_{UAV,i}^W + R_{B_i}^W r_{c,i}^{B_i} + l_i s_i^W`；
Kabsch 对 `rho_i^L` 和 `a_i^W` 的去均值点集拟合 SO(3)。

仅扰动无人机位置，其他量冻结时：
`delta a_i^W = delta p_i^W`；
`delta theta_L ≈ J_geo delta p_{UAV}`, `J_geo in R^{3x9}`, 单位 rad/m。
公共平移基向量 `C=[I3;I3;I3]` 应满足 `JC=0`。
实验确认零空间数值误差接近机器精度；实验中已有46帧的实际差分定位误差能由局部一阶雅可比较好预测。

**不可主张**：

- 这不是对完整动态 ESKF/多机传感器的局部可观测性证明。
- `sigma_max≈3.2rad/m` 不是“系统接近奇异”的证据；此段轨迹其波动较小。
- `13.743749°` 是相对扰动造成的 Kabsch 解变化，不等于 A4-2d `12.362075°` 的 ESKF 相对于 load ground-truth 的姿态 RMSE；两者基准和计算链路不同。
- 高相关性并不证明某个控制或主动构型策略能够降低该灵敏度；这需要改变几何构型的试验。
- 46帧、10Hz、仿真理想IMU、单一 seed42、PhysX初始帧真值辅助对齐、独立Sun-inspired滤波器；不能表述成真实 MoCap-Free 性能。

## 下一步关键科学问题

考虑把 A4-2g 设计为“误差方向投影与位置误差协方差传播”：

1. 逐时刻将相对位置扰动投影到 `J_geo` 的行空间及零空间（SVD）；分辨哪些差分误差方向真正影响载荷姿态。
2. 对比 `||J delta_p||` 与 `sigma_max ||delta_p||`：最坏情况放大上界和当前实际误差方向的作用不同。
3. 估计基于有限样本的 `Sigma_p` 并用 `Sigma_theta≈J Sigma_p J^T` 做**描述性**线性不确定性传播；不应把46帧时间相关样本视为充分独立。
4. 检查目标科学问题是否更应聚焦多机相对定位误差的各向异性/误差方向，而非“主动构型必然改善 Kabsch 数值条件”。由于载荷侧固定吊点几何可能决定大部分灵敏度，必须严格证实构型可调性。
