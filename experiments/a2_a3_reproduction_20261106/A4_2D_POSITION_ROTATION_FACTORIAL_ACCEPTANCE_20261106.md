# A4-2d 三机 LIO 位姿来源 2×2 消融正式验收（2026-11-06）

## 验收结论

**PASS（计算与对照执行）**。用户在 Ubuntu `zeng-isaac` 环境实际运行 `evaluation/a4_2d_pos_rot_factorial_eskf.py`，全部4种输入模式均完成，且脚本报告 `A4-2c B0/B1 exact-run parity: PASS`。原始 NPZ/JSON 在用户本机，以下数值来自终端输出，未在 GitHub 后台重新计算。

数据和条件：A4-2b 严格配对的 A3/E7 轨迹；10Hz，共46样本，t=0.4–4.9 s；pose_stride=1；三机相同 synthetic body specific force、同轨迹 collective rotor thrust、同一 Sun-inspired StreamingZengESKF 参数；只替换上游 UAV 位置/姿态。A4-1 LIO 坐标系仍由 PhysX 单时刻 oracle 对齐，且没有真实硬件/完全自主多机帧对齐。

## 四组结果

| Mode | UAV position | UAV attitude | Load pos RMSE m | Load rot RMSE deg | Load vel RMSE m/s | Load body rate RMSE rad/s | Kabsch fit mean m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| B0 | PhysX | PhysX | 0.007693 | 0.658196 | 0.131729 | 0.278787 | 0.002157 |
| P | LIO | PhysX | 0.061855 | 12.362075 | 0.159063 | 1.104941 | 0.022787 |
| R | PhysX | LIO | 0.008932 | 1.060298 | 0.132255 | 0.287712 | 0.003643 |
| B1 | LIO | LIO | 0.066035 | 13.444708 | 0.161092 | 1.221962 | 0.023339 |

相对B0的RMSE增量（源码报告到6位小数）：

| Mode | pos delta m | rot delta deg | vel delta m/s | bodyrate delta rad/s |
| --- | ---: | ---: | ---: | ---: |
| P | +0.054163 | +11.703880 | +0.027335 | +0.826154 |
| R | +0.001239 | +0.402103 | +0.000526 | +0.008925 |
| B1 | +0.058342 | +12.786512 | +0.029363 | +0.943175 |

2×2 RMSE interaction `B1 - P - R + B0`: pos 0.0029403686 m; rot 0.6805297657 deg; vel 0.00150220995 m/s; bodyrate 0.1080955343 rad/s.

## 解释

1. 在本次固定仿真轨迹、10Hz更新、理想 IMU 和 oracle 初始对齐下，**无人机位置误差是 B1 载荷姿态大幅退化的主导输入误差因素**：只替换 UAV 位置即从 0.658° 增到 12.362°，而只替换 UAV 姿态只增到 1.060°。
2. 本结果不能证明主因一定是 Kabsch 条件数或可观测性退化。位置误差还可能包含共享漂移、差分相对几何形变和初始化偏置。下一步应对位置误差做 common-mode vs differential decomposition 和几何条件敏感性检查。
3. 2×2 interaction 对非线性的RMSE指标描述输入的统计非加性，不是严格物理耦合定理。
4. 实验处于 Sun-inspired 独立的 ESKF、Zeng 1.4kg articulated massive-rope simulation；没有完全重现 Sun 作者原始滤波器。
5. ESKF输出存在姿态13.44°和bodyrate1.22rad/s误差，不能宣称可直接安全闭环部署。

本地结果：
`~/research/isaac_a1_diagnostics/A4_2D/pose_rotation_2x2_stride1/a4_2d_2x2_eskf_outputs.npz`
`~/research/isaac_a1_diagnostics/A4_2D/pose_rotation_2x2_stride1/a4_2d_2x2_summary.json`

## 下一步

A4-2e：以 `δp_i(t)=p_{i,LIO}^W-p_{i,GT}^W` 分解成 `δp_common=(1/3)Σδp_i` 与 `δp_i,diff=δp_i-δp_common`，并在 GT UAV attitude、相同 IMU 和 thrust 下分别运行估计器。这样可以测试三机共同平移与三机相对几何误差对载荷姿态的贡献。仍需进一步检查每架坐标系标定、LIO系统误差与时序相关性。
