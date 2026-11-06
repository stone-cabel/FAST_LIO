# A4-1 三机初始真值辅助坐标系对齐：运行验收记录（2026-11-06）

## 验收结论

**PASS：Oracle 初始对齐 + 三机公共时间栅格下的相对几何误差审计通过；尚未实现自主无真值帧对齐。**

此报告来自用户在 Ubuntu 终端执行归档的 `evaluation/a4_1_oracle_three_uav_frame_alignment.py` 的真实打印结果，所需原始 NPZ / JSON 保留在其本地。该程序把三个独立 A3-B2 LIO odometry 在公共初始时刻用 PhysX 真值求得三个固定 SE(3) 对齐变换，然后后续只用 LIO 轨迹，真值用于误差评价。不使用 Swarm-LIO2 自主地图匹配，不证明 MoCap-Free 在线多机对齐。

## 实际执行

```text
A4-1 ORACLE INITIAL FRAME ALIGNMENT — NOT AUTONOMOUS
t_init=0.397490s, common=0.400..4.900s, n=46
Falcon 1: pos RMSE=0.123067m; rot RMSE=0.653081deg
Falcon 2: pos RMSE=0.046766m; rot RMSE=0.431087deg
Falcon 3: pos RMSE=0.068027m; rot RMSE=0.532691deg
Pair 1-2: relative vector RMSE=0.112056m; distance RMSE=0.033481m; max vector error=0.206535m
Pair 1-3: relative vector RMSE=0.097711m; distance RMSE=0.016878m; max vector error=0.177243m
Pair 2-3: relative vector RMSE=0.059206m; distance RMSE=0.034499m; max vector error=0.136059m
Saved: /home/erkangshi/research/isaac_a1_diagnostics/A4_1/oracle_alignment_B2/a4_1_three_uav_common_frame.npz
Saved: /home/erkangshi/research/isaac_a1_diagnostics/A4_1/oracle_alignment_B2/a4_1_summary.json
```

## 解读

- **相对位置向量 RMSE** 和 **相对距离 RMSE** 是不同指标。距离误差较小不代表相对方向同样精确；载荷 EKF 的绳索连接几何受到相对向量误差影响。
- Falcon3 A4-1 姿态 RMSE 0.532691° 与 A3-B2 单机 0.346143° 不同，主要可从共同的 t_init/采样窗口与原单机首帧对齐定义不同解释；未经统一方法对比前，不能归因于 LIO 算法变化。
- 输出 `position_W_m[T,3,3]`、`quaternion_WB_xyzw[T,3,4]` 对下一步 EKF 接口有价值；`velocity_W_finite_diff_mps` 是10Hz 位置微分，不能当作原始 IMU/ESKF 状态或高质量100/300Hz速度。
- 当前有效评价时间0.4–4.9s只有46帧共同10Hz栅格。未来 Sun EKF 的实际更新频率必须另行处理，不能把46帧误认为300Hz连续实时状态。

## 下一项 A4-2

首先对照 Sun EKF 实现核查输入的物理定义、坐标系、机体/雷达/IMU/绳索吊点偏置、时间和频率、推力和IMU加速度语义，以及载荷EKF输出的六自由度状态与协方差。建议在不改变 EKF 算法的前提下，新增独立的接口检查和观测噪声实验；逐级将理想机体状态替换为 A4-1 的对齐 LIO 状态，真值只做离线评价。
