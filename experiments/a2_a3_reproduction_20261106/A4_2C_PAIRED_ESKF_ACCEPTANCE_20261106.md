# A4-2c 10Hz LIO→Sun-style ESKF 对照：实际运行结果（2026-11-06）

## 实验结论与输入

用户在 Ubuntu 上成功执行 `evaluation/a4_2c_run_paired_10hz_eskf.py`。输入已先通过 A4-2b 配对审计：两份 E7 JSON 451个控制时刻对应 A3 PhysX 三机位置 RMSE 6.85921272e-17m、最大误差3.38799977e-16m，A4-1栅格三机总推力差异0 N。使用46个采样点，`t=0.4..4.9s`，10Hz，`pose_stride=1`（每0.1s更新）；B0使用 PhysX 三机位置与姿态，B1仅替换三机位置与姿态为 A4-1 LIO。两组 IMU、总推力、ESKF 参数保持一致。

## 真实运行结果

| 指标 | B0 PhysX UAV pose | B1 LIO UAV pose | B1-B0 RMSE增量 |
| --- | ---: | ---: | ---: |
| 载荷位置 RMSE m | 0.007693 | 0.066035 | +0.058342 |
| 载荷姿态 RMSE deg | 0.658196 | 13.444708 | +12.786512 |
| 载荷速度 RMSE m/s | 0.131729 | 0.161092 | +0.029363 |
| 载荷角速度 RMSE rad/s | 0.278787 | 1.221962 | +0.943175 |
| Kabsch fit mean m | 0.002157 | 0.023339 | +0.021182 |

脚本输出：
```text
A4-2c 10Hz paired offline ESKF: 46 frames, t=0.400..4.900s
pose_stride=1, update_period=0.100s
B0 filter complete (payload truth not loaded).
B1 filter complete (payload truth not loaded).
B0 PhysX UAV pose: load pos RMSE=0.007693m, rot RMSE=0.658196deg, vel RMSE=0.131729m/s, body rate RMSE=0.278787rad/s, Kabsch fit mean=0.002157m
B1 LIO UAV pose: load pos RMSE=0.066035m, rot RMSE=13.444708deg, vel RMSE=0.161092m/s, body rate RMSE=1.221962rad/s, Kabsch fit mean=0.023339m
```

本地归档：
- `~/research/isaac_a1_diagnostics/A4_2C/paired_10hz_stride1/a4_2c_b0_b1_load_eskf_outputs.npz`
- `~/research/isaac_a1_diagnostics/A4_2C/paired_10hz_stride1/a4_2c_b0_b1_load_eskf_summary.json`

本报告仅依据用户终端真实输出，**未在 GitHub 端重新执行实验或上传本地 NPZ**。

## 技术解读与限制

- **PASS（数据接口/对照执行）**：两套已存在的 `StreamingZengESKF` 均成功运行并可输出载荷6DoF、速度/角速度误差。**B1 姿态鲁棒性未达良好性能**：13.44°姿态RMSE和1.22 rad/s角速度RMSE不宜用于直接闭环部署。
- 三机 A4-1 LIO 姿态RMSE 0.43–0.65°，但B1载荷姿态13.44°，不一定意味着姿态单独造成放大；三机相对平移误差及Kabsch条件数同样可能造成大姿态误差，必须做位置/姿态2×2消融后才能讨论主因。
- B1-B0 是 RMSE 差，不是逐帧误差的平均变化。
- 10Hz `pose_stride=1` 方案A不能等同原100Hz `pose_stride=100` 设置。
- 两组使用理想位姿派生IMU，`uav_com_acc_w` 在显式IMU路径中是未使用零占位，旋翼 `(3,4)` 为总推力四等分兼容适配而非真实四旋翼分配，A4-1坐标变换为PhysX真值初始化。
- 现有ESKF是 Sun-inspired 独立复现而非作者原代码；模拟Zeng 1.4kg载荷；无真实 MoCap-Free 在线运行。

## 下一步：A4-2d 2×2 消融

在同一时序、IMU、推力、滤波参数下运行四种模式：
- B0：GT position + GT attitude
- P：LIO position + GT attitude
- R：GT position + LIO attitude
- B1：LIO position + LIO attitude

用于判断载荷姿态退化对 UAV 位置与姿态的敏感性，不预设结果。新脚本 `evaluation/a4_2d_pos_rot_factorial_eskf.py` 验证本轮重现B0/B1输出与已保存 A4-2c 一致，拒绝覆盖历史数据。
