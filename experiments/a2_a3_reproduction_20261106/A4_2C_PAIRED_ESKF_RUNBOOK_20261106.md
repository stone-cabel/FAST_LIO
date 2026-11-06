# A4-2c 10Hz 同轨迹 ESKF 对照执行说明（2026-11-06）

## 输入验收证据

用户真实运行 `evaluation/a4_2b_verify_and_export_paired_inputs.py`，两份候选 E7 JSON 与 A3 300Hz PhysX 三机位置在451帧内的 RMSE = `6.85921272e-17 m`，MAX=`3.38799977e-16m`；两份 JSON 的三机总推力逐样本在46个A4-1采样点上差异 = `0 N`。已经导出：

`~/research/isaac_a1_diagnostics/A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz`

这提供可信的同轨迹数据配对；但只恢复三机**总推力**，没有12个独立电机推力。原 streaming ESKF 仅使用 `rotor.sum(axis=1)`，所以本次以每机总推力四等分占位适配 `(3,4)`，从来不宣称四个电机真实推力相等。

## A4-2c 执行

```bash
conda activate zeng-isaac
cd ~/research/fastlio-a2-a3-archive-upload
git pull --ff-only origin archive/a2-a3-20261106
python -m py_compile experiments/a2_a3_reproduction_20261106/evaluation/a4_2c_run_paired_10hz_eskf.py
python experiments/a2_a3_reproduction_20261106/evaluation/a4_2c_run_paired_10hz_eskf.py
```

无需运行 Isaac Sim 或 ROS2；脚本从既有 `~/research/payload_ekf_sun_E1/src` 导入**现有** `StreamingZengESKF`，而不修改旧工程。

- 10Hz 46帧 0.4–4.9s；
- B0：PhysX 三机位置姿态；B1：真值辅助初始对齐的 LIO 三机位置姿态；
- 两组相同理想机体系IMU比力、相同总推力、同一时间及滤波参数；
- 默认 `pose_stride=1` 即 10Hz 每次 Kabsch 位姿更新；并不等于之前100Hz、stride100的基线。若需 sensitivity，可以使用 `--pose-stride 10 --out-dir ~/research/isaac_a1_diagnostics/A4_2C/paired_10hz_stride10` 单独运行。
- 传入 `uav_com_acc_w` 零占位，**仅因已经传入显式 `specific_force_B`，原 ESKF 此时完全不使用该 acc 参数**；不代表真实无人机加速度为零。
- 拼装的 `uav_com_state_w_xyzw[:,7:13]` 速度、角速度槽位是接口兼容字段；当前源码的该估计路径不读取。不能误认为板载高精度100Hz估计。
- 两个滤波器完全运行完之后才读取 E7 JSON 的载荷真值，只用于计算位置、姿态、速度和角速度误差；load truth 不参与估计器初始化或更新。
- 结果保存在新的 `A4_2C/paired_10hz_stride1/`，拒绝覆盖既有输出。

### 数据依赖

`a4_2c_run_paired_10hz_eskf.py` 当前使用指定的历史 E7 truth JSON 中的逐步 `load_true_position_w_m, load_true_quaternion_xyzw, load_true_velocity_w_mps, load_true_angular_velocity_w_radps`，且要求与 10Hz 栅格完全匹配。如果旧 JSON 结构不符合，脚本应安全报错，修复解析而不是退化为用载荷真值驱动估计器。

### 结果解释

1. 对照 4类载荷 RMSE 和 B1-B0 增量，但B1-B0是**两个 RMSE 的差**而非每个时刻的误差差。
2. 三机对齐使用了 PhysX 初始位姿，不是自主多机坐标系对齐。
3. ESKF是 Sun-inspired 独立实现，对应 Zeng **1.4kg** 载荷，且使用简化绳索动力学；不要误认为完整Sun论文作者EKF源码复现。
4. 若出现 Kabsch 或 cable inversion 数值异常，先检查相同时刻的推力、IMU方向、绳索模型与姿态/参考点，禁止调B1独有参数使其表现更好。
5. 当前脚本没有完整的 NEES/NIS 统计或多个种子的泛化验证。
