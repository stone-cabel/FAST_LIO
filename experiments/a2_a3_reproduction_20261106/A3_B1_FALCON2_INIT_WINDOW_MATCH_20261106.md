# A3-B1 Falcon 2 初始化窗口匹配审计 — 2026-11-06

本报告来自用户 Ubuntu 终端中真实的 `a3_1_generate_three_uav_imu.py` 源码片段、`a3_1_imu_5s_300hz.npz` 原始数据统计和 `fastlio_drone_2.log`。不宣称已重跑 FAST-LIO2。

## 关键证据

日志：`IMU sample counter: 29`，`First lidar time: 0.1`，`Mean angular velocity: -2.02169 -0.299259 0.0439556`，`Initialized gyro bias` 与均值相同。

在原始 300Hz IMU 中，对28、29、30长度的候选连续数据窗口搜索，最接近上述偏置的窗口：
- `N=28`; `start_idx=30`；
- 原始时间 `[0.100000, 0.190000]` 秒；
- `mean=[-2.021693,-0.299259,0.043956]` rad/s；
- 与日志初始化 gyro bias 欧氏距离 `0.00000274 rad/s`。

`IMU sample counter=29` 与28个数据均值的关系可能来自初始化 `N=1`、首帧样本单独赋值并在遍历后递增，仍需逐条追踪 `meas.imu` 和递推计数确定，不可直接断言已严格恢复真实输入序列。

用户展示的生成器对角速度用 `(R[t-1]^{-1}R[t+1]).as_rotvec()/(2dt)`，保存 `gyro_body_radps` 与 `accel_body_mps2`，标签为 `Ideal pose-derived IMU at Falcon link origin; not an independent sensor measurement`。目前展示的处理和保存片段没有显式偏置/噪声注入，但完整生成器相关赋值部分仍可进一步核对。

归档 `FAST_LIO/IMU_Processing.hpp` 的 `IMU_init` 执行 `init_state.bg=mean_gyr`，后续 `UndistortPcl` 使用 `angvel_avr-imu_state.bg`，所以运动初始化确实存在把真实旋转速度吸收到估计偏置中的机制。

## 对定位误差的因果解释限度

Falcon2 A3-B1 的 0.4987s：位置误差 `(0.01026,-0.02329,-0.52768)m`，姿态误差 `4.3817°`；1.4992s 姿态误差 `45.8659°`，全程位置RMSE `31.295448m`。法向约束矩阵最小特征值在0.3s/0.4s分别 `0.268968`/`0.551565`。这与错误初始化假设相容，但尚不能定量确定单独影响，动态加速度污染重力初始化以及点云法向分布等因素仍需隔离。

## 下一步严谨实验

1. 只读确认生成器 `gyro_all` 和 `acc_all` 的全部赋值，确认没有人为注入 noise/bias。
2. 创建独立实验 A3-B2，仅用仿真已知为零的 gyro bias 进行**诊断性**初始化，不改变重力初始化、场景、采样、时间戳、IEKF参数。
3. 三架无人机分别重放同一份 A3-B1 输入数据，对比早期五点、完整5秒姿态/位置RMSE和匹配法向信息矩阵。实验与原始 B1 代码及结果应单独保存。
4. 使用仿真理想 bias 为零属于真值辅助诊断，不可作为不依赖外部真值的最终 LiDAR-only 自主定位算法结果。
