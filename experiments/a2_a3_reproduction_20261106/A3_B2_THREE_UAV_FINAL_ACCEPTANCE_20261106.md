# A3-B2 三无人机完整5秒运动初始化 gyro bias 消融验收（2026-11-06）

## 一、结论

**在固定 A3-B1 三平面仿真 LiDAR 扫描、300 Hz 理想位姿导出 IMU、相同5秒 MAPPO 轨迹、同一 FAST-LIO2 参数/初始化重力方法与离线首帧SE(3)对齐的条件下，三架无人机均因将初始 gyro bias 由运动初期角速度均值替换为零而明显改善姿态和位置。**

这是 `oracle gyro bias=0` 的仿真诊断消融，**不是可在真实非零偏 IMU 上直接部署的动态初始化方案**；结果也不是无动捕真实硬件验证。轨迹采集时真值用于离线评价、而本实验中的零偏零值是根据理想 IMU 已知生成假设设定，未由机载感知独立估计。

## 二、输入、输出及验证

- B2 独立 FAST-LIO2 工作空间 `~/research/fastlio2_b2_ws` 已编译。先前逐行 `diff` 确认 `IMU_Processing.hpp` 唯一更改为 `init_state.bg = V3D::Zero()`（原 `mean_gyr`），初始重力设定不变。
- ROS2 `/Odometry` 输出经节点启动参数 `-r /Odometry:=/drone_i/odometry` 匹配回放脚本；此 remap 不修改估计算法。
- 每机回放 50 scans / 1501 IMU samples，均产生47帧 Odometry，独立保存 `A3_1/A3_B2_gyro_zero/a3_b2_drone_i_odom.npz`。
- 初始 zero bias 已在 Falcon1、Falcon2、Falcon3 的 IEKF 日志中观测到。上传的 Falcon1 log 显示 `IMU sample counter: 31`, `Mean angular velocity: 1.03218 0.894228 0.0492909`, `Initialized gyro bias: 0 0 0`；Falcon3 显示 `counter:30`, `Mean angular velocity: -1.01643 1.50868 0.0534837`, `Initialized gyro bias: 0 0 0`；Falcon2 之前另有完整记录。
- GT `~/research/isaac_a1_diagnostics/a1_47_mappo_eskf_500steps_300hz.npz`。评价 `evaluation/a3_b1_vs_b2_single_drone.py`：每组以自己的首帧里程计对齐GT，真实时间戳内插真值；姿态SO3测地角误差；五点短时指标采用插值后的误差向量和误差角。每个 drone 的 B1/B2 时间序列起止一致，但三架的起止不同。

## 三、完整轨迹五秒验证（用户终端真实输出）

| Drone | B1 position RMSE m | B2 position RMSE m | pos improvement | B1 rotation RMSE deg | B2 rotation RMSE deg | rot improvement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Falcon1 | 5.787539 | **0.122495** | **97.88%** | 33.299092 | **0.651390** | **98.04%** |
| Falcon2 | 31.295448 | **0.050072** | **99.84%** | 102.566639 | **0.440285** | **99.57%** |
| Falcon3 | 1.421575 | **0.068384** | **95.19%** | 10.131991 | **0.346143** | **96.58%** |

| Drone | Valid odom | Time interval s | B1 end position error m | B2 end position error m |
| --- | --- | --- | ---: | ---: |
| Falcon1 | 47 | [0.397490, 4.997790] | 13.939957 | **0.093545** |
| Falcon2 | 47 | [0.393190, 4.998690] | 75.330308 | **0.031939** |
| Falcon3 | 47 | [0.380085, 4.985985] | 1.711349 | **0.045058** |

## 四、短时 0.4—0.8s 五点评价

| Drone | B1 pos RMSE m | B2 pos RMSE m | B1 rot RMSE deg | B2 rot RMSE deg |
| --- | ---: | ---: | ---: | ---: |
| Falcon1 | 0.286364 | **0.026640** | 2.891529 | **0.115263** |
| Falcon2 | 0.917405 | **0.099580** | 8.736099 | **0.322992** |
| Falcon3 | 0.199370 | **0.015079** | 2.765282 | **0.044536** |

历史早期 RMSE 有不同评价插值定义；只能在本脚本同一计算方法的B1/B2间直接对照。

## 五、用户上传的 ROS2 日志的额外证据

Falcon1 B2:
- `p_pre->lidar_type 1`, `Node init finished`, `IMU Initial Done`，初始化零偏确认为 `0 0 0`。
- IEKF 0.3s：undistort=877, downsample=377, matched=316, mean_abs_residual=0.00632548, eig_min=55.1655, eig_mid=104.107, eig_max=156.728.
- IEKF 4.9s：matched=293, residual=0.00881323, eig_min=75.9072.

Falcon3 B2:
- `p_pre->lidar_type 1`, `Node init finished`, `IMU Initial Done`，初始化零偏确认为 `0 0 0`。
- IEKF 0.3s：undistort=744, downsample=420, matched=331, mean_abs_residual=0.00663656, eig_min=94.871, eig_mid=101.021, eig_max=135.108.
- IEKF 4.9s：matched=380, residual=0.00819744, eig_min=74.0385.

注意日志中的 3×3 eig_min/eig_mid/eig_max 是 `Σ n nᵀ` 的方向几何代理，不是完整 IEKF 6-DoF 可观测性证明。

## 六、尾部报错说明

用户重复运行 `for i in 1 3` 时，Falcon1 比较输出并成功保存；Falcon3 已先单独成功评价并保存 `a3_b1_vs_b2_falcon3_comparison.json`。第二次 Falcon3 运行打印完整相同的 RMSE 后遇到：
`FileExistsError: Refusing to overwrite ...a3_b1_vs_b2_falcon3_comparison.json`。
这是保护结果的预期行为，**不是仿真、ROS、Odometry、定位精度或已保存 JSON 的失败**。不应覆盖/删除原文件，仅需用 `cat` 或新 `--out` 路径阅读/重跑。

## 七、验收判断与后续工作

- **A3-B2 三机5秒消融实验完成**，三机B2结果分别为位置RMSE约12.25cm / 5.01cm / 6.84cm，姿态RMSE约0.651° / 0.440° / 0.346°。不可笼统说“全三机均在5cm以内”。
- 对所测输入和真值辅助零偏更改有强的个例因果证据；不能据此证明无偏置情况下以外的泛化。
- 后续：先保存三机完整 B1/B2 结果与初始参数/日志；若继续算法研究，应该考虑未知零偏/有限噪声/动态重力初始化的可观测性与实测数据，避免将 oracle `bg=0` 直接当成动态初始化算法。
