# A3-B2 Falcon 2：陀螺仪初始化单因素消融定量结果（2026-11-06）

## 证据来源与运行状态

用户在 Ubuntu `zeng-isaac` 环境实际运行归档脚本 `evaluation/a3_b1_vs_b2_single_drone.py --drone 2`，输出如下表。原始 NPZ 仍在用户本机，**本文件归档的是原始终端输出指标而非 GitHub 云端重算**。

- B1: `A3_1/A3_B1_three_plane/a3_b1_drone_2_odom.npz`
- B2: `A3_1/A3_B2_gyro_zero/a3_b2_drone_2_odom.npz`
- GT: `a1_47_mappo_eskf_500steps_300hz.npz`
- 两组均47帧，时间区间 `[0.393190,4.998690]s`。
- B2 已证实通过独立 `fastlio2_b2_ws` 编译，并将 `IMU_Processing.hpp` 的单行初始陀螺仪零偏从 `init_state.bg = mean_gyr` 改为 `init_state.bg = V3D::Zero()`；同一份 B1 LiDAR 三平面扫描、300Hz 理想 IMU、重力初始化与其他参数保持不变。回放节点使用 `-r /Odometry:=/drone_2/odometry` 订阅重映射以修复初轮无 Odometry 问题，非算法变化。

## 结果：完整接近5秒的轨迹

| Metric | A3-B1 | A3-B2 | Improvement |
| --- | ---: | ---: | ---: |
| Position RMSE (m) | 31.295448 | **0.050072** | **99.84%** |
| Rotation RMSE (deg) | 102.566639 | **0.440285** | **99.57%** |
| Final position error (m) | 75.330308 | **0.031939** | ~99.96% |
| Valid odometry samples | 47 | 47 | equal |
| Time (s) | 0.393190–4.998690 | 0.393190–4.998690 | equal |

### 0.4–0.8s short-window five-point metrics

| Metric | A3-B1 | A3-B2 |
| --- | ---: | ---: |
| Position RMSE (m) | 0.917405 | **0.099580** |
| Rotation RMSE (deg) | 8.736099 | **0.322992** |
| Position error norm at t=[0.4,0.5,0.6,0.7,0.8] s | [0.03410,0.53427,0.98872,1.22065,1.20582] | [0.00876,0.13565,0.12934,0.09730,0.07004] |

**方法细节**：分别以各自第一帧 Odometry 对齐 PhysX 位置与旋转；完整 RMSE 用所有各自实际 Odometry 时间；五点是对*对齐后的误差向量*及姿态误差角度做线性插值，再算 RMSE。因此五点 B1 0.917405m 与此前不同算法的0.888798m并非严格同公式；本表内 B1/B2 方法一致，可以直接比较。

## 关联的 IEKF 过程日志

用户首次 B2 回放时，由于 ROS 2 里程计发布 `/Odometry`、回放监听 `/drone_2/odometry`，无法收集 Odometry。但 FILTER 完整运行并显示 `Initialized gyro bias: 0 0 0`，和 B1 的 `(-2.02169,-0.299259,0.0439556)` rad/s 对比。早期 ICP 过程扫描0.3s B1/B2 residual=0.0341874/0.0056351m，最小特征值0.268968/1.41895；0.6s B1/B2 residual=0.0392574/0.00654544m，最小特征值0.819653/15.384。过程日志仅为辅助，最重要的是本轮真值评价指标。

## 结论与限制

- **在 Falcon2 当前 A3-B1 输入及理想 pose-derived 无显式 bias IMU 的条件下**，初始零偏处理是导致大幅姿态、位置误差的主要贡献因素，B2 单因素更改使位置 RMSE 从31.3m降到约5.0cm。
- 这是一项**仿真 oracle 消融**：把 `bg=0` 写死只有在本次理想 IMU 数据下有诊断意义，不是可用于真实硬件的 MoCap-Free bias 估计方法。
- 此次尚未证明 Falcon1/Falcon3 同样改善、未知环境/现实硬件鲁棒性，未独立量化初始重力估计误差或完整状态可观测性。
- 此次仅一次回放；尽管输入数据与 47 帧时间戳相同，必要时可重复运行检查确定性。B2 末帧结果已在本地生成结构化 JSON `A3_1/A3_B2_gyro_zero/a3_b1_vs_b2_falcon2_comparison.json`，目前没有上传该本地文件。

## 下一项验收

用同一 B2 已编译二进制和 B1 原始配置分别回放 Falcon1、Falcon3，保证逐机唯一话题重映射 `-r /Odometry:=/drone_i/odometry`，分别运行 `evaluation/a3_b1_vs_b2_single_drone.py --drone i`。最终汇总三机5秒 RMSE，之后再规划不依赖 oracle 的动态 IMU 初始化研究。
