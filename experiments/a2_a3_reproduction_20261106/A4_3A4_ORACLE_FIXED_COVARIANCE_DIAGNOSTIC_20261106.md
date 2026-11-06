# A4-3a-4：新重放 LIO 协方差与 PhysX 真值辅助诊断（2026-11-06）

## 实际运行与输出

用户首次运行脚本因既有输出目录非空而被拒绝覆盖；经检查已存在完整的 `a4_3a4_oracle_fixed_covariance_arrays.npz` 与 `a4_3a4_oracle_fixed_covariance_summary.json`。其后用户使用带纳秒时间戳的新目录独立重跑，计算成功，得到以下相同报告。**脚本本身正常完成；最初的 FileExistsError 是预期覆盖保护。**

使用：
- `A4_3A/fastlio_covariance_replay/drone_{1,2,3}_odom_cov.npz`（同次重放位姿/协方差，47帧每机）
- `a1_47_mappo_eskf_500steps_300hz.npz` PhysX 真值（300Hz），刚体 `Falcon1_base_link_inertia` 等
- 各机使用第一帧 GT 和 LIO 位姿确定一个固定的 world-frame SE(3) 对齐；按 LIO 时间戳插值 PhysX 真值；用固定旋转变换 3x3 位置协方差。

| 指标 | UAV1 | UAV2 | UAV3 |
|---|---:|---:|---:|
| N | 47 | 47 | 47 |
| t 首帧 [s] | .397389999 | .393190002 | .380084999 |
| t 末帧 [s] | 4.997690002 | 4.998690002 | 4.985934997 |
| 固定 GT anchor 条件位置 RMSE [m] | .2519111983 | .0547547155 | .1277219327 |
| 姿态 RMS [deg] | 5.396584565 | 4.002131677 | 6.385955690 |
| 平均条件位置 Mahalanobis² | 638.7657509 | 107.5877743 | 474.8785588 |
| 中位数条件 Mahalanobis² | 657.5903289 | 82.7143244 | 282.4500045 |
| 最大条件 Mahalanobis² | 1437.573107 | 314.1233869 | 1501.594504 |
| 相邻位置增量残差 RMSE [m] | .0217748333 | .0260021572 | .0184553763 |
| 位置协方差最小特征值 [m²] | 1.4479055e-5 | 1.4918666e-5 | 1.5603920e-5 |

独立重跑输出：
`~/research/isaac_a1_diagnostics/A4_3A/oracle_fixed_covariance_diagnostic_retry_20261106_201154_313787778`

## 严格的结论边界

- **已发现强烈数量级失配**：固定GT辅助对齐后位置残差厘米/分米级，相比毫米级单机内部标准差甚大。
- 这是条件诊断而非严格 NEES：参考点是否一致未验证（LIO内部 IMU origin 与 PhysX base_link_inertia），初始对齐的随机性与误差未纳入协方差；单条5秒/47帧时间相关轨迹，不能使用每帧独立卡方校准结论。
- 当前报告只传播 3x3 位置块旋转；IKFoM SO(3) 误差切空间/交叉协方差、IMU/机体杆臂、LIO与多机 frame alignment 的相关性还未处理。
- `d²` 典型理想三维零均值高斯模型单帧期望是3；实际均值分别约638.8, 107.6, 474.9，但这些不构成显著性检验或“协方差必然失效”的证明。
- GT anchor 是用于分析的 oracle；不是自主多机坐标系对齐，也没有采用真值信息作为 LIO 输入。
- 旧 A3-B2 LIO 和新 A4-3a LIO 轨迹存在不同运行结果；本分析始终使用配对的新 LIO + 新协方差，而不是旧 A4-1 误差。
- 独立 UAV1 相同代码两次重放位姿和协方差严格逐元素一致（参见 A4_3A_UAV1_EXACT_REPLAY_REPEATABILITY_20261106.md）。

## 后续最优先核查

1. 核实 `mid360.yaml` 中 `extrinsic_T`、`extrinsic_R`、`extrinsic_est_en` 及传感器框架；确定 PhysX base_link_inertia 到模拟 IMU 的安装杆臂（不应从A3-1虚拟IMU“存在IMU”直接推断实际杆臂为零）。
2. 检查 `laserMapping.cpp` 的 `set_posestamp`、`state_point.pos`、`state_point.rot`、`geoQuat` 的参考系，并明确 EKF 内部小旋转扰动坐标系。
3. 若引用协方差推断 LIO 误差，请先做参考点匹配、协方差坐标变换及对齐不确定性处理。
4. 采用无GT leakage 的鲁棒载荷 ESKF 算法前，还应建模三机位置误差协方差及交叉相关性。避免以固定oracle对齐后马氏距离冒充严格 NEES。
