# A2-B0/B1/B3 与 A3-B0/B1 定位评估源码审计（2026-11-06）

审计依据：本归档分支内的**真实源文件**。未重新执行实验、未读取用户 Ubuntu 中原始 NPZ 数组；历史 RMSE 数字来自此前终端实测记录，原始文件本身可能不存在于 GitHub。

## 1. 已确定的 A2 厘米级指标的计算出处

`A2/a2_4b_3_three_group_comparison.py`：
- 真值文件：`a2_4a_7_init_7p2_to_8p0_300hz.npz`，仅读取 `Falcon1_base_link_inertia`。
- B0/B1/B3 里程计：`a2_4b_0_single_odom.npz`、`a2_4b_1_single_odom.npz`、`a2_4b_3_retry_odom.npz`。
- 对三组里程计与 GT 取共同时间区间 `[t0,t1]`，再 `np.linspace(t0,t1,5)`，**仅有5个插值评估时刻**。
- 对估计轨迹进行首帧姿态对齐 `R_align=R_gt(0)*R_est(0).inv()`，平移使用首帧相对位移；位置误差为对应五点的 `sqrt(mean(norm(delta_est-delta_gt)^2))`。旋转误差同样在首帧对齐后计算。
- 源脚本明确标注：`Short trajectory; not long-term localization accuracy`，且 `Single replay per successful configuration`。
- 历史终端报告 B0 ~0.35968m，B1 ~0.01096m，B3 ~0.0346m；须读取本机原始 NPZ 才能独立重算精确值与实际时间区间。

## 2. A2 数据生成和回放

`A2/a2_4b_1_replay_lidar_imu.py` 和 `A2/a2_4b_3_replay_lidar_imu.py`：
- 用 `range(8)` 构造8帧 0.0~0.7s 的100ms扫描。
- IMU 文件 `a2_4a_8_imu_7p2_to_8p0_300hz.npz`，241条（0.0~0.8s，300Hz），姿态差分所得的**诊断用合成 IMU**。
- A2-4B-3 在 `A2/a2_4b_3_equal_points.py` 中对8帧三平面数据按原始 B0 有效点数量进行分组/时序均衡抽样，保持束时间偏移。
- `A2/a2_4b_1_three_plane_mid360.py` 将两面正交墙面固定于完整轨迹 t=0 的初始位置；依赖 `A1_SCAN_POSE_CACHE`。
- **关键可复现性缺口**：A2-B1 生成器默认路径竟为 `a1_47_mappo_eskf_500steps_300hz.npz`，而评估和 IMU 使用 `a2_4a_7_init_7p2_to_8p0_300hz.npz` 及 `a2_4a_8_imu_7p2_to_8p0_300hz.npz`。如果真实运行时设置 `A1_SCAN_POSE_CACHE`，则可能完全正常；必须查 NPZ 中 `pose_cache_source` 字段，而不是从脚本默认值下结论。

## 3. A3 计算

`A3/a3_1_evaluate_odometry.py`、`A3/a3_b1_evaluate_odometry.py`：
- A3真值：`a1_47_mappo_eskf_500steps_300hz.npz`，三机分别评估。
- 各机实际47个 Odometry 时刻，约0.38~5.00s（各机略有不同）；真值插值在原始 Odometry 时间戳。
- 使用首帧姿态对齐和首帧位置对齐，**并非**对整条曲线最优拟合。
- 历史终端结果 B0/F1,F2,F3：19.5023 / 36.6551 / 4.4190m；B1：5.7875 / 31.2954 / 1.4216m。
- B1 同时改变了静态墙面 t=0 锚点和新增 X 法向墙面两个因素，不能将改善完全归因于 X 墙。

## 4. 建议本机离线核验命令

```bash
cd ~/research/isaac_a1_diagnostics
python - <<'PY'
import numpy as np
from pathlib import Path
root=Path('.')
for variant in ['0','1','3']:
    f=root / f'a2_4b_{variant}_mid360_window_00_2000rays.npz'
    if f.exists():
        with np.load(f,allow_pickle=False) as d:
            print(f, 'pose_cache_source=',d['pose_cache_source'].item() if 'pose_cache_source' in d else '<missing>',
                  'start=',d['scan_start_time_s'].item(), 'control=',d['scan_control_mode'].item() if 'scan_control_mode' in d else '<missing>')
for name in ['a2_4b_0_single_odom.npz','a2_4b_1_single_odom.npz','a2_4b_3_retry_odom.npz']:
    f=root/name
    if f.exists():
        with np.load(f,allow_pickle=False) as d:
            a=d['odom_time_pose_xyzw']
            print(name,len(a),a[0,0],a[-1,0])
PY
```

严格比较还需要统一实验时间区间和采样时刻，核对 LiDAR/IMU/GT 所属轨迹，记录实际生成命令或扫描 NPZ 元数据。
