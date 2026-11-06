# A2/A3 实验开发与 GitHub 归档规范

本文件用于确保已验证实验代码可以追溯；**不能替代在用户 Ubuntu 上实际执行 Git 命令**。

## 1. 源码管理

- **实验前**：明确输入轨迹 NPZ、IMU NPZ、扫描 NPZ、ROS2 启动配置及 Git commit SHA。
- **开发中**：保留原版，创建新的实验变体脚本与输出目录，例如 A3_B2；不覆盖 A3-B0/B1 结果。
- **最小验收**：Python 语法检查；输入 NPZ 的时间戳/形状/来源检查；至少一帧 LiDAR 和 ROS2 消息检查；所有真实结果注明观测样本数、轨迹长度、对齐方法。
- **每次通过一个实验验收**：当天执行本地 `git add` / `git diff --cached` / `git commit` / `git push`。记录 commit SHA。
- **不要声称已推送**，除非远程 `git ls-remote` 或 GitHub connector 已确认对应 SHA。

## 2. 归档入口

仓库：`stone-cabel/FAST_LIO`；分支：`archive/a2-a3-20261106`；归档根：`experiments/a2_a3_reproduction_20261106/`。

推荐命令（在 Ubuntu 的完整/稀疏克隆中执行）：

```bash
cd ~/research/fastlio-a2-a3-archive-upload
git branch --show-current  # 必须是 archive/a2-a3-20261106
git status --short
# 经人工检查后，复制本次新实验 .py/.cpp/.hpp/.yaml/.sh 到归档子目录
git add experiments/a2_a3_reproduction_20261106
git diff --cached --stat
git diff --cached --check   # 历史原始源代码可能有空白警告；不要擅自改动历史快照
git diff --cached           # 查密钥/隐私/第三方受限代码
git commit -m 'Archive <experiment-id> validated source and reproducibility metadata'
git push origin archive/a2-a3-20261106
git rev-parse HEAD
git ls-remote origin refs/heads/archive/a2-a3-20261106
```

## 3. 每次实验必须记录的元信息

- 实验名称 / 日期 / 方法修改 / 对照组 / 固定变量。
- 代码仓库 SHA；源码变体路径。
- 真实轨迹缓存：`pose_cache_source`（不能仅写生成器默认路径）。
- IMU 真值生成方式、时间基准与传感器机体系。
- LiDAR 方向/束时间/静态几何锚点和动态与静态网格版本。
- 评估：同一时间区间，多少个时刻，首帧 SE(3) 对齐还是全轨迹拟合；位置/姿态 RMSE、最终误差、匹配数/信息矩阵特征值。
- 未知项显式写“待核实”，不要从文件名猜测。

## 4. 保密与大型数据

此仓库为**公开仓库**。绝不上传 token、私钥、`.env`、课题组私有代码或未经授权的第三方代码。默认仅归档源码与 SHA256 清单，不上传大型 NPZ、训练权重、缓存或用户隐私；大型数据使用单独受控存储，只记录相对路径和 SHA256。

## 5. A2/A3 当前优先任务

1. 从原始 A2 NPZ 读取 `pose_cache_source`，核实是否匹配 A2 的 GT 和 IMU 轨迹。
2. 在重叠时间区间对 A3-B0/B1 做五点首帧对齐误差检验。
3. 生成可复现审计结果后提交新脚本到本分支。
4. 然后再决定是否需要单独做 IMU 初始化或 LiDAR/IMU 标定消融。
