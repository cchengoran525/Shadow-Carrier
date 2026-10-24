# PC 科研轨研究：ROS 2 + slam_toolbox + X3Pro 建图沙盒

> 写于 2026-10-24。`V2V3_ROADMAP.md` §5"PC 科研轨（SLAM 沙盒，无死线）"的正式展开，
> 与 `V2_LIDAR_RESEARCH.md` §4.5（L5 级）互为引用：那份回答"雷达在产品里是什么"，
> 本文回答"SLAM 在科研轨上怎么做"。**机器人侧零依赖原则不变：车上不跑 SLAM、不上云、
> 不做重定位（V2/V3），本文全部工作发生在 PC 上。**
>
> 脚手架已入仓：`pc_track/slam/`（X3Pro 定制配置 + 一条龙建图脚本 + 零依赖存图）。

---

## 0. 一页结论

| 问题 | 结论 |
|---|---|
| 选型 | **slam_toolbox 2.8.5 正选**（Jazzy 一等公民、无里程计可用、四模式齐备、posegraph 可序列化续图）；cartographer 留作回环不足时的备选；KISS-ICP 只做轨迹交叉验证 |
| X3Pro 建图的最大约束 | **稀疏（实测 1.66°/点）+ 无里程计（手持阶段）**。对策：慢走纪律 + 节点参数调密 + **建图用 6Hz（点密度×1.67）、避障用 10Hz，两通道独立配置** |
| 机器人→PC 数据契约 | 现在：PC 直录 rosbag2（已可用）；V2b 后：机器人无 ROS，走全速率 JSONL（**双时间戳**，吸取 guard 时钟 bug 教训）+ PC 侧转换器 |
| 首图怎么来 | `pc_track/slam/first_map.sh`：手持雷达绕房间一圈 60s → PGM 地图 + rosbag + posegraph 三件套 |
| 产物回流规则 | 参数标定值、门框提取对照、经验教训**可回流**入 docs；**地图本身永不上车**（总原则；车载轻量重定位是底盘 §10.5 L3 的独立立项口） |

---

## 1. 轨道纪律（先把边界立住）

1. **产品零依赖**：SLAM 任何阶段的失败不影响 V2/V3 行为——车上不 import 地图、不跑建图节点。roadmap 总原则 2 原文："机器人侧只做局部情境世界理解；不建全局地图、不 SLAM、不重定位、不连云；SLAM 归 PC 科研轨（离线，吃车上日志，无死线）"。
2. **数据单向流动**：机器人（V2b 起）只**产**原始帧日志；PC 只**吃**。地图/位姿图/轨迹留在 `pc_track/data/`。
3. **回流白名单**（§7 详述）：能上产品仓库的只有"参数值、对照实验结论、格式契约"，不是运行时数据。
4. **无死线 = 不许阻塞主线**：本文所有实验排期让位于 V2a/V2b/V2c。

---

## 2. 环境就绪状态（2026-10-24 本机盘点）

| 组件 | 状态 | 说明 |
|---|---|---|
| Ubuntu 24.04 + ROS 2 Jazzy | ✅ | t2linux 内核对 SLAM 无影响（纯 CPU 负载） |
| slam_toolbox 2.8.5 | ✅ 已装 | 含 online_async / online_sync / offline / localization 四种 launch |
| X3Pro ROS 驱动 | ✅ | `~/x3pro_ws`（仓库外），发布 `/scan`(10Hz) + `base_link→laser` 静态 TF（实测确认）——slam_toolbox 需要的 TF 链现成 |
| rosbag2 | ✅ | `ros2 bag record/play` 可用 |
| cartographer-ros 2.0.9003 | 🟡 在源未装 | 备选，用到再装（需要 sudo） |
| nav2_map_server (map_saver) | ❌ 未装 | **用自写零依赖 `save_map.py` 替代**（PGM+YAML，nav2 语义兼容） |
| 与机器人链路的桥 | ✅ | `world/fusion/ros_scan_bridge.py`（同一台机器时无需它——驱动直接出 /scan；桥的价值在机器人 JSONL→PC 时复用其方位角/噪声处理） |

---

## 3. 选型研究

### 3.1 正选：slam_toolbox

| 需求 | slam_toolbox 的回答 |
|---|---|
| Jazzy 支持 | apt 一等公民（2.8.5），社区默认答案 |
| **无里程计手持建图** | scan-matching 独立可用（无 odom 时仅告警），async 模式逐帧处理——手持慢走成立 |
| 离线/在线切换 | 同一配置四模式：online_async（边录边建）、offline（吃 bag 复现）、localization（对已存图重定位——E5 预演） |
| 续图 | posegraph 序列化（`.posegraph`+`.data`），下次从断点继续——多天扫楼可拼接 |
| 回环 | Ceres 位姿图 + 粗精两级搜索 + rviz 交互手动回环（手持建图人工补一个回环很实用） |
| 已知短板 | 大场景长走廊（特征退化）会漂；稀疏雷达下远处墙面命中少（§4 对策） |

### 3.2 备选与不选

| 方案 | 判断 | 理由 |
|---|---|---|
| cartographer_ros | 备选（回环不足时） | 子图+回环调参空间更大，但配置面大、调参成本高；Jazzy 包存在（2.0.9003）。**触发条件：slam_toolbox 在 E1/E3 中回环失败率肉眼可见** |
| KISS-ICP | 只做交叉验证 | 纯雷达里程计、极稳健、Python；不出占据图。用法：同一 bag 跑出轨迹，与 slam_toolbox 轨迹对照，定位"漂移在建图还是里程" |
| FAST-LIO2 / LIO-SAM | 不适用 | 3D + IMU 紧耦合栈，单线 2D 雷达无 IMU 数据（手持阶段）喂不饱 |
| gmapping / hector | 不选 | ROS1 时代答案，ROS2 无维护版；hector 无回环 |
| nav2-amcl | 后期（重定位阶段） | E5 之后若做"地图坐标系下重定位"再装；robot-localization（EKF）在 V2b 编码器里程上线后用于 odom+IMU 融合回放 |

---

## 4. X3Pro 先天约束 → 参数映射（建图视角，实测数据驱动）

`V2_LIDAR_RESEARCH.md` §2 的实测数字，逐条落到 SLAM 参数上（已写进 `mapper_params_x3pro.yaml` 注释）：

| 实测约束 | SLAM 后果 | 参数/纪律对策 |
|---|---|---|
| **1.66°/点（233 点/圈 @10Hz）** | 3m 处相邻点弧长 ≈8.7cm：远墙稀疏命中，5cm 栅格"打不满" | resolution 0.05 是下限（再细无信息量支撑）；`min_pass_through=2` 默认保守——远墙若不显影先降为 1 试验；**根治是 6Hz（0.99°/点，密度×1.67）** |
| **无里程计（手持阶段）** | scan matching 是唯一位姿源；快转/快走必漂 | 慢走纪律（≤0.3m/s、少急转、多直线）；`minimum_travel_distance 0.15`（默认 0.5 为车载设计）；`distance_variance_penalty` 调宽到 0.3 容忍真实漂移；V2b 编码器里程上线后：bag 里补 `/odom`，回放重跑（数据结构已预留） |
| **室内有效距离 P75=1.35m / max 4.61m** | 大房间远端墙面弱命中 | 房间尺度（<6m 对角）无碍；扫商场时接受"近处先成图" |
| **单平面（13.5cm 或装车高度）** | 桌面/台面不显影（只有腿）→ 地图有"悬空腿"伪影 | 地图语义按"腿高轮廓"理解；这是特性不是 bug——门框/墙恰好在此平面 |
| **动态行人** | 腿簇进地图成"鬼墙" | 会话级建图（短时段）；**复用 guard L2 的 Tracker 动点标记**：回放时剔除 `dynamic=true` 簇再配准（E4 验证）——机器人链路的资产直接喂科研轨，两轨第一次真交集 |
| **rosbag 用 epoch、JSONL 契约用 monotonic** | 时钟混用 → 帧序错乱（guard 双时钟 bug 的教训） | §5 数据契约：JSONL **双时间戳**（`ts_mono`+`ts_wall`）一次写清，转换器零歧义 |

**双通道配置决议（建议）**：避障通道 10Hz（时延优先），建图通道 6Hz（密度优先）。X3Pro 扫描频率是驱动参数，可运行时切换；V2b 上车后若板端只跑一个频率，选 10Hz 保安全，建图密度损失用"慢走+多圈"补偿（E3 实测后再定）。

---

## 5. 数据契约（机器人 → PC，V2b 冻结用提案）

### 5.1 现在就能用的路径（PC 直录）

X3Pro 插 PC、驱动出 `/scan` → `ros2 bag record /scan /tf /tf_static`。已由 `first_map.sh` 自动化。**局限**：只覆盖"雷达在 PC 上"的场景；机器人走动时的数据（真科研素材）必须来自车上。

### 5.2 机器人无 ROS 的路径（V2b"记录原始数据"行的格式提案）

机器人端 SDK 桥在全速率下追加写 JSONL（复用 `lidar_scan.json` 的原子写纪律，避免与 guard 抢文件——**独立文件**）：

```jsonc
// scan 行（10Hz 全速率；对照 guard 契约的 2Hz 调参档，两档分开）
{"t": "scan", "ts_mono": 10892.597, "ts_wall": 1792119149.819,
 "frame_id": 4211, "points": [[方位角°, 距离m], ...]}
// 里程行（编码器里程计，V2a 后有）
{"t": "odom", "ts_mono": ..., "ts_wall": ..., "x": 0.0, "y": 0.0, "yaw": 0.0,
 "vx": 0.0, "wz": 0.0}
// IMU 行（50Hz，与 [云台] IMU 同源）
{"t": "imu", "ts_mono": ..., "ts_wall": ..., "yaw_deg": ..., "gyro_z": ...}
```

**双时间戳是硬要求**：mono 保帧序/间隔（guard 时钟 bug 的教训），wall 对齐 rosbag 的 epoch 时间戳。PC 侧转换器（`jsonl→rosbag2`，scan→LaserScan、odom→Odometry、imu→Imu，方位角左正还原）**在 V2b 格式冻结后实现**，不提前写死。

### 5.3 目录与命名

`pc_track/data/<月日_时分秒>/`：`bag_*/`（rosbag2）、`map_*.pgm/.yaml`、`posegraph_*.posegraph/.data`、`*.log`（slam/record）、实验记录进本文 §6 表。

---

## 6. 实验计划（E1–E5，全部带判据；雷达插回即可跑）

| # | 实验 | 方法 | 判据 |
|---|---|---|---|
| **E1 首图** | `first_map.sh 60`：手持慢速绕房间一圈回起点 | PGM 出图；墙线目视连续、闭合 | 首图即成功；失败看 slam.log 的 scan matching response |
| **E2 尺度真值** | 卷尺量房间长宽 vs `图上像素 × resolution` | **误差 ≤5%** | 尺度漂移是 scan-matching 无里程计的主要风险，量化它 |
| **E3 10Hz vs 6Hz** | 同路径同速度各建一次（驱动参数切换） | 对比远墙连续性、回环残差 | 定"建图通道频率"决议的数据依据 |
| **E4 动态退化** | 同一空间：静止 vs 一人走动，各录各建 | 鬼墙栅格计数对比；Tracker 剔除缓解后复测 | 量化行人污染；验证 guard L2 资产复用价值 |
| **E5 重定位预演** | localization 模式加载 E1 的 posegraph；挪动雷达后给初始位姿 | 位姿收敛、地图不重影 | 车载轻量重定位（底盘 §10.5 L3）的可行性预演——**只在 PC 上** |

纪律：每实验 ≥3 次取中位数报分布（SLAM 随机性弱于 passby，但 scan-matching 有初值敏感性）；bag+posegraph 全套归档（复现的最小集合）；结果回填本表。

---

## 7. 产物与回流规则

| 产物 | 去向 | 可否回流产品仓 |
|---|---|---|
| PGM/YAML 地图 | `pc_track/data/` | ❌ 永不上车（总原则） |
| posegraph + rosbag | 同上，复现最小集 | ❌ |
| 标定/参数结论（如"6Hz 决议""resolution 下限"） | 回填本文/`V2_LIDAR_RESEARCH.md` | ✅ 以文档形式 |
| 门框提取 vs 手标 grid.json 对照 | `V2_LIDAR_RESEARCH.md` §4.3 的离线证据 | ✅ |
| **地图格式知识**（若底盘 §10.5 L3"车载轻量重定位"立项） | 届时定义地图裁剪下发格式 | ✅ 届时议 |

---

## 8. 脚手架使用（已入仓 `pc_track/slam/`）

| 文件 | 作用 |
|---|---|
| `mapper_params_x3pro.yaml` | X3Pro 定制配置（每处改动带注释与依据，对照 slam_toolbox 默认值） |
| `first_map.sh` | 一条龙：自检 /scan → 建图节点 → 录包 → （走完）存 PGM + posegraph → 产物清单 |
| `save_map.py` | 零依赖 map_saver 替代（nav2_map_server 未装也不卡） |

**跑首图**：插雷达 → 驱动在跑 → `cd pc_track/slam && ./first_map.sh 60`，按提示手持雷达绕房间一圈。回放复现：`ros2 bag play <bag> --clock` + `offline_launch.py`。

---

## 附：与既有文档的关系

- `V2V3_ROADMAP.md` §5：本文是其展开；排期服从主线（V2a/b/c 优先）。
- `V2_LIDAR_RESEARCH.md` §4.5 L5：本文即 L5 的实施研究；该文档的实测数字（§2.1）是本文参数映射的输入。
- `V2_LIDAR_AVOIDANCE.md` 附 D-2：`--record` 2Hz 定位收窄为 guard 调参档——本文 §5.2 的全速率 JSONL 是 SLAM 备料档，两者分开。
- 底盘 `V2_CHASSIS_GIMBAL_DESIGN.md` §10.5 L2/L3：PC 建图 → 车载重定位的回流路线图，本文 E5 是其第一步预演。
