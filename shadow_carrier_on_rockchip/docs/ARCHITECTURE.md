# 架构与边界契约（ARCHITECTURE）

> 用途：给所有 agent（和接手的模型）一条**唯一事实来源**——每层"答什么问题"、"拥有哪些文件"、
> "绝不碰什么"、以及**跨线只走哪些冻结接口**。开工前先读本文件 + `EXCHANGE.md` 当前状态。
> 本文件由 [HRI] 起草、[统筹] 裁决（EXCHANGE `[10-17 00:41]`）。

---

## 1. 一句话心智模型

系统按 **"四问 + 两横切"** 分层。每条线只回答一个问题：

| 层 | 回答 | 关键词 |
|---|---|---|
| **云台（状态/几何）** | 目标在车的**真实坐标**里在哪？ | **读数** |
| **世界理解** | 这些读数**哪好 / 哪差**？ | **打分** |
| **HRI** | 结合主人状态，我**该干嘛**？ | **行为** |
| **运动控制** | 车**怎么动**？ | **轮速/作动** |
| **认主**（横切） | **谁**是他？ | 身份 |
| **基建**（横切） | 让**链路活着** | 部署/自愈 |

> 口诀：**读数 → 哪好 → 干嘛 → 怎么动**；认主给"谁"，基建保"活着"。

⚠️ **世界理解 ≠ 理解人**。认人 = 认主，读意图 = HRI。世界理解只懂**空间**。

---

## 2. 数据流

```
   感知                读数                 哪好               干嘛              怎么动            作动
┌─────────┐  dets ┌──────────────┐      ┌────────────┐    ┌─────────┐      ┌────────────┐    ┌─────────┐
│ 相机/    │──────►│ 云台/状态     │─────►│ 世界理解    │───►│  HRI     │─────►│ 运动控制    │───►│ C3 / S3  │
│ YOLO NPU│ :8080 │ decoder/KF   │      │ sector_     │    │ hri_    │      │ follow_    │    │ 底盘+舵机│
│ (11fps) │      │ calib/imu    │      │ score/grid  │    │ state   │      │ ctrl/PID   │    │          │
└─────────┘      └──────┬───────┘      └────────────┘    └─────────┘      └─────┬──────┘    └──────────┘
                        │ 距离/速度/航向                  ▲                      │ UART
                        └──────────直接供 HRI────────────┘          DIFF(开环) / MEC(V2)
     [认主 owner_id.py] ──owner.json──► 云台 / HRI / 运动控制          (谁是他, 横切)
     [基建 deploy/]     ──守护链路───► 全部                            (让链路活着, 横切)
```

要点：
- **云台的输出是所有下游的公共输入**（HRI 要距离/速度，运动控制要 bearing，世界要位姿）。
- **HRI 不直接控车**：`send_cmd` 默认是空函数，只产出 `{state, action}` 与 `hri_state.json`。
- **转向通道属于运动控制**（"怎么动"）；HRI 只是停驻域的**消费方**（读 `parked` 决定是否 `turn_only`）。

---

## 3. 各线边界表（拥有 / 不碰）

| Agent | 一句话 | 拥有 | **不碰** |
|---|---|---|---|
| **认主** | 谁是他 | `owner_id.py`（模板/α-β/`weak_cue`/换人确认门）·`owner.json` | 运动 / 世界打分 / 行为 |
| **云台** | 状态·几何·读数 | `state/gaze.py`·`state/calib/`·`state/decoder.py`·`state/solve_geom.py`·`state/tracker.py`·`scripts/imu_daemon.py`（**采集**） | 底盘作动 / 行为决定 / 世界打分 |
| **世界** | 哪好/哪差（与人无关） | `world/**`：`door_v3`·`grid.json`·`sector_score`·`geometry_map`·`lidar_guard` | 认人 / 意图 / 运动 |
| **HRI** | 我该干嘛 | `scripts/hri_state.py` + `hri_state.json` | 运动律 / 几何标定 |
| **运动控制**（V2 新） | 车怎么动 | `follow_controller.py`·运动学/编码器/PID/里程计·S3 运动协议 | 认人 / 行为决策 / 几何 |
| **基建** | 让链路活着 | `deploy/`·systemd·`health_check.sh` | 行为 / 控制 / IMU |
| **统筹** | 协调 | `ROADMAP`/`EXCHANGE`/merge | 各线实现细节 |

---

## 4. 冻结接口（跨线只走这些）

| 接口 | 产出线 | 消费线 | 形式 / 路径 | 内容 | 稳定性 |
|---|---|---|---|---|---|
| `/api/detections` | **感知**（独立/基建托管：`yolo_daemon`+`video_stream_v7`） | 世界 / HRI / 认主 / 云台 | HTTP `:8080` | 检测框 + `stats` | 冻结（只允许**增量**字段） |
| `owner.json` | 认主 | HRI / 运动控制 / 云台 | `/dev/shm/owner.json` | `{ts,bbox,score?}` | 冻结 |
| `grid.json` + `sector_scores` | 世界 | HRI | `~/world_lab/fusion/grid.json`；`sector_score.py` 纯函数 | 8扇区×3环极坐标 / 扇区亲和度 | 冻结 |
| `hri_state.json` | HRI | 运动控制 | `/dev/shm/hri_state.json` | `{ts,state,action,parked}` | 冻结（新增）；读不到→运动控制回落 `tick()`（已实现） |
| `imu.json` | 云台（采集） | 运动控制 | `/dev/shm/imu.json` | `{ts,yaw_rate_dps,yaw_deg,gyro_raw,temp_c,quality}` | 冻结 |
| UART | 运动控制 | C3（V1）/ S3（V2） | `/dev/ttyACM0` 115200 | V1：`MOVE F/B/L/R <0..255>`、**`DIFF L R`（弧线/原地转主用）**、`STOP`；V2：`(vx,vy,wz)` 闭环 | V1 冻结；V2 迁移，**保 DIFF 回退** |
| 里程计（V2 新） | 运动控制 | 世界 / HRI | 待定 | 里程/位移（补 `dist_fn` 无源缺口） | 待设计 |

**规则**：改动冻结接口 = ⚠️ 在 `EXCHANGE.md` 显式登记影响哪些线；**只允许向后兼容的增量**。

> **补注（2026-10-19）**：V1 的 `dist_fn` 无源缺口已用**单目地面几何**部分补上
> （`state/range_from_box.py` + `rk_control._get_owner_dist`：bbox 底边 + 机位高度 → 地面距离，
> 有效 ≈1.6–6m；**<1.6m 时脚已出画 → 返回 None**，退回框高兜底 + follow 的速度门）。
> 这是运动控制线**内部**的接线，不改任何跨线契约；V2 里程计仍按本表待设计。

---

## 5. 两条最容易混的边界（含实例）

**① 运动控制 vs HRI = "怎么做" vs "要不要做"**
- `HIDE`：HRI 只挑**安全方位**（`sector_score` 最高的 bearing）；**怎么倒进去、轮速、掉不掉头**归运动控制。
- 实例 **passby**：转向通道是运动控制本职；HRI 只在停驻域请求有界转头 → **两者不矛盾，是上下游**。
- **A1.3 动作消费（2026-10-20）**：HRI 只写 `{state, action, parked}`；`rk_control` 的动作桥
  （`_hri_bridge_loop`）把 `APPROACH / BACK_OFF / GOTO_SAFE <bearing>` 映射成**有界短脉冲**
  （APPROACH ≤3s、BACK_OFF 0.6s、GOTO_SAFE 转向 ≤3s + 前进 ≤1.5s；障碍 ≤30cm / 动作变化 /
  IMU 丢失 → STOP）。桥激活期间**丢弃跟随命令**（单控制器纪律）。

**② 世界理解 vs HRI = "哪好/哪差（与主人无关）" vs "我该不该去（与主人状态有关）"**
- `sector_score` 只出"这个方位亲和度多少"（纯空间，可离线回归）；HRI 才引入"**主人静止 20s 才躲、躲最高分方位**"。
- 实例 **hide_occupied**：根因在 HRI 的 RECEIVE 判据（已修），**不是**世界打分错。

---

## 6. V1 / V2：运动控制的拆分计划

- **V1（当前 10 月 demo）冻结**：差速底盘，`follow_controller` + 转向通道（`TURN_ENABLED` 默认 `False`，逐步启用）。拆分**不在 V1 做**（demo 就差动作层，拆线=接口碎片化风险）。
- **V2 起跑时执行"重切云台"**（EXCHANGE `[10-17 00:41]`）：
  - **[云台] 留**：凝视 / 标定 / 解码 / 几何反解 / 极坐标KF / **IMU 采集** = 状态·几何·读数。
  - **[运动控制] 新**：底盘**本体运动学（麦轮逆/正解）** · 4 编码器闭环 · PID（轮速域+航向域） · **IMU 消费** · **里程计** · `follow_controller`（运动律+转向通道）· S3 运动协议 = 本体·作动。
  - 纪律（`V2_CHASSIS_GIMBAL_DESIGN.md §5.4`）：vx/vy→底盘平移、wz→慢速航向环、pan/tilt→云台；**保留 DIFF 一键回退**。
- **边界**：**IMU 采集 = 云台**、**IMU 消费/航向 PID = 运动控制**、**链路守护 = 基建**。

---

## 7. 术语消歧（撞车词）

| 词 | 在**云台**语境 | 在**运动控制**语境 |
|---|---|---|
| **运动学** | 目标的**外感受几何**（像素→极坐标/方位） | 底盘**本体运动学**（编码器→车体位移、麦轮逆解）→ 建议叫"底盘运动学" |
| **方位/bearing** | 输出即契约：**右正**；`theta_world = (pan-PAN_FORWARD) - decoder_bearing`，**勿与 pan 直接相加**（V1 埋过此 bug） | 消费它做转向 |
| **距离** | 视觉测距（宽高反推，`Kw/宽度`） | 编码器里程（更准，V2 上） |

---

## 8. 线名 ↔ 模块

`[云台]`= `state/` + `scripts/imu_daemon.py`（采集） · `[世界]`= `world/**` · `[认主]`= `scripts/owner_id.py` ·
`[HRI]`= `scripts/hri_state.py` · `[运动控制]`= `scripts/follow_controller.py` + `scripts/rk_control.py`（+V2 底盘） ·
`[基建]`= `deploy/` · `[统筹]`= 协调。

---

## 9. 变更规则

1. 动手前先读 `EXCHANGE.md`「当前状态」；`git pull`，确认目标文件不在别人「进行中」清单。
2. 改**冻结接口**：在 `EXCHANGE.md` 用 ⚠️ 标出影响哪些线，只做向后兼容增量。
3. 只追加、不改删别人在 `EXCHANGE.md` 的条目；长期决策写 `docs/ROADMAP.md`。
4. 提交本地，由 [统筹] 推送。

> 位置注：本文件在 `shadow_carrier_on_rockchip/docs/`，与根 `docs/` 并存——**以本文件为架构权威**，位置暂不统一。

---

## 10. 文档权威源（同一事实只信这三份）

> 项目文档已 18+ 份，同一事实多处并存会打架。**规则：下表"权威源"之外的都是"背景/细节"，冲突时以权威源为准；改权威源要在 `EXCHANGE.md` 登记。**

| 类别 | **权威源（改这个才算数）** | 降为背景（只读/互引） |
|---|---|---|
| 架构 / 边界 / 接口 | **`ARCHITECTURE.md`（本文件）** + `EXCHANGE.md`（动态） | — |
| 计划 / 排期 | **`ROADMAP.md`**（+ `REVIEW_AND_SEMESTER_PLAN.md`、`V2V3_ROADMAP.md`） | `PACKAGING_PLAN.md` 的排期表 |
| 动态 / 进度 / 契约变更 | **`EXCHANGE.md`**（本地，不入 git） | — |
| 演示 | `DEMO_PLAN.md` | `DEMO_VIDEO/` |
| **实地验收 / 拍摄** | **`FIELD_TEST_PLAN.md`** | `DEMO_PLAN.md`（剧本） |
| 平台 / 硬件 | `V2_CHASSIS_GIMBAL_DESIGN.md` | `V2_LIDAR_*`、`CAMERA_SELECTION.md` |
| 研究 / 定位 | `RESEARCH_POSITIONING.md` + `PRESENCE_MANAGEMENT_SURVEY.md` | `AFFINITY_SCORE_LITERATURE.md` |
| 封存清单 | `DEFERRED_WORK.md` | — |

**口诀**：**架构看 `ARCHITECTURE`，计划看 `ROADMAP`，动态看 `EXCHANGE`**；其余都是它们的注脚。
