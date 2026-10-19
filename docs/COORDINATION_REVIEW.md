# 统筹复核 v2（COORDINATION REVIEW）

> 写于 2026-10-19 23:0x（HEAD `7b4ddc5`）。距 10-31 demo **12 天**。
> 本文是**统筹级全面复核**：内容清单 / 预期效果 / 世界已有方案 / 项目定位 / 诊断 / 12 天作战表。
> 权威源：架构=`ARCHITECTURE.md`、计划=`ROADMAP.md`、动态=`EXCHANGE.md`；本文是它们的注脚。
> v1 写于 10-17（`b2aa88d`）；v2 的差异：**联网补全世界方案**、**把"实机缺口"从"passby 参数"纠正为"行为层/世界层/动作层未上板"**、**按"V1 只要求能动起来"重排优先级**。

---

## 0. 一句话结论

**软的部分已经不缺了，缺的是"把 demo 链真正接到车上"。**
当前实机只有 **感知 + 跟随 + 手动** 在跑；**HRI 行为层、世界选点、动作层（APPROACH/GOTO_SAFE/BACK_OFF）都没上板**。
12 天里不要再加能力、不要再调 V1 速度阈值——**只做"让 8 分镜在真机上跑起来"的接线与彩排**。

---

## 1. 现状全量表（2026-10-19 实测）

### 1.1 代码 / 测试

| 模块 | 文件 | 状态 |
|---|---|---|
| 感知 | `perception/yolo_daemon.cc` + `scripts/video_stream_v7.py` | 🟢 板端在跑（14.9fps，age≈40ms） |
| 认主 | `scripts/owner_id.py`（模板/α-β/`weak_cue`/换人确认门） | 🟢 代码在；**实机 enroll 曾 `samples=0` → 需复验** |
| 云台/状态 | `state/gaze.py`·`calib/`·`decoder.py`·`solve_geom.py`·`tracker.py`·`imu_daemon.py` | 🟢 采集在跑；云台 demo 期冻结 |
| 世界 | `world/**`（door_v3·geometry_map·sector_score·lidar_guard） | 🟡 离线 OK；**在线 daemon 未起，`grid.json` 陈旧** |
| HRI | `scripts/hri_state.py` + `hri_params.json` | 🟡 仿真/离线 OK；**板端无服务、无 `hri_state.json`** |
| 运动 | `follow_controller.py`（转向通道/距离环/地面测距）·`rk_control.py` | 🟢 板端在跑；**只消费 `parked`，不消费动作** |
| 执行 | `C3_USB_Controller/`（MOVE/DIFF/PAN/TLT/OBS） | 🟢 固件在；**C3 当前被拔掉** |
| 基建 | `deploy/`（c3-watchdog·camera-usb-fix）·`health_check.sh` | 🟢 服务在跑；**`deploy/` 未同步到板** |

**回归**：7 套主回归（turn_pivot / passby_lock_confirm / follow_dt / weak_cue / heading_hold / hri_receive_close / control_glue）**本地全绿**；`lidar_guard` 绿；`world/fusion/fusion_test.py` **仅因联网超时失败**（应改离线夹具，属 CI 卫生问题，不是代码回归）。

### 1.2 Git / 板端

- 本地 `HEAD = origin/main = 7b4ddc5`，工作区干净。
- 板端已同步 10-19 代码（`follow_controller 25b6485d` / `rk_control 502f7aaf` / `range_from_box bdedb769`），板端 3 套测试全绿。
- 板端 **active**：`rk-control` / `video-stream` / `imu` / `c3-watchdog` / `ap-hotspot`；`yolo_daemon` 在跑。
- 板端 **没有**：`hri_state`、`owner_id` 常驻、`geometry_daemon`、`hri_state.json`。
- **C3 未插**：`/dev/ttyACM0`、`/dev/serial/by-id` 均不存在 → 现在发不出电机命令。

### 1.3 10-19 已落地（不要重做）

- 近距框高门只认**正前 100px**（passby 不再"转 30° 就停"）。
- 离线审查四修：高度门 3 拍重播种 / 退出用原始 θ / 入转驻留 / 偏轴速度门。
- **单目地面测距** `state/range_from_box.py` → `dist_fn`（有效 1.6–6m，<1.6m 退回框高+速度门）。
- 机主裁定：**V1 只要求"能动起来"**；近距余量/制动模型/`--stall` 标定 → **V2**；撤回"主人 ≥2m 外侧绕过"。

---

## 2. 预期效果（各线"什么叫做成"）

| 线 | 预期效果 | 当前判定 |
|---|---|---|
| 认主 | 单人/多人/擦身**只跟主人、不漂路人**；分离后可重捕获 | 🟡 代码齐；**换人门未真机双人验** |
| 云台 | 提供 bearing/距离/速度/航向 | 🟢 读数可用；云台运动冻结 |
| 世界 | 出空间打分、可离线回归 | 🟡 离线 OK；**在线选点未跑** |
| HRI | 五态 FOLLOW/WAIT/HIDE/RECEIVE/YIELD，**不直接控车** | 🟡 仿真 OK；**板端未常驻** |
| 运动 | 跟得上、擦身不丢、停得住 | 🟢 跟随/转向可用；**passby 待复测** |
| 基建 | 链路一直活着 | 🟢 服务自愈在线 |
| **demo** | 8 分镜真机 ≥3 次；passby 必现 | 🔴 **实机只覆盖分镜 1/2/6/8 的一半** |

---

## 3. 世界已有方案（2025–2026）与我们的缝

> 结论先说：**"跟着走"已经是红海**（基准、MPC、RL、商用产品都齐）；**"分离后去哪等、怎么让主人找得到"仍是稀薄区**，
> 但已出现"目标重捕获/搜索"与"subtractive design/exit"的邻近工作——我们的说法必须更精确。

### 3.1 跟随 / RPF（红海，别在这层比）

| 工作 | 它做到什么 | 对我们的含义 |
|---|---|---|
| **Follow-Bench**（arXiv 2509.10796） | 首个 RPF 统一基准；8 个规划器重实现；安全-舒适权衡；真机验证 | 跟随已被系统化评测；我们**不卷拥挤人群规划** |
| **Adap-RPF**（arXiv 2510.11308） | 自适应轨迹采样 + MPPI；15 人 69.4% TSR | V3 可借鉴；V1 不需要 |
| **Proximity–Safety Balance**（arXiv 2608.10056） | 约束分解 RL；SR 78%，撞人率 10.7% | 说明"近-安全"是公开难题；我们 V1 用速度门+测距兜底已够 |
| **HiCrowd**（arXiv 2602.05608） | RL+MPC 跟人流；Expo 2025 大阪实部署 | 密集人流是别人的主场 |
| **Follow Everything**（arXiv 2504.19399） | 分割+距离帧缓冲；**Retreating/Following/Chasing/Planning/Switching** 状态机 | ⚠️ 最接近我们：它已有"退"状态，但仍是**跟随内部状态**，不是"主动分离+选等待点" |
| **CARPE-ID**（arXiv 2310.19413） | 持续自适应 ReID | 认主可借鉴；V1 不用 |

### 3.2 丢人 / 重逢（邻近，不是空白）

| 工作 | 它做到什么 | 对我们的含义 |
|---|---|---|
| **RPF-Search**（arXiv 2503.02188） | 未知动态环境找目标；地形遮挡 vs 动态遮挡两套机制 | "After"可借鉴；但它是**失败恢复**，不是"主动分" |
| **FoV-Aware Belief Graph Search**（RO-MAN 2026） | 视野感知的信念图搜索重捕获 | 同上 |
| **Map-informed tracking**（arXiv 2609.07091） | 结构地图约束；7s 遮挡 71.4% 重捕获 | 我们有 `grid.json`，V3 可对上 |

### 3.3 购物 / 载物陪伴（产品层已商品化）

| 方案 | 它做到什么 | 对我们的含义 |
|---|---|---|
| **Shopping Service Robot**（RA-L 2026, DOI 10.1109/lra.2026.3668142） | RGB-D + 无线电磁融合；货架遮挡下 4.56mm/2.98°；窄通道跟随 | 超市跟随已有强工程解 |
| **CartBot**（MDPI Electronics 2025）/ **Dobby-Cart**（商用，9h 跟随） | 购物清单路径 + 跟随载物 | "车+篮子+跟随"确实平庸 |
| **gita mini**（$2,875，20 lb） | 被动跟随 + 按键配对 | 我们的价格/无按键是差异，但不是研究护城河 |
| **LG CLOiD / Samsung Home Companion / GS Shop Chemi Friends**（CES 2026） | 家务/陪伴/订阅租赁 | 陪伴叙事正在升温；**"在场管理"是空位** |

### 3.4 擦身 / 社会距离（直接约束我们的 passby）

| 研究 | 结论 | 对我们的含义 |
|---|---|---|
| **Greenberg et al., IJSR 2025** | 走廊擦身：**中等曲率 k≈0.4 最舒适**；6m 开始避让优于 4m | 我们"原地转 15–30° 再走"是**高曲率**，舒适度差；V1 至少放慢、V2 改平滑弧线 |
| **Neggers et al., Frontiers 2022** | 擦身距离越大越舒适；**>90cm 后收益趋平**；速度越快越不适 | 走廊 0.8–1.5m 是"能接受但偏紧"；demo 尽量 ≥1.2m、降速 |
| **IJHCI 2025** | **>120cm + 低于步速** 不适感最低 | 与上条一致：宁可慢，不要贴 |
| **Follow-Me 2020** | 用户偏好**社会距离 ~2m、居中无侧偏**；±50cm 可接受 | 我们纵向 2.5m 设定有依据；侧偏别太大 |

### 3.5 在场 / 缺席 / 退场（我们的理论近邻）

| 工作 | 它说什么 | 对我们的含义 |
|---|---|---|
| **Designing for Exit**（WeRobot 2022） | HRI 长期只研究"维持参与"，**退出/分离被忽视** | 直接支撑"分离是一等公民"的问题重述 |
| **Subtractive Design**（SSRN 2026, "Eclipse of the Automaton"） | 机器人的"退"不是失败，是设计 | ⚠️ 最强近邻：我们不是唯一提"退"的人 |
| **Calm Technology in HRI**（2026） | 最小注意力、 periphery、失败可用 | 支撑"不打扰"的评价维度 |
| **Absence of companion robot**（Frontiers 2023） | 陪伴体缺席会造成真实情感后果 | 支撑"缺席"值得研究 |

### 3.6 精确的缺口（v2 修正）

❌ 不再说："世界上没人做分离。"
✅ 应说：**现有工作把"分离"当作三类东西**——
1. **失败恢复**（丢人→搜索/重捕获：RPF-Search、FoV-aware）；
2. **跟随内部状态**（Follow Everything 的 Retreating）；
3. **对话/关系层退出**（Designing for Exit、Subtractive Design）。

**仍然空着的是**：**空间性的、主动的、可被主人读懂的"分离—等待—重逢"**，尤其是
**"去哪等"（等待点的隐式契约）** 与 **"何时回"（信念驱动重逢）** 在**边缘、无地图、无云**条件下的实现与量化。
这就是 B3 + A1 + C1 的缝，也是我们唯一值得押的护城河。

---

## 4. 项目定位（三句话 + 诚实边界）

- **工程**：**边缘全栈陪伴体**——0.8 TOPS、单目、无云、无 SLAM、单机可跑，BOM ¥500–1000。
- **研究**：**Presence Management（在场管理）**——把"跟随"降为入场券，把"**何时在场/缺席/如何重逢**"当主问题；
  主脊 **A1（分离风险）→ B3（等待点隐式契约）→ C1（信念重逢）**。
- **作品/包装**：5 作品 · 3 轨 · 三位老师（世界模型 / 无人驾驶 / HRI）；同一仓库，三套说辞。

**诚实边界（套磁时必须主动说）**：
- 四条"研究装置"证据（预判分离 / 等待契约用户研究 / 主动重逢 / "不用想它在哪"）**目前 0/4**；
- V1 是**脚本化 demo**，不是用户研究；方法内核（KF/MPC/costmap）不前沿；
- 差异化在**问题重述 + 系统整合 + 边缘约束**，不在单点算法。

---

## 5. 统筹诊断（按 demo 影响排序）

### ✅ 做得好的
1. **契约化**：`ARCHITECTURE.md` 冻结接口 + `EXCHANGE` 记账 + 默认关 + 一键回退。
2. **合并门**：7 套离线回归 + 板端 smoke；10-19 的"近距门"就是被离线审查抓出 3 个真 bug 的。
3. **诚实**：边界、伪需求、失效模式都写进文档。

### 🔴 P0（不解决就没有 demo）
| # | 缺口 | 证据 | 影响 |
|---|---|---|---|
| 1 | **C3 被拔** | 无 `/dev/ttyACM0`、无 by-id | 现在**完全不能动** |
| 2 | **passby 未复测** | 近距门 10-19 才修 | 分镜 4 的核心叙事 |
| 3 | **认主 enroll 可能失败** | 旧日志 `samples=0 → largest-fallback` | 可能跟错人 |
| 4 | **HRI 未常驻** | 板端无 `hri_state` 服务/无 `hri_state.json` | 分镜 3/6/7 不可能；`turn_only` 失效 |
| 5 | **动作层未接** | `rk_control` 只读 `parked`，不消费 `APPROACH/GOTO_SAFE/BACK_OFF` | 分镜 3/7 不可能 |

### 🟠 P1（影响质量/效率）
| # | 缺口 | 影响 |
|---|---|---|
| 6 | 世界 `geometry_daemon` 未起，`grid.json` 陈旧 | HIDE 选点不是实时的 |
| 7 | `pivot_calib` 未实测 | 转向预算/超时只能拍脑袋 |
| 8 | `EXCHANGE` 停在 10-17，10-19 三个提交未记账 | 协调漂移（本次复核已补） |
| 9 | `fusion_test` 依赖联网 | CI 偶发红 |
| 10 | 板端缺 `deploy/` 与部分测试 | 板/仓漂移 |

### 纪律
- **V1 不再改速度/阈值**（机主 10-19 裁定）；只修"跑不通"的接线。
- **合并门 = 回归 + 板端 smoke**；默认关优先。
- **只声称真机跑过 3 次的东西**。

---

## 6. 12 天作战表（10-20 → 10-31）

| 日期 | 目标 | 验收 |
|---|---|---|
| **10-20/21** | C3 插回；重新 enroll（鲜艳衣物、3s、近距离）；**passby 复测**；跑 `pivot_calib` | 手动能动；passby 不再"转 30° 停"；PWM→°/s 有数 |
| **10-22/23** | 板端起 `hri_state --live`（systemd）；起 `geometry_daemon`；**接 A1.3 动作桥**（`hri_state.json` action → `/move`） | 每个动作可触发、可中止；`hri_state.json` 有 `parked/action` |
| **10-24/25** | A1.2 双人 `owner.source=file`；走廊 passby；HIDE 端到端 | 只跟主人；HIDE 落点靠墙不挡门 |
| **10-26/27** | 8 分镜全串 ×3；黑匣子 jsonl | ≥3 次不救场；失败 Top3 有归因 |
| **10-28/29** | 拍 demo 视频；手动兜底；热备 C3 | 素材可用；降级预案可演 |
| **10-30/31** | buffer + 三位老师的一页纸 | 每轨 1 页（T1/T2/T3） |

**关键路径**：`C3 插回 → passby 复测 → HRI/动作层上板 → 8 分镜串跑`。

---

## 7. 需要机主拍板的 3 件事

1. **demo 范围**：按"V1 只要求能动起来"→ 是保 **8 分镜**，还是砍成 **跟随 + passby + 载物 + 手动兜底**？
   （砍 HIDE/RECEIVE 可省 2 天接线，但叙事弱一半。）
2. **是否现在装 HRI/世界服务**：能解锁分镜 3/6/7，但增加现场故障面。
3. **场地/动线/人手**：白天、走廊 ≥1.2m 横向余量、1–2 位同学扮演路人与递物。

---

## 附：维护约定

- 本文件随"统筹复核"节奏更新；改动登记 `EXCHANGE`。
- 与 `ARCHITECTURE.md §10` 权威源冲突时，以 `ARCHITECTURE.md` 为准。
- 世界方案引用均为 2025–2026 检索所得；套磁引用前请再核 DOI/arXiv 版本。
