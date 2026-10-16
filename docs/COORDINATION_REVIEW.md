# 统筹复核（COORDINATION REVIEW）

> 写于 2026-10-17（HEAD `b2aa88d`）。一次**统筹级全面复核**：内容清单 / 预期效果 / 世界已有方案 / 项目定位 / 统筹诊断与纪律。
> 配套权威源：架构=`ARCHITECTURE.md`、计划=`ROADMAP.md`、动态=`EXCHANGE.md`；本文是它们的注脚。
> ⚠️ 本轮实时联网搜索被限流，§3 的"世界已有方案"取自**本项目既有文献账**（技术 README 论文雷达 + `PRESENCE_MANAGEMENT_SURVEY.md` + `AFFINITY_SCORE_LITERATURE.md`）。

---

## 0. 一句话结论

**统筹的"软"部分（架构/契约/研究/包装）已超前于"硬"部分（实机落地）**——现在最该做的是**把球踢回实机**（板子上电 → 同步 → 宿舍 passby 测试），别再在同一层反复打磨。

---

## 1. 内容全量清单

**代码**（`shadow_carrier_on_rockchip/`）

| 模块 | 文件 | 状态 |
|---|---|---|
| 感知 | `perception/yolo_daemon.cc` · `scripts/video_stream_v7.py`(v7.3 自愈) | 🟢 在跑 |
| 认主 | `scripts/owner_id.py`（模板/α-β/`weak_cue`/换人确认门） | 🟢 |
| 云台·状态 | `state/gaze.py`·`calib/`·`decoder.py`·`solve_geom.py`·`tracker.py`(极坐标KF)·`imu_daemon.py` | 🟢/⏸ |
| 世界 | `world/`(door_v3·polar_mapper·geometry_daemon/geometry_map·sector_score·lidar_guard·of_bridge) | 🟢 离线 OK，**在线 daemon 未起** |
| HRI | `scripts/hri_state.py`(+`hri_state.json`)·`hri_params.json` | 🟢 骨架，实机未串跑 |
| 运动 | `scripts/follow_controller.py`(转向通道/距离保持/丢人后退)·`rk_control.py` | 🟢 实机跑 |
| 基建 | `deploy/`(c3-watchdog·camera-usb-fix)·`configs/`·`health_check.sh` | 🟢 |
| 执行 | `C3_USB_Controller/`(MOVE/DIFF/PAN/TLT/OBS) | 🟢 |

**文档 18+ 份** + **7 套离线回归测试**（turn_pivot·passby_lock_confirm·follow_dt·weak_cue·lidar_guard·bearing_convention·hri_receive_close）+ **`ARCHITECTURE.md`（分层契约，单一事实来源）**。

---

## 2. 预期效果（各线"什么叫做成"）

| 线 | 预期效果 |
|---|---|
| 认主 | 单人/多人/擦身场景**只跟主人、不漂路人**；分离后能重捕获 |
| 云台 | 提供**真实坐标读数**（bearing/距离/速度/航向），供所有下游 |
| 世界 | 出**空间打分**（哪好/哪差），可离线回归、与主人无关 |
| HRI | 结合主人状态**决定行为**（FOLLOW/WAIT/HIDE/RECEIVE/YIELD），**不直接控车** |
| 运动控制 | **车怎么动**：跟随/转向/距离保持/避障（V2 起麦轮+编码器闭环） |
| 基建 | 链路**一直活着**（相机/USB/服务自愈） |
| **demo** | 8 分镜真机跑通 ≥3 次，**passby 必现不丢主** |
| **V2/V3** | V2 全硬件 + release；V3 纯固件（局部世界 + 多人 + 在场管理） |

---

## 3. 世界已有方案（对标）+ 缺口

**① 跟随载物产品**：Gita/gitamini（$2875，**被动跟随 + 按键配对**）｜跟随行李箱（Airwheel/Cowarobot/OVIS，一代）→ 只解决"跟着走"。
**② 智能购物车**：Caper Cart / Dash Cart（扫码即走+广告，**零售侧、非陪伴**）。
**③ 跟随研究**：Follow-Bench(2509.10796)｜EscortE(2506.23573，following/lagging/stopping)｜SDA(ICLR2025，Find→Seek→Follow→**Lost**)｜Adap-RPF(2510.11308)｜GUIDER/Synergy Map｜Event-Driven Proactive(2603.23950)｜DepthDrive(身高先验测距)｜RK3588S 自监督深度 67FPS。
**④ 社会导航/礼让**：Sisbot/Möller/Mirsky 综述、Hetherington yield cues——**都是"在人群里走"，不是"陪伴体主动分开"**。
**⑤ Calm/Ambient + 在场缺席**：Weiser/Case calm tech、SAGE「Presence and Absence」、Baraglia IJRR「when should a robot take initiative」——**理论根，无人做移动陪伴体的在场管理**。
**⑥ 同伴平台**：TurtleBot4 / 轮趣·幻尔·亚博 ROS 小车——**平台层商品化，应用层（陪伴/分离）空白**。

**缺口（我们的缝）**：**没人把"分离"当一等公民**（主动分/找地方等/重逢），也**没人做具身 calm-tech 的在场管理**。

---

## 4. 项目定位（三句话）

- **工程**：**边缘全栈陪伴体**——0.8TOPS、单目、**无云、无 SLAM**、单机可跑。
- **研究**：**Presence Management（在场管理）**——口诀 **读数→哪好→干嘛→怎么动**；核心命题=**主动分离**（预判→等/绕→重逢），主脊 **A1→B3→C1**。
- **作品/包装**：5 作品 · 3 轨（作品集/科研/专利）· 三位老师（世界模型/无人驾驶/HRI）。
- **一句话**：**把"跟随"当入场券，把"懂情境、管在场"当护城河。**

---

## 5. 统筹诊断

**✅ 做得好的**
1. **契约化协作**：冻结接口 + `EXCHANGE` 记账 + `ARCHITECTURE.md`（补上"名实不符"）；
2. **工程纪律**：默认关（TURN/GAZE/雷达）+ 一键回退 + **7 套离线回归当合并门** + 分步上板；
3. **诚实**：边界都写了（幻影/伪需求/不写"世界模型"/持续同色冒名者仍会锁）。

**⚠️ 风险 / 该收的地方**
1. **文档 18+ 份有漂移风险** → 用 `ARCHITECTURE.md §10 文档权威源`压住；
2. **工程（实机）是唯一瓶颈**：板子离线、**动作层 A1.3 未真机**、**passby 台架未跑**、**`pivot_calib` 未标**；
3. **协调开销**：多 agent + 多接口 → 靠权威源 + 冻结件压住；
4. **V1 冻结 vs V2 重切**：demo 前不动结构（已裁决）。

---

## 6. 高质统筹的三条纪律

1. **单一事实来源**：架构看 `ARCHITECTURE`、计划看 `ROADMAP`、动态看 `EXCHANGE`；其余是注脚。
2. **合并门 = 回归测试 + 默认关**（继续执行）。
3. **力气压在实机**：demo 前只做"能让 8 分镜真机跑通"的事（动作层 + passby 台架 + pivot_calib），其余延后。

---

## 附：维护约定

- 本文件随"统筹复核"节奏更新（如每次大节点/月）；改动登记 `EXCHANGE`。
- 与 `ARCHITECTURE.md §10` 权威源冲突时，以 `ARCHITECTURE.md` 为准。
