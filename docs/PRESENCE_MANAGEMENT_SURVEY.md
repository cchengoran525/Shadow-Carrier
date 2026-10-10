# Presence Management 调研 —— 从"跟随"到"在场管理"

> 写于 2026-10-09。把 SC 的核心从"跟人"上移一层到"**读世界 → 管理在场**"。
> 本文是 `docs/RESEARCH_POSITIONING.md` 的上层框架调研；"主动分离"是其最漂亮的一个特例。
> 文献来自 2026-10-09 网络调研（二手摘要，标 ⚠️ 者需精读原文核实细节）。

---

## 0. 一句话

**Presence Management（在场管理）** = 一个陪伴体根据对情境的理解，决定**"何时在场、以何种方式在场、何时缺席、如何重逢"**。

SC 的五种行为不是五种跟随，而是**五种在场决策**：

| 行为 | 在场语义 |
|---|---|
| FOLLOW | 持续在场（默认） |
| APPROACH / RECEIVE | 及时出场 |
| YIELD | 礼貌让位（短暂退让） |
| HIDE | 体贴缺席（退到不碍事处） |
| **SEPARATE / DETOUR** | **有意缺席**（主动分开，等 / 绕） |

**"跟随"只是 L2 策略表里的一个默认值；真正的智能在 L1。**

---

## 1. 三层阶梯（本项目的统一架构）

```text
L0 感知世界   : 几何 / 物体 / 人 / 事件
L1 理解情境   : 人群要来了？主人快要需要我了？现在是该出现还是该消失？
L2 决定在场   : appear / approach / remain / wait / hide / step-aside / absent
L3 执行       : 底盘 / 云台
```

- **L1 是硬骨头**（读懂情境/意图）——正是"世界模型 + HRI 上来后就轻松"的真正所指；
- L1 一旦立起来，L2/L3 大多是**拼装**（复用现有能力）。

---

## 2. 理论根（四条支柱）

Presence Management 不是凭空造的，它站在四条成熟理论线上：

### 2.1 Calm Technology /  periphery of attention（最直接的根）
- Weiser & Brown, *"The Computer for the 21st Century"* (1991) 与 *"Designing Calm Technology"* (1996)：技术应占据**注意力的边缘**而非中心；
- Amber Case, *Calm Technology*（2015）+ calmtech.com：**"技术应要求尽可能少的注意力"**；通知要**按相关性/时机/微妙度**投放，能不打扰就不打扰；
- 这正是 SC 初心的"**让用户忘记它是机器人**"——**在场管理就是 calm technology 的具身版**。
- 也对应 ambient/invisible computing（2026 综述）：从"显式命令"走向"隐式、情境驱动"。

### 2.2 Implicit Interaction / 隐式交互（SC 已写进根 README）
- 身体语言即接口；设备持续读意图，但用户无意识操作。

### 2.3 Proxemics & 礼貌（Goffman / Hall / Lakoff）
- Hall 的亲密/个人/社交/公共距离；
- Goffman 的 **civil inattention**（礼貌性忽视）——共处一室却不打扰；
- Lakoff 礼貌规则在非人形机器人上被验证可被感知（⚠️ PMC9387416 多实验）；
- **HIDE / YIELD / "不挡道" = civil inattention 的空间实现**。

### 2.4 主动性 / 可打断性（Initiative & Interruptibility）
- "机器人该何时主动"是 HRI 的经典问题（见 §3）。
- **本项目的对称问题：机器人该何时"不主动 / 消失"**——这半张牌少人打。

---

## 3. 相关研究地图

| 方向 | 代表工作（venue/year） | 它做什么 | 与 SC 的关系 |
|---|---|---|---|
| **主动性/倡议** | Baraglia et al., *IJRR 2017*「when should a robot take initiative?」 | 主动 help 提升团队流畅度，但人偏想要控制权 | **对照**：他们问"何时帮"，SC 问"何时退" |
| **主动协助（任务级）** | PACE, *ICRA 2025*；Li et al., *RCIM 2023*「Proactive HRC」 | 估计任务进度→同步协助 | 借鉴其"进度/情境估计"思路 |
| **非侵入式协助（新范式）** | Zhang et al., *2026*「Benchmark & LLM-based framework for **Non-Intrusive Assistance**」（arXiv 2605.01368） | 把"非侵入"形式化并建基准 | ⚠️ **最接近的邻居**，需精读确认边界 |
| **打断/可打断性** | Matsumoto et al., *HRI 2023*；*arXiv 2501.01568* 对话机器人打断处理；Leusmann et al., *2026*「How People Envision Robot Interruption」 | 预测可打断性、礼貌打断 | SC 的"出场时机"= 反向的打断管理 |
| **接近行为/礼貌发起** | Satake et al., *HRI 2009*「Strategies for Social Robots to Initiate Interaction」 | 三阶段礼貌接近、挑"可接近的人" | SC 的 RECEIVE/出场行为直接可用 |
| **Proxemics 评估** | Neef et al., *IJSR 2023*「What is Appropriate?」；Rahmah, *MDPI 2025*；Lawrence, *ACM 2025*「Social Norms in HRI」 | 舒适距离/社会规范建模 | SC 的等待点/间距理论依据 |
| **社会导航 yield** | Hetherington *2021*；Sisbot *2007*；Mirsky/Mavrogiannis/Francis 综述 | 人群中的让行/礼让 | SC 的"让开"= 社会性 yield 实例 |
| **在场与缺席（本体）** | *SAGE 2026*「Social Robots and the **Dialectics of Presence and Absence**」 | 机器人在"模拟在场"的同时**唤起缺席感** | ⚠️ 与本文主题**同名**，直接相关，必读 |
| **陪伴机器人的持续在场** | OlloNi SS1（*IEEE Spectrum 2026*）；长期缺席研究（*Frontiers 2023*） | 长时间 standby、ambient social presence | 产品侧佐证"在场管理"是真实需求 |
| **Calm/Ambient 交互** | Weiser 1991/1996；Case 2015；ambient computing 综述 2026 | 注意力边缘化、非侵入通知时机 | **理论根**（§2.1） |
| **跟人基准** | Follow-Bench（SC 论文雷达） | 跟人任务评测 | SC 动作的量化底座 |

---

## 4. 关键缺口（为什么要做 SC）

把上面串起来看，四个缺口：

1. **对话 ≠ 具身**：打断/主动性研究多在**对话机器人**上；**移动陪伴体**的"在场管理"少人做。
2. **静态 ≠ 移动**：接近/礼貌研究多在**静止社交机器人**上前置发起；**移动中的连续在场/缺席**（跟→分→重逢）缺统一策略。
3. **导航 ≠ 陪伴**：社会导航研究"机器人怎么在人群里走"；**没有把"缺席/退让/重逢"当成一等公民行为**。
4. **"何时退"被忽略**：主动性研究几乎都在问"**何时该主动帮**"；**"何时该不打扰、该消失"这半张牌**几乎没人系统打——**而它恰恰是"陪伴感"的来源**。

> **SC 的位置**：把 calm technology 的"注意力边缘化"**具身化**到一台移动陪伴机器人上，用**世界模型读情境**、用 **HRI 管理在场**，并把"主动缺席"（分离）作为核心实验对象。

---

## 5. SC 的在场三问（研究问题）

| RQ | 问题 | 对应行为 | 现状 |
|---|---|---|---|
| **RQ-τ** | **何时该解耦/退让/出场？**（情境触发） | Pre-separation / 出场时机 | ❌ 未做（唯一新零件） |
| **RQ-π** | **以何种方式在场/缺席？**（等 / 绕 / 靠近 / 保持） | 停等型 / 绕行型 / RECEIVE | 🟡 半成品（HIDE/affinity 复用） |
| **RQ-ρ** | **何时、如何重新在场？**（重准入/重逢） | reunion | 🟡 可复用（认主+跟随） |

**"主动分离" = RQ-τ/π/ρ 在"人群"这一情境上的实例化。**

### 形式化：Intentional Decoupling（有意解耦）
- 状态：`Coupled`（跟随）/ `Decoupled`（分离）；
- `Coupled --τ--> Decoupled(π) --ρ--> Coupled`；
- **安全优先**：分离期碰撞避免是**硬约束**，主人跟踪是**软目标** → **信念驱动重捕获**（belief-based rejoin）：不要求持续视觉锁定，只需维护 `b(x_owner)` 去规划汇合；
- 两型 π：**驻留让行（Stationary Yield）** / **解耦共行（Parallel Traversal）**。

---

## 6. 适用边界（ODD，必须写清）

| 维度 | 适用 | 不适用（明确排除） |
|---|---|---|
| 密度 | 中低密度、人群**短时通过** | 极高密度、人群长时间不散 |
| 空间 | **存在侧向净空 / 等待空间** | 无侧空间（封路/窄门） |
| 场景 | 走廊 / 人行道 / 展馆 / 商场动线 | 马拉松 / 音乐节 / 广场集会 |
| 越界回退 | — | `Safe-Stop` → 人工接管 |

> 声明"我们不往极高密度发展"是**诚实的 ODD 界定**，不是减分。

---

## 7. 复用的现有资产（SC 为什么"轻"）

| 在场能力 | 复用 |
|---|---|
| 等待点选择 | `world/fusion/sector_score.py`（affinity 打分，社会 costmap 雏形） |
| 退让/缺席 | `HIDE`（GOTO_SAFE + 停靠） |
| 重逢 | 认主锁主（owner.json）+ `follow_controller` |
| 情境信号 | `/api/detections`（人/物/朝向）/ `grid.json`（几何/方位）/ A3.5 时间戳 |
| 未来增强 | V2 雷达（360° 净空 + 扫描匹配锚定"记忆"）、麦轮（横向偏移 = 绕行几乎免费） |

---

## 8. 落地分级

| 档 | 内容 | 依赖 | 量级 |
|---|---|---|---|
| **P0 寒招可演** | 数前方 person + 迎面运动 → 停等型（复用 HIDE）→ 人散回 FOLLOW | 现有能力 | ~1 周 |
| **P1 进阶** | 世界模型出 `crowd` 信号 + 绕行型（侧向偏移跟随，麦轮横移）+ 信念重逢 | V2 平台 | ~1 月 |
| **P2 研究版** | **RQ-τ/π/ρ 的时机** + "隐式契约"用户研究（哪种在场让人最舒服） + 受控实验 | 科研轨 | 6-12 月 |

---

## 9. 一句话给老师

> *Existing work asks **when a robot should help** and **how it should navigate among crowds**. We ask the symmetric, under-studied questions — **when and how a mobile companion should be absent (step aside / wait / detour) and how it should return** — grounded in calm technology and social proxemics, and implemented as **presence management** driven by a world model.*

---

## 附：待精读（⚠️）
1. *Social Robots and the Dialectics of Presence and Absence*（SAGE 2026）——同名主题，必读；
2. Zhang et al. 2026「Non-Intrusive Assistance」——最接近的邻居，确认边界；
3. Weiser & Brown 1996 / Case 2015——理论根，引用打底；
4. Satake et al. HRI 2009——接近行为的可直接复用件；
5. Baraglia et al. IJRR 2017——主动性的正面对照。

> 维护约定：每精读一篇，去掉 ⚠️ 并补一行"与 SC 的具体差异"。
