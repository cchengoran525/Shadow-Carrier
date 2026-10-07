# HIDE 亲和度打分：文献地图 + 逛街场景扩展（2026-10-07）

> 对应 `docs/V2_CHASSIS_GIMBAL_DESIGN.md` §13「雷达 × HIDE 亲和度打分场」与
> `shadow_carrier_on_rockchip/world/fusion/sector_score.py`（唯一实现，三方消费：
> geometry_daemon 实机 / Webots 仿真 / hri_state 经 grid.json）。
> 本文回答两件事：①这套手写 social costmap 在文献里的位置与可引先例；②把同一套
> 打分哲学迁移到逛街/商场场景时的打分点清单。

---

## 1. 设计哲学的文献定位

当前 `sector_score.py` 的三条哲学，每条都有成熟理论对应：

| 设计哲学 | 打分项（现状） | 理论对应 |
|---|---|---|
| 低打扰（不占动线/交互预测） | 门锥、家电前方、热点连线=路中间 | Goffman"互动秩序"、行人动力学、活动区研究 |
| 安全（不可预测 + 物理风险） | 门（可能有人进出）、离墙过近 | 门槛行为、建筑净空规范 |
| 不引人注意（civil inattention） | 背墙/墙角加成、自由扇区 | Goffman"文明性不注意"、Hall sociopetal/sociofugal |

**这等于一个手工设计的 social costmap**，且打分层是"手写规则"而非学习得到的——
这是方法论选择（可解释、可冻结、可调），不是能力缺陷（见 §1.4 研究增量）。

## 2. 文献地图

标注说明：✅=本次已联网核实出处；△=凭记忆，写正式材料前建议再核对。

### 2.1 计算先例（related work 主干）

- ✅ **Sisbot, Marin-Urias, Alami, Siméon, "A Human Aware Mobile Robot Motion Planner",
  IEEE Transactions on Robotics 23(5):874–883, 2007**（被引 700+，LAAS-CNRS）。
  最先把 **visibility grid**（避免突然出现在人的视野死角/身后——"可见性代价"）和
  accessibility 代价放进运动规划器（HAMP）。本项目的 `wall_back` 加成与 V2 §13.1 规划的
  "可见性遮挡分析"在这里有算法定义先例，**是打分场最直接的祖宗**。
- ✅ **Mead & Matarić, "Autonomous human–robot proxemics: socially aware navigation based
  on interaction potential", Autonomous Robots, 2016**（DOI 10.1007/s10514-016-9572-2）。
  **"interaction potential"（交互势能）与本项目"交互亲和度"是同题反号**：他们算哪里
  适合发起交互，我们算哪里适合隐身。论文里对照写成"反向势场"最顺。
- ✅ **Rios-Martinez, Spalanzani, Laugier, "From Proxemics Theory to Socially-Aware
  Navigation: A Survey", International Journal of Social Robotics 7(2):137–153, 2015**。
  社会导航综述的标准引文。**注意期刊是 IJSR，不是常被误写的 IJRR。**
- △ Lu, Hershberger, Smart, "Layered Costmaps for Context-Sensitive Navigation",
  IROS 2014——nav2 layered costmap 架构源头；本项目的 `W` 字典 = 每层一套手写权重。
  加上 ROS nav2 social navigation layer 作开源对照（V2 §13.5 已提）。
- △ Helbing & Molnár, "Social force model for pedestrian dynamics", Physical Review E
  51(5), 1995——行人流自发成道，"路中间惩罚"的物理先例。
- △ Brock et al., "Children and robots learning to play hide and seek", HRI 2006；
  Georgia Tech Wagner & Arkin 的机器人欺骗/躲藏系列——"机器人躲藏行为"本身的文献。
- △ Meeussen et al., "Autonomous Door Opening and Plugging In with a Personal Robot",
  ICRA 2010——门作为一等动态对象的机器人处理先例（对应门状态规则）。

### 2.2 HRI 实证 / 民族志（"人实际希望机器人在哪"）

- ✅ **Forlizzi & DiSalvo, "Service Robots in the Domestic Environment: A Study of the
  Roomba Vacuum in the Home", HRI 2006**——家用机器人民族志开山作，记录家庭给扫地机
  划的禁区与干扰抱怨，是"家庭/宿舍物件打分"的实证原型。
- ✅ **Kuno, Sadazuka, Kawashima, Yamazaki, Yamazaki, Kuzuoka, "Museum Guide Robot Based
  on Sociological Interaction Analysis", CHI 2007**——直接用社会学互动分析（视频民族志）
  决定机器人站位。**是本项目"人类学 → 手写规则"方法论的同类先例**（注意：CHI 2007，
  不是 HRI 2008）。
- △ Dautenhahn et al., "How May I Serve You? A Robot Companion Approaching a Seated
  Human Person", HRI 2006——接近距离/方向偏好（对应 GOTO_SAFE 接近侧）。
- △ Mumm & Mutlu, "Human-Robot Proxemics: Physical Distance in Human-Robot Interaction",
  HRI 2011——注视对人际距离的影响（对应"朝向锥"规则）。
- △ Takayama & Pantofaru, "Influences on Proxemic Behaviors in Human-Robot Interaction",
  IROS 2009——养宠物者 proxemics 差异（宠物区规则的锚）。
- △ Walters et al., 2009 proxemics 实证框架（被引 200+）。
- ✅ **Eresha, Häring, Endrass, André, Obaid, "Investigating the Influence of Culture on
  Proxemic Behaviors for Humanoid Robots", RO-MAN 2013**——跨文化近距离学（阿拉伯 vs
  德国）；另有 Joosse et al. 2014 文化与接近距离。**接 §13.5"权重因文化/场景而异"的
  用户研究校准。**

### 2.3 人类学 / 社会学理论（每条规则的"为什么"，全是书，引用零风险）

- **Hall《The Hidden Dimension》(1966)**——proxemic 分区（亲密/个人/社交/公共）+
  **sociopetal / sociofugal**（鼓励/抑制交往的空间布置）——"路中间 vs 贴墙角"的理论版。
- **Goffman《Behavior in Public Places》(1963)**——**civil inattention（陌生人之间
  刻意不看、不打扰）是 HIDE 状态的规范性基础**；"territories of the self"（身体周边
  索求区）对应门/桌边惩罚。写论文时 HIDE 动机段落引它最合适。
- **Altman《The Environment and Social Behavior》(1975)**——领地性三分类（primary/
  secondary/public territory）：冰箱 = secondary territory（半公共但有人认领），
  走廊 = public territory。
- **Kendon《Conducting Interaction》(1990)**——F-formation：桌椅组合构成互动单元
  （o-space），对应桌/椅惩罚环。
- **Hillier & Hanson《The Social Logic of Space》(1984)**——空间句法 integration/
  segregation 给"穿行空间 vs 停驻空间"提供可计算的连续量，是"路中间"的一般化。
- **Alexander 等《A Pattern Language》(1977)**——Alcoves、Window Place 等现成模式，
  堪称"手写空间规则库"，与本方法哲学完全同构。
- △ 博物馆观众研究（Falk & Dierking 一系）反复发现参观者沿边缘走、避开开阔中心——
  "贴墙边"的人类实证锚。

## 3. 逛街场景打分点探索

### 3.1 场景设定与宿舍场景的差异

主人逛街（商场/商业街/超市），车跟随；主人驻足看商品 → HIDE。相比宿舍走廊：

1. **规则被放大并密集化**：宿舍的 1 扇门 → 商场的店门×N + 扶梯口 + 直梯口 + 防火门；
   饮水机 → 收银台/试衣间/食档；路中间 → 主通道；墙角 → 柱背/店铺间凹进（niche）。
   **规则不变，权重表换成"商场包"。**
2. **YOLO-World 开放词汇是关键杠杆**：类别表 = 手写规则的接口面。宿舍包与商场包 =
   两份类别→权重表，`score_map` 代码零改动。
3. **时段性极强**：高峰/平峰人流、营业/打烊（卷帘门 = 门状态规则的时段版）、临时
   促销/舞台 = 地图上突然出现的动态热点（"停留热图"规则在商场价值最大）。

### 3.2 打分点清单（按三类哲学分组）

**A. 低打扰 / 交互预测类**

| 物件 | 打分逻辑 | 权重直觉 | 传感器 |
|---|---|---|---|
| 收银台 | 高频交互区，商场版饮水机 | 强负（≈ -1.0） | YOLO-World |
| 试衣间/母婴室/卫生间门口 | 隐私 | **硬禁区 mask** | YOLO-World |
| 促销堆头/发传单点 | 人驻留聚集 + 会被搭话 | 中负 | YOLO-World |
| 排队队列（奶茶/收银） | 队尾延伸方向是动线 | 强负 | YOLO-World |
| 中岛 kiosk | 四面都是动线 = "路中间"的四面版 | 强负 | YOLO-World |
| 食档/香水柜台前 | 排队 + 嗅觉/驻留 | 中负 | YOLO-World |
| 玩具店/儿童乐园区 | 儿童乱跑不可预测 | 中负 + 行为降速 | YOLO-World |
| 长椅/休息区 | 人驻留预测（人即将出现） | 中负 | YOLO-World |
| 柱背/店铺间凹进 | refuge 加成（商场柱网 = 周期性藏点） | 正 | 相机/雷达 |

**B. 安全 / 硬禁区类**

| 物件 | 打分逻辑 | 权重直觉 | 传感器 |
|---|---|---|---|
| 扶梯上下口 | 商场铁律"扶梯口禁停" + 车上不去 | **硬禁区 mask** | YOLO-World |
| 直梯口 | 人流 generator（=门规则） | 强负 | YOLO-World |
| 防火门/安全出口 | 逃生通道必须畅通（占消防通道违规） | **硬禁区 mask** | door 检测 |
| 玻璃栏杆/幕墙 | 雷达盲区（V2 §13.4）+ 视觉脆弱 | 中负（相机检测） | 相机 |
| 店铺玻璃门 | 门规则 + 玻璃穿透（雷达看不见） | 相机为主 | 相机 |

**C. 动线 / 结构类**

| 物件 | 打分逻辑 | 权重直觉 | 传感器 |
|---|---|---|---|
| 主通道中央带 | 两排店铺 = 热点，连线规则**直接复用** | 中负 | 滚动地图 |
| 货架间窄道 | 瓶颈宽度 < 机器人宽+余量 → mask | 硬约束 | 雷达（V2 §13.1） |
| 中庭开阔区 | 人流汇合（V2 §13.1 已有"开阔区惩罚"） | 中负 | 雷达/滚动地图 |
| 购物车回收处 | 车流动线 | 中负 | YOLO-World |
| 打烊卷帘门 | 门状态"closed"的时段版 | 时段门规则 | 相机 |

**核心洞察**：宿舍场景的每个规则在商场都有同构体——规则框架可迁移，变的是
类别表和权重（这正是 §13.5"哪些层重要、权重因文化/场景而异"的用户研究素材）。

## 4. 规则代码落地记录（2026-10-07）

**改哪**：`world/fusion/sector_score.py` 单文件（三方消费方的唯一实现，
`regression.py`/`selftest_polar.py` 不覆盖它，模块自带 `__main__` 自测）。

**增量接口**（新字段缺席时与旧版行为逐字节一致，demo 冻结期安全）：

| 接口 | 类型 | 语义 |
|---|---|---|
| `CLASS_W` | dict cls→权重 | 手写类别权重表（YOLO-World 开放词汇直接喂字符串）：驻留预测/易碎/商场类 |
| `HARD_MASK_CLASSES` | set | 硬禁区（toilet/扶梯/灭火器…）：负分不够，直接剔除出 best 候选，安全=约束、社交=分数 |
| `DOOR_STATE_W` | dict state→权重 | 门状态化：ajar(-2.2) > closed(-1.6) > open(-1.0)——打分对象是**不可预测性**而非门本身 |
| `persons[].facing_deg` | 可选字段 | 朝向锥：人面向扇区负分（gaze），人背后扇区正分（blind，直径相对扇区） |
| `hour_bucket` | 可选字段 | 时段桶（default/peak/night）：负权重放大系数；纯函数不取系统时间，由调用方传入 |

**未做（留待雷达/联调）**：停留热图（geometry_map 累积人驻留点）、瓶颈宽度、
可见性遮挡——见 V2 §13.1/§13.3。
