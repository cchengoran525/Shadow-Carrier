# 全仓代码优化专项审查（2026-10-25）

> 审查基准：本地 main @ `e7f68f0`（工作区干净，与 origin/main 同步）。
> 方法：6 模块分片——3 片由并行子代理逐行深读（主控/跟随链、视觉/HRI 链、state+Webots 仿真），
> 3 片由主审亲读（Arduino 固件四套工程、world/fusion 几何栈、pc_track/deploy/测试脚本/仓库卫生）；
> 代理提出的 17 条高价值声明由主审逐条定向复核，**16 条证实、1 条部分证伪**（§7.3）。
> 性质：**只读优化专项**，未修改任何源代码（本报告文件除外）。与 `CODE_REVIEW_2026-10-24.md`
> （正确性 P0/P1 + 文档矛盾审计）互补；该报告已列问题此处只核对现状（§7.2），不重复展开。
> 标记：〔✅亲验〕= 主审亲自读到代码确认；〔⭕代理〕= 代理给出 file:line、主审已复核机制成立但未逐行复读全文。
> 时机标签：**demo前**（10 月底展示前的小改，低风险）/ **V2雷达前**（11 月雷达上车前）/ **demo后**（清账类，冻结期不动）。

---

## 0. 结论速览

| 章节 | 数量 | 最值得做的一条 |
|---|---|---|
| §1 demo 前小改提案 | 8 | imu.json 每拍最多 5 次 open+JSON 解析（≈50 次/秒），单拍缓存即可收回 |
| §2 热路径性能 | 21 | 全链路 ~46 次/秒短连接 HTTP 轮询打一个 11FPS 数据源（gimbal 33Hz 占大头） |
| §3 算法/复杂度 | 3 | merge_collinear 最坏 O(n³)（已有缓解，附顺手微优化） |
| §4 重复代码/结构 | 8 组 | `grab_mjpeg_frame` 全仓 6 份拷贝 |
| §5 死代码/死参数 | 6 组 | BACK_SPD 假调参键——面板"可调"但代码零引用 |
| §6 卫生/部署 | 6 | deploy.sh 8×scp 全量同步（`__pycache__` 也会被拷上板） |
| §7 附带正确性（新） | 5 | **SIM_TURN=0 已关不掉转向通道**，passby A/B off 臂失效 |

---

## 1. demo 前小改提案清单（待统筹批准后执行）

> 沿用 10-24 §7.2 约束：每条独立 commit；不并发改 `rk_control.py`/`follow_controller.py`（T4 期间是否仍冻结由统筹判断，#1/#2/#3 涉及这两个文件）；**不推 GitHub**；改后跑回归 + 板端 smoke。

| # | 项 | 位置 | 改法摘要 | 验收 | 风险 |
|---|---|---|---|---|---|
| 1 | shm 公共读取器 + 单拍缓存 | follow_controller.py:431-444/521/524/957/979/985；rk_control.py:76-122/534-586 | tick 开头取一次 yaw_rate/yaw_deg 存缓存，`_yaw_rate/_rate_world/_info` 全部消费缓存；6 处 "open+json.load+年龄判定" 模板抽公共 `read_shm_json()`（imu=monotonic、owner/hri=墙钟的契约集中成注释） | test_follow_dt + test_control_glue 绿；新增断言：同拍内 `_info` 记录值与控制用值一致 | 低（数值更一致，无行为变化） |
| 2 | BACK_SPD 假调参键摘除 | rk_control.py:1202（`_FC_KEYS`）；follow_controller.py:81-82/218/735-739 | 从 `_FC_KEYS` 删 BACK_SPD；删 BACK_MS + backing/back_done 死分支（或注明"预留"） | `/motion/config` 回显不再含 BACK_SPD；test_control_glue 绿 | 零 |
| 3 | 10Hz 主循环改截止期调度 | rk_control.py:773 | `next_t += CYCLE_S; sleep(max(0, next_t - monotonic()))`，`_hri_bridge_loop` 同法 | 日志时间戳间隔 100±5ms（tick 正常时）；环率 8.3-9.5Hz → 10Hz | 低 |
| 4 | gimbal_follow 轮询降频 | gimbal_follow.py:35-36 | POLL_S 0.03→0.09（或按响应 `frames` 版本号门控）；SEND_HZ 删除或改 11 并注明受上游钳制 | API 请求率 33→11Hz；云台跟随无肉眼退化 | 低（延迟上限 +60ms） |
| 5 | uart_reader 删空读双等待 | rk_control.py:433-434 | 删 `else: sleep(0.05)`（termios VTIME=1 已阻塞 0.1s，:226） | OBS 行到达→obs_state 更新延迟 ≤0.1s | 零（一行删除） |
| 6 | 仿真句柄缓存 + 无头模式 + grid 按需写 | shadow_carrier.py:570-581/564/207/220-223（句柄）；:96-101（enable）；:515（非原子写） | `__init__` 缓存 DEF 节点与 translation 字段句柄，值未变早退；`geom_det=True` 时跳过相机/识别/声呐 enable；grid 内容指纹不变跳写 + tmp+replace | run_robustness 回归通过；fast 模式 RTF 提升（数字待测） | 低 |
| 7 | geometry_daemon 发布块合一 + 原子写 | geometry_daemon.py:355-362/409-416/440-448 | 三个模式的"发布快照"块（map 更新/save/summary/score_map/写 grid.json/topdown）抽 `publish_snapshot()`；once/scan 走 tmp+replace | py_compile 过；once 模式产物与旧版 diff 仅 ts 字段 | 低 |
| 8 | video_stream_fast 隔离 + 旧版弃用横幅 | scripts/video_stream_fast.py；scripts/video_stream.py:1-12 | fast 移 `legacy/` 或删除（与现行 daemon 协议已不兼容，§7.3-6）；video_stream.py 头部加弃用横幅 | 误启动旧脚本时有显眼警告 | 零 |

**demo 前不做**：固件任何改动（pulseIn/String/GOT，§2.4——需烧录，一律 demo 后）；雷达核内部优化（§2.5，随 V2 雷达批次）；大结构收敛（§4）。

---

## 2. 热路径性能（21 项）

### 2.1 主控链（10Hz 控制环）

**1｜imu/owner/hri 三类 shm 文件重复读**〔✅亲验〕
- 现状：`_yaw_rate()`（follow_controller.py:431-438）每次调用都走 `yaw_rate_fn()` → `rk_control._get_yaw_rate()`（rk_control.py:76-88）完整 open+json.load `/dev/shm/imu.json`，无缓存。一拍内调用点：:521（E1 门）+:524（`_rate_world`）+:957（弧线闭环直连）+:979（`_info`）+:985（`_rate_world`）＝**最多 5 读/拍 ≈ 50 次/秒**。同拍多次采样还会取到 50Hz 序列中相邻的不同样本——E1 门与归算用的可能不是同一时刻的 yaw_rate，黑匣子记录与实际决策对不上。环外 `_turn_worker`、`_hri_execute` goto 又是独立 10Hz 读。
- 另：hri_state.json 被 follow 模式下两线程各 10Hz 读（`_hri_parked` :760、`_hri_action` :693）≈20 读/s；owner.json 被 dist_fn 与 bridge 循环合计 ~20 读/s——而写侧限频 0.2s（follow_controller.py:154），**50% 的读必然返回同一内容**。
- 建议：提案 #1（单拍缓存 + 公共读取器）。仓库里已有现成范式：`_safety_env` 的 mtime+0.5s 缓存（rk_control.py:313-329）。
- 收益：~90 次/s open+parse → ~30 次/s；syscall 抖动下降；同拍数值一致。
- 时机：**demo前**。

**2｜10Hz 固定 sleep**〔✅亲验〕
- 现状：`_follow_loop` 每拍 `tick(); time.sleep(0.1)`（rk_control.py:773）——周期 = tick 耗时 + 0.1s。tick 含检测 HTTP（1-3ms，抖动时至 1s）+ 帧解码 + 模板匹配，典型 5-12ms → 实际环率 8.3~9.5Hz 且波动。
- 建议：提案 #3（截止期调度）。
- 时机：**demo前**。

**3｜follow_status.json 每拍原子重写**〔⭕代理〕
- 现状：每拍 3 syscall（open/write/rename）×10Hz（rk_control.py:720-739，调用点 :769），内容大部分拍不变。
- 建议：改"状态或命令变化必写 + 0.5s 心跳写"两档；前提核消费者（记录器/面板）判活阈值 ≥0.8s。
- 时机：**demo后**（demo 期保留现状最稳）。

### 2.2 视觉链（11FPS 检测 + 3 个消费方）

**4｜gimbal_follow 33Hz 轮询 11FPS 源**〔✅亲验〕
- 现状：`POLL_S=0.03`（gimbal_follow.py:36）主循环每 30ms 调 `fetch_biggest_person()` → 每次新建 TCP + 服务端全量 dumps + 线程创建；≥2/3 请求拿到同一帧。`SEND_HZ=20`（:35）是被 11FPS 源钳制的**不可达参数**。
- 建议：提案 #4。
- 时机：**demo前**。

**5｜API 无 keep-alive，全链 ~46 连接/秒**〔✅亲验（protocol_version 未设）＋⭕代理（连接数口径）〕
- 现状：Handler 未设 `protocol_version`（video_stream_v7.py:30/:409），默认 HTTP/1.0 每请求即断连。消费方合计：gimbal 33Hz + follow 10Hz + hri 3.3Hz + weak_cue 按相位 ≈ 46+/s，每次完整握手 + ThreadingMixIn 线程创建销毁。
- 建议：`Handler.protocol_version = "HTTP/1.1"`（响应已带 Content-Length）；消费方长连接可选。
- 收益：~46 次连接建立/秒归零（估 1-3% CPU）+ 消除稳态 TIME_WAIT 积累。
- 时机：**demo后**（需实测浏览器 MJPEG 分帧；demo 前可承受）。

**6｜/api/detections 每请求全量 dumps**〔⭕代理〕
- 现状：~46 请求/秒每次在锁内对完整 dict `json.dumps`（video_stream_v7.py:375-384），相邻请求间内容多数未变（数据 11FPS < 请求 46Hz）。
- 建议：按 `frame_count` 缓存序列化 bytes，实时字段（age_ms 等）100ms 粒度单独拼。
- 时机：**demo后**（与 #5 一并做）。

**7｜yolo_daemon 无 viewer 时仍画框+编码+写盘**〔⭕代理〕
- 现状：daemon 每帧无条件画框循环 + 整帧 JPEG 写 /dev/shm（yolo_daemon.cc:96-111），Python 侧每帧读回 ~30KB（video_stream_v7.py:331-342）。浏览器无 /stream 连接时全速空转，估 5-10ms/帧。
- 建议：stdin 协议加可视化标记或环境变量开关，Python 侧统计 /stream 活跃数。
- 时机：**demo后**（需同步改两端 + 重编译 daemon）。

**8｜owner_id 同一 tick 双拉同一 API**〔⭕代理〕
- 现状：`_fetch_detections`（层 0 过滤）与 `_fetch_raw_persons`（raw）各自 urlopen 同一 URL（owner_id.py:168-194/:397-417）；`weak_cue` 不传 dets 时（rk_control.py:453-460 正是这么调）同一周期再拉第三次。注意接口陷阱：若把**过滤后**结果传入 weak_cue 的 dets 参数，low_conf 通道恒空。
- 建议：owner_id 内 50-80ms TTL 单飞缓存，或调用方拉一次 raw 分发。
- 时机：**demo后**。

**9｜全图 cvtColor 但只用 ROI/半带**〔⭕代理〕
- 现状：`select_target` 整幅 640×480 BGR→HSV（owner_id.py:378）但只对候选框切 ROI；`find_leg_band_info` 全图转换（:273）只取下半带（:275）；enroll 同（:338）。person 框典型仅占全图 1/12。
- 建议：ROI 先切后转（数学等价）。
- 收益：~2-4ms/拍（10Hz 下 ≈3% 单核）。
- 时机：**demo后**（若 demo 前跟车 CPU 吃紧可提前）。

**10｜follow 每拍全帧 JPEG 解码且不按帧复用**〔⭕代理〕
- 现状：`_fetch_person` 每拍 `cv2.imread` 完整解码（调用点 follow_controller.py:353 → owner_id.py:316-325），检测端 10FPS 与控制环 10Hz 不同步时同一帧被连续两拍重复解码+模板匹配（RK3566 上 3-6ms/拍）。
- 建议：以 detections 的 `frames` 号为 key 缓存"已解码帧 + select_target 结果"；V2 更优形态：检测进程直接下发 HSV ROI 特征，消灭环内解码。
- 时机：**demo后**（接口变更，与 #8 统一设计）。

### 2.3 仿真（125Hz，worlds/shadow_carrier.wbt:8 basicTimeStep=8ms）

**11｜字符串句柄每步重复查找**〔✅亲验〕
- 现状：`_prop_boxes()` 每次 3 次 `getFromDef`（shadow_carrier.py:570-581），被声呐（:651）与遮挡判定 `_seg_blocked`（:598，geom_detections 每步最多 3 目标）合计每步 ~4 次调用 ≈ **2100+ 次/秒** Python↔C 字符串查找；`_set_person/_set_object/_synthetic_box` 绕开构造器已缓存的 translation 字段句柄（:106/:112 vs :207/:220-223/:564）。
- 建议：提案 #6（__init__ 缓存 `{DEF: node}` 与字段句柄；SFVec3f 值未变早退）。
- 收益：每步削 ~17 次跨边界调用；压低步耗时长尾——realtime 模式下单步超 8ms 即造成墙钟/仿真钟滑差（已知 passby 4.5× 抖动的放大器）。
- 时机：**demo前**。

**12｜geom_det 模式仍 125Hz 启用相机/识别/声呐**〔⭕代理〕
- 现状：autotest 默认 geom_det=True 时 `_detections()` 直接返回几何真值（:785-787），相机渲染+识别唯一用途是日志里的 n_rec（:1343）；声呐设备从无读者（:100-101 enable，全文件无 getValue）。
- 建议：提案 #6（geom_det 时跳过 enable，n_rec 记 None；fx_px 只用 getWidth/getFov 不需 enable）。
- 时机：**demo前**。

**13｜检测/幻影 125Hz 计算但消费 3.3Hz**〔⭕代理〕
- 现状：`_detections()`/`_phantoms_update()` 在步循环逐拍执行（:1260-1261），真实消费者在 0.3s HRI tick（:1291）与 1s 日志（:1362）；唯一每步消费者是 GT_TURN 的 WAIT 假动作分支（:1264-1267），它只需要"主人是否可见"一个布尔。
- 建议：检测计算移入 HRI tick；WAIT 分支改单目标判空；`_phantoms_update` 同步移入（并修 §7.1-② 的拍率语义）。
- 收益：检测侧 Python 工作量降约 97%（125→3.3Hz）。
- 时机：**demo前**（至少移 `_phantoms_update`）；完整版若时间紧放 demo 后。

**14｜owner-lock 框先投影后节流，86% 白算**〔⭕代理〕
- 现状：`_publish_owner_lock` 每步无条件投影（:816-821→:1252），而 `_publish_owner_json` 第一行 0.2s 节流（:519-525）——每秒 125 次投影约 100 次丢弃；`_follow_person`（:823-832）又独立重算同一个框。
- 建议：节流提到投影之前；拍内共享框缓存（注意保留 `_owner_lock_box` 与 `_geom_detections` 的遮挡判定语义差）。
- 时机：**demo前**。

**15｜_virtual_scan 180 射线纯 Python**〔⭕代理〕
- 现状：10Hz 一圈 180 射线 × ~10 AABB 双层循环（:665-693），boxes 列表每次重建。
- 建议：numpy 向量化 slab 相交；`:626-628` 的距离修正历史说明这是安全相关路径，重写需保留对拍断言。
- 时机：**demo后**。

### 2.4 固件（loop 时序预算；全部需烧录 → 一律 demo 后）

**16｜pulseIn 阻塞 15% loop**〔✅亲验〕
- 现状：`pulseIn(ECHO, HIGH, 12000)` 每 80ms 阻塞 12ms（UltrasonicSensor.cpp:82，Config.h:36-37）——期间 `motor.update()` 不跑，斜坡/超时/超声波保护全部停摆，电机保持最后 PWM。
- 建议：中断沿计时（attachInterrupt 或 PCNT/RMT），loop 零阻塞。
- 时机：**demo后**（固件批）。

**17｜String 堆分配解析链**〔✅亲验〕
- 现状：每命令行 ~6-10 次堆分配：`handleGimbalCommand` 两次 substring（.ino:103-104）+ `cmdBuf.trim()`（:143）；CommandParser 整行拷贝 + trim + toUpperCase（CommandParser.cpp:36-38）+ nextToken×2-4（:5-20）+ `substring(1).toInt()`×2（:60-61）。长时运行堆碎片风险。
- 建议：零分配 char-buffer 解析（命令已限 48 字符）。
- 时机：**demo后**（固件批）。

**18｜GOT 回显**〔✅亲验〕
- 现状：每命令行回显 `GOT:<cmd>`（.ino:145-146）——调试残留，占 CDC 流量、扩大阻塞打印暴露面（已知 Serial.print 满缓冲会卡 loop）。
- 建议：debug 开关门控（编译期或运行期）。
- 时机：**demo后**（固件批）。

### 2.5 雷达核（V2 上车前顺手批）

**19｜clearance_dir 循环内常量三角 + 4 扫可合一**〔✅亲验〕
- 现状：`along = y * math.cos(th) + x * math.sin(th)`（lidar_guard.py:333-334）——th 是每方向常量，cos/sin 却在**每点每方向**循环内调用；4 方向（0/90/180/270，:477-478）各全量扫一遍 points，投影本可由 (x,y) 符号翻转一次算出。
- 建议：cos/sin 提出循环；单遍计算 4 方向投影（0/180 与 90/270 互为符号翻转）。
- 收益：每帧省 4×N×2 次三角调用（N=1000 时 ≈2-4ms），4 扫 → 1 扫。
- 时机：**V2雷达前**。

**20｜_hysteresis 每帧每轴重算常量 gap_required**〔✅亲验〕
- 现状：`want_clear_gap = gap_required(v_nom) + resume_margin`（:442）在 Guard 生命周期内是常量，却每帧每轴（5 轴）重算。
- 建议：`__init__` 预计算一次。
- 时机：**V2雷达前**（顺手）。

**21｜apply_to_ascii cfg=None 每命令重建默认字典**〔✅亲验〕
- 现状：`cfg = cfg or _load_defaults()`（:634）——雷达上线后每条运动命令 10Hz 走这里，每次重建 40 键 dict。
- 建议：调用方（rk_control `_safety_filter`）持 Guard 的 cfg 传入，或模块级缓存默认值。
- 时机：**V2雷达前**（随雷达接线一并）。

---

## 3. 算法/复杂度（3 项）

**22｜door_v3.merge_collinear 最坏 O(n³)**〔✅亲验〕
- 现状：while-changed × O(n²) 配对（door_v3.py:46-94），HoughLinesP 典型 n=50-200，每 pass 2-5ms、2-4 pass。已有缓解：geometry_daemon 自适应门间隔（>1.5s 跳帧，geometry_daemon.py:435-439）。
- 顺手微优化：`detect_segments` 的全图 `np.median(gray)`（:29，640×480 ≈ 数 ms）改 256 桶直方图中位数（~0.5ms）。
- 时机：**demo后**（当前 1Hz 节拍下不致命）。

**23｜save_map.py per-cell 纯 Python 循环**〔✅亲验〕
- 现状：`to_pgm_values` 逐格判断（save_map.py:25-36），大图 4M 格 ~2-4s；`n_occ/n_free` 再扫两遍（:66-67）。
- 建议：numpy 或 bytes.translate；一次性 PC 工具，低优先。
- 时机：**demo后**。

**24｜仿真 _virtual_scan / of_bridge cmd_eval**〔⭕代理/✅亲验〕
- `_virtual_scan` 见 §2-15；of_bridge `cmd_eval` 一次性把全部帧读进内存（of_bridge.py:190-193，60s@15fps ≈ 277MB）——PC 端可接受，注明即可。
- 时机：**demo后**。

---

## 4. 重复代码与结构收敛（8 组）

| # | 组 | 现状（验证方式） | 建议 | 时机 |
|---|---|---|---|---|
| 25 | `world/` vs `world/fusion/` 三对文件 | door_v3.py / params.py / polar_mapper.py **逐字节相同**（cmp 确认）；params.py 头部自述"冻结不许再动"——双拷贝恰是误改单边的高危形态 | world/ 侧改 `from fusion.door_v3 import *` 式 re-export | demo后 |
| 26 | 三套 C3 固件 8 共享文件 | 内容**仍逐字节一致**（`diff --strip-trailing-cr` 确认）；差异只是 DistributedRobot_C3_MotionController 工作区被 IDE 存成 CRLF（git 索引均 LF） | 单一源 + 构建变体（共享 lib 目录/符号开关）；顺手统一行尾 | demo后（固件批） |
| 27 | `grab_mjpeg_frame` | **6 份拷贝**：auto_calib_pose.py:49 / fusion_test.py:31 / geometry_daemon.py:88 / solve_desk_pose.py:34 / validate_desk.py:32 / of_bridge.py:33（变体）；`fetch_detections` 3 份 | 抽 fusion 公共 `stream_client.py` | demo后 |
| 28 | shm "open+json.load+年龄判定" 模板 | 6 处近似实现（rk_control.py:17-25/76-88/110-122/534-586） | 公共 `read_shm_json()`（并入提案 #1） | demo前（随 #1） |
| 29 | geometry_daemon 三模式发布块 | once:355-362 / scan:409-416 / loop:440-448 复制粘贴——恰是 tmp+replace 只存在于 loop 的不一致根源 | `publish_snapshot()`（提案 #7） | demo前（随 #7） |
| 30 | follow_controller tick() 结构 | ~250 行混合编排（:724-972），腿跟块 4 处 `_leg_*` 清理复制粘贴（:805-807/818-820/827-829/834-836）；:743-748 两个连续 `if person is not None` 应合并 | 拆 `_leg_follow_tick`/`_arc_tick` + `_leg_reset()` | demo后 |
| 31 | state/ 六组平行实现 | a) decoder↔gaze 去畸变逐行同构（decoder.py:26-41 vs gaze.py:49-56）；b) follow_controller 内 508.2/508.0/492.7 三种内参字面量散落 4 处（:420/:481/:775/:898，`state/calib/params.py` 就在 sys.path 上）；c) calib 三脚本相机打开/棋盘检测四份重复（PATTERN=(9,6) 定义四处）；d) pan_sweep/solve_geom 两套手写 LM 拟合骨架；e) owner.json 发布双份镜像（shadow_carrier.py:519-534 vs follow_controller.py:157-172，A1.2 契约漂移即事故）；f) fx 两算（shadow_carrier.py:99 vs :385-386） | V2 逐项收敛；b（内参统一 import）与 f 是已知 CX/FCX 分歧问题的维护面收缩，行为等价可先行 | b、f demo前顺手；其余 demo后 |
| 32 | 仿真-实机平行实现 | `_execute_ascii`（shadow_carrier.py:859-904）与固件 CommandParser 平行维护，协议语义漂移无守护 | 结构化 fast-path（`("DIFF", l, r)` 元组直通）或 golden 用例对拍 | demo后 |

---

## 5. 死代码/死参数清单（6 组）

**33｜假调参键（调了没反应，最危险的一类）**
- `BACK_SPD`（follow_controller.py:81）：本文件零引用，却被 `/_motion/config` 列为可调（rk_control.py:1202 `_FC_KEYS`）——现场调它毫无效果但被持久化并回显。〔✅亲验〕
- `SEND_HZ`（gimbal_follow.py:35）：被 11FPS 源钳制，永不构成实际限制。〔✅亲验〕
- 时机：**demo前**（提案 #2/#4）。

**34｜lidar_guard 死代码与假配置**〔✅亲验〕
- `read_safety`（:749-761）：全仓零调用方（grep 确认）。
- `grid_sensors`（:740-746）：只有 test_lidar_guard.py 调用，未接生产（文档声明的 V2b 待办）。
- `hour_bucket`（sector_score.py:108）：无生产者，PENALTY_SCALE 恒 1.0。
- `TRACK_MAX_MISS`（:98）：定义即死（Tracker.update :418 用硬编码 3）。
- **配置键旁路**：configs/lidar_guard.json 的 `track_assoc_deg/track_assoc_m/dynamic_mps` 被 Tracker 硬编码 10.0/0.6/0.15 旁路（:399-401/:417）——改配置静默无效，**雷达装车调参前必须接线**。
- 时机：接线 **V2雷达前**；删除 demo后。

**35｜follow_controller/rk_control 死代码**〔⭕代理，关键项亲验〕
- TARGET_BBOX_H/MIN_BBOX_H/FWD_SPD 仍无引用（:44-46）；TARGET_BBOX_H=400 与 STOP_BBOX_H=400 同值异义。
- `run_follow_loop`（:991-994）全仓零引用，且其 sleep 调度是旧式非截止期——留着会被人当范例抄。
- rk_control `except TypeError` 版本 shim 永不触发（:766-767）；版本横幅 v6（:2）vs v5（:1686）漂移仍在。
- imu_daemon `temp_c` 恒 None（:179）+ REG_TEMP 死常量。
- `DET_API` 三文件各抄一份（follow_controller.py:150 / owner_id.py:7 / pivot_calib.py:34）。
- 时机：**demo后**（删行级）。

**36｜几何栈杂项**〔✅亲验〕
- door_v3 `bw` 死变量 + 混淆表达式（:124 `bot - top and dx or 1`）。
- draw_topdown：`pts` 计算后未用（geometry_daemon.py:319）、P() 内 `* math.cos(...) * 0` 死项（:289-293，10-24 §2.1 已知未动）。
- of_bridge `of_beats_hold_ratio` 重复键（:306/:309，10-24 已知未动）。
- `FREE_DENS_MAX=0.06` 在 geometry_daemon.py:35 与 params.py FREE_DENSITY_MAX 双源。
- 时机：**demo后**。

**37｜state/ 杂项**〔⭕代理〕
- pan_sweep `lo, hi = sorted([fx, 0])` 未用（:118）；tracker.py `r_var_scale` 半死 API（:69/:77 两个调用方都不传——按检测置信度给 r 方差缩放正是其设计意图，接线或删二选一）；bearing_test.py:15 `sys.path.insert(0, "../calib")` 指向不存在目录；test_tracker_fake.py:84 "(≤3)" 文案 vs :88 判据 3.5 漂移。
- 时机：**demo后**。

**38｜三代 video_stream + S3 网关遗留**
- 旧 `video_stream.py` 每帧 `subprocess.run` 重载模型（:75-79）——误启动会吃满 CPU；`video_stream_fast.py` 与现行 daemon 协议不兼容（§7.3-6）已不可用；v7 与 fast 的 viewer HTML 逐字重复。〔✅亲验（fast）/⭕代理（HTML 重复）〕
- DistributedRobot_S3_Gateway 整工程为退役遗留（`buildControlPage` 每次 GET 重建 3200 字符 String，WebServerHandlers.cpp:12-64；`requireActiveHandlers` 只打印不阻断的空模式 :66-70）。〔✅亲验〕
- 建议：fast/旧版移 `legacy/` 加横幅（提案 #8）；S3 目录头部加"已退役，勿烧录"声明。
- 时机：**demo前**（#8）；S3 声明 demo后。

---

## 6. 仓库卫生/部署（6 项）

**39｜deploy.sh 同步方式**〔✅亲验〕
- 现状：8 次 `scp -r` 全量（每目录一次 SSH 握手、无增量、磁盘上的 `__pycache__` 会一起上板）；`echo kickpi | sudo -S` 密码硬编码（:37）；仍同步 docs-only 的 `communication/`（:21-22）；仍不同步 `perception/`（板上 yolo 二进制与仓库源码对应关系无记录）；`StrictHostKeyChecking=no`。10-24 已修的 deploy/configs/world 同步确认在位（:27-34）。
- 建议：`rsync -az --exclude __pycache__` 单次增量；perception/ 纳入或注明"板外维护"。
- 时机：**demo后**（部署面）。

**40｜跟踪的 jsonl 数据文件**〔✅亲验（git ls-files）〕
- docs/pose_offering_test×2、scripts/fake_script.jsonl、world/fusion/pose_pn×3（共 ~110KB）——标定/测试数据不是垃圾，但混在代码目录里。建议归 data/ 或各目录 README 注明用途。
- 时机：**demo后**。

**41｜pc_track 脚本健壮性**〔✅亲验〕
- first_map.sh 无 trap——Ctrl+C 会孤儿化 slam/bag/TF 三个后台进程（first_map.sh:57-97）；restart_offline_stack.sh 依赖不在仓库的 `/tmp/tfstatic_qos.yaml`（:29，换机不可复现，同 C3 udev 规则问题的形态）。
- 建议：加 `trap 'kill ...' INT TERM`；QoS 文件入仓。
- 时机：**demo后**（PC 科研轨）。

**42｜watchdog 日志无轮转**〔✅亲验〕
- /var/log/c3_watchdog.log 只增不减（事件驱动频率低，低优先）；对比 camera_usb_fix 用 journald（logger）是正解。
- 时机：**demo后**。

**43｜缺失的部署工件**（已知名项，仍在）
- rk-control.service 单元与 C3 udev 规则（303a:1001）不在仓库——换板不可复现、无规则的板上 watchdog 会循环 usbreset。
- 时机：**demo后**（V2 换板前必须）。

**44｜杂项**
- DistributedRobot_C3_MotionController 工作区 CRLF（索引 LF）——`.gitattributes` 已有 eol=lf，重 checkout 或编辑器配置即可归一。〔✅亲验〕
- c3_watchdog `usbreset` 二进制无存在性检查（缺则静默失败进日志）。〔✅亲验〕

---

## 7. 附带发现的正确性问题

### 7.1 新发现（5 条，请统筹裁决归属）

**①〔✅亲验〕SIM_TURN=0 已无法关闭转向通道——passby A/B 的 off 臂失效**
- `shadow_carrier.py:69-70` 只在 `TURN_ON` 为真时 `_fc_mod.TURN_ENABLED = True`，**从不设 False**；而 follow_controller.py:93 默认已是 True（10-20 T4 启用）。
- 后果：`run_passby.sh` 的 off 臂（SIM_TURN=0）与 on 臂行为完全相同，仅日志标签不同；10-16 的历史日志有效，**今日重跑 A/B 即拿到假对照**。
- 修法一行：`_fc_mod.TURN_ENABLED = bool(TURN_ON)`。建议 demo 前修（任何人重跑 A/B 都会中招）。

**②〔⭕代理，机制清晰〕幻影注入强度被步率放大 ~120×**
- `_phantoms_update`（shadow_carrier.py:727-744）在 125Hz 步循环逐拍掷骰（:1261），docstring 的"拍"按 0.3s 检测拍设计（rate=0.35/拍、寿命 3-10 拍 ≈ 0.9-3s）——实际变为每秒 ~44 次抽取、寿命 24-80ms，幻影从"间歇出现"变成"常态 ~2 个并发"。
- 后果：phantom 场景的 `phantom_adoption_risks` 结论是在被夸大的强度下测出的。
- 修法：随 §2-13 移入 HRI tick，或按 `rate * dt / 0.3` 缩放。

**③〔✅亲验〕imu_daemon 重零期间积分缺口**
- dt 在循环顶采样（imu_daemon.py:149-151），重零 `calibrate2(0.8)` 阻塞 0.8s 后的陀螺样本仍按旧的小 dt 积分（:176 `yaw = yaw * YAW_LEAK + yaw_rate * dt`）——50°/s 转动下重零瞬间丢 ~40° 航向（已知"calibrate2 阻塞"问题的量化增量：不止数据过期，还有恢复后航向阶跳）。
- 修法：cal 结束后重采 t_prev，或阻塞期弃一拍。

**④〔⭕代理 + Python 语义推演〕"PAN nan" 打满限位**
- 仿真 `clamp(float("nan"), 0, 180)` 经 Python min/max 的 nan 比较序落到 180°（shadow_carrier.py:906-907）；固件侧 `toFloat("nan")` → nan 钳制全跳过 → `static_cast<uint32_t>(nan)` 是 UB duty（C3_USB_Controller.ino:85-94 同类洞）。
- 修法：两侧各加 `isfinite` 判断。

**⑤〔⭕代理〕/turn TOCTOU 双 worker**
- `_turn_start` 的 `turn_state.get("active")`（rk_control.py:198-199）无锁读，与 worker 持锁置位（:158-160）之间有窗口——并发双 POST /turn 可产生两个 worker 抢底盘。检查与置位应同入 turn_lock。

### 7.2 已知名项现状核对（10-24 报告的优化面/关键项）

| 项 | 现状 | 证据 |
|---|---|---|
| lidar_guard epoch/monotonic 时钟域 | **已修**（1f337b5 属实） | lidar_guard.py:543-554 双时钟兼容 + 注释引 10-24 实测 |
| geometry ring≥3 IndexError | **已修** | geometry_daemon.py:132-146 stamp_objects 越界跳过 |
| geometry_map EWMA 跨缝 | **已修** | geometry_map.py:31-38 `_ewma_bearing` |
| deploy.sh 同步范围 | **已补** deploy/configs/world | deploy.sh:27-34；但 communication/ 仍在列表、perception/ 仍缺（§6-39） |
| hri_state 出画断流 / 帧原子写 / _low_s / /move 门禁 / OBS 兜底 | 已修（10-24 §6/§16 已录，本轮抽验在位） | — |
| rot 迟滞恢复阈值错用线性包络尺度 | **仍在** | lidar_guard.py:440-456（want_clear_gap 全轴同用） |
| ±180 缝裸角度差 | **仍在**（dedup :228 / cluster :285-286 裸差；spike :243 用了 _ang_diff——前后不一致） | lidar_guard.py |
| --replay 不传 now | **仍在** | lidar_guard_daemon.py:138 |
| free_directions 0..315 不 wrap | **仍在** | geometry_daemon.py:262-263；geometry_map.py:172 |
| read_safety / grid_sensors / hour_bucket | 仍死 / 未接线（§5-34） | — |
| once/scan 非原子写 grid.json | **仍在**（loop 已是 tmp+replace） | geometry_daemon.py:359/:413 |
| world/ 三对重复文件 | **仍在**（cmp SAME） | §4-25 |
| 三套固件 8 文件一致 | 内容仍一致；工作区行尾漂移（新） | §4-26 |
| 固件正确性批（DIFF 超声/fail-open/限位/阻塞打印） | 全部未动（符合 demo 冻结约束） | — |
| rk_control 死 fd 无重连 | **仍在**：uart_open 只在启动调一次（:1687-1689），横幅 "will retry on udev" 无对应实现 | — |
| 版本横幅 v6/v5 | **仍在** | rk_control.py:2 vs :1686 |
| 仿真-实机 BASE_SPD/FOV/注入分歧 | 仍在，且 **BASE_SPD 分歧数值漂移为 240 vs 245**（实机 10-21 重载档后仿真注释"实机不动"已失效） | shadow_carrier.py:22-23 vs follow_controller.py:40 |
| hri_state RECEIVE owner-None 死分支 | **仍在**（行号漂移至 :514-516） | — |

### 7.3 证伪记录（1 条，防误修）

| 代理声明 | 事实 | 依据 |
|---|---|---|
| "video_stream_fast 会与 daemon 互相卡死（stderr 管道 64KB 写满阻塞）" | **不成立**：fast 以 `stderr=DEVNULL` 起 daemon（video_stream_fast.py:27），stderr 写入直进 /dev/null 永不阻塞。正确结论是较弱但同样致命的：fast 从 stdout 读 JSON（:33-34）而 daemon 的 stdout 已被重定向到 /dev/null（yolo_daemon.cc:43-47）→ readline 恒 EOF → 恒返回 `{"error":"parse"}`、推流黑屏——**脚本已完全不可用**，处置并入提案 #8 | 亲读双方源码 |

---

## 8. 建议执行顺序

**demo 前（本报告 §1 的 8 条提案，待统筹批准）**——均为几行级局部改动，逐条独立 commit + 回归。
另建议把 §7.1-①（SIM_TURN 一行修）并入 demo 前批：不修则任何 A/B 重跑都是假对照。

**V2 雷达上车前（必须）**
- lidar_guard Tracker 配置键接线（§5-34——否则装车调参全是假象）；
- §2.5 三项热路径微优化（随雷达接线顺手）；
- rot 迟滞、时钟域、±180 缝等 10-24 已列雷达前置项（仍未修）。

**demo 后清账**
- 死代码批量删除（§5）；三对重复文件 re-export 与六组平行实现收敛（§4）；deploy.sh 改 rsync + perception 对账（§6）；三代 video_stream/S3 归档（§5-38）；固件单一源 + pulseIn/String/GOT 批（§2.4）。

---

*审查覆盖：scripts 全部（rk_control/follow_controller/owner_id/hri_state/imu_daemon/video_stream×3/gimbal/passby×2/pivot_calib/test×13/deploy.sh/health_check.sh）、world/fusion 全部（含新增 ros_scan_bridge.py）+ world/ 顶层、state/ 全部 + calib、simulation/webots（shadow_carrier.py 1375 行 + analyze_robustness + run_*.sh）、perception、四套 Arduino 工程（diff 验证）、pc_track、deploy、tools、.gitignore/.zcodeignore。全程只读，除本报告外未创建/修改任何文件。本 commit 不推送 GitHub。*
