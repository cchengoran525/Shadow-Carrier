# 全仓代码审查（2026-10-24）

> 审查基准：本地 main @ `7be364b`（fetch 因网络超时失败；本地与最后一次 fetch 的 origin/main 一致，网络恢复后可再 `git pull` 核对）。
> 方法：5 路并行分片审查（实机主链路 / world-fusion 几何栈 / Webots 仿真 / ESP32 固件 / 文档交叉审计），
> 其中 10 项高优先级发现经第二遍亲读源码逐行验证；**2 条代理报告的 P0 被证伪**（见 §4）。
> 标记说明：〔✅亲验〕= 已由主审复读源码确认；〔⭕代理〕= 代理给出 file:line 证据、可信但未逐行复读。
> 今天处于 demo 冻结期（10 月底展示），V2 十一月启动——优先级建议按此背景给出。

---

## 0. 结论速览

| 类别 | 数量 | 最危险的一条 |
|---|---|---|
| 会影响运行（P0/P1） | 18 | `geometry_daemon` 见 3m 外物体即 IndexError，世界快照整轮失败〔✅亲验〕 |
| 可优化（P2） | ~25 | 三份逐字节重复的 door_v3/polar_mapper/params 双拷贝 |
| 文档矛盾/优先级未定 | 21 | "V2a/V2b/V2c"代号在两份排期文档里指不同东西，11 月开工即撞车 |
| 证伪误报 | 2 | "固件不支持 PAN/TLT / 不发 OBS 遥测"均不成立 |

**demo 前建议立即处理（小改动）**：§1.1 的 ①②③（/move 门禁、_low_s 累加、uart 失败告警）。
**V2 雷达上车前必须处理**：§1.3 的 ①②（ring 越界、rot 迟滞锁死）。

---

## 1. 会影响运行的问题（P0/P1）

### 1.1 实机主链路（rk_control / follow_controller / hri_state / owner_id）

**①〔✅亲验〕串口断连后零重连，"will retry on udev" 是空头承诺，运动命令静默失败**
- `rk_control.py:848` 启动时 `uart_open` 只调用一次；`:850` 打印 "will retry on udev" 但全文件无任何 udev 监听/重连代码（grep 验证）。
- `uart_send`（:233-239）`except: return False` 裸吞一切写错误——C3 重新枚举后旧 fd 已死，所有命令静默失败，`/ping` 仍报 uart:true（只判 fd 非 None），跟随循环照跑但车不动。
- 外部看门狗救不了：`deploy/c3-watchdog/c3_watchdog.sh` 只在 "usbreset 后 symlink 仍缺失" 时重启服务；C3 自行恢复（symlink 重现）时判健康、不重启 rk-control → 死 fd 永不恢复。
- 缓解：固件 450ms 命令超时保证命令断流后车自停，不会失控——表现为"车僵死且无告警"。
- 附：`uart_reader`（:360-383）`while uart_fd is not None`，初始打开失败时线程立即退出。

**②〔✅亲验〕`/move` 端点绕过跟随模式门禁，底盘可被抢控**
- `rk_control.py:624-635` 的 `/move` 直接发 `MOVE`，没有 `_get_mode() != "manual"` 检查；而 `/forward` `/back` `/left` `/right` 在 :688-694 都有"跟随时锁死"门禁。跟随中任何热点内设备 POST `/move` 即可与 10Hz 跟随 DIFF 抢底盘。

**③〔✅亲验〕"连续低分清跟踪器"逃逸机制是死代码——错锁到路人将无限跟随**
- `follow_controller.py:342` 判据 `self._low_s >= LOW_STREAK_S(3.2s)`，但 `_low_s` 全文件只有 :181 初始化 0.0 和 :345 清零，**从未累加**（grep 验证）。逃逸永不触发；`low_streak` 本身也只在"无候选"分支累加（:339），有低分候选时不计。

**④〔✅亲验〕HRI 停驻域发布在主人出画时断流 → 停驻的车会倒退（潜伏，HRI 实机串跑即爆发）**
- `hri_state.py:314-322`：owner is None 提前 return，`_publish_state` 只在 :506（owner 在场）调用。WAIT/HIDE（parked=True）时主人出画是常态（±32° FOV），`/dev/shm/hri_state.json` 0.5s 过期 → `rk_control._hri_parked`（:16-24）返回 False → 退回完整跟随 → person=None 走丢失流程 → 0.4s 后 `MOVE B` 后退搜索（follow_controller.py:727-730）。**本应原地停驻等人的车会倒退**。
- 现状：hri_state 未常驻实机（COORDINATION_REVIEW.md §1），故未爆发。

**⑤〔⭕代理，双源交叉确认〕HRI 动作层在实机没有执行器**
- `hri_state.py:111-113` `send_cmd_fn` 默认 no-op；`run_live`（:551）构造状态机时不传 send_cmd → APPROACH/BACK_OFF/GOTO_SAFE 只进日志和 json，永远到不了底盘。动作消费只存在于仿真（`shadow_carrier.py:919-1010`）。
- 影响：仿真验证过的 HIDE 躲避 / RECEIVE 交接全链路在实机物理上不发生；demo 若含这两镜，需明确"仿真已验证、实机暂缓"或先接执行器。

**⑥〔⭕代理〕NEAR_STOP_M 真实距离急停分支数学上不可达**
- `follow_controller.py:790` 依赖 `range_from_box.ground_distance_m`，该函数脚出画（y_b≥479）返回 None（range_from_box.py:58-60），而 dist<0.7m 需要 y_b>734（超出 480 帧高）。即 <1.58m 永远返回 None，"首选真实距离"的近距急停永不触发；近距保护实际全靠框高兜底 + 速度门（30° 偏轴时仍保留 ~55% 速度）。demo 擦身镜头的真实风险点（与 DEFERRED_WORK §3"近距余量收口封存到 V2"一致，但注释与能力不符应改口）。

**⑦〔⭕代理〕杂项运行风险**
- `/dev/shm/yolo_frame_a/b.jpg` 非原子写（video_stream_v7.py:242 直接 imwrite），读写窗口重叠时 imread 得 None，约 5%/帧丢帧（owner_id.py:306-315 消费）；yolo_daemon 自己的 out.jpg 已改 tmp+rename，同一教训没应用到这里。
- `owner_id._fetch_detections` API 失败返回 `[]`（:174-178）与"无 person"不可区分：yolo daemon 重启期间 follow 视为"主人消失"走丢失流程（0.4s 后后退）。
- `follow_controller._fetch_person` 异常时 `self.profile = None`（:365-369）：一次瞬时 cv2 异常即**永久**丧失认主模板，降级 lost_stop 直到手动干预。
- imu 重零 `calibrate2(0.8)`（imu_daemon.py:157）阻塞主循环超 0.5s 新鲜度窗，`_start_follow` 时跟随头 1 秒无航向闭环。
- `hri_state.py:485-488` RECEIVE 内 owner-None 分支不可达（:314 已提前 return）——死代码。
- HTTP 面板 0.0.0.0:80 无鉴权（仅 WPA2 热点护体），与 ② 叠加放大。

### 1.2 固件（C3_USB_Controller 及三套副本）

**①〔✅亲验〕DIFF 差速前进没有即时超声波停车，仅 MOVE F 有——与 README 安全承诺矛盾**
- `MotorDriver.cpp:39` `update()` 中 `obstacleDetected_ && targetMode_ == Forward` 才立即 stop；`MotorDriver.cpp:150-153` `differential()` 入口虽拒绝新命令，但障碍在**两条命令之间**出现时（10Hz 下 ~100ms 窗口，主机卡顿时最长 450ms 由命令超时兜底），Diff 模式车轮继续按斜坡逼近。跟随主路径正是 DIFF 流。20cm 阈值 + 450ms@跟随速度越界足以接触。
- 根 README.md:286 承诺 "Forward movement is blocked **as soon as**…"——`DIFF L200 R200` 是前进运动但不被立即拦截。
- 修法一行：障碍停车条件扩为 `Forward || (Diff && 左右目标和 > 0)`。

**②〔⭕代理〕超声波"强制解除"使传感器故障 fail-open，且与两份文档直接矛盾**
- `UltrasonicSensor.cpp:40-49` 连续 5 次无回波强清障碍锁；`:52-57` 闭锁超 5s 强清。传感器断线后保护静默永久失效（`OBS 0 -1` 遥测有发出但 rk_control 未对 cm==-1 告警）。
- 矛盾：arduino_ide_setup.md:111 写 "invalid readings **do not** clear the block"，固件行为恰好相反；根 README "clears **only** after 3 valid samples" 的 "only" 不成立（同文档 :214 又自己承认 force-clear）。20-25cm 滞回带内不刷新闭锁时间戳，障碍持续 21-24cm 时 5s 后被强制放行。
- 需产品决策：fail-open（现状）还是 fail-safe。

**③〔⭕代理〕舵机安全限位(15-165°)+斜坡只存在宿主侧，固件层缺失**
- 固件 `servoSet` 只钳 0..180、脉宽 500–2500µs（.ino:35-36, 85-88），上电/换角无斜坡；15-165°/斜坡仅在 rk_control.py:31-32,319-354。`gimbal/gimbal_follow.py:31-32,132-133` 仍钳 0..180 且直发串口，绕开限位——手工串口或 gimbal_follow 发 PAN 0 可复现"顶死堵转→C3 USB 掉线"事故链（commit 6e6e809 自认 "firmware batch TODO" 未落地）。
- 畸形 PAN 行（`PAN abc`/小写/无参）从 `handleGimbalCommand` 落回运动解析器 → Invalid → `stop()`（.ino:106-108 + CommandParser.cpp:72-73）：云台命令错误与底盘运动耦合，一条被污染的 PAN 行即打断底盘行进。

**④〔⭕代理〕450ms 停车等全部软件安全网依赖 loop() 存活，而 loop 会被 Serial.print 阻塞**
- USB CDC TX 缓冲满时 `Serial.print` 阻塞 loop，期间 `motor.update()` 不跑 → 超时/超声波/斜坡全部停摆，电机保持最后 PWM。该死锁项目史上实际发生过（rockchip README:151-153），现行缓解纯靠宿主排空线程；固件未启用任何硬件看门狗。建议启用 TWDT 或改非阻塞写。

**⑤〔⭕代理〕副本与死代码**
- 三套 C3 固件的 8 个共享文件（CommandParser/Config/MotorDriver/Protocol/UltrasonicSensor）**逐字节一致**，安全参数无漂移（450ms/20cm-25cm×3/斜坡/STBY 均相同）；漂移集中在入口层：超长行处理三种行为（现役 .ino:152-154 静默截断后"执行前缀"最危险——被污染行会以合法前缀身份执行；MotionUart.cpp:53-56 清空+日志最合理）。
- wifi 副本断连期间 `delay(3000); return;` 完全跳过 motor.update()（掉线时车可失控 ≥3s）——已标"冻结勿烧录"但缺陷未在注释中标注。
- `tools/check_firmware_sync.sh` 只守卫 wifi 副本，DistributedRobot_C3_MotionController 不在守卫范围。
- `shadow_carrier_on_rockchip.bak/` 18 个 tracked 文件纯归档；`communication/` 代码已在 ec9f079（2026-08-22）删光只剩 README，两处 README 仍称其为"重复副本"已过期。
- `MotorDriver.cpp:159-163` Diff 残留速度：`DIFF L200 R200 → MOVE F 100 → DIFF L80` 时左轮瞬跳 200 再斜坡降，绕过缓启动。

### 1.3 世界几何栈（world/fusion）

**①〔✅亲验〕任何 >3.0m 的检测物体直接 IndexError 崩掉整轮世界快照——当前最可能现场炸的点**
- `polar_mapper.py:64-73` `polar_to_sector` 对 dist>3.0m 返回 `ring=3`（docstring 自述 "3=超远"）；`geometry_daemon.py:221` grid 只建 3 行，`:244` `grid[o["ring"]][o["sector"]]` → `grid[3]` 越界。跟人场景 3m 外检出目标极常见：once/scan 模式整次快照失败，loop 模式变每轮 "cycle error"，grid.json 停更、HIDE 断粮且只有一行日志。
- 修一行：`ring>=3` 跳过，或 grid 预建第 4 行。

**②〔✅亲验〕转向迟滞恢复阈值错用线性包络尺度 → rot 一次触发后近乎永久锁死**
- `lidar_guard.py:442` `_hysteresis` 对所有轴统一用 `want_clear_gap = gap_required(v_nom)+0.15 ≈ 0.74m`；rot 轴自身尺度只有 `rot_gap_full_m=0.20`（:71-72）。实测复现：0.27m 触发 blocked[rot] 后，障碍撤到 0.55m 持续 3s 仍锁死，需最近障碍 ≥0.97m 才恢复。室内最近障碍常 <1m → **V2 一开雷达，一次贴身事件后转向功能锁死**（wz 置 0 → DIFF 差速归零只能直行、MOVE L/R 永久 STOP）。56 条断言没有覆盖 rot-block 恢复。
- 修法：rot 轴恢复线改用 `rot_gap_full_m + margin ≈ 0.35m` 量级。

**③〔✅亲验〕lidar_guard 时钟域混用：文档规定的 epoch ts 让 guard 级 stale 判定在生产链路是死路**
- `lidar_guard.py:460` now=monotonic、`:543` `now >= ts` 才算帧龄；V2_LIDAR_AVOIDANCE.md:221 §5.1 规定生产者写 epoch（≈1.79e9）→ `now < ts` 恒成立 → scan_age=None → stale 判定与 env.mode 上报死路。现靠 rk_control 外层 wall-clock 兜底（:284-286 用 time.time()），行为等价但 `/safety` 会把"帧龄 3s"报成 ok=true，契约本身错。
- 附：`lidar_guard_daemon.py:138` `--replay` 不传 now，快速回放时 0.25s 确认窗不可达，回放结论与线上不一致。

**④〔⭕代理〕geometry_map EWMA 方位平均跨 ±180 缝不 wrap，门会瞬移到反方向**
- `geometry_map.py:90`（门）`:115`（物体）线性平均；关联判定用了 `_wrap180` 更新却没有。实测：172°/-172° 两次更新后 bearing=34.4°——正后方的门跳到右前方，直接污染 sector_score 选点与 hri_state。

**⑤〔⭕代理〕实机 grid.json 存在左右镜像风险（需一次实机实测裁决）**
- `polar_mapper.py:5,49-61` 声称"右正左负"，但 bearing 由 `y ∝ (u-CX)` 直接给出无镜像处理；本仓权威审计 `test_bearing_convention.py:5-8`（2026-10-14）记录"图像 u 增大 = 物理左（2026-08-27 实测）"，`state/gaze.py:75` 同。若相机仍是镜像款，整张 grid.json 左右镜像，且现有 validate_desk 的成对距离验证方法学上检不出镜像。**建议立即做一次实机左右放手实测**；若已换非镜像相机，更新 test_bearing_convention.py 并关闭此条。

**⑥〔⭕代理〕其余 P2 级运行边角**
- `geometry_daemon.py:345/399`（once/scan）直接 `open("w")` 写 grid.json，loop 模式却是 tmp+replace——scan 恰是 HIDE 高频触发模式，hri_state 周期读可能读到半截 JSON。
- `lidar_guard.py:399/401/417` 跟踪关联门与 dynamic 阈值硬编码，无视 configs/lidar_guard.json 里的 `track_assoc_deg/track_assoc_m/dynamic_mps`——改配置静默无效。
- ±180 缝两侧同角去重/聚类用裸角度差（:228, :285-286），正后方目标可能重复/假拆（spike 滤波 :243 用了 `_ang_diff` 是对的，前后不一致）。
- `solve_desk_pose.py:121-124` / `validate_desk.py:101-105` 全部角点不落地时除零+越界；`lidar_guard_daemon.py:147` 空 replay 文件 IndexError；`auto_calib_pose.py:45` gimbal() 无 try。
- `geometry_daemon.py:248/389` free_directions 输出 0..315 不 wrap，而 objects bearing 是 ±180——下游 `hri_state.py:105` `min(free, key=abs)` 把 315°（实为 -45° 近前）当"最远"，自由方向排序错误。
- 部署路径双标准：hri_state.py:43 默认 `~/world_lab/fusion/grid.json`、health_check.py:27 用 `/home/kickpi/...`，而 geometry_daemon 写自己源码目录——仓内运行与板上布局两套。
- sector_score 的 `hour_bucket`（时段桶）无人喂数——文档 §4 宣称的能力实际未启用。
- `lidar_guard.py:739-751` read_safety 全仓无调用方且与自身 docstring 矛盾；`grid_sensors`（sensors.lidar 合入）未接线（文档已声明的 V2b 待办，此处记录现状）。

**四机制承诺核对结果**：惰性契约 ✅、单一收口 ✅、故障放行 ✅、过期限速 ✅（但 guard 内层 stale 死路见 ③）；grid.json 消费方零改动 ✅；相机-only 完整出图 ✅。文档承诺与代码基本兑现，问题在上面列的 bug 里。

### 1.4 仿真（Webots）与实机分歧

**仿真-实机已知分歧清单**（影响"仿真绿灯"对实机的转移保证）：
- 距离环：仿真用地面真值刹车 `_follow_cmd`（shadow_carrier.py:1018-1031，<2.6m 改 STOP），实机的 NEAR_STOP_M/距离变速/地面几何测距在仿真全部未跑（follower 未注入 dist_fn）。
- 超声否决：仿真构造 FollowController 未传 obs_fn/weak_cue_fn（:187-193）——受阻否决、腿带弱线索在仿真不生效；passby 防吸路人门（_lock_confirmed）仿真也整体绕过，Webots passby 测的是"没装这道门"的版本（离线 test_passby_lock_confirm.py 有覆盖）。
- 声呐语义：仿真每帧瞬时判 ≤20cm、无滞回无锁存（:1277-1278）；固件 20cm 触发+25cm×3 恢复+5s 强解——边界抖动时行为分叉；后退 DIFF 实机会被拦、仿真放行。
- BASE_SPD：仿真进程内 240 vs 实机 220（shadow_carrier.py:23 vs follow_controller.py:35）。
- FOV 常量：裁剪用真机 tan=0.63（±32.2°），world 相机 fieldOfView 1.0（±28.65°）——差 ~7°（shadow_carrier.wbt:325 vs :63）。
- HRI：仿真强制 owner.source="file"，实机默认 "none"（conf 回退）；仿真喂 robot_moving、实机 run_live 不喂（hri_state.py:556）——自运动门控实机永远不触发。
- 帧率：实机 10Hz tick vs 仿真 ~0.3s tick，帧计数型确认（LOCK_CONFIRM_N=3）在仿真等效耗时 ×3。

**仿真内部 bug**：
- 〔⭕代理〕`_reset_scene` 重建 FollowController 丢 `bearing_of_fn`（:1210-1216 只传 3 个注入，初始 :187-193 是 4 个）→ 按 R 复位后转向通道 θ 带 +3.1°/×1.15 系统偏差。
- 〔⭕代理〕并行会话冲突残留：grid 快照与 owner.json 写 `$TMPDIR` 固定文件名无开关（:143,146），`_autotest.wbt` 多脚本共用；SIM_LOG 已在 runner 层修复但 `run_autotest.sh`/`run_demo.sh`/全部 .bat 未设；grid 写非原子（:515）。
- 〔⭕代理〕时基混用（控制拍墙钟 vs 剧本仿真钟）是 passby 抖动 4.5× 的根源，README 自己记录"单跑不能当门禁"；analyze_robustness.py:61 用墙钟 t 卡仿真钟窗口 [6,12]，窗口错位；无 fast 模式防呆。
- 〔⭕代理〕杂项：sonar 设备 enable 后从不读（死代码）；`_run_scenario.bat` 不拷世界直接跑陈旧 `_autotest.wbt`；analyze_robustness 空参数输出 "0/0 PASS" 退出码 0；`logs/rob_* (1).jsonl` 手工重名文件被通配双计；offline_door_test v1/v2 把 report 写进输入图片目录；test_heading_hold.py 硬编码 /home/kickpi 路径；README "8/8 PASS" 与 "baseline FAIL 未决" 同节并存（10-18 修复后无回归记录）。

### 1.5 转向通道（TURN_ENABLED=False 潜伏群，冻结期不动、解封前必修）

- 〔⭕代理〕符号链未闭合：`decoder.py` 模块头 :6 说"u>CX 为正"，:20-22/:37 又说"正值=物理左侧"，自相矛盾；`gaze.py:113` 断言画面镜像；仿真 test_turn_pivot 验证的是仿真自己的几何。`pivot_calib.py:114-125` 留"θ 左右符号人工核对"恰说明真机符号未经机器验证。符号错则 TURN/SEARCH 朝背离人的方向转。
- 〔⭕代理〕主点不一致：decoder CX=292.7 vs follow FCX=320 → θ 通道 ~3.1° 系统偏差。
- 〔⭕代理〕SEARCH 相无墙钟上限：总量上限 SEARCH_MAX_DEG=180° 只靠 IMU 积分累计，IMU 失效时 `_acc_turn_deg` 永不增长 → 无限原地旋转（BLIND 相有时间兜底，SEARCH 没有）。
- PolarCVKF：`follow_controller._kf_feed`（:439-463）已实例化并喂数，但整条链在 TURN_ENABLED=False 后面，当前生产路径确实只有 state/test_tracker_fake.py 自测在用（与"未接线"记忆吻合，属半接线状态）。

---

## 2. 可优化点（P2）

### 2.1 死代码与重复（建议清理/合并）
- `world/door_v3.py ≡ world/fusion/door_v3.py`、`world/params.py ≡ fusion/params.py`、`world/polar_mapper.py ≡ fusion/polar_mapper.py` 三对**逐字节相同**（diff 验证），params.py 头部写"冻结不许再动"——双拷贝恰是误改单边的高危形态。建议 world/ 侧改 re-export。
- `config/settings.yaml` 全项目无代码读取，内容三处失真（device_id:10/1920x1080、ttyUSB0+binary 协议、死 PID 参数）；真实部署目录是 `configs/`，README 架构树只列 config/。
- 死参数：follow_controller.py:39-41 TARGET_BBOX_H/MIN_BBOX_H/FWD_SPD 无引用；GAZE_ENABLED=False 下 `_gaze_loop/_gaze_drive_loop` 整块冻结；owner_id.py:88 `BAND_MAC.replace(":", ":")` no-op；draw_topdown 的 P() 死项（geometry_daemon.py:278,305）；of_bridge.py:306/309 重复键。
- 旧版 `video_stream.py` 检测键 `class/confidence` 与 v7 链的 `c/p` 不兼容——误起旧脚本时 owner 静默空 person 池无报错，建议加弃用横幅。
- `.bak` 目录、`communication/`（docs-only）见 §1.2⑤。

### 2.2 配置漂移与部署
- 部署硬编码：owner_id.py:41,51 实体 MAC；video_stream.py:11 `192.168.137.190`；pan_sweep.py:34 camera 10；hri_params.json `~/world_lab/...`。
- **仓库缺 rk-control.service 与 C3 的 udev 规则**：watchdog/health_check/deploy.sh 都依赖它们但都不在仓库（camera 的 udev 规则有，C3 的 303a:1001 没有）——换板无法复现；无该规则的板上 watchdog 会每 ~9s usbreset C3 并循环重启 rk-control。
- `deploy.sh` 只同步 scripts/gimbal/communication/state/C3_USB_Controller，**不同步 deploy/、configs/、world/**——看门狗与配置更新到不了板子。
- 版本号漂移：rk_control.py:2 文件头 "v6" vs :847 横幅 "v5"；README 自启表 4 项 vs health_check 检查 6 项（imu、c3-watchdog 缺登）。

### 2.3 性能（量级均不致命）
- 同一份 `/api/detections` 被 3 个消费者独立轮询+独立解析+三套键名（follow 10Hz/HRI 3.3Hz/gaze 冻结）；API RTT 计入 10Hz 控制环，daemon 重启时 tick 可阻塞至 1s（450ms 超时兜底停车，表现为跟随顿挫）。HRI 0.3s 轮询只消费 1/3 帧，帧计数判据实际时间窗比注释长 3 倍——可消费 v7 已透传的 ts 做事件对齐。
- 固件 `pulseIn` 12ms 超时每 80ms 阻塞 loop 一次（~15% 时间）——可改中断沿计时。
- door_v3.merge_collinear 最坏近 O(n³)（已有自适应 interval 缓解）；lidar_guard 每帧 4 次线性扫可预计算投影。

### 2.4 健壮性小项
- `hri_state.run_live` 每会话新 jsonl 无轮转，`~/hri_logs` 无限增长。
- `_turn_worker`（rk_control.py:164）无锁读 turn_state；`/turn/status` 与 worker 并发增键可能 RuntimeError（仅该请求 500）。
- rk_control 版本横幅、README 已知问题清单（:206-215）与现状部分对齐但 stale。

---

## 3. 矛盾与优先级未定点

### 3.1 文档间直接矛盾（影响采购与排期）

| # | 矛盾 | 双方证据 | 后果 |
|---|---|---|---|
| 1 | **"V2a/b/c" 代号两义**：底盘相位 vs 手机导航里程碑 | V2V3_ROADMAP.md:33-56（V2a=底盘）vs V2_PHONE_RELATIVE_NAV.md:200-206 + DEFERRED_WORK.md:122（V2a=相对估计） | 11 月开工两个"V2a"撞车；建议 PHONE_NAV 改称 PN-1..4 |
| 2 | **雷达 L1 归 V2b 还是 V2c** | V2_CHASSIS §0 红线 :33（V2c）vs V2V3_ROADMAP.md:40-56（V2b） | 装车排期不确定 |
| 3 | **绕行型分离归 V2 还是 V3** | DEFERRED_WORK.md:43,151（V2，解封后并列第一）vs V2V3_ROADMAP.md:53,89（V3b） | V2 主脊口径相反 |
| 4 | **电源方案五版并存、两处互斥**："XL4015 装不装"无终局 | §21.19（:1681-1693 全 3S+XL4015）vs §21.22/§21.32（:1732-1737, :1931 不装、三包分带）；§19.1 总图/§21.2 采购清单（:1391 "XL4015 必买"）仍是一包一宝 | 下单者看不同节买不同的东西 |
| 5 | **云台舵机三态**：STS3215 必买 vs ZP10S 够用 vs 继续冻结 | §21.7 采购表 :1443（STS3215 必买）vs 未提交 §21.34（:1947-1982，ZP10S 合格且更省）vs V2V3_ROADMAP.md:47（预算允许或冻结） | 采购清单未随最新判定更新 |
| 6 | **相机同日两节结论相反**：OV2735 不推荐 vs 可以买 | CAMERA_SELECTION.md §7.6（:212-226）vs §7.9（:262-276），无修订声明 | 买不买无终局；且 V2_CHASSIS §15.6 的 65cm 视野账按 120° 算，推荐款 96-98° 需重算 |
| 7 | **雷达选型/高度旧结论残留**：C1 vs X3 Pro、38cm vs 13.5cm vs "抬到 25-30cm" | §10.6:581 预算表 C1 vs §10.5.1:535 决议 X3 Pro；§13:711 38cm vs §15.7:881 13.5cm vs V2_LIDAR_FOLLOW_UPGRADE.md:159 抬高建议 | 若抬高，lidar_guard 全部安装几何与 configs 需重算（未拍板） |
| 8 | **权威源元规则打架** | ARCHITECTURE.md:139-149（PACKAGING 排期降为背景）vs PACKAGING_PLAN.md:320（冲突以本文档为准） | 两份元规则均未随对方更新 |
| 9 | **Pre v0 "已上线" vs "未实现"**（套磁话术冲突） | DEFERRED_WORK.md:32 + RESEARCH_POSITIONING.md:307（已上线）vs PACKAGING_PLAN.md:306（未实现） | 现场向老师介绍会自相矛盾 |
| 10 | **GAZE_ENABLED 常数表与三份裁决相反** | gimbal/README.md:117（True）vs DEMO_PLAN.md:51-53 / V2_LIDAR_AVOIDANCE.md:51 / COORDINATION_REVIEW.md:75（False） | 照常数表开凝视会引入已裁定的扰动源 |
| 11 | **demo 状态无 10-19 之后的快照** | DEMO_PLAN.md:36-39（A1.3"冻结前可完成"已逾期 10 天）vs FIELD_TRIAL_PLAYBOOK.md:64,180（未验）vs COORDINATION_REVIEW.md:80-81 | 分镜 3（HIDE）/7（APPROACH）完成状态不明 |
| 12 | baseline "8/8 PASS" vs "FAIL 未决"双记载，10-18 修复无回归记录 | webots README :51 vs :77-85；DEMO_PLAN.md:66 记已修；V2_LIDAR_AVOIDANCE.md:315-320 第三种归因 | demo 前无法判断剧本是否全链通过 |
| 13 | 冻结铁律 vs 冻结期后功能改动（转向通道 v5、BASE_SPD 130→220 等持续到 10-19） | REVIEW:110,205 + DEFERRED_WORK:13 vs 实际 commit 记录 | 计划文档未改口，拍摄排期按旧冻结点倒排 |
| 14 | 11 月-1 月三份时间账不对齐（PACKAGING 12 月整机 vs V2V3 12 月中旬还在装传感器） | PACKAGING_PLAN.md:269-278 vs V2V3_ROADMAP.md:31-56 vs REVIEW:183-197 | 三轨排期互相矛盾 |

### 3.2 代码层设计牵制（互相影响但未定优先级）

| # | 牵制点 | 细节 |
|---|---|---|
| 1 | **MOVE L/R 双语义** | `move_lr_is_rotation` 默认 True（lidar_guard.py:81）：V2 麦轮落地时若忘置 false，横移会错误套用转向净空判定——应与 V2 底盘版本绑定进检查单 |
| 2 | **主人=障碍的裕量太薄** | follow 停车线 0.7m vs 雷达 fwd 介入 ~0.59m 仅差 11cm；主人迎面走（closing>0）时提前量持续压 vx_max，与"雷达不卡跟随"承诺余量很薄——实测协议应加这条边界用例 |
| 3 | **超声 fail-open vs fail-safe** | §1.2②，需产品决策 |
| 4 | **rot 恢复阈值** | §1.3②，安全锁死 vs 恢复能力的取舍 |
| 5 | **实机/仿真方位角符号统一** | §1.3⑤，test_bearing_convention.py 应加 polar_mapper 镜像断言，否则每次换相机复发 |
| 6 | **已知风险-最晚修复倒挂** | V1 近距余量收口封存到 V2 且排在最长依赖链末端（电源→底盘→雷达→range_m 覆盖 <1.6m），与 FIELD_TRIAL:141"贴近人急停"现场风险提示构成倒挂 |
| 7 | **V2 相位代号**（文档 #1）同时是代码问题 | PHONE_NAV 的 V2a-d 与底盘 V2a-c 在排期、目录、文档引用上都会持续混淆 |

### 3.3 拍板清单（建议机主一次拍完）

1. 电源终局：XL4015 装不装（§21.19 vs §21.22/§21.32），采购表随之收敛。
2. 云台舵机：STS3215 / ZP10S / 继续冻结 三选一（§21.34 是最新判断，采购表未跟）。
3. 相机：A/B/C 三选一 + OV2735 §7.6 vs §7.9 裁定；若选 96-98° 款需重算 §15.6 几何账。
4. 雷达：L1 归 V2b 还是 V2c；平面高度 13.5cm 还是抬 25-30cm（牵连 lidar_guard 配置重算）；X3 Pro 是否已购。
5. V2 相位代号统一（建议 V2a/b/c 归底盘线，PHONE_NAV 改 PN-1..4）。
6. 绕行型分离归 V2 还是 V3。
7. 信任窗三选项（V2_LIDAR_FOLLOW_UPGRADE.md:206）。
8. 权威源元规则：ARCHITECTURE §10 vs PACKAGING §8 二选一。
9. 超声 fail-open/fail-safe。
10. HRI 动作层 demo 口径：接执行器还是"仿真已验证、实机暂缓"。

---

## 4. 审查中证伪的两条误报（重要：避免误修）

| 误报 | 事实 | 依据 |
|---|---|---|
| "C3 固件不支持 PAN/TLT，实机发云台指令=急停" | **现役固件支持**：`handleGimbalCommand` 在运动解析之前处理 PAN/TLT（.ino:110-121），rk_control 的 gimbal_send 发的是合法格式，不会停车。残余问题仅"畸形 PAN 行落回运动解析器→停车"（§1.2③） | 亲读 C3_USB_Controller.ino |
| "固件从不发 OBS 前缀行，实机受阻否决永远失效" | **固件发**：`OBS <0|1> <cm>` 每 500ms 心跳+状态变化即报（.ino:160-171），与 rk_control.py:371-377 的 `OBS ` 解析完全匹配。受阻否决链路实机在（前提是 uart 活着，见 §1.1①） | 亲读双方源码 |

（误报来源：仿真线审查只看了旧固件副本/共享解析文件。真实结论：三条"实机断线"只剩一条——HRI 动作无执行器，§1.1⑤。）

---

## 5. 优先级建议总表

**demo 前（本周，均为小改动）**
1. `/move` 加模式门禁（§1.1②，几行）
2. `_low_s` 在丢失/低分分支按 dt 真正累加（§1.1③，几行）
3. uart_send 失败计数+告警日志（哪怕不重连也别静默，§1.1①的止血版）
4. geometry_daemon ring≥3 跳过（§1.3①，一行，防世界快照崩溃）
5. 出一份晚于 10-19 的实机完成度快照 + baseline 回归一次，终结双记载（§3.1 #11/#12）
6. 演练一次 C3 拔插自愈（验证 §1.1① 的实际表现）

**V2 雷达上车前（必须）**
7. rot 迟滞恢复阈值改 rot 尺度（§1.3②）
8. epoch/monotonic 时钟域统一（§1.3③）
9. 实机左右放手实测裁决镜像问题（§1.3⑤）
10. lidar_guard 配置键接线（§1.3⑥）

**解封转向通道前（TURN_ENABLED=True 之前）**
11. decoder 符号自洽 + 真机符号机器验证（§1.5）
12. SEARCH 相加墙钟上限（§1.5）
13. CX/FCX 主点统一（§1.5）

**随手清理（无风险）**
14. 三对逐字节重复文件改 re-export；settings.yaml 删除或标注失效；.bak 移出 git；video_stream.py 加弃用横幅；gimbal/README GAZE_ENABLED 改 False。

---

---

## 6. 复核更新（2026-10-24 拉取 b8ae790 后）

拉取成功（`7be364b → b8ae790`，30 个新提交；本地未提交的 §21.34 改动完好）。逐条复核结果：

**已被新提交修掉（5 项）**
| 原编号 | 修复 | 证据 |
|---|---|---|
| §1.1④ hri_state 出画断流 | `0f1269e`：owner 缺席也发布（hri_state.py:349"主人不在也要发布"） | ✅ |
| §1.1⑤ HRI 动作层无执行器 | `bef9e10`：A1.3 动作桥 + hri-state systemd 服务 + parked grid 刷新（rk_control.py:525 起 `_hri_action`/`_hri_bridge_loop`，含 GOTO_SAFE 有界驶停）+ test_hri_bridge.py | ✅ |
| §1.1⑦ 帧文件非原子写 | v7.4（2026-10-20）：tmp.jpg + os.replace（video_stream_v7.py:251-256），注释明说修的就是我报告的并发读问题 | ✅ |
| §1.5 decoder 符号自相矛盾 | 头注释已改为自洽跨线契约（2026-10-14 审计后：`theta_world = (pan-PAN_FORWARD) - decoder_bearing`）——部分缓解；真机符号机器验证/标定记录仍缺 | ◐ |
| c3-watchdog 增强 | `389aa13` xHCI 深复位、`78ff111` VBUS 电源循环钩子——覆盖"C3 消失"场景；但 rk_control 死 fd 自愈仍缺（下表） | ◐ |

**仍未修（复核确认在 b8ae790 上原样存在）**
- §1.3① ring=3 IndexError（geometry_daemon.py / polar_mapper.py 未动）——**仍是最优先一行修**
- §1.3② rot 迟滞锁死（lidar_guard.py 未动）——雷达上车前必修
- §1.3③ epoch/monotonic stale 死路（lidar_guard.py 未动）
- §1.3④ EWMA 跨 ±180 缝（10-22 的 13 行改动是"objects 接入 summary"，不是 wrap 修复）
- §1.1① rk_control 死 fd：uart_open 仍只在启动调用一次（:1632），无进程内重连
- §1.1② /move 仍无跟随模式门禁（代码原样）
- §1.1③ _low_s 仍从未累加（:204/:366/:369）
- §1.2 固件全部未动（DIFF 超声即时停车、fail-open、舵机固件层限位、阻塞打印/无看门狗）
- §1.5 FCX=320 vs CX=292.7 主点偏差仍在（follow_controller.py:24）

**§1.5 SEARCH 无上限：部分缓解**——`_pivot_cmd` 自带 `PIVOT_TIMEOUT_S` 墙钟超时（单次 pivot 不会无限转），但 SEARCH 总量预算 `SEARCH_MAX_DEG` 仍只靠 IMU 积分（`_acc_turn_deg` yaw=None 直接 return）；IMU 失效时的 SEARCH 行为建议实测一次确认。

**改变风险格局的新变化**
1. **TURN_ENABLED=True**（follow_controller.py:93，10-20 为 T4 passby 启用）——§1.5 从"潜伏"变"现役"：符号链与主点偏差现在实时影响 passby。
2. 全时 IMU yaw-rate 闭环（`296334d`，修复 ±50px 死区门控）+ 近目标比例减速（`2557bb6`）。
3. leg-follow 原型（`ac473aa`，默认 OFF）+ 安全门（`0a754b6`：0.6m 行程帽、单腿带候选、面积增长拒绝，tests 8/8）。
4. 重载速度/力矩档 + 运行时调参 `/motion/config`（`f04c671`）。
5. web 测试台 `/test`、T 系列 web 化、面板重设计、OBS 遥测记录入黑匣子（`7e1fc9f`——侧面印证 OBS 链路在用，与 §4 证伪结论一致）。
6. sector_score 生产接入 objects + HIDE 落点远离门（`f090e2f`/`b8ae790`，含 10-22 修"objects 原先没进 summary"）——行为变更在冻结期后继续累积，§3 的"纯增量"张力更实。
7. 文档侧部分新鲜化：REVIEW v2 版盘点兑现（`5d86fce`）、playbook 诚实表更新到 post-10-20（`784281b`）、HRI 实地清单 H1-H9（`e650243`）、field test plan + 拍摄清单（`2f76dbd`）——§3.1 #11 的"demo 状态无新快照"已部分解决，但 baseline 回归双记载、GAZE_ENABLED 常数表、相机 §7.6/§7.9 等仍未动。

---

*审查覆盖：全部 docs/*.md、README×4、shadow_carrier_on_rockchip 的 scripts/state/world/gimbal/decision/perception/communication/deploy/config/configs、simulation/webots 全目录、DistributedRobot_C3_MotionController、DistributedRobot_S3_Gateway、wifi/C3_WiFi_Controller、C3_USB_Controller、tools。审查只读，未修改任何文件（本报告除外）。*

---

## 7. 统筹复核与白名单（2026-10-24，致审查者）

> 你不在 EXCHANGE 名单下：**后续回执直接追加在本文件**（§7/§8 之后），不用进 EXCHANGE。
> 本节是统筹对你这份报告的裁决；§8 是给你的下一项任务（passby）。

### 7.1 复核结论

- **报告可信**：抽查 6 条关键证据全部成立——`_low_s` 从不累加、`/move` 无 mode 门禁、
  `uart_open` 只在启动调用一次、`grid` 只建 3 行而 `ring=3` 越界、`profile=None` 异常后永久丢模板、
  两条误报（PAN/TLT、OBS）确实不成立。
- **“18 条 P0/P1”里真正影响 demo 的约 4–5 条**；其余属 V2/雷达/固件/文档，按 §7.2/§7.3 分级。
- **§6 的“已修/未修”对照准确**；`hri_state` 出画断流、HRI 动作层、帧原子写确已修。

### 7.2 允许你自行展开的白名单（demo 前，小改 + 测试）

| # | 项 | 验收 |
|---|---|---|
| 1 | `/move` 加跟随模式门禁 | 跟随中 POST `/move` 被拒绝；T0/T8 仍可用（先 manual） |
| 2 | `_low_s` 按 dt 真正累加 | 低分持续 ≥3.2s 清跟踪器；`test_follow_dt` + `test_passby_lock_confirm` 绿 |
| 3 | `uart_send` 失败计数 + 告警 | `/healthz` 暴露 fail 计数；不再静默；重连可选但必须有测试 |
| 4 | `geometry_daemon` `ring>=3` 跳过 | >3m 目标不再 IndexError；加一条单测 |
| 5 | `geometry_map` EWMA 跨 ±180 wrap | 172°/-172° 平均为 180°（不是 34°）；加一条单测 |
| 6 | 文档/清理 | `GAZE_ENABLED` 表、`settings.yaml`、`.bak`、`deploy.sh` 同步范围 |

**约束（硬）**
- 每条**独立 commit**；不并发改 `rk_control.py` / `follow_controller.py`（T4 正在用）；
- **不推 GitHub**（等统筹）；**不烧 C3 固件**；**不改 V1 速度/转向增益**；
- 改完跟一条：回归 + 板端 smoke + 本文件回执（改了什么/证据/未过项）。

### 7.3 暂不允许（demo 后）

- **固件改动**：DIFF 超声即时停车、超声 fail-open、舵机固件层限位、TWDT/非阻塞打印 →
  要烧 C3，**demo 前不动**；先做 host-side 兜底（见 §8.3-f）。
- 雷达/V2 几何（rot 迟滞、时钟域、配置键）、仿真重构、三对重复文件大改。
- 任何会让当前 T 系列结果失效的改动（若必须改，等 T4 跑完再动）。

### 7.4 对当前测试的影响

- 本报告是**只读**的 → 目前零影响。
- 白名单中 **#2 `_low_s`** 会改“错锁逃逸”行为 → 改后**重跑 T4**；
  **host-side OBS STOP** 会改跟随停车 → 改后**重跑 T4/T10**。
- **T4 前必做**（不受修复影响）：① T0b `-30°` 确认转向符号（`TURN_ENABLED=True` 后 §1.5 现役）；
  ② C3 拔插演练（验 §1.1① 死 fd）；③ T9 前左右手镜像实测（验 §1.3⑤）。
- **固件不要在 T4 期间烧**——烧了所有 T 系列结果都要重跑。

---

## 8. 请继续思考 passby（统筹派工）

> 目标：**先不改代码**，产出一份“passby 诊断表 + 补丁清单 + 实验设计”；每条给证据与验收。
> 可直接在本文件追加回执（§9 起），不用进 EXCHANGE。

### 8.1 要回答的问题

1. **失效模式清单**：把 passby 可能出的问题列全——停住、反向转、贴人、换主、出画、
   抖动/锯齿、蛇形、近距 off-axis、腿-only、镜像、延迟、丢人后乱走。
2. **根因 + 可观测信号**：每个模式对应哪个变量/日志/代码位置，怎么一眼判出来。
3. **demo 期可改 vs V2**：哪些是参数/门控/日志/分析器能解决的，哪些必须固件/雷达/重规划。
4. **T4 12 条怎么跑最省**：优先级、每条最小样本、如何用现有 `/test` + `passby_record/analyze` 自动判定。
5. **社会性约束**：结合 Greenberg 2025（擦身曲率 k≈0.4、6m 开始避让）、Neggers 2022（>90cm、速度）、
   IJHCI 2025（>120cm + 低于步速）——给 V1 可执行的限速/转向平滑/不要贴身旁原地转的约束，以及 V2 改法。
6. **A/B 实验**：`TURN_ENABLED` 开/关、近距门、速度档、腿跟开/关——用现有记录器做对照，别新增仪器。
7. **安全兜底**：DIFF 无即时超声停车 → 建议 host-side STOP 条件；分析器如何标记“危险帧”。
8. **输出格式**：先给表，再给补丁清单（file:line + 改法 + 风险 + 验收），最后给“建议不要做”的清单。

### 8.2 现有资产（直接用，别重造）

- 代码：`follow_controller.py`（转向通道 / 近距门 / 速度门 / 地面测距）、`rk_control.py`
  （动作桥、`/test`、`/motion/config`）、`passby_record.py` / `passby_analyze.py`、`hri_state.py`。
- 状态：`/dev/shm/follow_status.json`（`cmd/val/offset/lost/yaw_rate/w_des`）、`owner.json`、
  `hri_state.json`、C3 `OBS blocked/cm`。
- 文档：`docs/FIELD_TEST_PLAN.md` §3 T4、`docs/T0B_T4_RUNBOOK.md`、`docs/FIELD_TRIAL_PLAYBOOK.md` §6.1。

### 8.3 你能帮上忙的具体点（按价值排序）

- **(a) 失效模式 → 信号映射表**：把“看到什么现象”直接映射到“查哪个文件/字段/代码行”。
- **(b) 分析器增强**：停住**原因分类**（STOP 来自近距门/丢失/转向/人工？）、**转向方向正确性**、
  **横向净空 proxy**（用 bbox/地面几何/视频标注）、**危险帧标记**（人车 <0.6m 或 OBS <35cm）。
- **(c) 参数/门控建议**：`PIVOT_EXIT_DEG`、`NEAR_GATE_OFFSET`、速度门、`TURN_ENABLED` 开/关的边界；
  给“改一个值 → 预期哪个指标变”的因果表。
- **(d) 社会性**：把上面三篇的结论落成 V1 可执行的 3–5 条（限速、转向平滑、禁止贴身旁原地转等）。
- **(e) A/B 设计**：用现有 10Hz 记录，给出最小可判定的对照（别引入新硬件）。
- **(f) 安全**：host-side `OBS blocked/cm≤35 → STOP`（无论 DIFF/MOVE）的建议实现与回归；
  以及“固件批”留到 demo 后的理由/验收。
- **(g) 文档**：把 T4 判据、危险帧定义、失败归因写回 `T0B_T4_RUNBOOK` / `FIELD_TEST_PLAN`。

### 8.4 边界

- **不改固件/雷达/大重构**；**不改 V1 速度**；**不并发改 `rk_control`/`follow_controller`**（T4 期间）；
- **不推 GitHub**；回执写本文件 §9 起；每条建议必须带”证据 + 验收 + 风险”。

---

## 9. 回执 A：白名单修改（2026-10-24，已执行）

六条独立 commit，**均未推送**（等统筹）。基线 `f530587`，系列 `4ae2253..2b527a8`。

### 9.1 改动清单

| # | commit | 内容 | 验收（裁决原文） | 结果 |
|---|---|---|---|---|
| 1 | `4ae2253` | `/move` 补 `mode!=”manual”` 门禁（与 `/forward` 组同文案；`/stop` 例外不涉及本端点） | 跟随中 POST `/move` 被拒；T0/T8 仍可用（manual 下） | 代码层满足；**板端待验**（见 9.3-①） |
| 2 | `c75093f` | `_low_s` 按真实帧间隔累加（`_low_last_t` 锚点），命中主人清零；`_low_s≥3.2s` → `tracker.reset()` 复活 | 低分持续 ≥3.2s 清跟踪器；`test_follow_dt`+`test_passby_lock_confirm` 绿 | 两测试 PASS（离线）；**改后需重跑 T4**（§7.4） |
| 3 | `99fe04b` | `uart_send` 失败计数 `uart_stats` + 5s 节流告警 + `/healthz` 暴露 `uart` 字段；glue 测试新增 E 段 | `/healthz` 暴露 fail 计数；不再静默；重连未做（可选项，V2 配 udev 测试再做） | glue E 段 PASS，测试输出可见 `[UART] send FAIL ×1 ... err=TypeError`（证明不静默） |
| 4 | `2f921c7` | 落格循环抽成 `stamp_objects()` 纯函数，`ring/sector` 越界跳过（>3m=ring3 超远，无格可落） | >3m 目标不再 IndexError；加一条单测 | 新增 `test_stamp_objects.py` 3 断言 PASS（含 `polar_to_sector(10°,3.5m)→ring=3` 语义锚） |
| 5 | `d73661d` | 新增 `_ewma_bearing()` 最短角差 EWMA，门/物体两处替换；非跨缝与旧公式逐值一致 | 172°/-172° 平均为 180°（不是 34°）；加单测 | `test_geometry_map_wrap.py` 4 组 PASS：172/-172→178.4（旧公式 34.4），sector 4（正后方） |
| 6 | `2b527a8` | GAZE_ENABLED 表改 False（对齐代码+三份裁定）；settings.yaml 头部标失效（指向 configs/）；`.bak` 18 文件 `git rm --cached`+gitignore（本地保留）；deploy.sh 补同步 `deploy/ configs/ world/` | 文档/清理 | `bash -n` 过；**注意**：.bak 退出跟踪会在统筹批准推送后从他人 checkout 消失（有意为之，历史仍在 git） |

### 9.2 回归（2026-10-24，Mac 本地全部离线）

- scripts/：`test_follow_dt` `test_passby_lock_confirm` `test_control_glue` `test_turn_pivot` `test_yaw_rate` `test_weak_cue` `test_leg_follow` `test_hri_bridge` `test_pick_safe_spot` → **9/9 PASS**
- world/fusion/：`test_lidar_guard` `test_sector_score` `test_stamp_objects(新)` `test_geometry_map_wrap(新)` → **4/4 PASS**
- 修改文件 `py_compile` 全过；`deploy.sh` `bash -n` 过。

### 9.3 板端待办（我到不了板子，留给机主/安全员，按 §7.4）

1. **smoke（改后必做一次，manual 模式）**：`curl -X POST /move {“direction”:”F”}` 应 OK；切 follow 后同请求应返回 `follow mode: 手动运动端点已锁`；`/healthz` 应出现 `uart` 字段。
2. **拔插演练**：跟随中拔 C3→重插 → 观察 `/healthz` 的 `uart.fail` 持续增长 + `[UART] send FAIL` 日志（此前完全静默）；车应在 450ms 内自停（固件超时）。**死 fd 仍需重启 rk-control**——重连是 V2 项。
3. **T0b `pivot -30`（转向符号，T4 前置）**：车必须向右转。若向左 → `HRI_BEARING_TO_YAW_SIGN` 与 `IMU_TO_THETA_SIGN` 至少一个错，**先停 T4**。
4. **T4 重跑**：#2 改变错锁逃逸行为（此前恒不触发），按 §7.4 重跑 T4。
5. **T9 前左右手镜像实测**（验 §1.3⑤）：人在车正前 1.5m，先向**左**横移出画 → 车若向**右**转 = 镜像坐实（polar_mapper/decoder 符号链反），记录后回填 `test_bearing_convention.py`。

---

## 10. 回执 B：passby 诊断表（§8.1-1/2，§8.3-a）

> 信号源全部来自现有记录器（`passby_record.py` 10Hz JSONL）与 `follow_status.json`
> （`state/cmd/val/offset/lost/owner/phase/turn_deg/smooth_h/obs_blocked/obs_cm`）——零新增仪器。
> “一眼判”列 = 在 jsonl/日志里 grep 什么。

### 10.1 失效模式 → 根因 → 信号 → 位置

| # | 现象 | 主要根因（按概率排序） | 一眼判（信号） | 代码位置 |
|---|---|---|---|---|
| F1 | **转 30° 就停** | 近距门误触发：框高急停没被 `NEAR_GATE_OFFSET` 拦住（人在侧但 offset≤100px） | `follow_cmd==STOP` 且 `owner_ok==true` 且 `follow_state==FOLLOW` 且 `\|offset\|≤100` 且 `smooth_h≥400` | follow_controller.py:912,54 |
| F2 | **停住不动（非近距）** | ①丢失流程停车 ②TURN 相预算尽 ③超声否决 ④uart 死 fd（车根本没收到） | ①`owner_ok=false≥2.4s`+`state∈wait/lost` ②`phase=TURN`+`turn_deg≥180` ③`obs_blocked=true` ④`/healthz.uart.fail` 增长（本批新增暴露） | :723-744; :141; _obs_blocked; rk_control.py:237 |
| F3 | **反向转（背离人）** | 符号链三选一错：`IMU_TO_THETA_SIGN` / `HRI_BEARING_TO_YAW_SIGN` / 相机镜像（§1.3⑤） | `follow_cmd=MOVE L` 但 0.3s 后 `yaw` 增大**且** owner 最后在右侧（`owner_cx>320`） | :158 IMU_TO_THETA_SIGN; rk_control.py:447; polar_mapper.py:49 |
| F4 | **贴人（净空<0.6m）** | 侧向无近距传感：框高在 2.2m 内饱和、地面测距 <1.58m 出画、速度门 30° 仍留 55% 速度 | 最接近帧：`persons` 有框 + `\|bearing\|>20°` + `y2>460`（脚将出画=已进 1.6m） | :57 速度门注释自认; range_from_box.py:58 |
| F5 | **换主（锁路人）** | 换人确认门被”连续候选”绕过（tracker 预测恰好滑到路人）；或低分逃逸（本批 #2 修前永不触发） | jsonl 疑似换主段（owner_cx 跳变 >200px）+ `test_passby_lock_confirm` 离线复现 | :66-84 LOCK_CONFIRM*; 本批 c75093f |
| F6 | **出画后回不来** | BLIND 盲转不足/过头（60° 上限是 10-16 仿真实测防甩尾）；SEARCH 方向反（F3 同源）；SEARCH 预算只靠 IMU 积分，IMU 失效时预算不减 | `phase=BLIND/SEARCH` 后 `owner_ok` 长期 false；`yaw_rate=null`（IMU 死）但 `follow_cmd` 仍在发 MOVE L/R | :100-104,141-146; _acc_turn_deg :570 |
| F7 | **抖动/锯齿转** | 迟滞 30/15 边界震荡；REVERSAL_MAX 未熔断 | 5s 窗内 `MOVE L↔R` 交替 >3 次（REVERSAL 熔断前） | :92-93,106-107 |
| F8 | **蛇形直行** | `W_DES_GAIN`/`YAW_K` 与重载不匹配；MIN_SPD=155 顶住内轮下限 | 直行段 `yaw_rate` 绕 0 振荡 >±8dps；面板 `/motion/config` 现场调阻尼 K | :33-38; runbook §0.5-⑦ |
| F9 | **丢人后乱走** | SEARCH_BACK 后退 1.2s + 扫描转向，在人流里 = 无目标移动 | `state=LOST` + `follow_cmd=MOVE B` + `hri_state≠WAIT` | :64-66, SEARCH_BACK |
| F10 | **腿-only 段丢跟** | <0.8m 只剩腿带弱线索，`WEAK_TTL_S=3s` 过期；LEG_FOLLOW 默认关（统筹裁定 A） | `persons` 框高度 <150px（MIN_BBOX_H）段丢失 | :120-133; owner_id.find_leg_band |
| F11 | **顿挫（周期性停 0.45s）** | 检测 API 抖动 → follow tick 阻塞 >450ms → 固件超时自停再起步 | `det_age_ms` 尖峰 + `fps` 掉 + 随后 `follow_cmd` 从 STOP 恢复 | rk_control 10Hz tick 调 `_fetch_detections` timeout=1.0 |
| F12 | **仿真数字不可信** | 墙钟/仿真钟混用 → passby 单跑 4.5× 抖动（README:72-75 已记录） | 只影响 Webots 分析，实机无此问题 | shadow_carrier.py:1281 vs :1232 |

### 10.2 判据数字速查（T4 判定全表见 RUNBOOK §3）

停 ≤1.0s ｜ bbox ≥90% ｜ 最长缺失 ≤0.5s ｜ 净空 ≥0.6m ｜ 0 换主 ｜ 丢人 ≤2.4s 进 wait_owner。
危险帧定义（建议，供 §11-b 分析器落地）：`obs_cm≤35` 或 `obs_blocked=true` 或（`\|bearing\|>20°` 且 `y2>460`）。

---

## 11. 回执 C：补丁清单（**提案，未实施**——除标注外均需统筹批准）

### 11-a 分析器增强（passby_analyze.py，纯增量，不动控制链）

| 补丁 | 位置 | 改法 | 验收 | 风险 |
|---|---|---|---|---|
| 停住原因分类 | 新函数 `classify_stops(recs)` | 对每个 ≥0.3s 停止段，按 10.1-F2 的四分法打标（near/lost/turn/obs/uart）输出计数表 | 人造 jsonl（各造 1 段）分类正确 | 无（只读分析） |
| 转向方向正确性 | 复用已有 `bearing_of(cx)` | 每次 `MOVE L/R` 起拍后 0.3s，`yaw` 变号 × 最后 owner 侧 → 朝人/背离 计数 | 与视频抽检 5 例一致 | 无 |
| 危险帧标记 | 主循环 | 按 10.2 定义打 `danger` 标 + 段汇总 | T4 实测含 OBS 段能标出 | 无 |
| 横向净空 proxy | `persons` 框宽×bearing | 只作”疑似 <0.6m”旗标，**不替代卷尺/视频**（标注为 proxy） | 与 3 次卷尺实测误差 <0.15m | 误报（宁可多报） |

### 11-b host-side OBS 兜底（§8.3-f，对应 §1.2① 固件缺陷的宿主侧止血）

**为什么不改固件**：烧录 C3 会让全部 T 系列结果作废重跑（§7.4 明令）；且 5s 强解/450ms 超时等语义改动需要单独的台架验证。host 侧一层 10Hz 拦截即可覆盖”DIFF 命令间隙继续前进”的主风险。

**落点**：`rk_control.uart_send` 最前（在 `_safety_filter` 之前，超声与雷达两套包络各自独立）：

```python
def _obs_guard(cmd):
    “””超声兜底(提案): OBS 新鲜且 blocked 或 cm<=35 时, 前进类命令即时改 STOP。
    固件只拦 MOVE F 且只在下一条命令时拒 DIFF(§1.2①); 本层补 DIFF 间隙。”””
    with obs_lock:
        fresh = time.monotonic() - obs_state[“t”] < 1.0
        blocked = bool(fresh and obs_state[“blocked”])
        cm = obs_state[“cm”] if fresh else None
    if not blocked and (cm is None or cm > 35):
        return cmd
    c = cmd.strip().upper()
    if c.startswith(“MOVE F”):
        return “STOP”
    if c.startswith(“DIFF”):
        try:   # 左右和>0 = 净前进
            l = int(c.split()[1].lstrip(“L”)); r = int(c.split()[2].lstrip(“R”))
            if l + r > 0: return “STOP”
        except Exception: pass
    return cmd          # STOP / MOVE B / MOVE L / R / PING 原样放行
```

**验收**：glue 测试 F 段（注入 `obs_state` 三态：fresh blocked / fresh cm=30 / stale → 只有前两者拦 `MOVE F 200` 与 `DIFF L200 R100`，`MOVE B`/`STOP` 永远放行）；板端重跑 **T4+T10**（§7.4）。
**风险与已知边界**：①继承固件 fail-open（传感器断线 5 次后 OBS 恒 0 → 本层同样失效——治本在 demo 后固件批）；②路人腿在 20cm 处会触发 → 这正是想要的行为；③manual 模式同样生效（面板顶头也会被拦）——**设计如此**，若机主要求手动豁免需另加开关。
**代价**：uart_send 热路径 +一次锁与字符串判断（<10µs），10Hz 无感。

### 11-c 参数因果表（§8.3-c；**V1 一律只观测不改**，改=需统筹特批）

| 参数（现值） | 改向 | 预期指标变化 | 风险 |
|---|---|---|---|
| `PIVOT_EXIT_DEG`(15) | ↓10 | 更早回弧线 → 命令停止段↓，但 30/15 迟滞带变窄 → F7 抖动↑ | 边界震荡 |
| `NEAR_GATE_OFFSET`(100px≈11°) | ↑130 | F1 误停更少；但正前近距保护窗变窄 | 贴正前 |
| `V_GATE_DROP`(0.45@30°) | ↑0.6 | F4 净空↑（30° 处速度 55%→40%）；T4 通过时长↑ | 跟不上人（F11 放大） |
| `LOCK_CONFIRM_STRICT_N`(5) | ↑7 | 换主↓；重捕获延迟↑ → 最长缺失可能破 0.5s 判据 | 判据内耗 |
| `WEAK_TTL_S`(3.0) | ↑4 | F10 腿段维持更久；弱线索误维持↑ | 错方向多转 |
| `MIN_SPD`(155) | — | **勿动**（§7.3 V1 速度冻结）；蛇形先用面板阻尼 K（runbook §0.5-⑦） | — |
| `TURN_ENABLED`(True) | False | 回退开关：出画时长↑↑、但 F1/F4/F7 全消失——**A/B 对照臂**（§12） | — |

### 11-d 社会性约束落地（§8.1-5）

文献口径（统筹给定）：Greenberg 2025 擦身曲率 k≈0.4、~6m 开始避让；Neggers 2022 净空 >90cm；IJHCI 2025 >120cm 且低于步速。**V1 无雷达无测距，只做可观测的三条**：
1. **净空硬线 0.6m**（现行 T4 判据，介于”产品下限”与”文献 0.9m”之间——demo 场地 2.5m 走廊限制，文档如实写”受限场景”）；
2. **贴身不原地转**：人不出画（<30°）绝不 pivot（`TTE_MAX_DEG=30`+`PIVOT_ENTER` 双门已保证），擦身中只减速让行（速度门）——这条已是代码行为，写进话术即可；
3. **擦身段前向 ≤0.45m/s**（≈步速一半，IJHCI 口径）：V1 只在分析器里**报告**实测值（BASE_SPD×速度门反推），V2 用编码器闭环真正执行。
V2 改法：雷达 6m 预避让 + 曲率平滑（W_DES 限速+斜率限制，逼近 k≈0.4）+ 编码器测速闭环。

### 11-e 建议不做清单（§8.1-8）

1. 不改 `BASE_SPD/NEAR_STOP_M/框高三阈值`（机主 10-19 裁定 + §7.3）；
2. 不启用 leg-follow 做 passby 兜底（统筹裁定 A 默认关，安全门刚上 8/8 测试但无实车数据）；
3. 不动 `LOCK_CONFIRM*` 门（passby 核心防线，改则 T 系列全重跑）；
4. 不加 360° 扫描/持续旋转（§12.4 “不动比动更被信任”）；
5. 不烧固件（§7.4）；不引入新仪器（§8.2）；
6. 不拿 Webots passby 单跑数字当门禁（F12，4.5× 抖动，README 已载）。

---

## 12. 回执 D：T4 实验设计（§8.1-4/6）

### 12.1 12 条排序（省时：先证符号、再探边界）

| 序 | 条件 | 理由 | 最少样本 |
|---|---|---|---|
| 1-2 | 1.2m-正常-**右** / -**左** | 中等净空最宽容；一对即可验 F3 符号对称性（左右各一） | 各 2 |
| 3-4 | 0.8m-慢-右 / 左 | 最难档（腿段+速度门）；F4/F10 集中暴露 | 各 2 |
| 5-6 | 1.5m-慢-右 / 左 | 宽松上界（不出画的纯弧线段基线） | 各 1 |
| 7-12 | 0.6 步速×(0.8/1.2/1.5)×左右 | 快步速补全（前 6 条全绿才跑） | 各 1 |

**自动判定**：`passby_record`（全程）+ `passby_analyze`（锁定率/缺失/换主/停止段/朝向）；手填只剩**净空**（最接近帧视频+地面卷尺，一次一读）。
**前置不满足即中止**：T0b `pivot -30` 向右（9.3-③）；health_check FAIL=0；`mode=follow` 认主成功。
**中止线**（RUNBOOK §3 已有）：反向转 / 贴人 <0.6m / 换主 → 立即 `/stop` + 抓 journalctl。

### 12.2 A/B（现有记录器，不加仪器；两臂同日同场地同人）

| 对照 | 臂 | n | 主指标 | 假设 |
|---|---|---|---|---|
| A/B-1 | `TURN_ENABLED` True vs False | 3+3（1.2m-正常-右） | 最长缺失 / 出画时长 / 最长停顿 | ON 显著缩短缺失，停顿持平或略增 |
| A/B-2 | 近距门观察 | 复用 A/B-1 数据 | F1 发生率 vs `\|offset\|` 分布 | F1 只在 offset≤100 且 smooth_h≥400 出现（验证门正确性，不改参数） |

A/B-1 的 OFF 臂即回退开关演练（RUNBOOK §4），一举两得。**V_GATE_DROP/leg-follow 的 A/B 需统筹特批后再排**（§11-c/11-e）。

### 12.3 文档回写点（§8.3-g，待统筹批后我来做）

`T0B_T4_RUNBOOK.md` §3 补”停止原因分类/危险帧”两个判定口径；`FIELD_TEST_PLAN.md` §3 T4 表加 danger 列；本文件 §10.2 的危险帧定义作为两处引用源。

---

*回执人：审查线（ZCode 会话）。所有改动未推送；板端 smoke 与 T4 由机主执行。*

---

## 13. 回执 E：文档矛盾增量复审（2026-10-24 下午，HEAD=26f7bff）

> 任务：核对 §3 旧 21 条现状 + 挖 10-17 后新增/变更文档的新矛盾。方法：逐条 grep/复读，
> 每条给 file:line。**T4 判据已确认单一来源**（RUNBOOK 声明"判据来源 FIELD_TEST_PLAN §3"，
> 两处 0.6m/停≤1.0s/0 换主口径一致，不列矛盾）。

### 13.1 已解决（自上轮审计以来）

| 旧# | 内容 | 解决方式 |
|---|---|---|
| #10 | GAZE_ENABLED 表 True | 本批 `2b527a8` 修正；全 docs 无残留 True 引用（REVIEW:152 / V2_LIDAR:51 均为合法条件句） |
| #3b | DEMO_PLAN"原地转已实装" vs Playbook 旧行为 | `784281b` playbook 诚实表更新到 post-10-20（:168 现行为 True），DEMO_PLAN 说法与事实一致化 |
| #11a | demo 状态无 10-19 后快照 | `5d86fce` COORDINATION_REVIEW v2（10-19/20，12 天作战表 §6）+ playbook 诚实表提供新快照 |
| #11b | "A1.3 动作层无执行器"缺口描述 | `bef9e10` 动作桥+hri-state 服务落地，COORDINATION_REVIEW v2 已按"接线"口径更新 |

### 13.2 仍开放（原样或残留）

| 旧# | 矛盾 | 现状证据 |
|---|---|---|
| #1 | "V2a/b/c"两义 | V2V3_ROADMAP:33（底盘）vs V2_PHONE_RELATIVE_NAV:202（相对估计）vs DEFERRED_WORK:122——三处原样 |
| #2 | 雷达 L1 归 V2b/V2c | V2V3_ROADMAP:44（V2b）vs V2_CHASSIS:33（V2c）；附带 V2V3:45 仍写"51 断言"（实际 56） |
| #3 | 绕行型 V2 vs V3 | DEFERRED_WORK:43,157（V2）vs V2V3_ROADMAP V3b——原样 |
| #4 | 电源五版/XL4015 无终局 | 不装侧 §21.22:1675,:1733 vs 必买/推荐侧 :1074,:1106,:1286,:1328,:1391,:1507,:1515——原样无作废标记 |
| #5 | 舵机三态 | V2V3:46（预算允许/冻结）vs §21.7 采购表 STS3215 vs 未提交 §21.34 ZP10S——原样 |
| #6 | 相机 §7.6 vs §7.9 | CAMERA_SELECTION:210"不推荐" vs :262"可以买"，同日两节仍无修订声明 |
| #8 | 权威源元规则打架 | shadow_carrier_on_rockchip/docs/ARCHITECTURE.md:139-149（PACKAGING 排期降背景）vs PACKAGING_PLAN:320（以本文档为准）|
| #9 | Pre v0 已上线 vs 未实现 | DEFERRED_WORK:32 vs PACKAGING_PLAN:306——原样（套磁口径风险仍在） |
| #11c | DEMO_PLAN §1b"整体≈78%" | DEMO_PLAN:25（10-08 口径）已被 COORDINATION_REVIEW v2 实质取代，但无"以 v2 为准"指针 |
| #12 | baseline 8/8 PASS vs FAIL | webots README:51 vs :77——原样，10-18 修复后仍无回归记录 |
| #13 | 冻结铁律 vs 实际 | REVIEW_AND_SEMESTER_PLAN:110,205"10 月中冻结"未改口（COORDINATION_REVIEW:14 已给新口径但上游未回改）；DEMO_VIDEO/README:4 仍按"10-11 冻结后立即拍"倒排 |
| #14 | 12 月时间账 | PACKAGING:276"12 月–1 月 V2 整机+续航" vs V2V3:40（12月W2 才装完传感器）、:50（W3 起整合） |
| #15 | ROADMAP.md 过时且无标记 | :5"躲到门边"（vs DEMO_PLAN 墙根离门9m）；且被 ARCHITECTURE §10 列为**计划权威**——权威本身最旧 |
| #18b | 果冻/躲门边残留 | DEMO_PLAN:17 仍"果冻/水瓶"（:35 自己说须换）；REVIEW:16 场景一句话仍"躲到门边+果冻" |
| §3.2 | 代码牵制 7 条 | 全部原样（属代码/拍板项，非文档修复） |

### 13.3 新发现矛盾（10-17 后新增文档引入）

| # | 矛盾 | 双方证据 | 影响 |
|---|---|---|---|
| N1 | **FIELD_TEST_PLAN 速度档过期一天** | FIELD_TEST_PLAN:6（10-20 写，HEAD 7b4ddc5）"BASE_SPD=220/MIN_SPD=120/PIVOT_PWM=190" vs 10-21 重载档 245/155/240（rockchip README:114、DEFERRED_WORK:135、代码） | T4 判据文档与车实际差一档；按 220 心算制动距离会低估 |
| N2 | **DEFERRED_WORK §1.4 说转向链默认关** | DEFERRED_WORK:64"TURN_ENABLED=False"（10-11）vs FIELD_TRIAL_PLAYBOOK:64,168（True，10-20）、T0B_RUNBOOK:121、follow_controller.py:93 | §1.4 是"当前实现"权威描述节，读它的人会以为转向链关着 |
| N3 | **playbook 内部：标定前置 vs 已启用** | FIELD_TRIAL_PLAYBOOK:101"只有标定完，才考虑置 True" vs :64 已 True 且待 T4 | §4.3 未更新；且 10-20 开启前 pivot_calib 是否跑过**无任何记载**（启用前置无记录，见 §9.3-③） |
| N4 | **c3-watchdog README 能力表未随新功能更新** | 同文件 :12"C3 完全消失 ❌ 只能记日志" vs :90 起 xHCI 深复位（10-21 已验证恢复"消失"，脚本 deep_reset() 在 c3_watchdog.sh:46） | 表格低估现有能力（真掉电仍 ❌，枚举级消失已 ✅）；现场可能白插拔 |
| N5 | **服务清单三套** | 板上 README:50-56 自启表 4 项（缺 imu/c3-watchdog/hri-state）vs FIELD_TEST_PLAN:24 列 5 项（缺 hri-state）vs health_check 实查 6-7 项（含 hri-state、geometry-parked.timer） | 换板/排障时对"该有几个服务"三个答案 |
| N6 | **world/README.md 定位过期** | 自称"world_lab 独立实验区…收编进仓库时整体迁移为 world/ 模块"（08-24 口径）vs 该目录即生产代码（fusion 喂 sector_score/grid.json）且板上部署目录也叫 ~/world_lab | 三重身份（实验区/生产模块/板上目录名）没说清，新人必踩 |
| N7 | **playbook 教现场改 BASE_SPD 与裁定相反** | FIELD_TRIAL_PLAYBOOK:52"L1 限速…临时改 BASE_SPD（改完必须跑回归）" vs DEFERRED_WORK:131"V1 期不再改 BASE_SPD"（机主裁定）+ COORDINATION_REVIEW:14"不调 V1 速度阈值" | 现场预案给了与两份裁定相反的操作指引（若"统筹批准后可改"应写明） |
| N8 | **板上 README 四代速度并存** | rockchip README:112"BASE_SPD 100→140~160"（远古复标建议）、:273"BASE_SPD=220"、:277"PIVOT_PWM 110→190" vs :114 重载档 245/155/240 | 同文件四代数字，排障时不知道信哪个 |
| N9 | **RUNBOOK 行号漂移** | T0B_T4_RUNBOOK:176"follow_controller.py:88 改回 False"——实际 :93 | 小；现场按行号找不到 |

### 13.4 建议（按 demo 前收益排序，多为一行改动，待统筹批）

1. N1+N8：FIELD_TEST_PLAN:6 与 rockchip README:112/273/277 的速度数字统一为重载档（245/155/240）并注明"10-21 重载档，/motion/config 可现场调"。
2. N2：DEFERRED_WORK §1.4:64 补一句"2026-10-20 起TURN_ENABLED=True（T4 待验），本节其余描述为 10-11 快照"。
3. N3：playbook §4.3:101 改"TURN_ENABLED 已于 10-20 开启；若换底盘/电池需重跑 pivot_calib 后再重启跟随"；并在 EXCHANGE 补记 10-20 开启时标定状态。
4. N4：watchdog README:12 表拆两行："枚举级消失→✅ xHCI 深复位（:90）/真掉电→❌"。
5. N5：三处服务清单统一为 health_check 的口径（6 服务+timer），板上 README 加 imu/c3-watchdog/hri-state 三行。
6. N7：playbook:52 改"临时改 BASE_SPD **需统筹批准**（V1 速度冻结，DEFERRED_WORK §3）"。
7. 其余旧账（#1/2/3/4/5/6/8/9/14/15）属 V2 规划拍板项，建议 demo 后按 §3.3 清单一次拍完再回改。

---

## 14. 回执 F：补审（2026-10-24 晚，视觉服务本体 / 部署面 / 测试台面板）

> 覆盖此前从未审过的三块：`perception/`（yolo_daemon 真身）、`deploy/`+`configs/` 部署面、
> `/test` 面板防误触。另补 /move 门禁 handler 级测试（`755dd51`）。**全部只读审查**，
> 发现项未改代码（除新测试文件）。

### 14.1 视觉服务本体（perception/）

- **yolov8.cpp / camera.cpp 是 TODO 占位桩**（load_model 恒 false、infer 恒空）——README 已声明"native 封装是占位"，与事实一致；真身是 `yolo_daemon.cc` 直接调外部 rknn_model_zoo。死代码，无风险。
- **yolo_daemon.cc 本体（140 行）质量好**：v3 stdout/stderr 隔离、v4 out.jpg 原子写、A3.5 ts/t_mono 双时钟都有。发现三点：
  1. **模型与外部构建产物都在仓库外**（v7:34-35 硬编码 `/home/kickpi/shopping_car_vision/.../yolov8.rknn` 与 daemon 二进制路径），且 **deploy.sh 不同步 perception/**——改仓库里的 yolo_daemon.cc 源码不会到达板上，板上跑的二进制与仓库源码的对应关系无 md5 记录。复现性缺口（README 承认，但"源码≠板上二进制"这一条比 README 说的更具体）。
  2. **检测置信度/NMS 阈值不在仓库**：daemon 未暴露阈值参数，阈值在外部 rknn_model_zoo 的 postprocess 里编译期定死（yolov8.cpp 注释提到 0.25/0.45 是桩里的默认值）。T 系列任何涉及"检出率"的结论都无法从仓库复现。
  3. **stderr 是 JSON 协议+日志的混合通道**（`[stage]` 行、`[watchdog]` 行、`{"error":...}` 都走 stderr）：v7 按行过滤 JSON 没问题，但任何第三方往 stderr 打印以 `{` 开头的行都会注入假检测——现状无此打印，仅记录该契约脆弱点。
- **崩溃恢复在宿主侧闭环**（v7:132-172 watchdog：进程退出→restart + 心跳超时→restart，restarts 计数进 /api/detections 并在 /test 健康页可见）✓；`video-stream.service` Restart=always/5s 兜底 v7 本身 ✓。

### 14.2 部署面（deploy/ + configs/）

- **hri-state.service**：`--owner-source file`（消费 owner.json，与仿真同配置）✓、After/Wants=video-stream ✓、Restart=always/2s ✓。小瑕疵：stdout 落 `/tmp/hri_state.log`（重启即丢，与黑匣子 jsonl 的 ~/hri_logs 双标准，排障时容易找错地方）。
- **geometry-parked.timer（20s，只在 WAIT/HIDE 停驻域跑 `geometry_daemon.py once`）**：
  1. 设计合理（停驻才刷 grid、避免抢云台），退出码恒 0（失败只在 journal）。
  2. **该定时器正是本批 ring=3 修复（2f921c7）救活的路径**：停驻刷新时画面里 3m 外的行人/物体此前会让 `once` 整次 IndexError——修复的实战价值由此坐实。
  3. `once` 每次会把云台摆到工作角（pan=90/tilt=112）：GAZE 关停后无竞争方，暂无冲突；若 V2 重开 gaze 需回头审这个交互。
- **dnsmasq.conf**：99.7% CPU 自环修复有完整根因记录 ✓；DHCP 段 192.168.4.2-50 与面板 IP 4.1 一致 ✓。
- **网络双上下文混用**：deploy.sh 默认 `kickpi@192.168.137.190`（笔记本 ICS 网段，开发用）、旧 video_stream.py 硬编码 137.190、demo 网是板子自发热点 4.1——三处口径并存，文档没写"开发网 vs demo 网"的切换说明。换环境排障容易连错网。
- **camera-usb-fix**（2s 轮询写回 power/control=on）与 udev 规则、v7 camera_watchdog 三层互补，注释清楚 ✓。
- 仍缺：**rk-control.service 单元与 C3 的 udev 规则（303a:1001）不在仓库**（§2.2 老问题，换板不可复现、且无规则的板上 watchdog 会循环 usbreset）。

### 14.3 /test 面板防误触（test_panel.html，455 行）

- **已有的防护**（比我担心的好）：T0/T8 动车前有 `confirm("车会短暂动作，确认四周 ≥1m 且无人？")` ✓；腿跟开启有 confirm ✓；急停按钮顺带切回 manual ✓；T0/T8 旧版服务端还有 dry-run 回退路径（2bc0b1f）。
- **发现四点**：
  1. **[P1] "标准档"预设是 10-21 重载档之前的全套旧值**（test_panel.html:428-430：BASE 220/MIN 120/PIVOT 190/W_DES_GAIN 0.08/BACK 85/SEARCH_BACK 70/HRI 80-110）。排障时点"标准档"想恢复默认 → 实际把重载车的力矩降回带不动的档位、转向增益砍半（0.18→0.08）。**"标准档"现值应以代码为准更新或直接删掉只留重载档+自定义**。
  2. **[P1] 重载档预设 YAW_K=2.0 与代码现值 1.0 相反**（:425 vs follow_controller.py:33"2.0→1.0 降增益防振荡"+runbook"阻尼 K=1.0 默认"）——点"重载档"会把已弃用的振荡增益写回去。重载档其余 11 项与代码一致，唯此一项错。
  3. [P2] `setTurn(1)`（TURN_ENABLED 运行时开关）无 confirm——与腿跟的防护不对称；误触关闭会让 T4 在"无转向通道"臂上跑（实验臂被悄悄换掉，A/B 数据作废）。建议与腿跟同级 confirm。
  4. [P2] `pivot(90/180/-30)` 按钮无 confirm（服务端有 manual+IMU 双门，跟随态点了会被拒，manual 下一击即转 180°）。
- 修 1/2 只动 test_panel.html 的两个常量字典（不动 rk_control/follow_controller，不在 T4 冻结名单内），**待统筹批后两分钟可改**。

### 14.4 本轮补的测试

- `test_move_gate.py`（`755dd51`）：起真实 HTTPServer 打桩 uart_send——follow 拒（零串口副作用）/manual 放行（MOVE F 150）/模式相关（再切回 follow 仍拒）/400 路径不吞。白名单#1 的"必须有测试"欠账已清；离线回归现为 **14/14**。

### 14.5 杂项

- `.git/objects` 4 个 `tmp_obj_*` 残片（前几日被杀的 fetch 遗留）已清，`count-objects` 无 warning；仓库 1.75MiB 无体积问题。
- EXCHANGE.md 不在本 checkout——§13 的文档裁决无法对照动态权威源，建议机主把它带进此工作区或在 EXCHANGE 里登记本报告已读（§7 约定"不入 EXCHANGE"，矛盾点：裁决让回执写本文件，但 EXCHANGE 作为动态权威源看不到这些回执）。

---

## 15. 回执 G：多模块底盘/云台仲裁架构评审（2026-10-24 晚）

> 问题：视觉(follow)、HRI(动作桥)、运动执行(转向通道/手动)、未来雷达——互相打架怎么办。
> 结论：**骨架已经搭对**（单一收口 + 分层门禁），雷达按既设计（约束层）接入不会打架；
> 残余风险在三个具体的缝 + 雷达要守住四条规则。

### 15.1 现状全景：谁能动车，被谁管着（全部亲验）

| 写入方 | 线程 | 管着它的门 | 证据 |
|---|---|---|---|
| 面板手动 /move /forward 等 | HTTP 线程 | mode 门（manual 才放行；/stop 例外） | rk_control.py:1423+(本批修)、:888 |
| /turn_pivot /turn | worker 线程 | mode 门（follow 拒）+ IMU 门 | :195-202 |
| /test T0/T8 自检 | 测试线程 | **强制接管**：先 `_stop_follow()` 再跑 | `_test_start` |
| follow 弧线/转向通道 | follow 线程 | **HRI 桥激活期间 `_follow_send` 丢命令**（单控制器纪律）；停驻域 turn_only | :681-686, :436-443 |
| HRI 动作桥 APPROACH/GOTO | 桥线程 | 仅 follow 模式；每个动作有界(max_s)；finally 必清 flag | :653-676, :521-536 |
| 云台 gaze 驱动 | gaze 线程 | GAZE_ENABLED=False 整块冻结 | :56 |
| 雷达（未来） | 无线程！ | **过滤器不驱动**：惰性契约挂 uart_send | `_safety_filter`, V2_LIDAR_AVOIDANCE |
| 超声 | 固件层 | MOVE F 拦截 + TURN 通道 obs_fn 否决 | MotorDriver/`_get_obs` |
| 固件最终裁决 | — | last-writer-wins + 450ms 超时自停 | MotorDriver.cpp |

**核心不变量**：uart_send 是全仓唯一串口写点（21 个调用点全部经过，grep 验证）；所有"打架"最终表现为这里的命令竞争——所以仲裁做在它上方/内部就够，不需要改固件。

### 15.2 现存的三个缝（按风险排序）

1. **门禁是"每个端点各自记得加"**（/move 就是这么漏的，10-16 加门时漏了它、10-24 才补）。同类面：每个新增 HTTP 端点/后台线程都要重新记得查 mode。**结构性修法（提案）**：把 driver 所有权检查下沉进 uart_send——命令带来源标签（manual/follow/hri/test），非活跃 driver 的运动命令在收口点拒绝+记日志，泄漏从机制上消失。代价：21 个调用点标注 + 热路径一次比较。
2. **黑匣子归因缺口**：桥激活期间 follow_status 仍记 follow 的意图命令（实际被丢）；雷达/超声改写命令也只记过滤后结果。打起来时**无法事后判定谁赢了**。修法：uart_send/收口处记 `vetoed_by: bridge/radar/ultrasonic/mode`（§11-b 分析器停止原因分类直接吃这个字段）。
3. **云台没有同款仲裁**：gimbal_send 只有限位没有 owner；geometry-parked 定时器每 20s 把云台摆回工作角（§14.2-3），V2 重开 gaze 就会打架。修法：给云台同款 driver 标签（gaze/geometry/manual）。

### 15.3 雷达接入四条规则（守住就不打架）

1. **永远只否决/限速，绝不发起运动**（既设计，勿破例——雷达一旦会"开车"，仲裁复杂度上一个数量级）。
2. **阈值一张表 + 不变量测试**：跟随停距 0.7 > 雷达 fwd 介入 ≈0.59 > HRI goto 0.30 > 超声 0.20 > 硬间隙——写成一张全局表并加不变量断言（跟随停距必须大于雷达介入，否则"想停在 0.7"永远被雷达提前否决）；"主人=障碍"的 11cm 薄裕量进实测协议（§3.2-2）。
3. **否决可观测**：每条被改写的命令记 vetoed_by（同 15.2-2）。
4. **rot 否决 × TURN 通道是已知雷**：rot 迟滞锁死 bug（§1.3②，未修）会让雷达一次触发后 passby 转向永久锁死——**修 rot 迟滞是雷达上车硬前置**。

### 15.4 一句话优先级梯子（现状即应如此，文档化即可）

**急停 /stop > 安全约束（超声/雷达 veto）> 模式所有权（manual/follow 互斥）> HRI 有界行为 > follow 弧线 > 手动测试（T0/T8 接管前先踢 follow）**。现状代码与这条梯子一致；把它写进 ARCHITECTURE.md 防后人破例。

---

## 16. 统筹批准与执行（2026-10-24，统筹直接执行）

按 §7 白名单 + §14.3/§11-b，已执行三项（commit `8f713eb`，**未推**）。

### 16.1 已执行

1. **`/test` 面板预设（§14.3-1/2）**：删除过期“标准档”（BASE 220/MIN 120/PIVOT 190/W_DES 0.08 会降回带不动的档位）；唯一预设=当前重载档，`YAW_K=1.0`（其余 11 项与代码一致）；`setTurn` 加 confirm（§14.3-3，防 A/B 臂被误切）。
2. **host-side OBS 兜底（§11-b）**：`rk_control._obs_guard` 前置 `uart_send`——OBS 新鲜且 `blocked` 或 `cm≤35` 时，`MOVE F` 与**前进 DIFF（L+R>0）**即时改 `STOP`；`MOVE B / STOP / DIFF L0 R0` 放行；`/healthz` 暴露 `obs_guard_veto`；`test_control_glue` 新增 F 段（blocked/cm30/DIFF/后退/过期五态）。
3. **文档 N1/N2/N7 + N4/N5/N8**：`FIELD_TEST_PLAN` 速度档统一到重载 245/155/240/255；`DEFERRED_WORK §1.4` 注明 10-20 起 `TURN_ENABLED=True`；playbook L1 改“需统筹批准”；watchdog 能力表拆“枚举级/真掉电”；开机自启表补齐 6 服务+timer；旧速度数字标历史。

### 16.2 回归

- scripts **11 套** + world/fusion **4 套** = **15/15 PASS**；HTML JS `node --check` 通过。
- 未跑板端（AP-only，SSH 超时）。

### 16.3 未批准/押后

- §11-c 参数改动（**观察不改**，V1 速度冻结）；§11-e 不做项。
- §15.2 驱动所有权重构、`vetoed_by` 归因（V2）。
- 固件、雷达/V2 几何、§13 的 V2 项、仿真重构。

### 16.4 板端待办（上电后）

1. sync + `systemctl restart rk-control`；
2. smoke：`/move` 门禁（follow 拒 / manual 放行）、`/healthz.uart.fail`、`/healthz.obs_guard_veto`；
3. **T0b `pivot -30` 必须向右**；T9 前左右手镜像实测；
4. T4 按 §12.1：**1.2m 正常左右 → 0.8m 慢左右 → 1.5m → 快步速**；`_low_s` 与 OBS 兜底都会改跟随行为 → **T4/T10 必须重跑**。
