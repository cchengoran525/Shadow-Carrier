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
