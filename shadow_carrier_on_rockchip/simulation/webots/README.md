# Webots Simulation

This project runs on a desktop with Webots; the RK3566 board, camera, C3, and chassis are not required.

## Open

Install Webots for macOS or Windows, then open `worlds/shadow_carrier.wbt` from Webots. The world starts the `shadow_carrier` Python controller automatically. The controller imports the repository's existing `hri_state.py`; it only needs Python's standard library plus Webots' `controller` module.

## Demo Scene (corridor)

The world is the demo scenario: a dormitory corridor with the dorm door (left wall), a water dispenser at the far end, and the "next door" refrigerator beside it. Press `G` (or run `run_demo.sh`) and the scripted owner walks the full demo: 出宿舍门 → 打水(静止) → 车躲到饮水机旁墙根(离门远) → 路人从宿舍门进来 → 到冰箱取物回身递给车 → 车上前接住 → 一起回宿舍. The whole run is ~95 s and exercises `FOLLOW / WAIT / HIDE / RECEIVE / YIELD`, the owner lock, and HIDE affinity scoring end to end.

The chassis is kinematic (the world has no `physics` node): the controller integrates differential-drive kinematics from the C3-style wheel commands. This keeps the protocol semantics (ramp, 450 ms timeout, `DIFF`/`MOVE`/sonar stop) while avoiding solver instabilities on macOS. Wheel visual spin and collisions are not simulated.

## Controls

| Key | Action |
|---|---|
| `W` / `S` | Forward / backward |
| `A` / `D` | Turn left / right |
| `Space` | Stop |
| `G` | Owner autopilot: scripted demo scenario on/off |
| `I` / `K` | Move the simulated owner closer / farther |
| `J` / `L` | Move the simulated owner left / right |
| `Arrow keys` | Move the simulated passerby (second person) |
| `P` | Show / hide the passerby |
| `O` | Show / hide the held bottle |
| `H` | Hide / restore the owner |
| `T` | Enable / disable HRI automation |
| `,` / `.` | Pan the camera left / right |
| `-` / `=` | Tilt the camera down / up |
| `R` | Reset the scene and HRI state |

## Owner Lock (HRI A1.2) and the Passerby

The controller emulates the real-machine cross-process owner lock: the follow side (simulated template = the `OWNER` node, told apart from the blue `PASSERBY` by recognition color) publishes `{ts, bbox}` to a lock file with the same format and 0.2 s rate limit as `follow_controller._publish_owner`, and the HRI side consumes it through the unmodified `hri_state._owner_box_from_file` path (`owner.source=file`). When the owner is not visible the file goes stale and HRI falls back exactly as it would on the board. When the owner is invisible during `WAIT` the robot turns toward the owner's true position (search gaze); during `HIDE` (after parking) it keeps facing the owner (rescue gaze). **Both are ground-truth theatre** (they write `_kin_yaw` directly, no wheel motion): set `SIM_GT_TURN=0` to disable them — required when the point of the run is "did the robot turn by itself" (e.g. the `passby` scenario).

## Simulated Interfaces

- Differential drive (kinematic), C3-style motor ramping, and the 450 ms command timeout.
- The current line protocol: `MOVE`, `DIFF`, `STOP`, `PING`, `PAN`, and `TLT`.
- Corridor world: dorm door, side door, water dispenser, refrigerator, bench; two-axis camera gimbal, front camera, front sonar; movable owner, movable passerby, hand-held bottle.
- The existing `FollowController` and HRI state machine, fed detections shaped as `label/conf/bbox` (autotest synthesizes them from ground-truth 3D positions via a pinhole model with lateral projection; interactive mode uses Webots recognition with real occlusion).
- **Horizontal FOV culling (2026-10-16)**: `_project_box` now also rejects targets outside the REAL camera's horizontal field of view (±32.2°, `tan=0.63`, i.e. FX=508.2 at 640 px) and logs `fov_exit`. Before this the projection was culled by depth only, so an owner 80° off-axis still produced a box at `cx≈6000 px` and the sim could never lose the owner laterally — the single most important sim→real gap for follow. `SIM_TURN=1` additionally enables the real `follow_controller` turn channel (TRACK/TURN/BLIND/SEARCH, pivot via the real `MOVE L/R` semantics) and feeds it the sim's IMU equivalents plus a sim-faithful bearing decoder.
- The follow side keeps a ~2.5-3 m distance to the owner (sim-only comfort feature, ground-truth gated); the grid snapshot embeds a `sector_scores` block produced by the real `world/fusion/sector_score.py`.

Use this to iterate behavior and interfaces, then validate timing and calibration on the physical robot.

## Robustness Suite（9 个场景码，2026-10-12/13/16）

`run_robustness.sh`（跑测）+ `analyze_robustness.py`（判定）+ 控制器 `SIM_SCENARIO` 场景系统，realtime 逐场景自动判定。**默认集 8/8 PASS**（`lidar_sweep_block` 需 `SIM_LIDAR=1`，单独验：2026-10-16 PASS）。⚠️ 2026-10-16 加了水平 FOV 裁剪（见上）→ 老场景的绿若变化，那是**真机也有的问题**被暴露出来，按"修复向前"处理，不要回退裁剪。

| 场景 | 扰动 |
|---|---|
| `baseline` | 回归基线 |
| `cross2` | 推进段路人横穿"主人-车"连线 |
| `occlude` | 主人躲冰箱后 8s（真遮挡） |
| `bench_block` | 长椅挡车道（考验虚拟声呐 + HIDE 回退链） |
| `handoff_cross` | 交接窗口路人垂直穿越视线 |
| `hide_occupied` | 路人常驻首选躲避点（考验"换扇区回退"） |
| `phantom` | 幻影注入（复刻宿舍夜景 95/98 帧失败模式，`SIM_PHANTOM_SEED` 可复现） |
| `lidar_sweep_block` | **V2 雷达 360° 互补验证**（需 `SIM_LIDAR=1`）：路人站在车正后方 0.40m——正前超声看不见，雷达应否决原地转向 |
| `passby` | **擦身掉头验收**（需 `SIM_TURN=1` + `SIM_GT_TURN=0`，`run_robustness.sh` 会自动带上）：t=6~12s 主人沿直线从车正前走到正后，最近 1.0m 擦身。判定：转向通道启动、擦身期间最长不可见 ≤2.5s、方向反转 ≤3 |

**passby A/B 实测（2026-10-16，`./run_passby.sh`）**

| 组 | 最长不可见 | 可见率 | 转向帧 | 结局 |
|---|---|---|---|---|
| `SIM_TURN=1`（转向通道开） | 0.62 / **2.40** / **2.79** s（3 跑） | 0.89 / 0.58 / 0.55 | 9 / 14 / 2 | 方向性 PASS，但**跨过 2.5s 判据** |
| `SIM_TURN=0`（现状基线） | **4.03 s**（1 跑） | 0.35 | 0 | 跟丢 → HRI 退 WAIT→HIDE 放弃 |

⚠️ **同配置重复 3 次抖动 0.62→2.79s（4.5 倍）** → 这个场景**单跑不能当门禁**，A/B 必须 N≥5 取中位数；
抖动源是仿真侧 HRI/控制拍用墙钟而剧本走仿真钟（负载一变就错相）。修法与纪律见
`docs/V2_LIDAR_FOLLOW_UPGRADE.md` §8。这条对照仍然是"云台冻结 + 相机 ±32° 时，主人从旁边过
必须靠底盘自转"的方向性证据，但**不要引用单跑数字**。

**⚠️ 未决项：`baseline`（含 FOV 裁剪后）判 FAIL —— 缺 RECEIVE（2026-10-16，留给 HRI 线）**

补上水平 FOV 裁剪后 `baseline` 变红，且**与转向通道无关**，实测证据：
`SIM_TURN=1 SIM_GT_TURN=0` 跑 baseline → HIDE 期间主人**仍被检测到**（`dets=['person']`，距 2.5 m），
follow 侧是 `turn_idle/TRACK`（θ<30°，不需要转头），全场 **0 帧进入转向相**；中断来自 HRI 的
`HIDE -> FOLLOW (主人消失, 扫掠中断)`。也就是说：递物交接起不来是 HIDE 的中断/扫掠判据在
"相机真的有 ±32° 视场"之后暴露出的连锁反应（旧仿真的无限视场假框把这条盖住了）。
`occlude` 在同样裁剪下 PASS，说明裁剪本身没有把套件打垮。**处理方式：不回退裁剪**（那是真机
也有的约束），由 HRI 线排查 HIDE 的中断判据/递物编排（或在 demo 编排上让主人绕到车正前方递物）。

```bash
./run_robustness.sh                              # 全场景 × 1 次（~11 min）
./run_robustness.sh "baseline hide_occupied" 3   # 指定场景 × 3
python3 analyze_robustness.py logs/rob_*.jsonl   # 单独出报告
# Windows 单场景: _run_scenario.bat <scenario> [phantom_seed] [phantom_rate]

# V2 虚拟雷达（与真机同一条 lidar_guard 代码路径；设计见 docs/V2_LIDAR_AVOIDANCE.md）
SIM_LIDAR=1 ./run_robustness.sh "lidar_sweep_block baseline" 1
SIM_LOG=/tmp/my_run.jsonl SIM_LIDAR=1 ./run_robustness.sh baseline 1   # 并行跑: 各自指定黑匣子

# 擦身掉头 A/B（转向通道 开 vs 关；两跑都关掉 ground-truth 掉头假动作）
./run_passby.sh
```

**三条纪律（踩过的坑）**：
1. **必须 `--mode=realtime`**——`fast` 模式墙钟与仿真时间脱钩 3.4×，时序阈值全乱（仓库旧"参考日志"即 fast 失败产物）。
2. **HIDE 候选点（如 `SPOT_B`）必须落在"无门 / 无家电 / 无人"且**墙内**的扇区**——否则会被亲和度打分正确拒绝（曾两次踩：先与冰箱同扇区、再越出左墙）。
3. `logs/` 为测试产物，不入库。**多人并行跑同一个 world 必须各自 `SIM_LOG=<路径>`**——默认路径是共享的 `sim_session.jsonl`，两进程交错写会产生截断的 JSON 行（解析器已容错跳过，但报告会失真）。

`Simulated Interfaces` 补充：Webots 原生 sonar 在本世界失效，控制器用**几何真值虚拟声呐**（5 射线 ±10°，2026-10-16 修正距离换算：原实现把米当段长比例，读数放大 2× 且 1m 外判"无交"）；几何真值检测带**高度感知遮挡**（路人可真挡住主人框，owner lock 被真考验）。
`SIM_LIDAR=1` 时另有一条**几何真值虚拟雷达**（360° × 2°、扫描面 13.5cm、8mm 噪声），写入与真机 daemon 同 schema 的帧、走同一个 `lidar_guard` → 命令过滤收口在 `_execute_ascii`（等价实机 `rk_control.uart_send` 前的那道闸）。

## Layout

```text
webots/
├── worlds/shadow_carrier.wbt
├── controllers/shadow_carrier/shadow_carrier.py
├── run_demo.sh            (macOS/Linux; motion mode, full demo scenario ~95 s)
├── run_autotest.sh        (macOS/Linux; observe mode, state-machine regression)
├── run_robustness.sh      (macOS/Linux; 8-scenario robustness suite + report)
├── analyze_robustness.py  (parse logs/rob_*.jsonl → per-scenario verdict + summary)
├── run_autotest.bat       (Windows)
├── _run_scenario.bat      (Windows; single scenario: <scenario> [seed] [rate])
└── _run_demo_once.bat     (Windows; one-shot full demo)
```
