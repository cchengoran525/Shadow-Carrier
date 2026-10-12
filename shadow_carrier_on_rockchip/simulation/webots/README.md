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

The controller emulates the real-machine cross-process owner lock: the follow side (simulated template = the `OWNER` node, told apart from the blue `PASSERBY` by recognition color) publishes `{ts, bbox}` to a lock file with the same format and 0.2 s rate limit as `follow_controller._publish_owner`, and the HRI side consumes it through the unmodified `hri_state._owner_box_from_file` path (`owner.source=file`). When the owner is not visible the file goes stale and HRI falls back exactly as it would on the board. When the owner is invisible during `WAIT` the robot turns toward the owner's true position (search gaze); during `HIDE` (after parking) it keeps facing the owner (rescue gaze).

## Simulated Interfaces

- Differential drive (kinematic), C3-style motor ramping, and the 450 ms command timeout.
- The current line protocol: `MOVE`, `DIFF`, `STOP`, `PING`, `PAN`, and `TLT`.
- Corridor world: dorm door, side door, water dispenser, refrigerator, bench; two-axis camera gimbal, front camera, front sonar; movable owner, movable passerby, hand-held bottle.
- The existing `FollowController` and HRI state machine, fed detections shaped as `label/conf/bbox` (autotest synthesizes them from ground-truth 3D positions via a pinhole model with lateral projection; interactive mode uses Webots recognition with real occlusion).
- The follow side keeps a ~2.5-3 m distance to the owner (sim-only comfort feature, ground-truth gated); the grid snapshot embeds a `sector_scores` block produced by the real `world/fusion/sector_score.py`.

Use this to iterate behavior and interfaces, then validate timing and calibration on the physical robot.

## Robustness Suite（8 场景，2026-10-12/13）

`run_robustness.sh`（跑测）+ `analyze_robustness.py`（判定）+ 控制器 `SIM_SCENARIO` 场景系统，realtime 逐场景自动判定。**当前 8/8 PASS**。

| 场景 | 扰动 |
|---|---|
| `baseline` | 回归基线 |
| `cross2` | 推进段路人横穿"主人-车"连线 |
| `occlude` | 主人躲冰箱后 8s（真遮挡） |
| `bench_block` | 长椅挡车道（考验虚拟声呐 + HIDE 回退链） |
| `handoff_cross` | 交接窗口路人垂直穿越视线 |
| `hide_occupied` | 路人常驻首选躲避点（考验"换扇区回退"） |
| `phantom` | 幻影注入（复刻宿舍夜景 95/98 帧失败模式，`SIM_PHANTOM_SEED` 可复现） |

```bash
./run_robustness.sh                              # 全场景 × 1 次（~11 min）
./run_robustness.sh "baseline hide_occupied" 3   # 指定场景 × 3
python3 analyze_robustness.py logs/rob_*.jsonl   # 单独出报告
# Windows 单场景: _run_scenario.bat <scenario> [phantom_seed] [phantom_rate]
```

**三条纪律（踩过的坑）**：
1. **必须 `--mode=realtime`**——`fast` 模式墙钟与仿真时间脱钩 3.4×，时序阈值全乱（仓库旧"参考日志"即 fast 失败产物）。
2. **HIDE 候选点（如 `SPOT_B`）必须落在"无门 / 无家电 / 无人"且**墙内**的扇区**——否则会被亲和度打分正确拒绝（曾两次踩：先与冰箱同扇区、再越出左墙）。
3. `logs/` 为测试产物，不入库。

`Simulated Interfaces` 补充：Webots 原生 sonar 在本世界失效，控制器用**几何真值虚拟声呐**（5 射线 ±10°）；几何真值检测带**高度感知遮挡**（路人可真挡住主人框，owner lock 被真考验）。

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
