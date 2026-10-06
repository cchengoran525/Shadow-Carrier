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

## Layout

```text
webots/
├── worlds/shadow_carrier.wbt
├── controllers/shadow_carrier/shadow_carrier.py
├── run_demo.sh         (macOS/Linux; motion mode, full demo scenario ~95 s)
├── run_autotest.sh     (macOS/Linux; observe mode, state-machine regression)
└── run_autotest.bat    (Windows)
```
