# Webots Simulation

This project runs on a desktop with Webots; the RK3566 board, camera, C3, and chassis are not required.

## Open

Install Webots for macOS or Windows, then open `worlds/shadow_carrier.wbt` from Webots. The world starts the `shadow_carrier` Python controller automatically. The controller imports the repository's existing `hri_state.py`; it only needs Python's standard library plus Webots' `controller` module.

## Controls

| Key | Action |
|---|---|
| `W` / `S` | Forward / backward |
| `A` / `D` | Turn left / right |
| `Space` | Stop |
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

The owner starts in front of the robot. With HRI enabled, Webots camera recognition supplies bounding boxes to the existing `FOLLOW / WAIT / HIDE / RECEIVE / YIELD` state machine and `FollowController`. The simulated detector is injected through an optional provider; the physical follow path still uses its enrolled owner profile. The simulated camera supplies a perfect `person` label, so owner identity and real YOLO performance are not measured here.

## Owner Lock (HRI A1.2) and the Passerby

The controller emulates the real-machine cross-process owner lock: the follow side (simulated template = the `OWNER` node, told apart from the blue `PASSERBY` by recognition color) publishes `{ts, bbox}` to a lock file with the same format and 0.2 s rate limit as `follow_controller._publish_owner`, and the HRI side consumes it through the unmodified `hri_state._owner_box_from_file` path (`owner.source=file`). When the owner is not visible the file goes stale and HRI falls back exactly as it would on the board.

Use this to rehearse the A1.2 acceptance ("dual-person scene picks the right person"): show the passerby (`P`, arrow keys), park it closer/larger than the owner, and check that HRI keeps tracking the owner — `dist_m` in the frame log reflects the locked person's distance.

## Simulated Interfaces

- Differential drive, C3-style motor ramping, and the 450 ms command timeout.
- The current line protocol: `MOVE`, `DIFF`, `STOP`, `PING`, `PAN`, and `TLT`.
- A two-axis camera gimbal, front camera, front sonar, room, obstacle, movable owner, movable passerby, hand-held bottle, and a door landmark used by HIDE.
- The existing `FollowController` and HRI state machine, fed detections shaped as `label/conf/bbox`.
- The grid snapshot embeds a `sector_scores` block produced by the real `world/fusion/sector_score.py`, so `pick_safe_spot` exercises the same affinity scoring and confidence fallback as on the board.

The camera uses Webots' built-in object recognition to produce bounding boxes (the autotest instead synthesizes boxes from ground-truth 3D positions via a pinhole model with lateral projection); it does not run the RK3566 YOLO/NPU model. The sonar represents a 20 cm forward-motion stop. HIDE drives a simple 1 m waypoint along the selected door bearing; it does not perform path planning. C3 echo timing, the USB CDC link, real servo dynamics, wheel calibration, and physical floor friction are not reproduced. Use this to iterate behavior and interfaces, then validate timing and calibration on the physical robot.

## Layout

```text
webots/
├── worlds/shadow_carrier.wbt
├── controllers/shadow_carrier/shadow_carrier.py
├── run_autotest.sh     (macOS/Linux; ~110 s scripted run incl. dual-person owner-lock)
└── run_autotest.bat    (Windows)
```
