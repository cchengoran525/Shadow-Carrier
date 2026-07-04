# Shadow Carrier 逛街搭子
 
> *"不知道啥时候来的，我顺手把外套挂上去了。"*
 
A proactive shopping companion robot that predicts separation before it happens, waits somewhere findable, and reunites without being asked.
 
**Portfolio Project #3 — Automation × HRI**
 
---
 
## Motivation
 
You finish buying something, hands full, and the last thing you want is to babysit a robot. Existing follower products (Gita, smart luggage) solve the "following" problem — but not the "actually feeling like a companion" problem.
 
The real goal: **make the user forget it's a robot.**
 
> "The boss just needs to enjoy shopping. All complexity is transparent to the user."
 
---
 
## Core Insight: Implicit Interaction
 
The user reaches for their milk tea → Shadow Carrier predicts this and moves alongside to receive it.
 
No buttons. No voice commands. No app. **Body language is the interface.**
 
This design philosophy is called **Implicit Interaction** — the user has no conscious awareness of operating the device, but the device is continuously reading their intent.
 
---
 
## What Sets It Apart
 
| Dimension | Existing Products | Shadow Carrier |
|---|---|---|
| Following mode | Reactive (you moved, now I move) | Anticipatory (predict before you move) |
| Handling separation | Keep following or alert | Self-preserve, then return |
| User burden | Occasional glance-back required | Completely invisible |
| Spatial awareness | None | Proxemics sensing (beside / behind) |
 
---
 
## Research Framework: "Inevitable Separation"
 
Existing following research asks *how to stay close*. This project asks **what happens when you're separated** — a more honest model of what a real companion does.
 
### Pre-Separation: Can it predict?
 
- Crowd density rising → move closer instead of trailing behind
- Changes in your gait/pace → anticipate turns or stops
- Spatial topology changes (doors, elevators, stairs) → proactive response
**Research Question:** Can separation risk be quantified as a continuous value and predicted in real time?
 
### During Separation: Where does it wait?
 
- Not blocking foot traffic
- Within your line of sight / at a predictable location
- Safe from being kicked or knocked over
- **Implicit contract:** you know where it'll be — you don't need to look for it, you naturally walk toward it
*Core challenge: formalizing the implicit contract. Pure control theory can't solve this — user research is required.*
 
### After Separation: How does it reunite?
 
- Navigates actively to your phone's location
- Moves toward you proactively rather than waiting for you to arrive
- Reunion experience directly determines user trust
---
 
## Technical Architecture
 
| Layer | Function | Approach | Course Context |
|---|---|---|---|
| Perception | Where are you / crowd density | Phone BLE/WiFi RTT + LiDAR | SDM273 |
| Localization | Relative position + noise filtering | Kalman filter | SDM366 |
| Decision | Separation prediction + wait point selection | MPC + online optimization | EE346 |
| Actuation | Low-level motor control | Differential drive + embedded | SDM358 |
 
### User Side: Zero Extra Hardware
 
Only requires the phone you already carry (BLE + IMU). No tags, no wearables.
 
- **Phone IMU** → gait/turn prediction
- **WiFi RTT** → coarse localization
- **Robot camera** → visual tracking (precise)
Fusion: phone signal maintains direction during occlusion; vision takes over when line-of-sight is clear.
 
---
 
## MVP (¥50 Budget)
 
**Core hypothesis to validate:** "It knows when you've gone too far, and it comes after you."
 
| Component | Purpose | Cost |
|---|---|---|
| RC car chassis (stripped) | Mobile platform | ¥30–40 |
| L298N motor driver | Motor control | ¥10 |
| ESP32-C3 (on hand) | BLE ranging + control | ¥0 |
| Phone BLE broadcast | Distance signal | ¥0 |
| **Total** | | **¥40–50** |
 
**Logic:** Phone broadcasts BLE → ESP32 reads RSSI → signal weakens, drive forward → signal strengthens, stop.
 
**Acceptance test:** Follow a person 5 meters down a dormitory hallway. Capture video.
 
---
 
## Research Value
 
Each sub-problem is independently publishable in a controls venue:
 
- Separation risk quantification model (stochastic processes + Bayesian estimation)
- Safe wait-point selection in real time (constrained online optimization)
- Anticipatory following control law (MPC application)
- Reunion trajectory planning (dynamic environment path planning)
**The differentiating position:**
 
Automation depth + genuine care about "does this feel right to the user" = a rare combination in HRI. Pure HRI researchers lack the control theory depth. Pure automation researchers don't care about user experience. The gap between them is the opportunity.
 
---
 
## Generalization
 
The **Separation–Wait–Reunion** model abstracts into a general theory of human-robot physical interruption, applicable to:
 
| Domain | Why it matters |
|---|---|
| Elderly / post-op care | Zero cognitive burden is a hard requirement, not a nice-to-have |
| Wheelchair users | Hands already occupied; physical carrying is critical |
| Airport / exhibition luggage | Clean scenario, predictable crowd flow, lower robustness demands |
| Factory material following | B2B market, clear commercialization path |
 
The framework is worth more than the shopping cart.
 
---
 
## Roadmap
 
| Phase | Goal | Prerequisites |
|---|---|---|
| Now → Sophomore | ¥50 MVP: hallway following | HappyMac / other demos first |
| Sophomore → Junior | Read core literature, identify specific research entry point | Control theory, Kalman filter |
| Junior (Spring) | Add EE346 mobile robotics, build comparative experiment version | SDM366 optimal estimation |
| Senior | Thesis or paper based on this | Full stack in place |
 
---
 
## Project Status

🟡 **Pre-MVP / v0.2 firmware bring-up**

Current implementation focuses on the low-level distributed robot platform:

- `DistributedRobot_S3_Gateway/`: ESP32-S3 Network Gateway. Connects WiFi, serves a simple HTTP control page, and forwards button actions to UART as ASCII protocol commands.
- `DistributedRobot_C3_MotionController/`: ESP32-C3 Motion Controller. Receives UART lines, parses the shared protocol, and prints parsed commands to Serial Monitor.
- Current firmware intentionally does not enable the camera, AI, motor output, or complex behavior logic.

### v0.2 UART Protocol

Commands are ASCII text, one command per line:

```text
MOVE F 180
MOVE B 180
MOVE L 150
MOVE R 150
STOP
PING
```

### UART Wiring

| Link | TX | RX | Baud |
|---|---:|---:|---:|
| ESP32-S3 Gateway | GPIO1 | GPIO2 | 115200 |
| ESP32-C3 Motion Controller | GPIO11 | GPIO10 | 115200 |

Wire S3 TX GPIO1 to C3 RX GPIO10, S3 RX GPIO2 to C3 TX GPIO11, and connect GND to GND.

### Arduino IDE

See [docs/arduino_ide_setup.md](docs/arduino_ide_setup.md) for board package, board selection, and upload settings.
 
---
 
## Related Work
 
- Gita (Piaggio) — reactive cargo follower
- Smart luggage (various) — handle-tethered following
- Proxemics in HRI — personal space modeling
- Model Predictive Control for mobile robots
- BLE/WiFi RTT indoor localization
---
