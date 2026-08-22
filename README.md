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

- `DistributedRobot_S3_Gateway/`: ESP32-S3 Network Gateway for the original WiFi-to-UART control path.
- `DistributedRobot_C3_MotionController/`: ESP32-C3 Motion Controller for the original GPIO UART path. It parses the shared ASCII command format, drives the TB6612 dual motor driver, and stops forward motion when the front ultrasonic sensor detects an obstacle.
- `shadow_carrier_on_rockchip/`: **KickPi (RK3566) vision brain** — the current decision maker. YOLO NPU detection pipeline, 2-axis servo gimbal driven by the C3, and an extended ASCII protocol (`MOVE`/`STOP`/`DIFF`/`PING`/`PAN`/`TLT`) over USB CDC to a dedicated C3 USB firmware.
- The C3/S3 firmware stays low-level (no AI); AI and behavior run on the KickPi brain (below).

### v0.5 C3 Direct-Drive Gimbal (`shadow_carrier_on_rockchip`)

The 2-axis camera gimbal is driven by the **ESP32-C3's hardware PWM** (LEDC 50Hz @14bit) instead of
RK3566 GPIO software PWM — RK3566's header has no free pins for servos (measured: GPIO4_A6/A7 are
not routed to the 30-pin header; all other free pins are claimed by UART7/UART9/I2C/I2S/PDM).

| Area | What it does |
|---|---|
| **Firmware** | `shadow_carrier_on_rockchip/C3_USB_Controller/` — adds `PAN <deg>` / `TLT <deg>` commands with range clamping, direction/pulse-width calibration constants, power-on centering |
| **Servo power** | Independent 5V from drive battery via buck module, common ground with C3 (never power servos from the board) |
| **Web control** | Drag pad on the control page: horizontal = pan, vertical = tilt, continuous 10Hz absolute-angle stream; ◎ re-centers |
| **Auto follow** | `gimbal/gimbal_follow.py` v2 — P-control on the largest person, sends angles over the same USB serial link |

See `shadow_carrier_on_rockchip/gimbal/README.md` for the full wiring/pinout investigation and calibration guide.

### v0.4 USB Direct Link

The S3→C3 UART hop was replaced by **USB CDC ACM direct** (KickPi USB-A → C3 USB-C): zero extra
hardware, `/dev/ttyACM0`, udev alias `c3_controller`. Five alternatives failed first — see
`shadow_carrier_on_rockchip/communication/README.md` for the post-mortem.

### v0.3 KickPi Vision Brain (`shadow_carrier_on_rockchip`)

The S3 gateway is being replaced by a **KickPi RK3566** running a real-time vision brain:

| Area | What it does |
|---|---|
| **Perception** | USB camera → YOLOv8 (RKNN NPU, 0.8 TOPS) → detection pipeline ~11 FPS, with crash-recovering watchdog and memory-disk frames (no disk I/O) |
| **Gimbal** | 2-axis servo pan/tilt — now driven by the C3 over USB (see v0.5 below); RK3566 header pins proved unusable for servos |
| **Communication** | Sends ASCII motion commands (`MOVE F 180` / `STOP` / `PING`) to the C3 USB firmware over USB CDC |
| **Roadmap** | Owner identification (BLE RSSI + HSV color + posture fusion), person-following loop, ESP32 serial control |

The full project lives in `shadow_carrier_on_rockchip/` (perception pipeline, gimbal, C3 protocol spec).

### Current Engineering Notes

The RK3566 path is currently a working demo pipeline, not yet a self-contained production deployment. The known boundaries are recorded in detail in [`shadow_carrier_on_rockchip/README.md`](shadow_carrier_on_rockchip/README.md), including:

- The RK path uses USB CDC to the dedicated `C3_USB_Controller` sketch. The old `communication/` sketch is a duplicate legacy copy and does not include the latest `DIFF` command.
- RK serial reconnect and `PING/PONG` health verification are not complete. The current control service reports whether the device opened, but does not expose a confirmed protocol handshake.
- The vision service depends on an externally built YOLO daemon and model paths on the RK board; the native C++ camera and YOLO wrappers remain TODO placeholders.
- Follow mode currently uses image horizontal position for steering. Several distance/PID settings are reserved but are not yet connected to the active controller.
- Ultrasonic obstacle protection contains timeout-based force-clear paths to avoid permanent lockout. These paths are not equivalent to confirming that the path is clear.
- The RK control page and gimbal scripts still contain a few deployment-specific IP/GPIO references that must be verified against the actual board image and wiring.

### Shared ASCII Motion Protocol

The command format is transport-independent: ASCII text, one command per line. The original S3 path transports it over GPIO UART; the current RK3566 path transports it over USB CDC.

```text
MOVE F 180
MOVE B 180
MOVE L 150
MOVE R 150
STOP
PING
```

### Legacy S3-C3 UART Wiring

| Link | TX | RX | Baud |
|---|---:|---:|---:|
| ESP32-S3 Gateway | GPIO1 | GPIO2 | 115200 |
| ESP32-C3 Motion Controller | GPIO11 | GPIO10 | 115200 |

Wire S3 TX GPIO1 to C3 RX GPIO10, S3 RX GPIO2 to C3 TX GPIO11, and connect GND to GND.

This wiring applies only to the original ESP32-S3 Gateway path. The current RK3566 path does not use the RK board header UART or GPIO pins because those pins are not convenient for the physical RK-C3 connection.

### Current RK3566-C3 USB Connection

Connect the KickPi/RK3566 USB host port directly to the ESP32-C3 USB port:

```text
RK3566 USB-A  ->  ESP32-C3 USB-C
```

The C3 appears on RK Linux as a USB CDC device, normally `/dev/ttyACM0`. This path is USB communication and USB power; it is not the old GPIO UART wiring. Compile the USB firmware with `CDCOnBoot=cdc` and use `shadow_carrier_on_rockchip/C3_USB_Controller/` as the C3 sketch.

### S3 Control WiFi

The S3 creates its own control network:

```text
SSID: ShadowCarrier-S3
Password: shadow123456
Control page: http://192.168.4.1
```

### C3 TB6612 Wiring

| TB6612 Pin | ESP32-C3 GPIO |
|---|---:|
| STBY | GPIO20 |
| AIN1 | GPIO4 |
| AIN2 | GPIO5 |
| PWMA | GPIO6 |
| BIN1 | GPIO7 |
| BIN2 | GPIO8 |
| PWMB | GPIO9 |

### C3 Ultrasonic Wiring

Use a 3.3V-safe echo signal for the ESP32-C3 input. If the ultrasonic module outputs 5V on ECHO, add a voltage divider before GPIO3.

| Ultrasonic Pin | ESP32-C3 GPIO |
|---|---:|
| TRIG | GPIO2 |
| ECHO | GPIO3 |

### C3 Safety Behavior

- The S3 web page sends repeated MOVE commands while a direction button is held, and sends STOP when released.
- The C3 stops if it does not receive a fresh movement command within 450 ms.
- Forward movement is blocked as soon as the ultrasonic distance is 20 cm or less. The block clears only after 3 valid samples at 25 cm or farther.
- Motor PWM ramps gradually to reduce startup jerk and wheel slip.
- TB6612 `STBY` is held LOW while stopped, and is enabled only when non-zero PWM is being applied.
- Open-loop wheel trim is configured in `DistributedRobot_C3_MotionController/Config.h`.

If a motor spins briefly right after power is connected, add a physical pulldown resistor on TB6612 `STBY` to GND. Firmware pulls STBY low early, but it cannot control pin levels before the ESP32-C3 has booted.

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
