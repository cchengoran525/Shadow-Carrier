# Arduino IDE Setup

This project currently uses Arduino IDE and the official ESP32 Arduino core.

## Required Board Package

Install the ESP32 board package from Espressif:

1. Open Arduino IDE.
2. Go to `Arduino IDE > Settings`.
3. Add this URL to `Additional Boards Manager URLs`:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

4. Go to `Tools > Board > Boards Manager`.
5. Search for `esp32`.
6. Install `esp32 by Espressif Systems`.

No extra Library Manager packages are required for v0.2. The firmware only uses libraries included with Arduino and the ESP32 core, such as `WiFi.h` and `WebServer.h`.

## ESP32-S3 Gateway Upload

Open:

```text
DistributedRobot_S3_Gateway/DistributedRobot_S3_Gateway.ino
```

Recommended Arduino IDE settings:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| Upload Speed | 921600 or 115200 |
| CPU Frequency | 240MHz |
| Flash Mode | QIO |
| Flash Size | Match your board |
| Partition Scheme | Default |
| Serial Monitor | 115200 baud |

The S3 starts a local control WiFi network:

```text
SSID: ShadowCarrier-S3
Password: shadow123456
Control page: http://192.168.4.1
```

After upload, connect your phone or computer to `ShadowCarrier-S3`, then open `http://192.168.4.1` in a browser.

## ESP32-C3 Motion Controller Upload

Open:

```text
DistributedRobot_C3_MotionController/DistributedRobot_C3_MotionController.ino
```

Recommended Arduino IDE settings:

| Setting | Value |
|---|---|
| Board | ESP32C3 Dev Module |
| USB CDC On Boot | Enabled |
| Upload Speed | 921600 or 115200 |
| CPU Frequency | 160MHz |
| Flash Mode | QIO or DIO, matching your board |
| Flash Size | Match your board |
| Partition Scheme | Default |
| Serial Monitor | 115200 baud |

This sketch is the original GPIO UART C3 firmware. It receives commands from the ESP32-S3 on GPIO10/GPIO11 and drives a TB6612 dual motor driver.

TB6612 wiring:

| TB6612 Pin | ESP32-C3 GPIO |
|---|---:|
| STBY | GPIO20 |
| AIN1 | GPIO4 |
| AIN2 | GPIO5 |
| PWMA | GPIO6 |
| BIN1 | GPIO7 |
| BIN2 | GPIO8 |
| PWMB | GPIO9 |

Ultrasonic wiring:

| Ultrasonic Pin | ESP32-C3 GPIO |
|---|---:|
| TRIG | GPIO2 |
| ECHO | GPIO3 |

Use a voltage divider on ECHO if your ultrasonic module outputs 5V.

Open-loop tuning constants live in `DistributedRobot_C3_MotionController/Config.h`:

```cpp
constexpr uint32_t MOTOR_COMMAND_TIMEOUT_MS = 450;
constexpr int MOTOR_LEFT_TRIM_PERCENT = 0;
constexpr int MOTOR_RIGHT_TRIM_PERCENT = 0;
constexpr int OBSTACLE_STOP_DISTANCE_CM = 20;
constexpr int OBSTACLE_CLEAR_DISTANCE_CM = 25;
constexpr uint8_t OBSTACLE_CLEAR_CONFIRM_SAMPLES = 3;
```

If the robot veers left while driving forward, the left side is slower than the right side. Increase `MOTOR_LEFT_TRIM_PERCENT` or reduce `MOTOR_RIGHT_TRIM_PERCENT`. If it veers right, do the opposite. Start with small values such as `-5`, `0`, or `5`.

Obstacle stop is intentionally conservative: one close reading blocks forward movement, invalid readings do not clear the block, and several valid far readings are required before forward movement is allowed again. Backward and turning commands remain available so you can move the robot away from the obstacle.

## Legacy S3-C3 UART Test

Connect the boards:

| ESP32-S3 | ESP32-C3 |
|---|---|
| GPIO1 TX | GPIO10 RX |
| GPIO2 RX | GPIO11 TX |
| GND | GND |

Then:

1. Upload the C3 firmware and keep its Serial Monitor open at 115200 baud.
2. Upload the S3 firmware.
3. Connect to the `ShadowCarrier-S3` WiFi network.
4. Open `http://192.168.4.1` in a browser.
5. Press `Forward`, `Back`, `Left`, `Right`, or `Stop`.
6. Confirm the C3 Serial Monitor shows commands such as:

   ```text
   UART <- S3: MOVE F 180
   Parsed command: MOVE F speed=180
   ```

## RK3566-C3 USB Firmware

The current RK3566/KickPi path does not use the RK board header UART or GPIO pins. The RK board connects to the C3 directly through USB:

```text
RK3566 USB-A  ->  ESP32-C3 USB-C
```

For this path, open this separate sketch:

```text
shadow_carrier_on_rockchip/C3_USB_Controller/C3_USB_Controller.ino
```

Use `ESP32C3 Dev Module` with `USB CDC On Boot` enabled. With Arduino CLI, the board option is:

```text
esp32:esp32:esp32c3:CDCOnBoot=cdc
```

The C3 then receives the same ASCII motion commands through USB CDC. On RK Linux it normally appears as `/dev/ttyACM0`. Do not use the GPIO10/GPIO11 UART wiring for this USB path.

## Common Notes

- If upload fails, lower upload speed to `115200`.
- If Serial Monitor output is unreadable, confirm it is set to `115200`.
- If the S3 web page does not load, confirm your device is connected to `ShadowCarrier-S3` and open `http://192.168.4.1`.
- If a motor spins briefly as soon as power is connected, add a pulldown resistor from TB6612 `STBY` to GND so the motor driver stays disabled during ESP32-C3 boot.
- Do not install camera, AI, or BLE libraries for v0.2 unless a later firmware version needs them.
