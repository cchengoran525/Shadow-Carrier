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

Before upload, edit `DistributedRobot_S3_Gateway/Config.h` and set:

```cpp
constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
```

After upload, open Serial Monitor and copy the printed IP address into your browser.

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

The C3 firmware prints received and parsed UART commands. It does not drive motors in v0.2.

## UART Test

Connect the boards:

| ESP32-S3 | ESP32-C3 |
|---|---|
| GPIO1 TX | GPIO10 RX |
| GPIO2 RX | GPIO11 TX |
| GND | GND |

Then:

1. Upload the C3 firmware and keep its Serial Monitor open at 115200 baud.
2. Upload the S3 firmware.
3. Open the S3 IP address in a browser.
4. Press `Forward`, `Back`, `Left`, `Right`, or `Stop`.
5. Confirm the C3 Serial Monitor shows commands such as:

   ```text
   UART <- S3: MOVE F 180
   Parsed command: MOVE F speed=180
   ```

## Common Notes

- If upload fails, lower upload speed to `115200`.
- If Serial Monitor output is unreadable, confirm it is set to `115200`.
- If the S3 web page does not load, confirm WiFi credentials and check the IP printed by Serial Monitor.
- Do not install camera, AI, motor-control, or BLE libraries for v0.2 unless a later firmware version needs them.
