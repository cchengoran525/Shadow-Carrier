#!/usr/bin/env python3
"""gimbal_follow.py v2 - 云台追随最大person (C3串口直驱版)

两自由度舵机(Pan/Tilt)自动追随画面中面积最大的person。
舵机由 ESP32-C3 的 LEDC 硬件PWM驱动, 本脚本只发角度命令, 不再碰GPIO。

接线见 gimbal/README.md:
  Pan(水平270°) 橙线 -> C3 GPIO0   Tilt(俯仰180°) 橙线 -> C3 GPIO1
  舵机红棕线 -> 独立5V电源, 与C3共地

用法: python3 gimbal_follow.py [--port 8080] [--uart /dev/c3_controller]
权限: 需要读写串口 (sudo 或把用户加入 dialout 组)
"""
import argparse
import json
import os
import termios
import threading
import time
import urllib.request

# ================= 配置 =================
API_URL = "http://127.0.0.1:8080/api/detections"
UART_PATH = "/dev/c3_controller"
UART_BAUD = 115200

W, H = 640, 480          # 画面尺寸 (与 video_stream_v7 一致)

PAN_CENTER = 90.0        # 上电中位角 (Pan 实测为 180° 舵机)
TILT_CENTER = 90.0
PAN_RANGE = 180.0        # 行程限幅
TILT_RANGE = 180.0
PAN_K = 0.05             # P控制增益: 像素偏差 -> 角度
TILT_K = 0.04
SEND_HZ = 20             # 命令发送频率
POLL_S = 0.03            # API轮询间隔
# ========================================


class C3Link:
    """极简串口: termios 直用, 不依赖 pyserial (与 rk_control.py 同风格)"""

    def __init__(self, path, baud):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{baud}")
        attrs[0] = termios.IGNBRK          # iflag
        attrs[1] = 0                       # oflag
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD  # cflag
        attrs[3] = 0                       # lflag: 非规范模式, 收发即达
        attrs[4] = speed                   # ispeed
        attrs[5] = speed                   # ospeed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def send(self, line):
        try:
            os.write(self.fd, (line + "\n").encode())
        except OSError as e:
            print(f"串口写入失败: {e}", file=__import__("sys").stderr)

    def drain(self):
        """丢弃C3的回显(GOT:/PAN:/TLT:), 防止缓冲堆积"""
        try:
            while os.read(self.fd, 256):
                pass
        except BlockingIOError:
            pass

    def close(self):
        os.close(self.fd)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def fetch_biggest_person():
    """从v7 API取面积最大person框, 无则None"""
    try:
        req = urllib.request.Request(API_URL, headers={"User-Agent": "gimbal"})
        d = json.loads(urllib.request.urlopen(req, timeout=1.5).read())
    except Exception:
        return None
    best = None
    for det in d.get("detections", []):
        if det.get("c") != "person":
            continue
        area = (det["x2"] - det["x1"]) * (det["y2"] - det["y1"])
        if best is None or area > best["area"]:
            best = dict(det, area=area)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=API_URL)
    ap.add_argument("--uart", default=UART_PATH)
    args = ap.parse_args()

    print("gimbal_follow v2: 云台追随最大person (C3硬件PWM)")
    print(f"  串口: {args.uart} @ {UART_BAUD}")
    print(f"  依赖: {args.url} (video_stream_v7)")

    link = C3Link(args.uart, UART_BAUD)

    # 上电回中位, 连发几次确保收到
    pan, tilt = PAN_CENTER, TILT_CENTER
    for _ in range(3):
        link.send(f"PAN {pan:.1f}")
        link.send(f"TLT {tilt:.1f}")
        time.sleep(0.05)
    print(f"已回中位: PAN={pan} TILT={tilt}")

    interval = 1.0 / SEND_HZ
    last_send = 0.0
    print("运行中... Ctrl+C 退出")
    try:
        while True:
            person = fetch_biggest_person()
            now = time.monotonic()
            if person is not None and now - last_send >= interval:
                cx = (person["x1"] + person["x2"]) / 2
                cy = (person["y1"] + person["y2"]) / 2
                dx = cx - W / 2
                dy = cy - H / 2
                pan = clamp(pan + dx * PAN_K, 0.0, PAN_RANGE)
                tilt = clamp(tilt + dy * TILT_K, 0.0, TILT_RANGE)
                link.send(f"PAN {pan:.1f}")
                link.send(f"TLT {tilt:.1f}")
                last_send = now
            link.drain()
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        link.close()


if __name__ == "__main__":
    main()
