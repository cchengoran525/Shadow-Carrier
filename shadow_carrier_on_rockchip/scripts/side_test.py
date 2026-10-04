#!/usr/bin/env python3
"""单侧扭矩量化 (诊断用, 临时): 分别只驱动左/右轮, 观察哪侧弱"""
import os
import termios
import time

PORT = "/dev/ttyACM0"


def send(fd, s):
    os.write(fd, (s + "\r\n").encode())


def burst(fd, cmd, secs=1.5):
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        send(fd, cmd)
        time.sleep(0.2)
    send(fd, "STOP")
    time.sleep(0.5)


def main():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY)
    attr = termios.tcgetattr(fd)
    attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attr[3] = 0
    attr[4] = attr[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    print(">>> 左轮 DIFF L200 R0 (1.5s)", flush=True)
    burst(fd, "DIFF L200 R0")
    print(">>> 右轮 DIFF L0 R200 (1.5s)", flush=True)
    burst(fd, "DIFF L0 R200")
    print(">>> 双轮 DIFF L200 R200 (1.5s)", flush=True)
    burst(fd, "DIFF L200 R200")
    print(">>> 完成", flush=True)
    os.close(fd)


if __name__ == "__main__":
    main()
