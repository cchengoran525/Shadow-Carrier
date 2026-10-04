#!/usr/bin/env python3
"""imu_daemon.py - MPU 航向角速度读取器 (KickPi I2C3, 零依赖)

背景: 板上无 smbus 库且无外网 → 用 /dev/i2c-3 + I2C_RDWR ioctl 裸读写。
芯片: GY-521 实测 WHO_AM_I=0x70 (MPU6500 类兼容片, 非正宗6050; 不用DMP)
输出: /dev/shm/imu.json   {"ts","yaw_rate_dps","yaw_deg","gyro_raw","temp_c","quality"}
用法:
  sudo python3 imu_daemon.py --test      # 上电静置校准 + 打印3秒实时角速度, 退出
  sudo python3 imu_daemon.py             # 常驻, 50Hz 写 /dev/shm/imu.json
注意: 需要 i2c 读写权限 (sudo 或 SupplementaryGroups=i2c)
"""
import argparse
import ctypes
import fcntl
import json
import os
import struct
import time

I2C_SLAVE = 0x0703
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001

REG_WHO = 0x75
REG_GYRO = 0x43          # 0x43..0x48: gx,gy,gz (int16 BE)
REG_TEMP = 0x41
REG_CONFIG = 0x1A
REG_GYRO_CFG = 0x1B
REG_ACCEL_CFG = 0x1C
REG_PWR = 0x6B

GYRO_LSB_PER_DPS = 131.0   # ±250 dps 档
ALPHA = 0.35               # EMA 低通
YAW_LEAK = 0.99995         # 极慢泄漏, 限制积分漂移


class _i2c_msg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16), ("buf", ctypes.POINTER(ctypes.c_ubyte))]


class _i2c_rdwr_data(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(_i2c_msg)), ("nmsgs", ctypes.c_uint32)]


class MPU:
    def __init__(self, bus=3, addr=0x68):
        self.addr = addr
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def _xfer(self, msgs):
        arr = (_i2c_msg * len(msgs))(*msgs)
        data = _i2c_rdwr_data(ctypes.cast(arr, ctypes.POINTER(_i2c_msg)), len(msgs))
        fcntl.ioctl(self.fd, I2C_RDWR, data)

    def read(self, reg, n):
        wbuf = (ctypes.c_ubyte * 1)(reg)
        rbuf = (ctypes.c_ubyte * n)()
        m0 = _i2c_msg(self.addr, 0, 1, ctypes.cast(wbuf, ctypes.POINTER(ctypes.c_ubyte)))
        m1 = _i2c_msg(self.addr, I2C_M_RD, n, ctypes.cast(rbuf, ctypes.POINTER(ctypes.c_ubyte)))
        self._xfer([m0, m1])
        return bytes(rbuf)

    def write(self, reg, val):
        buf = (ctypes.c_ubyte * 2)(reg, val)
        m0 = _i2c_msg(self.addr, 0, 2, ctypes.cast(buf, ctypes.POINTER(ctypes.c_ubyte)))
        self._xfer([m0])

    def whoami(self):
        return self.read(REG_WHO, 1)[0]

    def begin(self):
        self.write(REG_PWR, 0x00)      # 唤醒, 内部时钟
        time.sleep(0.05)
        self.write(REG_CONFIG, 0x03)   # DLPF ~44Hz (降噪)
        self.write(REG_GYRO_CFG, 0x00) # ±250 dps
        self.write(REG_ACCEL_CFG, 0x00)
        time.sleep(0.05)

    def gyro_raw(self):
        d = self.read(REG_GYRO, 6)
        return struct.unpack(">hhh", d)

    def temp_c(self):
        d = self.read(REG_TEMP, 2)
        return struct.unpack(">h", d)[0] / 340.0 + 36.53


def calibrate(dev, seconds=2.0, rate=100.0):
    n, sums = 0, [0.0, 0.0, 0.0]
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        gx, gy, gz = dev.gyro_raw()
        sums[0] += gx; sums[1] += gy; sums[2] += gz
        n += 1
        time.sleep(1.0 / rate)
    return [s / max(n, 1) for s in sums]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=3)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x68)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--out", default="/dev/shm/imu.json")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--cal", type=float, default=2.0)
    args = ap.parse_args()

    dev = MPU(args.bus, args.addr)
    who = dev.whoami()
    dev.begin()
    bias = calibrate(dev, args.cal)
    bz = bias[2]
    print(f"[imu] whoami=0x{who:02x}  bias(raw x,y,z)={bias[0]:.1f},{bias[1]:.1f},{bz:.1f}"
          f"  -> z偏置 {bz / GYRO_LSB_PER_DPS:.2f} dps")

    yaw = 0.0
    yaw_rate = 0.0
    t_prev = time.monotonic()
    t_start = t_prev
    t_print = t_start

    while True:
        t = time.monotonic()
        dt = t - t_prev
        t_prev = t
        try:
            gx, gy, gz = dev.gyro_raw()
            r = (gz - bz) / GYRO_LSB_PER_DPS
            yaw_rate = ALPHA * r + (1 - ALPHA) * yaw_rate
            yaw = yaw * YAW_LEAK + yaw_rate * dt
            payload = {"ts": round(t, 3), "yaw_rate_dps": round(yaw_rate, 2),
                       "yaw_deg": round(yaw, 2), "gyro_raw": [gx, gy, gz],
                       "temp_c": None, "quality": 1.0}
            if not args.test:
                tmp = f"{args.out}.tmp"
                with open(tmp, "w") as f:
                    json.dump(payload, f)
                os.replace(tmp, args.out)
        except OSError as e:
            yaw_rate = 0.0
            print(f"[imu] ioctl错误: {e}")
            time.sleep(0.2)
            continue

        if args.test:
            if t - t_print >= 0.25:
                print(f"t={t - t_start:5.2f}s  yaw_rate={yaw_rate:+7.2f} dps  "
                      f"yaw={yaw:+7.2f}°  raw_z={gz}")
                t_print = t
            if t - t_start > 4.0:
                break
        time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - t)))


if __name__ == "__main__":
    main()
