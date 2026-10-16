#!/usr/bin/env python3
"""pivot_calib.py — 原地转角速度标定 + 符号约定核对 (实机, 约 3 分钟)

为什么需要: follow_controller 的底盘转向通道里, PIVOT_PWM / TTE 阈值 / 扫描驻留都建立在
"PWM → 原地转角速度(°/s)"这条曲线上; 仿真与实机的轮径/摩擦/电压都不同, 必须实测。
另外 θ(方位角) 的左右符号链历史上出过事故(2026-10-14 审计), 这里给一次人机可判的核对。

⚠️ 安全: 本脚本直接开串口发 MOVE L/R(绕过 rk_control), 车会原地转。
   先把 rk_control 切手动(或停跟随), 车放空旷地面(四周 ≥1m), 人不要站在转动半径内。

用法:
  python3 pivot_calib.py --yes                  # 测 80/110/140 三档 + 符号核对
  python3 pivot_calib.py --yes --secs 1.5       # 每档时长
  python3 pivot_calib.py --check-only           # 只做符号/IMU 核对, 不动电机
"""
import argparse
import json
import math
import os
import sys
import time

PORT = "/dev/ttyACM0"
IMU_JSON = "/dev/shm/imu.json"
OWNER_JSON = "/dev/shm/owner.json"
CTRL = "http://127.0.0.1:80"
DET_API = "http://127.0.0.1:8080/api/detections"


def read_imu(max_age=0.5):
    try:
        with open(IMU_JSON) as f:
            d = json.load(f)
        if time.monotonic() - float(d.get("ts", 0)) > max_age:
            return None
        return d
    except Exception:
        return None


def open_serial(port):
    import termios
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attr = termios.tcgetattr(fd)
    attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attr[3] = 0
    attr[4] = attr[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    return fd


def burst(fd, cmd, secs):
    """持续重发一条命令 secs 秒(固件 450ms 看门狗), 末尾 STOP 并等静止"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        os.write(fd, (cmd + "\r\n").encode())
        time.sleep(0.1)
    os.write(fd, b"STOP\r\n")
    time.sleep(0.6)


def measure(fd, cmd, secs):
    """返回 (Δyaw度, °/s); Δyaw 按 IMU 积分航向(raw, 逆时针为正的约定见 rk_control)"""
    y0 = read_imu()
    if y0 is None:
        return None, None
    burst(fd, cmd, secs)
    y1 = read_imu()
    if y1 is None:
        return None, None
    d = float(y1["yaw_deg"]) - float(y0["yaw_deg"])
    return d, d / secs


def sign_check_visual():
    """读主人框(或最大 person) → θ: 打印"θ 正对应哪一侧/会发哪个转向指令"供人工确认"""
    cx = None
    try:
        with open(OWNER_JSON) as f:
            o = json.load(f)
        if time.monotonic() - float(o.get("ts", 0)) < 2.0:
            b = o["bbox"]
            cx = (b[0] + b[2]) / 2
            print(f"  owner.json bbox cx={cx:.0f} (认主框, 最可信)")
    except Exception:
        pass
    if cx is None:
        try:
            import urllib.request
            d = json.loads(urllib.request.urlopen(DET_API, timeout=1.5).read())
            best = None
            for x in d.get("detections", []):
                if x.get("c") != "person":
                    continue
                area = (float(x["x2"]) - float(x["x1"])) * (float(x["y2"]) - float(x["y1"]))
                if best is None or area > best[0]:
                    best = (area, (float(x["x1"]) + float(x["x2"])) / 2)
            if best:
                cx = best[1]
                print(f"  最大 person cx={cx:.0f} (无新鲜 owner.json, 用最大 person 近似)")
        except Exception as e:
            print(f"  拿不到检测({e}) —— 先起 video-stream/yolo_daemon 再跑本项")
            return
    if cx is None:
        print("  视野里没有 person")
        return
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "state"))
        from decoder import pixel_to_bearing_offset as bearing
        th = bearing(cx)
    except Exception:
        th = math.degrees(math.atan((cx - 320.0) / 508.2))
    d = "R" if th > 0 else "L"
    print(f"  → θ={th:+.1f}° → follow 会发 MOVE {d} (应朝【人】转)")
    print("  请人工确认: 人站在车的哪一侧? 车是否朝人转?")
    print("  若车转向【背离】人 → 修 follow_controller._dir_of_theta 的映射"
          "(与弧线的 offset 符号同源, 弧线若一起反了说明是不变量本身的问题)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--secs", type=float, default=1.2, help="每档持续时长(秒)")
    ap.add_argument("--pwms", default="80,110,140")
    ap.add_argument("--yes", action="store_true", help="确认车已放好(会原地转)")
    ap.add_argument("--check-only", action="store_true", help="只做符号/IMU 核对, 不动电机")
    args = ap.parse_args()

    imu = read_imu()
    if imu is None:
        print("❌ 读不到新鲜 imu.json —— 先起 imu.service (或在板上跑 imu_daemon.py)")
        return 1
    print(f"IMU OK: yaw={imu['yaw_deg']}° yaw_rate={imu['yaw_rate_dps']}°/s")
    print("\n== θ 符号核对 (云台冻结 → 相机=车体) ==")
    sign_check_visual()
    if args.check_only:
        print("\n(check-only: 未动电机)")
        return 0
    if not args.yes:
        print("\n⚠️ 未加 --yes: 本步会开串口让车原地转, 确认安全后再跑。")
        return 1

    fd = open_serial(args.port)
    try:
        print("\n== 原地转角速度 (MOVE L/R, 逐档实测) ==")
        print("  pwm   方向    Δyaw(°)   °/s")
        rows = []
        for pwm in [int(x) for x in args.pwms.split(",")]:
            for d in ("L", "R"):
                dy, rate = measure(fd, f"MOVE {d} {pwm}", args.secs)
                if dy is None:
                    print(f"  {pwm:<5} {d}       (IMU 丢了)")
                    continue
                print(f"  {pwm:<5} {d}       {dy:+7.1f}   {rate:+7.1f}")
                rows.append((pwm, d, dy, rate))
        ok = [r for r in rows if r[3] is not None]
        if ok:
            avg = sum(abs(r[3]) for r in ok) / len(ok)
            print(f"\n  → 三档平均 |ω| ≈ {avg:.0f}°/s (写回 follow_controller 的口径: "
                  f"PIVOT_PWM={args.pwms.split(',')[1]} 对应 ≈ "
                  f"{[round(abs(r[3])) for r in ok if r[0] == int(args.pwms.split(',')[1])]})")
            print("  → 该值用于: TTE 阈值(需 > 人的角速度 v/d) / 扫描 90° 一档的用时 / 驻留")
        print("\n== IMU 符号核对 ==")
        dl = [r[2] for r in rows if r[1] == "L"]
        dr = [r[2] for r in rows if r[1] == "R"]
        if dl and dr:
            print(f"  MOVE L → Δyaw {sum(dl)/len(dl):+.1f}°   (期望 >0: 逆时针/左)")
            print(f"  MOVE R → Δyaw {sum(dr)/len(dr):+.1f}°   (期望 <0)")
            if sum(dl) / len(dl) > 0 and sum(dr) / len(dr) < 0:
                print("  [PASS] 与 rk_control/_get_yaw_rate 的'逆时针为正'约定一致 → "
                      "IMU_TO_THETA_SIGN=-1 成立")
            else:
                print("  [FAIL] 与文档约定相反 → 改 follow_controller.IMU_TO_THETA_SIGN 符号, "
                      "并复核 rk_control 的直行航向环(YAW_K 支路)")
    finally:
        os.write(fd, b"STOP\r\n")
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
