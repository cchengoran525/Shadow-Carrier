#!/usr/bin/env python3
"""pivot_calib.py — 实机标定一条龙: 一次跑出四个数 (2026-10-19 扩展)

四件事:
  A. 原地转 PWM → °/s (默认档 120/150/190/230, 覆盖 MIN_SPD 与 PIVOT_PWM)
  B. 弧线 (DIFF 快/慢) 的实测角速度 —— 校核 BASE_SPD/MIN_SPD 的真实差速
  C. 满 PWM 地面速度 v_at_spd_max (直行 N 秒 + 人工量距离; 写 configs/lidar_guard.json)
  D. IMU 符号核对 (MOVE L 应使 yaw 增大) + θ 左右符号的人工核对 (云台冻结 → 相机=车体)
输出: calib.json + "写回清单"(要改哪几个常量)

为什么需要: follow 的 PIVOT_RATE_DPS / TTE 阈值 / 扫描节奏 / 雷达间隙律, 全建在这四个数上;
仿真与实机的轮径/摩擦/电压都不同。θ 的符号链历史上出过事故(2026-10-14 审计), 这里给一次
人机可判的核对。

⚠️ 安全: 本脚本直接开串口发 MOVE L/R 与 MOVE F(绕过 rk_control), 车会动。
   先把 rk_control 切手动(或停跟随), 车放空旷地面(四周 ≥1m), 人不要站在转动/直行路径上。

用法:
  python3 pivot_calib.py --yes                      # 全量(默认 4 档原地转 + 弧线 + 询问直行距离)
  python3 pivot_calib.py --yes --vmax-dist 2.4      # 直行距离直接给, 非交互
  python3 pivot_calib.py --check-only               # 只做符号/IMU 核对, 不动电机
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
    ap.add_argument("--pwms", default="120,150,190,230",
                    help="原地转 PWM 档(默认覆盖 MIN_SPD=120 与 PIVOT_PWM=190)")
    ap.add_argument("--arc", default="220,120",
                    help="弧线实测档 '快,慢'(对应 BASE_SPD,MIN_SPD); 传 '' 跳过")
    ap.add_argument("--vmax-pwm", type=int, default=255, help="满速直行用的 PWM(测 v_at_spd_max)")
    ap.add_argument("--vmax-secs", type=float, default=3.0, help="满速直行时长")
    ap.add_argument("--vmax-dist", type=float, default=None,
                    help="操作员量到的直行距离(m); 不给则交互询问(或跳过)")
    ap.add_argument("--out", default="calib.json", help="结果 JSON 输出路径")
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

    report = {"measured_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
              "secs": args.secs, "pivot": {}, "arc": {}, "v_mps_at_pwm": None,
              "sign": {}}
    fd = open_serial(args.port)
    try:
        print("\n== A. 原地转角速度 (MOVE L/R) ==")
        print("  pwm   方向    Δyaw(°)   °/s")
        rows = []
        for pwm in [int(x) for x in args.pwms.split(",") if x.strip()]:
            for d in ("L", "R"):
                dy, rate = measure(fd, f"MOVE {d} {pwm}", args.secs)
                if dy is None:
                    print(f"  {pwm:<5} {d}       (IMU 丢了)")
                    continue
                print(f"  {pwm:<5} {d}       {dy:+7.1f}   {rate:+7.1f}")
                rows.append((pwm, d, dy, rate))
                report["pivot"].setdefault(str(pwm), {})[d] = round(rate, 1)

        print("\n== B. 弧线角速度 (DIFF, 校核 BASE/MIN 的真实差速) ==")
        if args.arc.strip():
            fast, slow = [int(x) for x in args.arc.split(",")]
            for (l, r) in ((fast, slow), (slow, fast)):
                dy, rate = measure(fd, f"DIFF L{l} R{r}", args.secs)
                if dy is None:
                    print(f"  DIFF L{l} R{r}: (IMU 丢了)")
                    continue
                print(f"  DIFF L{l} R{r}: Δyaw {dy:+7.1f}°  → {rate:+6.1f}°/s")
                report["arc"][f"L{l}R{r}"] = round(rate, 1)

        print("\n== C. 满 PWM 地面速度 (v_at_spd_max) ==")
        dist = args.vmax_dist
        if dist is None:
            print(f"  即将以 MOVE F {args.vmax_pwm} 直行 {args.vmax_secs}s —— 直行距离需人工量。")
            try:
                if sys.stdin.isatty():
                    s = input(f"  量到的直行距离(m, 回车跳过): ").strip()
                    dist = float(s) if s else None
            except (EOFError, ValueError):
                dist = None
        if dist is not None:
            burst(fd, f"MOVE F {args.vmax_pwm}", args.vmax_secs)
            v = dist / args.vmax_secs
            report["v_mps_at_pwm"] = {"pwm": args.vmax_pwm, "mps": round(v, 3)}
            v_at_255 = v * (255.0 / max(1, args.vmax_pwm))
            print(f"  {dist:.2f}m / {args.vmax_secs:.1f}s → {v:.2f} m/s @ PWM{args.vmax_pwm}"
                  f" → 折合满量程 ≈{v_at_255:.2f} m/s (写 configs/lidar_guard.json v_at_spd_max)")
        else:
            print("  (跳过: 未提供距离。可重跑 --vmax-dist <米>)")

        print("\n== D. IMU 符号核对 ==")
        dl = [r[2] for r in rows if r[1] == "L"]
        dr = [r[2] for r in rows if r[1] == "R"]
        sign_ok = None
        if dl and dr:
            mvl = sum(dl) / len(dl)
            mvr = sum(dr) / len(dr)
            print(f"  MOVE L → Δyaw {mvl:+.1f}°   (期望 >0: 逆时针/左)")
            print(f"  MOVE R → Δyaw {mvr:+.1f}°   (期望 <0)")
            sign_ok = (mvl > 0 and mvr < 0)
            report["sign"] = {"move_l_dyaw": round(mvl, 1), "move_r_dyaw": round(mvr, 1),
                              "convention_ok": sign_ok}
            if sign_ok:
                print("  [PASS] 与 rk_control/_get_yaw_rate 的'逆时针为正'约定一致 → "
                      "IMU_TO_THETA_SIGN=-1 成立")
            else:
                print("  [FAIL] 与文档约定相反 → 改 follow_controller.IMU_TO_THETA_SIGN 符号, "
                      "并复核 rk_control 的直行航向环(YAW_K 支路)")

        print("\n== 写回清单 ==")
        try:
            pwm_p = 190
            r190 = report["pivot"].get(str(pwm_p), {})
            if r190:
                est = sum(abs(v) for v in r190.values()) / len(r190)
                print(f"  follow_controller.PIVOT_RATE_DPS = {est:.0f}   "
                      f"(PIVOT_PWM={pwm_p} 实测)")
            if report.get("v_mps_at_pwm"):
                vm = report["v_mps_at_pwm"]
                print(f"  configs/lidar_guard.json  v_at_spd_max = "
                      f"{vm['mps'] * 255.0 / max(1, vm['pwm']):.2f}")
            print("  (θ 左右符号另见上方人工核对项; 改完跑一遍 7 套离线回归)")
        except Exception:
            pass
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=1)
        print(f"\n结果已写入 {args.out}")
    finally:
        os.write(fd, b"STOP\r\n")
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
