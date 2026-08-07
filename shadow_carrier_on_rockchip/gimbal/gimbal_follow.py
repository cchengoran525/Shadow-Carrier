#!/usr/bin/env python3
"""gimbal_follow.py - 云台追随最大person
两自由度舵机(Pan/Tilt)自动追随画面中面积最大的person。
独立于 video_stream_v7 运行, 只读它的 HTTP API, 不影响主视频管线。

接线见 gimbal/README.md (Pan→GPIO4_A4, Tilt→GPIO4_A5, VCC→5V, GND→GND)
需要 sudo 运行 (GPIO sysfs 权限):  sudo python3 gimbal_follow.py
"""
import os, sys, time, json, threading, urllib.request

# ================= 配置 =================
PAN_GPIO = 134          # GPIO4_A6 (sysfs号)
TILT_GPIO = 135         # GPIO4_A7
API_URL = "http://127.0.0.1:8080/api/detections"
W, H = 640, 480        # 画面尺寸 (与 video_stream_v7 一致)

PAN_CENTER = 90         # 舵机中位角
TILT_CENTER = 90
PAN_K = 0.05            # P控制增益: 像素偏差 -> 角度
TILT_K = 0.04
SERVO_MIN, SERVO_MAX = 0, 180
PULSE_MIN, PULSE_MAX = 0.5, 2.5   # 脉宽 ms (0°=0.5ms, 180°=2.5ms)
CYCLE_MS = 20.0         # 50Hz
POLL_S = 0.03           # API轮询间隔
# ========================================

state = {"pan": PAN_CENTER, "tilt": TILT_CENTER}
lock = threading.Lock()


def clamp(v, lo=SERVO_MIN, hi=SERVO_MAX):
    return max(lo, min(hi, v))


def busy_sleep(ms):
    """忙等, 精度~0.1ms (软件PWM核心)"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < ms / 1000.0:
        pass


def pulse_ms(angle):
    return PULSE_MIN + (angle / SERVO_MAX) * (PULSE_MAX - PULSE_MIN)


def pwm_thread(pan_fd, tilt_fd):
    """持续50Hz输出当前角度到两个舵机, 舵机保持位置"""
    while True:
        with lock:
            pa, ta = state["pan"], state["tilt"]
        pp = pulse_ms(pa)
        tp = pulse_ms(ta)
        # Pan 脉冲
        os.write(pan_fd, b'1')
        busy_sleep(pp)
        os.write(pan_fd, b'0')
        # Tilt 脉冲
        os.write(tilt_fd, b'1')
        busy_sleep(tp)
        os.write(tilt_fd, b'0')
        # 周期剩余
        busy_sleep(max(CYCLE_MS - pp - tp, 0.5))


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


def control_thread():
    """每100ms从API取人物, P控制更新pan/tilt目标角度"""
    while True:
        person = fetch_biggest_person()
        if person is not None:
            cx = (person["x1"] + person["x2"]) / 2
            cy = (person["y1"] + person["y2"]) / 2
            dx = cx - W / 2
            dy = cy - H / 2
            with lock:
                state["pan"] = clamp(state["pan"] + dx * PAN_K)
                state["tilt"] = clamp(state["tilt"] + dy * TILT_K)
        time.sleep(POLL_S)


def gpio_init(gpio):
    """确保GPIO已导出且为输出, 返回value fd"""
    vpath = f"/sys/class/gpio/gpio{gpio}/value"
    if not os.path.exists(vpath):
        with open("/sys/class/gpio/export", "w") as f:
            f.write(str(gpio))
        time.sleep(0.1)
    with open(f"/sys/class/gpio/gpio{gpio}/direction", "w") as f:
        f.write("out")
    return os.open(vpath, os.O_WRONLY)


def main():
    print("gimbal_follow: 云台追随最大person")
    print(f"  Pan=GPIO{PAN_GPIO}(GPIO4_A6)  Tilt=GPIO{TILT_GPIO}(GPIO4_A7)")
    print(f"  依赖: {API_URL} (video_stream_v7)")
    if not os.path.exists(f"/sys/class/gpio/gpio{PAN_GPIO}"):
        print("警告: Pan GPIO 未导出, 尝试导出...", file=sys.stderr)
    try:
        pan_fd = gpio_init(PAN_GPIO)
        tilt_fd = gpio_init(TILT_GPIO)
    except Exception as e:
        print(f"GPIO初始化失败: {e}", file=sys.stderr)
        print("请用 sudo 运行: sudo python3 gimbal_follow.py", file=sys.stderr)
        sys.exit(1)

    threading.Thread(target=pwm_thread, args=(pan_fd, tilt_fd), daemon=True).start()
    threading.Thread(target=control_thread, daemon=True).start()
    print("运行中... Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        os.close(pan_fd)
        os.close(tilt_fd)


if __name__ == "__main__":
    main()
