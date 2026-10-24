#!/usr/bin/env python3
"""test_move_gate.py —— /move 跟随模式门禁回归（2026-10-24 白名单#1 补测）

背景: /forward~/stop 组 2026-10-16 有"跟随时锁死"门禁, /move 当时漏了 ——
跟随中任何热点内设备 POST /move 可直接抢底盘(修复=commit 4ae2253)。
本测试起真实 HTTPServer(handler 层), uart_send 打桩(不碰串口)、不动跟随线程。

用法: python3 test_move_gate.py
"""
import json
import os
import sys
import threading
import urllib.error
import urllib.request
from http.server import HTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rk_control as rk  # noqa: E402


class QuietHandler(rk.Handler):
    def log_message(self, *a):        # 静音访问日志, 保持测试输出干净
        pass


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'},
                                 method='POST')
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


def main():
    sent = []
    rk.uart_send = lambda cmd: (sent.append(cmd), True)[1]   # 打桩: 记录而非发串口
    srv = HTTPServer(("127.0.0.1", 0), QuietHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    try:
        # 1) follow 模式: /move 必须被门禁拒绝, 且不产生任何串口命令
        with rk.mode_lock:
            rk.mode = "follow"
        _st, body = _post(base + "/move", {"direction": "F", "speed": 200})
        assert "follow mode" in body, body
        assert sent == [], f"跟随中被拒的 /move 不得触串口, 实发 {sent}"
        print("  follow 模式: /move 被拒(未触串口) ✓")

        # 2) manual 模式: 门禁放行, 命令正常下发(T0/T8 走的正是这条路)
        with rk.mode_lock:
            rk.mode = "manual"
        st, body = _post(base + "/move", {"direction": "F", "speed": 150})
        assert st == 200 and body.strip() == "OK", (st, body)
        assert sent == ["MOVE F 150"], sent
        print("  manual 模式: /move 放行 → MOVE F 150 ✓")

        # 3) 切回 follow 再验一次(门禁必须模式相关, 不是一次性)
        with rk.mode_lock:
            rk.mode = "follow"
        _st, body = _post(base + "/move", {"direction": "B", "speed": 120})
        assert "follow mode" in body and sent == ["MOVE F 150"], (body, sent)
        print("  再切回 follow: 依旧被拒 ✓")

        # 4) manual 下坏参数仍走 400(门禁不得吞掉原校验路径)
        with rk.mode_lock:
            rk.mode = "manual"
        st, _body = _post(base + "/move", {"direction": "X", "speed": 150})
        assert st == 400 and sent == ["MOVE F 150"], (st, sent)
        print("  非法方向 400 ✓")
    finally:
        with rk.mode_lock:
            rk.mode = "manual"        # 还原, 防止影响同进程后续测试
        srv.shutdown()
        srv.server_close()
    print("\n全部断言通过")


if __name__ == "__main__":
    main()
