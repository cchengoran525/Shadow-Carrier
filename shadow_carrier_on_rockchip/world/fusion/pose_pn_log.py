#!/usr/bin/env python3
"""pose_pn_log.py —— 采集 pose 特征做 P/N 标定
用法: python3 pose_pn_log.py <label A|B> <seconds>
每次读取 /dev/shm/pose_out.json 的最大 person, 追加 features 到 pose_pn.jsonl
"""
import json, time, sys, os

OUT = "/home/kickpi/pose_demo/pose_pn.jsonl"

def main():
    label = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12
    t_end = time.time() + secs
    n = 0
    with open(OUT, "a") as f:
        while time.time() < t_end:
            try:
                with open("/dev/shm/pose_out.json") as pf:
                    d = json.load(pf)
                if d.get("persons"):
                    p = max(d["persons"], key=lambda x: (x["bbox"][2]-x["bbox"][0])*(x["bbox"][3]-x["bbox"][1]))
                    row = {"label": label, "t": round(time.time(), 3),
                           "conf": p.get("conf"), "facing": p.get("facing"),
                           "offering": p.get("offering"),
                           "arms": p.get("arms")}
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
                    n += 1
            except Exception:
                pass
            time.sleep(0.2)
    print("logged %d rows label=%s -> %s" % (n, label, OUT))

if __name__ == "__main__":
    main()
