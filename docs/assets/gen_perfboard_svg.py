#!/usr/bin/env python3
"""生成 V2 非 RK 板(洞洞板)【针位级】布局图 SVG。
板: 100x150mm, 孔距 2.54mm, 38 列 x 56 行(列/行号从 1 计), 孔逐一画出;
元件画成落在具体孔位上的焊盘, 焊盘旁竖排标信号; 端子标到每一位。
用法: python3 gen_perfboard_svg.py > ../docs/assets/v2_perfboard_layout.svg
"""
import sys

S = 5.0                      # 1mm = 5px
PITCH = 2.54
BOARD_W, BOARD_H = 100.0, 150.0
COLS, ROWS = 38, 56
MARGIN = 16.0
LEGEND_W = 300
W = int((BOARD_W + MARGIN * 2) * S + LEGEND_W)
H = int((BOARD_H + MARGIN * 2) * S)

out = []


def hx(c):   # 孔列 c(1..38) → px
    return (c - 1) * PITCH * S + MARGIN * S + PITCH * S


def hy(r):   # 孔行 r(1..56) → px
    return (r - 1) * PITCH * S + MARGIN * S + PITCH * S


def board_x(mm):
    return MARGIN * S + mm * S


def board_y(mm):
    return MARGIN * S + mm * S


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;")


def txt(x, y, s, size=7, fill="#111", anchor="middle", rot=None, weight="normal"):
    tr = f' transform="rotate({rot} {x:.1f} {y:.1f})"' if rot else ""
    out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-family="Helvetica,Arial,sans-serif" '
               f'font-size="{size}" fill="{fill}" text-anchor="{anchor}" '
               f'font-weight="{weight}"{tr}>{esc(s)}</text>')


def pad(c, r, label="", color="#c0392b", lsize=5.4, lcolor=None):
    """落在孔位 (c,r) 上的焊盘 + 竖排信号名"""
    x, y = hx(c), hy(r)
    out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{PITCH*S*0.34:.1f}" fill="{color}"/>')
    if label:
        txt(x + 2.4, y - 3.4, label, lsize, lcolor or color, anchor="start", rot=-90)


def holes_pattern():
    step = PITCH * S
    out.append(f'''<defs><pattern id="holes" width="{step:.2f}" height="{step:.2f}" patternUnits="userSpaceOnUse">
    <circle cx="{step/2:.2f}" cy="{step/2:.2f}" r="{step*0.17:.2f}" fill="#ffffff" stroke="#b9a37e" stroke-width="1.1"/>
    </pattern></defs>''')


def zone(c1, r1, c2, r2, label, color, fill, lsize=8.0):
    x1, y1 = hx(c1) - PITCH * S * 0.5, hy(r1) - PITCH * S * 0.5
    x2, y2 = hx(c2) + PITCH * S * 0.5, hy(r2) + PITCH * S * 0.5
    out.append(f'<rect x="{x1:.1f}" y="{y1:.1f}" width="{x2-x1:.1f}" height="{y2-y1:.1f}" '
               f'fill="{fill}" stroke="{color}" stroke-width="1" rx="3" opacity="0.5"/>')
    txt(x1 + 5, y1 + 12, label, lsize, color, anchor="start", weight="bold")


def module(c1, r1, c2, r2, label, sub="", color="#8e44ad"):
    x1, y1 = hx(c1) - PITCH * S * 0.45, hy(r1) - PITCH * S * 0.45
    x2, y2 = hx(c2) + PITCH * S * 0.45, hy(r2) + PITCH * S * 0.45
    out.append(f'<rect x="{x1:.1f}" y="{y1:.1f}" width="{x2-x1:.1f}" height="{y2-y1:.1f}" '
               f'fill="#ffffff" stroke="{color}" stroke-width="1.2" rx="2" opacity="0.92"/>')
    txt((x1 + x2) / 2, y1 + 13, label, 8, color, weight="bold")
    if sub:
        txt((x1 + x2) / 2, y1 + 23, sub, 6.4, "#666")


def wire(c1, r1, c2, r2, color, width=2.2, dash=None, junction=True):
    x1, y1, x2, y2 = hx(c1), hy(r1), hx(c2), hy(r2)
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" '
               f'stroke-width="{width}" stroke-linecap="round"{d}/>')
    if junction:
        out.append(f'<circle cx="{x2:.1f}" cy="{y2:.1f}" r="2.6" fill="{color}"/>')


def build():
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    holes_pattern()
    out.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
    txt(MARGIN * S, MARGIN * S - 18,
        "Shadow Carrier V2 — 非 RK 板针位级布局图 · 100×150mm · 孔距 2.54mm · 38 列 × 56 行", 13, "#111",
        anchor="start", weight="bold")
    txt(MARGIN * S, MARGIN * S - 5,
        "实心红/黄/绿/紫圆 = 焊盘(信号名竖排) · 白圈 = 空孔 · 打印按 1:1 (A3 或 150%)", 8, "#555", anchor="start")
    # 板面 + 孔阵
    out.append(f'<rect x="{board_x(0):.1f}" y="{board_y(0):.1f}" width="{BOARD_W*S:.1f}" '
               f'height="{BOARD_H*S:.1f}" fill="#f2ead8" stroke="#8a7a5c" stroke-width="1.6" rx="3"/>')
    out.append(f'<rect x="{board_x(0):.1f}" y="{board_y(0):.1f}" width="{BOARD_W*S:.1f}" '
               f'height="{BOARD_H*S:.1f}" fill="url(#holes)"/>')
    # 行列标尺(每 5 格)
    for c in range(5, COLS + 1, 5):
        txt(hx(c), board_y(0) - 9, str(c), 7, "#8a7a5c")
        txt(hx(c), board_y(BOARD_H) + 14, str(c), 7, "#8a7a5c")
    for r in range(5, ROWS + 1, 5):
        txt(board_x(0) - 10, hy(r) + 3, str(r), 7, "#8a7a5c")
        txt(board_x(BOARD_W) + 10, hy(r) + 3, str(r), 7, "#8a7a5c")

    # ── 母线 ──
    wire(2, 1, 35, 1, "#c0392b", 3.0, junction=False)
    txt(hx(2), hy(1) - 7, "12V 母线 (背面 20AWG 飞线; 各支路在此取电)", 7.2, "#c0392b", anchor="start")
    wire(2, 26, 35, 26, "#2c3e50", 3.0, dash="6,4", junction=False)
    txt(hx(2), hy(26) - 7, "GND 星地母线 (背面裸铜线; 所有 GND 就近点焊)", 7.2, "#2c3e50", anchor="start")

    # ── ① 电源入口 ──
    zone(1, 3, 8, 21, "① 电源入口", "#c0392b", "#fdecea")
    pad(2, 4, "XT30+"); pad(4, 4, "XT30-")
    txt(hx(3), hy(4) + 34, "XT30 电源座", 6.6, "#c0392b")
    pad(2, 7, "保险A"); pad(4, 7, "保险B")
    txt(hx(3), hy(7) + 34, "保险丝座 7.5A 慢熔", 6.4, "#c0392b")
    pad(2, 10, "开关A"); pad(4, 10, "开关B")
    txt(hx(3), hy(10) + 34, "船型开关 ≥10A", 6.4, "#c0392b")
    txt(hx(4), hy(13) + 4, "12V 端子排 (4 位)", 6.6, "#c0392b")
    for i, lab in enumerate(["12V_IN", "12V", "GND", "GND"]):
        pad(2 + i * 1.7, 16, lab, "#c0392b", 5.0)
    for i, (c, lab) in enumerate([(2, "C1 470µ/25V"), (4.7, "C2 1000µ/16V"), (7.4, "C3 470µ/10V")]):
        pad(round(c), 19, "+"); pad(round(c) + 2, 19, "-")
        txt(hx(round(c) + 1), hy(19) + 34, lab, 5.6, "#c0392b")
    # 入口链路: XT30+ → 保险 → 开关 → 12V 端子排 → 母线
    wire(2, 1, 2, 4, "#c0392b", 2.2, junction=False)
    wire(2, 7, 2, 7, "#c0392b", 2.0, junction=False)
    wire(4, 4, 4, 7, "#c0392b", 2.0); wire(2, 7, 2, 10, "#c0392b", 2.0)
    wire(2, 10, 2, 16, "#c0392b", 2.0); wire(2, 1, 2, 16, "#c0392b", 2.4, junction=False)

    # ── ② 降压区 ──
    zone(9, 3, 16, 21, "② 降压区", "#2471a3", "#eaf4fd")
    module(9, 4, 15, 8, "XL4015", "12V→7.4V/3A (云台轨)", "#2471a3")
    for c, lab in [(10, "IN+"), (11, "IN-"), (13, "OUT+"), (14, "OUT-")]:
        pad(c, 9, lab, "#2471a3", 5.2, "#1b4f72")
    module(9, 12, 15, 16, "MP1584", "12V→5V/2A (A4950 逻辑)", "#2471a3")
    for c, lab in [(10, "IN+"), (11, "IN-"), (13, "OUT+"), (14, "OUT-")]:
        pad(c, 17, lab, "#2471a3", 5.2, "#1b4f72")
    wire(10, 1, 10, 4, "#c0392b", 2.0)
    wire(10, 1, 10, 12, "#c0392b", 2.0)
    wire(13, 9, 13, 23, "#e67e22", 2.0, junction=False)   # 7.4V 出, 竖下到 row23 空行
    wire(13, 23, 29, 23, "#e67e22", 2.0, junction=False)  # 沿 row23 横走到 col29
    wire(29, 23, 29, 41, "#e67e22", 2.0, junction=False)  # 沿 col29 竖下(避开 S3 区)
    wire(29, 41, 23, 41, "#e67e22", 2.0, junction=False)  # 沿 row41 横走到 col23
    wire(23, 41, 23, 46, "#e67e22", 2.0, junction=False)  # 下到云台总线 V 脚
    txt(hx(30), hy(30), "7.4V 走 col29→row41 空车道", 5.8, "#e67e22", anchor="start", rot=-90)
    wire(13, 17, 20, 19, "#16a085", 2.0, junction=False)  # 5V → A4950#2 VCC (相邻短接)
    wire(20, 19, 27, 19, "#16a085", 2.0, junction=False)  # 绕到 col27
    wire(27, 19, 27, 10, "#16a085", 2.0, junction=False)  # 竖上(col27, 不穿模块)
    wire(27, 10, 20, 10, "#16a085", 2.0, junction=False)  # → A4950#1 VCC

    # ── ③ 驱动区 ──
    zone(17, 3, 35, 22, "③ 驱动区", "#b7950b", "#fef9e7")
    module(17, 4, 26, 9, "A4950 #1 双路", "M1 左前 / M2 右前", "#b7950b")
    for c, lab in [(18, "VM"), (19, "GND"), (20, "VCC"), (21, "A1"), (22, "A2"), (24, "B1"), (25, "B2")]:
        pad(c, 10, lab, "#b7950b", 5.0, "#7d6608")
    module(17, 13, 26, 18, "A4950 #2 双路", "M3 左后 / M4 右后", "#b7950b")
    for c, lab in [(18, "VM"), (19, "GND"), (20, "VCC"), (21, "A1"), (22, "A2"), (24, "B1"), (25, "B2")]:
        pad(c, 19, lab, "#b7950b", 5.0, "#7d6608")
    wire(18, 1, 18, 4, "#c0392b", 2.2); wire(18, 1, 18, 13, "#c0392b", 2.2)
    txt(hx(32), hy(4) - 8, "电机端子 (贴右板边, 每路 2 位)", 6.6, "#7d6608", anchor="start")
    for r, names in [(4, ("M1+", "M1-")), (7, ("M2+", "M2-")), (13, ("M3+", "M3-")), (16, ("M4+", "M4-"))]:
        pad(33, r, names[0], "#7d6608", 5.0)
        pad(35, r, names[1], "#7d6608", 5.0)
        out.append(f'<rect x="{hx(33)-PITCH*S*0.5:.1f}" y="{hy(r)-PITCH*S*0.5:.1f}" '
                   f'width="{PITCH*S*3.0:.1f}" height="{PITCH*S*1.0:.1f}" fill="none" stroke="#7d6608"/>')

    # ── ④ 主控区 ──
    zone(2, 29, 27, 35, "④ 主控: ESP32-S3-DevKitC-1  2×22 排母座 (列序按本图; 焊装对照实板丝印)", "#1e8449", "#eafaf1")
    rowA = ["M1_PWM", "M1_DIR", "M2_PWM", "M2_DIR", "M3_PWM", "M3_DIR", "M4_PWM", "M4_DIR",
            "E1_A", "E1_B", "E2_A", "E2_B", "E3_A", "E3_B", "E4_A", "E4_B",
            "I2C_SDA", "I2C_SCL", "GIM_TX", "GIM_RX", "US_TRIG", "US_ECHO"]
    rowB = ["3V3", "3V3", "GND", "GND", "GND", "GND", "5V", "5V", "LED", "BTN", "BUZ",
            "—", "—", "—", "—", "—", "GND", "—", "—", "—", "—", "GND"]
    for i, lab in enumerate(rowA):
        pad(3 + i, 31, lab, "#1e8449", 5.0, "#0e5c31")
    for i, lab in enumerate(rowB):
        pad(3 + i, 34, lab, "#1e8449", 5.0, "#0e5c31")
    txt(hx(2), hy(28) + 4, "上排 22 位 = 信号 (4 组编码器 / 4 路电机 / I2C / 云台 / 超声)", 6.2, "#0e5c31", anchor="start")
    txt(hx(2), hy(37) + 4, "下排 22 位 = 电源 + LED/按钮/蜂鸣 (未用位留空)", 6.2, "#0e5c31", anchor="start")
    # 3.3V 支线 → 编码器 VCC
    wire(4, 34, 4, 45, "#16a085", 1.8, junction=False)
    txt(hx(4) - 6, hy(40), "3.3V → 编码器 VCC", 5.6, "#16a085", anchor="start", rot=-90)

    # ── ⑤ 连接器排 ──
    zone(1, 43, 36, 49, "⑤ 连接器排 (贴下板边: 插拔方便 + 扎带应力释放)", "#7d3c98", "#f4ecf7")
    conns = [
        (2, "编码器1 JST-XH 4P", ["V3V3", "GND", "A", "B"]),
        (7, "编码器2", ["V3V3", "GND", "A", "B"]),
        (12, "编码器3", ["V3V3", "GND", "A", "B"]),
        (17, "编码器4", ["V3V3", "GND", "A", "B"]),
        (23, "云台总线 3P", ["7V4", "GND", "DATA"]),
        (27, "I2C 4P", ["3V3", "GND", "SDA", "SCL"]),
        (32, "超声波 4P", ["5V", "GND", "TRIG", "ECHO"]),
    ]
    for c0, name, pins in conns:
        txt(hx(c0), hy(44) - 7, name, 5.8, "#7d3c98", anchor="start")
        for i, p in enumerate(pins):
            pad(c0 + i, 46, p, "#7d3c98", 5.0, "#5b2c6f")
        w = PITCH * S * (len(pins) - 1) + PITCH * S * 1.0
        out.append(f'<rect x="{hx(c0)-PITCH*S*0.5:.1f}" y="{hy(46)-PITCH*S*0.5:.1f}" '
                   f'width="{w:.1f}" height="{PITCH*S*1.0:.1f}" fill="none" stroke="#7d3c98"/>')
    wire(4, 46, 19, 46, "#16a085", 1.6, junction=False)  # 3.3V 沿排到编码器 1/2/3/4 的 V 脚
    txt(hx(30), hy(48) + 26, "分压 1k/2k 焊在超声 ECHO 位旁 (ECHO→1k→节点→2k→GND)", 6.0, "#5b2c6f", anchor="start")

    # ── 图例 ──
    lx = (BOARD_W + MARGIN) * S + 14
    txt(lx, MARGIN * S - 18, "引脚 ↔ 信号 (§19.10.2)", 10, "#111", anchor="start", weight="bold")
    for i, row in enumerate([("M1 4/5", "M2 6/7", "M3 15/16", "M4 17/18"),
                             ("E1 1/2", "E2 11/12", "E3 13/14", "E4 38/39"),
                             ("I2C 8/9", "云台 10/21", "超声 47/48", "备用 40/41/42")]):
        for j, s in enumerate(row):
            txt(lx + j * 74, MARGIN * S + 4 + i * 14, s, 8, "#111", anchor="start")
    y0 = MARGIN * S + 72
    txt(lx, y0, "装配顺序 (§19.9.4)", 10, "#111", anchor="start", weight="bold")
    for i, s in enumerate([
            "1. 背面: GND 裸铜母线 + 12V 飞线",
            "2. 矮件: 分压电阻 / 跳线",
            "3. 排母座(S3) / 端子排 / JST 座",
            "4. 高件: 电解电容 (极性!)",
            "5. 走线 + 热缩 + 板边扎带 + 网管",
            "6. 上电三查: 蜂鸣查短路→串保险→测三轨"]):
        txt(lx, y0 + 15 + i * 13, s, 8, "#333", anchor="start")
    y1 = y0 + 110
    txt(lx, y1, "配色", 9, "#111", anchor="start", weight="bold")
    for i, (c, s) in enumerate([("#c0392b", "12V 域 / 电源入口"), ("#2471a3", "降压模块"),
                                ("#b7950b", "驱动 / 电机"), ("#1e8449", "主控 S3"),
                                ("#7d3c98", "连接器"), ("#16a085", "3.3V / 5V 支线"),
                                ("#e67e22", "7.4V 云台轨")]):
        out.append(f'<rect x="{lx:.1f}" y="{y1+10+i*13:.1f}" width="10" height="8" fill="{c}"/>')
        txt(lx + 16, y1 + 18 + i * 13, s, 8, "#333", anchor="start")
    y2 = y1 + 115
    txt(lx, y2, "说明", 9, "#111", anchor="start", weight="bold")
    for i, s in enumerate(["· 每个实心圆都落在真实孔位上, 相邻孔距 2.54mm",
                           "· 行列号见板边标尺, 便于报位置/写装配笔记",
                           "· 排母座位置以实板丝印为准, 本图给列序",
                           "· 板背走线面朝车体内侧, 板下架尼龙柱 ≥5mm"]):
        txt(lx, y2 + 14 + i * 12, s, 7.4, "#555", anchor="start")
    out.append('</svg>')
    return "\n".join(out)


if __name__ == "__main__":
    sys.stdout.write(build())
