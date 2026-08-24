#!/usr/bin/env python3
"""world_lab v3 冻结参数 —— 三轮回归后不许再动
选定依据: 5张基准照片(2正门样+1走廊门+2负样) 一次选定
"""

# --- 检测管线 ---
CANNY_LO_K = 0.55          # 自适应Canny下阈 = median * K
CANNY_HI_K = 1.45          # 上阈 = median * K
HOUGH_THRESHOLD = 40
HOUGH_MINLEN_RATIO = 0.18  # 相对画面高
HOUGH_MAXGAP = 25
MERGE_ANG_TOL = 6.0        # 共线合并角度容差(度)
MERGE_GAP_TOL = 40         # 共线合并端点间隙(px)
MERGE_DIST_TOL = 8         # 中点到直线距离容差(px)

# --- 竖线筛选 ---
V_ANG_TOL = 14             # 垂直线角度容差(度)
V_MINLEN_RATIO = 0.22      # 最短竖线占画面高比例

# --- 门候选判定 (冻结, 修改需回归基准通过) ---
DOOR_SCORE_MIN = 450       # 最低得分
DOOR_INNER_DENS_MAX = 0.12 # 门内边缘密度上限 (真门<0.10, 杂物区>0.13)
DOOR_ASPECT_MIN = 0.25     # 宽/高 下限 (单扇门叶约0.33)
DOOR_ASPECT_MAX = 1.30     # 宽/高 上限 (双开门约0.5~0.9)
DOOR_BOTTOM_MIN_RATIO = 0.60  # 门框底边必须到达画面高度60%以下(门是落地的)

# --- 可通行性网格 ---
GRID_ROWS = 3
GRID_COLS = 8
FREE_DENSITY_MAX = 0.06
FREE_VAR_MIN = 200

WORK_W = 640
