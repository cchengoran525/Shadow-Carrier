# world_lab — [世界] 线实验记录与设计决策

> 独立实验区，不依赖/不修改工程任何现有代码。
> 收编进仓库时整体迁移为 `world/` 模块。

## 定位

世界模型 = 让决策端(HRI)知道"周围有什么、哪里能走"。
**不只是认门**——门会开、场景会变，门只是区域(region)的一个填充者。
所有接口按通用语义设计，预留扩展。

## 架构定稿：三层各管各的（2026-08-24）

| 层 | 模型/方法 | 频率 | 职责 |
|---|---|---|---|
| 快语义 | yolov8n INT8（现有 yolo_daemon） | 10-15Hz | 追人、COCO 类物体 |
| 慢语义 | **YOLO-World v2s INT8**（待部署） | **1Hz** | 开放词汇：门/冰箱/饮水机/障碍类别 |
| 几何 | OpenCV（本目录 v3 管线） | 1-2Hz | 可通行性/墙地交界/门框几何验证 |

依据（2026-08 网络调研）：
- RK3566 NPU 只有 INT8 可用（12-18FPS），FP16 掉到 3-5FPS 且 CPU 回退
- yolov8n INT8 ≈ 18FPS；YOLO-World v2s 估 4-8FPS → 只能当慢环
- NPU 预算：yolov8n 55ms×10Hz + YW 200ms×1Hz ≈ 75% 峰值，可行但 YW 不得超 1Hz
- "检测器+几何地图=语义网格"是 2025 机器人主流架构（DIV-Nav/OpenVox/OneMap），方向验证无误

### YOLO-World 部署路径（未完成）
1. PC 装 rknn-toolkit2 **2.0.0**（1.3/1.6 有报错前科）
2. 词汇表**导出时定死**（prompt-then-detect 重参数化进权重），建议词表：
   `door, doorway, refrigerator, vending machine, water dispenser, couch, chair, table, potted plant, tv, staircase, person`
3. `model_zoo/examples/yolo_world/python/convert.py` 转 v2s INT8 (rk3566)；clip_text 只支持 FP16，**离线跑一次**存文本嵌入即可
4. sftp 上板 → 先离线测速（验收线 ≥3FPS）→ 接入融合架构当慢语义环
5. 板上无外网，模型文件 PC 下载后传输

## 实验记录

### B1 门检测三轮（2026-08-24，全部板上实测）

| 轮 | 方案 | 结果 |
|---|---|---|
| v1 | 固定阈值 Canny 50/150 + Hough | ❌ 低对比度门框 0 候选 |
| v2 | 自适应 Canny(median±) + 膨胀 + 共线段合并 + 内部边缘密度 | ✅ demo 门正对/斜对均检出，但有无门槛误报 |
| v3 | + 几何先验（宽高比 0.25~1.3、门框底边须达画面 60% 高） | ✅ **回归基准 3/3 命中、0 误报，参数冻结** |

- 基准集：正对门 / 斜对门 / 走廊门（必须检出）+ 宿舍 / 卫生间（必须零误报）
- 验收脚本 `regression.py`；**改 `params.py` 任何参数必须重跑基准**
- 单帧耗时（带负载）：340~1570ms → 几何环定档 1-2Hz
- 已知边界：消失点仅走廊等强结构场景可靠（不进 grid.json）；杂乱场景 Hough 会飙到 3.5s，需限线数/降分辨率/早退（待做）
- 门检测最终角色：给 YOLO-World 的 door 检出做**几何验证**与兜底，不单独当主力

### 融合第零步（2026-08-24）

`fusion/fusion_test.py`：MJPEG 抓帧(只读) + `/api/detections`(只读) + v3 几何 → 单帧网格快照
- ✅ 机制全通：YOLO bbox 落格正确（obj:tv 对准显示器）
- 标签体系（通用槽位，预留扩展）：`free / blocked / region:<name> / obj:<class>`
- 优先级：obj > region > blocked > free
- 待做：60s 连续压测；帧与 detections 有几十毫秒错位，需量化偏差

### grid.json v1 字段草案（[世界]→[HRI] 冻结接口）

```json
{
  "ts": 0.0,
  "frame_size": [640, 480],
  "objects":  [ {"cls": "person|door|refrigerator|...", "conf": 0.0,
                 "bbox": [x1,y1,x2,y2], "cell": [r,c], "source": "yolo_world|yolov8n|hough"} ],
  "grid":     [ [{"label": "free|blocked|region:*|obj:*", "sources": [], "conf": 0.0}, ...8], ...3 ],
  "timing_ms": {"geometry": 0, "total": 0}
}
```
- `objects[]` 完全开放类别，HRI 按 cls 查询，不写死业务语义
- 待加：地面单应映射后的极坐标（8扇区×3距离环），当前 grid 是图像空间占位版

## 文件清单

| 文件 | 说明 |
|---|---|
| `params.py` | 冻结参数（改动须过回归） |
| `door_v3.py` | 门检测/几何管线（终版） |
| `regression.py` | 验收脚本（基准 8 张照片） |
| `offline_door_test.py / _v2.py` | v1/v2 历史版本（留档） |
| `fusion/fusion_test.py` | YOLO+OpenCV 单帧融合 |
| `photos/` | 基准照片 + 各轮标注图 |

## 下一步

1. 融合 60s 压测 + 帧错位量化
2. 地面单应标定（相机俯角）→ 极坐标网格替换图像网格
3. YOLO-World 转换上板测速（PC 侧环境）
4. geometry_daemon 常驻化 → 产出 grid.json → 交 [HRI]
