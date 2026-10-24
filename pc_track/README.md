# PC 科研轨（SLAM 沙盒）

Roadmap §5 的实施目录：**只在 PC 上、离线、无死线**；机器人侧零依赖（车上不跑 SLAM）。

- 研究文档（选型/约束/契约/实验计划）：`../docs/PC_SLAM_TRACK.md`
- `slam/`：X3Pro 定制 slam_toolbox 配置 + 首图一条龙脚本 + 零依赖存图
- `data/`：实验产物（bag / PGM 地图 / posegraph），git 不跟踪（见 .gitignore 约定）

## 跑首图

```bash
# 前置: X3Pro 插 PC、驱动在跑（/scan 有发布者）
cd slam && ./first_map.sh 60      # 手持雷达慢速绕房间一圈; RVIZ=1 可视化
```
