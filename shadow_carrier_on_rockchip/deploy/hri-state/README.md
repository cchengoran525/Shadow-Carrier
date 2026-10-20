# HRI 行为状态机（板端常驻）

> 2026-10-20。A1.3 的**动作生产端**：`hri_state --live` 读 `/api/detections` + `owner.json`，
> 写 `/dev/shm/hri_state.json`（`{ts,state,action,parked}`）。**不直接控车**。
> 动作由 `rk_control` 的 HRI 动作桥消费（见 `scripts/rk_control.py` 的 `_hri_bridge_loop`）。

## 依赖
- `video-stream.service`（`/api/detections`）
- 认主：`/dev/shm/owner.json` 新鲜（跟随启动时 enroll；`--owner-source file` 只认它）
- 可选：`/dev/shm/grid.json`（`~/world_lab/fusion/grid.json`）用于 HIDE 选点

## 安装（板端）
```bash
sudo cp deploy/hri-state/hri-state.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now hri-state
systemctl status hri-state --no-pager
```

## 验证
```bash
# 1) 文件在更新
watch -n1 'cat /dev/shm/hri_state.json'
# 2) 状态机在跑
journalctl -u hri-state -n 20 --no-pager
# 3) 跟随模式下，动作应被动作桥消费
grep -n '\[hri\]' /tmp/hri_state.log 2>/dev/null | tail
```

## 注意
- `--owner-source file`：**主人未 enroll 时不会误跟路人**（宁可不动）。
- 动作桥只在 `mode=follow` 工作；手动模式不消费。
- 所有动作有界：APPROACH ≤3s、BACK_OFF 0.6s、GOTO_SAFE 转向 ≤3s + 前进 ≤1.5s；
  障碍 ≤30cm / 动作变化 / IMU 丢失 → STOP。
