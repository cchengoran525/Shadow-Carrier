# 停驻域网格刷新（grid.json）

> 2026-10-20。HIDE 选点需要新鲜的 `~/world_lab/fusion/grid.json`；但 `geometry_daemon once`
> **会动云台**，所以只能在**停驻域（WAIT/HIDE）**跑，跟随/移动中禁止。
> 本单元用 systemd timer 每 20s 检查一次：只有 `hri_state.json` 新鲜且 `parked=true` 时才抓一帧。

## 依赖
- `rk-control.service`（`/gimbal` 接口）
- `hri-state.service`（`/dev/shm/hri_state.json`）
- `state/calib/` 标定文件；相机可用

## 安装（板端）
```bash
sudo cp deploy/geometry/geometry-parked.service deploy/geometry/geometry-parked.timer /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now geometry-parked.timer
```

## 验证
```bash
systemctl list-timers geometry-parked --no-pager
journalctl -u geometry-parked -n 20 --no-pager
ls -l ~/world_lab/fusion/grid.json   # ts 应该是最近 1 分钟内
python3 - <<'PY'
import json,time,os
p=os.path.expanduser('~/world_lab/fusion/grid.json')
g=json.load(open(p)); print('age', round(time.time()-g.get('ts',0),1),'s',
      'sectors', bool(g.get('sector_scores')), 'confident',
      (g.get('sector_scores') or {}).get('confident'))
PY
```

## 注意
- **只在 `parked=true` 时跑**；否则脚本直接退出（退出码 0）。
- `once` 会把云台移到工作位，跑完不自动归中；停驻域下可接受。
- 若 `grid.json` 一直不更新：先看 `hri-state` 是否在跑、`hri_state.json` 的 `parked` 是否为 true。
- 该单元是 **P1**：不装也能跑跟随/passby；只影响 HIDE 分镜。
