# Gimbal — 两自由度舵机云台（v2：C3 直驱方案）

摄像头两轴云台（Pan 水平 / Tilt 垂直）。**v2 架构：舵机由 ESP32-C3 的 LEDC 硬件 PWM 直驱**，
RK3566 只发角度命令，不再自己生成 PWM。

## 为什么放弃 RK3566 直接驱动（2026-08-22 实测结论）

| 原计划 | 实测结果 |
|---|---|
| Pan=GPIO4_A6(134) / Tilt=GPIO4_A7(135) | ❌ 软件能翻转，但**官方引脚图确认这两个脚不在 30pin 排针上**（万用表实测无任何脚跳动） |
| 排针其他 GPIO | ❌ 全被占用：GPIO3_C6~D7=音频I2S/PDM、GPIO4_A0/A1=PDM、GPIO4_A2/A3=uart7、GPIO4_A4/A5=uart9、GPIO1_A0/A1=i2c3 |
| 改 DTB 释放 16/18 脚 | ❌ p3/p4 是 U-Boot FIT 镜像（内核+dtb 打包+SHA256 校验），重打包变砖风险高 |

排针上唯一空闲的 GPIO 是 **28 脚（GPIO0_C6）**，只有一个不够用。
而 C3 Super Mini 空闲引出脚有 GPIO0/1/10/21（GPIO11 接内部 Flash，板上没有），足够。

## v2 架构

```
RK3566 (决策/视觉)                    C3 (执行)
gimbal_follow.py / rk_control.py ──USB CDC──> PAN <deg> / TLT <deg> 命令
        ↑ YOLO检测API(:8080)                  → LEDC 硬件PWM 50Hz → 舵机
```

- 舵机无编码器，**指令角即真实角**，无需回传；将来加 IMU/角度传感器走 `TEL:` 遥测行
- USB 全双工，C3 回显 `GOT:` / `PAN:xx.x`

## 接线（最终版）

| 舵机线 | 接哪 |
|--------|------|
| Pan 橙线 | **C3 GPIO0** |
| Tilt 橙线 | **C3 GPIO1** |
| 红 ×2 | 独立供电：电池(2S锂电7.4V)→二极管降压(~1.4V)或 mini360 模块调6V；MG996R 可容忍直连 |
| 棕 ×2 | 与 C3 **共地** |

⚠️ 不要用 KickPi 板载 5V 或 C3 板上电源带舵机（峰值 >1A，会电压跌落导致断连）。

## 固件改动（communication/C3_USB_Controller.ino）

- 新命令 `PAN <deg>` / `TLT <deg>`，超范围自动钳制，回 `PAN:x.x` 确认
- LEDC 50Hz @14bit（步进≈0.17°），兼容 arduino-esp32 2.x/3.x API
- 标定常量：`SERVO_PULSE_MIN_US=500` / `SERVO_PULSE_MAX_US=2500`，行程不对改这里
- 上电回中位 Pan=90°/Tilt=90°（1500µs 电学中位；机械正向偏移属正常，待标定）
- 烧录必须带 `CDCOnBoot=cdc`

## 板端脚本

| 文件 | 说明 |
|------|------|
| `scripts/rk_control.py` (v5) | 新增 POST `/gimbal {pan,tilt}` 和 `/gimbal/center`；网页新增云台按钮（⟲⟳⬆⬇◎，按住连发，仅遥控模式）；切跟随模式自动回中。旧版备份 rk_control.py.v4.bak |
| `gimbal/gimbal_follow.py` (v2) | 自动追随最大 person；termios 直开 `/dev/c3_controller` 发命令（无 pyserial 依赖、不再需要 sudo 跑PWM）；P 控制 + 20Hz 限频 |

## 手动测试

```bash
# 烧录新固件并接好舵机后:
echo "PAN 200" > /dev/c3_controller   # 水平应转动
echo "TLT 45"  > /dev/c3_controller   # 俯仰应转动
# 或网页遥控面板直接点云台按钮
# 或 curl -X POST http://127.0.0.1/gimbal -d '{"pan":200,"tilt":90}'
```

## 后续路线

1. ✅ 遥控界面玩舵机（rk_control v5）
2. ⬜ opencv+yolo 视觉深化
3. ⬜ 现实坐标解码（像素+pan/tilt+内参 → 方位角，即"云台坐标解耦"）
4. ⬜ 解码完成后把 gimbal_follow.py 的自动跟随挂进模式系统（注意与 rk_control 串口独占问题，届时合并为单一 owner）

## 凝视控制器设计（跟随模式云台自动化，v3 规划）

> M1 完成后本线的主战场：把云台从"手动遥控玩具"升级为跟随模式下的**主动视线管理**，
> 并把云台信息反哺运动与环境感知。设计定稿 2026-08-27，实施节奏见文末里程碑。

### 触发状态机（滞回防抖，边沿不抽搐）

| 状态 | 进入条件 | 行为 |
|------|----------|------|
| **TRACK** | 主人检出且 conf≥CONF_MIN（连续5帧确认） | 解码方位角→限幅追踪 |
| **HOLD** | 丢检 <1.5s | 角度冻结，KF 盲推外推 |
| **SEARCH** | 丢检 1.5~4s | 最后方位 ±20° 慢扫（ReID 重捕获舞台） |
| **CENTER** | 丢检 >4s 或 STOP | 归中 90.58°，低功耗待命 |

- 迁移滞回：检出→丢检需连续5帧，丢检恢复1帧即回——边界不抖
- 恢复优先级：HOLD 直接回 TRACK 不经 SEARCH

### 接口形态（单一管道，来源可换）

```
GazeTarget { pan_deg, tilt_deg, quality(0~1), source(TRACK/HOLD/MANUAL...) }
    → 限速 60°/s + 死区 ±2° → 写 C3
```

手动拖拽与自动跟随走同一条管道，source 字段可追溯。

### 与底盘的绞合（分配律：小偏差云台管，大偏差底盘管）

```
                     ┌─ |pan_err| ≤ 25°:  底盘直行，云台微调
真方位(解码输出) ──┤
                     └─ |pan_err| > 25° 持续0.5s: DIFF 弧线转向
                                          + 云台反向补偿保持人在画面
```

- **滞回**：进入转向阈值 25°，退出 12°——防阈值附近左扭右扭
- **超声波联动**：C3 侧 BLOCKED 时底盘冻结，云台继续盯人（撞墙与跟随是两件事）
- **follow_controller 升级路径**：输入从"bbox 在画面哪儿"换成"解码方位角"，差速执行框架复用

### 反向信息流（云台=免费航姿传感器）

1. **Yaw 残差在线校正**：DIFF 转向指令 vs 解码方位角变化量的残差 = 底盘转向效能实测，
   在线校正 follow_controller 的转向增益（电池电压/地面摩擦漂移顺手解决）
2. **ṙ/θ̇ 行为语义**（tracker 输出，M2）：
   - `ṙ < -0.3m/s` 主人快速走近 → HRI: RECEIVE 预备
   - `ṙ > +0.5m/s 持续5s` 主人远离 → HRI: WAIT 判定的量化证据
3. **极坐标投影**：`(方位θ, 距离r)` 打进机体系网格(8扇区×3距离环) → [世界] grid 度量增强，
   "躲门边/避开障碍"变成可计算选址

### 串口 owner 落定

凝视控制器**并入 rk_control 进程**（manual/follow 模式互锁即天然仲裁），MVP 不另立 daemon。
正式版如需拆分，先建 owner 守护进程再迁移。

### 里程碑与验收

| 周 | 目标 | 成功判据 |
|---|---|---|
| W1 | 凝视控制器雏形并入 rk_control | 跟随中死角时间占比 <10% |
| W2 | KF 接入，HOLD 桥接丢检 | 3s 遮挡不断跟 |
| W3 | 双层分配律落地 | 直行时段占比↑，DIFF 转向频率↓ |
| W4 | ṙ/θ̇ 特征输出 | HRI RECEIVE/WAIT 判定有数值依据 |

### 草案常数（实施时收进 config，全部待实机调优）

```
PAN_ERR_ENTER=25°   PAN_ERR_EXIT=12°   ERR_HOLD_S=0.5
HOLD_TIMEOUT=1.5s   SEARCH_TIMEOUT=4s  SEARCH_AMP=±20°
RATE_LIMIT=60°/s    DEADZONE=±2°       CONF_MIN=0.5(与[认主]对齐)
```

## 已知遗留

- kickpi 用户不在 dialout 组，手动跑脚本需 sudo（或给 udev 规则加 MODE="0666"）
- 板子 DNS 曾坏过：修复方法 `echo nameserver 8.8.8.8 > /etc/resolv.conf`
