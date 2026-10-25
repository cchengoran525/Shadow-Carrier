#ifndef MECANUM_CONFIG_H
#define MECANUM_CONFIG_H

// ============================================================================
// V2 麦轮底盘 S3 固件 —— 全部硬件相关常数占位。
// 规则：【所有标 TODO 的值在实车 bring-up 前都是猜测】，按 docs/V2_MECANUM_S3.md
// §8/§9 逐项实测后回填。符号逻辑不依赖任何具体数值。
// ============================================================================

// ---- 几何（TODO：§9-5 实测；当前占位来自 V2_CHASSIS_GIMBAL_DESIGN §2.2b/§10.1）
#define WHEEL_DIAMETER_MM      80.0f   // TODO: 麦轮滚动直径实测（辊子公差 ±2mm 常见）
#define HALF_WHEELBASE_MM      120.0f  // TODO: 半轴距 L（x 方向，轮心到车心）
#define HALF_TRACK_MM          135.0f  // TODO: 半轮距 W（y 方向）
#define CAL_VY_SCALE           1.0f    // TODO: 横移标定（辊子地面投影角≠45° 时≠1，文档 §2.6）
#define CAL_WZ_SCALE           1.0f    // TODO: 自转标定（同上）

// ---- 编码器（TODO：§9-4 实测）
#define ENC_CPR                1320    // TODO: 轮轴每转计数（11PPR×4×30；廉价霍尔板可能只有一半）
#define ENC_EMA_ALPHA          0.5f    // 速度一阶滤波系数（0..1，小=更平滑更迟钝）
#define ENC_SIGN_FL            (+1)    // TODO: 正转（驱车前进方向）时计数应增加，否则改 -1
#define ENC_SIGN_FR            (+1)    // TODO
#define ENC_SIGN_RL            (+1)    // TODO
#define ENC_SIGN_RR            (+1)    // TODO

// ---- 电机引脚（占位来自设计文档 §19.10.2，TODO: 按实际接线核对）
#define PIN_M_FL_PWM           4       // A4950 板1-CH1 IN1
#define PIN_M_FL_DIR           5       //                                IN2
#define PIN_M_FR_PWM           6       // A4950 板1-CH2
#define PIN_M_FR_DIR           7
#define PIN_M_RL_PWM           15      // A4950 板2-CH1
#define PIN_M_RL_DIR           16
#define PIN_M_RR_PWM           17      // A4950 板2-CH2
#define PIN_M_RR_DIR           18
#define MOTOR_SIGN_FL          (+1.0f) // TODO: 架空点动——正 duty 必须驱动车体 +x，否则改 -1
#define MOTOR_SIGN_FR          (+1.0f) // TODO:（左右镜像安装的电机常一正一负）
#define MOTOR_SIGN_RL          (+1.0f) // TODO
#define MOTOR_SIGN_RR          (+1.0f) // TODO

// ---- PWM（LEDC 由 Arduino core 自动分配通道）
#define PWM_FREQ_HZ            20000   // 20kHz 超听域（§12.4）；TODO: 核对 core API 支持值
#define PWM_RES_BITS           10      // 0..1023
#define MEC_PWM_FREQ_API_PERPIN 1      // 1 = Arduino-ESP32 core 3.x 每脚 analogWriteFrequency；
                                       // 0 = core 2.x 全局 analogWriteFrequency。TODO: 按核心版本定

// ---- 速度限幅（§12.3 安全分层）
#define VX_MAX_MPS             0.8f    // 巡航上限
#define VY_MAX_MPS             0.5f    // TODO: 横移限幅（bring-up 从小值起）
#define WZ_MAX_RADS            2.0f
#define WHEEL_V_MAX_MPS        1.4f    // 单轮线速度上限（反解饱和阈值；空载极速约 1.38）

// ---- V1 兼容映射
#define DIFF_MAX_MPS           0.8f    // TODO: DIFF 满档 255 对应车速（实测量）
#define DIFF_TRACK_M           0.270f  // TODO: 差速等效轨距（≈2×半轮距；复现 V1 弧线行为用）
#define MOVE_MAX_MPS           0.8f    // MOVE F/B 满档对应车速
#define PIVOT_WZ_RADS          1.5f    // TODO: MOVE L/R 原地转角速度（参照 pivot_calib.py）

// ---- 轮速环（100Hz）
#define CTRL_TICK_MS           10
#define PID_KP                 0.8f    // TODO: 台架整定（duty per m/s 误差）
#define PID_KI                 6.0f    // TODO
#define PID_KD                 0.0f    // TODO
#define PID_ITERM_MAX          0.4f    // 积分钳位
#define DUTY_MIN               (-1.0f)
#define DUTY_MAX               (+1.0f)
#define DUTY_RAMP_PER_TICK     0.08f   // 输出缓启动（复用 V1 缓启防堵转思想）/拍

// ---- 安全（§7）
#define CMD_TIMEOUT_MS         450     // V1 同款：无命令即滑行停
#define STALL_DUTY             0.5f    // duty 超此值且轮速近零 → 疑似堵转/编码器故障
#define STALL_MPS              0.02f
#define STALL_MS               600
#define FAULT_CODE_STALL       1
#define FAULT_CODE_ENC         2
#define ENABLE_HW_WDT          0       // TODO: bring-up 开（esp_task_wdt，loop 挂死→重启）
#define LOOP_WDT_MS            200

// ---- 遥测
#define TEL_INTERVAL_MS        100     // ENC/HDG 10Hz
#define FW_VERSION             "0.1.0-skeleton"

// ---- 可选外设（骨架未实现，接口预留）
#define USE_MPU6050            0       // 航向环（启用后发 HDG；实现 TODO V2a）
#define USE_ULTRASONIC         0       // 超声急停（V1 逻辑迁移 TODO V2a）
#define PIN_I2C_SDA            8
#define PIN_I2C_SCL            9
#define PIN_SONAR_TRIG         47
#define PIN_SONAR_ECHO         48      // 需 1k/2k 分压（V1 老坑）

#endif  // MECANUM_CONFIG_H
