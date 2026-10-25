// ============================================================================
// Shadow Carrier V2 — ESP32-S3 麦轮底盘运动控制器（骨架，无实车预写）
// 规格：docs/V2_MECANUM_S3.md（推导/符号表/协议/安全/验收）
// 依赖：仅 Arduino-ESP32 core（LEDC analogWrite + IDF PCNT 驱动），无第三方库。
//
// 分层：
//   纯逻辑（PC 可测）: MecanumKinematics.{h,cpp} / WheelPid.{h,cpp} + tests/
//   硬件薄层（本文件）: PCNT 编码器、A4950 PWM/DIR、USB CDC 解析、安全看守
//
// 板端 bring-up 前必读：MecanumConfig.h 全部 TODO 项 + 文档 §9 五个实测确认点。
// ============================================================================

#include <Arduino.h>

#include "MecanumConfig.h"
#include "MecanumKinematics.h"
#include "WheelPid.h"

using mecanum::WHEEL_COUNT;  // 骨架全局数组/循环长度统一用它

// ---------------------------------------------------------------------------
// PCNT 适配：IDF5 (core 3.x) pulse_cnt.h = 正交闭环；其余情况退化为开环前馈。
// TODO(板端): 若被迫用 core 2.x，按同理补 IDF4 pcnt.h 版本；
// 无编码器时本骨架以开环 duty = u/WHEEL_V_MAX_MPS 运行（boot 遥测 enc=0）。
// ---------------------------------------------------------------------------
#if __has_include(<driver/pulse_cnt.h>)
#include "driver/pulse_cnt.h"
#define MEC_PCNT_V2 1
#endif

namespace {

// ---- 派生常量 -------------------------------------------------------------
constexpr int   kPwmMax = (1 << PWM_RES_BITS) - 1;
const    float  kWheelR = WHEEL_DIAMETER_MM * 0.5f * 0.001f;
const    float  kHalfL  = HALF_WHEELBASE_MM * 0.001f;
const    float  kHalfW  = HALF_TRACK_MM * 0.001f;

constexpr int   kPwmPins[WHEEL_COUNT]   = {PIN_M_FL_PWM, PIN_M_FR_PWM, PIN_M_RL_PWM, PIN_M_RR_PWM};
constexpr int   kDirPins[WHEEL_COUNT]   = {PIN_M_FL_DIR, PIN_M_FR_DIR, PIN_M_RL_DIR, PIN_M_RR_DIR};
constexpr float kMotorSign[WHEEL_COUNT] = {MOTOR_SIGN_FL, MOTOR_SIGN_FR, MOTOR_SIGN_RL, MOTOR_SIGN_RR};
constexpr int   kEncSign[WHEEL_COUNT]   = {ENC_SIGN_FL, ENC_SIGN_FR, ENC_SIGN_RL, ENC_SIGN_RR};

// 编码器 A/B 引脚（占位自设计文档 §19.10.2，TODO: 接线后核对）
constexpr int   kEncAPins[WHEEL_COUNT]  = {1, 11, 13, 38};
constexpr int   kEncBPins[WHEEL_COUNT]  = {2, 12, 14, 39};

// ---- 运行状态 ---------------------------------------------------------------
WheelPid gPid[WHEEL_COUNT];
float    gTargetWheels[WHEEL_COUNT] = {0, 0, 0, 0};   // m/s（已限幅/反解饱和）
float    gMeasWheels[WHEEL_COUNT]   = {0, 0, 0, 0};   // m/s（EMA 后）
float    gDuty[WHEEL_COUNT]         = {0, 0, 0, 0};   // 已缓启、未乘 MOTOR_SIGN
uint32_t gLastCmdMs  = 0;
uint32_t gLastTickMs = 0;
uint32_t gLastTelMs  = 0;
bool     gFaultLatched = false;
int      gFaultCode = 0;
uint32_t gStallSinceMs = 0;
bool     gStallTiming = false;
int32_t  gEncAbsCounts = 0;   // 本周期 |counts| 累计（区分堵转/编码器断线）
String   gLineBuf;

#if defined(MEC_PCNT_V2)
pcnt_unit_handle_t gPcntUnit[WHEEL_COUNT] = {nullptr, nullptr, nullptr, nullptr};
#endif

// ---- 编码器 -----------------------------------------------------------------
#if defined(MEC_PCNT_V2)
bool encoderBegin() {
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    pcnt_unit_config_t unitCfg = {};
    unitCfg.high_limit = 32767;
    unitCfg.low_limit = -32768;
    if (pcnt_new_unit(&unitCfg, &gPcntUnit[i]) != ESP_OK) return false;
    pcnt_glitch_filter_config_t glitch = {};
    glitch.max_glitch_ns = 1000;               // 1us 毛刺滤波（TODO: 干扰大则加大）
    pcnt_unit_set_glitch_filter(gPcntUnit[i], &glitch);

    // AB 正交 4x：各自沿计数，对方电平为高时反向（IDF 例程同款组合）。
    // 方向反了改 ENC_SIGN 宏，不换线（文档 §4.1）。
    pcnt_chan_handle_t chA = nullptr;
    pcnt_chan_handle_t chB = nullptr;
    pcnt_chan_config_t cfgA = {};
    cfgA.edge_gpio_num = kEncAPins[i];
    cfgA.level_gpio_num = kEncBPins[i];
    pcnt_chan_config_t cfgB = {};
    cfgB.edge_gpio_num = kEncBPins[i];
    cfgB.level_gpio_num = kEncAPins[i];
    if (pcnt_new_channel(gPcntUnit[i], &cfgA, &chA) != ESP_OK) return false;
    if (pcnt_new_channel(gPcntUnit[i], &cfgB, &chB) != ESP_OK) return false;
    pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_enable(gPcntUnit[i]);
    pcnt_unit_clear_count(gPcntUnit[i]);
    pcnt_unit_start(gPcntUnit[i]);
  }
  return true;
}

// 读取并清零一个控制周期的计数增量（乘 ENC_SIGN 归一到"正转=+"）。
int32_t encoderDelta(int i) {
  int count = 0;
  pcnt_unit_get_count(gPcntUnit[i], &count);
  pcnt_unit_clear_count(gPcntUnit[i]);
  return (int32_t)count * kEncSign[i];
}
#else
bool encoderBegin() { return false; }            // 开环退化模式
int32_t encoderDelta(int) { return 0; }
#endif  // MEC_PCNT_V2

// ---- 电机 -------------------------------------------------------------------
void writeMotor(int i, float duty /* 未乘 MOTOR_SIGN */) {
  const float d = duty * kMotorSign[i];
  const int val = (int)(fabsf(d) * (float)kPwmMax + 0.5f);
  if (d >= 0.0f) {
    digitalWrite(kDirPins[i], LOW);             // IN2=0，IN1=PWM
    analogWrite(kPwmPins[i], val);
  } else {
    digitalWrite(kPwmPins[i], LOW);             // IN1=0，IN2=PWM
    analogWrite(kDirPins[i], val);
  }
}

void motorsCoast() {  // A4950 双 IN 低 = 滑行（§19.4-4；坡道刹车模式 TODO）
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    analogWrite(kPwmPins[i], 0);
    analogWrite(kDirPins[i], 0);
    digitalWrite(kPwmPins[i], LOW);
    digitalWrite(kDirPins[i], LOW);
  }
}

// ---- 目标设置（命令层唯一入口）----------------------------------------------
void applyTwistCommand(const mecanum::Twist& t) {
  mecanum::Twist tc = t;
  tc.vy *= CAL_VY_SCALE;                        // §2.6 标定因子（默认 1）
  tc.wz *= CAL_WZ_SCALE;
  mecanum::clampTwistUniform(tc, VX_MAX_MPS, VY_MAX_MPS, WZ_MAX_RADS);
  mecanum::WheelSpeeds w;
  if (!mecanum::inverse(tc, kHalfL, kHalfW, w)) {  // 几何宏非法 → 拒动
    motorsCoast();
    return;
  }
  mecanum::desaturateUniform(w, WHEEL_V_MAX_MPS);  // 四轮同比例缩放，保方向
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    gTargetWheels[i] = w.v[i];
    gPid[i].setTarget(w.v[i]);
  }
  gLastCmdMs = millis();
  gFaultLatched = false;                        // 新有效运动命令清故障锁存（§7）
}

void commandStop(bool resetPid) {
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    gTargetWheels[i] = 0.0f;
    if (resetPid) gPid[i].reset();
    gDuty[i] = 0.0f;
  }
  motorsCoast();
}

// ---- 安全看守（§7：堵转 / 编码器断线）---------------------------------------
void safetyMonitor(uint32_t now) {
  bool stalled = false;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    if (fabsf(gDuty[i]) >= STALL_DUTY && fabsf(gMeasWheels[i]) < STALL_MPS) {
      stalled = true;
    }
  }
  if (!stalled) {
    gStallTiming = false;
    return;
  }
  if (!gStallTiming) {
    gStallTiming = true;
    gStallSinceMs = now;
    return;
  }
  if (now - gStallSinceMs > STALL_MS) {
    // 观测窗内完全无计数 → 疑编码器断线；有计数但速度近零 → 疑堵转。
    gFaultLatched = true;
    gFaultCode = (gEncAbsCounts == 0) ? FAULT_CODE_ENC : FAULT_CODE_STALL;
    commandStop(true);
    Serial.print("TEL FAULT ");
    Serial.println(gFaultCode);
  }
}

// ---- 100Hz 控制节拍 ----------------------------------------------------------
void controlTick(uint32_t now, uint32_t tickMs) {
  if (gFaultLatched) return;                    // 锁存：等待下一条有效运动命令解锁

  if (now - gLastCmdMs > CMD_TIMEOUT_MS) {      // V1 同款 450ms 超时
    commandStop(true);
    return;
  }

#if defined(MEC_PCNT_V2)
  const float dtS = (float)tickMs * 0.001f;
  // 轮速闭环：PCNT → m/s → EMA → PID → 缓启 → 写 PWM
  int32_t absWindow = 0;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    const int32_t dc = encoderDelta(i);
    absWindow += (dc < 0) ? -dc : dc;
    const float mps = mecanum::countsToMps(dc, (float)ENC_CPR, kWheelR, dtS);
    gMeasWheels[i] += ENC_EMA_ALPHA * (mps - gMeasWheels[i]);
    float duty = gPid[i].step(gMeasWheels[i], dtS);
    // 输出缓启（限每拍变化，复用 V1 防堵转思想）
    duty = gDuty[i] + constrain(duty - gDuty[i], -DUTY_RAMP_PER_TICK, DUTY_RAMP_PER_TICK);
    gDuty[i] = duty;
    writeMotor(i, duty);
  }
  gEncAbsCounts = absWindow;
  safetyMonitor(now);
#else
  // 开环退化（无编码器）：duty = u / WHEEL_V_MAX_MPS 前馈 + 缓启。
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    const float want = gTargetWheels[i] / WHEEL_V_MAX_MPS;
    const float d =
        gDuty[i] + constrain(want - gDuty[i], -DUTY_RAMP_PER_TICK, DUTY_RAMP_PER_TICK);
    gDuty[i] = d;
    writeMotor(i, d);
  }
#endif
}

// ---- 遥测 ---------------------------------------------------------------------
void telemetry(uint32_t now) {
  Serial.print("ENC ");
  Serial.print(now);
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    Serial.print(' ');
    Serial.print(gMeasWheels[i], 3);
  }
  Serial.println();
#if USE_MPU6050
  // TODO(V2a): MPU6050 gz 积分航向 + 原始 gz；在此之前不发 HDG（不发假数据）。
#endif
}

// ---- 命令解析辅助（V1 CommandParser 同风格）-----------------------------------
String tokenize(const String& text, int& startIndex) {
  while (startIndex < (int)text.length() && text[startIndex] == ' ') startIndex++;
  int end = text.indexOf(' ', startIndex);
  if (end == -1) {
    String tok = text.substring(startIndex);
    startIndex = text.length();
    return tok;
  }
  String tok = text.substring(startIndex, end);
  startIndex = end + 1;
  return tok;
}

bool parseThreeFloats(const String& text, int idx, float& a, float& b, float& c) {
  const String t1 = tokenize(text, idx);
  const String t2 = tokenize(text, idx);
  const String t3 = tokenize(text, idx);
  if (t1.length() == 0 || t2.length() == 0 || t3.length() == 0) return false;
  if (tokenize(text, idx).length() != 0) return false;   // 尾随垃圾 → 拒
  a = t1.toFloat();
  b = t2.toFloat();
  c = t3.toFloat();
  return true;
}

// ---- 命令分发（协议表：文档 §6）----------------------------------------------
void handleLine(const String& raw) {
  String line = raw;
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  if (line == "STOP") {
    commandStop(true);
    gLastCmdMs = millis();                      // STOP 自身重置超时窗口
    gFaultLatched = false;
    return;
  }
  if (line == "PING") {                         // PING 不刷新超时（V1 语义）
    Serial.println("PONG");
    return;
  }

  int idx = 0;
  const String act = tokenize(line, idx);

  if (act == "MEC") {                           // MEC <vx> <vy> <wz> (m/s, m/s, rad/s)
    mecanum::Twist t = {0, 0, 0};
    if (parseThreeFloats(line, idx, t.vx, t.vy, t.wz)) {
      applyTwistCommand(t);
    } else {
      commandStop(true);                        // V1 语义：解析失败即停车
      Serial.println("TEL ERR MEC");
    }
    return;
  }

  if (act == "DIFF") {                          // DIFF L<n> R<n> —— V1 一键回退
    const String lt = tokenize(line, idx);
    const String rt = tokenize(line, idx);
    if (lt.length() >= 2 && lt[0] == 'L' && rt.length() >= 2 && rt[0] == 'R' &&
        tokenize(line, idx).length() == 0) {
      mecanum::Twist t;
      if (mecanum::diffToTwist(lt.substring(1).toInt(), rt.substring(1).toInt(),
                               DIFF_MAX_MPS, DIFF_TRACK_M, t)) {
        applyTwistCommand(t);
        return;
      }
    }
    commandStop(true);
    Serial.println("TEL ERR DIFF");
    return;
  }

  if (act == "MOVE") {                          // MOVE F/B/L/R <n> —— V1 兼容
    const String dir = tokenize(line, idx);
    const String spd = tokenize(line, idx);
    if (dir.length() == 1 && spd.length() > 0 && tokenize(line, idx).length() == 0) {
      mecanum::Twist t;
      if (mecanum::moveToTwist(dir[0], spd.toInt(), MOVE_MAX_MPS, PIVOT_WZ_RADS, t)) {
        applyTwistCommand(t);
        return;
      }
    }
    commandStop(true);
    Serial.println("TEL ERR MOVE");
    return;
  }

  commandStop(true);                            // 未知命令 → 停（V1 语义）
  Serial.println("TEL ERR CMD");
}

}  // namespace

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < WHEEL_COUNT; ++i) {
    pinMode(kPwmPins[i], OUTPUT);
    pinMode(kDirPins[i], OUTPUT);
    gPid[i].configure(PID_KP, PID_KI, PID_KD, DUTY_MIN, DUTY_MAX, PID_ITERM_MAX);
    gPid[i].reset();
  }
  analogWriteResolution(PWM_RES_BITS);
#if MEC_PWM_FREQ_API_PERPIN
  for (int i = 0; i < WHEEL_COUNT; ++i) analogWriteFrequency(kPwmPins[i], PWM_FREQ_HZ);
#else
  analogWriteFrequency(PWM_FREQ_HZ);
#endif
  motorsCoast();

  const bool encOk = encoderBegin();
  Serial.print("TEL BOOT ");
  Serial.print(FW_VERSION);
  Serial.print(" enc=");
  Serial.println(encOk ? 1 : 0);                // enc=0 → 开环退化模式（文档 §10）

  const uint32_t now = millis();
  gLastCmdMs = now;
  gLastTickMs = now;
  gLastTelMs = now;
}

void loop() {
  while (Serial.available()) {                  // 按行收集
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (gLineBuf.length()) {
        handleLine(gLineBuf);
        gLineBuf = "";
      }
    } else if (gLineBuf.length() < 64) {
      gLineBuf += c;
    }
  }

  const uint32_t now = millis();
  if (now - gLastTickMs >= CTRL_TICK_MS) {
    const uint32_t tick = now - gLastTickMs;
    gLastTickMs = now;
    controlTick(now, tick);
  }
  if (now - gLastTelMs >= TEL_INTERVAL_MS) {
    gLastTelMs = now;
    telemetry(now);
  }
}
