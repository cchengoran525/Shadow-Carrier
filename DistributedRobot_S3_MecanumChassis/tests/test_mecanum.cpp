// ============================================================================
// 离线单测（PC，无 Arduino）：g++ 编译运行，脚本 tests/run_tests.sh
// 覆盖（对应 docs/V2_MECANUM_S3.md）：
//   A. 符号表 12 组断言（§3 表：3 指令 × 4 轮）
//   B. 逆→正 round-trip < 1%（§2.3/§2.4，27 组合含负数与混合）
//   C. 手算抽查（K = L+W 混合项）
//   D. 反解饱和（§5：四轮同比例、保方向）/ twist 限幅
//   E. DIFF / MOVE 兼容映射（§6）
//   F. countsToMps、PID 收敛与 anti-windup 冒烟
// ============================================================================
#include <cmath>
#include <cstdio>

#include "../MecanumKinematics.h"
#include "../WheelPid.h"

static int gPass = 0, gFail = 0;

#define CHECK(cond, msg)                                          \
  do {                                                            \
    if (cond) {                                                   \
      ++gPass;                                                    \
    } else {                                                      \
      ++gFail;                                                    \
      std::printf("  FAIL L%d: %s\n", __LINE__, msg);             \
    }                                                             \
  } while (0)

// 几何占位与固件一致（实车前均为 TODO，但符号结论与数值无关）
static const float L = 0.120f;   // 半轴距
static const float W = 0.135f;   // 半轮距
static const float K = L + W;    // 0.255
static const float R = 0.040f;   // 轮半径

using mecanum::Twist;
using mecanum::WheelSpeeds;

// ---------------------------------------------------------------- 符号表（12 组）
static void testSignTable() {
  std::printf("[A] 符号表 12 组断言（文档 §3）\n");
  WheelSpeeds w;

  // vx > 0 前进：四轮全正转
  Twist tf{0.3f, 0.0f, 0.0f};
  CHECK(mecanum::inverse(tf, L, W, w), "inverse vx ok");
  CHECK(w.v[mecanum::WHEEL_FL] > 1e-6f, "vx>0: FL 正转");
  CHECK(w.v[mecanum::WHEEL_FR] > 1e-6f, "vx>0: FR 正转");
  CHECK(w.v[mecanum::WHEEL_RL] > 1e-6f, "vx>0: RL 正转");
  CHECK(w.v[mecanum::WHEEL_RR] > 1e-6f, "vx>0: RR 正转");

  // vy > 0 左移：FL/RR 反转，FR/RL 正转
  Twist ts{0.0f, 0.3f, 0.0f};
  CHECK(mecanum::inverse(ts, L, W, w), "inverse vy ok");
  CHECK(w.v[mecanum::WHEEL_FL] < -1e-6f, "vy>0: FL 反转");
  CHECK(w.v[mecanum::WHEEL_FR] > 1e-6f, "vy>0: FR 正转");
  CHECK(w.v[mecanum::WHEEL_RL] > 1e-6f, "vy>0: RL 正转");
  CHECK(w.v[mecanum::WHEEL_RR] < -1e-6f, "vy>0: RR 反转");

  // wz > 0 逆时针：左退右进（同差速左转）
  Twist tr{0.0f, 0.0f, 1.0f};
  CHECK(mecanum::inverse(tr, L, W, w), "inverse wz ok");
  CHECK(w.v[mecanum::WHEEL_FL] < -1e-6f, "wz>0: FL 反转");
  CHECK(w.v[mecanum::WHEEL_FR] > 1e-6f, "wz>0: FR 正转");
  CHECK(w.v[mecanum::WHEEL_RL] < -1e-6f, "wz>0: RL 反转");
  CHECK(w.v[mecanum::WHEEL_RR] > 1e-6f, "wz>0: RR 正转");

  // 派生：右移整行取反
  Twist tsNeg{0.0f, -0.3f, 0.0f};
  mecanum::inverse(tsNeg, L, W, w);
  CHECK(w.v[mecanum::WHEEL_FL] > 1e-6f && w.v[mecanum::WHEEL_RR] > 1e-6f,
        "vy<0: FL/RR 取反");
  CHECK(w.v[mecanum::WHEEL_FR] < -1e-6f && w.v[mecanum::WHEEL_RL] < -1e-6f,
        "vy<0: FR/RL 取反");
}

// ------------------------------------------------------- 逆→正 round-trip < 1%
static void testRoundTrip() {
  std::printf("[B] 逆→正 round-trip（<1%%，27 组合）\n");
  const float vxs[3] = {-0.3f, 0.0f, 0.5f};
  const float vys[3] = {-0.2f, 0.0f, 0.4f};
  const float wzs[3] = {-1.0f, 0.0f, 2.0f};
  float maxRelErr = 0.0f;
  int cases = 0;
  for (float vx : vxs)
    for (float vy : vys)
      for (float wz : wzs) {
        Twist t{vx, vy, wz};
        WheelSpeeds w;
        Twist back{0, 0, 0};
        if (!mecanum::inverse(t, L, W, w) || !mecanum::forward(w, L, W, back)) {
          CHECK(false, "inverse/forward 返回 false");
          continue;
        }
        const float comp[3] = {back.vx, back.vy, back.wz};
        const float ref[3] = {t.vx, t.vy, t.wz};
        for (int j = 0; j < 3; ++j) {
          const float err = std::fabs(comp[j] - ref[j]);
          const float rel = err / std::fmax(std::fabs(ref[j]), 1e-3f);
          if (rel > maxRelErr) maxRelErr = rel;
          CHECK(rel < 0.01f, "round-trip 分量相对误差 <1%");
        }
        ++cases;
      }
  std::printf("  27 组合完成，最大相对误差 %.2e（阈值 1e-2）\n", (double)maxRelErr);
  CHECK(cases == 27, "组合数 = 27");
  CHECK(maxRelErr < 0.01f, "maxRelErr < 1%");
}

// ---------------------------------------------------------------- 手算抽查
static void testHandComputed() {
  std::printf("[C] 手算抽查（混合 K 项）\n");
  WheelSpeeds w;
  Twist t{0.5f, 0.2f, 0.0f};
  mecanum::inverse(t, L, W, w);
  CHECK(std::fabs(w.v[mecanum::WHEEL_FL] - 0.3f) < 1e-5f, "纯平移 FL=0.3");
  CHECK(std::fabs(w.v[mecanum::WHEEL_FR] - 0.7f) < 1e-5f, "纯平移 FR=0.7");
  CHECK(std::fabs(w.v[mecanum::WHEEL_RL] - 0.7f) < 1e-5f, "纯平移 RL=0.7");
  CHECK(std::fabs(w.v[mecanum::WHEEL_RR] - 0.3f) < 1e-5f, "纯平移 RR=0.3");

  // vx=0.3, vy=-0.2, wz=1.0, K=0.255（手算见文档 §2.3 代入）
  Twist m{0.3f, -0.2f, 1.0f};
  mecanum::inverse(m, L, W, w);
  CHECK(std::fabs(w.v[mecanum::WHEEL_FL] - 0.245f) < 1e-5f, "混合 FL=0.245");
  CHECK(std::fabs(w.v[mecanum::WHEEL_FR] - 0.355f) < 1e-5f, "混合 FR=0.355");
  CHECK(std::fabs(w.v[mecanum::WHEEL_RL] - (-0.155f)) < 1e-5f, "混合 RL=-0.155");
  CHECK(std::fabs(w.v[mecanum::WHEEL_RR] - 0.755f) < 1e-5f, "混合 RR=0.755");

  // 非法几何必须拒动（固件 applyTwistCommand 依赖此返回值）
  Twist junk{0, 0, 0};
  CHECK(!mecanum::inverse(junk, 0.0f, W, w), "L<=0 拒绝");
  CHECK(!mecanum::forward(w, L, -1.0f, junk), "W<0 拒绝");
}

// ------------------------------------------------------------ 饱和与限幅
static void testDesaturation() {
  std::printf("[D] 反解饱和（保方向弃幅值）与 twist 限幅\n");
  // 超速指令 → 四轮同比例缩到上限，方向不变
  Twist big{1.2f, 0.8f, 3.0f};
  WheelSpeeds w;
  mecanum::inverse(big, L, W, w);
  const float s = mecanum::desaturateUniform(w, 1.0f);
  CHECK(s < 1.0f && s > 0.0f, "超限时施加缩放 <1");
  float peak = 0.0f;
  for (int i = 0; i < 4; ++i) peak = std::fmax(peak, std::fabs(w.v[i]));
  CHECK(peak <= 1.0f + 1e-6f, "缩放后 max|u| <= 1.0");

  Twist t2{0, 0, 0};
  mecanum::forward(w, L, W, t2);
  // 方向保持：三分量同比例于原指令（比例 = s）
  CHECK(std::fabs(t2.vx / s - big.vx) < 1e-3f, "方向保持 vx");
  CHECK(std::fabs(t2.vy / s - big.vy) < 1e-3f, "方向保持 vy");
  CHECK(std::fabs(t2.wz / s - big.wz) < 1e-3f, "方向保持 wz");

  // 未超限不动
  WheelSpeeds wOk;
  Twist ok{0.1f, 0.1f, 0.1f};
  mecanum::inverse(ok, L, W, wOk);
  CHECK(mecanum::desaturateUniform(wOk, 1.4f) == 1.0f, "未超限缩放 =1");

  // twist 限幅：任一分量超限 → 整体等比
  Twist c{0.8f, 0.5f, 2.5f};
  const float sc = mecanum::clampTwistUniform(c, 0.8f, 0.5f, 2.0f);
  CHECK(std::fabs(sc - 0.8f) < 1e-5f, "限幅比例 = 2.0/2.5");
  CHECK(std::fabs(c.vx - 0.64f) < 1e-5f && std::fabs(c.vy - 0.4f) < 1e-5f &&
            std::fabs(c.wz - 2.0f) < 1e-5f,
        "整体等比缩放");
}

// ------------------------------------------------------------ V1 兼容映射
static void testV1Compat() {
  std::printf("[E] DIFF / MOVE 兼容映射（§6）\n");
  Twist t;
  CHECK(mecanum::diffToTwist(100, 100, 0.8f, 0.27f, t), "diff 解析 ok");
  CHECK(std::fabs(t.wz) < 1e-6f, "L=R 直行 wz=0");
  CHECK(std::fabs(t.vx - 0.8f * 100.0f / 255.0f) < 1e-5f, "vx = (uL+uR)/2");
  CHECK(std::fabs(t.vy) < 1e-6f, "DIFF 无横移");

  CHECK(mecanum::diffToTwist(150, 100, 0.8f, 0.27f, t), "diff 弧线 ok");
  CHECK(t.wz < 0.0f, "左快右慢 → 顺时针（wz<0，V1 右转语义）");
  CHECK(t.vx > 0.0f, "弧线仍前进");

  CHECK(mecanum::diffToTwist(999, -5, 0.8f, 0.27f, t), "diff 越界钳位 ok");
  CHECK(std::fabs(t.vx - 0.4f) < 1e-5f, "999/-5 钳到 255/0 → vx=(0.8+0)/2");
  CHECK(std::fabs(t.wz - (-0.8f / 0.27f)) < 1e-4f, "wz = (0-0.8)/0.27");

  CHECK(mecanum::moveToTwist('F', 255, 0.8f, 1.5f, t) && std::fabs(t.vx - 0.8f) < 1e-5f,
        "MOVE F 满档 = +MOVE_MAX");
  CHECK(mecanum::moveToTwist('B', 255, 0.8f, 1.5f, t) && t.vx < 0.0f, "MOVE B 后退");
  CHECK(mecanum::moveToTwist('L', 255, 0.8f, 1.5f, t) && std::fabs(t.wz - 1.5f) < 1e-5f,
        "MOVE L 原地左转");
  CHECK(mecanum::moveToTwist('R', 255, 0.8f, 1.5f, t) && t.wz < 0.0f, "MOVE R 原地右转");
  CHECK(std::fabs(t.vx) < 1e-6f && std::fabs(t.vy) < 1e-6f, "MOVE L/R 只有 wz");
  CHECK(!mecanum::moveToTwist('X', 128, 0.8f, 1.5f, t), "MOVE 非法方向拒绝");
}

// ------------------------------------------------------- countsToMps / PID 冒烟
static void testEncoderAndPid() {
  std::printf("[F] countsToMps 与 PID 冒烟\n");
  const float expect = 6.28318530717958647692f * 0.040f;  // 一整转 / 1s
  const float got = mecanum::countsToMps(1320, 1320.0f, 0.040f, 1.0f);
  CHECK(std::fabs(got - expect) < 1e-4f, "1320 counts/1s = 一转轮速 0.2513 m/s");
  CHECK(mecanum::countsToMps(-660, 1320.0f, 0.040f, 1.0f) < 0.0f, "反转计数为负速");
  CHECK(mecanum::countsToMps(100, 0.0f, 0.040f, 1.0f) == 0.0f, "cpr=0 防呆");

  // 一阶轮系模拟：tau=0.12s，duty 1.0 → 1 m/s；目标 0.3 m/s，100Hz
  WheelPid pid;
  pid.configure(0.8f, 6.0f, 0.0f, -1.0f, 1.0f, 0.4f);
  pid.setTarget(0.3f);
  float v = 0.0f;
  const float dt = 0.01f;
  for (int i = 0; i < 300; ++i) {  // 3s
    const float duty = pid.step(v, dt);
    v += (duty - v) / 0.12f * dt;
  }
  CHECK(std::fabs(v - 0.3f) < 0.03f, "PID 阶跃收敛 ±10%（实测误差见输出）");
  std::printf("  PID 3s 后轮速 %.4f m/s（目标 0.3）\n", (double)v);

  // anti-windup：不可达目标（5 m/s，duty 顶格 2s）→ 目标回零应立即反向出力
  pid.reset();
  pid.setTarget(5.0f);
  v = 0.0f;
  for (int i = 0; i < 200; ++i) {
    const float duty = pid.step(v, dt);
    v += (duty - v) / 0.12f * dt;
  }
  pid.setTarget(0.0f);
  const float dutyAfter = pid.step(v, dt);
  CHECK(dutyAfter < 0.0f, "饱和后目标回零，duty 立即为负（积分未失控）");
  std::printf("  饱和 2s 后目标回零，duty = %.3f\n", (double)dutyAfter);

  pid.reset();
  CHECK(pid.target() == 0.0f, "reset 清目标");
}

int main() {
  std::printf("V2 麦轮 S3 离线单测（几何占位 L=%.3f W=%.3f K=%.3f r=%.3f）\n\n",
              (double)L, (double)W, (double)K, (double)R);
  testSignTable();
  testRoundTrip();
  testHandComputed();
  testDesaturation();
  testV1Compat();
  testEncoderAndPid();

  std::printf("\n==== %d passed, %d failed ====\n", gPass, gFail);
  return gFail == 0 ? 0 : 1;
}
