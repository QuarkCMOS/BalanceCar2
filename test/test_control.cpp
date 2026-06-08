// ================================================================
// test_control.cpp — Unit tests cho PIDController & BalanceController
//
// Framework: Unity (tích hợp sẵn trong PlatformIO)
// Chạy trên native host (không cần phần cứng thật):
//   pio test -e native
//
// Thêm vào platformio.ini:
//   [env:native]
//   platform = native
//   test_build_src = yes
//   build_flags = -std=c++17
// ================================================================

#include <unity.h>
#include <cmath>

// ── Stub Arduino functions (native host không có Arduino.h) ─────
#ifndef ARDUINO
    inline int constrain(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    inline float constrain(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
#endif

// ── Stub MotorDriver ─────────────────────────────────────────────
// Dùng cấu trúc tối giản — KHÔNG có copy constructor
// (HBridgeMotor thật cũng không có → pattern giống nhau)
struct MotorPinConfig { int pinIN1, pinIN2; };

class MotorDriver {
public:
    // Chỉ cho phép construct từ MotorPinConfig
    // Copy và move đều bị xóa để phản ánh đúng class thật
    explicit MotorDriver(const MotorPinConfig& cfg) : _cfg(cfg) {}
    MotorDriver(const MotorDriver&)            = delete;
    MotorDriver& operator=(const MotorDriver&) = delete;

    void begin()         {}
    void setSpeed(int s) { lastSpeed = s; }
    void hardBrake()     { lastSpeed = 0; brakeCount++; }
    void softBrake()     { lastSpeed = 0; }
    void coast()         { lastSpeed = 0; }
    int getCurrentSpeed() const { return lastSpeed; }

    // Spy fields
    int lastSpeed  = 0;
    int brakeCount = 0;

private:
    MotorPinConfig _cfg;
};

// ── Stub constants ───────────────────────────────────────────────
constexpr float PID_KP           = 20.0f;
constexpr float PID_KI           = 0.5f;
constexpr float PID_KD           = 0.8f;
constexpr float PID_OUTPUT_MAX   = 1023.0f;
constexpr float PID_INTEGRAL_MAX = 200.0f;
constexpr float TARGET_ANGLE     = 0.0f;
constexpr float FALL_ANGLE       = 30.0f;
constexpr float ALPHA            = 0.98f;
constexpr int   PWM_MAX          = 1023;
constexpr int   PWM_DEADBAND     = 30;

// Include source thật
#include "../src/control/pid_controller.h"
#include "../src/control/balance_controller.h"

// ================================================================
// Helper: tạo motor tại chỗ — KHÔNG copy, dùng direct init
// Macro tạo MotorPinConfig để truyền vào constructor
// ================================================================
#define LEFT_CFG  MotorPinConfig{26, 25}
#define RIGHT_CFG MotorPinConfig{33, 32}

static const float DT = 0.005f;   // 5ms — khớp LOOP_INTERVAL_US = 5000

// ================================================================
// ──────────────  PID CONTROLLER TESTS  ──────────────
// ================================================================

// ── TC-PID-01: Zero error → output = 0 ──────────────────────────
void test_pid_zero_error_zero_output() {
    PIDController pid;
    pid.setTunings(20.0f, 0.5f, 0.8f);
    pid.setOutputLimits(-1023.0f, 1023.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    float out = pid.compute(0.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out);
}

// ── TC-PID-02: Proportional term chính xác ──────────────────────
void test_pid_proportional_only() {
    PIDController pid;
    pid.setTunings(10.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-5000.0f, 5000.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    // error = 5 → P = 10 * 5 = 50
    float out = pid.compute(5.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, out);
}

// ── TC-PID-03: Integral tích lũy theo thời gian ─────────────────
void test_pid_integral_accumulates() {
    PIDController pid;
    pid.setTunings(0.0f, 100.0f, 0.0f);
    pid.setOutputLimits(-10000.0f, 10000.0f);
    pid.setIntegralLimits(-10000.0f, 10000.0f);

    // Sau 3 bước error=1, dt=0.01 → integral=0.03 → iTerm=3.0
    float dt = 0.01f, out = 0.0f;
    for (int i = 0; i < 3; i++) out = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.0f, out);
}

// ── TC-PID-04: Integral clamp chống windup ──────────────────────
void test_pid_integral_clamp() {
    PIDController pid;
    pid.setTunings(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-10000.0f, 10000.0f);
    pid.setIntegralLimits(-5.0f, 5.0f);

    for (int i = 0; i < 1000; i++) pid.compute(100.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, pid.getIntegral());
}

// ── TC-PID-05: Output clamp ──────────────────────────────────────
void test_pid_output_clamp() {
    PIDController pid;
    pid.setTunings(1000.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-100.0f, 100.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    float out = pid.compute(50.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, out);

    pid.reset();
    out = pid.compute(-50.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -100.0f, out);
}

// ── TC-PID-06: Derivative bỏ qua lần đầu (no spike) ─────────────
void test_pid_derivative_no_spike_first_call() {
    PIDController pid;
    pid.setTunings(0.0f, 0.0f, 1000.0f);
    pid.setOutputLimits(-10000.0f, 10000.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    float out = pid.compute(10.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out);
}

// ── TC-PID-07: Derivative tính đúng lần thứ 2 ───────────────────
void test_pid_derivative_second_call() {
    PIDController pid;
    pid.setTunings(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-10000.0f, 10000.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    float dt = 0.01f;
    pid.compute(4.0f, 0.0f, dt);                // lần 1 — skip
    float out = pid.compute(3.0f, 0.0f, dt);    // lần 2 — dError=(3-4)/0.01=-100
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, out);
}

// ── TC-PID-08: Reset xóa trạng thái ─────────────────────────────
void test_pid_reset_clears_state() {
    PIDController pid;
    pid.setTunings(10.0f, 1.0f, 0.5f);
    pid.setOutputLimits(-1023.0f, 1023.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    for (int i = 0; i < 50; i++) pid.compute(5.0f, 0.0f, DT);
    TEST_ASSERT_TRUE(pid.getIntegral() != 0.0f);

    pid.reset();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.getIntegral());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.getPreviousError());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.getLastOutput());
}

// ── TC-PID-09: dt <= 0 → trả output trước, không crash ──────────
void test_pid_invalid_dt_safe() {
    PIDController pid;
    pid.setTunings(20.0f, 0.5f, 0.8f);
    pid.setOutputLimits(-1023.0f, 1023.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    float out1 = pid.compute(5.0f, 0.0f, DT);
    float out2 = pid.compute(5.0f, 0.0f, 0.0f);
    float out3 = pid.compute(5.0f, 0.0f, -0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, out1, out2);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, out1, out3);
}

// ── TC-PID-10: Output dương khi đang dưới setpoint ──────────────
void test_pid_positive_output_when_below_setpoint() {
    PIDController pid;
    pid.setTunings(20.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1023.0f, 1023.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    // setpoint=0, measurement=-5 → error=5 → output dương
    float out = pid.compute(0.0f, -5.0f, DT);
    TEST_ASSERT_TRUE(out > 0.0f);
}

// ── TC-PID-11: Setters và getters đúng ──────────────────────────
void test_pid_setters_getters() {
    PIDController pid;
    pid.setTunings(1.1f, 2.2f, 3.3f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.1f, pid.getKp());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.2f, pid.getKi());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.3f, pid.getKd());
}

// ── TC-PID-12: P-only steady-state error ────────────────────────
void test_pid_proportional_steady_state_error() {
    PIDController pid;
    pid.setTunings(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1023.0f, 1023.0f);
    pid.setIntegralLimits(-200.0f, 200.0f);

    float out = pid.compute(10.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out);
}

// ================================================================
// ──────────────  BALANCE CONTROLLER TESTS  ──────────────
// ================================================================

// ── TC-BAL-01: begin() trả true và state = IDLE ──────────────────
void test_balance_begin_returns_true() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);

    TEST_ASSERT_TRUE(bal.begin());
    TEST_ASSERT_EQUAL(BalanceState::IDLE, bal.getState());
}

// ── TC-BAL-02: enable() chuyển sang RUNNING ──────────────────────
void test_balance_enable_sets_running() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    TEST_ASSERT_EQUAL(BalanceState::RUNNING, bal.getState());
}

// ── TC-BAL-03: disable() dừng motor và về IDLE ───────────────────
void test_balance_disable_brakes_motor() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();
    bal.update(2.0f, DT);
    bal.disable();

    TEST_ASSERT_EQUAL(BalanceState::IDLE, bal.getState());
    TEST_ASSERT_EQUAL(0, left.lastSpeed);
    TEST_ASSERT_EQUAL(0, right.lastSpeed);
}

// ── TC-BAL-04: update() khi IDLE không ghi lệnh motor ────────────
void test_balance_update_idle_no_motor_command() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    // Không gọi enable()

    left.lastSpeed  = 999;
    right.lastSpeed = 999;
    bal.update(5.0f, DT);

    TEST_ASSERT_EQUAL(999, left.lastSpeed);
    TEST_ASSERT_EQUAL(999, right.lastSpeed);
}

// ── TC-BAL-05: RUNNING với góc nhỏ → motor được điều khiển ──────
void test_balance_running_small_angle_drives_motor() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    // Góc 5° → error=-5, Kp=20 → output=-100 (lớn hơn deadband)
    bal.update(5.0f, DT);
    TEST_ASSERT_TRUE(bal.getPIDOutput() != 0.0f);
}

// ── TC-BAL-06: Góc vượt FALL_ANGLE → FALLEN, brake ───────────────
void test_balance_fallen_when_angle_exceeds_limit() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(35.0f, DT);   // 35° > 30°

    TEST_ASSERT_EQUAL(BalanceState::FALLEN, bal.getState());
    TEST_ASSERT_EQUAL(0, left.lastSpeed);
    TEST_ASSERT_EQUAL(0, right.lastSpeed);
    TEST_ASSERT_TRUE(left.brakeCount > 0);
}

// ── TC-BAL-07: Góc âm cũng bị detect ngã ────────────────────────
void test_balance_fallen_negative_angle() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(-35.0f, DT);
    TEST_ASSERT_EQUAL(BalanceState::FALLEN, bal.getState());
}

// ── TC-BAL-08: Tự phục hồi sau khi được dựng lại ────────────────
void test_balance_auto_recover_after_fallen() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(35.0f, DT);   // ngã
    bal.update(5.0f, DT);    // dựng lại — 5° < 15° (FALL_ANGLE * 0.5)

    TEST_ASSERT_EQUAL(BalanceState::RUNNING, bal.getState());
}

// ── TC-BAL-09: Hysteresis — 16° chưa đủ để phục hồi ─────────────
void test_balance_fallen_hysteresis() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(35.0f, DT);   // ngã
    bal.update(16.0f, DT);   // 16° > 15° → vẫn FALLEN

    TEST_ASSERT_EQUAL(BalanceState::FALLEN, bal.getState());
}

// ── TC-BAL-10: setTargetAngle() ──────────────────────────────────
void test_balance_set_target_angle() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();

    bal.setTargetAngle(2.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, bal.getTargetAngle());
}

// ── TC-BAL-11: leftPWM, rightPWM khớp với motor ─────────────────
void test_balance_pid_output_matches_motor_speed() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(3.0f, DT);

    TEST_ASSERT_EQUAL(bal.getLeftPWM(), bal.getRightPWM());
    TEST_ASSERT_EQUAL(bal.getLeftPWM(),  left.lastSpeed);
    TEST_ASSERT_EQUAL(bal.getRightPWM(), right.lastSpeed);
}

// ── TC-BAL-12: setPIDTunings() runtime không crash ───────────────
void test_balance_set_pid_tunings_runtime() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.setPIDTunings(25.0f, 1.0f, 1.2f);
    bal.update(1.0f, DT);
    TEST_PASS();
}

// ── TC-BAL-13: getCurrentAngle() phản ánh góc đầu vào ───────────
void test_balance_current_angle_updated() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(7.5f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.5f, bal.getCurrentAngle());
}

// ── TC-BAL-14: Zero angle → PID output = 0 ───────────────────────
void test_balance_zero_angle_zero_motor() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    // Lần đầu: error=0, integral=0, derivative skip → output=0
    bal.update(0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, bal.getPIDOutput());
}

// ── TC-BAL-15: enable() gọi 2 lần không reset PID giữa chừng ────
void test_balance_double_enable_is_safe() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();
    bal.enable();  // lần 2 — không có gì xảy ra

    TEST_ASSERT_EQUAL(BalanceState::RUNNING, bal.getState());
}

// ================================================================
// ──────────────  INTEGRATION TESTS  ──────────────
// ================================================================

// ── TC-INT-01: 100 bước mô phỏng — xe không ngã ──────────────────
// Model: output âm khi angle dương → angle += output*gain → angle giảm về 0
void test_integration_100_steps_no_fall() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    float angle = 5.0f;
    bool  fell  = false;

    for (int i = 0; i < 100; i++) {
        bal.update(angle, DT);

        if (bal.getState() == BalanceState::FALLEN) { fell = true; break; }

        // Feedback: output âm khi angle dương → angle giảm
        angle += bal.getPIDOutput() * 0.001f;
        if (angle >  25.0f) angle =  25.0f;
        if (angle < -25.0f) angle = -25.0f;
    }

    TEST_ASSERT_FALSE(fell);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, angle);
}

// ── TC-INT-02: Nhiễu đột ngột 10° — phục hồi không ngã ───────────
void test_integration_impulse_disturbance() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    float angle = 0.0f;

    for (int i = 0; i < 200; i++) {
        if (i == 50) angle += 10.0f;   // nhiễu tại bước 50

        bal.update(angle, DT);
        if (bal.getState() == BalanceState::FALLEN) break;

        angle += bal.getPIDOutput() * 0.001f;
        if (angle >  25.0f) angle =  25.0f;
        if (angle < -25.0f) angle = -25.0f;
    }

    TEST_ASSERT_NOT_EQUAL(BalanceState::FALLEN, bal.getState());
}

// ── TC-INT-03: Fall detection ngay khi góc vượt ngưỡng ───────────
void test_integration_immediate_fall_detection() {
    MotorDriver left(LEFT_CFG), right(RIGHT_CFG);
    BalanceController bal(left, right);
    bal.begin();
    bal.enable();

    bal.update(31.0f, DT);   // 31° > 30° → FALLEN ngay lập tức
    TEST_ASSERT_EQUAL(BalanceState::FALLEN, bal.getState());
}

// ================================================================
// Setup & Loop (PlatformIO Unity runner)
// ================================================================

void setup() {
    UNITY_BEGIN();

    // PID tests
    RUN_TEST(test_pid_zero_error_zero_output);
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_integral_accumulates);
    RUN_TEST(test_pid_integral_clamp);
    RUN_TEST(test_pid_output_clamp);
    RUN_TEST(test_pid_derivative_no_spike_first_call);
    RUN_TEST(test_pid_derivative_second_call);
    RUN_TEST(test_pid_reset_clears_state);
    RUN_TEST(test_pid_invalid_dt_safe);
    RUN_TEST(test_pid_positive_output_when_below_setpoint);
    RUN_TEST(test_pid_setters_getters);
    RUN_TEST(test_pid_proportional_steady_state_error);

    // Balance Controller tests
    RUN_TEST(test_balance_begin_returns_true);
    RUN_TEST(test_balance_enable_sets_running);
    RUN_TEST(test_balance_disable_brakes_motor);
    RUN_TEST(test_balance_update_idle_no_motor_command);
    RUN_TEST(test_balance_running_small_angle_drives_motor);
    RUN_TEST(test_balance_fallen_when_angle_exceeds_limit);
    RUN_TEST(test_balance_fallen_negative_angle);
    RUN_TEST(test_balance_auto_recover_after_fallen);
    RUN_TEST(test_balance_fallen_hysteresis);
    RUN_TEST(test_balance_set_target_angle);
    RUN_TEST(test_balance_pid_output_matches_motor_speed);
    RUN_TEST(test_balance_set_pid_tunings_runtime);
    RUN_TEST(test_balance_current_angle_updated);
    RUN_TEST(test_balance_zero_angle_zero_motor);
    RUN_TEST(test_balance_double_enable_is_safe);

    // Integration tests
    RUN_TEST(test_integration_100_steps_no_fall);
    RUN_TEST(test_integration_impulse_disturbance);
    RUN_TEST(test_integration_immediate_fall_detection);

    UNITY_END();
}

void loop() {}