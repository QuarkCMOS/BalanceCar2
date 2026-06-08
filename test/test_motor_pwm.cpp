#include <Arduino.h>
#include "config/pins.h"
#include "config/constants.h"

// ────────────────────────────────────────────────────────────────
// MOCK MotorDriver
// ────────────────────────────────────────────────────────────────
struct MockMotorDriver {
    String name;
    int    speed = 0;

    MockMotorDriver(const String& n) : name(n) {}

    void begin() {
        Serial.printf("[INIT] %s  GPIO_IN1=%d  GPIO_IN2=%d  freq=%dHz  max=%d\n",
                      name.c_str(),
                      (name == "L") ? PIN_MOTOR_L1 : PIN_MOTOR_R1,
                      (name == "L") ? PIN_MOTOR_L2 : PIN_MOTOR_R2,
                      PWM_FREQ_HZ, PWM_MAX);
    }

    // Trả về speed thực sự sau deadband/clamp
    int setSpeed(int s) {
        s = constrain(s, -PWM_MAX, PWM_MAX);
        if (abs(s) < PWM_DEADBAND) {
            speed = 0;
        } else {
            speed = s;
        }
        return speed;
    }

    void hardBrake() { speed = 0; }
    void softBrake() { speed = 0; }
};

// ────────────────────────────────────────────────────────────────
// Teleplot helpers
// ────────────────────────────────────────────────────────────────

// Plot single value
inline void tplot(const char* name, float value) {
    Serial.print(">");
    Serial.print(name);
    Serial.print(":");
    Serial.println(value, 2);
}

// Plot left, right, diff và target cùng lúc
void tplotMotors(int left, int right, int target = 0) {
    tplot("pwm_left",  (float)left);
    tplot("pwm_right", (float)right);
    tplot("pwm_diff",  (float)(left - right));
    tplot("target",    (float)target);
    tplot("deadband_hi",  (float) PWM_DEADBAND);   // đường kẻ tham chiếu
    tplot("deadband_lo",  (float)-PWM_DEADBAND);
}

// ────────────────────────────────────────────────────────────────
// Mock PWMManager
// ────────────────────────────────────────────────────────────────
struct MockPWMManager {
    MockMotorDriver left  = MockMotorDriver("L");
    MockMotorDriver right = MockMotorDriver("R");

    void begin() {
        left.begin();
        right.begin();
    }

    void write(int l, int r) {
        int al = left.setSpeed(l);
        int ar = right.setSpeed(r);
        tplotMotors(al, ar);
    }

    void emergencyBrake() {
        left.hardBrake();
        right.hardBrake();
        tplotMotors(0, 0);
        Serial.println("[EMERGENCY BRAKE]");
    }

    void softBrake() {
        left.softBrake();
        right.softBrake();
        tplotMotors(0, 0);
    }
};

// ────────────────────────────────────────────────────────────────
// TEST SCENARIOS
// ────────────────────────────────────────────────────────────────
MockPWMManager pwm;

// Ramp tốc độ từ start → end trong steps bước, delay ms giữa mỗi bước
void ramp(int startL, int endL, int startR, int endR,
          int steps, int delayMs) {
    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / steps;
        int   l  = (int)(startL + t * (endL - startL));
        int   r  = (int)(startR + t * (endR - startR));
        pwm.write(l, r);
        delay(delayMs);
    }
}

void section(const char* name) {
    // Dòng không có ">" → hiện trong Teleplot console
    Serial.printf("\n[SECTION] %s\n", name);
    delay(100);
}

// ────────────────────────────────────────────────────────────────
// MAIN
// ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(5000);

    Serial.println("[BOOT] Balance Bot — Motor PWM Test");
    Serial.printf("[INFO] PWM_MAX=%d  DEADBAND=%d  FREQ=%dHz\n",
                  PWM_MAX, PWM_DEADBAND, PWM_FREQ_HZ);
    pwm.begin();
    delay(300);

    // ── 1. Deadband sweep ─────────────────────────────────────
    // Thấy rõ ngưỡng deadband trên chart
    section("DEADBAND SWEEP");
    for (int s = 0; s <= PWM_DEADBAND + 20; s += 2) {
        pwm.write(s, s);
        delay(60);
    }
    pwm.write(0, 0);
    delay(200);

    // ── 2. Forward ramp ───────────────────────────────────────
    section("FORWARD RAMP 0 -> MAX -> 0");
    ramp(0, PWM_MAX, 0, PWM_MAX, 40, 40);
    ramp(PWM_MAX, 0, PWM_MAX, 0, 40, 40);
    delay(200);

    // ── 3. Reverse ramp ───────────────────────────────────────
    section("REVERSE RAMP 0 -> -MAX -> 0");
    ramp(0, -PWM_MAX, 0, -PWM_MAX, 40, 40);
    ramp(-PWM_MAX, 0, -PWM_MAX, 0, 40, 40);
    delay(200);

    // ── 4. Turn sim ───────────────────────────────────────────
    // pwm_diff thể hiện rõ trên chart
    section("TURN SIMULATION");
    ramp(PWM_MAX, PWM_MAX/2, PWM_MAX/2, PWM_MAX, 20, 50);  // curve right
    delay(100);
    ramp(PWM_MAX/2, PWM_MAX, PWM_MAX, PWM_MAX/2, 20, 50);  // curve left
    delay(100);
    // Spin in place
    ramp(0, 512, 0, -512, 20, 40);
    delay(200);
    ramp(512, 0, -512, 0, 20, 40);
    delay(200);

    // ── 5. Balance PID simulation ─────────────────────────────
    // Simulate robot bị đẩy rồi tự cân bằng
    section("BALANCE PID SIMULATION");
    Serial.println("[INFO] Simulating tilt disturbance and recovery");

    // angle profile: lean forward → PID corrects → settle
    float angleProfile[] = {
        0.0f, 0.3f, 0.8f, 1.5f, 2.5f, 4.0f, 5.5f,  // bị đẩy
        4.0f, 2.5f, 1.2f, 0.5f, 0.2f, 0.0f,          // PID kéo về
        -0.3f, -0.8f, -0.5f, -0.2f, 0.0f              // overshoot nhỏ
    };

    for (float angle : angleProfile) {
        int output = (int)(angle * PID_KP);
        output = constrain(output, -PWM_MAX, PWM_MAX);

        // Plot thêm angle để so sánh với PWM trên cùng chart
        tplot("angle_x10", angle * 10.0f);  // scale x10 để nhìn rõ hơn

        if (abs(angle) >= FALL_ANGLE) {
            Serial.printf("[WARN] angle=%.1f >= FALL_ANGLE=%.0f\n",
                          angle, FALL_ANGLE);
            pwm.emergencyBrake();
        } else {
            pwm.write(output, output);
        }
        delay(120);
    }

    // ── 6. Emergency brake test ───────────────────────────────
    section("EMERGENCY BRAKE");
    pwm.write(PWM_MAX, PWM_MAX);
    delay(200);
    pwm.emergencyBrake();
    delay(300);

    // Flatline về 0
    tplotMotors(0, 0);
    Serial.println("[DONE] All tests complete — check Teleplot charts");
}

void loop() {
    // Không làm gì — test chạy 1 lần trong setup()
    // Giữ kết nối Serial để Teleplot không disconnect
    delay(1000);
}