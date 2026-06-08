// ================================================================
// test_gpio_pwm.cpp — test GPIO thật bằng LEDC, quan sát LED
//
// KHÔNG dùng mock — ghi thẳng ra GPIO để LED sáng thật
// Gắn LED + 220Ω vào:
//   GPIO 26 (L_IN1) → LED → GND
//   GPIO 25 (L_IN2) → LED → GND
//   GPIO 33 (R_IN1) → LED → GND
//   GPIO 32 (R_IN2) → LED → GND
// ================================================================

#include <Arduino.h>
#include "config/pins.h"
#include "config/constants.h"

// LEDC channel assignment — mỗi pin một channel riêng
#define CH_L_IN1  0
#define CH_L_IN2  1
#define CH_R_IN1  2
#define CH_R_IN2  3

#define PWM_BITS  10      // resolution 10-bit → 0..1023, khớp PWM_MAX

// ────────────────────────────────────────────────────────────────
// LEDC helpers
// ────────────────────────────────────────────────────────────────
void pwmSetup() {
    ledcSetup(CH_L_IN1, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(CH_L_IN2, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(CH_R_IN1, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(CH_R_IN2, PWM_FREQ_HZ, PWM_BITS);

    ledcAttachPin(PIN_MOTOR_L1, CH_L_IN1);
    ledcAttachPin(PIN_MOTOR_L2, CH_L_IN2);
    ledcAttachPin(PIN_MOTOR_R1, CH_R_IN1);
    ledcAttachPin(PIN_MOTOR_R2, CH_R_IN2);

    Serial.printf("[LEDC] freq=%dHz  bits=%d  max=%d\n",
                  PWM_FREQ_HZ, PWM_BITS, PWM_MAX);
    Serial.printf("[PIN]  L_IN1=GPIO%d(ch%d)  L_IN2=GPIO%d(ch%d)\n",
                  PIN_MOTOR_L1, CH_L_IN1, PIN_MOTOR_L2, CH_L_IN2);
    Serial.printf("[PIN]  R_IN1=GPIO%d(ch%d)  R_IN2=GPIO%d(ch%d)\n",
                  PIN_MOTOR_R1, CH_R_IN1, PIN_MOTOR_R2, CH_R_IN2);
}

// speed: -PWM_MAX → +PWM_MAX
// dương → IN1 chạy PWM, IN2 = 0  (LED_IN1 sáng)
// âm    → IN1 = 0, IN2 chạy PWM  (LED_IN2 sáng)
// 0     → cả 2 = 0                (cả 2 tắt)
void setMotorL(int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);
    if (abs(speed) < PWM_DEADBAND) {
        ledcWrite(CH_L_IN1, 0);
        ledcWrite(CH_L_IN2, 0);
    } else if (speed > 0) {
        ledcWrite(CH_L_IN1, speed);
        ledcWrite(CH_L_IN2, 0);
    } else {
        ledcWrite(CH_L_IN1, 0);
        ledcWrite(CH_L_IN2, -speed);
    }
}

void setMotorR(int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);
    if (abs(speed) < PWM_DEADBAND) {
        ledcWrite(CH_R_IN1, 0);
        ledcWrite(CH_R_IN2, 0);
    } else if (speed > 0) {
        ledcWrite(CH_R_IN1, speed);
        ledcWrite(CH_R_IN2, 0);
    } else {
        ledcWrite(CH_R_IN1, 0);
        ledcWrite(CH_R_IN2, -speed);
    }
}

void hardBrake() {
    // IN1=IN2=MAX → cả 2 LED sáng full = DRV8833 short brake
    ledcWrite(CH_L_IN1, PWM_MAX);
    ledcWrite(CH_L_IN2, PWM_MAX);
    ledcWrite(CH_R_IN1, PWM_MAX);
    ledcWrite(CH_R_IN2, PWM_MAX);
}

void coast() {
    ledcWrite(CH_L_IN1, 0);
    ledcWrite(CH_L_IN2, 0);
    ledcWrite(CH_R_IN1, 0);
    ledcWrite(CH_R_IN2, 0);
}

// In status + Teleplot
void report(int l, int r, const char* label = "") {
    float pctL = abs(l) * 100.0f / PWM_MAX;
    float pctR = abs(r) * 100.0f / PWM_MAX;
    Serial.printf("[PWM] L=%5d(%5.1f%%)  R=%5d(%5.1f%%)  %s\n",
                  l, pctL, r, pctR, label);
    // Teleplot
    Serial.printf(">pwm_left:%d\n>pwm_right:%d\n", l, r);
}

// ────────────────────────────────────────────────────────────────
// TEST CASES — delay dài hơn để mắt thấy LED thay đổi
// ────────────────────────────────────────────────────────────────

void testDeadband() {
    Serial.println("\n[TEST] DEADBAND — LED không sáng khi speed < 30");
    int vals[] = {0, 10, 20, 29, 30, 50};
    for (int v : vals) {
        setMotorL(v); setMotorR(v);
        report(v, v);
        delay(800);   // đủ lâu để quan sát
    }
    coast();
    delay(500);
}

void testRampUp() {
    Serial.println("\n[TEST] RAMP UP — LED sáng dần từ mờ → sáng full");
    for (int s = 0; s <= PWM_MAX; s += 50) {
        setMotorL(s); setMotorR(s);
        report(s, s);
        delay(300);
    }
    coast(); delay(600);
}

void testRampDown() {
    Serial.println("\n[TEST] RAMP DOWN — LED mờ dần → tắt");
    for (int s = PWM_MAX; s >= 0; s -= 50) {
        setMotorL(s); setMotorR(s);
        report(s, s);
        delay(300);
    }
    coast(); delay(600);
}

void testReverse() {
    Serial.println("\n[TEST] REVERSE — LED_IN1 tắt, LED_IN2 sáng");
    for (int s = 0; s >= -PWM_MAX; s -= 50) {
        setMotorL(s); setMotorR(s);
        report(s, s, s == 0 ? "(coast)" : "(reverse)");
        delay(300);
    }
    coast(); delay(600);
}

void testDirection() {
    Serial.println("\n[TEST] DIRECTION — quan sát LED đổi chiều");
    Serial.println("       Forward: IN1 sang, IN2 tat");
    setMotorL(600); setMotorR(600);
    report(600, 600, "FORWARD");
    delay(2000);

    Serial.println("       Reverse: IN1 tat, IN2 sang");
    setMotorL(-600); setMotorR(-600);
    report(-600, -600, "REVERSE");
    delay(2000);

    coast(); delay(500);
}

void testBrake() {
    Serial.println("\n[TEST] BRAKE — hardBrake: ca 2 LED sang full");
    setMotorL(700); setMotorR(700);
    report(700, 700, "running...");
    delay(1500);

    Serial.println("       Hard brake!");
    hardBrake();
    Serial.println(">pwm_left:0\n>pwm_right:0");
    delay(2000);

    coast();
    delay(500);
}

void testTurn() {
    Serial.println("\n[TEST] TURN — L va R khac nhau");
    Serial.println("       Curve right: L=full R=half");
    setMotorL(PWM_MAX); setMotorR(PWM_MAX / 2);
    report(PWM_MAX, PWM_MAX / 2, "curve right");
    delay(2000);

    Serial.println("       Spin: L=forward R=reverse");
    setMotorL(512); setMotorR(-512);
    report(512, -512, "spin");
    delay(2000);

    coast(); delay(500);
}

// ────────────────────────────────────────────────────────────────
// MAIN
// ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n[BOOT] GPIO PWM test — LED hardware");
    Serial.println("[INFO] Gan LED + 220ohm vao GPIO 26,25,33,32");

    pwmSetup();
    delay(300);

    testDeadband();
    testRampUp();
    testRampDown();
    testReverse();
    testDirection();
    testBrake();
    testTurn();

    coast();
    Serial.println("\n[DONE] Test xong. LED phai tat.");
}

void loop() {
    delay(1000);
}