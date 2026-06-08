// ================================================================
// test_motor_basic.cpp — Test từng motor riêng lẻ
//
// Upload env:  test_motor
//
// Gõ lệnh qua Serial Monitor (115200):
//   l  → Test motor TRÁI  (TC-01..04)
//   r  → Test motor PHẢI  (TC-01..04)
//   b  → Test cả HAI motor cùng lúc
//   s  → Dừng ngay lập tức
//   h  → Hiện menu
//
// Kịch bản mỗi lần test (10s/kịch bản, nghỉ 2s giữa):
//   TC-01  TIẾN  70% MAX
//   TC-02  LÙI   70% MAX
//   TC-03  TIẾN  30% MAX  (kiểm tra deadband)
//   TC-04  Ramp  0 → 100% → 0
//
// Teleplot: >L_speed:value  >R_speed:value
// ================================================================

#include <Arduino.h>
#include "drivers/pwm_manager.h"

// ── Cấu hình test ────────────────────────────────────────────────
static constexpr int      SPEED_70   = PWM_MAX * 70 / 100;
static constexpr int      SPEED_30   = PWM_MAX * 30 / 100;
static constexpr uint32_t RUN_MS     = 10000;
static constexpr uint32_t PAUSE_MS   = 2000;
static constexpr uint32_t LOG_MS     = 500;
static constexpr uint32_t RAMP_STEP  = 100;

PWMManager pwm;

// ── Teleplot helpers ─────────────────────────────────────────────
static void tplot(const char* name, float val) {
    Serial.printf(">%s:%.1f\n", name, val);
}
static void tplotBoth(int l, int r) {
    tplot("L_speed", (float)l);
    tplot("R_speed", (float)r);
}

static void stopAll(bool loud = true) {
    pwm.emergencyBrake();
    tplotBoth(0, 0);
    if (loud) Serial.println("  >> DUNG");
}

static void header(int num, const char* title) {
    stopAll(false);
    delay(PAUSE_MS);
    Serial.println();
    Serial.println("----------------------------------------");
    Serial.printf(" TC-%02d: %s\n", num, title);
    Serial.println("----------------------------------------");
    delay(200);
}

// ── Chạy cố định, gõ 's' để dừng sớm ───────────────────────────
static bool runFixed(int l, int r, uint32_t durationMs) {
    pwm.write(l, r);
    tplotBoth(l, r);

    uint32_t t0 = millis(), lastLog = 0;
    while (millis() - t0 < durationMs) {
        uint32_t elapsed = millis() - t0;
        tplotBoth(l, r);
        if (elapsed - lastLog >= LOG_MS) {
            Serial.printf("  t=%5ums  TRAI=%-5d  PHAI=%-5d\n", elapsed, l, r);
            lastLog = elapsed;
        }
        if (Serial.available() && Serial.read() == 's') {
            stopAll();
            return false;  // bị ngắt
        }
        delay(20);
    }
    return true;
}

// ── Ramp 0 → 100% → 0 ───────────────────────────────────────────
static void runRamp(bool doLeft, bool doRight) {
    Serial.println("  Ramp: 0 -> 100% -> 0");
    for (int dir : {1, -1}) {
        for (int spd = (dir == 1 ? 0 : PWM_MAX);
             dir == 1 ? spd <= PWM_MAX : spd >= 0;
             spd += dir * (PWM_MAX / 20)) {
            int l = doLeft  ? spd : 0;
            int r = doRight ? spd : 0;
            pwm.write(l, r);
            tplotBoth(l, r);
            Serial.printf("  speed = %4d\n", spd);
            delay(RAMP_STEP);
            if (Serial.available() && Serial.read() == 's') { stopAll(); return; }
        }
    }
    stopAll(false);
}

// ── Bộ 4 kịch bản ───────────────────────────────────────────────
static void runSuite(bool doLeft, bool doRight) {
    int l70 = doLeft  ? SPEED_70 : 0;
    int r70 = doRight ? SPEED_70 : 0;
    int l30 = doLeft  ? SPEED_30 : 0;
    int r30 = doRight ? SPEED_30 : 0;

    header(1, "TIEN 70% MAX");
    if (!runFixed( l70,  r70, RUN_MS)) return;

    header(2, "LUI 70% MAX");
    if (!runFixed(-l70, -r70, RUN_MS)) return;

    header(3, "TIEN 30% MAX (kiem tra deadband)");
    if (SPEED_30 <= PWM_DEADBAND)
        Serial.println("  CANH BAO: SPEED_30 <= DEADBAND, motor co the khong quay!");
    if (!runFixed(l30, r30, RUN_MS)) return;

    header(4, "RAMP 0 -> 100% -> 0");
    runRamp(doLeft, doRight);

    stopAll();
    delay(PAUSE_MS);
    Serial.println("  == HOAN THANH ==");
}

// ── Menu ─────────────────────────────────────────────────────────
static void printMenu() {
    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║    TEST MOTOR CO BAN — MENU          ║");
    Serial.println("╠══════════════════════════════════════╣");
    Serial.println("║  l → Test motor TRAI (rieng le)      ║");
    Serial.println("║  r → Test motor PHAI (rieng le)      ║");
    Serial.println("║  b → Test CA HAI motor               ║");
    Serial.println("║  s → Dung khan cap                   ║");
    Serial.println("║  h → Hien menu nay                   ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.printf("  PWM_MAX=%d  DEADBAND=%d  FREQ=%dHz\n",
                  PWM_MAX, PWM_DEADBAND, PWM_FREQ_HZ);
    Serial.printf("  TRAI IN1=GPIO%d IN2=GPIO%d | PHAI IN1=GPIO%d IN2=GPIO%d\n",
                  PIN_MOTOR_L1, PIN_MOTOR_L2, PIN_MOTOR_R1, PIN_MOTOR_R2);
    Serial.println();
}

// ================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    pwm.begin();
    stopAll(false);
    printMenu();
}

void loop() {
    tplotBoth(0, 0);

    if (!Serial.available()) { delay(200); return; }

    char cmd = Serial.read();
    while (Serial.available()) Serial.read();  // flush \r\n

    switch (cmd) {
        case 'l': Serial.println("\n>> Test motor TRAI"); runSuite(true,  false); printMenu(); break;
        case 'r': Serial.println("\n>> Test motor PHAI"); runSuite(false, true);  printMenu(); break;
        case 'b': Serial.println("\n>> Test CA HAI");     runSuite(true,  true);  printMenu(); break;
        case 's': stopAll(); break;
        case 'h': printMenu(); break;
        default:  Serial.printf("  Lenh '%c' khong hop le. Goc 'h'.\n", cmd); break;
    }
}