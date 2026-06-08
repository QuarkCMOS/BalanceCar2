// ================================================================
// test_motor_hardware.cpp — Test chạy động cơ thật trên DRV8833
//
// Upload env:  esp32dev_motor_hw
// Xem kết quả: Serial Monitor 115200 hoặc Teleplot
//
// Mỗi test case chạy ~10 giây, in header rõ ràng ra Serial.
// Giữa các test có 2s dừng hoàn toàn để dễ phân biệt.
//
// Danh sách test:
//   TC-HW-01  Deadband boundary       — tìm ngưỡng motor bắt đầu quay
//   TC-HW-02  Forward ramp            — tăng tốc 0 → MAX → 0
//   TC-HW-03  Reverse ramp            — tăng tốc 0 → -MAX → 0
//   TC-HW-04  Direction reversal      — đảo chiều qua deadband
//   TC-HW-05  Asymmetry check         — so sánh L vs R cùng setpoint
//   TC-HW-06  Hard brake response     — chạy full rồi hardBrake()
//   TC-HW-07  Soft brake response     — chạy full rồi softBrake()
//   TC-HW-08  Coast response          — chạy full rồi coast()
//   TC-HW-09  Step response           — bật/tắt đột ngột nhiều lần
//   TC-HW-10  Independent L/R control — mỗi motor tốc độ khác nhau
// ================================================================

#include <Arduino.h>
#include "drivers/pwm_manager.h"

// ────────────────────────────────────────────────────────────────
// Teleplot helpers
// ────────────────────────────────────────────────────────────────

static void tplot(const char* name, float val) {
    Serial.printf(">%s:%.1f\n", name, val);
}

static void tplotMotors(int l, int r) {
    tplot("L_speed",   (float)l);
    tplot("R_speed",   (float)r);
    tplot("deadband+", (float) PWM_DEADBAND);
    tplot("deadband-", (float)-PWM_DEADBAND);
}

// ────────────────────────────────────────────────────────────────
// Helpers
// ────────────────────────────────────────────────────────────────

PWMManager pwm;

// In header nổi bật, dừng motor trước và sau test
static void testHeader(int num, const char* name, const char* desc) {
    pwm.emergencyBrake();
    tplotMotors(0, 0);
    delay(2000);    // 2s dừng hoàn toàn giữa các test

    Serial.println();
    Serial.println("========================================");
    Serial.printf( " TC-HW-%02d: %s\n", num, name);
    Serial.printf( " %s\n", desc);
    Serial.printf( " Thoi gian: ~10 giay\n");
    Serial.println("========================================");
    delay(500);
}

// Ramp tuyến tính từ (lStart,rStart) → (lEnd,rEnd) trong duration ms
static void ramp(int lStart, int lEnd, int rStart, int rEnd,
                 uint32_t durationMs, uint32_t stepMs = 20) {
    uint32_t steps = durationMs / stepMs;
    if (steps == 0) steps = 1;
    for (uint32_t i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        int   l = (int)(lStart + t * (lEnd - lStart));
        int   r = (int)(rStart + t * (rEnd - rStart));
        pwm.write(l, r);
        tplotMotors(l, r);
        delay(stepMs);
    }
}

// Giữ tốc độ cố định trong duration ms, in log mỗi 200ms
static void hold(int l, int r, uint32_t durationMs) {
    pwm.write(l, r);
    uint32_t t0   = millis();
    uint32_t last = 0;
    while (millis() - t0 < durationMs) {
        uint32_t now = millis() - t0;
        tplotMotors(l, r);
        if (now - last >= 200) {
            Serial.printf("  t=%4ums  L=%-5d  R=%-5d\n", now, l, r);
            last = now;
        }
        delay(20);
    }
}

// ────────────────────────────────────────────────────────────────
// TC-HW-01: Deadband boundary
// Tăng từ 0 lên PWM_DEADBAND+30 theo từng bước 1, log từng bước.
// Quan sát: motor bắt đầu quay ở giá trị nào?
// Expected: không quay ở < PWM_DEADBAND, bắt đầu quay ≥ PWM_DEADBAND
// ────────────────────────────────────────────────────────────────
void tc01_deadband_boundary() {
    testHeader(1, "DEADBAND BOUNDARY",
               "Tang tu 0 len DEADBAND+30 theo buoc 1. Xem motor bat dau quay o dau.");

    int limit = PWM_DEADBAND + 30;
    uint32_t stepMs = 10000 / (limit * 2 + 10); // chia đều 10s
    if (stepMs < 30) stepMs = 30;

    // Tăng dần
    for (int s = 0; s <= limit; s++) {
        pwm.write(s, s);
        tplotMotors(s, s);
        if (s % 5 == 0) {
            Serial.printf("  setpoint=%-4d  (DEADBAND=%d)\n", s, PWM_DEADBAND);
        }
        delay(stepMs);
    }
    // Giảm dần về 0
    for (int s = limit; s >= 0; s--) {
        pwm.write(s, s);
        tplotMotors(s, s);
        delay(stepMs);
    }
}

// ────────────────────────────────────────────────────────────────
// TC-HW-02: Forward ramp
// 0 → PWM_MAX (3s), giữ MAX (2s), MAX → 0 (3s), dừng (2s)
// Expected: cả 2 motor quay cùng chiều, tăng đều, không giật
// ────────────────────────────────────────────────────────────────
void tc02_forward_ramp() {
    testHeader(2, "FORWARD RAMP",
               "0 -> MAX (3s) -> giu MAX (2s) -> 0 (3s). Ca 2 motor cung chieu tien.");

    Serial.println("  [RAMP UP]");
    ramp(0, PWM_MAX, 0, PWM_MAX, 3000);

    Serial.println("  [HOLD MAX]");
    hold(PWM_MAX, PWM_MAX, 2000);

    Serial.println("  [RAMP DOWN]");
    ramp(PWM_MAX, 0, PWM_MAX, 0, 3000);

    Serial.println("  [STOP]");
    pwm.emergencyBrake();
    tplotMotors(0, 0);
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-03: Reverse ramp
// 0 → -PWM_MAX (3s), giữ (2s), -PWM_MAX → 0 (3s), dừng (2s)
// Expected: cả 2 motor quay chiều ngược lại
// ────────────────────────────────────────────────────────────────
void tc03_reverse_ramp() {
    testHeader(3, "REVERSE RAMP",
               "0 -> -MAX (3s) -> giu -MAX (2s) -> 0 (3s). Ca 2 motor lui.");

    Serial.println("  [RAMP DOWN]");
    ramp(0, -PWM_MAX, 0, -PWM_MAX, 3000);

    Serial.println("  [HOLD -MAX]");
    hold(-PWM_MAX, -PWM_MAX, 2000);

    Serial.println("  [RAMP UP]");
    ramp(-PWM_MAX, 0, -PWM_MAX, 0, 3000);

    Serial.println("  [STOP]");
    pwm.emergencyBrake();
    tplotMotors(0, 0);
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-04: Direction reversal
// Chuyển từ +MAX sang -MAX liên tục 4 lần, mỗi chiều 2s
// Expected: không có chết motor, đảo chiều mượt qua deadband
// ────────────────────────────────────────────────────────────────
void tc04_direction_reversal() {
    testHeader(4, "DIRECTION REVERSAL",
               "+MAX <-> -MAX moi 2 giay, 4 lan. Kiem tra dao chieu qua deadband.");

    int speeds[] = {PWM_MAX, -PWM_MAX, PWM_MAX, -PWM_MAX};
    const char* labels[] = {"TIEN", "LUI ", "TIEN", "LUI "};

    for (int i = 0; i < 4; i++) {
        int target = speeds[i];
        Serial.printf("  [%s] target=%d\n", labels[i], target);

        // Ramp nhanh 300ms thay vì cut đột ngột để tránh current spike
        int cur = pwm.getLeftSpeed();
        ramp(cur, target, cur, target, 300, 20);
        hold(target, target, 1700);
    }

    ramp(speeds[3], 0, speeds[3], 0, 500);
    pwm.emergencyBrake();
    tplotMotors(0, 0);
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-05: Asymmetry check
// Chạy L và R cùng setpoint, log riêng từng motor để so sánh
// Thay đổi 4 mức: 25%, 50%, 75%, 100% MAX — mỗi mức 2s
// Expected: 2 motor quay cùng tốc độ (xe đi thẳng)
// ────────────────────────────────────────────────────────────────
void tc05_asymmetry_check() {
    testHeader(5, "ASYMMETRY CHECK",
               "L = R o 4 muc 25/50/75/100%% MAX. Xe phai di thang.");

    int levels[] = {PWM_MAX/4, PWM_MAX/2, PWM_MAX*3/4, PWM_MAX};
    const char* pcts[] = {"25%", "50%", "75%", "100%"};

    for (int i = 0; i < 4; i++) {
        int s = levels[i];
        Serial.printf("  [%s = %d] L=%d R=%d — check xe di thang\n",
                      pcts[i], s, s, s);
        hold(s, s, 2000);
    }

    ramp(PWM_MAX, 0, PWM_MAX, 0, 500);
    pwm.emergencyBrake();
    tplotMotors(0, 0);
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-06: Hard brake response
// Chạy 75% MAX → hardBrake() đột ngột → quan sát thời gian dừng
// Lặp 3 lần
// Expected: dừng rất nhanh (< 0.5s), không trượt
// ────────────────────────────────────────────────────────────────
void tc06_hard_brake() {
    testHeader(6, "HARD BRAKE RESPONSE",
               "75%% MAX -> hardBrake() dot ngot. Lap 3 lan. Dong ho thoi gian dung.");

    int runSpeed = PWM_MAX * 3 / 4;

    for (int i = 1; i <= 3; i++) {
        Serial.printf("  [RUN %d] Chay %d...\n", i, runSpeed);
        ramp(0, runSpeed, 0, runSpeed, 500);
        hold(runSpeed, runSpeed, 1500);

        Serial.printf("  [BRAKE %d] hardBrake!\n", i);
        uint32_t t0 = millis();
        pwm.emergencyBrake();
        tplotMotors(0, 0);
        Serial.printf("  [BRAKE %d] Lenh brake gui trong %ums\n", i, millis()-t0);

        delay(800);     // quan sát bánh có thật sự dừng không
        tplotMotors(0, 0);
    }
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-07: Soft brake response
// Chạy 75% MAX → softBrake() → quan sát xe giảm tốc mượt
// Lặp 3 lần
// Expected: dừng êm hơn hard brake, motor không giật
// ────────────────────────────────────────────────────────────────
void tc07_soft_brake() {
    testHeader(7, "SOFT BRAKE RESPONSE",
               "75%% MAX -> softBrake(). Lap 3 lan. Dung em hon hard brake.");

    int runSpeed = PWM_MAX * 3 / 4;

    for (int i = 1; i <= 3; i++) {
        Serial.printf("  [RUN %d] Chay %d...\n", i, runSpeed);
        ramp(0, runSpeed, 0, runSpeed, 500);
        hold(runSpeed, runSpeed, 1500);

        Serial.printf("  [SOFT BRAKE %d]\n", i);
        pwm.softBrake();
        tplotMotors(0, 0);

        delay(800);
        tplotMotors(0, 0);
    }
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-08: Coast response
// Chạy 75% MAX → coast() → bánh tự do xoay theo quán tính
// Lặp 3 lần
// Expected: xe trượt thêm 1 đoạn sau khi coast (khác hard brake)
// ────────────────────────────────────────────────────────────────
void tc08_coast() {
    testHeader(8, "COAST RESPONSE",
               "75%% MAX -> coast(). Motor tha tu do. So sanh voi hard/soft brake.");

    int runSpeed = PWM_MAX * 3 / 4;

    for (int i = 1; i <= 3; i++) {
        Serial.printf("  [RUN %d] Chay %d...\n", i, runSpeed);
        ramp(0, runSpeed, 0, runSpeed, 500);
        hold(runSpeed, runSpeed, 1500);

        Serial.printf("  [COAST %d]\n", i);
        pwm.getLeftSpeed();     // flush getter
        pwm.write(0, 0);        // coast qua MotorDriver (deadband → hardBrake)
        // Gọi coast() trực tiếp qua PWMManager không có public API,
        // nên test ở mức write(0,0) — trong deadband → hardBrake
        // Nếu muốn coast thật, cần thêm PWMManager::coast() — ghi chú bên dưới
        tplotMotors(0, 0);
        Serial.println("  [NOTE] write(0,0) trong deadband -> hardBrake()");
        Serial.println("         De test coast that: them PWMManager::coast() vao pwm_manager.h");

        delay(800);
        tplotMotors(0, 0);
    }
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-09: Step response
// Bật/tắt đột ngột giữa 0 và 60% MAX, 10 lần, mỗi nhịp 0.5s
// Total ~10s
// Expected: motor phản hồi nhanh, không bỏ lỡ lệnh nào
// ────────────────────────────────────────────────────────────────
void tc09_step_response() {
    testHeader(9, "STEP RESPONSE",
               "Bat/tat dot ngot 0 <-> 60%% MAX, 10 lan x 0.5s. Kiem tra do tre.");

    int onSpeed = PWM_MAX * 60 / 100;
    Serial.printf("  ON speed = %d (60%% of %d)\n", onSpeed, PWM_MAX);

    for (int i = 0; i < 10; i++) {
        // ON
        pwm.write(onSpeed, onSpeed);
        tplotMotors(onSpeed, onSpeed);
        Serial.printf("  [STEP %d] ON  = %d\n", i+1, onSpeed);
        delay(500);

        // OFF (hardBrake trong deadband)
        pwm.emergencyBrake();
        tplotMotors(0, 0);
        Serial.printf("  [STEP %d] OFF = 0\n", i+1);
        delay(500);
    }
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// TC-HW-10: Independent L/R control
// Kiểm tra 2 motor hoàn toàn độc lập:
//   Phase 1 (2.5s): L tiến, R dừng
//   Phase 2 (2.5s): L dừng, R tiến
//   Phase 3 (2.5s): L tiến, R lùi  (quay tại chỗ)
//   Phase 4 (2.5s): L lùi,  R tiến (quay ngược)
// Expected: mỗi motor hoạt động độc lập, không ảnh hưởng nhau
// ────────────────────────────────────────────────────────────────
void tc10_independent_lr() {
    testHeader(10, "INDEPENDENT L/R CONTROL",
               "4 phase: L-only, R-only, spin-CW, spin-CCW. Moi phase 2.5s.");

    int spd = PWM_MAX * 60 / 100;

    Serial.printf("  [PHASE 1] L=%d R=0 (quay trai)\n", spd);
    hold(spd, 0, 2500);

    Serial.printf("  [PHASE 2] L=0 R=%d (quay phai)\n", spd);
    hold(0, spd, 2500);

    Serial.printf("  [PHASE 3] L=%d R=-%d (spin CW)\n", spd, spd);
    hold(spd, -spd, 2500);

    Serial.printf("  [PHASE 4] L=-%d R=%d (spin CCW)\n", spd, spd);
    hold(-spd, spd, 2500);

    ramp(pwm.getLeftSpeed(), 0, pwm.getRightSpeed(), 0, 500);
    pwm.emergencyBrake();
    tplotMotors(0, 0);
    delay(2000);
}

// ────────────────────────────────────────────────────────────────
// MAIN
// ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(3000);    // chờ Serial Monitor kết nối

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  BALANCE BOT — MOTOR HARDWARE TEST   ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.printf("  PWM_MAX    = %d\n", PWM_MAX);
    Serial.printf("  DEADBAND   = %d\n", PWM_DEADBAND);
    Serial.printf("  FREQ       = %d Hz\n", PWM_FREQ_HZ);
    Serial.println();
    Serial.println("  Pins:");
    Serial.printf("    Motor L : IN1=GPIO%d  IN2=GPIO%d\n", PIN_MOTOR_L1, PIN_MOTOR_L2);
    Serial.printf("    Motor R : IN1=GPIO%d  IN2=GPIO%d\n", PIN_MOTOR_R1, PIN_MOTOR_R2);
    Serial.println();
    Serial.println("  Bat dau sau 3 giay...");
    delay(3000);

    pwm.begin();
    delay(200);

    // ── Chạy tất cả test cases ──────────────────────────────────
    tc01_deadband_boundary();
    tc02_forward_ramp();
    tc03_reverse_ramp();
    tc04_direction_reversal();
    tc05_asymmetry_check();
    tc06_hard_brake();
    tc07_soft_brake();
    tc08_coast();
    tc09_step_response();
    tc10_independent_lr();

    // ── Done ────────────────────────────────────────────────────
    pwm.emergencyBrake();
    tplotMotors(0, 0);

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  TAT CA TEST HOAN THANH              ║");
    Serial.println("║  Kiem tra Serial Monitor va Teleplot  ║");
    Serial.println("╚══════════════════════════════════════╝");
}

void loop() {
    // Giữ Teleplot connected, plot flatline
    tplotMotors(0, 0);
    delay(500);
}