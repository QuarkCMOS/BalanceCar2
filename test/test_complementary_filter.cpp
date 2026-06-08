// ================================================================
// test_complementary_filter.cpp — Unit tests cho ComplementaryFilter
//
// Framework: Unity (tích hợp sẵn trong PlatformIO)
// Chạy trên native host (không cần phần cứng thật):
//   pio test -e native -f test_complementary_filter
//
// Cấu trúc test:
//   TC-CF-01..05  — khởi tạo & cấu hình
//   TC-CF-06..10  — hành vi update cơ bản
//   TC-CF-11..14  — edge cases & robustness
//   TC-CF-15..17  — integration (mô phỏng nhiều bước)
// ================================================================

#include <unity.h>
#include <cmath>

// ── Stub Arduino (native host không có Arduino.h) ────────────────
#ifndef ARDUINO
    #ifndef M_PI
        #define M_PI 3.14159265358979323846
    #endif
#endif

// Include source thật (cùng pattern với test_control.cpp)
#include "../src/filters/complementary_filter.cpp"

// ────────────────────────────────────────────────────────────────
// Hằng số helper
// ────────────────────────────────────────────────────────────────
static constexpr float DT      = 0.005f;   // 5ms — khớp LOOP_INTERVAL_US
static constexpr float ALPHA   = 0.98f;    // hệ số mặc định
static constexpr float EPS     = 0.01f;    // dung sai chung (độ)
static constexpr float EPS_DEG = 0.1f;     // dung sai rộng hơn cho tích lũy

// Helper: tính góc accelerometer từ trục X, Z (độ)
static float accelAngle(float ax, float az) {
    return static_cast<float>(atan2(ax, az) * 180.0 / M_PI);
}

// ================================================================
// TC-CF-01: Khởi tạo mặc định — alpha = 0.98, chưa initialized
// ================================================================
void test_cf_default_alpha() {
    ComplementaryFilter cf;
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.98f, cf.getAlpha());
    TEST_ASSERT_FALSE(cf.isInitialized());
}

// ================================================================
// TC-CF-02: Khởi tạo với alpha tùy chỉnh
// ================================================================
void test_cf_custom_alpha() {
    ComplementaryFilter cf(0.95f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.95f, cf.getAlpha());
}

// ================================================================
// TC-CF-03: setAlpha() cập nhật đúng
// ================================================================
void test_cf_set_alpha() {
    ComplementaryFilter cf;
    cf.setAlpha(0.90f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.90f, cf.getAlpha());
}

// ================================================================
// TC-CF-04: alpha bị clamp vào [0, 1]
// ================================================================
void test_cf_alpha_clamp() {
    ComplementaryFilter cf;

    cf.setAlpha(1.5f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, cf.getAlpha());

    cf.setAlpha(-0.5f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, cf.getAlpha());
}

// ================================================================
// TC-CF-05: reset() đặt góc về giá trị chỉ định và xóa initialized
// ================================================================
void test_cf_reset() {
    ComplementaryFilter cf;

    // Seed trước
    cf.update(0.0f, 1.0f, 0.0f, DT);
    TEST_ASSERT_TRUE(cf.isInitialized());

    cf.reset(15.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 15.0f, cf.getAngle());
    TEST_ASSERT_FALSE(cf.isInitialized());
}

// ================================================================
// TC-CF-06: Lần update đầu tiên = 100% accel (seed, không drift)
// ================================================================
void test_cf_first_update_uses_accel_only() {
    ComplementaryFilter cf;

    // accelX=0, accelZ=1 → accelAngle = 0°
    float angle = cf.update(0.0f, 1.0f, 999.0f, DT);  // gyro lớn, nhưng bị bỏ qua lần đầu

    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, angle);
    TEST_ASSERT_TRUE(cf.isInitialized());
}

// ================================================================
// TC-CF-07: Xe thẳng đứng hoàn toàn, gyro = 0 → angle ổn định = 0
// ================================================================
void test_cf_upright_stable() {
    ComplementaryFilter cf;

    // Chạy 100 bước: accel cho biết 0°, gyro = 0
    float angle = 0.0f;
    for (int i = 0; i < 100; i++) {
        angle = cf.update(0.0f, 1.0f, 0.0f, DT);
    }

    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, angle);
}

// ================================================================
// TC-CF-08: accel cho biết 10° tĩnh, gyro = 0 → hội tụ về 10°
// ================================================================
void test_cf_accel_angle_converges() {
    ComplementaryFilter cf;

    // accelX = sin(10°), accelZ = cos(10°) → accelAngle = 10°
    float rad = 10.0f * static_cast<float>(M_PI) / 180.0f;
    float ax  = sinf(rad);
    float az  = cosf(rad);

    // Chạy 500 bước với gyro = 0
    // Sau nhiều bước: angle → accelAngle = 10° (drift accel chiếm ưu thế)
    float angle = 0.0f;
    for (int i = 0; i < 500; i++) {
        angle = cf.update(ax, az, 0.0f, DT);
    }

    // Sau 500*DT = 2.5s, với alpha=0.98:
    // angle = 0.98^500 * seed + (1-0.98^500) * 10° ≈ 10°
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, angle);
}

// ================================================================
// TC-CF-09: gyro tích phân chính xác khi accel = vị trí giả định
// ================================================================
void test_cf_gyro_integration_short_term() {
    // Alpha = 1.0 → 100% tin gyro, accel bị bỏ qua hoàn toàn
    ComplementaryFilter cf(1.0f);

    // Seed về 0
    cf.update(0.0f, 1.0f, 0.0f, DT);

    // gyro = 90 độ/s trong 0.1s → angle nên tăng 9°
    float angle = 0.0f;
    int   steps = static_cast<int>(0.1f / DT);  // = 20 bước
    for (int i = 0; i < steps; i++) {
        angle = cf.update(0.0f, 1.0f, 90.0f, DT);
    }

    // 90 deg/s * 20 * 0.005s = 9.0°
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 9.0f, angle);
}

// ================================================================
// TC-CF-10: updateWithAngle() — nhận accelAngle trực tiếp
// ================================================================
void test_cf_update_with_angle_direct() {
    ComplementaryFilter cf;

    // Seed bằng accelAngle = 5°
    cf.updateWithAngle(5.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 5.0f, cf.getAngle());

    // Lần 2: accelAngle = 5°, gyro = 0 → angle ≈ 5°
    float angle = cf.updateWithAngle(5.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 5.0f, angle);
}

// ================================================================
// TC-CF-11: dt <= 0 → trả góc cũ, không crash
// ================================================================
void test_cf_invalid_dt_returns_last_angle() {
    ComplementaryFilter cf;
    cf.update(0.0f, 1.0f, 0.0f, DT);   // seed

    float angleBefore = cf.getAngle();

    float a1 = cf.update(0.0f, 1.0f, 100.0f, 0.0f);   // dt = 0
    float a2 = cf.update(0.0f, 1.0f, 100.0f, -0.01f);  // dt âm

    TEST_ASSERT_FLOAT_WITHIN(EPS, angleBefore, a1);
    TEST_ASSERT_FLOAT_WITHIN(EPS, angleBefore, a2);
}

// ================================================================
// TC-CF-12: getAngle() khớp với giá trị trả về của update()
// ================================================================
void test_cf_get_angle_consistent_with_return() {
    ComplementaryFilter cf;

    float returned = cf.update(0.1f, 0.99f, 5.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(EPS, returned, cf.getAngle());

    returned = cf.update(0.1f, 0.99f, 5.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(EPS, returned, cf.getAngle());
}

// ================================================================
// TC-CF-13: reset(0) sau khi đã tích lũy → angle trở về 0
// ================================================================
void test_cf_reset_clears_accumulated_angle() {
    ComplementaryFilter cf(1.0f);   // 100% gyro để angle dễ thay đổi

    cf.update(0.0f, 1.0f, 0.0f, DT);  // seed
    for (int i = 0; i < 50; i++) {
        cf.update(0.0f, 1.0f, 30.0f, DT);   // tích lũy ~7.5°
    }
    TEST_ASSERT_TRUE(cf.getAngle() > 1.0f);

    cf.reset(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, cf.getAngle());
    TEST_ASSERT_FALSE(cf.isInitialized());

    // Update đầu tiên sau reset: seed = accelAngle = 0°
    cf.update(0.0f, 1.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, cf.getAngle());
}

// ================================================================
// TC-CF-14: Alpha = 0 → angle = accelAngle mỗi bước (100% accel)
// ================================================================
void test_cf_alpha_zero_full_accel() {
    ComplementaryFilter cf(0.0f);

    float rad = 20.0f * static_cast<float>(M_PI) / 180.0f;
    float ax  = sinf(rad);
    float az  = cosf(rad);

    cf.update(ax, az, 0.0f, DT);   // seed

    // Với alpha=0: angle = 0*gyroAngle + 1*accelAngle = accelAngle = 20°
    float angle = cf.update(ax, az, 9999.0f, DT);   // gyro lớn nhưng bị bỏ qua
    TEST_ASSERT_FLOAT_WITHIN(EPS, 20.0f, angle);
}

// ================================================================
// TC-CF-15 (Integration): Mô phỏng cân bằng 200 bước — angle hội tụ về 0
// ================================================================
void test_cf_integration_balance_simulation() {
    // alpha = 0.98, seed = 5° nghiêng
    ComplementaryFilter cf(0.98f);

    // Seed ban đầu: xe nghiêng 5°
    float rad0 = 5.0f * static_cast<float>(M_PI) / 180.0f;
    cf.update(sinf(rad0), cosf(rad0), 0.0f, DT);

    float angle    = 5.0f;
    float gyroRate = 0.0f;

    // Mô phỏng: gyro báo tốc độ dựng lại (-angle * Kp_gyro),
    // accel dần trở về 0° khi xe thẳng
    for (int i = 0; i < 200; i++) {
        // Mô hình đơn giản: gyro đẩy về 0 với gain = 3
        gyroRate = -angle * 3.0f;

        // accelAngle ≈ angle hiện tại (có nhiễu nhỏ)
        float accelNoise = (i % 7 == 0) ? 0.2f : 0.0f;
        float accelAng   = angle + accelNoise;

        angle = cf.updateWithAngle(accelAng, gyroRate, DT);
    }

    // Sau 1s mô phỏng, góc phải về gần 0
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, angle);
}

// ================================================================
// TC-CF-16 (Integration): Nhiễu accel đột ngột — gyro giữ ổn định
// ================================================================
void test_cf_integration_accel_noise_rejection() {
    // alpha = 0.98 → gyro chiếm ưu thế trong ngắn hạn
    ComplementaryFilter cf(0.98f);

    // Seed: xe thẳng đứng
    cf.update(0.0f, 1.0f, 0.0f, DT);

    // Chạy ổn định 50 bước
    for (int i = 0; i < 50; i++) {
        cf.update(0.0f, 1.0f, 0.0f, DT);
    }
    float stableAngle = cf.getAngle();

    // Bước nhiễu: accel báo 45° (shock/vibration), gyro = 0
    float angle = cf.update(sinf(45.0f * static_cast<float>(M_PI) / 180.0f),
                             cosf(45.0f * static_cast<float>(M_PI) / 180.0f),
                             0.0f, DT);

    // Với alpha=0.98: angle chỉ bị lệch 2% của 45° ≈ 0.9° so với stable
    TEST_ASSERT_FLOAT_WITHIN(1.0f, stableAngle, angle);
}

// ================================================================
// TC-CF-17 (Integration): Gyro drift dài hạn bị accel hiệu chỉnh
// ================================================================
void test_cf_integration_gyro_drift_corrected() {
    // Dùng alpha nhỏ hơn để thấy hiệu chỉnh rõ hơn trong test thời gian ngắn
    ComplementaryFilter cf(0.95f);

    cf.update(0.0f, 1.0f, 0.0f, DT);   // seed: thẳng đứng

    // Giả lập gyro drift: báo 0.5 deg/s liên tục mặc dù xe thật = 0°
    // accel vẫn báo đúng 0°
    float angle = 0.0f;
    for (int i = 0; i < 400; i++) {
        angle = cf.update(0.0f, 1.0f, 0.5f, DT);  // gyro drift 0.5 deg/s
    }

    // Nếu không có accel: angle = 0.5 * 400 * 0.005 = 1.0°
    // Với complementary filter alpha=0.95: accel kéo angle về 0°
    // Sau 2s, angle phải < 0.5° (accel đã hiệu chỉnh phần lớn drift)
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, angle);
}

// ================================================================
// Setup & Loop (PlatformIO Unity runner)
// ================================================================

void setup() {
    UNITY_BEGIN();

    // Khởi tạo & cấu hình
    RUN_TEST(test_cf_default_alpha);
    RUN_TEST(test_cf_custom_alpha);
    RUN_TEST(test_cf_set_alpha);
    RUN_TEST(test_cf_alpha_clamp);
    RUN_TEST(test_cf_reset);

    // Hành vi update cơ bản
    RUN_TEST(test_cf_first_update_uses_accel_only);
    RUN_TEST(test_cf_upright_stable);
    RUN_TEST(test_cf_accel_angle_converges);
    RUN_TEST(test_cf_gyro_integration_short_term);
    RUN_TEST(test_cf_update_with_angle_direct);

    // Edge cases & robustness
    RUN_TEST(test_cf_invalid_dt_returns_last_angle);
    RUN_TEST(test_cf_get_angle_consistent_with_return);
    RUN_TEST(test_cf_reset_clears_accumulated_angle);
    RUN_TEST(test_cf_alpha_zero_full_accel);

    // Integration
    RUN_TEST(test_cf_integration_balance_simulation);
    RUN_TEST(test_cf_integration_accel_noise_rejection);
    RUN_TEST(test_cf_integration_gyro_drift_corrected);

    UNITY_END();
}

void loop() {}