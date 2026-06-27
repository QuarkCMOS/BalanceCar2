#pragma once
#include <cstdint>

// ── WiFi AP ───────────────────────────────────────────────────
// ESP32 tự phát AP, không cần router
constexpr const char* AP_SSID             = "BalanceCar";
constexpr const char* AP_PASSWORD         = "12345678";
// IP cố định: 192.168.4.1

// ── WebSocket ─────────────────────────────────────────────────
constexpr uint16_t  WS_PORT               = 81;
constexpr uint32_t  TELEMETRY_INTERVAL_MS = 50;   // 20 Hz — giảm từ 50Hz để giảm tải WiFi

// ── Motor / PWM ───────────────────────────────────────────────
constexpr int       PWM_FREQ_HZ           = 20000;
constexpr int       PWM_MAX               = 1023;

// Deadband compensation:
// Thực tế motor cần tối thiểu ~500/1023 mới bắt đầu quay.
// PWM_DEADBAND_INPUT: ngưỡng tín hiệu đầu vào — dưới mức này → brake (không quay)
// PWM_DEADBAND_OUTPUT: giá trị PWM thực tế gửi xuống motor khi bắt đầu chuyển động
// Áp dụng: output = PWM_DEADBAND_OUTPUT + (|input| / PWM_MAX) * (PWM_MAX - PWM_DEADBAND_OUTPUT)
constexpr int       PWM_DEADBAND_INPUT    = 20;   // < 20 → brake hẳn
constexpr int       PWM_DEADBAND_OUTPUT   = 500;  // Điểm bắt đầu quay thực tế

// ── Encoder GA25-370 ──────────────────────────────────────────
constexpr int       ENC_PPR               = 26;
constexpr float     GEAR_RATIO            = 18.8f;
constexpr float     WHEEL_DIAMETER_M      = 0.065f;
constexpr float     ENC_TICKS_PER_REV     = ENC_PPR * GEAR_RATIO * 4;

// ── PID ───────────────────────────────────────────────────────
constexpr float     PID_KP                = 200.0f;
constexpr float     PID_KI                = 0.0f;
constexpr float     PID_KD                = 0.1f;
constexpr float     PID_OUTPUT_MAX        = PWM_MAX;
constexpr float     PID_INTEGRAL_MAX      = 200.0f;

// ── Balance ───────────────────────────────────────────────────
constexpr float     TARGET_ANGLE          = -4.0f;
constexpr float     FALL_ANGLE            = 30.0f;
constexpr float     ALPHA                 = 0.98f;

// ── Position / Velocity loop (Tầng 1) ────────────────────────
// VEL_KP_DEFAULT:  damping vận tốc tức thời.
//   Khi xe đang trôi với v (m/s), cộng v*VEL_KP vào target angle để phanh lại.
//   Tăng nếu xe phanh chậm. Giảm nếu xe bị oscillate (lắc qua lại).
//   Khuyến nghị bắt đầu: 0.3 ~ 0.8
constexpr float     VEL_KP_DEFAULT        = 0.5f;

// POS_KI_DEFAULT:  tích phân vị trí (kéo xe về chỗ ban đầu sau khi bị đẩy).
//   Mỗi tick encoder lệch khỏi vị trí ref → tích lũy correction góc.
//   Tăng nếu xe vẫn trôi sau khi dừng. Giảm nếu xe rung/dao động chậm.
//   Khuyến nghị bắt đầu: 0.002 ~ 0.01
constexpr float     POS_KI_DEFAULT        = 0.005f;

// VEL_CORRECTION_MAX: giới hạn angle correction từ tầng 1 (độ).
//   Cần nhỏ hơn FALL_ANGLE nhiều, không override tầng 2 quá mạnh.
//   Khuyến nghị bắt đầu: 3.0 ~ 8.0
constexpr float     VEL_CORRECTION_MAX    = 5.0f;

// ── Timing ────────────────────────────────────────────────────
constexpr uint32_t  LOOP_INTERVAL_US      = 5000;  // 200 Hz control loop

// ── WebSocket TX guard ────────────────────────────────────────
// Timeout tối đa cho một lần gửi telemetry (µs).
// Nếu vượt quá → bỏ frame, không block control loop.
constexpr uint32_t  WS_SEND_TIMEOUT_US    = 2000;  // 2 ms
