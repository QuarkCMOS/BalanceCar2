#pragma once
#include <Arduino.h>

// ================================================================
// EncoderManager — Quadrature Hall encoder, 2 bộ độc lập
//
// Mỗi instance quản lý 1 encoder (pinA + pinB).
// ISR được định nghĩa static bên ngoài class vì ESP32 yêu cầu
// hàm ISR phải ở global/static scope với IRAM_ATTR.
//
// Sử dụng:
//   EncoderManager encL(PIN_ENC_L_A, PIN_ENC_L_B, &g_ticksLeft);
//   EncoderManager encR(PIN_ENC_R_A, PIN_ENC_R_B, &g_ticksRight);
//   encL.begin(); encR.begin();
//   // trong loop:
//   encL.update(dt);
//   EncoderData d = encL.getData();
// ================================================================

struct EncoderData {
    int32_t ticks;      // tổng số tick kể từ reset (có dấu)
    int32_t deltaTicks; // tick thay đổi trong lần update gần nhất
    float   rpm;        // vòng/phút tại trục bánh xe (sau hộp số)
    float   velocityMs; // tốc độ dài (m/s)
};

class EncoderManager {
public:
    // pinA: chân RISING interrupt, pinB: xác định chiều quay
    // pTicks: trỏ tới biến volatile int32_t global để ISR ghi vào
    EncoderManager(int pinA, int pinB, volatile int32_t* pTicks);

    void begin();

    // Gọi mỗi control loop, truyền dt (giây) để tính RPM chính xác
    void update(float dt);

    EncoderData getData() const { return _data; }

    // Reset về 0 (ví dụ khi bắt đầu chạy mới)
    void reset();

private:
    int                  _pinA;
    int                  _pinB;
    volatile int32_t*    _pTicks;   // trỏ vào biến ISR global
    int32_t              _lastTicks;
    EncoderData          _data;
};