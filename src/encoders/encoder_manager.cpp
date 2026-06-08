#include "encoder_manager.h"
#include "../config/pins.h"
#include "../config/constants.h"

// ================================================================
// Biến volatile toàn cục — ISR ghi, main-loop đọc
// Mỗi encoder cần 1 biến riêng và 1 hàm ISR riêng.
// ESP32 không cho phép ISR là lambda hay method của class.
// ================================================================

volatile int32_t g_ticksLeft  = 0;
volatile int32_t g_ticksRight = 0;

// ── ISR encoder trái ─────────────────────────────────────────────
// Kích hoạt trên cả RISING và FALLING của pinA (quadrature x2)
// pinB xác định chiều: HIGH khi RISING pinA → tiến, LOW → lùi
void IRAM_ATTR isrEncLeft() {
    // RISING pinA: B=HIGH → tiến (+), B=LOW → lùi (-)
    // FALLING pinA: B=LOW → tiến (+), B=HIGH → lùi (-)
    bool aRising = (digitalRead(PIN_ENC_L_A) == HIGH);
    bool bHigh   = (digitalRead(PIN_ENC_L_B) == HIGH);
    if (aRising == bHigh) g_ticksLeft++;
    else                  g_ticksLeft--;
}

// ── ISR encoder phải ─────────────────────────────────────────────
void IRAM_ATTR isrEncRight() {
    bool aRising = (digitalRead(PIN_ENC_R_A) == HIGH);
    bool bHigh   = (digitalRead(PIN_ENC_R_B) == HIGH);
    if (aRising == bHigh) g_ticksRight++;
    else                  g_ticksRight--;
}

// ================================================================
// EncoderManager implementation
// ================================================================

EncoderManager::EncoderManager(int pinA, int pinB, volatile int32_t* pTicks)
    : _pinA(pinA), _pinB(pinB), _pTicks(pTicks),
      _lastTicks(0), _data{0, 0, 0.0f, 0.0f}
{}

void EncoderManager::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // Attach ISR đúng với encoder nào
    // (phân biệt qua pinA)
    if (_pinA == PIN_ENC_L_A) {
        attachInterrupt(digitalPinToInterrupt(_pinA), isrEncLeft,  CHANGE);
    } else {
        attachInterrupt(digitalPinToInterrupt(_pinA), isrEncRight, CHANGE);
    }

    reset();
}

void EncoderManager::update(float dt) {
    // Đọc ticks thread-safe
    int32_t currentTicks;
    noInterrupts();
    currentTicks = *_pTicks;
    interrupts();

    int32_t delta = currentTicks - _lastTicks;
    _lastTicks    = currentTicks;

    _data.ticks      = currentTicks;
    _data.deltaTicks = delta;

    if (dt > 0.0001f) {
        // tick/s → rev/s tại trục bánh (sau hộp số, x2 vì CHANGE = x2 PPR)
        float revsPerSec = (float)delta / (ENC_TICKS_PER_REV * 0.5f) / dt;
        // (* 0.5f vì CHANGE mode đã nhân 2, ENC_TICKS_PER_REV có x4 quadrature)
        _data.rpm        = revsPerSec * 60.0f;
        _data.velocityMs = revsPerSec * (float)M_PI * WHEEL_DIAMETER_M;
    }
}

void EncoderManager::reset() {
    noInterrupts();
    *_pTicks = 0;
    interrupts();
    _lastTicks       = 0;
    _data            = {0, 0, 0.0f, 0.0f};
}