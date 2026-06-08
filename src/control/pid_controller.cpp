// ================================================================
// pid_controller.cpp
// ================================================================

#include "pid_controller.h"

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float minOut, float maxOut) {
    if (minOut >= maxOut) return;
    _minOut = minOut;
    _maxOut = maxOut;
}

void PIDController::setIntegralLimits(float minI, float maxI) {
    if (minI >= maxI) return;
    _minI = minI;
    _maxI = maxI;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return _lastOutput;

    float error = setpoint - measurement;

    // ── Proportional ─────────────────────────────────────────
    float pTerm = _kp * error;

    // ── Integral với anti-windup clamp ───────────────────────
    _integral += error * dt;
    _integral  = clamp(_integral, _minI, _maxI);
    float iTerm = _ki * _integral;

    // ── Derivative on Measurement (không phải on Error) ──────
    // FIX: dùng -Kd * d(measurement)/dt thay vì Kd * d(error)/dt
    // → tránh derivative kick khi setpoint thay đổi đột ngột từ web
    float dTerm = 0.0f;
    if (!_firstRun) {
        float dMeasure = (measurement - _prevMeasure) / dt;
        dTerm = -_kd * dMeasure;
    }
    _firstRun    = false;
    _prevMeasure = measurement;

    // ── Tổng hợp và clamp ────────────────────────────────────
    float output = clamp(pTerm + iTerm + dTerm, _minOut, _maxOut);
    _lastOutput  = output;
    return output;
}

void PIDController::reset() {
    _integral    = 0.0f;
    _prevMeasure = 0.0f;
    _lastOutput  = 0.0f;
    _firstRun    = true;
}

float PIDController::clamp(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}
