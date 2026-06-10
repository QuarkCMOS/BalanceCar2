#pragma once

#include <Arduino.h>
#include <ESP32_MCPWM.h>
#include "../config/constants.h"

// Hardware config cho một motor
struct MotorPinConfig {
    int                 pinIN1;   // LPWM
    int                 pinIN2;   // RPWM
    mcpwm_unit_t        unit;
    mcpwm_timer_t       timer;
    mcpwm_io_signals_t  sigA;
    mcpwm_io_signals_t  sigB;
};

class MotorDriver {
public:
    // reversed = true  → đảo chiều (dùng cho motor gắn đối xứng)
    explicit MotorDriver(const MotorPinConfig& cfg, bool reversed = false);

    void begin();

    // speed: -PWM_MAX → +PWM_MAX (giá trị PID thô)
    // Nội bộ áp dụng deadband compensation trước khi gửi xuống MCPWM
    void setSpeed(int speed);

    void hardBrake();
    void softBrake();
    void coast();

    int  getCurrentSpeed() const { return _currentSpeed; }
    bool isReversed()      const { return _reversed; }

    // Đảo chiều runtime — dùng để test không cần recompile
    void setReversed(bool r) { _reversed = r; }

private:
    MotorPinConfig _cfg;
    HBridgeMotor   _motor;
    int            _currentSpeed = 0;
    bool           _reversed;

    // Áp dụng deadband compensation:
    // Trả về 0 nếu |speed| < PWM_DEADBAND_INPUT
    // Ngược lại map [0..MAX] → [DEADBAND_OUTPUT..MAX]
    int _applyDeadband(int speed) const;
};
