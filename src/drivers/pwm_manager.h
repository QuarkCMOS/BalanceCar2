#pragma once

#include "motor_driver.h"
#include "../config/pins.h"
#include "../config/constants.h"

class PWMManager {
public:
    PWMManager();

    void begin();

    // speed: -PWM_MAX → +PWM_MAX cho từng motor
    void write(int leftSpeed, int rightSpeed);

    // Hard brake cả 2 motor — gọi khi |angle| > FALL_ANGLE
    void emergencyBrake();

    // Soft brake cả 2 motor — dừng bình thường
    void softBrake();

    int getLeftSpeed()  const { return _left.getCurrentSpeed(); }
    int getRightSpeed() const { return _right.getCurrentSpeed(); }

private:
    MotorDriver _left;
    MotorDriver _right;
};