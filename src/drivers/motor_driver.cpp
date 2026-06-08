#include "motor_driver.h"

MotorDriver::MotorDriver(const MotorPinConfig& cfg, bool reversed)
    : _cfg(cfg), _reversed(reversed) {}

void MotorDriver::begin() {
    MotorMCPWMConfig hw{
        _cfg.pinIN1,
        _cfg.pinIN2,
        -1,
        _cfg.unit,
        _cfg.timer,
        _cfg.sigA,
        _cfg.sigB
    };
    hw.pwm_freq_hz  = PWM_FREQ_HZ;
    hw.input_max    = PWM_MAX;
    hw.counter      = MCPWM_UP_DOWN_COUNTER;
    hw.use_deadtime = false;

    MotorBehaviorConfig beh{
        FreewheelMode::HiZ_Awake,
        60,
        0,
        600,
        false
    };

    _motor.setup(hw, beh);
    _motor.start();

    // Đảm bảo motor ở trạng thái dừng ngay sau khi init
    hardBrake();
}

void MotorDriver::setSpeed(int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    if (abs(speed) < PWM_DEADBAND) {
        hardBrake();
        return;
    }

    int effective = _reversed ? -speed : speed;
    _currentSpeed = speed;

    if (effective > 0) {
        _motor.setSpeed(effective, Dir::CW);
    } else {
        _motor.setSpeed(-effective, Dir::CCW);
    }
}

void MotorDriver::hardBrake() {
    _currentSpeed = 0;

    // Chỉ dùng thư viện — KHÔNG gọi raw IDF mcpwm_set_duty_type().
    // mcpwm_set_duty_type() reset OPR_A/B về DUTY_MODE_0, làm hỏng
    // duty mode mà thư viện đã set cho Dir::CCW (dùng DUTY_MODE_1),
    // khiến motor không quay được một chiều sau khi brake.
    _motor.setHardBrake();
}

void MotorDriver::softBrake() {
    _currentSpeed = 0;
    // FIX LỖI 3: dùng setHardBrake() để stop hẳn, sau đó mới
    // chuyển sang freewheel — đây là cách dừng "mềm" đơn giản và đáng tin cậy.
    // DitherBrake API của thư viện này không hoạt động đúng với setSpeed(0).
    _motor.setHardBrake();
    delay(50);
    _motor.setFreewheel();
}

void MotorDriver::coast() {
    _currentSpeed = 0;
    _motor.setFreewheel();
}