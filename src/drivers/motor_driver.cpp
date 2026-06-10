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

// Áp dụng deadband compensation:
//   - |speed| < PWM_DEADBAND_INPUT → brake hẳn, không gửi xung
//   - Ngược lại → map tuyến tính từ [1..PWM_MAX] sang [PWM_DEADBAND_OUTPUT..PWM_MAX]
//     để motor luôn nhận đủ điện thế khi bắt đầu quay
//
// Ví dụ với DEADBAND_INPUT=20, DEADBAND_OUTPUT=500, MAX=1023:
//   speed=20   → effective_out=500
//   speed=511  → effective_out=500 + (511/1023)*(1023-500) ≈ 761
//   speed=1023 → effective_out=1023
int MotorDriver::_applyDeadband(int speed) const {
    if (abs(speed) < PWM_DEADBAND_INPUT) return 0;

    // Map: [PWM_DEADBAND_INPUT .. PWM_MAX] → [PWM_DEADBAND_OUTPUT .. PWM_MAX]
    int sign = (speed > 0) ? 1 : -1;
    int mag  = abs(speed);

    // Tỉ lệ trong dải có hiệu lực (0.0 → 1.0)
    float ratio = (float)(mag) / (float)(PWM_MAX);

    // Effective output: bắt đầu từ PWM_DEADBAND_OUTPUT, scale theo ratio
    int out = (int)(PWM_DEADBAND_OUTPUT
                  + ratio * (float)(PWM_MAX - PWM_DEADBAND_OUTPUT));

    if (out > PWM_MAX) out = PWM_MAX;
    return sign * out;
}

void MotorDriver::setSpeed(int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    // Áp dụng deadband — nếu quá nhỏ thì brake
    int compensated = _applyDeadband(speed);
    if (compensated == 0) {
        hardBrake();
        return;
    }

    int effective = _reversed ? -compensated : compensated;
    _currentSpeed = speed;   // lưu speed gốc (trước deadband) để telemetry phản ánh đúng PID output

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
    _motor.setHardBrake();
    delay(50);
    _motor.setFreewheel();
}

void MotorDriver::coast() {
    _currentSpeed = 0;
    _motor.setFreewheel();
}
