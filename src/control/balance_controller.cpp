#include "balance_controller.h"
#include <math.h>

BalanceController::BalanceController(MotorDriver&          leftMotor,
                                     MotorDriver&          rightMotor,
                                     MPU6050Manager&       imu,
                                     ComplementaryFilter&  filter,
                                     EncoderManager&       encLeft,
                                     EncoderManager&       encRight)
    : _leftMotor(leftMotor)
    , _rightMotor(rightMotor)
    , _imu(imu)
    , _filter(filter)
    , _encLeft(encLeft)
    , _encRight(encRight)
{}

bool BalanceController::begin() {
    _anglePID.setTunings(PID_KP, PID_KI, PID_KD);
    _anglePID.setOutputLimits(-PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    _anglePID.setIntegralLimits(-PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    _anglePID.reset();
    _state = BalanceState::IDLE;
    return true;
}

void BalanceController::update(float dt) {
    // ── 1. Đọc IMU ──────────────────────────────────────────────
    _imu.update();
    IMUData d = _imu.getData();

    // ── 2. Tính góc acc + complementary filter ──────────────────
    float a1 = d.accY;
    float a2 = -d.accZ;
    float gyroRate = -d.gyroX;

    _accAngle     = atan2f(a1, a2) * (180.0f / PI);
    _currentAngle = _filter.update(_accAngle, gyroRate, dt);

    // ── 3. IDLE / ERROR → dừng cứng ─────────────────────────────
    if (_state == BalanceState::IDLE || _state == BalanceState::ERROR) {
        _leftMotor.hardBrake();
        _rightMotor.hardBrake();
        return;
    }

    // ── 4. FALLEN → chờ dựng lại ────────────────────────────────
    if (_state == BalanceState::FALLEN) {
        _leftMotor.hardBrake();
        _rightMotor.hardBrake();
        if (_currentAngle < FALL_ANGLE * 0.5f &&
            _currentAngle > -FALL_ANGLE * 0.5f) {
            _anglePID.reset();
            _positionTicks = 0;
            _angleCorrection = 0.0f;
            _state = BalanceState::RUNNING;
        }
        return;
    }

    // ── 5. Phát hiện ngã ────────────────────────────────────────
    if (_currentAngle > FALL_ANGLE || _currentAngle < -FALL_ANGLE) {
        handleFall();
        return;
    }

    // ── 6. Tầng 1: tính angle correction từ encoder ─────────────
    // Mục đích: bù drift/trôi, giữ xe đứng yên tại chỗ.
    // Output: _angleCorrection (độ) — cộng vào target angle tầng 2.
    _angleCorrection = computePositionCorrection(dt);

    // ── 7. Tầng 2: Angle PID → PWM ──────────────────────────────
    float effectiveTarget = _targetAngle + _angleCorrection;
    _pidOutput = _anglePID.compute(effectiveTarget, _currentAngle, dt);
    applyMotorCommand(_pidOutput);
}

// ================================================================
// Tầng 1: Position / Velocity correction
//
// Cơ chế 2 thành phần:
//
//   VEL_KP * velocity:
//     Damping tức thời — khi xe đang chạy nhanh theo hướng nào,
//     nghiêng ngược chiều để phanh lại.
//     Ví dụ: xe trôi về phía trước (velocity > 0) → correction > 0
//     → effective_target tăng → xe nghiêng ra sau → phanh lại.
//
//   POS_KI * positionTicks:
//     Tích phân vị trí — nếu xe đã bị đẩy ra khỏi vị trí ban đầu,
//     correction tích lũy dần để kéo xe về.
//     Ví dụ: xe bị đẩy 10cm về phía trước, velocity = 0 nhưng
//     positionTicks tăng → correction lớn dần → xe tự về.
//
// Tune:
//   - Tắt cả (=0): xe cân bằng nhưng trôi tự do.
//   - Tăng VEL_KP: phanh tốt hơn khi chạy nhanh, nhưng nếu quá lớn
//     → oscillation (xe lắc qua lại).
//   - Tăng POS_KI: giữ vị trí tốt hơn, nhưng nếu quá lớn
//     → integral windup → xe chạy loạn.
// ================================================================
float BalanceController::computePositionCorrection(float dt) {
    // Cập nhật vận tốc từ encoder
    EncoderData el = _encLeft.getData();
    EncoderData er = _encRight.getData();
    _velocity = (el.velocityMs + er.velocityMs) * 0.5f;

    // Tích phân vị trí (ticks — không cần chuyển sang m vì chỉ dùng tương đối)
    // Dùng delta ticks thay vì velocityMs để tránh noise khi tính RPM
    int32_t dL = el.deltaTicks;
    int32_t dR = er.deltaTicks;
    _positionTicks += (dL + dR) / 2;  // trung bình 2 bánh (integer)

    // Clamp tích phân tránh windup khi xe bị đẩy quá xa
    constexpr int32_t POS_TICKS_MAX = 5000;  // ~vài chục cm tuỳ encoder
    if (_positionTicks >  POS_TICKS_MAX) _positionTicks =  POS_TICKS_MAX;
    if (_positionTicks < -POS_TICKS_MAX) _positionTicks = -POS_TICKS_MAX;

    // Correction = velocity damping + position integral
    float correction = _velKp * _velocity
                     + _posKi * (float)_positionTicks;

    // Clamp correction để không override quá mạnh tầng 2
    if (correction >  _maxCorrection) correction =  _maxCorrection;
    if (correction < -_maxCorrection) correction = -_maxCorrection;

    return correction;
}

void BalanceController::setAxisMode(uint8_t mode) {
    if (mode > 5) return;
    _axisMode = mode;
    float realAngle = _imu.readAccAngleOnce();
    _filter.reset(realAngle);
}

void BalanceController::setTargetAngle(float angle) {
    _targetAngle = angle;
}

void BalanceController::setDifferentialPWM(float delta) {
    _diffDelta = delta;
}

void BalanceController::setPIDTunings(float kp, float ki, float kd) {
    _anglePID.setTunings(kp, ki, kd);
}

void BalanceController::setPositionTunings(float velKp, float posKi, float maxCorrection) {
    _velKp         = velKp;
    _posKi         = posKi;
    _maxCorrection = maxCorrection;
}

void BalanceController::resetPositionRef() {
    _positionTicks   = 0;
    _angleCorrection = 0.0f;
    _velocity        = 0.0f;
}

void BalanceController::enable() {
    if (_state == BalanceState::IDLE || _state == BalanceState::FALLEN) {
        float realAngle = _imu.readAccAngleOnce();
        _filter.reset(realAngle);
        _anglePID.reset();
        resetPositionRef();
        _state = BalanceState::RUNNING;
    }
}

void BalanceController::disable() {
    _leftMotor.hardBrake();
    _rightMotor.hardBrake();
    _leftPWM  = 0;
    _rightPWM = 0;
    _anglePID.reset();
    resetPositionRef();
    _diffDelta = 0.0f;
    _state = BalanceState::IDLE;
}

void BalanceController::handleFall() {
    _leftMotor.hardBrake();
    _rightMotor.hardBrake();
    _leftPWM   = 0;
    _rightPWM  = 0;
    _pidOutput = 0.0f;
    _anglePID.reset();
    resetPositionRef();
    _state = BalanceState::FALLEN;
}

void BalanceController::applyMotorCommand(float command) {
    int pwm = static_cast<int>(command);
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    int deltaInt = static_cast<int>(_diffDelta);
    if (deltaInt >  PWM_MAX) deltaInt =  PWM_MAX;
    if (deltaInt < -PWM_MAX) deltaInt = -PWM_MAX;

    int leftSpd  = constrain(pwm - deltaInt, -PWM_MAX, PWM_MAX);
    int rightSpd = constrain(pwm + deltaInt, -PWM_MAX, PWM_MAX);

    _leftPWM  = leftSpd;
    _rightPWM = rightSpd;

    _leftMotor.setSpeed(_leftPWM);
    _rightMotor.setSpeed(_rightPWM);
}
