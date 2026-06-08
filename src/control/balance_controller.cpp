#include "balance_controller.h"
#include <math.h>

BalanceController::BalanceController(MotorDriver&          leftMotor,
                                     MotorDriver&          rightMotor,
                                     MPU6050Manager&       imu,
                                     ComplementaryFilter&  filter)
    : _leftMotor(leftMotor)
    , _rightMotor(rightMotor)
    , _imu(imu)
    , _filter(filter)
{}

bool BalanceController::begin() {
    _pid.setTunings(PID_KP, PID_KI, PID_KD);
    _pid.setOutputLimits(-PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    _pid.setIntegralLimits(-PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    _pid.reset();
    _state = BalanceState::IDLE;
    return true;
}

void BalanceController::update(float dt) {
    // ── 1. Đọc IMU ──────────────────────────────────────────
    _imu.update();
    IMUData d = _imu.getData();

    // ── 2. Tính góc acc theo axisMode ───────────────────────
    // float a1, a2, gyroRate;
    // switch (_axisMode) {
    //     default:
    //     case 0: a1 =  d.accX; a2 = d.accZ; gyroRate =  d.gyroY; break;
    //     case 1: a1 =  d.accY; a2 = d.accZ; gyroRate =  d.gyroX; break;
    //     case 2: a1 =  d.accX; a2 = d.accY; gyroRate =  d.gyroZ; break;
    //     case 3: a1 = -d.accX; a2 = d.accZ; gyroRate =  d.gyroY; break;
    //     case 4: a1 = -d.accY; a2 = d.accZ; gyroRate =  d.gyroX; break;
    //     case 5: a1 = -d.accX; a2 = d.accY; gyroRate =  d.gyroZ; break;
    // }
    float a1 = d.accY;
    float a2 = -d.accZ;
    float gyroRate = -d.gyroX;


    _accAngle = atan2f(a1, a2) * (180.0f / PI);
    float filteredAngle = _filter.update(_accAngle, gyroRate, dt);
    _currentAngle = filteredAngle;

    // ── 3. IDLE / ERROR → brake cứng ────────────────────────
    if (_state == BalanceState::IDLE || _state == BalanceState::ERROR) {
        _leftMotor.hardBrake();
        _rightMotor.hardBrake();
        return;
    }

    // ── 4. FIX: xử lý FALLEN trước khi check góc ────────────
    // Bug cũ: check góc trước → handleFall() set FALLEN → return
    // Lần sau vào lại, góc vẫn > FALL_ANGLE → handleFall() lại → loop reset PID
    // Fix: nếu đang FALLEN thì brake và check xem có thể recover không
    if (_state == BalanceState::FALLEN) {
        _leftMotor.hardBrake();
        _rightMotor.hardBrake();

        // Xe được dựng lại: góc về gần 0 → auto-recover
        if (filteredAngle < FALL_ANGLE * 0.5f &&
            filteredAngle > -FALL_ANGLE * 0.5f) {
            _pid.reset();
            _state = BalanceState::RUNNING;
        }
        return;
    }

    // ── 5. Phát hiện ngã (chỉ khi đang RUNNING) ─────────────
    if (filteredAngle > FALL_ANGLE || filteredAngle < -FALL_ANGLE) {
        handleFall();
        return;
    }

    // ── 6. RUNNING: PID → motor ─────────────────────────────
    _pidOutput = _pid.compute(_targetAngle, filteredAngle, dt);
    applyMotorCommand(_pidOutput);
}

void BalanceController::setAxisMode(uint8_t mode) {
    if (mode > 5) return;
    _axisMode = mode;

    // FIX: reset filter về góc thực tế, không phải 0
    float realAngle = _imu.readAccAngleOnce();
    _filter.reset(realAngle);
}

void BalanceController::setTargetAngle(float angle) {
    _targetAngle = angle;
}

// FIX: cho phép enable từ cả IDLE lẫn FALLEN
// FIX: reset filter về góc acc thực tế trước khi chạy PID
void BalanceController::enable() {
    if (_state == BalanceState::IDLE || _state == BalanceState::FALLEN) {
        // Đọc góc thực tế và init filter từ đó → tránh PID spike lần đầu
        float realAngle = _imu.readAccAngleOnce();
        _filter.reset(realAngle);
        _pid.reset();
        _state = BalanceState::RUNNING;
    }
}

void BalanceController::disable() {
    _leftMotor.hardBrake();
    _rightMotor.hardBrake();
    _leftPWM  = 0;
    _rightPWM = 0;
    _pid.reset();
    _state = BalanceState::IDLE;
}

void BalanceController::setPIDTunings(float kp, float ki, float kd) {
    _pid.setTunings(kp, ki, kd);
}

void BalanceController::handleFall() {
    _leftMotor.hardBrake();
    _rightMotor.hardBrake();
    _leftPWM   = 0;
    _rightPWM  = 0;
    _pidOutput = 0.0f;
    _pid.reset();
    _state = BalanceState::FALLEN;
}

void BalanceController::applyMotorCommand(float command) {
    int pwm = static_cast<int>(command);
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    _leftPWM  = pwm;
    _rightPWM = pwm;

    _leftMotor.setSpeed(_leftPWM);
    _rightMotor.setSpeed(_rightPWM);
}
