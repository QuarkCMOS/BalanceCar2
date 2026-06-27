#pragma once

// ================================================================
// balance_controller.h — Cascade control: Position → Angle → PWM
//
// Tầng 1 (Position/Velocity):
//   Input:  encoder ticks (vị trí tích phân) + vận tốc thực tế
//   Output: angle_correction (offset cộng vào target angle)
//   Mục đích: giữ xe đứng yên tại chỗ, không trôi
//
// Tầng 2 (Angle/Balance):
//   Input:  filtered_angle vs (target_angle + angle_correction)
//   Output: PWM command
//   Mục đích: giữ xe thẳng đứng
//
// Tune theo thứ tự:
//   1. Tắt tầng 1 (VEL_KP=VEL_KI=POS_KI=0), chỉnh Kp/Kd tầng 2 cho xe cân bằng.
//   2. Bật VEL_KP nhỏ để chống drift tức thời.
//   3. Tăng POS_KI để kéo xe về vị trí ban đầu.
// ================================================================

#include "pid_controller.h"
#include "../drivers/motor_driver.h"
#include "../sensors/mpu6050_manager.h"
#include "../filters/complementary_filter.h"
#include "../encoders/encoder_manager.h"
#include "../config/constants.h"

enum class BalanceState : uint8_t {
    IDLE,       // Chưa khởi động hoặc đã dừng thủ công
    RUNNING,    // Đang điều khiển cân bằng bình thường
    FALLEN,     // Phát hiện ngã — emergency stop, chờ dựng lại
    ERROR       // Lỗi khởi tạo sensor
};

class BalanceController {
public:
    BalanceController(MotorDriver&          leftMotor,
                      MotorDriver&          rightMotor,
                      MPU6050Manager&       imu,
                      ComplementaryFilter&  filter,
                      EncoderManager&       encLeft,
                      EncoderManager&       encRight);

    bool begin();

    // Vòng điều khiển chính — gọi mỗi LOOP_INTERVAL_US
    void update(float dt);

    // Chọn cặp trục tính góc (0-5)
    void setAxisMode(uint8_t mode);

    void setTargetAngle(float angle);

    // Differential steering: delta PWM giữa 2 motor
    // Dương = quay phải (L fast, R slow), âm = quay trái
    void setDifferentialPWM(float delta);

    void enable();
    void disable();

    // Tune tầng 2 (Angle PID) — gọi từ web/serial
    void setPIDTunings(float kp, float ki, float kd);

    // Tune tầng 1 (Velocity/Position) — gọi từ web/serial
    // velKp: damping vận tốc tức thời (chống trôi nhanh)
    // posKi: tích phân vị trí (kéo về chỗ cũ sau khi bị đẩy)
    // maxCorrection: giới hạn angle correction (°)
    void setPositionTunings(float velKp, float posKi, float maxCorrection);

    // Reset vị trí tham chiếu về vị trí hiện tại
    // (gọi khi bắt đầu chạy hoặc sau khi điều khiển FWD/BWD)
    void resetPositionRef();

    // Getters cho telemetry
    BalanceState getState()           const { return _state; }
    float        getTargetAngle()     const { return _targetAngle; }
    float        getCurrentAngle()    const { return _currentAngle; }
    float        getAccAngle()        const { return _accAngle; }
    float        getPIDOutput()       const { return _pidOutput; }
    int          getLeftPWM()         const { return _leftPWM; }
    int          getRightPWM()        const { return _rightPWM; }
    uint8_t      getAxisMode()        const { return _axisMode; }
    float        getVelocity()        const { return _velocity; }
    float        getAngleCorrection() const { return _angleCorrection; }
    int32_t      getPositionTicks()   const { return _positionTicks; }

private:
    MotorDriver&         _leftMotor;
    MotorDriver&         _rightMotor;
    MPU6050Manager&      _imu;
    ComplementaryFilter& _filter;
    EncoderManager&      _encLeft;
    EncoderManager&      _encRight;

    // Tầng 2: Angle PID
    PIDController _anglePID;

    BalanceState _state        = BalanceState::IDLE;
    float        _targetAngle  = TARGET_ANGLE;
    float        _currentAngle = 0.0f;
    float        _accAngle     = 0.0f;
    float        _pidOutput    = 0.0f;
    int          _leftPWM      = 0;
    int          _rightPWM     = 0;
    uint8_t      _axisMode     = 0;
    float        _diffDelta    = 0.0f;

    // Tầng 1: Position / Velocity
    // Hệ số — có thể tune runtime
    float   _velKp         = VEL_KP_DEFAULT;
    float   _posKi         = POS_KI_DEFAULT;
    float   _maxCorrection = VEL_CORRECTION_MAX;

    float   _velocity        = 0.0f;   // m/s, trung bình 2 bánh
    int32_t _positionTicks   = 0;      // tổng tick từ lần enable (vị trí tích phân)
    float   _angleCorrection = 0.0f;   // angle offset từ tầng 1 → tầng 2

    void  handleFall();
    void  applyMotorCommand(float command);
    float computePositionCorrection(float dt);
};
