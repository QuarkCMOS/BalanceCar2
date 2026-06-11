#pragma once

// ================================================================
// balance_controller.h
// ================================================================

#include "pid_controller.h"
#include "../drivers/motor_driver.h"
#include "../sensors/mpu6050_manager.h"
#include "../filters/complementary_filter.h"
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
                      ComplementaryFilter&  filter);

    bool begin();

    // Vòng điều khiển chính — gọi mỗi LOOP_INTERVAL_US
    void update(float dt);

    // Chọn cặp trục tính góc (0-5)
    // FIX: reset filter về góc thực tế (không phải 0)
    void setAxisMode(uint8_t mode);

    void setTargetAngle(float angle);
    
    // Differential steering: delta PWM giữa 2 motor
    // Dương = quay phải (L fast, R slow)
    // Âm = quay trái (R fast, L slow)
    void setDifferentialPWM(float delta);

    // FIX: enable() hoạt động từ cả IDLE lẫn FALLEN
    // FIX: init filter về góc acc thực tế → tránh PID spike
    void enable();
    void disable();

    void setPIDTunings(float kp, float ki, float kd);

    // Getters cho telemetry
    BalanceState getState()        const { return _state; }
    float        getTargetAngle()  const { return _targetAngle; }
    float        getCurrentAngle() const { return _currentAngle; }
    float        getAccAngle()     const { return _accAngle; }
    float        getPIDOutput()    const { return _pidOutput; }
    int          getLeftPWM()      const { return _leftPWM; }
    int          getRightPWM()     const { return _rightPWM; }
    uint8_t      getAxisMode()     const { return _axisMode; }

private:
    MotorDriver&         _leftMotor;
    MotorDriver&         _rightMotor;
    MPU6050Manager&      _imu;
    ComplementaryFilter& _filter;

    PIDController _pid;

    BalanceState _state        = BalanceState::IDLE;
    float        _targetAngle  = TARGET_ANGLE;
    float        _currentAngle = 0.0f;
    float        _accAngle     = 0.0f;
    float        _pidOutput    = 0.0f;
    int          _leftPWM      = 0;
    int          _rightPWM     = 0;
    uint8_t      _axisMode     = 0;
    float        _diffDelta    = 0.0f;  // differential steering delta

    void handleFall();
    void applyMotorCommand(float command);
};
