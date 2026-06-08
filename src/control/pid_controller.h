#pragma once

// ================================================================
// pid_controller.h — PID Controller cho xe cân bằng
//
// Dùng "Derivative on Measurement" thay vì "Derivative on Error"
// → tránh derivative kick khi setpoint thay đổi đột ngột
// ================================================================

class PIDController {
public:
    PIDController() = default;

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minOut, float maxOut);
    void setIntegralLimits(float minI, float maxI);

    // Tính PID output
    //   setpoint    — giá trị mong muốn (độ)
    //   measurement — giá trị đo thực tế (độ)
    //   dt          — bước thời gian (giây), phải > 0
    float compute(float setpoint, float measurement, float dt);

    void reset();

    // Getters để debug / telemetry
    float getKp()           const { return _kp; }
    float getKi()           const { return _ki; }
    float getKd()           const { return _kd; }
    float getIntegral()     const { return _integral; }
    float getLastOutput()   const { return _lastOutput; }

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integral      = 0.0f;
    float _prevMeasure   = 0.0f;   // FIX: lưu measurement, không phải error
    float _lastOutput    = 0.0f;
    bool  _firstRun      = true;

    float _minOut = -1023.0f;
    float _maxOut =  1023.0f;
    float _minI   = -200.0f;
    float _maxI   =  200.0f;

    static float clamp(float val, float lo, float hi);
};
