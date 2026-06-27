#pragma once
#include <Arduino.h>

struct TelemetryPayload {
    float       angle;
    float       targetAngle;
    float       pidOutput;
    int         pwmLeft;
    int         pwmRight;
    float       rpmLeft;
    float       rpmRight;
    float       velocity;        // m/s trung bình 2 bánh (từ encoder)
    float       angleCorrection; // correction từ tầng 1 position loop (°)
    int32_t     positionTicks;   // tích phân vị trí encoder (tầng 1)
    const char* state;           // "IDLE"|"RUN"|"FALL"|"ERR"
};

class TelemetryManager {
public:
    void   update(const TelemetryPayload& p);
    String buildTelemetryJSON() const;

private:
    TelemetryPayload _p{};
};