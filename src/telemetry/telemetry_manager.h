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
    const char* state;   // "IDLE"|"RUN"|"FALL"|"ERR"
};

class TelemetryManager {
public:
    void   update(const TelemetryPayload& p);
    String buildTelemetryJSON() const;

private:
    TelemetryPayload _p{};
};