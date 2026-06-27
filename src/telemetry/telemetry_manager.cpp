#include "telemetry_manager.h"

void TelemetryManager::update(const TelemetryPayload& p) {
    _p = p;
}

// JSON nhỏ gọn, không có space — tối ưu cho WebSocket realtime
// {"a":0.12,"t":0.0,"pid":45,"pwl":200,"pwr":200,"rl":12.1,"rr":12.0,"v":0.12,"s":"RUN","ms":12345}
// a=angle  t=target  pid=pid_output  pwl/pwr=pwm  rl/rr=rpm  v=velocity(m/s)  s=state  ms=timestamp
String TelemetryManager::buildTelemetryJSON() const {
    static char buf[200];
    snprintf(buf, sizeof(buf),
        "{\"a\":%.2f,\"t\":%.2f,\"pid\":%.0f"
        ",\"pwl\":%d,\"pwr\":%d"
        ",\"rl\":%.1f,\"rr\":%.1f"
        ",\"v\":%.3f"
        ",\"ac\":%.2f,\"pt\":%ld"
        ",\"s\":\"%s\",\"ms\":%lu}",
        _p.angle,
        _p.targetAngle,
        _p.pidOutput,
        _p.pwmLeft,
        _p.pwmRight,
        _p.rpmLeft,
        _p.rpmRight,
        _p.velocity,
        _p.angleCorrection,
        (long)_p.positionTicks,
        _p.state,
        (unsigned long)millis()
    );
    return String(buf);
}