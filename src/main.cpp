// ================================================================
// main.cpp — Xe 2 bánh tự cân bằng
//
// Kiến trúc WiFi:
//   ESP32 phát AP "BalanceCar" / "12345678"
//   Chỉ cho phép 1 thiết bị kết nối (max_connection=1)
//   Dashboard chạy độc lập trên máy/điện thoại, kết nối:
//     ws://192.168.4.1:81
//
// Telemetry JSON (50 Hz mặc định, chỉnh TELEMETRY_INTERVAL_MS):
//   {"a":<angle>,"t":<target>,"pid":<out>,
//    "pwl":<pwm_l>,"pwr":<pwm_r>,
//    "rl":<rpm_l>,"rr":<rpm_r>,
//    "s":<"IDLE"|"RUN"|"FALL"|"ERR">,"ms":<millis>}
//
// Commands từ dashboard → ESP32:
//   {"type":"PID",  "p":30.0,"i":0.0,"d":0.5}
//   {"type":"POWER","on":true}
//   {"type":"CTRL", "dir":"fwd"|"bwd"|"lft"|"rgt","state":1|0}
//
// Serial debug:
//   e/d/r     — enable / disable / reset+enable
//   p         — in PID hiện tại
//   P kp ki kd
//   x 0..5    — chọn trục góc
//   M L|R spd — test motor
//   h         — help
// ================================================================

#include <Arduino.h>
#include "config/constants.h"
#include "config/pins.h"
#include "sensors/mpu6050_manager.h"
#include "encoders/encoder_manager.h"
#include "filters/complementary_filter.h"
#include "control/balance_controller.h"
#include "drivers/motor_driver.h"
#include "communication/wifi_manager.h"
#include "communication/websocket_manager.h"
#include "telemetry/telemetry_manager.h"

extern volatile int32_t g_ticksLeft;
extern volatile int32_t g_ticksRight;

// ── Hardware ───────────────────────────────────────────────────
MPU6050Manager      imu;
ComplementaryFilter filter(ALPHA);

MotorPinConfig cfgLeft  = { PIN_MOTOR_L1, PIN_MOTOR_L2,
                              MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B };
MotorPinConfig cfgRight = { PIN_MOTOR_R1, PIN_MOTOR_R2,
                              MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B };

MotorDriver motorLeft (cfgLeft,  true);
MotorDriver motorRight(cfgRight, false);

BalanceController balancer(motorLeft, motorRight, imu, filter);

EncoderManager encLeft (PIN_ENC_L_A, PIN_ENC_L_B, &g_ticksLeft);
EncoderManager encRight(PIN_ENC_R_A, PIN_ENC_R_B, &g_ticksRight);

// ── Communication ─────────────────────────────────────────────
WiFiManagerCustom wifi;
WebSocketManager  wsManager(WS_PORT);
TelemetryManager  telemetry;

// ── Timing ────────────────────────────────────────────────────
uint32_t lastLoopUs      = 0;
uint32_t lastPrintMs     = 0;
uint32_t lastTelemetryMs = 0;

// ── Runtime state ─────────────────────────────────────────────
float runtimeKp = PID_KP;
float runtimeKi = PID_KI;
float runtimeKd = PID_KD;
float runtimeTargetAngle = TARGET_ANGLE;  // target angle có thể chỉnh từ web

bool motorTestMode = false;

// Tạm dừng gửi telemetry lên web (vẫn nhận lệnh)
bool telemetryPaused = false;

// Góc lúc khởi động — dùng làm target angle mặc định
float bootAngle = 0.0f;
bool  bootAngleCaptured = false;

// Hướng di chuyển: 0=STOP 1=FWD 2=BWD 3=LEFT 4=RIGHT
volatile int moveDir = 0;

// Offset target angle / PWM delta khi điều khiển
// FWD/BWD: thay đổi target angle
// LEFT/RIGHT: sử dụng differential motor control (delta PWM)
constexpr float FWD_OFFSET   =  2.5f;
constexpr float BWD_OFFSET   = -2.5f;
constexpr float TURN_DELTA   = 100.0f;  // PWM delta cho left/right (max 1023)

const char* axisModeStr[] = {
    "atan2(accX,accZ)/gyroY [DEFAULT]",
    "atan2(accY,accZ)/gyroX",
    "atan2(accX,accY)/gyroZ",
    "atan2(-accX,accZ)/gyroY",
    "atan2(-accY,accZ)/gyroX",
    "atan2(-accX,accY)/gyroZ"
};

char    inputBuf[48];
uint8_t inputLen = 0;

// ── Helpers ───────────────────────────────────────────────────
const char* stateStr(BalanceState s) {
    switch (s) {
        case BalanceState::IDLE:    return "IDLE";
        case BalanceState::RUNNING: return "RUN";
        case BalanceState::FALLEN:  return "FALL";
        case BalanceState::ERROR:   return "ERR";
    }
    return "?";
}

void applyPID() {
    balancer.setPIDTunings(runtimeKp, runtimeKi, runtimeKd);
    Serial.printf("[PID] Kp=%.2f  Ki=%.3f  Kd=%.3f\n",
                  runtimeKp, runtimeKi, runtimeKd);
}

void printHelp() {
    Serial.println(F("\n===== BALANCE CAR ====="));
    Serial.println(F("  e           — Enable"));
    Serial.println(F("  d           — Disable"));
    Serial.println(F("  r           — Reset + Enable"));
    Serial.println(F("  p           — In PID"));
    Serial.println(F("  P kp ki kd  — Set PID"));
    Serial.println(F("  x 0-5       — Trục góc"));
    Serial.println(F("  M L|R spd   — Test motor"));
    Serial.println(F("  h           — Help"));
    Serial.printf (  "WiFi AP: %-15s  ws://192.168.4.1:%d\n",
                   AP_SSID, WS_PORT);
    Serial.println(F("=======================\n"));
}

// ── Áp dụng hướng di chuyển + differential steering ────────────
// Gọi mỗi control loop
// FWD/BWD: thay đổi target angle
// LEFT/RIGHT: applied as PWM delta (differential motor control)
void applyMoveDir() {
    float base = runtimeTargetAngle;
    
    switch (moveDir) {
        case 1:  // FWD
            balancer.setTargetAngle(base + FWD_OFFSET);
            balancer.setDifferentialPWM(0.0f);  // no turn
            break;
        case 2:  // BWD
            balancer.setTargetAngle(base + BWD_OFFSET);
            balancer.setDifferentialPWM(0.0f);  // no turn
            break;
        case 3:  // LEFT: right motor faster
            balancer.setTargetAngle(base);
            balancer.setDifferentialPWM(-TURN_DELTA);
            break;
        case 4:  // RIGHT: left motor faster
            balancer.setTargetAngle(base);
            balancer.setDifferentialPWM(TURN_DELTA);
            break;
        default: // STOP
            balancer.setTargetAngle(base);
            balancer.setDifferentialPWM(0.0f);
            break;
    }
}

// ── Serial command parser ─────────────────────────────────────
void processLine(char* line) {
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == ' '))
        line[--len] = 0;
    if (len == 0) return;

    char cmd = line[0];

    if (len == 1) {
        switch (cmd) {
            case 'e': case 'E':
                motorTestMode = false; moveDir = 0;
                balancer.enable();
                Serial.println(F("[CMD] ENABLED"));
                break;
            case 'd': case 'D':
                motorTestMode = false; moveDir = 0;
                balancer.disable();
                Serial.println(F("[CMD] DISABLED"));
                break;
            case 'r': case 'R':
                motorTestMode = false; moveDir = 0;
                balancer.disable();
                delay(50);
                balancer.enable();
                Serial.println(F("[CMD] RESET + ENABLED"));
                break;
            case 'p': case 'P': applyPID();   break;
            case 'h': case 'H': printHelp();  break;
            default:
                Serial.printf("[?] %s\n", line);
        }
        return;
    }

    if ((cmd == 'M' || cmd == 'm') && line[1] == ' ') {
        char side; int spd;
        if (sscanf(line + 2, " %c %d", &side, &spd) == 2) {
            balancer.disable();
            motorTestMode = true;
            (side == 'L' || side == 'l' ? motorLeft : motorRight).setSpeed(spd);
            Serial.printf("[TEST] motor%c spd=%d\n",
                          (side == 'L' || side == 'l') ? 'L' : 'R', spd);
        } else {
            Serial.println(F("[ERR] M <L|R> <spd>"));
        }
        return;
    }

    if (cmd == 'P' && line[1] == ' ') {
        float kp, ki, kd;
        if (sscanf(line + 2, "%f %f %f", &kp, &ki, &kd) == 3
            && kp >= 0 && ki >= 0 && kd >= 0) {
            runtimeKp = kp; runtimeKi = ki; runtimeKd = kd;
            applyPID();
        } else {
            Serial.println(F("[ERR] P <kp> <ki> <kd>"));
        }
        return;
    }

    if ((cmd == 'x' || cmd == 'X') && line[1] == ' ') {
        int mode = atoi(line + 2);
        if (mode >= 0 && mode <= 5) {
            balancer.setAxisMode((uint8_t)mode);
            Serial.printf("[AXIS] %d: %s\n", mode, axisModeStr[mode]);
        } else {
            Serial.println(F("[ERR] x 0-5"));
        }
        return;
    }

    Serial.printf("[?] %s\n", line);
}

// ── WebSocket command handler ─────────────────────────────────
void onWSCommand(const String& key, float value) {
    static float pk = PID_KP, pi = PID_KI, pd = PID_KD;

    if      (key == "set_kp") { pk = value; }
    else if (key == "set_ki") { pi = value; }
    else if (key == "set_kd") {
        // Nhận đủ 3 hệ số → apply ngay
        runtimeKp = pk; runtimeKi = pi; runtimeKd = value;
        balancer.setPIDTunings(runtimeKp, runtimeKi, runtimeKd);
        Serial.printf("[WS-PID] Kp=%.2f Ki=%.3f Kd=%.3f\n",
                      runtimeKp, runtimeKi, runtimeKd);
    }
    else if (key == "set_target") {
        float angle = constrain(value, -30.0f, 30.0f);
        runtimeTargetAngle = angle;
        balancer.setTargetAngle(angle);
        Serial.printf("[WS-TARGET] targetAngle=%.2f\n", angle);
    }
    else if (key == "telemetry_pause") {
        telemetryPaused = (value > 0.5f);
        Serial.printf("[WS] Telemetry %s\n", telemetryPaused ? "PAUSED" : "RESUMED");
    }
    else if (key == "power") {
        motorTestMode = false; moveDir = 0;
        if (value > 0.5f) {
            balancer.enable();
            Serial.println("[WS] ON");
        } else {
            balancer.disable();
            Serial.println("[WS] OFF");
        }
    }
    else if (key == "move") {
        // 0=STOP 1=FWD 2=BWD 3=LEFT 4=RIGHT
        moveDir = (int)value;
    }
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(200);
    printHelp();

    // DRV8833 nEnable
    pinMode(PIN_DRV_ULT, OUTPUT);
    digitalWrite(PIN_DRV_ULT, HIGH);

    // MPU6050
    Serial.print(F("[INIT] MPU6050... "));
    if (!imu.begin()) {
        while (true) { Serial.println(F("FAIL!")); delay(2000); }
    }
    Serial.println(F("OK"));

    // Motors + Encoders
    motorLeft.begin();
    motorRight.begin();
    encLeft.begin();
    encRight.begin();

    // Balance controller
    balancer.begin();

    // WiFi AP — không cần internet, tự phát sóng
    wifi.begin(AP_SSID, AP_PASSWORD);

    // WebSocket server
    wsManager.begin();
    wsManager.onCommand(onWSCommand);

    Serial.printf("[INIT] Dashboard: mở file HTML → ws://192.168.4.1:%d\n", WS_PORT);
    Serial.println(F("[INIT] Gõ 'e' để bắt đầu, hoặc nhấn ON trên dashboard.\n"));
    Serial.println(F("ms,angle,target,pid,pwl,pwr,rl,rr,state"));

    lastLoopUs      = micros();
    lastPrintMs     = millis();
    lastTelemetryMs = millis();
}

// ═══════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
    // WS events — ưu tiên cao, gọi đầu loop
    wsManager.handleClient();

    // AP watchdog (5s check)
    wifi.update();

    // Serial debug input
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputLen > 0) {
                inputBuf[inputLen] = 0;
                processLine(inputBuf);
                inputLen = 0;
            }
        } else if (inputLen < sizeof(inputBuf) - 1) {
            inputBuf[inputLen++] = c;
        }
    }

    // ── Control loop 200 Hz ───────────────────────────────────
    // QUAN TRỌNG: Vòng điều khiển chạy ĐỘCNHẬT với WebSocket!
    // Nếu mất kết nối → xe vẫn tiếp tục cân bằng, không dừng
    uint32_t nowUs   = micros();
    uint32_t elapsed = nowUs - lastLoopUs;
    if (elapsed < LOOP_INTERVAL_US) return;
    lastLoopUs = nowUs;

    float dt = elapsed * 1e-6f;

    if (!motorTestMode) {
        // Capture góc tại lần chạy balancer đầu tiên (~500ms sau boot)
        if (!bootAngleCaptured && millis() > 500) {
            bootAngle = balancer.getCurrentAngle();
            bootAngleCaptured = true;
            runtimeTargetAngle = bootAngle;
            balancer.setTargetAngle(bootAngle);
            Serial.printf("[BOOT] bootAngle=%.2f — dùng làm target angle mặc định\n",
                          bootAngle);
        }
        applyMoveDir();
        balancer.update(dt);
    }

    encLeft.update(dt);
    encRight.update(dt);

    uint32_t nowMs = millis();

    // ── Telemetry WS @ TELEMETRY_INTERVAL_MS ─────────────────
    // Chỉ gửi nếu có client, nhưng KHÔNG ảnh hưởng đến balance loop
    if (!telemetryPaused && nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = nowMs;

        if (wsManager.hasClient()) {
            EncoderData el = encLeft.getData();
            EncoderData er = encRight.getData();

            TelemetryPayload pl{
                .angle       = balancer.getCurrentAngle(),
                .targetAngle = balancer.getTargetAngle(),
                .pidOutput   = balancer.getPIDOutput(),
                .pwmLeft     = balancer.getLeftPWM(),
                .pwmRight    = balancer.getRightPWM(),
                .rpmLeft     = el.rpm,
                .rpmRight    = er.rpm,
                .state       = stateStr(balancer.getState())
            };
            telemetry.update(pl);
            wsManager.sendTelemetry(telemetry.buildTelemetryJSON());
        }
    } else if (telemetryPaused) {
        lastTelemetryMs = nowMs;
    }

    // ── Serial CSV 20 Hz ──────────────────────────────────────
    if (nowMs - lastPrintMs >= 50) {
        lastPrintMs = nowMs;
        EncoderData el = encLeft.getData();
        EncoderData er = encRight.getData();
        Serial.printf("%lu,%.2f,%.2f,%.0f,%d,%d,%.1f,%.1f,%s\n",
                      (unsigned long)nowMs,
                      balancer.getCurrentAngle(),
                      balancer.getTargetAngle(),
                      balancer.getPIDOutput(),
                      balancer.getLeftPWM(), balancer.getRightPWM(),
                      el.rpm, er.rpm,
                      stateStr(balancer.getState()));
    }
}
