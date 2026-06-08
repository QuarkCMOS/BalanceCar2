#include "pwm_manager.h"

PWMManager::PWMManager()
    : _left(MotorPinConfig{
          PIN_MOTOR_L1,        // IN1 motor trái
          PIN_MOTOR_L2,        // IN2 motor trái
          MCPWM_UNIT_0,
          MCPWM_TIMER_0,
          MCPWM0A,
          MCPWM0B
      }, true),                // reversed=true: motor trái gắn đối xứng
      _right(MotorPinConfig{
          PIN_MOTOR_R1,        // IN1 motor phải
          PIN_MOTOR_R2,        // IN2 motor phải
          MCPWM_UNIT_0,
          MCPWM_TIMER_1,       // Timer khác để 2 motor hoàn toàn độc lập
          MCPWM1A,
          MCPWM1B
      }, false)                // reversed=false: motor phải đúng chiều
{}

void PWMManager::begin() {
    // FIX LỖI 2: init chân nSLEEP của DRV8833 — phải HIGH thì IC mới hoạt động
    // Nếu bỏ qua, GPIO 27 floating → DRV8833 có thể ở chế độ sleep bất định
    pinMode(PIN_DRV_ULT, OUTPUT);
    digitalWrite(PIN_DRV_ULT, HIGH);
    delay(10);  // chờ DRV8833 wake up (datasheet: t_wake ~ 1ms)

    _left.begin();
    _right.begin();

    Serial.printf("[PWM] nSLEEP GPIO%d = HIGH (DRV8833 awake)\n", PIN_DRV_ULT);
    Serial.printf("[PWM] MCPWM ready — %d Hz, max %d\n", PWM_FREQ_HZ, PWM_MAX);
}

void PWMManager::write(int leftSpeed, int rightSpeed) {
    _left.setSpeed(leftSpeed);
    _right.setSpeed(rightSpeed);
}

void PWMManager::emergencyBrake() {
    _left.hardBrake();
    _right.hardBrake();
    Serial.println("[PWM] EMERGENCY BRAKE");
}

void PWMManager::softBrake() {
    _left.softBrake();
    _right.softBrake();
}