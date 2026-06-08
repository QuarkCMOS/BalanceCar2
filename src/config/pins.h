#pragma once
 
// pins.h — ESP32 pin assignments
 
// I2C — MPU6050
constexpr int PIN_SDA       = 21;
constexpr int PIN_SCL       = 22;
 
// DRV8833 Sleep mode (OFF)
constexpr int PIN_DRV_ULT   = 27;   // ULT

// DRV8833 — Motor trái (IN3/IN4)
//constexpr int PIN_MOTOR_L1  = 33;   // IN3
//constexpr int PIN_MOTOR_L2  = 32;   // IN4
 
// DRV8833 — Motor phải (IN1/IN2)
//constexpr int PIN_MOTOR_R1  = 26;   // IN1
//constexpr int PIN_MOTOR_R2  = 25;   // IN2
 


// DRV8833 — Motor trái (IN3/IN4)
constexpr int PIN_MOTOR_L1  = 25;   // IN3
constexpr int PIN_MOTOR_L2  = 26;   // IN4
 
// DRV8833 — Motor phải (IN1/IN2)
constexpr int PIN_MOTOR_R1  = 32;   // IN1
constexpr int PIN_MOTOR_R2  = 33;   // IN2


// Encoder trái — GA25-370
constexpr int PIN_ENC_L_A   = 19;   // CA
constexpr int PIN_ENC_L_B   = 18;   // CB
 
// Encoder phải — GA25-370
constexpr int PIN_ENC_R_A   = 17;   // CA
constexpr int PIN_ENC_R_B   = 16;   // CB
