#include "mpu6050_manager.h"

#include <Wire.h>
#include <math.h>

#include "../config/pins.h"

#define MPU_ADDR 0x68

#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT 0x3B

static bool writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission((uint8_t)MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static bool readRegisters(uint8_t reg,
                          uint8_t* buffer,
                          uint8_t len)
{
    Wire.beginTransmission((uint8_t)MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t received = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len);
    if (received != len) return false;

    for (uint8_t i = 0; i < len; i++) buffer[i] = Wire.read();
    return true;
}

bool MPU6050Manager::begin()
{
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    delay(100);

    // WHO_AM_I
    uint8_t whoami = 0;
    if (!readRegisters(REG_WHO_AM_I, &whoami, 1)) {
        Serial.println("[MPU] WHO_AM_I read failed");
        return false;
    }
    Serial.print("[MPU] WHO_AM_I = 0x");
    Serial.println(whoami, HEX);

    if (whoami != 0x68 && whoami != 0x70) {
        Serial.println("[MPU] Unsupported MPU");
        return false;
    }

    // Wake sensor
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
        Serial.println("[MPU] Wake failed");
        return false;
    }
    delay(100);

    calibrateGyro();

    Serial.println("[MPU] Init OK");
    return true;
}

// FIX: calibrate cả 3 trục gyro thay vì chỉ gyroY
void MPU6050Manager::calibrateGyro()
{
    Serial.println("[MPU] Calibrating gyro (all 3 axes) — giữ xe thẳng...");

    constexpr int samples = 1000;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int   validCount = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t data[14];
        if (!readRegisters(REG_ACCEL_XOUT, data, 14)) continue;

        int16_t gx = (data[8]  << 8) | data[9];
        int16_t gy = (data[10] << 8) | data[11];
        int16_t gz = (data[12] << 8) | data[13];

        sumX += gx / 131.0f;
        sumY += gy / 131.0f;
        sumZ += gz / 131.0f;
        validCount++;

        delay(2);
    }

    if (validCount > 0) {
        gyroBiasX = sumX / validCount;
        gyroBiasY = sumY / validCount;
        gyroBiasZ = sumZ / validCount;
    }

    Serial.printf("[MPU] Gyro bias  X=%.4f  Y=%.4f  Z=%.4f  (samples=%d)\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ, validCount);
}

void MPU6050Manager::update()
{
    uint8_t data[14];
    if (!readRegisters(REG_ACCEL_XOUT, data, 14)) return;

    int16_t ax = (data[0]  << 8) | data[1];
    int16_t ay = (data[2]  << 8) | data[3];
    int16_t az = (data[4]  << 8) | data[5];
    int16_t gx = (data[8]  << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];

    // Convert raw -> physical, trừ bias cả 3 trục
    imuData.accX  = ax / 16384.0f;
    imuData.accY  = ay / 16384.0f;
    imuData.accZ  = az / 16384.0f;

    imuData.gyroX = (gx / 131.0f) - gyroBiasX;
    imuData.gyroY = (gy / 131.0f) - gyroBiasY;
    imuData.gyroZ = (gz / 131.0f) - gyroBiasZ;

    // Accelerometer angle (dùng cả 3 trục để ổn định hơn)
    imuData.accAngle = atan2f(
        -imuData.accX,
        sqrtf(imuData.accY * imuData.accY + imuData.accZ * imuData.accZ)
    ) * (180.0f / PI);
}

IMUData MPU6050Manager::getData()
{
    return imuData;
}

// FIX: đọc góc acc thực tế một lần — dùng để init filter đúng góc
float MPU6050Manager::readAccAngleOnce()
{
    uint8_t data[14];
    if (!readRegisters(REG_ACCEL_XOUT, data, 14)) return 0.0f;

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    float x = ax / 16384.0f;
    float y = ay / 16384.0f;
    float z = az / 16384.0f;

    return atan2f(-x, sqrtf(y * y + z * z)) * (180.0f / PI);
}
