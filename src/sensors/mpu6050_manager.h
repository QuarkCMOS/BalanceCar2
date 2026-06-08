#ifndef MPU6050_MANAGER_H
#define MPU6050_MANAGER_H

#include <Arduino.h>

struct IMUData
{
    // Acceleration (g)
    float accX;
    float accY;
    float accZ;

    // Gyroscope (deg/s) — đã trừ bias
    float gyroX;
    float gyroY;
    float gyroZ;

    // Accelerometer angle only
    float accAngle;
};

class MPU6050Manager
{
public:
    bool begin();

    void update();

    IMUData getData();

    // Trả về bias của từng trục (dùng để debug)
    float getGyroBiasX() const { return gyroBiasX; }
    float getGyroBiasY() const { return gyroBiasY; }
    float getGyroBiasZ() const { return gyroBiasZ; }

    // Đọc góc thực tế một lần (dùng khi reset filter)
    float readAccAngleOnce();

private:
    void calibrateGyro();

private:
    IMUData imuData {};

    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;
};

#endif
