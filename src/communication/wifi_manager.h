#pragma once

#include <Arduino.h>
#include <WiFi.h>

// wifi_manager.h — ESP32 phát AP, chỉ cho 1 thiết bị kết nối

class WiFiManagerCustom {
public:
    // Khởi động AP mode. Trả về true nếu thành công.
    bool begin(const char* ssid, const char* password, uint32_t unused = 0);

    // Gọi trong loop() — log nếu có hơn 1 client (không nên xảy ra)
    void update();

    bool    isConnected()    const;  // true khi AP đang chạy
    String  getIP()          const;  // luôn "192.168.4.1"
    uint8_t stationCount()   const;

private:
    bool     _apStarted    = false;
    uint32_t _lastCheckMs  = 0;
};