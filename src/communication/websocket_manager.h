#pragma once

#include <Arduino.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// Callback: key-value từ dashboard
// Keys: "set_kp","set_ki","set_kd" | "power"(1/0) | "move"(0-4)
using WSCommandCallback = std::function<void(const String& key, float value)>;

class WebSocketManager {
public:
    explicit WebSocketManager(uint16_t port = 81);

    void begin();
    void handleClient();

    // Gửi JSON telemetry — chỉ gửi khi có client, non-blocking
    void sendTelemetry(const String& json);

    void onCommand(WSCommandCallback cb);
    bool hasClient() const;

private:
    WebSocketsServer  _ws;
    WSCommandCallback _commandCallback = nullptr;
    bool              _hasClient       = false;

    void _onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len);

    static WebSocketManager* _instance;
    static void _staticHandler(uint8_t num, WStype_t type,
                                uint8_t* payload, size_t len);
};