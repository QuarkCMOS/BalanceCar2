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

    // Phải gọi đầu mỗi loop() — xử lý incoming events
    void handleClient();

    // Gửi JSON telemetry.
    // Non-blocking: nếu WiFi stack busy quá WS_SEND_TIMEOUT_US → bỏ frame.
    // Trả về true nếu đã gửi, false nếu bỏ qua.
    bool sendTelemetry(const String& json);

    void onCommand(WSCommandCallback cb);
    bool hasClient() const;

    // Số frame bị drop do timeout — dùng để debug
    uint32_t droppedFrames() const { return _droppedFrames; }

private:
    WebSocketsServer  _ws;
    WSCommandCallback _commandCallback = nullptr;
    bool              _hasClient       = false;
    uint32_t          _droppedFrames   = 0;

    void _onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len);

    static WebSocketManager* _instance;
    static void _staticHandler(uint8_t num, WStype_t type,
                                uint8_t* payload, size_t len);
};
