#include "websocket_manager.h"
#include "../../src/config/constants.h"

WebSocketManager* WebSocketManager::_instance = nullptr;

WebSocketManager::WebSocketManager(uint16_t port) : _ws(port) {
    _instance = this;
}

void WebSocketManager::begin() {
    _ws.begin();
    _ws.onEvent(_staticHandler);

    // Tắt Nagle (TCP_NODELAY) để giảm latency gói nhỏ.
    // WebSocketsServer không expose setNoDelay trực tiếp,
    // nhưng có thể set sau qua WiFiClient nếu cần.
    Serial.printf("[WS] Server ready  port=%d\n", 81);
}

void WebSocketManager::handleClient() {
    _ws.loop();
}

// Gửi telemetry non-blocking:
//   Đo thời gian trước/sau broadcastTXT.
//   Nếu tốn hơn WS_SEND_TIMEOUT_US → đánh dấu drop, không làm trễ loop.
//
// Lý do loop bị block trước đây:
//   broadcastTXT() của WebSocketsServer gọi WiFiClient.write() bên dưới,
//   và write() có thể block tới vài chục ms khi TCP TX buffer đầy
//   (client chậm / WiFi congestion). Control loop 200 Hz cần mỗi iteration
//   <= 5000 µs, nên bất kỳ blocking nào > 2 ms là đủ để gây jitter.
//
// Giải pháp: đo elapsed và bỏ frame nếu hệ thống bị chậm.
// Nếu drop nhiều → giảm TELEMETRY_INTERVAL_MS trong constants.h.
bool WebSocketManager::sendTelemetry(const String& json) {
    if (!_hasClient) return false;

    uint32_t t0 = micros();
    _ws.broadcastTXT(json.c_str(), json.length());
    uint32_t elapsed = micros() - t0;

    if (elapsed > WS_SEND_TIMEOUT_US) {
        _droppedFrames++;
        // Log thưa thớt để không spam Serial (mỗi 64 frame drop in a row)
        if ((_droppedFrames & 0x3F) == 1) {
            Serial.printf("[WS] WARN: send took %lu us, frame #%lu dropped\n",
                          (unsigned long)elapsed,
                          (unsigned long)_droppedFrames);
        }
        return false;
    }

    return true;
}

void WebSocketManager::onCommand(WSCommandCallback cb) {
    _commandCallback = cb;
}

bool WebSocketManager::hasClient() const {
    return _hasClient;
}

// ─── Event handler ────────────────────────────────────────────

void WebSocketManager::_onEvent(uint8_t num, WStype_t type,
                                 uint8_t* payload, size_t len)
{
    switch (type) {

        case WStype_CONNECTED:
            _hasClient = true;
            _droppedFrames = 0;  // reset counter khi client mới kết nối
            Serial.printf("[WS] #%u connected  %s\n",
                          num, _ws.remoteIP(num).toString().c_str());
            _ws.sendTXT(num, R"({"status":"connected"})");
            break;

        case WStype_DISCONNECTED:
            _hasClient = false;
            Serial.printf("[WS] #%u disconnected  (dropped=%lu)\n",
                          num, (unsigned long)_droppedFrames);
            break;

        case WStype_TEXT: {
            JsonDocument doc;
            if (deserializeJson(doc, payload, len) != DeserializationError::Ok)
                break;

            const char* msgType = doc["type"] | "";

            // {"type":"PID","p":30.0,"i":0.0,"d":0.5}
            if (strcmp(msgType, "PID") == 0) {
                if (!_commandCallback) break;
                _commandCallback("set_kp", doc["p"] | 0.0f);
                _commandCallback("set_ki", doc["i"] | 0.0f);
                _commandCallback("set_kd", doc["d"] | 0.0f);
            }

            // {"type":"POWER","on":true}
            else if (strcmp(msgType, "POWER") == 0) {
                if (!_commandCallback) break;
                _commandCallback("power", (doc["on"] | false) ? 1.0f : 0.0f);
            }

            // {"type":"TARGET","angle":-5.0}
            else if (strcmp(msgType, "TARGET") == 0) {
                if (!_commandCallback) break;
                _commandCallback("set_target", doc["angle"] | 0.0f);
            }

            // {"type":"TELEMETRY_PAUSE","pause":true|false}
            // Tạm dừng / tiếp tục stream telemetry từ xe
            else if (strcmp(msgType, "TELEMETRY_PAUSE") == 0) {
                if (!_commandCallback) break;
                _commandCallback("telemetry_pause", (doc["pause"] | false) ? 1.0f : 0.0f);
            }

            // {"type":"CTRL","dir":"fwd","state":1}
            // dir: fwd|bwd|lft|rgt   state: 1=press 0=release
            else if (strcmp(msgType, "CTRL") == 0) {
                if (!_commandCallback) break;
                const char* dir = doc["dir"] | "stop";
                int         st  = doc["state"] | 0;

                float code = 0;
                if      (strcmp(dir, "fwd") == 0) code = st ? 1.0f : 0.0f;
                else if (strcmp(dir, "bwd") == 0) code = st ? 2.0f : 0.0f;
                else if (strcmp(dir, "lft") == 0) code = st ? 3.0f : 0.0f;
                else if (strcmp(dir, "rgt") == 0) code = st ? 4.0f : 0.0f;

                _commandCallback("move", code);
            }
            break;
        }

        default:
            break;
    }
}

void WebSocketManager::_staticHandler(uint8_t num, WStype_t type,
                                       uint8_t* payload, size_t len) {
    if (_instance) _instance->_onEvent(num, type, payload, len);
}