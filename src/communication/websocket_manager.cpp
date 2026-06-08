#include "websocket_manager.h"

WebSocketManager* WebSocketManager::_instance = nullptr;

WebSocketManager::WebSocketManager(uint16_t port) : _ws(port) {
    _instance = this;
}

void WebSocketManager::begin() {
    _ws.begin();
    _ws.onEvent(_staticHandler);
    Serial.printf("[WS] Server ready  port=%d\n", 81);
}

void WebSocketManager::handleClient() {
    _ws.loop();
}

void WebSocketManager::sendTelemetry(const String& json) {
    if (!_hasClient) return;
    _ws.broadcastTXT(json.c_str(), json.length());
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
            Serial.printf("[WS] #%u connected  %s\n",
                          num, _ws.remoteIP(num).toString().c_str());
            // Xác nhận kết nối
            _ws.sendTXT(num, R"({"status":"connected"})");
            break;

        case WStype_DISCONNECTED:
            _hasClient = false;
            Serial.printf("[WS] #%u disconnected\n", num);
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