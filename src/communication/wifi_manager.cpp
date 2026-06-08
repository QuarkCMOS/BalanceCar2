#include "wifi_manager.h"

bool WiFiManagerCustom::begin(const char* ssid,
                               const char* password,
                               uint32_t /*unused*/) {
    WiFi.mode(WIFI_AP);

    // max_connection=1: driver-level reject client thứ 2
    bool ok = WiFi.softAP(ssid, password,
                           /*channel=*/1,
                           /*hidden=*/false,
                           /*max_connection=*/1);
    if (ok) {
        _apStarted = true;
        Serial.printf("[WiFi-AP] SSID: %s  IP: %s\n",
                      ssid, WiFi.softAPIP().toString().c_str());
        Serial.println("[WiFi-AP] Tối đa 1 thiết bị kết nối.");
    } else {
        Serial.println("[WiFi-AP] softAP() thất bại!");
    }
    return ok;
}

void WiFiManagerCustom::update() {
    uint32_t now = millis();
    if (now - _lastCheckMs < 5000) return;
    _lastCheckMs = now;

    uint8_t n = WiFi.softAPgetStationNum();
    if (n > 1) {
        // max_connection=1 đã set, trường hợp này không nên xảy ra
        Serial.printf("[WiFi-AP] WARN: %u clients (max=1)\n", n);
    }
}

bool WiFiManagerCustom::isConnected() const {
    return _apStarted;
}

String WiFiManagerCustom::getIP() const {
    return WiFi.softAPIP().toString();
}

uint8_t WiFiManagerCustom::stationCount() const {
    return WiFi.softAPgetStationNum();
}