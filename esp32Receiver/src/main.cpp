#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include "MQTTManager.h"

HardwareSerial lora(2);
constexpr uint32_t BAUD = 9600;
static const uint8_t SOF = 0xAA;
static const uint8_t HANDSHAKE = 0x55;

enum RxState { WAIT_HDR, READ_PAYLOAD };
static RxState rxState = WAIT_HDR;
static uint8_t buf6[6];
static uint8_t bufIndex = 0;

#ifndef MQTT_SERVER
#define MQTT_SERVER ""
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER ""
#endif
#ifndef MQTT_PASS
#define MQTT_PASS ""
#endif

MQTTManager mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS);
WiFiManager wm;

uint8_t crc8(const uint8_t *d, size_t len) {
  uint8_t c = 0xFF;
  while (len--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; ++i)
      c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1);
  }
  return c;
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin();
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() != WL_CONNECTED) {
    wm.setConfigPortalTimeout(120);
    if (!wm.startConfigPortal("ESP32-ECHO")) ESP.restart();
    Preferences p;
    p.begin("wifi-creds", false);
    p.putString("ssid", WiFi.SSID());
    p.putString("password", WiFi.psk());
    p.end();
  }
  Serial.println("\nWiFi ready");

  lora.begin(BAUD, SERIAL_8N1, 16, 17);
  lora.write(HANDSHAKE);

  mqtt.begin();
  Serial.println("LoRa RX ready");
}

void loop() {
  while (lora.available()) {
    uint8_t b = lora.read();
    if (rxState == WAIT_HDR) {
      if (b == SOF) { rxState = READ_PAYLOAD; bufIndex = 0; }
    } else {
      buf6[bufIndex++] = b;
      if (bufIndex == sizeof(buf6)) {
        uint8_t  type = buf6[0];
        float    val; memcpy(&val, buf6 + 1, 4);
        uint8_t  ok   = (buf6[5] == crc8(buf6, 5));

        Serial.printf("RX type=0x%02X val=%.2f %s\n",
                      type, val, ok ? "OK" : "CRC FAIL");

        if (ok) {
          const char* t = nullptr;
          switch (type) {
            case 0x01: t = "temperature"; break;
            case 0x02: t = "tds";         break;
            case 0x03: t = "battery";     break;
            case 0x04: t = "turbidity";   break;
          }
          if (t) { float p[] = { val }; mqtt.sendData(t, p, 1); }
        }
        rxState = WAIT_HDR;
      }
    }
  }
  mqtt.loop();
}