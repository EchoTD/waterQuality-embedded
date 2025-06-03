#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h> 
#include "MQTTManager.h"

HardwareSerial lora(2);
constexpr uint32_t BAUD   = 9600;
static const uint8_t HANDSHAKE = 0x55;
static const uint8_t HEADER    = 0xAA;

enum RxState { WAIT_HDR, READ_PAYLOAD };
static RxState  rxState     = WAIT_HDR;
static uint8_t  buf5[5];
static uint8_t  bufIndex    = 0;

const char* mqttServer = "154.53.180.35";
const int mqttPort = 1883;
const char* mqttUser = "myuser";
const char* mqttPass = "12415500";
MQTTManager mqtt(mqttServer, mqttPort, mqttUser, mqttPass);

WiFiManager wm;

void printHexBuf(const uint8_t *buf, size_t len, const char *label);

uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  // try auto-connect for 10s
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(500); Serial.print('.');
  }

  if (WiFi.status() != WL_CONNECTED) {
    wm.setConfigPortalTimeout(120);
    if (!wm.startConfigPortal("ESP32-ECHO")) {
      ESP.restart();
    }
    Serial.println("Connected via portal, IP=" + WiFi.localIP().toString());

    // <— WRITE into Preferences here:
    Preferences prefs;
    prefs.begin("wifi-creds", false);
    prefs.putString("ssid",     WiFi.SSID());
    prefs.putString("password", WiFi.psk());
    prefs.end();
  }
  else {
    Serial.println("\nAuto-connected, IP=" + WiFi.localIP().toString());
  }

  lora.begin(BAUD, SERIAL_8N1, 16, 17);
  lora.write(HANDSHAKE);
  Serial.println(F("Sent handshake to TX"));

  mqtt.begin();
  Serial.println(F("LoRa RX ready"));
}

void loop() {
  while (lora.available()) {
    uint8_t b = lora.read();
    unsigned long t = millis();
    Serial.printf("[RX %lums] byte=0x%02X\n", t, b);

    if (rxState == WAIT_HDR) {
      if (b == HEADER) {
        rxState   = READ_PAYLOAD;
        bufIndex  = 0;
      }
    } else {  // READ_PAYLOAD
      buf5[bufIndex++] = b;
      if (bufIndex == sizeof(buf5)) {
        // unpack float
        float temp;
        memcpy(&temp, buf5, sizeof(temp));
        uint8_t crcRx   = buf5[4];
        uint8_t crcCalc = crc8(buf5, sizeof(temp));
        Serial.printf(
          "[RX %lums] unpack=%.2f  CRC recv=0x%02X calc=0x%02X %s\n",
          millis(), temp, crcRx, crcCalc,
          (crcRx==crcCalc) ? "OK" : "FAIL"
        );
        if (crcRx == crcCalc) {
          // only push to MQTT on good CRC
          float payload[] = { temp };
          mqtt.sendData("temperature", payload, 1);
        }
        rxState = WAIT_HDR;
      }
    }
  }

  mqtt.loop();
}

void printHexBuf(const uint8_t *buf, size_t len, const char *label) {
  Serial.print(label);
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}
