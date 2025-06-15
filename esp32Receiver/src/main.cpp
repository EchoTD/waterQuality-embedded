#include <Arduino.h>
#include <cstring>
#include "MQTTManager.h"

// LoRa
#define LORA_AUX_PIN     25
#define LINK_TIMEOUT_MS  30000
#define PING_SILENCE_MS  10000
#define PING_PERIOD_MS   2000

HardwareSerial lora(2);
constexpr uint32_t BAUD      = 9600;
constexpr uint8_t  HANDSHAKE = 0x55;
constexpr uint8_t  SOF       = 0xAA;

// MQTT
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
static MQTTManager mqttManager(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS);

// Types and variables
enum PktType : uint8_t {  PKT_TEMP  = 0x11, 
                          PKT_TDS   = 0x22,
                          PKT_BATT  = 0x44,
                          PKT_TURB  = 0x88};
const char* pktName(uint8_t t)
{
  switch (t) {
    case PKT_TEMP: return "temperature";
    case PKT_TDS:  return "tds";
    case PKT_BATT: return "battery";
    case PKT_TURB: return "turbidity";
    default:       return "unknown";
  }
}

static uint8_t  frame[7];
static uint8_t  idx          = 0;
static uint32_t lastByteMs   = 0;
static uint32_t lastGoodMs   = 0;
static uint32_t lastPingMs   = 0;

void coldStartRadio();
uint8_t crc8(const uint8_t *data, size_t len);

void setup()
{
  pinMode(LORA_AUX_PIN, INPUT);
  Serial.begin(115200);

  lora.begin(BAUD, SERIAL_8N1, 16, 17);
  lora.write(HANDSHAKE);
  Serial.println("[BOOT] RX → handshake sent");

  mqttManager.begin();
}

void loop()
{
  while (lora.available())
  {
    uint8_t b = lora.read();
    lastByteMs = millis();

    if (idx == 0) { if (b == SOF) frame[idx++] = b; }
    else
    {
      if (b == SOF) { idx = 1; frame[0] = SOF; continue; }
      frame[idx++] = b;

      if (idx == sizeof(frame))
      {
        if (frame[6] == crc8(frame + 1, 5)) {
          uint8_t type = frame[1];
          union { uint8_t b[4]; float f; } u;
          memcpy(u.b, frame + 2, 4);

          Serial.printf("[OK] 0x%02X  %.2f\n", type, u.f);

          float v[1] = { u.f };
          if (!mqttManager.sendData(pktName(type), v, 1))
            Serial.println("[MQTT] publish failed");

          lastGoodMs = millis();
        } else {
          Serial.println("[CRC] fail");
        }
        idx = 0;
      }
    }
  }
  if (idx && millis() - lastByteMs > 200) idx = 0;

  // Keep Alive
  if (millis() - lastByteMs > 20000 && millis() - lastPingMs > 2000 &&
      digitalRead(LORA_AUX_PIN) == LOW) {
    lora.write(HANDSHAKE);
    lastPingMs = millis();
    Serial.println("[PING] handshake");
  }

  // Watchdog
  if (millis() - lastGoodMs > LINK_TIMEOUT_MS) {
    Serial.println("[WDG] no good pkt → cold-start");
    coldStartRadio();
    lastGoodMs = millis();
  }

  // MQTT Loop
  mqttManager.loop();
}

// Helpers
void coldStartRadio() {
  lora.end();
  delay(20);
  lora.begin(BAUD, SERIAL_8N1, 16, 17);
  delay(20);
  lora.write(HANDSHAKE);
}

uint8_t crc8(const uint8_t *d, size_t n)
{
  uint8_t c = 0xFF;
  while (n--) { c ^= *d++; for (uint8_t i = 0; i < 8; ++i)
      c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1); }
  return c;
}