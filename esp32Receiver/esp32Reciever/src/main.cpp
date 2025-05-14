#include <Arduino.h>
#include "MQTTManager.h"

// LoRa
HardwareSerial lora(2);
constexpr uint32_t LORA_BAUD   = 9600;
constexpr uint8_t  PKT_HEADER  = 0xAA;

MQTTManager mqtt("172.34.6.157", 1883);

uint8_t crc8(const uint8_t *d, size_t n);

void setup() {
  Serial.begin(115200);
  lora.begin(LORA_BAUD, SERIAL_8N1, 16, 17);
  
  //mqtt.begin();
  Serial.println(F("RX ready"));
}

void loop() {
  static uint8_t buf[5];
  static uint8_t idx  = 0;
  static bool    sync = true;

  while (lora.available()) {
    uint8_t b = lora.read();

    if (sync) {
      if (b == PKT_HEADER) { sync = false; idx = 0; }
      continue;
    }

    buf[idx++] = b;
    if (idx == sizeof(buf)) {
      float   temp;  memcpy(&temp, buf, 4);
      uint8_t crcRx = buf[4];

      if (crcRx == crc8(buf, 4)) {
        Serial.print(F("OK  ←  ")); Serial.println(temp, 2);

        float payload[] = { temp };
        // mqtt.sendData("temperature", payload, 1);
      } else {
        Serial.println(F("CRC fail"));
      }

      sync = true;
    }
  }

  // mqtt.loop();
}

uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0xFF;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; ++i)
      c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1);
  }
  return c;
}
