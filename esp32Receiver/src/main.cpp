#include <Arduino.h>
#include "MQTTManager.h"   // enable if you actually want to push to MQTT

HardwareSerial lora(2);
constexpr uint32_t BAUD   = 9600;
constexpr uint8_t  HEADER = 0xAA;

const char* mqttServer = "172.34.6.157";
const int mqttPort = 1883;
MQTTManager mqtt(mqttServer, mqttPort);

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
  lora.begin(BAUD, SERIAL_8N1, 16, 17);
  mqtt.begin();  // if using MQTTManager
  Serial.println(F("LoRa RX ready"));
}

void loop() {
  /* // 1) show how many bytes are in RX buffer
  int avail = lora.available();
  if (avail > 0) {
    Serial.printf("RX buffer has %d byte(s)\n", avail);

    // 2) if we have at least full packet, read it all at once
    if (avail >= 6) {
      uint8_t buf[6];
      int len = lora.readBytes(buf, sizeof(buf));
      Serial.printf("readBytes returned %d\n", len);
      printHexBuf(buf, len, "RAW PACKET: ");

      // 3) check header
      if (buf[0] != HEADER) {
        Serial.printf("  → BAD HEADER 0x%02X  discarding one byte\n", buf[0]);
        // flush one byte so next loop can realign
        lora.read();
        return;
      }

      // 4) extract & print payload and CRC
      float temp;
      memcpy(&temp, buf + 1, sizeof(temp));
      uint8_t crcRx   = buf[5];
      uint8_t crcCalc = crc8(buf + 1, sizeof(temp));

      Serial.printf("  → unpacked float=%.2f, CRC recv=0x%02X, CRC calc=0x%02X\n",
                    temp, crcRx, crcCalc);

      // 5) final CRC check
      if (crcRx == crcCalc) {
        Serial.print(F("  → CRC OK, value="));
        Serial.println(temp, 2);
      } else {
        Serial.println(F("  → CRC MISMATCH!"));
      }
    }
  } */
  if (lora.available() < 6) return;

  uint8_t buf[6];
  int len = lora.readBytes(buf, sizeof(buf));
  if (len != sizeof(buf)) return;

  // check header
  if (buf[0] != HEADER) {
    // drop first byte and realign on next loop
    lora.read();  
    return;
  }

  // extract float + CRC
  float temp;
  memcpy(&temp, buf + 1, sizeof(temp));
  uint8_t crcRx   = buf[5];
  uint8_t crcCalc = crc8(buf + 1, sizeof(temp));

  if (crcRx == crcCalc) {
    Serial.print(F("Got ← ")); Serial.println(temp, 2);
    float payload[] = { temp };
    mqtt.sendData("temperature", payload, 1);
  } else {
    Serial.println(F("CRC fail"));
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