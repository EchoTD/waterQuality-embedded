#include <Arduino.h>
#include <cstring>

HardwareSerial lora(2);
constexpr uint32_t BAUD       = 9600;
constexpr uint8_t  HANDSHAKE  = 0x55;
constexpr uint8_t  SOF        = 0xAA;

uint8_t crc8(const uint8_t *d, size_t n)
{
  uint8_t c = 0xFF;
  while (n--) { c ^= *d++; for (uint8_t i = 0; i < 8; ++i) c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1); }
  return c;
}

void setup()
{
  Serial.begin(115200);
  lora.begin(BAUD, SERIAL_8N1, 16, 17);
  lora.write(HANDSHAKE);
  Serial.println("[DBG] RX boot → handshake sent");
}

void loop()
{
  static uint8_t  frame[7];
  static uint8_t  idx          = 0;
  static uint32_t lastByte     = 0;   // any byte seen
  static uint32_t lastGoodPkt  = 0;   // crc-ok packet
  static uint32_t lastPing     = 0;

  while (lora.available())
  {
    uint8_t b = lora.read(); lastByte = millis();

    if (idx == 0) { if (b == SOF) frame[idx++] = b; }
    else
    {
      if (b == SOF) { idx = 1; frame[0] = SOF; continue; }
      frame[idx++] = b;

      if (idx == sizeof(frame))
      {
        if (frame[6] == crc8(frame + 1, 5)) {
          uint8_t type = frame[1];
          union { uint8_t b[4]; float f; } u; memcpy(u.b, frame + 2, 4);
          Serial.printf("[DBG] OK  type=0x%02X val=%.2f\n", type, u.f);
          lastGoodPkt = millis();
        } else {
          Serial.println("[DBG] CRC fail");
        }
        idx = 0;
      }
    }
  }

  if (idx && millis() - lastByte > 200) idx = 0;

  /* ping only if 6 s of radio silence */
  if (millis() - lastByte > 20000 && millis() - lastPing > 2000) {
    lora.write(HANDSHAKE);
    lastPing = millis();
    Serial.println("[DBG] handshake ping");
  }
}
