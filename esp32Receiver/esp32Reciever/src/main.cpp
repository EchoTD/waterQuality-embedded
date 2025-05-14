/* LoRa_RX_Binary.ino ------------------------------------------------------ */
#include <Arduino.h>

HardwareSerial lora(2);                      // TX = GPIO 2, RX = GPIO 4
const uint8_t HEADER = 0xAA;   

void setup() {
  Serial.begin(115200);
  lora.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println(F("LoRa RX (binary float)"));
}

void loop()
{
  static bool awaitingHeader = true;
  static uint8_t buf[4];
  static uint8_t idx = 0;

  while (lora.available())
  {
    uint8_t b = lora.read();

    if (awaitingHeader)
    {
      if (b == HEADER) { awaitingHeader = false; idx = 0; }
      continue;                              // discard until we see header
    }

    buf[idx++] = b;

    if (idx == sizeof(buf))                  // got full float
    {
      float temp;
      memcpy(&temp, buf, sizeof(temp));

      Serial.print(F("Got ← ")); Serial.println(temp, 2);

      awaitingHeader = true;                 // look for next packet
    }
  }
}
