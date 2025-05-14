#include <Arduino.h>
#include "SensorManager.h"

// LoRa
HardwareSerial lora(2);
constexpr uint32_t LORA_BAUD   = 9600;
constexpr uint8_t  PKT_HEADER  = 0xAA;

// Timing Config
constexpr uint32_t SAMPLING_WINDOW_MS = 5000;
constexpr uint32_t IDLE_WINDOW_MS     = 5000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1;


enum CycleState { SAMPLING, SENDING, IDLE };
CycleState currentState = SAMPLING;

// Object Inititaitons
SensorManager sensorManager;

// Variables
float     sumTemperature = 0.0f;
uint16_t  sampleCount    = 0;
uint32_t  stateStartMs   = 0;
uint32_t  lastSampleMs   = 0;

void updateSampling();
void updateSending();
void updateIdle();
uint8_t crc8(const uint8_t *data, size_t len);
void sendTemperature(float value);
void printHexBuf(const uint8_t *buf, size_t len, const char *label);

void setup() {
  Serial.begin(115200);
  lora.begin(LORA_BAUD, SERIAL_8N1, 16, 17);

  sensorManager.begin();

  Serial.println(F("LoRa-TX   ——   system initialised"));
  stateStartMs  = lastSampleMs = millis();
}

void loop() {
  /* sensorManager.update();
  float temp = sensorManager.getSensorData().temperature;

  // ——— build packet: [ HEADER | 4-byte float | 1-byte CRC ] ———
  uint8_t packet[6];
  packet[0] = PKT_HEADER;
  memcpy(packet + 1, &temp, sizeof(temp));
  packet[5] = crc8(packet + 1, sizeof(temp));

  Serial.print(F("TX → float="));
  Serial.print(temp, 2);
  Serial.print(F("  packet: "));
  printHexBuf(packet, sizeof(packet), "");

  // send in one go
  lora.write(packet, sizeof(packet));
  Serial.print(F("Sent → ")); Serial.println(temp, 2);

  delay(1000);                      // send once per second */
  
  switch (currentState) {
    case SAMPLING:
      updateSampling(); 
      break;
    case SENDING:
      updateSending();
      break;
    case IDLE:
      updateIdle();
      break;

    default:                        
      currentState = SAMPLING;
      stateStartMs = millis();
      break;
  }
}

void updateSampling() {
  const uint32_t now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    sensorManager.update();
    SensorData data = sensorManager.getSensorData();

    if (!isnan(data.temperature)){
      sumTemperature += data.temperature;
      sampleCount++;

      // Debug
      /* Serial.print(data.temperature);
      Serial.print("\t");
      Serial.println(sampleCount); */
    }
    lastSampleMs = now;
  }

  if (now - stateStartMs >= SAMPLING_WINDOW_MS) {
    currentState = SENDING;
    stateStartMs = now;
  }
}

void updateSending() {
  if (sampleCount == 0) {
    Serial.println(F("No samples, skipping send"));
  }
  else {
    const float avgTemp = sumTemperature / sampleCount;
    sendTemperature(avgTemp);

    // Debug
    /* Serial.print(F("Sent average temp: "));
    Serial.println(avgTemp, 2); */
  }

  sumTemperature = 0.0f;
  sampleCount    = 0;

  currentState = IDLE;
  stateStartMs = millis();
}

void updateIdle() {
  if (millis() - stateStartMs >= IDLE_WINDOW_MS) {
    currentState = SAMPLING;
    stateStartMs = millis();
  }
}

void sendTemperature(float value) {
  uint8_t payload[4];
  memcpy(payload, &value, sizeof(value));          // little-endian

  const uint8_t crc = crc8(payload, sizeof(payload));

  lora.write(PKT_HEADER);                          // 1 B header
  lora.write(payload, sizeof(payload));            // 4 B float
  lora.write(crc);                                 // 1 B CRC
}

uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
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