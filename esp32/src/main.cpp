#include <Arduino.h>
#include <cstring>
#include "Pins.h"
#include "SensorManager.h"

HardwareSerial lora(2);
static bool rxReady = false;
constexpr uint32_t LORA_BAUD      = 9600;
static const uint8_t HANDSHAKE    = 0x55;
static const uint8_t SOF          = 0xAA;

enum PktType : uint8_t { PKT_TEMP, PKT_TDS, PKT_BATT, PKT_TURB};

constexpr uint32_t SAMPLING_WINDOW_MS = 3000;
constexpr uint32_t IDLE_WINDOW_MS     = 2000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1;
constexpr uint32_t MIN_TX_GAP_MS      = 60;

enum CycleState { SAMPLING, SENDING, IDLE };
CycleState currentState = SAMPLING;

SensorManager sensorManager;
void updateSampling();
void updateSending();
void updateIdle();
uint8_t crc8(const uint8_t *data, size_t len);
void    sendValue(PktType type, float value);

float   sumTemperature = 0; uint16_t cntTemp = 0;
float   sumTDS = 0;         uint16_t cntTDS  = 0;
float   sumTurb = 0;        uint16_t cntTurb = 0;
uint32_t stateStartMs = 0, lastSampleMs = 0;

void setup() {
  Serial.begin(115200);
  lora.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  sensorManager.begin();
  stateStartMs = lastSampleMs = millis();
}

void loop() {
  if (!rxReady) {
    while (lora.available())
      if (lora.read() == HANDSHAKE) { rxReady = true; }
    return;
  }
  switch (currentState) {
    case SAMPLING: updateSampling(); break;
    case SENDING:  updateSending();  break;
    case IDLE:     updateIdle();     break;
  }
}

void updateSampling() {
  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    sensorManager.update();
    auto d = sensorManager.getSensorData();
    if (!isnan(d.temperature)) { sumTemperature += d.temperature; cntTemp++; }
    if (!isnan(d.tdsValue))    { sumTDS        += d.tdsValue;    cntTDS++;  }
    if (!isnan(d.turbidity))   { sumTurb       += d.turbidity;   cntTurb++; }
    lastSampleMs = now;
  }
  if (now - stateStartMs >= SAMPLING_WINDOW_MS) { currentState = SENDING; stateStartMs = now; }
}

void updateSending() {
  float avgT   = cntTemp ? sumTemperature / cntTemp : NAN;
  float avgTDS = cntTDS  ? sumTDS        / cntTDS  : NAN;
  float avgTb  = cntTurb ? sumTurb       / cntTurb : NAN;

  sendValue(PKT_TEMP, avgT);
  sendValue(PKT_TDS,  avgTDS);
  sendValue(PKT_TURB, avgTb);

  sumTemperature = sumTDS = sumTurb = 0;
  cntTemp = cntTDS = cntTurb = 0;
  currentState = IDLE; stateStartMs = millis();
}

void updateIdle() {
  if (millis() - stateStartMs >= IDLE_WINDOW_MS) { currentState = SAMPLING; stateStartMs = millis(); }
}

uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

void sendValue(PktType type, float value) {
  static uint32_t lastTx = 0;
  while (millis() - lastTx < MIN_TX_GAP_MS);
  lastTx = millis();

  union { float f; uint8_t b[4]; } u; u.f = value;
  uint8_t payload[5];                      // type + float
  payload[0] = type;
  memcpy(payload + 1, u.b, 4);
  uint8_t crc = crc8(payload, 5);

  lora.write(SOF);
  lora.write(payload, 5);
  lora.write(crc);
}