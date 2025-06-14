#include <Arduino.h>
#include "Pins.h"
#include "SensorManager.h"

// LoRa
HardwareSerial lora(2);
static bool rxReady = false;
constexpr uint32_t LORA_BAUD   = 9600;
static const uint8_t HANDSHAKE = 0x55;
constexpr uint8_t SOF          = 0xAA;   

enum PktType : uint8_t {
  PKT_TEMP = 0x01,
  PKT_TDS  = 0x02,
  PKT_BATT = 0x03,
  PKT_TURB = 0x04
};

// Timing Config
constexpr uint32_t SAMPLING_WINDOW_MS = 10000;
constexpr uint32_t IDLE_WINDOW_MS     = 10000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1;
constexpr uint32_t IDLE_POLL_MS       = 500;
constexpr uint64_t DEEP_SLEEP_SEC     = 30;

enum CycleState { SAMPLING, SENDING, IDLE };
CycleState currentState = SAMPLING;

// Object Inititaitons
SensorManager sensorManager;
void updateSampling();
void updateSending();
void updateIdle();
uint8_t crc8(const uint8_t *data, size_t len);
void sendValue(PktType type,float value);
void printHexBuf(const uint8_t *buf, size_t len, const char *label);

// Variables
float     sumTemperature = 0.0f;   uint16_t cntTemp = 0;
float     sumTDS         = 0.0f;   uint16_t cntTDS  = 0;
float     sumTurb        = 0.0f;   uint16_t cntTurb = 0;
uint32_t  stateStartMs    = 0;
uint32_t  lastSampleMs    = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  lora.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  Serial.println(F("LoRa-TX   ——   system initialised"));

  sensorManager.begin();
  stateStartMs  = lastSampleMs = millis();  
}

void loop() {

  if (!rxReady) {
    while (lora.available()) {
      if (lora.read() == HANDSHAKE) {
        rxReady = true;
        Serial.println(F("Handshake received ▶ start TX"));
      }
    }
    return;
  }
  
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

    if (!isnan(data.temperature)) { sumTemperature  += data.temperature; cntTemp++; }
    if (!isnan(data.tdsValue))    { sumTDS          += data.tdsValue;    cntTDS++;  }
    if (!isnan(data.turbidity))   { sumTurb         += data.turbidity;   cntTurb++; }

    lastSampleMs = now;
  }

  if (now - stateStartMs >= SAMPLING_WINDOW_MS) {
    currentState = SENDING;
    stateStartMs = now;
  }
}

void updateSending() {
  float avgTemp = cntTemp ? sumTemperature / cntTemp : NAN;
  float avgTDS  = cntTDS  ? sumTDS        / cntTDS  : NAN;
  float avgTurb = cntTurb ? sumTurb       / cntTurb : NAN;

  // Debug
  Serial.printf("[DBG] samples T=%u  TDS=%u  Turb=%u\n",
                cntTemp, cntTDS, cntTurb);
  Serial.printf("[DBG] averages T=%.2f °C  TDS=%.0f ppm  Turb=%.2f NTU\n",
                avgTemp, avgTDS, avgTurb);

  sendValue(PKT_TEMP, avgTemp);
  sendValue(PKT_TDS,  avgTDS);
  sendValue(PKT_TURB, avgTurb);

  // reset accumulators
  sumTemperature = sumTDS = sumTurb = 0.0f;
  cntTemp = cntTDS = cntTurb = 0;

  currentState = IDLE;
  stateStartMs = millis();
}

void updateIdle() 
{
  if (millis() - stateStartMs < IDLE_WINDOW_MS) {
    vTaskDelay(IDLE_POLL_MS / portTICK_PERIOD_MS);
    return;
  }
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_SEC * 1000000ULL);
  Serial.println(F("💤 deep-sleep…"));
  Serial.flush();
  esp_deep_sleep_start();
}

void sendValue(PktType type, float val) {
  uint8_t frame[1 /*SOF*/ + 1 /*type*/ + 4 /*float*/];
  frame[0] = type;
  memcpy(frame + 1, &val, sizeof(val));
  uint8_t crc = crc8(frame, sizeof(frame));

  lora.write(SOF);
  lora.write(frame, sizeof(frame));
  lora.write(crc);
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