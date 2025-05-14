#include <Arduino.h>
#include "SensorManager.h"
#include "MQTTManager.h"
/* #include "TinyGPSPlus.h" */

// Temp?
const char* mqttServer = "172.34.6.157";
const int mqttPort = 1883;

// Object inititaitons
SensorManager sensorManager;
MQTTManager mqttManager(mqttServer, mqttPort);

// State Definitions
enum CycleState { SAMPLING, SENDING, IDLE };
CycleState currentState = SAMPLING;

// GPS Config
/* static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); */

// LoRa Config
HardwareSerial lora(1);
const uint8_t HEADER = 0xAA;   

const unsigned long samplingDuration = 5000;
const unsigned long idleDuration     = 5000;
const unsigned long sampleInterval   = 1;
unsigned long stateStartTime = 0;
unsigned long lastSampleTime = 0;

float sumTemperature = 0.0;
int sampleCount = 0;
float averageTemperature = 0.0;

void updateSampling();
void updateSending();
void updateIdle();
void sendFloat(float f);

void setup() {
  //gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  lora.begin(9600, SERIAL_8N1, 4, 2);
  
  Serial.begin(115200);
  sensorManager.begin();
  //mqttManager.begin();
  Serial.println("System Initialized.");

  currentState = SAMPLING;
  stateStartTime = millis();
  lastSampleTime = millis();
}

void loop() {
  sensorManager.update();
  float temp = sensorManager.getSensorData().temperature;

  sendFloat(temp);
  
  
  /* // 1. Feed incoming bytes to TinyGPS++
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // 2. Once every second, print the latest data
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    if (gps.location.isValid()) {
      Serial.print(F("LAT=")); Serial.print(gps.location.lat(), 6);
      Serial.print(F("  LON=")); Serial.print(gps.location.lng(), 6);
      Serial.print(F("  SAT=")); Serial.print(gps.satellites.value());
      Serial.print(F("  HDOP=")); Serial.println(gps.hdop.hdop());
    } else {
      Serial.println(F("No fix yet…"));
    }
  } */

  /* mqttManager.loop();

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
      stateStartTime = millis();
      break;
  } */
}

void sendFloat(float f){
  lora.write(HEADER);                        // 1-byte marker
  lora.write(reinterpret_cast<uint8_t*>(&f), sizeof(f));
}

void updateSampling() {
  unsigned long now = millis();
  
  if (now - lastSampleTime >= sampleInterval) {
    sensorManager.update();
    SensorData data = sensorManager.getSensorData();
    if (!isnan(data.temperature)) {
      sumTemperature += data.temperature;
      Serial.print(data.temperature);
      sampleCount++;
      Serial.print("\t");
      Serial.println(sampleCount);
    } else {
      Serial.println("Warning: Invalid sensor reading encountered!");
    }
    lastSampleTime = now;
  }
  
  if (now - stateStartTime >= samplingDuration) {
    if (sampleCount > 0) {
      averageTemperature = sumTemperature / sampleCount;
    } else {
      Serial.println("Error: No valid sensor data collected during sampling.");
      averageTemperature = 0.0;
    }
    currentState = SENDING;
    stateStartTime = now;
  }
}

void updateSending() {
  Serial.print("Sending average temperature: ");
  Serial.println(averageTemperature);

  bool success = mqttManager.sendData("temperature", &averageTemperature, 1);
  if (success) {
    Serial.println("Data sent successfully.");
  } else {
    Serial.println("Failed to send data. Will retry in next cycle.");
  }
  currentState = IDLE;
  stateStartTime = millis();
}

void updateIdle() {
  if (millis() - stateStartTime >= idleDuration) {
    // Reset accumulated data
    sumTemperature = 0.0;
    sampleCount = 0;
    averageTemperature = 0.0;
    currentState = SAMPLING;
    stateStartTime = millis();
    Serial.println("Starting a new sampling cycle.");
  }
}
