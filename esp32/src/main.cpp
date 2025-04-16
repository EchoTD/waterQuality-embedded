#include <Arduino.h>
#include "SensorManager.h"
#include "MQTTManager.h"

const char* mqttServer = "172.34.6.157";
const int mqttPort = 1883;

SensorManager sensorManager;
MQTTManager mqttManager(mqttServer, mqttPort);

enum CycleState { SAMPLING, SENDING, IDLE };
CycleState currentState = SAMPLING;

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

void setup() {
  Serial.begin(115200);
  sensorManager.begin();
  mqttManager.begin();
  Serial.println("System Initialized.");

  currentState = SAMPLING;
  stateStartTime = millis();
  lastSampleTime = millis();
}

void loop() {
  mqttManager.loop();

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
  }
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
