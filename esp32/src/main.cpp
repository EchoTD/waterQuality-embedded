#include <Arduino.h>
#include "SensorManager.h"
#include "MQTTManager.h"

const char* mqttServer = "mosquitto";
const int mqttPort = 1883;

// Instantiate sensor and MQTT managers.
SensorManager sensorManager;
MQTTManager mqttManager(mqttServer, mqttPort);

// Define the cycle states.
enum CycleState { SAMPLING, SENDING, IDLE };
CycleState currentState = SAMPLING;

// Timing settings (in milliseconds)
const unsigned long samplingDuration = 5000;
const unsigned long idleDuration     = 5000;
const unsigned long sampleInterval   = 1;
unsigned long stateStartTime = 0;
unsigned long lastSampleTime = 0;

// Variables for accumulating sensor data.
float sumTemperature = 0.0;
int sampleCount = 0;
float averageTemperature = 0.0;

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
      // Reset to the sampling state if an undefined state is encountered.
      currentState = SAMPLING;
      stateStartTime = millis();
      break;
  }
}

void updateSampling() {
  unsigned long now = millis();
  
  // Sample at regular intervals.
  if (now - lastSampleTime >= sampleInterval) {
    sensorManager.update();
    SensorData data = sensorManager.getSensorData();
    if (!isnan(data.temperature)) {
      sumTemperature += data.temperature;
      Serial.println(data.temperature);
      sampleCount++;
    } else {
      Serial.println("Warning: Invalid sensor reading encountered!");
    }
    lastSampleTime = now;
  }

  // If the sampling phase is complete, compute the average and switch to the sending state.
  if (now - stateStartTime >= samplingDuration) {
    if (sampleCount > 0) {
      averageTemperature = sumTemperature / sampleCount;
    } else {
      Serial.println("Error: No valid sensor data collected during sampling.");
      averageTemperature = 0.0;  // Consider a safe default or a re-sampling approach.
    }
    currentState = SENDING;
    stateStartTime = now;  // Reset state start time for the SENDING phase.
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
    sumTemperature = 0.0;
    sampleCount = 0;
    averageTemperature = 0.0;
    currentState = SAMPLING;
    stateStartTime = millis();
    Serial.println("Starting a new sampling cycle.");
  }
}