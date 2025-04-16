#include "SensorManager.h"
#include "Data.h"

SensorManager::SensorManager(): oneWire(TEMP_SENSOR_PIN),
                                tempSensor(&oneWire){
    _data.temperature    = 0.0f;
    _data.tempTemperature = 0.0f;
}

void SensorManager::begin() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    tempSensor.begin();
}

void SensorManager::update() {
    readTemperature();
}

SensorData SensorManager::getSensorData() {
    return _data;
}

void SensorManager::readTemperature() {
    tempSensor.requestTemperatures();
    _data.temperature = tempSensor.getTempCByIndex(0);
}