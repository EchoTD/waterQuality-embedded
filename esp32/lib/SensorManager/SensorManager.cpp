#include "SensorManager.h"
#include "Data.h"
#include "Pins.h"

SensorManager::SensorManager(): oneWire(TEMP_SENSOR_PIN),
                                tempSensor(&oneWire){
    _data.temperature=  0.0f;
    _data.tdsValue=     0.0f;
    _data.battery=      0.0f;
    _data.turbidity=    0.0f;

    ntu = 0.0f;
    calibrationFactor = 0.0f;
}

void SensorManager::begin() {
    pinMode(TURB_PIN, INPUT_PULLUP);
    pinMode(TDS_PIN, INPUT);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    tempSensor.begin();

    calibrationFactor = 0.000185f;
}

void SensorManager::update() {
    readTemperature();
    readTDS();
    readTurbidity();
}

SensorData SensorManager::getSensorData() {
    return _data;
}

void SensorManager::readTemperature() {
    tempSensor.requestTemperatures();
    _data.temperature = tempSensor.getTempCByIndex(0);
}

void SensorManager::readTDS() {
    int adcValue = analogRead(TDS_PIN);
    float voltage = (adcValue / 4095.0f) * 3.3f; 
    float conductivity = voltage / calibrationFactor;
    float temperatureCoefficient = 1.0f + (0.02f * (_data.temperature - 25.0f));
    float tds = conductivity * temperatureCoefficient * 0.5f;
    _data.tdsValue = tds;
}

void SensorManager::readTurbidity() {
    int adcValue = analogRead(TURB_PIN);
    float voltage = (adcValue / 4095.0f) * 5.0f;

    if (voltage < 2.5f) {
        ntu = 3000.0f;
    } else {
        ntu = (-1120.4f * voltage * voltage) + (5742.3f * voltage) - 4352.9f;
        if (ntu < 0) ntu = 0.0f;
    }
    _data.turbidity = ntu;
}