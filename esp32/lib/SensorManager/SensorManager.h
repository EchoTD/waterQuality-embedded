#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include "Data.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "TinyGPSPlus.h"

class SensorManager {
public:
    SensorManager();
    void begin();
    void update();
    SensorData getSensorData();

private:
    SensorData _data;

    OneWire oneWire;
    DallasTemperature tempSensor;

    void readTDS();
    void readTurbidity();
    void readTemperature();

    float ntu;
    float calibrationFactor;
};

#endif