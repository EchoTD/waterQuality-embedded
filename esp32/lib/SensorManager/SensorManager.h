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

    //TinyGPSPlus gps;
    //HardwareSerial gpsSerial(2);

    void readTemperature();
};

#endif