#ifndef DATA_H
#define DATA_H

#include <Arduino.h>

static const uint8_t TEMP_SENSOR_PIN = 4;

typedef struct {
    float temperature;      // in °C
    float tempTemperature;
} SensorData;

#endif