#ifndef DATA_H
#define DATA_H

#include <Arduino.h>

typedef struct {
    float temperature;      // in °C
    float tds;              // ppm
    float battery;          // V
    bool turbidityLow;      // 1 = Clear

} SensorData;

#endif