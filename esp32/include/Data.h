#ifndef DATA_H
#define DATA_H

#include <Arduino.h>

typedef struct {
    float temperature;      // in °C
    float tdsValue;         // ppm
    float battery;          // V
    float turbidity;      

} SensorData;

#endif