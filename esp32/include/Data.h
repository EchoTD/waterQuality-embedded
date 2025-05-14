#ifndef DATA_H
#define DATA_H

#include <Arduino.h>

static const uint8_t TEMP_SENSOR_PIN = 4;
/* static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600; */

typedef struct {
    float temperature;      // in °C
    float tempTemperature;
    //char gpsSerial;
} SensorData;

#endif