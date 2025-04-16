#ifndef HTTPMANAGER_H
#define HTTPMANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Data.h>

class HTTPManager {
public:
    HTTPManager();
    HTTPClient http;
    void setup(const char* ssid, const char* password);
    void sendData(const char* serverName, float data);
    void echoPacket(const char* serverName, float data);

private:
    SensorData _data;
};

#endif