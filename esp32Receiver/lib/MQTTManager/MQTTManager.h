#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <Preferences.h>

class MQTTManager {
public:
  MQTTManager(const char* mqttServer, int mqttPort, 
              const char* mqttUser = "", const char* mqttPassword = "");
  
  void begin();
  bool sendData(const char* sensorType, float data[], int dataSize);
  void loop();

private:
  const char* _mqttServer;
  int _mqttPort;
  const char* _mqttUser;
  const char* _mqttPassword;

  WiFiClient _wifiClient;
  PubSubClient _mqttClient;
  Preferences _prefs;             // For storing WiFi credentials

  void startAPMode();             // Launch AP for configuration
  boolean connectWiFi();             // Connect using stored credentials
  void connectMQTT();
};

#endif