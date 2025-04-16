#include "MQTTManager.h"
#include "WiFiManager.h"
#include <ArduinoJson.h>

MQTTManager::MQTTManager(const char* mqttServer, int mqttPort, 
                         const char* mqttUser, const char* mqttPassword)
  : _mqttServer(mqttServer), _mqttPort(mqttPort),
    _mqttUser(mqttUser), _mqttPassword(mqttPassword),
    _mqttClient(_wifiClient) {}

void MQTTManager::begin() {
  _prefs.begin("wifi-creds");

  if (!connectWiFi()) {
    startAPMode();
  }
  _mqttClient.setServer(_mqttServer, _mqttPort);
  connectMQTT();
}

bool MQTTManager::sendData(const char* sensorType, float data[], int dataSize) {
  if (!_mqttClient.connected()) connectMQTT();
  
  StaticJsonDocument<256> doc;
  JsonArray dataArray = doc.createNestedArray(sensorType);
  for (int i = 0; i < dataSize; i++) dataArray.add(data[i]);
  doc["type"] = sensorType;

  char payload[256];
  serializeJson(doc, payload);

  char topic[32];
  snprintf(topic, 32, "sensors/%s", sensorType);
  return _mqttClient.publish(topic, payload);
}

void MQTTManager::loop() {
  if (!_mqttClient.connected()) connectMQTT();
  _mqttClient.loop();
}

bool MQTTManager::connectWiFi() {
  String ssid = _prefs.getString("ssid", "");
  String password = _prefs.getString("password", "");

  if (ssid.isEmpty() || password.isEmpty()) {
    return false;
  }

  WiFi.begin(ssid.c_str(), password.c_str());
  for (int i = 0; i < 20; i++) {  // 10-second timeout
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(500);
  }
  return false;
}

void MQTTManager::startAPMode() {
  WiFiManager wifiManager;

  if (!wifiManager.startConfigPortal("ESP32-WQ-Sensor")) {
    Serial.println("Failed to configure WiFi. Restarting...");
    ESP.restart();
  }

  _prefs.putString("ssid", WiFi.SSID());
  _prefs.putString("password", WiFi.psk());
  Serial.println("WiFi credentials saved.");
}

void MQTTManager::connectMQTT() {
  while (!_mqttClient.connected()) {
    if (_mqttClient.connect("ESP32Client", _mqttUser, _mqttPassword)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT failed (rc=");
      Serial.print(_mqttClient.state());
      Serial.println("). Retrying in 5s...");
      delay(5000);
    }
  }
}