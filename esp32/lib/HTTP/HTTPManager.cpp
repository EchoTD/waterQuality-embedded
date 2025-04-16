#include "HTTPManager.h"
#include "Data.h"

HTTPManager::HTTPManager() {
    _data.temperature    = 0.0f;
}

void HTTPManager::setup(const char* ssid, const char* password) {
    WiFi.begin(ssid, password);
    Serial.println("Connecting...");
    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("#");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
}

void HTTPManager::sendData(const char* serverName, float data) {
    if(WiFi.status() == WL_CONNECTED) {
        char url[128];
        sprintf(url, "%s/temperature?value=%.2f", serverName, _data.temperature);

        http.begin(url);
        http.addHeader("Content-Type", "application/json");

        String payload = "{\"temperature\": " + String(_data.temperature) + "}";
        int httpResponse = http.POST(payload);

        if (httpResponse > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponse);
        String response = http.getString();
        Serial.println("Response: " + response);
        } else {
        Serial.print("Error code: ");
        Serial.println(httpResponse);
        }
    http.end();
    } else {
        Serial.println("WiFi not connected");
    }
}

void HTTPManager::echoPacket(const char* serverName, float data) {
    if(WiFi.status() == WL_CONNECTED) {
        char url[128];
        sprintf(url, "%s/echo", serverName);

        http.begin(url);
        http.addHeader("Content-Type", "application/json");

        String payload = "{\"temperature\": " + String(data) + "}";
        int httpResponse = http.POST(payload);

        if (httpResponse > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponse);
        String response = http.getString();
        Serial.println("Response: " + response);
        } else {
        Serial.print("Error code: ");
        Serial.println(httpResponse);
        }
    http.end();
    } else {
        Serial.println("WiFi not connected");
    }
}