// ============================================================
//  ARCHIVED — Hardware Version 1: Seeed XIAO ESP32-C6
//  Board: xiao_esp32c6
//  Sensor pin: GPIO 2 (set via build flag -DSENSOR_PIN=2)
//  Transport: WiFi only → MQTT TLS (HiveMQ Cloud)
//  Archived: 2026-03-25
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// WiFi — tries each network in order, uses first one found
const char* WIFI_NETWORKS[][2] = {
  { "WookieBear",       "7737443630" },   // home
  { "airport", "HitekPrinting" },  // vacation home
};

// HiveMQ Cloud
const char* MQTT_HOST  = "ad21434501c84aad995bc5621bf77f15.s1.eu.hivemq.cloud";
const int   MQTT_PORT  = 8883;
const char* MQTT_USER  = "vacation-sensor";
const char* MQTT_PASS  = "6WJJY9C@3R7rY7d";
const char* MQTT_TOPIC = "vacation/cust1/il/temp/status";

#ifndef SENSOR_PIN
#define SENSOR_PIN 4
#endif
#define PUBLISH_INTERVAL 30000  // ms

OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);

WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastPublish = 0;

void connectWiFi() {
  int numNetworks = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);
  for (int i = 0; i < numNetworks; i++) {
    Serial.print("Trying ");
    Serial.print(WIFI_NETWORKS[i][0]);
    WiFi.begin(WIFI_NETWORKS[i][0], WIFI_NETWORKS[i][1]);
    for (int t = 0; t < 20; t++) {  // 10s timeout per network
      if (WiFi.status() == WL_CONNECTED) break;
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(" connected");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      return;
    }
    Serial.println(" failed");
    WiFi.disconnect();
  }
  Serial.println("ERROR: No WiFi networks reachable");
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to HiveMQ...");
    String clientId = "esp32-" + WiFi.macAddress();
    if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println(" connected");
    } else {
      Serial.print(" failed rc=");
      Serial.print(mqtt.state());
      Serial.println(", retrying in 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Vacation Home Temperature Monitor");

  sensors.begin();
  Serial.print("Sensors found: ");
  Serial.println(sensors.getDeviceCount());

  wifiClient.setInsecure();  // encrypts but skips cert verification
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(60);

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;

    sensors.requestTemperatures();
    float tempF = sensors.getTempFByIndex(0);
    float tempC = sensors.getTempCByIndex(0);

    if (tempC == DEVICE_DISCONNECTED_C) {
      Serial.println("ERROR: Sensor disconnected");
      return;
    }

    char payload[128];
    snprintf(payload, sizeof(payload),
      "{\"customerId\":\"cust1\",\"houseId\":\"il\",\"mode\":\"realtime\",\"temp_f\":%.1f,\"temp_c\":%.1f}",
      tempF, tempC);

    Serial.print("Publishing: ");
    Serial.println(payload);

    mqtt.publish(MQTT_TOPIC, payload);
  }
}
