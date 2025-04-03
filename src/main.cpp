#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <Arduino.h>
#include <Servo.h>
#include <DHT.h>
#include <tuple>

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
  #include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
  #include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
  #include <WiFiS3.h>
#endif

WiFiSSLClient wifiClient;
MqttClient mqttClient(wifiClient);
Servo motor;
DHT dht(DHT_DIG_PIN, DHT11);

void safeMove(const int degrees) {
  motor.attach(MOTOR_DIG_PIN);
  motor.write(degrees);

  delay(250);

  motor.detach();
}

std::pair<float, float> getTempData() {
  float temperature = dht.readTemperature();
  const float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return std::pair<float, float>(0.0f, 0.0f);
  }

  return std::pair<float, float>(temperature, humidity);
}

void onMqttMessage(const int _) {
  while (mqttClient.available()) {
    String text = mqttClient.readString();

    Serial.print("Received message: ");
    Serial.println(text);

    if (text == String(0)) {
      digitalWrite(LED_BUILTIN, HIGH);
      safeMove(180);
    } else if (text == String(1)) {
      digitalWrite(LED_BUILTIN, LOW);
      safeMove(0);
    }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  dht.begin();

  safeMove(0);

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(SECRET_SSID);
  while (WiFi.begin(SECRET_SSID, SECRET_PASS) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  mqttClient.setUsernamePassword(MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(MQTT_BROKER);
  Serial.println(MQTT_BROKER_USERNAME);
  Serial.println(MQTT_BROKER_PASSWORD);

  if (!mqttClient.connect(MQTT_BROKER, MQTT_CLIENT_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (true);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(MQTT_DEFAULT_TOPIC);
  Serial.println();

  mqttClient.subscribe(MQTT_DEFAULT_TOPIC);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(MQTT_DEFAULT_TOPIC);
  Serial.println();
}

void loop() {
  mqttClient.poll();

  float t;
  float h;

  std::tie<float, float>(t, h) = getTempData();

  mqttClient.beginMessage("nikolaj/temperature");
  mqttClient.print(String(t) + ", " + String(h));
  mqttClient.endMessage();

  delay(100);
}
