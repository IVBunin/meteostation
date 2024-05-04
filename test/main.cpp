#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DHT.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>


const char* ssid = "WiFi_01";
const char* password = "08092004";

const int DHTPIN = 13;
const int DHTTYPE = DHT22;

Adafruit_BMP280 bme;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);

  if (!bme.begin()) {
    Serial.println("Could not find BME280 sensor, check wiring!");
    while (1);
  }

  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();

  float temperatureBME = bme.readTemperature();
  float humidityBME = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;

  float temperatureDHT = dht.readTemperature();
  float humidityDHT = dht.readHumidity();

  int batteryLevel = analogRead(A13) * 100 / 4095; // Пример для ESP32-S2

  server.handleClient();

  delay(1000);
}

void handleRoot() {
  float temperatureBME = bme.readTemperature();
  float humidityBME = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;

  float temperatureDHT = dht.readTemperature();
  float humidityDHT = dht.readHumidity();

  int batteryLevel = analogRead(A13) * 100 / 4095; // Пример для ESP32-S2

  String message = "<html><body>";
  message += "<h1>Weather Station</h1>";
  message += "<p>BME280 Sensor Data:</p>";
  message += "<p>Temperature: " + String(temperatureBME, 2) + " °C</p>";
  message += "<p>Humidity: " + String(humidityBME, 2) + " %</p>";
  message += "<p>Pressure: " + String(pressure, 2) + " hPa</p>";
  message += "<p>DHT22 Sensor Data:</p>";
  message += "<p>Temperature: " + String(temperatureDHT, 2) + " °C</p>";
  message += "<p>Humidity: " + String(humidityDHT, 2) + " %</p>";
  message += "<p>Battery Level: " + String(batteryLevel) + " %</p>";
  message += "</body></html>";

  server.send(200, "text/html", message);
}
