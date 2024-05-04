#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DHT.h>
#include <ArduinoJson.h>
const char* ssid = "esp";
const char* password = "08092004";

AsyncWebServer server(80);

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int irPin = 5;  // пин для инфракрасного датчика скорости вращения
const int discDiameter = 36;  // диаметр диска с отверстиями в мм
const int circleDiameter = 280; // диаметр окружности на которую воздействует ветер в мм
const int measurementInterval = 1000; // интервал измерения скорости ветра в миллисекундах

const int batteryPin = 3; // пин для считывания показаний с аккумулятора через внутренний АЦП
const float batteryDividerRatio = 2.062195781503515; // соотношение делителя напряжения
const float voltageReference = 3.3; // напряжение опорного источника ESP32 S2
const float ADCResolution = 4096; // разрешение АЦП

unsigned long lastMeasurementTime = 0; // время последнего измерения скорости ветра
volatile int rotationCount = 0; // количество оборотов диска с отверстиями

float temperature = 0;
float humidity = 0;
float speed = 0; // скорость ветра в м/с
float batteryVoltage = 0; // Напряжение с аккумулятора

void IRAM_ATTR handleInterrupt() {
  rotationCount++;
}

float calculateWindSpeed(int rotations) {
  // Переводим количество оборотов в обороты в секунду
  float rotationsPerSecond = rotations / (measurementInterval / 1000.0);
  
  // Считаем скорость ветра в м/с
  float windSpeed = 2 * PI * 28 * (rotationsPerSecond/60);
  
  return windSpeed;
}


void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  dht.begin();

  pinMode(irPin, INPUT_PULLUP); // устанавливаем пин для датчика оборотов в режим входа с подтяжкой
  attachInterrupt(digitalPinToInterrupt(irPin), handleInterrupt, RISING);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String page = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>Weather Info</title><style>.container {display: flex;flex-direction: column;align-items: center;height: 100vh;}.weather-info {width: 50%;text-align: center;padding: 20px;background-color: #f0f0f0;border-radius: 10px;box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);margin-bottom: 10px;}.humidity-bar, .temperature-bar, .wind-speed-bar, .battery-level-bar {width: 100%;height: 20px;background-color: #3498db;margin-top: 5px;}.temperature-bar {background-color: #e74c3c;}.wind-speed-bar {background-color: #27ae60;}.battery-level-bar {background-color: #f39c12;}</style></head><body><div class='container'><div class='weather-info'>Humidity: <span id='humidity'>" + String(humidity) + "</span>%<div class='humidity-bar' style='width:" + String(humidity) + "%;'></div></div><div class='weather-info'>Temperature: <span id='temperature'>" + String(temperature) + "</span>°C<div class='temperature-bar' style='width:" + String(map(temperature, -40, 40, 0, 100)) + "%;'></div></div><div class='weather-info'>Wind Speed: <span id='speed'>" + String(speed, 2) + "</span> m/s<div class='wind-speed-bar' style='width:" + String(map(speed, 0, 100, 0, 100)) + "%;'></div></div><div class='weather-info'>Battery Level: <span id='battery'>" + String(batteryVoltage, 2) + "</span>V<div class='battery-level-bar' style='width:" + String(map(batteryVoltage, 3.0, 4.2, 0, 100)) + "%;'></div></div></div><script>setInterval(function() {fetch('/data').then(response => response.json()).then(data => {document.getElementById('humidity').innerText = data.humidity;document.getElementById('temperature').innerText = data.temperature;document.getElementById('speed').innerText = data.speed;document.getElementById('battery').innerText = data.battery;document.querySelector('.humidity-bar').style.width = data.humidity + '%';document.querySelector('.temperature-bar').style.width = (data.temperature + 40) * (100 / 80) + '%';document.querySelector('.wind-speed-bar').style.width = data.speed + '%';document.querySelector('.battery-level-bar').style.width = data.battery + '%';});}, 5000);</script></body></html>";
    request->send(200, "text/html", page);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument jsonData(200);
    jsonData["humidity"] = (int)humidity;
    jsonData["temperature"] = temperature;
    jsonData["speed"] = speed;
    jsonData["battery"] = batteryVoltage;
    String jsonString;
    serializeJson(jsonData, jsonString);
    request->send(200, "application/json", jsonString);
  });

  server.begin();
}

void loop() {
  unsigned long currentTime = millis();

  // Если прошло достаточно времени для измерения скорости ветра
  if (currentTime - lastMeasurementTime >= measurementInterval) {
    // Получаем количество оборотов за измеренный интервал времени
    int rotations = rotationCount;

    // Сбрасываем количество оборотов
    rotationCount = 0;

    // Считаем скорость ветра
    speed = calculateWindSpeed(rotations);

    // Сохраняем текущее время для последующих измерений
    lastMeasurementTime = currentTime;
  }

  // Считываем данные с датчиков и аккумулятора
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  batteryVoltage = (analogRead(batteryPin) / (float)ADCResolution) * batteryDividerRatio * 1.3571; // преобразование значения АЦП в напряжение
  
  delay(100); // Короткая задержка для уменьшения нагрузки на процессор
}

