#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <BLE.h>

#define ONE_WIRE_BUS 15
#define MOISTURE_PIN 26

constexpr unsigned long MEASUREMENT_INTERVAL_MS = 60UL * 1000UL;
constexpr float LCD_OFF_LUX = 5.0F;
constexpr float LCD_ON_LUX = 10.0F;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
BH1750 lightMeter;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Nordic UART Service: the Node server discovers this notify characteristic
// and receives the newline-delimited JSON sensor readings.

class ServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer *server) override {
        Serial.println("BLE central connected");
    }

    void onDisconnect(BLEServer *server) override {
        Serial.println("BLE central disconnected");
        BLE.startAdvertising();
    }
};

BLEServiceUART bleUart(32, 20);
ServerCallbacks serverCallbacks;

unsigned long lastMeasurementAt = 0;
bool lcdIsOn = true;

void updateLcdPower(float lux) {
    if (lcdIsOn && lux <= LCD_OFF_LUX) {
        lcd.noBacklight();
        lcd.noDisplay();
        lcdIsOn = false;
    } else if (!lcdIsOn && lux >= LCD_ON_LUX) {
        lcd.display();
        lcd.backlight();
        lcdIsOn = true;
    }
}

void sendReading(float temperature, float moisture, float light) {
    if (!bleUart) {
        Serial.println("BLE central is not connected");
        return;
    }

    StaticJsonDocument<128> reading;
    reading["temperature"] = temperature;
    reading["moisture"] = moisture;
    reading["light"] = light;

    String json;
    serializeJson(reading, json);
    bleUart.println(json);
    bleUart.flush();
    Serial.println(json);
}

void measureAndSend() {
    sensors.requestTemperatures();
    const float temperature = sensors.getTempCByIndex(0);
    const float lux = lightMeter.readLightLevel();
    const int rawMoisture = analogRead(MOISTURE_PIN);
    const float moisturePercent = constrain(map(rawMoisture, 800, 350, 0, 100), 0, 100);

    updateLcdPower(lux);
    if (lcdIsOn) {
        lcd.setCursor(0, 0); lcd.print("Temp:  "); lcd.print(temperature, 1); lcd.print(" C  ");
        lcd.setCursor(0, 1); lcd.print("Moist: "); lcd.print(moisturePercent, 0); lcd.print(" %  ");
        lcd.setCursor(0, 2); lcd.print("Light: "); lcd.print(lux, 1); lcd.print(" lx ");
        lcd.setCursor(0, 3); lcd.print("BLE: SmartFarm  ");
    }

    sendReading(temperature, moisturePercent, lux);
}

void setup() {
    Serial.begin(115200);
    Wire.setSDA(4); Wire.setSCL(5); Wire.begin();
    sensors.begin();
    lightMeter.begin();

    lcd.init(); lcd.backlight();
    lcd.setCursor(0, 0); lcd.print("Pico Monitoring");

    BLE.begin("SmartFarm-Pico");

    BLEServer *server = BLE.server();
    server->setCallbacks(&serverCallbacks);
    server->addService(&bleUart);

    BLE.startAdvertising();
    Serial.println("BLE advertising started");
}

void loop() {
    const unsigned long now = millis();
    if (lastMeasurementAt == 0 || now - lastMeasurementAt >= MEASUREMENT_INTERVAL_MS) {
        lastMeasurementAt = now;
        measureAndSend();
    }
    delay(20);
}
