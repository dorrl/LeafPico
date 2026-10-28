#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <BLE.h>

// Arduino-Pico core
#include <pico/stdlib.h>

#define ONE_WIRE_BUS 15
#define MOISTURE_PIN 26

constexpr unsigned long MEASUREMENT_INTERVAL_MS = 60UL * 1000UL;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 10UL * 1000UL;
constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 5UL * 1000UL;
constexpr uint8_t MAX_CONSECUTIVE_HEARTBEAT_FAILURES = 3;
constexpr unsigned long ADVERTISING_RESTART_DELAY_MS = 500UL;

constexpr float LCD_OFF_LUX = 5.0F;
constexpr float LCD_ON_LUX = 10.0F;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
BH1750 lightMeter;
LiquidCrystal_I2C lcd(0x27, 20, 4);

BLEServiceUART bleUart(32, 20);

bool advertisingRestartPending = false;
unsigned long advertisingRestartAt = 0;

bool lcdIsOn = true;
bool heartbeatStarted = false;
bool heartbeatAwaitingResponse = false;
uint8_t consecutiveHeartbeatFailures = 0;

unsigned long lastMeasurementAt = 0;
unsigned long lastHeartbeatSentAt = 0;
unsigned long heartbeatRequestSentAt = 0;

char heartbeatLineBuffer[32] = {};
size_t heartbeatLineLength = 0;
bool discardHeartbeatLine = false;

// BLE callbacks --------------------------------------------------------------

class ServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer *server) override {
        Serial.println("BLE central connected");

        const unsigned long now = millis();
        heartbeatStarted = true;
        heartbeatAwaitingResponse = false;
        consecutiveHeartbeatFailures = 0;
        lastHeartbeatSentAt = now;
        heartbeatRequestSentAt = 0;
        heartbeatLineLength = 0;
        discardHeartbeatLine = false;

        advertisingRestartPending = false;
    }

    void onDisconnect(BLEServer *server) override {
        Serial.println("BLE central disconnected");

        // A disconnected BLE link is not a heartbeat failure. Let the server
        // reconnect and begin a fresh heartbeat session without rebooting.
        heartbeatStarted = false;
        heartbeatAwaitingResponse = false;
        consecutiveHeartbeatFailures = 0;
        heartbeatLineLength = 0;
        discardHeartbeatLine = false;

        advertisingRestartPending = true;
        advertisingRestartAt =
            millis() + ADVERTISING_RESTART_DELAY_MS;
    }
};

ServerCallbacks serverCallbacks;

// LCD ------------------------------------------------------------------------

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

// Sensor readings ------------------------------------------------------------

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

    const float moisturePercent = constrain(
        map(rawMoisture, 800, 350, 0, 100),
        0,
        100
    );

    updateLcdPower(lux);

    if (lcdIsOn) {
        lcd.clear();

        lcd.setCursor(0, 0);
        lcd.print("Leaf Pico");

        lcd.setCursor(0, 1);
        lcd.print("Temp:  ");
        lcd.print(temperature, 1);
        lcd.print(" C  ");

        lcd.setCursor(0, 2);
        lcd.print("Moist: ");
        lcd.print(moisturePercent, 0);
        lcd.print(" %  ");

        lcd.setCursor(0, 3);
        lcd.print("Light: ");
        lcd.print(lux, 1);
        lcd.print(" lx ");
    }

    sendReading(temperature, moisturePercent, lux);
}

// Heartbeat ------------------------------------------------------------------

// Server protocol:
// Pico -> server: "PING\n"
// Server -> Pico: "PONG\n"
//
// BLEServiceUART must expose received UART data through available()/read().
// If the installed BLE library uses different methods, adapt this function
// to that library's documented UART RX API.

void processHeartbeatInput() {
    if (!heartbeatStarted || !bleUart) {
        return;
    }

    while (bleUart.available() > 0) {
        const int incoming = bleUart.read();

        if (incoming < 0) {
            break;
        }

        const char c = static_cast<char>(incoming);

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (!discardHeartbeatLine) {
                heartbeatLineBuffer[heartbeatLineLength] = '\0';

                if (strcmp(heartbeatLineBuffer, "PONG") == 0) {
                    // Any valid PONG proves the link is alive and clears the
                    // consecutive-failure counter.
                    consecutiveHeartbeatFailures = 0;
                    heartbeatAwaitingResponse = false;
                    Serial.println("Heartbeat response received");
                }
            }

            heartbeatLineLength = 0;
            discardHeartbeatLine = false;
            continue;
        }

        if (discardHeartbeatLine) {
            continue;
        }

        if (heartbeatLineLength < sizeof(heartbeatLineBuffer) - 1) {
            heartbeatLineBuffer[heartbeatLineLength++] = c;
        } else {
            // Ignore the entire oversized line until its newline delimiter.
            heartbeatLineLength = 0;
            discardHeartbeatLine = true;
        }
    }
}

void sendHeartbeat() {
    if (!bleUart || !heartbeatStarted) {
        return;
    }

    const unsigned long now = millis();

    // Never send another PING while the previous request is still pending.
    if (heartbeatAwaitingResponse ||
        now - lastHeartbeatSentAt < HEARTBEAT_INTERVAL_MS) {
        return;
    }

    lastHeartbeatSentAt = now;
    heartbeatRequestSentAt = now;
    heartbeatAwaitingResponse = true;

    bleUart.println("PING");
    bleUart.flush();

    Serial.println("Heartbeat PING sent");
}

void checkHeartbeatTimeout() {
    if (!bleUart || !heartbeatStarted || !heartbeatAwaitingResponse) {
        return;
    }

    const unsigned long now = millis();
    if (now - heartbeatRequestSentAt < HEARTBEAT_TIMEOUT_MS) {
        return;
    }

    heartbeatAwaitingResponse = false;
    ++consecutiveHeartbeatFailures;

    Serial.print("Heartbeat response timeout (");
    Serial.print(consecutiveHeartbeatFailures);
    Serial.print("/");
    Serial.print(MAX_CONSECUTIVE_HEARTBEAT_FAILURES);
    Serial.println(")");

    if (consecutiveHeartbeatFailures >= MAX_CONSECUTIVE_HEARTBEAT_FAILURES) {
        Serial.println("Three consecutive heartbeat failures; rebooting Pico");
        delay(100);
        rp2040.reboot();
    }
}

// Setup / loop ---------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();

    sensors.begin();
    lightMeter.begin();

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Pico Monitoring");

    BLE.begin("Leaf-Pico");

    BLEServer *server = BLE.server();
    server->setCallbacks(&serverCallbacks);
    server->addService(&bleUart);

    BLE.startAdvertising();

    Serial.println("BLE advertising started");
}

void loop() {
    const unsigned long now = millis();

    if (advertisingRestartPending &&
        static_cast<long>(now - advertisingRestartAt) >= 0) {
        advertisingRestartPending = false;

        BLE.startAdvertising();
        Serial.println("BLE advertising restarted");
    }

    processHeartbeatInput();
    sendHeartbeat();
    checkHeartbeatTimeout();

    if (lastMeasurementAt == 0 ||
        now - lastMeasurementAt >= MEASUREMENT_INTERVAL_MS) {
        lastMeasurementAt = now;
        measureAndSend();
    }

    delay(20);
}