#include <Arduino.h>
#include <Servo.h>
#include "HX711.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>

SoftwareSerial bt(7,6); // RX, TX (Bluetooth)

// ------------------- PINS -------------------
#define SERVO_PIN 9
#define BUTTON_PIN 2
#define BUZZER_PIN 8

#define LED_GREEN A1
#define LED_RED A2

#define PIR_PIN 11

// HX711
#define LOADCELL_DOUT_PIN 5
#define LOADCELL_SCK_PIN 4

// DHT
#define DHTPIN 3
#define DHTTYPE DHT11 // or DHT22 if you use 22

// LCD
LiquidCrystal_I2C lcd(0x27,16,2);

// ------------------- OBJECTS -------------------
Servo servo;
HX711 scale;
DHT dht(DHTPIN, DHTTYPE);

// ------------------- VARIABLES -------------------
float targetGrams = 15.0;
bool isFeeding = false;
bool stopRequest = false;

// Button toggle state
bool isOpen = false;
int buttonPrevState = HIGH;
unsigned long buttonPressStart = 0;
/*
// PIR cooldown: 1 hour to avoid repeated dispensing while animal stays
unsigned long lastPIRDispense = 0;
const unsigned long PIR_COOLDOWN_MS = 3600000UL; // 1 hour
*/
bool pirWasHigh = false;

// -------------------------------------------------
void beep(int t = 120) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(t);
    digitalWrite(BUZZER_PIN, LOW);
}

// -------------------------------------------------
void showStatus(const char* status, float t, float h) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(status);

    lcd.setCursor(0, 1);
    lcd.print(t, 1);
    lcd.print("C ");
    lcd.print(h, 0);
    lcd.print("%");
}

// -------------------------------------------------
void stopFeedingEarly() {
    servo.write(90); // close
    isOpen = false;
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    isFeeding = false;
    stopRequest = false; // consume stop request

    bt.println("Feeding stopped");
    Serial.println("Feeding stopped");
    beep(200);
}

// -------------------------------------------------
void dispenseFood(float t, float h) {
    if (isFeeding) return;

    // BLOCK if too hot
    if (t > 30.0) {
        showStatus("TEMP TOO HIGH", t, h);
        beep(300);
        delay(1000);
        return;
    }

    isFeeding = true;
    stopRequest = false; // clear any previous stop request

    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);

    showStatus("FEEDING", t, h);
    beep();

    servo.write(0); // open lid
    isOpen = true;
    delay(300);

    // measure until target reached or stop requested
    while (true) {
        // Poll Bluetooth inside feeding loop so 'S' works immediately
        while (bt.available()) {
            char bc = (char)bt.read();
            if (bc == '\r' || bc == '\n') continue;
            // accept both uppercase and lowercase S
            if (bc == 'S' || bc == 's') {
                bt.println("STOP requested (during feeding)");
                Serial.println("BT Stop received during feeding");
                stopRequest = true;
                // immediately stop feeding and return
                stopFeedingEarly();
                return;
            }
            // ignore other characters (optionally buffer if needed)
        }

        if (stopRequest) {
            Serial.println("Stop requested - aborting dispense");
            stopFeedingEarly();
            return;
        }

        float oneReading = scale.get_units();
        float avgReading = scale.get_units(10);

        Serial.print("one:\t");
        Serial.print(oneReading, 1);
        Serial.print("\tavg:\t");
        Serial.println(avgReading, 2);

        if (avgReading >= targetGrams) break;

        // allow manual stop by holding button
        if (digitalRead(BUTTON_PIN) == LOW) {
            unsigned long pressStart = millis();
            while (digitalRead(BUTTON_PIN) == LOW) {
                if (millis() - pressStart > 800) {
                    Serial.println("Button long-press detected during feeding - stopping");
                    stopFeedingEarly();
                    return;
                }
                // also poll BT while waiting on button to avoid missing 'S'
                if (bt.available()) {
                    char bc = (char)bt.read();
                    if (bc == '\r' || bc == '\n') continue;
                    if (bc == 'S' || bc == 's') {
                        bt.println("STOP requested (during feeding)");
                        Serial.println("BT Stop received during feeding");
                        stopRequest = true;
                        stopFeedingEarly();
                        return;
                    }
                }
                delay(10);
            }
        }

        delay(200);
    }

    servo.write(90); // close
    isOpen = false;
    beep();

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);

    isFeeding = false;
    delay(800);
}

void handleBluetooth() {
    if (!bt.available()) return;

    // Read a small command; ignore CR/LF
    String cmd = "";
    while (bt.available()) {
        char c = (char)bt.read();
        if (c == '\r' || c == '\n') continue;
        cmd += c;
        delay(5);
    }
    if (cmd.length() == 0) return;

    // Use first char, uppercase
    char key = toupper((unsigned char)cmd.charAt(0));

    if (key == 'F') {
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        if (isnan(t) || isnan(h)) {
            bt.println("DHT error");
            return;
        }

        bt.println("Feeding...");
        stopRequest = false;
        dispenseFood(t, h);
    }
    else if (key == 'S') {
        // Close cap immediately and request stop of any feeding
        bt.println("STOP requested");
        stopRequest = true;

        if (isFeeding) {
            // If currently feeding, stop and close via stopFeedingEarly()
            stopFeedingEarly();
        } else {
            // Not feeding: just ensure cap closed and update indicators
            servo.write(90); // close cap
            isOpen = false;
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(LED_RED, HIGH);
            bt.println("Cap closed");
            Serial.println("Cap closed (Bluetooth S)");
        }
    } else {
        bt.print("Unknown cmd: ");
        bt.println(cmd);
        Serial.print("Unknown BT cmd: ");
        Serial.println(cmd);
    }
}

// -------------------------------------------------
void setup() {
    Serial.begin(57600);
    bt.begin(9600); // Bluetooth only

    Serial.println("USB Serial OK");
    bt.println("Bluetooth OK");

    servo.attach(SERVO_PIN);
    servo.write(90);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(1048); // correct calibration factor
    scale.tare();

    dht.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);

    Wire.begin();
    lcd.init();
    lcd.backlight();

    lcd.setCursor(0,0);
    lcd.print("System ready");
    delay(1000);
}

// -------------------------------------------------
void loop() {
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    handleBluetooth();

    if (isnan(temp) || isnan(hum)) {
        lcd.clear();
        lcd.print("DHT ERROR");
        delay(500);
        return;
    }

    if (!isFeeding) showStatus("STANDBY", temp, hum);
/*    // PIR: trigger only if cooldown passed and not currently feeding
    if (digitalRead(PIR_PIN) == HIGH && !isFeeding) {
        unsigned long now = millis();
        if (now - lastPIRDispense >= PIR_COOLDOWN_MS) {
            Serial.println("Animal detected (PIR) - dispensing");
            lastPIRDispense = now; // start cooldown (prevents repeated dispenses)
            dispenseFood(temp, hum);
            delay(2000);
        } else {
            Serial.println("PIR detected but cooldown active");
        }
    }*/
bool pirState = digitalRead(PIR_PIN);

if (pirState == HIGH && pirWasHigh == false && !isFeeding) {
    Serial.println("Animal detected");
    dispenseFood(temp, hum);
}

pirWasHigh = pirState;

    // Button handling: short press toggles open/close; long press stops feeding
    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonPrevState == HIGH && buttonState == LOW) {
        // pressed
        buttonPressStart = millis();
    } else if (buttonPrevState == LOW && buttonState == HIGH) {
        // released
        unsigned long pressDuration = millis() - buttonPressStart;
        if (pressDuration < 500) {
            // short press -> toggle only when not feeding
            if (!isFeeding) {
                if (isOpen) {
                    servo.write(90); // close
                    isOpen = false;
                    digitalWrite(LED_GREEN, LOW);
                    digitalWrite(LED_RED, HIGH);
                    Serial.println("Button: closed (toggle)");
                } else {
                    servo.write(0); // open
                    isOpen = true;
                    digitalWrite(LED_GREEN, HIGH);
                    digitalWrite(LED_RED, LOW);
                    Serial.println("Button: opened (toggle)");
                }
            } else {
                Serial.println("Button short-press ignored during feeding");
            }
        } else {
            // long press -> if feeding, stop feeding; otherwise toggle
            if (isFeeding) {
                Serial.println("Button long-press -> stop feeding");
                stopRequest = true;
                stopFeedingEarly();
            } else {
                Serial.println("Button long-press (no feeding) - toggling lid");
                if (isOpen) {
                    servo.write(90);
                    isOpen = false;
                    digitalWrite(LED_GREEN, LOW);
                    digitalWrite(LED_RED, HIGH);
                } else {
                    servo.write(0);
                    isOpen = true;
                    digitalWrite(LED_GREEN, HIGH);
                    digitalWrite(LED_RED, LOW);
                }
            }
        }
    }
    buttonPrevState = buttonState;

    delay(200);
}