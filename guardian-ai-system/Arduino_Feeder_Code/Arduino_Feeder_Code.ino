#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define SERVO_PIN 7
#define BUTTON_PIN 2
#define BUZZER_PIN 8
#define LED_GREEN A1
#define LED_RED A2
#define PIR_PIN 11
#define DHTPIN 3

#define DHTTYPE DHT11
#define EEPROM_FACTOR_ADDR 0

enum DeviceState {
  BOOTING,
  STANDBY,
  FEEDING,
  COOLDOWN,
  ERROR_STATE,
  WAITING_FOR_MOTION
};

SoftwareSerial espSerial(10, 12);
Servo feederServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

DeviceState currentState = BOOTING;
float targetGrams = 15.0;
unsigned long cooldownDuration = 60;
unsigned long cooldownStartMillis = 0;

float gramsPerSecond = 5.0;
float currentWeight = 0.0;
float currentTemp = 22.0;
float currentHum = 45.0;

unsigned long lastHeartbeatMillis = 0;
unsigned long lastTelemetryMillis = 0;
unsigned long feedingStartMillis = 0;
const char* feedingSource = "BUTTON";
unsigned long lastPirMotionMillis = 0;
unsigned long waitingStartMillis = 0;
bool cameraOnline = true;
const unsigned long motionVerificationTimeoutMs = 10000UL;

unsigned long buttonPressStartMillis = 0;
bool buttonWasPressedInFeeding = false;
bool lastManualButtonState = HIGH;
unsigned long lastManualButtonTriggerMillis = 0;
const unsigned long manualButtonDebounceMs = 250UL;

String inputBuffer = "";

void detachServo() {
  feederServo.detach();
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
}

void attachServo() {
  feederServo.attach(SERVO_PIN);
}

void setDeviceStatus(DeviceState newState) {
  currentState = newState;

  switch (currentState) {
    case BOOTING:
      espSerial.println(F("EVT:STATUS:BOOTING"));
      break;
    case STANDBY:
      espSerial.println(F("EVT:STATUS:STANDBY"));
      break;
    case FEEDING:
      espSerial.println(F("EVT:STATUS:FEEDING"));
      break;
    case COOLDOWN:
      espSerial.println(F("EVT:STATUS:COOLDOWN"));
      break;
    case ERROR_STATE:
      espSerial.println(F("EVT:STATUS:ERROR"));
      break;
    case WAITING_FOR_MOTION:
      espSerial.println(F("EVT:STATUS:VERIFYING"));
      break;
  }
}

void beep(int durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationMs);
  digitalWrite(BUZZER_PIN, LOW);
}

void readSensors() {

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) currentTemp = t;
  if (!isnan(h)) currentHum = h;
}

const char* getDeviceStatusStr() {
  switch (currentState) {
    case BOOTING: return "BOOTING";
    case STANDBY: return "STANDBY";
    case FEEDING: return "FEEDING";
    case COOLDOWN: return "COOLDOWN";
    case ERROR_STATE: return "ERROR";
    case WAITING_FOR_MOTION: return "VERIFYING";
    default: return "UNKNOWN";
  }
}

void sendTelemetry() {

  espSerial.print(F("EVT:TELEMETRY:"));
  espSerial.print(currentTemp, 1);
  espSerial.print(F(":"));
  espSerial.print(currentHum, 0);
  espSerial.print(F(":"));
  espSerial.print(currentWeight, 1);
  espSerial.print(F(":"));
  espSerial.println(getDeviceStatusStr());
}

void startFeeding(const char* source) {
  if (currentState == COOLDOWN && strcmp(source, "FORCE") != 0) {
    Serial.println(F("startFeeding BLOCKED: Feeder is currently in COOLDOWN!"));
    return;
  }

  feedingSource = source;
  feedingStartMillis = millis();

  buttonWasPressedInFeeding = (digitalRead(BUTTON_PIN) == LOW);
  buttonPressStartMillis = millis();

  setDeviceStatus(FEEDING);
  espSerial.println(F("EVT:FEEDING_START"));

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);

  beep(150);
  delay(100);
  beep(150);

  attachServo();
  feederServo.write(0);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("FEEDING: "));
  lcd.print(targetGrams, 0);
  lcd.print(F("g"));
  lcd.setCursor(0, 1);
  lcd.print(F("Dispensed: 0.0g"));
}

void stopFeeding() {
  feederServo.write(90);
  delay(500);
  detachServo();

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Feeding Stopped"));
  beep(500);
  delay(1000);

  unsigned long elapsed = millis() - feedingStartMillis;
  float dispensedWeight = (elapsed / 1000.0) * gramsPerSecond;
  if (dispensedWeight > targetGrams) dispensedWeight = targetGrams;
  currentWeight = dispensedWeight;

  espSerial.println(F("EVT:FEEDING_END"));

  espSerial.print(F("EVT:SUMMARY:"));
  espSerial.print(dispensedWeight, 1);
  espSerial.print(F(":"));
  espSerial.print(currentTemp, 1);
  espSerial.print(F(":"));
  espSerial.print(currentHum, 0);
  espSerial.print(F(":"));
  espSerial.println(feedingSource);

  cooldownStartMillis = millis();
  setDeviceStatus(COOLDOWN);
  espSerial.println(F("EVT:COOLDOWN_START"));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("COOLDOWN "));
  lcd.print(cooldownDuration);
  lcd.print(F("s"));
  lcd.setCursor(0, 1);
  lcd.print(F("Please wait"));
}

void updateFeedingState() {

  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonWasPressedInFeeding) {
      buttonPressStartMillis = millis();
      buttonWasPressedInFeeding = true;
    } else {
      if (millis() - buttonPressStartMillis >= 1000) {
        stopFeeding();
        return;
      }
    }
  } else {
    buttonWasPressedInFeeding = false;
  }

  unsigned long elapsed = millis() - feedingStartMillis;
  float dispensedWeight = (elapsed / 1000.0) * gramsPerSecond;
  if (dispensedWeight > targetGrams) dispensedWeight = targetGrams;
  currentWeight = dispensedWeight;

  unsigned long requiredDurationMs = (unsigned long)((targetGrams / gramsPerSecond) * 1000.0);
  if (requiredDurationMs < 1000UL) requiredDurationMs = 1000UL;

  lcd.setCursor(0, 0);
  lcd.print(F("FEEDING: "));
  lcd.print(targetGrams, 0);
  lcd.print(F("g   "));
  lcd.setCursor(0, 1);
  lcd.print(F("Dispensed: "));
  lcd.print(dispensedWeight, 1);
  lcd.print(F("g  "));

  static unsigned long lastFeedingTelemetryMillis = 0;
  if (millis() - lastFeedingTelemetryMillis >= 1000) {
    lastFeedingTelemetryMillis = millis();
    sendTelemetry();
  }

  if (elapsed >= requiredDurationMs) {

    feederServo.write(90);
    delay(500);
    detachServo();

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Feeding Done!"));
    lcd.setCursor(0, 1);
    lcd.print(F("Total: "));
    lcd.print(targetGrams, 1);
    lcd.print(F("g"));
    beep(300);
    delay(1500);

    espSerial.println(F("EVT:FEEDING_END"));

    espSerial.print(F("EVT:SUMMARY:"));
    espSerial.print(targetGrams, 1);
    espSerial.print(F(":"));
    espSerial.print(currentTemp, 1);
    espSerial.print(F(":"));
    espSerial.print(currentHum, 0);
    espSerial.print(F(":"));
    espSerial.println(feedingSource);

    cooldownStartMillis = millis();
    setDeviceStatus(COOLDOWN);
    espSerial.println(F("EVT:COOLDOWN_START"));

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("COOLDOWN "));
    lcd.print(cooldownDuration);
    lcd.print(F("s"));
    lcd.setCursor(0, 1);
    lcd.print(F("Please wait"));
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (!cmd.startsWith("CMD:")) return;

  Serial.print(F("Received command from ESP: "));
  Serial.println(cmd);

  if (cmd == "CMD:MANUAL_FEED") {
    if (currentState == COOLDOWN) {
      Serial.println(F("MANUAL FEED REJECTED: System is currently in COOLDOWN mode!"));
      espSerial.println(F("EVT:BLOCKED:COOLDOWN"));
      return;
    }
    if (currentState == STANDBY) {
      espSerial.println(F("EVT:ACK:MANUAL_FEED"));
      Serial.println(F("Authorized manual app feed. Dispensing configured portion."));
      startFeeding("MANUAL_APP");
    }
  }
  // AI commands authorize a fresh PIR check; they never open the feeder directly.
  else if (cmd == "CMD:FEED" || cmd == "CMD:FORCE_FEED" || cmd == "CMD:WAIT_MOTION" || cmd == "CMD:VERIFY") {

    if (currentState == COOLDOWN) {
      Serial.println(F("FEED REJECTED: System is currently in COOLDOWN mode!"));
      espSerial.println(F("EVT:BLOCKED:COOLDOWN"));
      return;
    }
    if (currentState == STANDBY) {
      waitingStartMillis = millis();
      setDeviceStatus(WAITING_FOR_MOTION);
      Serial.println(F("AI recognized authorized pet! Waiting for PIR motion for 10 seconds..."));
      espSerial.println(F("EVT:ACK:WAIT_MOTION"));
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("PET VERIFIED!"));
      lcd.setCursor(0, 1);
      lcd.print(F("Wait Motion: 10s"));
    }
  }
  else if (cmd == "CMD:CAM_OFFLINE") {
    cameraOnline = false;
    Serial.println(F("Camera offline status received. Standalone PIR mode active."));
  }
  else if (cmd == "CMD:CAM_ONLINE") {
    cameraOnline = true;
    Serial.println(F("Camera online status received. Smart AI mode active."));
  }
  else if (cmd == "CMD:STOP") {
    espSerial.println(F("EVT:ACK:STOP"));
    if (currentState == FEEDING) {
      stopFeeding();
    } else if (currentState == WAITING_FOR_MOTION) {
      setDeviceStatus(STANDBY);
      espSerial.println(F("EVT:SUMMARY:0.0:0.0:0.0:WEB_CANCELLED"));
    }
  }
  else if (cmd.startsWith("CMD:SET_TARGET:")) {
    float val = cmd.substring(15).toFloat();
    if (val > 0 && val < 500) {
      targetGrams = val;
      espSerial.print(F("EVT:ACK:SET_TARGET:"));
      espSerial.println(targetGrams);
    }
  }
  else if (cmd.startsWith("CMD:SET_COOLDOWN:")) {
    long val = cmd.substring(17).toInt();

    if (val >= 60 && val <= 43200) {
      cooldownDuration = (unsigned long)val;
      espSerial.print(F("EVT:ACK:SET_COOLDOWN:"));
      espSerial.println(cooldownDuration);
    } else {
      Serial.println(F("SET_COOLDOWN REJECTED: expected 60..43200 seconds."));
      espSerial.println(F("EVT:ERROR:COOLDOWN_OUT_OF_RANGE"));
    }
  }
  else if (cmd == "CMD:TARE") {

    espSerial.println(F("EVT:ACK:TARE"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Test Dispense..."));
    lcd.setCursor(0, 1);
    lcd.print(F("5 Sec Run"));

    attachServo();
    feederServo.write(0);
    delay(5000);
    feederServo.write(90);
    delay(500);
    detachServo();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Test Complete!"));
    lcd.setCursor(0, 1);
    lcd.print(F("Weigh food now"));
    delay(2000);
  }
  else if (cmd == "CMD:GET_RAW") {
    espSerial.println(F("EVT:ACK:GET_RAW"));

    espSerial.println(F("EVT:RAW_VALUE:5.0"));
  }
  else if (cmd.startsWith("CMD:SET_FACTOR:")) {
    float val = cmd.substring(15).toFloat();
    if (val >= 0.5 && val <= 50.0) {
      gramsPerSecond = val;
      EEPROM.put(EEPROM_FACTOR_ADDR, gramsPerSecond);

      espSerial.print(F("EVT:ACK:SET_FACTOR:"));
      espSerial.println(gramsPerSecond);
    }
  }
  else if (cmd.startsWith("CMD:CALIBRATE:")) {
    float measuredGrams = cmd.substring(14).toFloat();
    if (measuredGrams > 0.0) {

      float rate = measuredGrams / 5.0;
      if (rate >= 0.5 && rate <= 50.0) {
        gramsPerSecond = rate;
        EEPROM.put(EEPROM_FACTOR_ADDR, gramsPerSecond);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calibrated Rate:"));
        lcd.setCursor(0, 1);
        lcd.print(gramsPerSecond, 2);
        lcd.print(F(" g/sec"));

        espSerial.print(F("EVT:CALIBRATION_DONE:"));
        espSerial.println(gramsPerSecond);
        delay(3000);
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calib Error"));
        lcd.setCursor(0, 1);
        lcd.print(F("Rate Out Range"));
        espSerial.println(F("EVT:ERROR:RATE_OUT_OF_RANGE"));
        delay(2000);
      }
    } else {
      espSerial.println(F("EVT:ERROR:INVALID_REF_WEIGHT"));
    }
  }
}

void setup() {

  pinMode(SERVO_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  Serial.begin(115200);
  espSerial.begin(9600);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Feeder Booting..."));

  setDeviceStatus(BOOTING);

  dht.begin();

  float savedRate = 0.0;
  EEPROM.get(EEPROM_FACTOR_ADDR, savedRate);
  if (!isnan(savedRate) && savedRate >= 0.5 && savedRate <= 50.0) {
    gramsPerSecond = savedRate;
  }

  detachServo();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Ready"));
  beep(100);
  delay(1500);

  espSerial.println(F("EVT:SYSTEM_READY"));
  espSerial.println(F("EVT:PROTOCOL:1"));

  setDeviceStatus(STANDBY);
}

void loop() {

  while (espSerial.available() > 0) {
    char c = espSerial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  if (currentState == FEEDING) {
    updateFeedingState();
  }
  else if (currentState == WAITING_FOR_MOTION) {

    if (digitalRead(PIR_PIN) == HIGH) {
      lastPirMotionMillis = millis();
      Serial.println(F("PIR Motion detected during wait! Starting feed."));
      startFeeding("WEB");
    }

    else if (millis() - waitingStartMillis >= motionVerificationTimeoutMs) {
      Serial.println(F("No PIR Motion detected within timeout. Feed request ignored."));
      setDeviceStatus(STANDBY);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("No Motion"));
      lcd.setCursor(0, 1);
      lcd.print(F("Feed Cancelled"));
      beep(100);
      delay(1500);
      espSerial.println(F("EVT:SUMMARY:0.0:0.0:0.0:AI_PIR_TIMEOUT"));
    }

    static unsigned long lastWaitLcdMillis = 0;
    if (millis() - lastWaitLcdMillis >= 1000) {
      lastWaitLcdMillis = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("PET VERIFIED!"));
      lcd.setCursor(0, 1);
      unsigned long elapsedSec = (millis() - waitingStartMillis) / 1000;
      unsigned long remainingSec = (elapsedSec >= 10) ? 0 : (10 - elapsedSec);
      lcd.print(F("Wait Motion: "));
      lcd.print(remainingSec);
      lcd.print(F("s"));
    }
  }
  else {

    const bool manualButtonState = digitalRead(BUTTON_PIN);
    if (currentState == STANDBY &&
        lastManualButtonState == HIGH &&
        manualButtonState == LOW &&
        millis() - lastManualButtonTriggerMillis >= manualButtonDebounceMs) {
      lastManualButtonTriggerMillis = millis();
      Serial.println(F("Physical button pressed. Dispensing configured portion."));
      espSerial.println(F("EVT:ACK:MANUAL_BUTTON"));
      startFeeding("MANUAL_BUTTON");
    }
    lastManualButtonState = manualButtonState;

    if (currentState == STANDBY && digitalRead(PIR_PIN) == HIGH) {
      lastPirMotionMillis = millis();
      static unsigned long lastPirReportMillis = 0;
      if (millis() - lastPirReportMillis >= 3000) {
        lastPirReportMillis = millis();
        espSerial.println(F("EVT:PIR_MOTION"));
      }
    }

    if (currentState == COOLDOWN) {
      unsigned long elapsedSec = (millis() - cooldownStartMillis) / 1000;
      if (elapsedSec >= cooldownDuration) {
        setDeviceStatus(STANDBY);
        espSerial.println(F("EVT:COOLDOWN_END"));
      }
    }

    unsigned long now = millis();

    if (now - lastHeartbeatMillis >= 10000) {
      lastHeartbeatMillis = now;
      espSerial.println(F("EVT:ALIVE"));
    }

    if (now - lastTelemetryMillis >= 10000) {
      lastTelemetryMillis = now;
      readSensors();
      sendTelemetry();
    }

    static unsigned long lastLcdUpdateMillis = 0;
    if (now - lastLcdUpdateMillis >= 1000) {
      lastLcdUpdateMillis = now;

      readSensors();

      lcd.clear();

      lcd.setCursor(0, 0);
      if (currentState == COOLDOWN) {
        unsigned long elapsedCooldownSec = (now - cooldownStartMillis) / 1000;
        unsigned long remaining = elapsedCooldownSec < cooldownDuration
          ? cooldownDuration - elapsedCooldownSec
          : 0;
        lcd.print(F("COOLDOWN "));
        lcd.print(remaining);
        lcd.print(F("s"));
      } else if (currentState == STANDBY) {
        lcd.print(F("STANDBY"));
      } else if (currentState == FEEDING) {
        lcd.print(F("FEEDING: "));
        lcd.print(targetGrams, 0);
        lcd.print(F("g"));
      } else if (currentState == WAITING_FOR_MOTION) {
        lcd.print(F("WAITING MOTION"));
      } else {
        lcd.print(getDeviceStatusStr());
      }

      lcd.setCursor(0, 1);
      if (currentState == COOLDOWN) {
        lcd.print(F("Please wait     "));
      } else {
        lcd.print((int)currentTemp);
        lcd.print(F("C "));
        lcd.print((int)currentHum);
        lcd.print(F("%  Ready"));
      }
    }
  }
}
