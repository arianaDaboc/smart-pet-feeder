#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


// ------------------- PINS CONFIGURATION -------------------
#define SERVO_PIN 7         // Servo motor control pin
#define BUTTON_PIN 2        // Manual button
#define BUZZER_PIN 8        // Active buzzer
#define LED_GREEN A1        // Green LED (Feeding)
#define LED_RED A2          // Red LED (Standby/Cooldown)
#define PIR_PIN 11          // PIR Motion Sensor
#define DHTPIN 3            // DHT11 Pin

// ------------------- CONSTANTS & DEFINES -------------------
#define DHTTYPE DHT11
#define EEPROM_FACTOR_ADDR 0     // EEPROM address for gramsPerSecond flow rate

// ------------------- DEVICE STATES -------------------
enum DeviceState {
  BOOTING,
  STANDBY,
  FEEDING,
  COOLDOWN,
  ERROR_STATE,
  WAITING_FOR_MOTION
};

// ------------------- GLOBAL OBJECTS -------------------
SoftwareSerial espSerial(10, 12); // RX = Pin 10, TX = Pin 12
Servo feederServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// ------------------- STATE VARIABLES -------------------
DeviceState currentState = BOOTING;
float targetGrams = 15.0;            // Portion size in grams
unsigned long cooldownDuration = 60;  // Default cooldown in seconds (1 minute)
unsigned long cooldownStartMillis = 0;

float gramsPerSecond = 5.0;          // Flow rate in grams per second (default 5.0g/s)
float currentWeight = 0.0;           // Simulated/estimated weight output
float currentTemp = 22.0;
float currentHum = 45.0;

// Timing variables
unsigned long lastHeartbeatMillis = 0;
unsigned long lastTelemetryMillis = 0;
unsigned long feedingStartMillis = 0;
const char* feedingSource = "BUTTON"; // "PIR", "WEB", "MANUAL_BUTTON", "PIR_SAFETY" or "WEB_FORCE"
unsigned long lastPirMotionMillis = 0; // Timestamp of last PIR motion detection
unsigned long waitingStartMillis = 0;   // Timestamp when AI authorized feed but waiting for PIR motion
bool cameraOnline = true;               // Flag indicating if the camera is online/functional
const unsigned long motionVerificationTimeoutMs = 10000UL;


// Button long-press state tracking during feeding
unsigned long buttonPressStartMillis = 0;
bool buttonWasPressedInFeeding = false;
bool lastManualButtonState = HIGH;
unsigned long lastManualButtonTriggerMillis = 0;
const unsigned long manualButtonDebounceMs = 250UL;

// Serial buffer
String inputBuffer = "";

// ------------------- HELPER FUNCTIONS -------------------

// Force Servo pin to ground to prevent RF noise jitter
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
  
  // Report state to ESP
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
  // Read Temp & Hum (DHT11)
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
  // Send single compact telemetry line
  // EVT:TELEMETRY:temp:humidity:weight:status
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
  
  // Initialize button state tracking for long-press stop
  buttonWasPressedInFeeding = (digitalRead(BUTTON_PIN) == LOW);
  buttonPressStartMillis = millis();
  
  setDeviceStatus(FEEDING);
  espSerial.println(F("EVT:FEEDING_START"));
  
  // LED signals
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  
  // Play buzzer melody
  beep(150);
  delay(100);
  beep(150);
 
  // Attach servo and open lid (position 0)
  attachServo();
  feederServo.write(0);

  // Show FEEDING immediately; do not wait for the next loop iteration.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("FEEDING: "));
  lcd.print(targetGrams, 0);
  lcd.print(F("g"));
  lcd.setCursor(0, 1);
  lcd.print(F("Dispensed: 0.0g"));
}

void stopFeeding() {
  feederServo.write(90); // Close lid
  delay(500);
  detachServo();
  
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Feeding Stopped"));
  beep(500);
  delay(1000);
  
  // Calculate elapsed dispensed weight
  unsigned long elapsed = millis() - feedingStartMillis;
  float dispensedWeight = (elapsed / 1000.0) * gramsPerSecond;
  if (dispensedWeight > targetGrams) dispensedWeight = targetGrams;
  currentWeight = dispensedWeight;
  
  espSerial.println(F("EVT:FEEDING_END"));
  
  // Send Feed Summary
  espSerial.print(F("EVT:SUMMARY:"));
  espSerial.print(dispensedWeight, 1);
  espSerial.print(F(":"));
  espSerial.print(currentTemp, 1);
  espSerial.print(F(":"));
  espSerial.print(currentHum, 0);
  espSerial.print(F(":"));
  espSerial.println(feedingSource);
  
  // Start Cooldown
  cooldownStartMillis = millis();
  setDeviceStatus(COOLDOWN);
  espSerial.println(F("EVT:COOLDOWN_START"));

  // Show COOLDOWN immediately after closing the lid.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("COOLDOWN "));
  lcd.print(cooldownDuration);
  lcd.print(F("s"));
  lcd.setCursor(0, 1);
  lcd.print(F("Please wait"));
}

void updateFeedingState() {
  // Check for button long-press to stop feeding
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonWasPressedInFeeding) {
      buttonPressStartMillis = millis();
      buttonWasPressedInFeeding = true;
    } else {
      if (millis() - buttonPressStartMillis >= 1000) { // 1 second long press
        stopFeeding();
        return;
      }
    }
  } else {
    buttonWasPressedInFeeding = false;
  }

  // Calculate dispensed weight based on elapsed time and flow rate
  unsigned long elapsed = millis() - feedingStartMillis;
  float dispensedWeight = (elapsed / 1000.0) * gramsPerSecond;
  if (dispensedWeight > targetGrams) dispensedWeight = targetGrams;
  currentWeight = dispensedWeight;

  // Calculate total time needed for portion
  unsigned long requiredDurationMs = (unsigned long)((targetGrams / gramsPerSecond) * 1000.0);
  if (requiredDurationMs < 1000UL) requiredDurationMs = 1000UL;

  // Update LCD display during feeding
  lcd.setCursor(0, 0);
  lcd.print(F("FEEDING: "));
  lcd.print(targetGrams, 0);
  lcd.print(F("g   "));
  lcd.setCursor(0, 1);
  lcd.print(F("Dispensed: "));
  lcd.print(dispensedWeight, 1);
  lcd.print(F("g  "));
  
  // Rate-limit telemetry transmission to ESP (every 1000ms)
  static unsigned long lastFeedingTelemetryMillis = 0;
  if (millis() - lastFeedingTelemetryMillis >= 1000) {
    lastFeedingTelemetryMillis = millis();
    sendTelemetry();
  }
  
  // Check if target duration reached
  if (elapsed >= requiredDurationMs) {
    // Finish feeding
    feederServo.write(90); // Close lid
    delay(500);            // Let motor rotate
    detachServo();         // Active ground and release
    
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
    
    // Send Feed Summary
    espSerial.print(F("EVT:SUMMARY:"));
    espSerial.print(targetGrams, 1);
    espSerial.print(F(":"));
    espSerial.print(currentTemp, 1);
    espSerial.print(F(":"));
    espSerial.print(currentHum, 0);
    espSerial.print(F(":"));
    espSerial.println(feedingSource);
    
    // Start Cooldown
    cooldownStartMillis = millis();
    setDeviceStatus(COOLDOWN);
    espSerial.println(F("EVT:COOLDOWN_START"));

    // Show COOLDOWN immediately after the configured portion is complete.
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
  
  // Debug print to hardware Serial so user can see received commands
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
  else if (cmd == "CMD:FEED" || cmd == "CMD:FORCE_FEED" || cmd == "CMD:WAIT_MOTION" || cmd == "CMD:VERIFY") {
    // Remote commands authorize verification; they never open the lid directly.
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
    // The app sends seconds and supports 1–720 minutes (60–43200 seconds).
    // Never accept zero because it would make COOLDOWN expire immediately.
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
    // Run a 5-second test dispense for flow-rate calibration
    espSerial.println(F("EVT:ACK:TARE"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Test Dispense..."));
    lcd.setCursor(0, 1);
    lcd.print(F("5 Sec Run"));
    
    attachServo();
    feederServo.write(0); // Open lid
    delay(5000);          // Dispense for exactly 5 seconds
    feederServo.write(90); // Close lid
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
    // Send 5.0 (5 seconds test duration)
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
      // Calculate grams per second based on the 5-second test dispense
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

// ------------------- SETUP -------------------
void setup() {
  // Pin modes
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  // Serial setup
  Serial.begin(115200);
  espSerial.begin(9600);

  // Initial LED states
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH); // Standby LED

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Feeder Booting..."));
  
  setDeviceStatus(BOOTING);

  // Initialize sensors
  dht.begin();
  
  // Read flow rate from EEPROM
  float savedRate = 0.0;
  EEPROM.get(EEPROM_FACTOR_ADDR, savedRate);
  if (!isnan(savedRate) && savedRate >= 0.5 && savedRate <= 50.0) {
    gramsPerSecond = savedRate;
  }
  
  detachServo(); // Attach on-demand

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Ready"));
  beep(100);
  delay(1500);
  
  // Report protocol readiness
  espSerial.println(F("EVT:SYSTEM_READY"));
  espSerial.println(F("EVT:PROTOCOL:1"));
  
  setDeviceStatus(STANDBY);
}

// ------------------- LOOP -------------------
void loop() {
  // Read Serial inputs from ESP
  while (espSerial.available() > 0) {
    char c = espSerial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // State-specific loops
  if (currentState == FEEDING) {
    updateFeedingState();
  } 
  else if (currentState == WAITING_FOR_MOTION) {
    // If PIR sensor detects motion, start feeding immediately!
    if (digitalRead(PIR_PIN) == HIGH) {
      lastPirMotionMillis = millis();
      Serial.println(F("PIR Motion detected during wait! Starting feed."));
      startFeeding("WEB");
    } 
    // Cancel when PIR does not confirm motion within the authorization window.
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
    
    // LCD screen real-time update during waiting
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
    // Normal loop: STANDBY or COOLDOWN

    // Physical manual button: a new press in STANDBY starts one configured portion.
    // Holding the same button for at least one second while FEEDING is handled by
    // updateFeedingState(), which closes the lid and enters COOLDOWN via stopFeeding().
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

    // Check PIR sensor: send EVT:PIR_MOTION to ESP bridge for real-time tracking
    if (currentState == STANDBY && digitalRead(PIR_PIN) == HIGH) {
      lastPirMotionMillis = millis(); // Update local motion timestamp
      static unsigned long lastPirReportMillis = 0;
      if (millis() - lastPirReportMillis >= 3000) { // 3s rate-limit for hardware PIR reporting
        lastPirReportMillis = millis();
        espSerial.println(F("EVT:PIR_MOTION"));
      }
    }

    // Check cooldown expiry
    if (currentState == COOLDOWN) {
      unsigned long elapsedSec = (millis() - cooldownStartMillis) / 1000;
      if (elapsedSec >= cooldownDuration) {
        setDeviceStatus(STANDBY);
        espSerial.println(F("EVT:COOLDOWN_END"));
      }
    }

    // Regular tasks (Heartbeat + Telemetry + LCD update)
    unsigned long now = millis();
    
    // Heartbeat every 10 seconds
    if (now - lastHeartbeatMillis >= 10000) {
      lastHeartbeatMillis = now;
      espSerial.println(F("EVT:ALIVE"));
    }

    // Telemetry send every 10 seconds
    if (now - lastTelemetryMillis >= 10000) {
      lastTelemetryMillis = now;
      readSensors();
      sendTelemetry();
    }

    // LCD screen real-time update every 1 second
    static unsigned long lastLcdUpdateMillis = 0;
    if (now - lastLcdUpdateMillis >= 1000) {
      lastLcdUpdateMillis = now;
      
      readSensors();
      
      lcd.clear();
      
      // Line 1: State
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

      // Line 2: Temp + Hum
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
