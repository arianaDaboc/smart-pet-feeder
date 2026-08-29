#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

const char* convexHost = "YOUR_DEPLOYMENT.convex.site";
const char* ownerId = "YOUR_CLERK_USER_ID";

SoftwareSerial arduinoSerial(D5, D6);
ESP8266WebServer localServer(80);

struct FeedEvent {
  float weight;
  float temp;
  float hum;
  String source;
};

#define QUEUE_MAX 10
FeedEvent feedQueue[QUEUE_MAX];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

void pushQueue(float w, float t, float h, String src) {
  if (queueCount >= QUEUE_MAX) {
    // Keep the newest offline events when the fixed queue is full.
    queueTail = (queueTail + 1) % QUEUE_MAX;
    queueCount--;
  }
  feedQueue[queueHead].weight = w;
  feedQueue[queueHead].temp = t;
  feedQueue[queueHead].hum = h;
  feedQueue[queueHead].source = src;
  queueHead = (queueHead + 1) % QUEUE_MAX;
  queueCount++;
}

String deviceStatus = "BOOTING";
bool online = false;
bool wifiWasConnected = false;
unsigned long lastHeartbeatMillis = 0;
unsigned long lastWifiCheckMillis = 0;

float currentTemp = 0.0;
float currentHum = 0.0;
float currentWeight = 0.0;

unsigned long lastSettingsPollMillis = 0;
unsigned long lastTelemetryUploadMillis = 0;

bool lastPendingFeed = false;
float lastPortion = -1.0;
int lastCooldown = -1;
String lastExecutedCommand = "";

bool feedRequestPendingAck = false;
unsigned long lastFeedRequestTime = 0;
String pendingAckCommand = "CMD:WAIT_MOTION";

String inputBuffer = "";

void addCorsHeaders() {
  localServer.sendHeader("Access-Control-Allow-Origin", "*");
  localServer.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  localServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleCorsPreflight() {
  addCorsHeaders();
  localServer.send(204, "text/plain", "");
}

void handleLocalAuthorize() {
  const String body = localServer.arg("plain");
  const bool authorized = body.indexOf("\"authorized\":true") != -1 ||
                          body.indexOf("\"autorizat\":true") != -1;
  addCorsHeaders();
  if (!authorized) {
    localServer.send(403, "application/json", "{\"status\":\"blocked\",\"reason\":\"ai_authorization_required\"}");
    return;
  }

  arduinoSerial.println("CMD:WAIT_MOTION");
  pendingAckCommand = "CMD:WAIT_MOTION";
  lastFeedRequestTime = millis();
  feedRequestPendingAck = true;
  Serial.println("Local AI authorization received; waiting for Arduino ACK.");
  localServer.send(202, "application/json", "{\"status\":\"accepted\",\"action\":\"waiting_for_pir\",\"timeoutSeconds\":10}");
}

void handleLocalManualFeed() {
  addCorsHeaders();
  if (!online || deviceStatus == "OFFLINE") {
    localServer.send(503, "application/json", "{\"status\":\"blocked\",\"reason\":\"arduino_offline\"}");
    return;
  }
  if (deviceStatus == "COOLDOWN") {
    localServer.send(409, "application/json", "{\"status\":\"blocked\",\"reason\":\"cooldown\"}");
    return;
  }

  const String body = localServer.arg("plain");
  const int amountIndex = body.indexOf("\"amount\":");
  if (amountIndex != -1) {
    const float requestedAmount = body.substring(amountIndex + 9).toFloat();
    if (requestedAmount > 0.0 && requestedAmount < 500.0) {
      arduinoSerial.print("CMD:SET_TARGET:");
      arduinoSerial.println(requestedAmount, 1);
      delay(30);
    }
  }

  arduinoSerial.println("CMD:MANUAL_FEED");
  pendingAckCommand = "CMD:MANUAL_FEED";
  lastFeedRequestTime = millis();
  feedRequestPendingAck = true;
  Serial.println("Local manual feed requested.");
  localServer.send(202, "application/json", "{\"status\":\"accepted\",\"action\":\"manual_feed\"}");
}

void handleLocalStatus() {
  addCorsHeaders();
  String payload = "{\"wifi\":";
  payload += WiFi.status() == WL_CONNECTED ? "true" : "false";
  payload += ",\"arduinoOnline\":";
  payload += online ? "true" : "false";
  payload += ",\"deviceStatus\":\"";
  payload += deviceStatus;
  payload += "\",\"ip\":\"";
  payload += WiFi.localIP().toString();
  payload += "\"}";
  localServer.send(200, "application/json", payload);
}

bool postFeedEvent(float weight, float temp, float hum, String source) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://";
  url += convexHost;
  url += "/api/feed-event";

  if (http.begin(client, url)) {
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"ownerId\":\"";
    payload += ownerId;
    payload += "\",\"amountDispensed\":";
    payload += String(weight, 1);
    payload += ",\"triggerSource\":\"";
    payload += source;
    payload += "\",\"temperature\":";
    payload += String(temp, 1);
    payload += ",\"humidity\":";
    payload += String(hum, 0);
    payload += "}";

    int httpCode = http.POST(payload);
    http.end();
    return (httpCode == HTTP_CODE_OK || httpCode == 200);
  }
  return false;
}

bool postTelemetry(float temp, float hum, float weight, bool isOnline, String status, int rssi, float rawScale = -999.0, float calFactor = -999.0, bool clearCmd = false, bool clearFeed = false) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://";
  url += convexHost;
  url += "/api/telemetry";

  if (http.begin(client, url)) {
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"ownerId\":\"";
    payload += ownerId;
    payload += "\",\"temperature\":";
    payload += String(temp, 1);
    payload += ",\"humidity\":";
    payload += String(hum, 0);
    payload += ",\"weight\":";
    payload += String(weight, 1);
    payload += ",\"online\":";
    payload += isOnline ? "true" : "false";
    payload += ",\"wifiRSSI\":";
    payload += String(rssi);
    payload += ",\"deviceStatus\":\"";
    payload += status;
    payload += "\"";
    if (rawScale != -999.0) {
      payload += ",\"rawScaleValue\":";
      payload += String(rawScale, 2);
    }
    if (calFactor != -999.0) {
      payload += ",\"calibrationFactor\":";
      payload += String(calFactor, 2);
    }
    if (clearCmd) {
      payload += ",\"clearPendingCommand\":true";
    }
    if (clearFeed) {
      payload += ",\"clearPendingFeedRequest\":true";
    }
    payload += "}";

    int httpCode = http.POST(payload);
    http.end();
    return (httpCode == HTTP_CODE_OK || httpCode == 200);
  }
  return false;
}

void processQueue() {
  while (queueCount > 0 && WiFi.status() == WL_CONNECTED) {
    FeedEvent ev = feedQueue[queueTail];
    if (postFeedEvent(ev.weight, ev.temp, ev.hum, ev.source)) {
      queueTail = (queueTail + 1) % QUEUE_MAX;
      queueCount--;
    } else {
      break;
    }
  }
}

void pollDeviceSettings() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://";
  url += convexHost;
  url += "/api/device-settings?ownerId=";
  url += ownerId;

  if (http.begin(client, url)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK || httpCode == 200) {
      String payload = http.getString();

      bool pendingFeed = (payload.indexOf("\"pendingFeedRequest\":true") != -1);

      String pendingCommand = "";
      int cmdIndex = payload.indexOf("\"pendingCommand\":\"");
      if (cmdIndex != -1) {
        int start = cmdIndex + 18;
        int end = payload.indexOf("\"", start);
        if (end != -1) {
          pendingCommand = payload.substring(start, end);
        }
      }

      float portion = -1.0;
      int portionIndex = payload.indexOf("\"foodPortion\":");
      if (portionIndex != -1) {
        int start = portionIndex + 14;
        int end = payload.indexOf(",", start);
        if (end == -1) end = payload.indexOf("}", start);
        if (end != -1) {
          portion = payload.substring(start, end).toFloat();
        }
      }

      int cooldown = -1;
      int cooldownIndex = payload.indexOf("\"cooldownMinutes\"");
      if (cooldownIndex != -1) {
        int colon = payload.indexOf(":", cooldownIndex);
        if (colon != -1) {
          int start = colon + 1;
          while (start < payload.length() && (payload.charAt(start) == ' ' || payload.charAt(start) == '\t')) start++;
          int end = start;
          while (end < payload.length() && (isdigit(payload.charAt(end)) || payload.charAt(end) == '.')) end++;
          if (end > start) {
            cooldown = (int)payload.substring(start, end).toFloat();
          }
        }
      }

      if (pendingCommand.length() > 0) {
        if (pendingCommand != lastExecutedCommand) {
          lastExecutedCommand = pendingCommand;
          Serial.print("Forwarding Command to Arduino: ");
          Serial.println(pendingCommand);
          arduinoSerial.println(pendingCommand);
        }

        postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI(), -999.0, -999.0, true);
      } else {
        lastExecutedCommand = "";
      }

      if (pendingFeed && !lastPendingFeed) {
        arduinoSerial.println("CMD:WAIT_MOTION");
        lastFeedRequestTime = millis();
        feedRequestPendingAck = true;
        lastPendingFeed = true;

        postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI(), -999.0, -999.0, false, true);
      } else if (!pendingFeed) {
        lastPendingFeed = false;
      }

      if (portion > 0 && portion != lastPortion) {
        lastPortion = portion;
        arduinoSerial.print("CMD:SET_TARGET:");
        arduinoSerial.println(portion, 1);
      }

      if (cooldown > 0 && cooldown != lastCooldown) {
        lastCooldown = cooldown;
        arduinoSerial.print("CMD:SET_COOLDOWN:");
        arduinoSerial.println(cooldown * 60);
      }
    }
    http.end();
  }
}

void parseArduinoMessage(String msg) {
  msg.trim();
  if (!msg.startsWith("EVT:")) return;

  lastHeartbeatMillis = millis();
  online = true;

  if (msg == "EVT:SYSTEM_READY") {
    deviceStatus = "STANDBY";
    lastHeartbeatMillis = millis();
    online = true;
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
  }
  else if (msg.startsWith("EVT:STATUS:")) {
    deviceStatus = msg.substring(11);
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
  }
  else if (msg == "EVT:ALIVE") {
    lastHeartbeatMillis = millis();
    online = true;
  }
  else if (msg == "EVT:PIR_MOTION") {
    postTelemetry(currentTemp, currentHum, currentWeight, online, "PIR_MOTION", WiFi.RSSI());
  }
  else if (msg == "EVT:FEEDING_START") {
    deviceStatus = "FEEDING";
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
  }
  else if (msg == "EVT:FEEDING_END") {

  }
  else if (msg.startsWith("EVT:TELEMETRY:")) {

    int firstColon = msg.indexOf(':', 14);
    int secondColon = msg.indexOf(':', firstColon + 1);
    int thirdColon = msg.indexOf(':', secondColon + 1);
    if (firstColon != -1 && secondColon != -1) {
      currentTemp = msg.substring(14, firstColon).toFloat();
      currentHum = msg.substring(firstColon + 1, secondColon).toFloat();
      if (thirdColon != -1) {
        currentWeight = msg.substring(secondColon + 1, thirdColon).toFloat();
        deviceStatus = msg.substring(thirdColon + 1);
      } else {
        currentWeight = msg.substring(secondColon + 1).toFloat();
      }
      postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
    }
  }
  else if (msg.startsWith("EVT:SUMMARY:")) {

    int firstColon = msg.indexOf(':', 12);
    int secondColon = msg.indexOf(':', firstColon + 1);
    int thirdColon = msg.indexOf(':', secondColon + 1);

    if (firstColon != -1 && secondColon != -1 && thirdColon != -1) {
      float w = msg.substring(12, firstColon).toFloat();
      float t = msg.substring(firstColon + 1, secondColon).toFloat();
      float h = msg.substring(secondColon + 1, thirdColon).toFloat();
      String src = msg.substring(thirdColon + 1);

      if (!postFeedEvent(w, t, h, src)) {
        pushQueue(w, t, h, src);
      }

      postTelemetry(t, h, w, online, deviceStatus, WiFi.RSSI());
    }
  }
  else if (msg == "EVT:ACK:WAIT_MOTION" || msg == "EVT:ACK:FEED" || msg == "EVT:ACK:MANUAL_FEED") {
    feedRequestPendingAck = false;
    Serial.println("Arduino ACK: AI authorization received; waiting for PIR.");
  }
  else if (msg == "EVT:ACK:TARE") {
    Serial.println("Arduino ACK: Test Dispense Completed.");
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
  }
  else if (msg.startsWith("EVT:RAW_VALUE:")) {
    float rawVal = msg.substring(14).toFloat();
    Serial.print("Raw Scale Value received: ");
    Serial.println(rawVal);
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI(), rawVal);
  }
  else if (msg.startsWith("EVT:CALIBRATION_DONE:")) {
    float factor = msg.substring(21).toFloat();
    Serial.print("Calibration complete. Flow Rate: ");
    Serial.println(factor);
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI(), -999.0, factor);
  }
}

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(9600);

  Serial.println("\n--- ESP8266 Wi-Fi Bridge Started ---");

  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  localServer.on("/authorize", HTTP_POST, handleLocalAuthorize);
  localServer.on("/authorize", HTTP_OPTIONS, handleCorsPreflight);
  localServer.on("/manual-feed", HTTP_POST, handleLocalManualFeed);
  localServer.on("/manual-feed", HTTP_OPTIONS, handleCorsPreflight);
  localServer.on("/status", HTTP_GET, handleLocalStatus);
  localServer.on("/status", HTTP_OPTIONS, handleCorsPreflight);
  localServer.begin();
  Serial.println("Local feeder API started on port 80.");

  lastWifiCheckMillis = millis();
  lastHeartbeatMillis = millis();
}

void loop() {
  localServer.handleClient();

  while (arduinoSerial.available() > 0) {
    char c = arduinoSerial.read();
    if (c == '\n') {
      Serial.print("Recv Serial: ");
      Serial.println(inputBuffer);
      parseArduinoMessage(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {

      if (inputBuffer.length() < 160) {
        inputBuffer += c;
      } else {
        inputBuffer = "";
        Serial.println("Serial frame discarded: too long or corrupted.");
      }
    }
  }

  if (feedRequestPendingAck && (millis() - lastFeedRequestTime > 5000)) {
    Serial.println("No ACK received for FEED command, retrying...");
    arduinoSerial.println(pendingAckCommand);
    lastFeedRequestTime = millis();
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("Wi-Fi Connected! IP: ");
      Serial.println(WiFi.localIP());
      arduinoSerial.println("CMD:CAM_ONLINE");
      postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
    }

    processQueue();
  } else {
    if (wifiWasConnected) {
      wifiWasConnected = false;
      online = false;
      deviceStatus = "OFFLINE";
      arduinoSerial.println("CMD:CAM_OFFLINE");
    }

    if (millis() - lastWifiCheckMillis > 30000) {
      lastWifiCheckMillis = millis();
      Serial.println("Retrying Wi-Fi connection...");
      WiFi.reconnect();
    }
  }

  if (online && (millis() - lastHeartbeatMillis > 30000)) {
    Serial.println("Arduino Heartbeat TIMEOUT! Feeder offline.");
    online = false;
    deviceStatus = "OFFLINE";
    postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
  }

  unsigned long now = millis();

  if (now - lastSettingsPollMillis >= 15000) {
    lastSettingsPollMillis = now;
    if (WiFi.status() == WL_CONNECTED) {
      pollDeviceSettings();
    }
  }

  if (now - lastTelemetryUploadMillis >= 10000) {
    lastTelemetryUploadMillis = now;
    if (WiFi.status() == WL_CONNECTED) {
      postTelemetry(currentTemp, currentHum, currentWeight, online, deviceStatus, WiFi.RSSI());
    }
  }
}
