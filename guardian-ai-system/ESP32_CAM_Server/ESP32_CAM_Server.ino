#include <esp_camera.h>
#include <WiFi.h>
#include <esp_http_server.h>

// Helper macro for safe min calculation
#define MIN_VAL(a, b) ((a) < (b) ? (a) : (b))

// ============================================================
// WI-FI CONFIGURATION
// ============================================================
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ============================================================
// CAMERA PINOUT (AI-THINKER ESP32-CAM MODULE)
// ============================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_FLASH_PIN      4
#define LED_RED_PIN       33

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ============================================================
// HANDLER: GET /capture (Single JPEG frame snapshot)
// ============================================================
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ============================================================
// HANDLER: GET /stream (Live MJPEG video stream)
// ============================================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera stream capture failed");
      res = ESP_FAIL;
    } else {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
      esp_camera_fb_return(fb);
      fb = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS); // ~30 FPS frame pacing
  }
  return res;
}

// ============================================================
// HANDLER: GET/POST /feed (blocked: AI authorization must use /result)
// ============================================================
static esp_err_t feed_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_status(req, "403 Forbidden");
  httpd_resp_sendstr(req, "{\"status\":\"blocked\",\"reason\":\"ai_authorization_required\"}");
  return ESP_OK;
}

// ============================================================
// HANDLER: POST /result (Visual indicator feedback from Guardian AI)
// ============================================================
static esp_err_t result_handler(httpd_req_t *req) {
  char buf[200];
  int ret, remaining = req->content_len;

  if (remaining >= sizeof(buf)) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  while (remaining > 0) {
    int read_len = MIN_VAL(remaining, (int)sizeof(buf) - 1);
    if ((ret = httpd_req_recv(req, buf, read_len)) <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
      return ESP_FAIL;
    }
    remaining -= ret;
  }
  buf[req->content_len] = '\0';

  String content = String(buf);
  Serial.print("ESP32-CAM AI Result Payload: ");
  Serial.println(content);

  bool isAuthorized = (content.indexOf("\"autorizat\":true") != -1 || content.indexOf("\"authorized\":true") != -1);

  if (isAuthorized) {
    // Visual feedback only. NodeMCU is the sole module allowed to command Arduino.
    Serial.println("✅ AI RECOGNITION MATCH: AUTHORIZED PET DETECTED!");
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(150);
    digitalWrite(LED_FLASH_PIN, LOW);
  } else {
    Serial.println("⚠️ AI RECOGNITION: UNAUTHORIZED / UNKNOWN SUBJECT");
    digitalWrite(LED_RED_PIN, LOW);
    delay(200);
    digitalWrite(LED_RED_PIN, HIGH);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
  return ESP_OK;
}

// ============================================================
// START CAMERA WEB SERVER (Port 81)
// ============================================================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 81;

  httpd_uri_t capture_uri = {
    .uri       = "/capture",
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t feed_get_uri = {
    .uri       = "/feed",
    .method    = HTTP_GET,
    .handler   = feed_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t feed_post_uri = {
    .uri       = "/feed",
    .method    = HTTP_POST,
    .handler   = feed_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t result_uri = {
    .uri       = "/result",
    .method    = HTTP_POST,
    .handler   = result_handler,
    .user_ctx  = NULL
  };

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    httpd_register_uri_handler(camera_httpd, &feed_get_uri);
    httpd_register_uri_handler(camera_httpd, &feed_post_uri);
    httpd_register_uri_handler(camera_httpd, &result_uri);
  }
}

// ============================================================
// SETUP & INITIALIZATION
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n--- Guardian AI ESP32-CAM Camera Server ---");

  pinMode(LED_FLASH_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_FLASH_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH); // Off

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size & quality configuration
  if (psramFound()) {
    // 640x480 retains more face and coat detail for identity matching.
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wi-Fi connected!");
  Serial.print("ESP32-CAM Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
  Serial.print("ESP32-CAM Capture URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/capture");

  // Start HTTP Server
  startCameraServer();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  delay(10000);
}
