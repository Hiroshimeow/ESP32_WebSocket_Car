// =======================================================================
// =======================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "mbedtls/base64.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// --- Camera Model ---
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "web_index.h" // Giữ nguyên file web_index.h cũ của bạn

// =======================================================================
// ======================= GLOBAL VARIABLES ==============================
// =======================================================================

// --- Cấu hình WiFi từ code cũ ---
const char *ap_ssid = "+00000_257d48";
const char *ap_password = "86a0710b";

String hostname = "WIFI Car"; // Tên AP sẽ được tạo ra nếu kết nối thất bại
String WiFiAddr = "";

// =======================================================================
// === CẤU HÌNH GPIO ĐÚNG CHO 2 MOTOR ===
// =======================================================================
// Motor sau (Tiến/Lùi) - Chỉ 1 motor
int gpDriveF = 14; // Chân điều khiển motor sau chạy TỚI
int gpDriveB = 12; // Chân điều khiển motor sau chạy LÙI

// Motor trước (Rẽ Trái/Phải) - Chỉ 1 motor
int gpSteerL = 15; // Chân điều khiển motor trước rẽ TRÁI
int gpSteerR = 13; // Chân điều khiển motor trước rẽ PHẢI

// Các chân phụ (giữ nguyên)
int gpLed = 4; int gpClaxon = 2;

// --- PWM Configuration ---
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
// *** BIẾN TOÀN CỤC MỚI ĐỂ LƯU CẤU HÌNH MOTOR ***
int driveMotorPower = 255; // Công suất motor sau (0-255), mặc định 100%
int steerMotorPower = 102; // Công suất motor trước (0-255), mặc định 40%
bool driveMotorReversed = false; // Cờ đảo chiều motor sau
bool steerMotorReversed = false; // Cờ đảo chiều motor trước

// --- HTTP & Stream Server ---
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// --- WEBSOCKET SERVER ---
WebSocketsServer webSocket = WebSocketsServer(82);

// =======================================================================
// ======================= HELPER & MOTOR FUNCTIONS ======================
// =======================================================================

// --- Các hàm đọc/ghi file từ code cũ ---
String readFile(fs::FS &fs, const char * path){
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){ return String(); }
  String fileContent;
  while(file.available()){ fileContent += (char)file.read(); }
  file.close();
  return fileContent;
}
void writeFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, "w");
  if(!file){ file.close(); return; }
  file.print(message);
  file.close();
}
void setupPWM() {
    ledcAttach(gpDriveF, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(gpDriveB, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(gpSteerL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(gpSteerR, PWM_FREQ, PWM_RESOLUTION);
}

void Drive(int speed) { // Điều khiển motor sau
    // Tắt cả 2 chân trước khi ra lệnh để đảm bảo an toàn
    ledcWrite(gpDriveF, 0);
    ledcWrite(gpDriveB, 0);

    if (speed > 0) { // Lệnh Tiến
        // Nếu không đảo chiều, chạy chân F. Nếu đảo chiều, chạy chân B.
        int pinToPower = driveMotorReversed ? gpDriveB : gpDriveF;
        ledcWrite(pinToPower, speed);
    } else if (speed < 0) { // Lệnh Lùi
        // Nếu không đảo chiều, chạy chân B. Nếu đảo chiều, chạy chân F.
        int pinToPower = driveMotorReversed ? gpDriveF : gpDriveB;
        ledcWrite(pinToPower, -speed); // Dùng -speed vì speed đang là số âm
    }
    // Nếu speed = 0, cả 2 chân đã được tắt ở trên.
}

void Steer(int direction) { // Điều khiển motor trước
    // Tắt cả 2 chân trước khi ra lệnh để đảm bảo an toàn
    ledcWrite(gpSteerL, 0);
    ledcWrite(gpSteerR, 0);

    if (direction < 0) { // Lệnh Rẽ Trái (W)
        // Nếu không đảo chiều, chạy chân L. Nếu đảo chiều, chạy chân R.
        int pinToPower = steerMotorReversed ? gpSteerR : gpSteerL;
        ledcWrite(pinToPower, steerMotorPower);
    } else if (direction > 0) { // Lệnh Rẽ Phải (E)
        // Nếu không đảo chiều, chạy chân R. Nếu đảo chiều, chạy chân L.
        int pinToPower = steerMotorReversed ? gpSteerL : gpSteerR;
        ledcWrite(pinToPower, steerMotorPower);
    }
    // Nếu direction = 0, cả 2 chân đã được tắt ở trên.
}

void StopAllMotors() {
    Drive(0);
    Steer(0);
}

// =======================================================================
// ======================= WEBSOCKET LOGIC ===============================
// =======================================================================
void handleWebSocketMessage(uint8_t * payload, size_t length) {
    if (length > 256) return;
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload)) return;

    if (doc.containsKey("direction")) {
        const char* direction = doc["direction"];
        int speedPercent = doc["speed"].as<int>();

        // *** SỬA LỖI LOGIC MAP TỐC ĐỘ ***
        // 1. Chỉ map khi công suất tối đa > 0.
        // 2. Tốc độ khởi điểm (để thắng quán tính) sẽ là 40% của công suất tối đa, nhưng không vượt quá 120.
        int speedPwm = 0;
        if (driveMotorPower > 0) {
            int minPwm = min(120, (int)(driveMotorPower * 0.4)); // Tốc độ tối thiểu để thắng quán tính
            speedPwm = map(speedPercent, 1, 100, minPwm, driveMotorPower);
        }
        // Nếu speedPercent quá thấp (dưới 15%), coi như là dừng.
        if (speedPercent < 15) speedPwm = 0;
        
        Serial.printf("Received Command -> Direction: %s, Speed: %d%%, PWM: %d\n", direction, speedPercent, speedPwm);

        // Logic điều khiển đồng thời (không đổi)
        if (strstr(direction, "N")) { Drive(speedPwm); }
        else if (strstr(direction, "S")) { Drive(-speedPwm); }
        else { Drive(0); }
        
        if (strstr(direction, "W")) { Steer(-1); }
        else if (strstr(direction, "E")) { Steer(1); }
        else { Steer(0); }

        if (strcmp(direction, "stop") == 0) {
            StopAllMotors();
        }

    } else {
        StopAllMotors();
    }
    
    if (doc.containsKey("lighton")) {
        digitalWrite(gpLed, doc["lighton"].as<bool>() ? HIGH : LOW);
    }
    if (doc.containsKey("claxonon")) {
        digitalWrite(gpClaxon, doc["claxonon"].as<bool>() ? HIGH : LOW);
    }
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    handleWebSocketMessage(payload, length);
  } else if (type == WStype_DISCONNECTED) {
    StopAllMotors(); // Dừng xe nếu client mất kết nối
  }
}

// =======================================================================
// ======================= HTTP SERVER LOGIC =============================
// =======================================================================

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t * _jpg_buf;
  char * part_buf[64];
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){ return res; }
  while(true){
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    _jpg_buf_len = fb->len; _jpg_buf = fb->buf;
    if(res == ESP_OK){ res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)); }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){ res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len); }
    esp_camera_fb_return(fb);
    if(res != ESP_OK){ break; }
  }
  return res;
}

static esp_err_t index_handler(httpd_req_t *req){
  String sta_ssid = readFile(SPIFFS, "/wifi_ssid.txt");
  String sta_pass = readFile(SPIFFS, "/wifi_pass.txt");
  String sta_usercontrol = readFile(SPIFFS, "/user_control.txt");
  // *** ĐỌC CẤU HÌNH MOTOR TỪ SPIFFS ĐỂ GỬI LÊN GIAO DIỆN ***
  String sta_drivepower = readFile(SPIFFS, "/drive_power.txt");
  String sta_steerpower = readFile(SPIFFS, "/steer_power.txt");
  String sta_driverev = readFile(SPIFFS, "/drive_rev.txt");
  String sta_steerrev = readFile(SPIFFS, "/steer_rev.txt");

  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Usercontrol", sta_usercontrol.c_str());
  httpd_resp_set_hdr(req, "Wifissid", sta_ssid.c_str());
  httpd_resp_set_hdr(req, "Wifipass", sta_pass.c_str());
  httpd_resp_set_hdr(req, "Server", WiFiAddr.c_str());
  // *** GỬI CÁC HEADER MỚI CHỨA CẤU HÌNH MOTOR ***
  httpd_resp_set_hdr(req, "Drivepower", sta_drivepower.c_str());
  httpd_resp_set_hdr(req, "Steerpower", sta_steerpower.c_str());
  httpd_resp_set_hdr(req, "Driverev", sta_driverev.c_str());
  httpd_resp_set_hdr(req, "Steerrev", sta_steerrev.c_str());

  return httpd_resp_send(req, (const char*)frontend_html_gz, frontend_html_gz_len);
}

static esp_err_t wifilist_handler(httpd_req_t *req){
  String wifi_list_csv_resp = "{\"result\":[";
  int n = WiFi.scanNetworks();
  if (n > 0) {
      for (int i = 0; i < n; ++i) {
          wifi_list_csv_resp += "{\"ssid\":\"" + WiFi.SSID(i)
            + "\", \"signal\":\"" + String(WiFi.RSSI(i))
            + "\", \"security\": \"" + ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "NEED-PASS")
            + ((i == (n - 1)) ? "\"}" : "\"}, ");
          delay(10);
      }
  }
  wifi_list_csv_resp += "]}";
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, wifi_list_csv_resp.c_str(), wifi_list_csv_resp.length());
}

static esp_err_t cmd_handler(httpd_req_t *req){
  char buf[256]; // *** TĂNG BỘ ĐỆM ĐỂ CHỨA THÊM THAM SỐ ***
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char command[32], ssid[64], password[64], usercontrol[32];
    // *** THÊM BIẾN ĐỂ NHẬN CẤU HÌNH MOTOR TỪ GIAO DIỆN ***
    char drivepower[8], steerpower[8], driverev[4], steerrev[4];

    if (httpd_query_key_value(buf, "command", command, sizeof(command)) == ESP_OK &&
        strcmp(command, "saveconfig") == 0 &&
        httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid)) == ESP_OK &&
        httpd_query_key_value(buf, "password", password, sizeof(password)) == ESP_OK &&
        httpd_query_key_value(buf, "usercontrol", usercontrol, sizeof(usercontrol)) == ESP_OK &&
        // *** LẤY CÁC GIÁ TRỊ CẤU HÌNH MOTOR TỪ URL ***
        httpd_query_key_value(buf, "drivepower", drivepower, sizeof(drivepower)) == ESP_OK &&
        httpd_query_key_value(buf, "steerpower", steerpower, sizeof(steerpower)) == ESP_OK &&
        httpd_query_key_value(buf, "driverev", driverev, sizeof(driverev)) == ESP_OK &&
        httpd_query_key_value(buf, "steerrev", steerrev, sizeof(steerrev)) == ESP_OK
      ) {
      
      Serial.println("Saving WiFi & Motor config...");
      writeFile(SPIFFS, "/wifi_ssid.txt", ssid);
      writeFile(SPIFFS, "/wifi_pass.txt", password);
      writeFile(SPIFFS, "/user_control.txt", usercontrol);
      // *** LƯU CẤU HÌNH MOTOR VÀO CÁC FILE RIÊNG ***
      writeFile(SPIFFS, "/drive_power.txt", drivepower);
      writeFile(SPIFFS, "/steer_power.txt", steerpower);
      writeFile(SPIFFS, "/drive_rev.txt", driverev);
      writeFile(SPIFFS, "/steer_rev.txt", steerrev);
      
      httpd_resp_set_type(req, "application/json");
      const char* resp = "{\"result\": \"OK, restarting...\"}";
      httpd_resp_send(req, resp, strlen(resp));
      delay(1000);
      ESP.restart();
      return ESP_OK;
    }
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
    httpd_uri_t cmd_uri = { .uri = "/control", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL };
    httpd_uri_t wifilist_uri = { .uri = "/wifilist", .method = HTTP_GET, .handler = wifilist_handler, .user_ctx = NULL };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &wifilist_uri);
    }
    config.server_port += 1;
    config.ctrl_port += 1;
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

// =======================================================================
// ======================= SETUP & LOOP ==================================
// =======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println();

  if(!SPIFFS.begin(true)){ Serial.println("SPIFFS Mount Failed"); return; }
  
  // *** SỬA LỖI LOGIC ĐỌC CẤU HÌNH MOTOR ***
  String drivePowerStr = readFile(SPIFFS, "/drive_power.txt");
  if (drivePowerStr.length() > 0) {
    // Nếu file có nội dung, đọc và áp dụng
    driveMotorPower = map(drivePowerStr.toInt(), 0, 100, 0, 255);
  } else {
    // Nếu file không tồn tại hoặc rỗng, ghi giá trị mặc định và áp dụng ngay lập tức
    driveMotorPower = 255; // Mặc định 100% công suất (255)
    writeFile(SPIFFS, "/drive_power.txt", "100");
  }

  String steerPowerStr = readFile(SPIFFS, "/steer_power.txt");
  if (steerPowerStr.length() > 0) {
    steerMotorPower = map(steerPowerStr.toInt(), 0, 100, 0, 255);
  } else {
    steerMotorPower = 102; // Mặc định 40% công suất (~102)
    writeFile(SPIFFS, "/steer_power.txt", "40");
  }

  // Logic đọc cờ đảo chiều (không đổi, vẫn đúng)
  String driveRevStr = readFile(SPIFFS, "/drive_rev.txt");
  driveMotorReversed = (driveRevStr == "1");
  if (driveRevStr.length() == 0) writeFile(SPIFFS, "/drive_rev.txt", "0");

  String steerRevStr = readFile(SPIFFS, "/steer_rev.txt");
  steerMotorReversed = (steerRevStr == "1");
  if (steerRevStr.length() == 0) writeFile(SPIFFS, "/steer_rev.txt", "0");
  
  Serial.printf("Motor Config Loaded -> Drive Power: %d, Steer Power: %d, Drive Reversed: %d, Steer Reversed: %d\n",
    driveMotorPower, steerMotorPower, driveMotorReversed, steerMotorReversed);

  // --- Cấu hình chân GPIO ---
  pinMode(gpDriveF, OUTPUT);
  pinMode(gpDriveB, OUTPUT);
  pinMode(gpSteerL, OUTPUT);
  pinMode(gpSteerR, OUTPUT);
  pinMode(gpLed, OUTPUT);
  pinMode(gpClaxon, OUTPUT);
  setupPWM();
  StopAllMotors();

  // --- CẤU HÌNH CAMERA ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; config.jpeg_quality = 9; config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA; config.jpeg_quality = 12; config.fb_count = 1;
  }
  
  if (esp_camera_init(&config) != ESP_OK) { Serial.println("Camera init failed"); return; }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);

  // --- Logic kết nối WiFi ---
  String sta_ssid_b64 = readFile(SPIFFS, "/wifi_ssid.txt");
  String sta_pass_b64 = readFile(SPIFFS, "/wifi_pass.txt");
  
  unsigned char decoded_ssid_buf[40] = {0};
  unsigned char decoded_pass_buf[70] = {0};
  size_t ssid_len = 0;
  size_t pass_len = 0;

  mbedtls_base64_decode(decoded_ssid_buf, sizeof(decoded_ssid_buf), &ssid_len, (const unsigned char *)sta_ssid_b64.c_str(), sta_ssid_b64.length());
  mbedtls_base64_decode(decoded_pass_buf, sizeof(decoded_pass_buf), &pass_len, (const unsigned char *)sta_pass_b64.c_str(), sta_pass_b64.length());

  String ssid_decoded = String((char*)decoded_ssid_buf);
  String pass_decoded = String((char*)decoded_pass_buf);

  WiFi.mode(WIFI_STA);
  if (ssid_decoded.length() > 0) {
    WiFi.begin(ssid_decoded.c_str(), pass_decoded.c_str());
    Serial.print("Connecting to: "); Serial.println(ssid_decoded);
  }

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500); Serial.print(".");
  }

  IPAddress myIP;
  if (WiFi.status() == WL_CONNECTED) {
    myIP = WiFi.localIP();
    Serial.println("\n*WiFi-STA-Mode*");
    Serial.print("IP: "); Serial.println(myIP);
  } else {
    Serial.println("\nWiFi connection failed. Starting AP: " + hostname);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(hostname.c_str());
    myIP = WiFi.softAPIP();
    Serial.println("*WiFi-AP-Mode*");
    Serial.print("AP IP address: "); Serial.println(myIP);
  }
  WiFiAddr = myIP.toString();

  // --- Khởi tạo các server ---
  startCameraServer();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.print("Camera Ready! Use 'http://"); Serial.print(myIP); Serial.println("' to connect");
}

void loop() {
  webSocket.loop();
}