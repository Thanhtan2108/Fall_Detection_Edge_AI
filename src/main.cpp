// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ============================================================
//  CẤU HÌNH WIFI
// ============================================================
const char* WIFI_SSID = "Tu Quynh";
const char* WIFI_PASS = "tamsotam";
const char* SERVER_URL = "http://192.168.1.7:5000/upload";

// ============================================================
//  PIN
// ============================================================
#define BUTTON_PIN   0    // BOOT button
#define LED_PIN      2    // LED onboard
#define SDA_PIN      21
#define SCL_PIN      22

// ============================================================
//  CẤU HÌNH THU THẬP
// ============================================================
#define SAMPLE_RATE_HZ      25
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)   // 40ms
#define RECORD_DURATION_MS  5000                       // 5 giây
#define TOTAL_SAMPLES       (SAMPLE_RATE_HZ * RECORD_DURATION_MS / 1000)  // 125

// ============================================================
//  HÀM XỬ LÝ
// ============================================================
void startRecording();
void sendDataToServer();
void stopAndPrint();

// ============================================================
//  BUFFER
// ============================================================
struct Sample {
  float x, y, z;
};
Sample buffer[TOTAL_SAMPLES];
int sampleCount = 0;

// ============================================================
//  TRẠNG THÁI
// ============================================================
bool      isRecording    = false;
bool      btnLastState   = HIGH;
unsigned long recordStart = 0;
unsigned long lastSample  = 0;
int           fileIndex   = 1;   // Đếm số lần đo trong session

Adafruit_MPU6050 mpu;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN,    OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Tắt WiFi ngay từ đầu để tiết kiệm (không bắt buộc)
  WiFi.mode(WIFI_OFF);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("❌ Không tìm thấy MPU6050!");
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("========================================");
  Serial.println("   ESP32 + MPU6050 — Data Collector    ");
  Serial.println("========================================");
  Serial.println("Nhấn BOOT để bắt đầu thu thập 5 giây");
  Serial.println("Mỗi lần nhấn = 1 file CSV");
  Serial.println("========================================\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // ── Đọc nút BOOT (cạnh xuống) ─────────────────────────
  bool btnState = digitalRead(BUTTON_PIN);
  if (btnLastState == HIGH && btnState == LOW && !isRecording) {
    delay(50);  // debounce
    startRecording();
  }
  btnLastState = btnState;

  // ── Đang thu thập ──────────────────────────────────────
  if (isRecording) {
    unsigned long now = millis();

    // Lấy mẫu đúng 25Hz
    if (now - lastSample >= SAMPLE_INTERVAL_MS) {
      lastSample = now;

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      buffer[sampleCount].x = a.acceleration.x / 9.80665f;
      buffer[sampleCount].y = a.acceleration.y / 9.80665f;
      buffer[sampleCount].z = a.acceleration.z / 9.80665f;
      sampleCount++;
    }

    // Hết 5 giây → dừng
    if (millis() - recordStart >= RECORD_DURATION_MS) {
      stopAndPrint();
    }
  }
}

// ============================================================
//  BẮT ĐẦU THU THẬP
// ============================================================
void startRecording() {
  sampleCount   = 0;
  isRecording   = true;
  
  // Báo hiệu chuẩn bị: nhấp nháy LED nhanh 5 lần (tổng 2 giây)
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  recordStart   = millis();
  lastSample    = millis();
  digitalWrite(LED_PIN, HIGH);  // Bật LED trong suốt quá trình thu
  Serial.printf("🔴 [%02d] Đang thu thập...\n", fileIndex);
}

// ============================================================
//  KẾT NỐI WIFI VÀ GỬI DỮ LIỆU
// ============================================================
void sendDataToServer() {
  // Đảm bảo WiFi ở chế độ STA trước khi kết nối
  WiFi.mode(WIFI_STA);
  Serial.println("Kết nối wifi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
    
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
    
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ Không kết nối được wifi");
    WiFi.mode(WIFI_OFF);
    return;
  }

  Serial.println("\n✅ Kết nối được wifi");
  Serial.print("IP : ");
  Serial.println(WiFi.localIP());

  // Tạo JSON payload
  String payload = "{";
  payload += "\"filename\":\"data_" + String(fileIndex) + "\",";
  payload += "\"samples\":[";
  for (int i = 0; i < sampleCount; i++) {
    payload += "{\"x\":" + String(buffer[i].x, 6) +
                ",\"y\":" + String(buffer[i].y, 6) +
                ",\"z\":" + String(buffer[i].z, 6) + "}";
    if (i < sampleCount - 1) payload += ",";
  }
  payload += "]}";

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000); // Thêm timeout để tránh treo
  int httpCode = http.POST(payload);
  if (httpCode > 0) {
    Serial.printf("✅ Gửi thành công, HTTP code: %d\n", httpCode);
    // Có thể in response để kiểm tra
    // String response = http.getString();
    // Serial.println("Response: " + response);
  } else {
    Serial.printf("❌ Gửi thất bại : %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();

  // Tắt wifi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("📴 Tắt wifi");
}

// ============================================================
//  DỪNG VÀ XUẤT CSV
// ============================================================
void stopAndPrint() {
  isRecording = false;
  digitalWrite(LED_PIN, LOW);

  Serial.printf("⏹  Xong — %d samples\n", sampleCount);
  
  // Gửi dữ liệu qua WiFi
  sendDataToServer();

  Serial.printf("\n>>> Đã gửi file_%03d\n", fileIndex);
  Serial.println(">>> Nhấn BOOT để đo lần tiếp theo\n");
  Serial.println("----------------------------------------");

  fileIndex++;
}
