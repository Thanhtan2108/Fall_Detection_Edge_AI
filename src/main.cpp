// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
  recordStart   = millis();
  lastSample    = millis();

  digitalWrite(LED_PIN, HIGH);
  Serial.printf("🔴 [%02d] Đang thu thập...\n", fileIndex);
}

// ============================================================
//  DỪNG VÀ XUẤT CSV
// ============================================================
void stopAndPrint() {
  isRecording = false;
  digitalWrite(LED_PIN, LOW);

  Serial.printf("⏹  Xong — %d samples\n\n", sampleCount);

  // Header
  Serial.println("=== BEGIN CSV ===");
  Serial.println("AccelerationX;AccelerationY;AccelerationZ");

  // Data
  for (int i = 0; i < sampleCount; i++) {
    Serial.print(buffer[i].x, 6);  Serial.print(";");
    Serial.print(buffer[i].y, 6);  Serial.print(";");
    Serial.println(buffer[i].z, 6);
  }

  Serial.println("=== END CSV ===");
  Serial.printf("\n>>> Lưu thành file_%02d.csv\n", fileIndex);
  Serial.println(">>> Nhấn BOOT để đo lần tiếp theo\n");
  Serial.println("----------------------------------------");

  fileIndex++;
}
