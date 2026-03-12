/*
 * ============================================================
 *  ESP32 DevKit V1 + MPU6050 — FreeFall Data Logger
 *  Xuất CSV đúng chuẩn dataset Edge Impulse
 *
 *  Cấu trúc CSV output:
 *  Date;Timestamp;DeviceOrientation;AccelerationX;AccelerationY;AccelerationZ;Label
 *
 *  Thư viện cần cài (Library Manager):
 *    - Adafruit MPU6050
 *    - Adafruit Unified Sensor
 *    - NTPClient
 *    - ArduinoJson (optional)
 *
 *  Kết nối phần cứng:
 *    MPU6050  →  ESP32
 *    VCC      →  3.3V
 *    GND      →  GND
 *    SDA      →  GPIO 21
 *    SCL      →  GPIO 22
 *    AD0      →  GND  (I2C address = 0x68)
 *    INT      →  GPIO 19 (optional)
 * ============================================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>

// ============================================================
//  CẤU HÌNH WiFi (để lấy thời gian thực qua NTP)
// ============================================================
const char* WIFI_SSID     = "P203&P204&P205";
const char* WIFI_PASSWORD = "102duylam";

// ============================================================
//  CẤU HÌNH THU THẬP DATA
// ============================================================
#define SAMPLE_RATE_HZ      25        // Đúng với dataset gốc (25 Hz)
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 40ms

// Thời gian ghi mỗi file (giây) — dataset gốc ~38s/file
#define RECORD_DURATION_S   40

// Tổng số samples mỗi lần ghi
#define TOTAL_SAMPLES       (RECORD_DURATION_S * SAMPLE_RATE_HZ)  // 1000 samples

// Nút bấm để bắt đầu ghi
#define BUTTON_PIN          0   // BOOT button trên ESP32 DevKit V1
#define LED_PIN             2   // LED onboard

// ============================================================
//  THANG ĐO GIA TỐC: ±16g
//  (bắt buộc vì freeFall có thể đạt 10.6g khi chạm đất)
// ============================================================
#define ACCEL_RANGE         MPU6050_RANGE_16_G

// ============================================================
//  BIẾN TOÀN CỤC
// ============================================================
Adafruit_MPU6050 mpu;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7*3600, 60000); // UTC+7

struct SampleData {
  float accX, accY, accZ;
  unsigned long timestamp_ms;  // millis()
};

SampleData buffer[TOTAL_SAMPLES];
int sampleCount = 0;
bool isRecording = false;
unsigned long recordStartMillis = 0;
double epochStartTime = 0;  // Unix timestamp khi bắt đầu ghi

void startRecording();
void collectSample();
void stopRecordingAndPrint();

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("=== ESP32 + MPU6050 FreeFall Logger ===");

  // --- Khởi tạo MPU6050 ---
  Wire.begin(21, 22);  // SDA=21, SCL=22
  if (!mpu.begin()) {
    Serial.println("ERR: Không tìm thấy MPU6050! Kiểm tra kết nối.");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  Serial.println("OK: MPU6050 đã kết nối");

  // --- Cấu hình MPU6050 ---
  // Thang đo ±16g (bắt buộc cho freeFall)
  mpu.setAccelerometerRange(ACCEL_RANGE);

  // Gyro (không dùng nhưng cần init)
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Low Pass Filter: 10 Hz — lọc noise, phù hợp sampling 25Hz
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("Cấu hình MPU6050:");
  Serial.println("  - Accelerometer: ±16g");
  Serial.println("  - Low Pass Filter: 10 Hz");
  Serial.println("  - Sampling Rate: 25 Hz (40ms interval)");

  // --- Kết nối WiFi để lấy thời gian NTP ---
  Serial.print("Kết nối WiFi: ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int wifiRetry = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetry < 20) {
    delay(500);
    Serial.print(".");
    wifiRetry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK!");
    timeClient.begin();
    timeClient.update();
    Serial.print("Thời gian NTP: ");
    Serial.println(timeClient.getFormattedTime());
  } else {
    Serial.println(" THẤT BẠI — sẽ dùng millis() thay thế");
  }

  Serial.println("\n>>> Nhấn BOOT button để bắt đầu ghi data...");
  Serial.println(">>> LED sáng = đang ghi, tắt = xong");
}

// ============================================================
void loop() {
  // Chờ nhấn nút BOOT
  if (!isRecording && digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      startRecording();
    }
  }

  // Đang ghi data
  if (isRecording) {
    collectSample();

    // Kiểm tra đủ samples chưa
    if (sampleCount >= TOTAL_SAMPLES) {
      stopRecordingAndPrint();
    }
  }
}

// ============================================================
void startRecording() {
  sampleCount = 0;
  isRecording = true;
  digitalWrite(LED_PIN, HIGH);

  // Lấy Unix timestamp hiện tại
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    epochStartTime = (double)timeClient.getEpochTime();
  } else {
    // Fallback: dùng epoch giả (không có WiFi)
    epochStartTime = 1700000000.0;
  }
  recordStartMillis = millis();

  Serial.println("\n>>> BẮT ĐẦU GHI DATA...");
  Serial.print(">>> Thời gian ghi: ");
  Serial.print(RECORD_DURATION_S);
  Serial.println(" giây");
  Serial.print(">>> Tổng samples: ");
  Serial.println(TOTAL_SAMPLES);
}

// ============================================================
void collectSample() {
  static unsigned long lastSampleTime = 0;
  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (sampleCount < TOTAL_SAMPLES) {
      // Chuyển từ m/s² sang g (1g = 9.80665 m/s²)
      buffer[sampleCount].accX = a.acceleration.x / 9.80665;
      buffer[sampleCount].accY = a.acceleration.y / 9.80665;
      buffer[sampleCount].accZ = a.acceleration.z / 9.80665;
      buffer[sampleCount].timestamp_ms = now - recordStartMillis;
      sampleCount++;

      // Progress indicator mỗi 5 giây
      if (sampleCount % (5 * SAMPLE_RATE_HZ) == 0) {
        Serial.print(">>> Đã ghi: ");
        Serial.print(sampleCount);
        Serial.print("/");
        Serial.print(TOTAL_SAMPLES);
        Serial.print(" samples (");
        Serial.print(sampleCount / SAMPLE_RATE_HZ);
        Serial.println("s)");
      }
    }
  }
}

// ============================================================
void stopRecordingAndPrint() {
  isRecording = false;
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n>>> GHI XONG! Xuất CSV...");
  Serial.println(">>> Sao chép toàn bộ output bên dưới vào file .csv");
  Serial.println("=== BẮT ĐẦU CSV ===");
  delay(100);

  // In header — đúng định dạng dataset gốc
  Serial.println("Date;Timestamp;DeviceOrientation;AccelerationX;AccelerationY;AccelerationZ;Label");

  // In từng dòng data
  for (int i = 0; i < sampleCount; i++) {
    // Tính Unix timestamp cho sample này
    double sampleEpoch = epochStartTime + (buffer[i].timestamp_ms / 1000.0);

    // Tính Date string (YYYY-MM-DD HH:MM:SS)
    time_t t = (time_t)sampleEpoch;
    struct tm* tmInfo = gmtime(&t);  // UTC+7 đã cộng khi init NTPClient
    char dateStr[20];
    strftime(dateStr, sizeof(dateStr), "%Y-%m-%d %H:%M:%S", tmInfo);

    // In dòng CSV
    // Format: Date;Timestamp;DeviceOrientation;AccX;AccY;AccZ;Label
    Serial.print(dateStr);
    Serial.print(";");
    Serial.print(sampleEpoch, 7);   // 7 chữ số thập phân như dataset gốc
    Serial.print(";");
    Serial.print("portrait");        // DeviceOrientation cố định
    Serial.print(";");
    Serial.print(buffer[i].accX, 10);
    Serial.print(";");
    Serial.print(buffer[i].accY, 10);
    Serial.print(";");
    Serial.print(buffer[i].accZ, 10);
    Serial.print(";");              // Label trống — giống dataset gốc
    Serial.println();

    // Tránh watchdog reset khi in nhiều
    if (i % 100 == 0) delay(10);
  }

  Serial.println("=== KẾT THÚC CSV ===");
  Serial.println("\n>>> Nhấn BOOT button để ghi file tiếp theo...");
}
