#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Fall_Detection_inferencing.h>
#include <WiFi.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define BUZZER_PIN          14
#define LED_SEND_WIFI       2
#define LED_MPU6050_ACTIVE  13
#define SDA_PIN             21
#define SCL_PIN             22

// ============================================================
//  SENSOR & AI CONFIGURATION
// ============================================================
#define SAMPLE_RATE_HZ      25
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 40msN
#define BUZZER_DURATION_MS  3000
#define FALL_THRESHOLD      0.99f
#define NORMAL_THRESHOLD    0.60f
#define STRIDE_SAMPLES      5
#define STRIDE_FLOATS       (STRIDE_SAMPLES * 3)      // 15
#define WINDOW_FLOATS       EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE // 62 x 3

// ============================================================
//  WI-FI CONFIGURATION
// ============================================================
const char* WIFI_SSID = "Thanh Tan";
const char* WIFI_PASS = "daquenmatroi";
const int   WIFI_TIMEOUT = 10000;  // 10 giây timeout kết nối

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
Adafruit_MPU6050 mpu;

float features[WINDOW_FLOATS];
int   samples_collected = 0;
bool  buffer_full       = false;
int   stride_count      = 0;

unsigned long last_sample_ms    = 0;
unsigned long buzzer_start_ms   = 0;
bool          buzzer_on         = false;

// Biến trạng thái gửi cảnh báo
bool alert_sent_for_current_fall = false;
bool wifi_enabled = false;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
int  get_signal_data(size_t offset, size_t length, float* out_ptr);
void run_inference();
void initWiFi();
bool connectWiFi();
void disconnectWiFi();
void enableWiFi();
void disableWiFi();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.printf("CPU frequency set to %d MHz\n", getCpuFrequencyMhz());

    pinMode(LED_SEND_WIFI, OUTPUT);
    pinMode(LED_MPU6050_ACTIVE, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_SEND_WIFI, LOW);
    digitalWrite(LED_MPU6050_ACTIVE, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    // Khởi tạo I2C và MPU6050
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        while (true) {
            digitalWrite(LED_MPU6050_ACTIVE, LOW);
        }
    } else {
        digitalWrite(LED_MPU6050_ACTIVE, HIGH);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    // WiFi ở chế độ tắt ngay từ đầu
    initWiFi();

    Serial.println("Fall Detection System Ready");
    Serial.printf("Buffer size : %d floats (%d samples)\n", WINDOW_FLOATS, WINDOW_FLOATS / 3);
    Serial.printf("Sample rate : %d Hz\n", SAMPLE_RATE_HZ);
    Serial.printf("Fall threshold: %.0f%%\n", FALL_THRESHOLD * 100);
    Serial.println("Filling initial buffer (2 seconds)...");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // Xử lý tắt buzzer
    if (buzzer_on && (now - buzzer_start_ms >= BUZZER_DURATION_MS)) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzer_on = false;
        Serial.println("Buzzer turned off");
    }

    // Đảm bảo tốc độ lấy mẫu 25Hz
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    last_sample_ms = now;

    // Đọc dữ liệu từ MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Chuyển đổi sang đơn vị G (trọng lực)
    float x = a.acceleration.x / 9.80665f;
    float y = a.acceleration.y / 9.80665f;
    float z = a.acceleration.z / 9.80665f;

    // ----- GIAI ĐOẠN 1: TÍCH LŨY BUFFER LẦN ĐẦU -----
    if (!buffer_full) {
        int idx = samples_collected * 3;
        features[idx]     = x;
        features[idx + 1] = y;
        features[idx + 2] = z;
        samples_collected++;

        if (samples_collected >= WINDOW_FLOATS / 3) {
            buffer_full = true;
            stride_count = 0;
            Serial.println("Buffer full — starting inference!");
            run_inference();
        }
        return;
    }

    // ----- GIAI ĐOẠN 2: SLIDING WINDOW -----
    int write_idx = WINDOW_FLOATS - STRIDE_FLOATS + stride_count * 3;
    features[write_idx]     = x;
    features[write_idx + 1] = y;
    features[write_idx + 2] = z;
    stride_count++;

    if (stride_count >= STRIDE_SAMPLES) {
        // Dịch chuyển buffer: loại bỏ STRIDE_FLOATS phần tử đầu
        memmove(features, features + STRIDE_FLOATS, (WINDOW_FLOATS - STRIDE_FLOATS) * sizeof(float));

        stride_count = 0;
        run_inference();
    }
}

// ============================================================
//  INFERENCE FUNCTION
// ============================================================
void run_inference() {
    signal_t signal;
    signal.total_length = WINDOW_FLOATS;
    signal.get_data     = &get_signal_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    if (err != EI_IMPULSE_OK) {
        Serial.printf("Classifier error: %d\n", err);
        return;
    }

    // Tìm class có confidence cao nhất
    float       max_confidence = 0;
    const char* predicted_label = "";
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_confidence) {
            max_confidence = result.classification[i].value; // kết quả xác xuất chính xác của mỗi lần dự đoán ~ softmax
            predicted_label = result.classification[i].label;
        }
    }

    // In kết quả ra Serial để theo dõi
    Serial.print("→ ");
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("%s: %.0f%%  ", result.classification[i].label, result.classification[i].value * 100);
    }
    Serial.printf("| %s\n", predicted_label);

    // ----- XỬ LÝ KHI PHÁT HIỆN TÉ NGÃ -----
    if (strcmp(predicted_label, "Fall") == 0 && max_confidence >= FALL_THRESHOLD) {
        // Bật còi nếu chưa bật VÀ chưa gửi cảnh báo (lần đầu tiên của sự kiện)
        if (!buzzer_on && !alert_sent_for_current_fall) {
            Serial.printf("FALL DETECTED (%.0f%%)!\n", max_confidence * 100);
            digitalWrite(BUZZER_PIN, HIGH);
            buzzer_on = true;
            buzzer_start_ms = millis();
        }

        // Gửi tín hiệu cảnh báo qua WiFi (chỉ một lần)
        if (!alert_sent_for_current_fall) {
            alert_sent_for_current_fall = true;
            enableWiFi();
            if (connectWiFi()) {
                Serial.println("WiFi connected, blinking LED 5 times...");
                for (int i = 0; i < 5; i++) {
                    digitalWrite(LED_SEND_WIFI, HIGH);
                    delay(500);
                    digitalWrite(LED_SEND_WIFI, LOW);
                    delay(500);
                }
                disconnectWiFi();
            } else {
                Serial.println("Không thể kết nối WiFi để gửi cảnh báo");
            }
            disableWiFi();
        }
    }
    else {
        // Khi không phải Fall, reset trạng thái gửi cảnh báo nếu confidence > 50%
        if (max_confidence >= NORMAL_THRESHOLD) {
            alert_sent_for_current_fall = false;
        }
    }
}

// ============================================================
//  CALLBACK DATA PROVIDER FOR EDGE IMPULSE
// ============================================================
int get_signal_data(size_t offset, size_t length, float* out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return EIDSP_OK;
}

// ============================================================
//  WI-FI MANAGEMENT FUNCTIONS
// ============================================================
void initWiFi() {
    WiFi.mode(WIFI_OFF);
    wifi_enabled = false;
    Serial.println("WiFi initialized in OFF mode (power saving)");
}

void enableWiFi() {
    if (!wifi_enabled) {
        WiFi.mode(WIFI_STA);
        wifi_enabled = true;
        Serial.println("WiFi modem enabled");
    }
}

void disableWiFi() {
    if (wifi_enabled) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        wifi_enabled = false;
        Serial.println("WiFi modem disabled (power saving)");
    }
}

bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.printf("Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nWiFi connection failed");
        return false;
    }
}

void disconnectWiFi() {
    WiFi.disconnect();
    Serial.println("WiFi disconnected");
}
