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
#define BUTTON_PIN          18          // nút nhấn (INPUT_PULLUP)

// ============================================================
//  SENSOR & AI CONFIGURATION
// ============================================================
#define SAMPLE_RATE_HZ      25
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 40ms
#define BUZZER_DURATION_MS  3000
#define FALL_THRESHOLD      0.99f
#define NORMAL_THRESHOLD    0.60f
#define STRIDE_SAMPLES      5
#define STRIDE_FLOATS       (STRIDE_SAMPLES * 3)      // 15
#define WINDOW_FLOATS       EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE // 62 x 3

// ============================================================
//  WI-FI CONFIGURATION
// ============================================================
const char* WIFI_SSID = "Thanhtan";
const char* WIFI_PASS = "12345678";
const int   WIFI_TIMEOUT = 10000;  // 10 giây timeout kết nối (non-blocking)

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

bool alert_sent_for_current_fall = false;
bool wifi_enabled = false;

// ================= NÚT NHẤN (DEBOUNCE) =================
bool lastButtonState = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_TIME = 50;

// ================= WI-FI STATE MACHINE (non-blocking) =================
enum WiFiState {
    WIFI_IDLE,
    WIFI_CONNECTING,
    WIFI_BLINKING,
    WIFI_DONE
};
WiFiState wifiState = WIFI_IDLE;
unsigned long wifiStateStartTime = 0;
int blinkCounter = 0;          // 0..9 (5 lần sáng + 5 lần tắt)

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
int  get_signal_data(size_t offset, size_t length, float* out_ptr);
void run_inference();
void initWiFi();
void enableWiFi();
void disableWiFi();
void handleButton();
void wifiTask();               // state machine cho WiFi
void disconnectWiFi();

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
    pinMode(BUTTON_PIN, INPUT_PULLUP);
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

    handleButton();               // luôn xử lý nút nhấn
    wifiTask();                   // xử lý WiFi không blocking

    // Tắt buzzer nếu hết thời gian
    if (buzzer_on && (now - buzzer_start_ms >= BUZZER_DURATION_MS)) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzer_on = false;
        Serial.println("Buzzer turned off (timeout)");
    }

    // Đảm bảo tốc độ lấy mẫu 25Hz
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    last_sample_ms = now;

    // Đọc dữ liệu MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float x = a.acceleration.x / 9.80665f;
    float y = a.acceleration.y / 9.80665f;
    float z = a.acceleration.z / 9.80665f;

    // Giai đoạn 1: tích lũy buffer lần đầu
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

    // Giai đoạn 2: sliding window
    int write_idx = WINDOW_FLOATS - STRIDE_FLOATS + stride_count * 3;
    features[write_idx]     = x;
    features[write_idx + 1] = y;
    features[write_idx + 2] = z;
    stride_count++;

    if (stride_count >= STRIDE_SAMPLES) {
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

    float       max_confidence = 0;
    const char* predicted_label = "";
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_confidence) {
            max_confidence = result.classification[i].value;
            predicted_label = result.classification[i].label;
        }
    }

    Serial.print("→ ");
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("%s: %.0f%%  ", result.classification[i].label, result.classification[i].value * 100);
    }
    Serial.printf("| %s\n", predicted_label);

    // Xử lý khi phát hiện té ngã
    if (strcmp(predicted_label, "Fall") == 0 && max_confidence >= FALL_THRESHOLD) {
        // Bật còi lần đầu của sự kiện té ngã
        if (!buzzer_on && !alert_sent_for_current_fall) {
            Serial.printf("FALL DETECTED (%.0f%%)!\n", max_confidence * 100);
            digitalWrite(BUZZER_PIN, HIGH);
            buzzer_on = true;
            buzzer_start_ms = millis();
        }

        // Gửi cảnh báo WiFi chỉ một lần (kích hoạt state machine)
        if (!alert_sent_for_current_fall) {
            alert_sent_for_current_fall = true;
            // Chỉ khởi tạo nếu WiFi chưa hoạt động
            if (wifiState == WIFI_IDLE) {
                enableWiFi();
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                wifiState = WIFI_CONNECTING;
                wifiStateStartTime = millis();
                Serial.println("WiFi connecting (non-blocking)...");
            }
        }
    }
    else {
        // Reset trạng thái cảnh báo khi bình thường (confidence > 60%)
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
//  WI-FI MANAGEMENT (non-blocking state machine)
// ============================================================
void initWiFi() {
    WiFi.mode(WIFI_OFF);
    wifi_enabled = false;
    Serial.println("WiFi initialized in OFF mode");
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
        Serial.println("WiFi modem disabled");
    }
}

void wifiTask() {
    unsigned long now = millis();

    switch (wifiState) {
        case WIFI_IDLE:
            // không làm gì
            break;

        case WIFI_CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("WiFi connected");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                // Chuyển sang trạng thái nháy LED (5 lần)
                wifiState = WIFI_BLINKING;
                wifiStateStartTime = now;
                blinkCounter = 0;
                digitalWrite(LED_SEND_WIFI, HIGH);  // bắt đầu sáng
            } else if (now - wifiStateStartTime > WIFI_TIMEOUT) {
                Serial.println("WiFi connection failed");
                wifiState = WIFI_DONE;  // kết thúc, sẽ tắt modem
            }
            break;

        case WIFI_BLINKING:
            // Nháy 5 lần: mỗi lần sáng 100ms, tắt 100ms -> tổng 1 chu kỳ 200ms
            // 5 lần = 10 nửa chu kỳ (blinkCounter từ 0 đến 9)
            if (blinkCounter < 10) {
                if (now - wifiStateStartTime >= 100) {
                    // Đảo trạng thái LED
                    digitalWrite(LED_SEND_WIFI, !digitalRead(LED_SEND_WIFI));
                    wifiStateStartTime = now;
                    blinkCounter++;
                }
            } else {
                // Kết thúc nháy, tắt LED
                digitalWrite(LED_SEND_WIFI, LOW);
                wifiState = WIFI_DONE;
            }
            break;

        case WIFI_DONE:
            disconnectWiFi();  // gọi disconnect và tắt modem
            disableWiFi();
            wifiState = WIFI_IDLE;
            break;
    }
}

void disconnectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect();
        Serial.println("WiFi disconnected");
    }
}

// ============================================================
//  BUTTON HANDLER (TOGGLE BUZZER)
// ============================================================
void handleButton() {
    unsigned long now = millis();
    bool currentState = digitalRead(BUTTON_PIN);

    if (currentState != lastButtonState) {
        lastDebounceTime = now;
    }
    lastButtonState = currentState;

    if (now - lastDebounceTime > DEBOUNCE_TIME) {
        if (currentState != stableButtonState) {
            stableButtonState = currentState;
            if (stableButtonState == LOW) {  // nhấn nút
                Serial.println(">>> Button pressed - Toggle buzzer");
                buzzer_on = !buzzer_on;
                if (buzzer_on) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    buzzer_start_ms = millis();
                    Serial.println("Buzzer turned ON by button");
                } else {
                    digitalWrite(BUZZER_PIN, LOW);
                    Serial.println("Buzzer turned OFF by button");
                }
            }
        }
    }
}
