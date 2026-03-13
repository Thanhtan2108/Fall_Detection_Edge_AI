#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Fall_Detection_inferencing.h>

// ============================================================
//  PIN
// ============================================================
#define BUZZER_PIN   18
#define LED_PIN       2
#define SDA_PIN      21
#define SCL_PIN      22

// ============================================================
//  CẤU HÌNH
// ============================================================
#define SAMPLE_RATE_HZ      25
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 40ms
#define BUZZER_DURATION_MS  3000
#define FALL_THRESHOLD      0.70f

// Stride = 200ms = 5 samples × 3 axes = 15 floats
#define STRIDE_SAMPLES      5
#define STRIDE_FLOATS       (STRIDE_SAMPLES * 3)      // 15
#define WINDOW_FLOATS       EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // 150

// ============================================================
//  GLOBALS
// ============================================================
Adafruit_MPU6050 mpu;

float         features[WINDOW_FLOATS];
int           samples_collected = 0;    // đếm số samples từ 0 → 50
bool          buffer_full       = false;
int           stride_count      = 0;    // đếm samples sau mỗi stride

unsigned long last_sample_ms    = 0;
unsigned long buzzer_start_ms   = 0;
bool          buzzer_on         = false;

// ============================================================
//  CALLBACK
// ============================================================
int get_signal_data(size_t offset, size_t length, float* out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return EIDSP_OK;
}

void run_inference();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN,    OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_PIN,    LOW);
    digitalWrite(BUZZER_PIN, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        Serial.println("❌ Không tìm thấy MPU6050!");
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(1000);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    Serial.println("✅ Fall Detection Ready");
    Serial.printf("   Buffer size : %d floats (%d samples)\n",
                  WINDOW_FLOATS, WINDOW_FLOATS / 3);
    Serial.printf("   Sample rate : %d Hz\n", SAMPLE_RATE_HZ);
    Serial.printf("   Threshold   : %.0f%%\n", FALL_THRESHOLD * 100);
    Serial.println("   Đang tích lũy buffer lần đầu (2 giây)...");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // ── Tắt buzzer sau 3 giây ─────────────────────────────
    if (buzzer_on && now - buzzer_start_ms >= BUZZER_DURATION_MS) {
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN,    LOW);
        buzzer_on = false;
        Serial.println("🔕 Buzzer OFF");
    }

    // ── Giới hạn 25Hz ─────────────────────────────────────
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    last_sample_ms = now;

    // ── Đọc MPU6050 ───────────────────────────────────────
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float x = a.acceleration.x / 9.80665f;
    float y = a.acceleration.y / 9.80665f;
    float z = a.acceleration.z / 9.80665f;

    // ── Giai đoạn 1: Tích lũy buffer lần đầu ─────────────
    if (!buffer_full) {
        int idx = samples_collected * 3;
        features[idx]     = x;
        features[idx + 1] = y;
        features[idx + 2] = z;
        samples_collected++;

        if (samples_collected >= WINDOW_FLOATS / 3) {
            buffer_full = true;
            stride_count = 0;
            Serial.println("   Buffer đầy — bắt đầu inference!");
            run_inference();
        }
        return;
    }

    // ── Giai đoạn 2: Sliding window sau khi buffer đầy ────
    // Tích lũy đủ STRIDE_SAMPLES thì mới slide và inference
    // Lưu tạm vào cuối buffer (overwrite phần stride cũ)
    int write_idx = WINDOW_FLOATS - STRIDE_FLOATS
                    + stride_count * 3;
    features[write_idx]     = x;
    features[write_idx + 1] = y;
    features[write_idx + 2] = z;
    stride_count++;

    if (stride_count >= STRIDE_SAMPLES) {
        // Đủ 5 samples mới → slide buffer và inference
        memmove(features,
                features + STRIDE_FLOATS,
                (WINDOW_FLOATS - STRIDE_FLOATS) * sizeof(float));

        // Copy 5 samples mới vào cuối
        // (đã ghi đúng vị trí ở trên, memmove giữ nguyên thứ tự)
        stride_count = 0;
        run_inference();
    }
}

// ============================================================
//  INFERENCE
// ============================================================
void run_inference() {
    signal_t signal;
    signal.total_length = WINDOW_FLOATS;
    signal.get_data     = &get_signal_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    if (err != EI_IMPULSE_OK) {
        Serial.printf("❌ Classifier error: %d\n", err);
        return;
    }

    // Tìm class cao nhất
    float       max_val   = 0;
    const char* max_label = "";
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_val) {
            max_val   = result.classification[i].value;
            max_label = result.classification[i].label;
        }
    }

    // In kết quả
    Serial.print("→ ");
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("%s: %.0f%%  ",
            result.classification[i].label,
            result.classification[i].value * 100);
    }
    Serial.printf("| %s\n", max_label);

    // Kích hoạt buzzer nếu FALL
    if (strcmp(max_label, "Fall") == 0 && max_val >= FALL_THRESHOLD) {
        if (!buzzer_on) {
            Serial.printf("⚠️  FALL DETECTED (%.0f%%)!\n", max_val * 100);
            digitalWrite(BUZZER_PIN, HIGH);
            digitalWrite(LED_PIN,    HIGH);
            buzzer_on       = true;
            buzzer_start_ms = millis();
            Serial.println("🔔 Buzzer ON");
        }
    }
}
