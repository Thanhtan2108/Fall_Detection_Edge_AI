#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Fall_Detection_inferencing.h>
#include <HardwareSerial.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define LED_WARNING         LED_BUILTIN
#define BUZZER_PIN          14
#define SDA_PIN             21
#define SCL_PIN             22
#define BUTTON_PIN          18          // nút nhấn (INPUT_PULLUP)
#define SIM_RX_PIN          16
#define SIM_TX_PIN          17

// ============================================================
//  SENSOR & AI CONFIGURATION
// ============================================================
#define SAMPLE_RATE_HZ      25
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 40ms
#define FALL_THRESHOLD      0.99f
#define NORMAL_THRESHOLD    0.60f
#define STRIDE_SAMPLES      5
#define STRIDE_FLOATS       (STRIDE_SAMPLES * 3)      // 15
#define WINDOW_FLOATS       EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE // 62 x 3

// ============================================================
//  SIM CONFIGURATION
// ============================================================
#define PHONE_NUMBER        "+84833538486"
#define SMS_MESSAGE         "CANH BAO: Phat hien te nga! Can kiem tra ngay!"

constexpr unsigned long NETWORK_READY_TIMEOUT = 60000;
constexpr unsigned long AT_RESPONSE_TIMEOUT   = 1000;
constexpr unsigned long SIM_STATE_TIMEOUT_SMS = 15000;
constexpr unsigned long SIM_STATE_TIMEOUT_ATD = 10000;
constexpr unsigned long SIM_STATE_TIMEOUT_ATH = 3000;

// ============================================================
//  SIM STATE MACHINE
// ============================================================
enum SimState {
    SIM_IDLE,
    SIM_SMS_INIT,       // Gửi AT+CMGF=1
    SIM_SMS_SEND,       // Gửi AT+CMGS="số"
    SIM_SMS_BODY,       // Gửi nội dung + Ctrl+Z
    SIM_SMS_WAIT,       // Chờ +CMGS: hoặc ERROR
    SIM_CALL_DIAL,      // Gửi ATD+84xxx;
    SIM_CALL_ACTIVE,    // Chờ URC (bắt máy / bận / hết giờ)
    SIM_HANGUP          // Gửi ATH
};

// ============================================================
//  GLOBAL VARIABLES — MPU / AI
// ============================================================
Adafruit_MPU6050 mpu;

float         features[WINDOW_FLOATS];
int           samples_collected = 0;
bool          buffer_full       = false;
int           stride_count      = 0;
unsigned long last_sample_ms    = 0;

bool buzzer_on                   = false;
bool alert_sent_for_current_fall = false;

// ============================================================
//  NÚT NHẤN (DEBOUNCE)
// ============================================================
bool                    lastButtonState         = HIGH;
bool                    stableButtonState       = HIGH;
unsigned long           lastDebounceTime        = 0;
constexpr unsigned long DEBOUNCE_DELAY_MS       = 50;

// ============================================================
//  GLOBAL VARIABLES — SIM
// ============================================================
HardwareSerial simSerial(2);

SimState      simState          = SIM_IDLE;
unsigned long simStateStartTime = 0;
String        simRxBuffer       = "";
String        urcLineBuffer     = "";
bool          moduleReady       = false;
bool          callInProgress    = false;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
int  get_signal_data(size_t offset, size_t length, float* out_ptr);
void run_inference();
void handleButton();

void simTask();
void handleURC(const String& line);
void triggerAlert();                    // Gọi khi AI phát hiện Fall

String readModuleResponse(unsigned long timeoutMs);  // chỉ dùng trong setup
bool   waitForNetworkReady();
void   cleanSerialBuffer();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // GPIO
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    // MPU6050
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found! Halting.");
        while (true);
    }    
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    Serial.println("MPU6050 ready.");

    // SIM module — blocking chỉ tại đây, chấp nhận được
    simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
    Serial.print("Waiting for SIM module network...");
    moduleReady = waitForNetworkReady();
    if (moduleReady) {
        Serial.println("\nSIM module ready.");
    } else {
        Serial.println("\nSIM module WARNING: network not ready after 60s. Will retry on alert.");
    }
    cleanSerialBuffer();

    Serial.println("Fall Detection System Ready.");
    Serial.println("Filling initial buffer...");
}

// ============================================================
//  MAIN LOOP — hoàn toàn non-blocking
// ============================================================
void loop() {
    unsigned long now = millis();

    // 1. Xử lý URC từ SIM module (đọc từng ký tự, không chờ)
    while (simSerial.available()) {
        char c = simSerial.read();
        simRxBuffer += c;
        urcLineBuffer += c;
        if (c == '\n') {
            handleURC(urcLineBuffer);
            urcLineBuffer = "";
        }
    }

    // 2. Tiến state machine SIM thêm 1 bước
    simTask();

    // 3. Xử lý nút nhấn — luôn phản hồi bất kể SIM đang làm gì
    handleButton();

    // 4. Lấy mẫu MPU6050 và chạy inference đúng 25Hz
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    last_sample_ms = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float x = a.acceleration.x / 9.80665f;
    float y = a.acceleration.y / 9.80665f;
    float z = a.acceleration.z / 9.80665f;

    if (!buffer_full) {
        int idx = samples_collected * 3;
        features[idx]     = x;
        features[idx + 1] = y;
        features[idx + 2] = z;
        samples_collected++;
        if (samples_collected >= WINDOW_FLOATS / 3) {
            buffer_full  = true;
            stride_count = 0;
            Serial.println("Buffer full — starting inference!");
            run_inference();
        }
        return;
    }

    int write_idx = WINDOW_FLOATS - STRIDE_FLOATS + stride_count * 3;
    features[write_idx]     = x;
    features[write_idx + 1] = y;
    features[write_idx + 2] = z;
    stride_count++;

    if (stride_count >= STRIDE_SAMPLES) {
        memmove(features, features + STRIDE_FLOATS,
                (WINDOW_FLOATS - STRIDE_FLOATS) * sizeof(float));
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
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

    float       max_conf  = 0;
    const char* label     = "";
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_conf) {
            max_conf = result.classification[i].value;
            label    = result.classification[i].label;
        }
    }

    Serial.print("→ ");
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("%s: %.0f%%  ", result.classification[i].label,
                      result.classification[i].value * 100);
    }
    Serial.printf("| %s\n", label);

    if (strcmp(label, "Fall") == 0 && max_conf >= FALL_THRESHOLD) {
        if (!alert_sent_for_current_fall) {
            alert_sent_for_current_fall = true;
            Serial.printf("FALL DETECTED (%.0f%%)!\n", max_conf * 100);
            triggerAlert();
        }
    } else {
        if (max_conf >= NORMAL_THRESHOLD) {
            alert_sent_for_current_fall = false;
        }
    }
}

int get_signal_data(size_t offset, size_t length, float* out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return EIDSP_OK;
}

// ============================================================
//  TRIGGER ALERT — điểm khởi động toàn bộ chuỗi cảnh báo
// ============================================================
void triggerAlert() {
    // 1. Bật còi NGAY LẬP TỨC — không blocking
    if (!buzzer_on) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzer_on = true;
        Serial.println(">> BUZZER ON");
    }

    // 2. Khởi động SIM state machine nếu đang rảnh
    if (simState == SIM_IDLE) {
        simRxBuffer       = "";
        simStateStartTime = millis();
        simState          = SIM_SMS_INIT;
        Serial.println(">> SIM: Starting alert sequence (SMS → Call)");
    }
    // Nếu simState != IDLE (đang xử lý sự kiện trước) → bỏ qua,
    // còi đã kêu là đủ cho sự kiện này
}

// ============================================================
//  SIM STATE MACHINE — fix hoàn chỉnh
// ============================================================

bool simCmdSent = false;  // thêm biến này vào global

void simTask() {
    unsigned long now = millis();

    switch (simState) {

        case SIM_SMS_INIT:
            if (!simCmdSent) {
                simRxBuffer = "";
                simSerial.println("AT+CMGF=1");
                simStateStartTime = now;
                simCmdSent = true;
                Serial.println(">> SIM: AT+CMGF=1 sent");
            }
            if (simRxBuffer.indexOf("OK") >= 0) {
                Serial.println(">> SIM: CMGF OK");
                simCmdSent = false;
                simRxBuffer = "";
                simStateStartTime = now;
                simState = SIM_SMS_SEND;
            } else if (now - simStateStartTime > 3000) {
                Serial.println(">> SIM: CMGF timeout, retry");
                simCmdSent = false;
            }
            break;

        case SIM_SMS_SEND:
            if (!simCmdSent) {
                simRxBuffer = "";
                simSerial.println("AT+CMGS=\"" PHONE_NUMBER "\"");
                simStateStartTime = now;
                simCmdSent = true;
                Serial.println(">> SIM: AT+CMGS sent, waiting '>'");
            }
            if (simRxBuffer.indexOf('>') >= 0) {
                Serial.println(">> SIM: Got '>'. Sending body...");
                simCmdSent = false;
                simRxBuffer = "";
                simStateStartTime = now;
                simState = SIM_SMS_BODY;
            } else if (now - simStateStartTime > 5000) {
                Serial.println(">> SIM: No '>' timeout, skip to call");
                simCmdSent = false;
                simRxBuffer = "";
                simStateStartTime = now;
                simState = SIM_CALL_DIAL;
            }
            break;

        case SIM_SMS_BODY:
            if (!simCmdSent) {
                simSerial.print(SMS_MESSAGE);
                delay(50);
                simSerial.write(0x1A);
                simStateStartTime = now;
                simCmdSent = true;
                Serial.println(">> SIM: SMS body + Ctrl+Z sent, waiting +CMGS");
            }
            if (simRxBuffer.indexOf("+CMGS:") >= 0) {
                Serial.println(">> SIM: SMS sent OK!");
                simCmdSent = false;
                simRxBuffer = "";
                simStateStartTime = now;
                simState = SIM_CALL_DIAL;
            } else if (simRxBuffer.indexOf("ERROR") >= 0 ||
                       now - simStateStartTime > SIM_STATE_TIMEOUT_SMS) {
                Serial.println(">> SIM: SMS failed/timeout, proceed to call");
                simCmdSent = false;
                simRxBuffer = "";
                simStateStartTime = now;
                simState = SIM_CALL_DIAL;
            }
            break;

        case SIM_CALL_DIAL:
            if (!simCmdSent) {
                simRxBuffer = "";
                simSerial.println("ATD" PHONE_NUMBER ";");
                simStateStartTime = now;
                simCmdSent = true;
                Serial.println(">> SIM: ATD sent, dialing...");
            }
            if (simRxBuffer.indexOf("OK") >= 0) {
                Serial.println(">> SIM: Call dialing OK");
                simCmdSent = false;
                simRxBuffer = "";
                simStateStartTime = now;
                simState = SIM_CALL_ACTIVE;
            } else if (now - simStateStartTime > SIM_STATE_TIMEOUT_ATD) {
                Serial.println(">> SIM: ATD timeout");
                simCmdSent = false;
                simRxBuffer = "";
                simState = SIM_IDLE;
            }
            break;

        case SIM_CALL_ACTIVE:
            // Kết quả xử lý qua handleURC()
            // Timeout 60s phòng trường hợp URC không đến
            if (now - simStateStartTime > 60000UL) {
                Serial.println(">> SIM: Call timeout, hanging up");
                simCmdSent = false;
                simState = SIM_HANGUP;
            }
            break;

        case SIM_HANGUP:
            if (!simCmdSent) {
                simSerial.println("ATH");
                simStateStartTime = now;
                simCmdSent = true;
                Serial.println(">> SIM: ATH sent");
            }
            if (simRxBuffer.indexOf("OK") >= 0 ||
                now - simStateStartTime > SIM_STATE_TIMEOUT_ATH) {
                Serial.println(">> SIM: Hangup done. IDLE.");
                simCmdSent = false;
                simRxBuffer = "";
                callInProgress = false;
                simState = SIM_IDLE;
            }
            break;

        case SIM_IDLE:
        default:
            break;
    }
}

// ============================================================
//  URC HANDLER — nhận thông báo chủ động từ module
// ============================================================
void handleURC(const String& raw) {
    String line = raw;
    line.trim();
    if (line.isEmpty()) return;

    if (line.indexOf("VOICE CALL: BEGIN") >= 0) {
        Serial.println(">> URC: Call connected.");
        callInProgress = true;
        // Đang ở SIM_CALL_ACTIVE — giữ nguyên, chờ kết thúc
    }
    else if (line.indexOf("VOICE CALL: END") >= 0 ||
             line.indexOf("NO CARRIER")      >= 0) {
        Serial.println(">> URC: Call ended.");
        callInProgress = false;
        if (simState == SIM_CALL_ACTIVE) simState = SIM_IDLE;
    }
    else if (line.indexOf("BUSY") >= 0) {
        Serial.println(">> URC: Busy.");
        callInProgress = false;
        if (simState == SIM_CALL_ACTIVE) simState = SIM_IDLE;
    }
    else if (line.indexOf("NO ANSWER") >= 0) {
        Serial.println(">> URC: No answer.");
        callInProgress = false;
        if (simState == SIM_CALL_ACTIVE) simState = SIM_IDLE;
    }
}

// ============================================================
//  BUTTON HANDLER
//  - Tắt còi ngay lập tức
//  - KHÔNG hủy SMS/Call (ưu tiên thông báo tuyệt đối)
//  - Nếu đang có cuộc gọi active → hangup sau khi còi tắt
// ============================================================
void handleButton() {
    unsigned long now = millis();
    bool reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) lastDebounceTime = now;
    lastButtonState = reading;

    if (now - lastDebounceTime > DEBOUNCE_DELAY_MS) {
        if (reading != stableButtonState) {
            stableButtonState = reading;
            if (stableButtonState == LOW) {
                Serial.println(">>> Button pressed");

                if (buzzer_on) {
                    // Tắt còi
                    digitalWrite(BUZZER_PIN, LOW);
                    buzzer_on = false;
                    alert_sent_for_current_fall = true;
                    Serial.println(">> BUZZER OFF by button");

                    // Nếu cuộc gọi đang active thì cúp máy
                    if (callInProgress) {
                        Serial.println(">> Hanging up active call...");
                        simState = SIM_HANGUP;
                    }
                } else {
                    // Bật còi thủ công
                    digitalWrite(BUZZER_PIN, HIGH);
                    buzzer_on = true;
                    Serial.println(">> BUZZER ON by button");
                }
            }
        }
    }
}

// ============================================================
//  SIM UTILITIES — chỉ dùng trong setup (blocking OK)
// ============================================================
void cleanSerialBuffer() {
    while (simSerial.available()) simSerial.read();
    simRxBuffer   = "";
    urcLineBuffer = "";
}

String readModuleResponse(unsigned long timeoutMs) {
    String resp;
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        while (simSerial.available()) {
            char c = simSerial.read();
            resp += c;
        }
        delay(5);
    }
    return resp;
}

bool waitForNetworkReady() {
    unsigned long start = millis();
    bool gsmOk = false, lteOk = false;
    while (millis() - start < NETWORK_READY_TIMEOUT) {
        simSerial.println("AT+CREG?");
        String r = readModuleResponse(AT_RESPONSE_TIMEOUT);
        if (r.indexOf("+CREG: 0,1") >= 0 || r.indexOf("+CREG: 0,5") >= 0) gsmOk = true;

        simSerial.println("AT+CEREG?");
        r = readModuleResponse(AT_RESPONSE_TIMEOUT);
        if (r.indexOf("+CEREG: 0,1") >= 0 || r.indexOf("+CEREG: 0,5") >= 0) lteOk = true;

        if (gsmOk && lteOk) return true;
        delay(1000);
        Serial.print(".");
    }
    return false;
}
