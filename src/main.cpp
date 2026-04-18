#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// === Cấu hình chân và thông số cho pin ======================================
#define PIN_BATTERY     34
const float R1 = 97000.0;   // 97kΩ
const float R2 = 97000.0;   // 97kΩ
const float ADC_REF = 3.3;
const int ADC_RES = 4095;
const int NUM_SAMPLES = 10;

// Ngưỡng điện áp pin
const float BATTERY_FULL_VOLTAGE = 4.20;
const float BATTERY_EMPTY_VOLTAGE = 3.00;

// === Cấu hình cho màn hình OLED =============================================
#define SCREEN_WIDTH 128   // Độ rộng màn hình: 128 pixels
#define SCREEN_HEIGHT 64   // Độ cao màn hình: 64 pixels
#define OLED_RESET    -1   // Không dùng chân reset (nối với GND hoặc để trống)
#define SCREEN_ADDRESS 0x3C // Địa chỉ I2C của màn hình (thử 0x3D nếu không hoạt động)

// Khởi tạo đối tượng display với kích thước và địa chỉ I2C phù hợp[reference:5]
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Các hàm xử lý pin (giữ nguyên từ code của bạn) =========================
float readBatteryVoltage() {
    int rawSum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        rawSum += analogRead(PIN_BATTERY);
        delayMicroseconds(100);
    }
    int rawAdc = rawSum / NUM_SAMPLES;
    float vOut = (rawAdc / (float)ADC_RES) * ADC_REF;
    float vBat = vOut * (R1 + R2) / R2;
    return vBat;
}

float batteryPercentage(float voltage) {
    if (voltage >= BATTERY_FULL_VOLTAGE) return 100.0;
    if (voltage <= BATTERY_EMPTY_VOLTAGE) return 0.0;
    float percent = (voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100.0;
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

// === Hàm hiển thị thông tin lên OLED =======================================
void displayInfo(float voltage, float percent) {
    display.clearDisplay();           // Xóa nội dung cũ trên màn hình
    display.setTextSize(1);           // Cỡ chữ nhỏ (1)
    display.setTextColor(SSD1306_WHITE); // Màu chữ trắng

    // Dòng 1: Tiêu đề (căn giữa)
    display.setCursor(30, 0);
    display.println("SYSTEM STATUS");

    // Dòng 2: Điện áp pin
    display.setCursor(0, 16);
    display.print("Battery: ");
    display.print(voltage, 2);        // In số thực với 2 chữ số thập phân
    display.println(" V");

    // Dòng 3: Phần trăm pin (có thể thay bằng thanh đồ họa nếu muốn)
    display.setCursor(0, 32);
    display.print("Level: ");
    display.print(percent, 0);        // In số nguyên
    display.println("%");

    // Dòng 4: Trạng thái (ví dụ)
    display.setCursor(0, 48);
    if (percent < 20.0) {
        display.println("STATUS: LOW BATTERY!");
    } else {
        display.println("STATUS: NORMAL");
    }

    display.display();                // Gửi dữ liệu lên màn hình để hiển thị
}

// === Các hàm setup và loop =================================================
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    // Khởi tạo màn hình OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed!"));
        for(;;); // Dừng chương trình nếu không có màn hình
    }
    display.clearDisplay();
    Serial.println("OLED initialized successfully!");
}

void loop() {
    float vBat = readBatteryVoltage();
    float percent = batteryPercentage(vBat);
    
    // In ra Serial Monitor
    Serial.printf("Pin: %.2f V (%.1f%%)\n", vBat, percent);
    
    // Hiển thị lên OLED
    displayInfo(vBat, percent);
    
    delay(2000); // Cập nhật 2 giây một lần
}
