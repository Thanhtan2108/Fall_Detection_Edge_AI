#include <Arduino.h>
#include <HardwareSerial.h>

// ========== CẤU HÌNH PHẦN CỨNG ==========
#define SIM_RX_PIN 16      // Chân RX2 (GPIO16) -> TX của A7680C
#define SIM_TX_PIN 17      // Chân TX2 (GPIO17) -> RX của A7680C
#define BUTTON_PIN 13      // Chân nút nhấn (active LOW)

// Số điện thoại nhận SMS (định dạng quốc tế)
#define PHONE_NUMBER "+84327524504"

// ========== BIẾN TOÀN CỤC ==========
HardwareSerial simSerial(2);

bool smsInProgress = false;           // Cờ báo đang gửi SMS, tránh gửi chồng lệnh

unsigned long lastDebounceTime = 0;   // Thời điểm thay đổi trạng thái nút gần nhất
unsigned long debounceDelay = 50;     // Khoảng thời gian chống dội (50ms)
int lastButtonState = HIGH;           // Trạng thái trước đó của nút
int buttonState = HIGH;               // Trạng thái hiện tại của nút (đã được ổn định)

void sendSMS(String number, String message);
void printResponse();

void setup() {
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

  // Cấu hình nút nhấn: dùng điện trở kéo lên nội (INPUT_PULLUP)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("ESP32 UART Bridge + SMS Button da san sang.");
  Serial.println("Nhan nut (GPIO13 -> GND) de gui SMS.");
  Serial.println("Ban van co the go lenh AT thu cong nhu binh thuong.\n");

  // Đợi module khởi động (tự động nhờ mạch auto-power)
  delay(10000);
  Serial.println(">> Module A7680C da san sang.\n");
}

void loop() {
  // ---------- XỬ LÝ NÚT NHẤN (CÓ CHỐNG DỘI) ----------
  int reading = digitalRead(BUTTON_PIN);

  // Nếu trạng thái đọc khác với trạng thái trước đó, reset thời gian debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Nếu đã qua thời gian debounce, cập nhật trạng thái nút ổn định
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Nếu trạng thái ổn định thay đổi so với trước đó
    if (reading != buttonState) {
      buttonState = reading;

      // Chỉ kích hoạt khi nút được nhấn (LOW) và không đang gửi SMS
      if (buttonState == LOW && !smsInProgress) {
        Serial.println("\n>> Nut duoc nhan! Dang gui SMS...");
        sendSMS(PHONE_NUMBER, "Canh bao: Co nguoi nhan nut!");
      }
    }
  }
  lastButtonState = reading;

  // ---------- CẦU NỐI UART (GIỮ NGUYÊN) ----------
  if (simSerial.available()) {
    while (simSerial.available()) {
      Serial.write(simSerial.read());
    }
  }

  if (Serial.available()) {
    String command = Serial.readString();
    simSerial.print(command);
  }
}

/**
 * Hàm gửi tin nhắn SMS ở chế độ Text
 */
void sendSMS(String number, String message) {
  smsInProgress = true;   // Đặt cờ để không bị gửi đè

  // Bước 1: Chọn chế độ Text
  simSerial.println("AT+CMGF=1");
  delay(200);
  Serial.println("[1] Text Mode");

  // Bước 2: Bắt đầu soạn tin
  String cmd = "AT+CMGS=\"" + number + "\"";
  simSerial.println(cmd);
  delay(200);
  Serial.println("[2] Dang nhap so dien thoai...");

  // Bước 3: Gửi nội dung
  simSerial.print(message);
  delay(200);
  Serial.println("[3] Dang nhap noi dung...");

  // Bước 4: Gửi Ctrl+Z
  simSerial.write(0x1A);
  delay(5000);  // Đợi phản hồi

  Serial.println("[4] Da gui lenh ket thuc.");
  Serial.print(">> Phan hoi: ");
  printResponse();

  smsInProgress = false;  // Hoàn tất, sẵn sàng cho lần nhấn tiếp theo
}

/**
 * Đọc phản hồi từ module trong 3 giây
 */
void printResponse() {
  unsigned long timeout = millis() + 3000;
  while (millis() < timeout) {
    while (simSerial.available()) {
      Serial.write(simSerial.read());
    }
  }
  Serial.println();
}
