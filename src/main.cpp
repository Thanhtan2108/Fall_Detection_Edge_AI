#include <Arduino.h>
#include <HardwareSerial.h>

// Định nghĩa chân UART2 để giao tiếp với A7680C
#define SIM_RX_PIN 16  // Chân RX2 (GPIO16) kết nối với chân TX của A7680C
#define SIM_TX_PIN 17  // Chân TX2 (GPIO17) kết nối với chân RX của A7680C

#define PHONE_NUMBER "+84327524504"

bool smsSent = false;

// Khởi tạo UART2
HardwareSerial simSerial(2);

void sendSMS(String number, String message);
void printResponse();

void setup() {
  // Khởi tạo Serial Monitor (UART0) để giao tiếp với máy tính
  Serial.begin(115200);
  
  // Khởi tạo UART2 để giao tiếp với module A7680C
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  
  Serial.println("ESP32 UART Bridge + SMS Sender da san sang.");
  Serial.println("Module se tu dong gui SMS sau 10 giay khoi dong.");
  Serial.println("Ban van co the go lenh AT thu cong nhu truoc.");
}

void loop() {
  // ---- Chức năng 1: Tự động gửi SMS sau khi khởi động ----
  if (!smsSent) {
    delay(10000); // Đợi 10 giây cho module chắc chắn sẵn sàng
    sendSMS(PHONE_NUMBER, "Xin chao tu ESP32 va A7680C!");
    smsSent = true;
    Serial.println("\n>> SMS da duoc gui. Bay gio ban co the go lenh AT thu cong.\n");
  }

  // ---- Chức năng 2: Cầu nối UART (đọc/phát giữa module và Serial Monitor) ----
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
  Serial.println(">> Bat dau gui SMS...");

  // Bước 1: Chọn chế độ Text
  simSerial.println("AT+CMGF=1");
  delay(200);
  Serial.println("[1] Da chon Text Mode");

  // Bước 2: Bắt đầu soạn tin
  String cmd = "AT+CMGS=\"" + number + "\"";
  simSerial.println(cmd);
  delay(200);
  Serial.println("[2] Dang nhap so dien thoai...");

  // Bước 3: Gửi nội dung
  simSerial.print(message);
  delay(200);
  Serial.println("[3] Dang nhap noi dung...");

  // Bước 4: Gửi ký tự Ctrl+Z để kết thúc
  simSerial.write(0x1A);
  delay(5000); // Đợi module trả kết quả

  Serial.println("[4] Da gui lenh ket thuc (Ctrl+Z)");
  Serial.print(">> Phan hoi tu module: ");
  printResponse();
}

/**
 * Hàm đọc phản hồi từ module trong 3 giây và in ra
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
