#include <Arduino.h>
#include <HardwareSerial.h>

// Định nghĩa chân UART2 để giao tiếp với A7680C
#define SIM_RX_PIN 16  // Chân RX2 (GPIO16) kết nối với chân TX của A7680C
#define SIM_TX_PIN 17  // Chân TX2 (GPIO17) kết nối với chân RX của A7680C

// Khởi tạo UART2
HardwareSerial simSerial(2);

void setup() {
  // Khởi tạo Serial Monitor (UART0) để giao tiếp với máy tính
  Serial.begin(115200);
  
  // Khởi tạo UART2 để giao tiếp với module A7680C
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  
  Serial.println("ESP32 UART Bridge da san sang. Hay mo Serial Monitor.");
}

void loop() {
  // 1. Đọc dữ liệu từ module A7680C và in ra Serial Monitor
  if (simSerial.available()) {
    // Dùng vòng lặp while để đọc từng byte, tránh mất dữ liệu
    while (simSerial.available()) {
      Serial.write(simSerial.read());
    }
  }

  // 2. Đọc lệnh từ Serial Monitor và gửi tới module A7680C
  if (Serial.available()) {
    String command = Serial.readString();
    simSerial.print(command); // Gửi nguyên văn (đã có CRLF từ PIO tự thêm)
  }
}
