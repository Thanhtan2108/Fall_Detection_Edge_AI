# MODULE 4G SIMCOM A7860C-V2-VOLTE

## Breakout Board

Mặt trên Module Sim

![mặt trên module sim](./image/mặt%20trên%20module%20sim.png)

Mặt dưới Module Sim

![mặt dưới module sim](./image/mặt%20dưới%20module%20sim.png)

Ăn-ten lá

![ăn ten lá](./image/ăn%20ten%20lá.png)

## Pinout Function

| Chân trên Board | Tên đầy đủ | Chức năng chi tiết | Ghi chú quan trọng |
| --- | --- | --- | --- |
| VCC | Nguồn vào chính | Cấp nguồn cho toàn bộ module. Module dùng nguồn 3.8V | Tuy nhiên, board đã được tích hợp sẵn mạch ổn áp, cho phép cấp nguồn rộng hơn, từ 5V-16V. Nguồn phải đủ dòng (tối thiểu 1A). |
| GND | Groud | Nối đất chung cho cả module và VĐK | Luôn luôn nối GND chung! Đây là nguyên tắc bất di bất dịch. |
| TX (UTX) | UART Transmit | Chân truyền dữ liệu từ module. | Nối chéo với chân RX của vi điều khiển (Arduino, ESP32...). Mức logic thường là 3.3V. |
| RX (URX) | UART Receive | Chân nhận dữ liệu của module. | Nối chéo với chân TX của vi điều khiển. Cẩn thận với mức logic 5V từ một số dòng Arduino. |
| PEN | Power Key | Chân điều khiển bật/tắt module | Mặc định chân này được kéo lên mức cao. Để khởi động hoặc tắt module, kéo chân này xuống thấp (GND) trong khoảng 1-2 giây. |
| NET (NETLIGHT) | Network Light | Chân tín hiệu đèn báo trạng thái kết nối mạng. | Đấu nối chân này với một con LED để trực quan hóa trạng thái mạng (nhấp nháy theo nhịp khác nhau báo hiệu đang tìm mạng, đã kết nối, v.v...). |
| DTR | Data Terminal Ready | Điều khiển chế độ ngủ (Sleep Mode) | Có thể bỏ qua nếu chưa cần dùng đến các chế độ tiết kiệm năng lượng phức tạp. |

## Sơ đồ kết nối tham khảo với ESP8266, Project Cảnh báo chống trộm bằng Module Sim, GPS Neo 6M

Sơ đồ này dùng cho mục đích tham khảo cách kết nối và cấp nguồn cho Module SIM

![Sơ đồ tham khảo](./image/Sơ%20đồ%20tham%20khảo.png)

## Test và Prototype Module SIM với ESP32 DEVKIT V1

### Tài liệu tham khảo

[DOIT ESP32 DEVKIT V1 Development Board Details, Pinout](https://www.espboards.dev/esp32/esp32doit-devkit-v1/)

[ESP32 DevKit V1 Schematic | Complete Circuit Diagram, Pinout & Design Guide](https://azadtechhub.com/esp32-devkit-v1-schematic-circuit-diagram-pinout/)

[ESP32 UART0, UART1, UART2 Access Using the Arduino IDE](https://copperhilltech.com/blog/esp32-serial-ports-uart0-uart1-uart2-access-using-the-arduino-ide/)

[Video Mạch định vị toàn cầu dùng module GPS neo 6m và module sim 4G A7680C - dùng kít wifi nodemcu esp8266](https://www.youtube.com/watch?v=QQDtmFYWplA&t=225s)

[AT Command Test cho các dòng Module SIM](https://maker.tdlogy.com/wiki/firmware-development/programming-guide/at-command-test-cho-cac-dong-module-sim/)

[Video tham khảo BanLinhKien](https://www.tiktok.com/@banlinhkienretail/video/7484572987828522247)

[Github BLKLab_Test_Module_Sim_4G_A7680C](https://github.com/BanLinhKien/Arduino/tree/main/Th%C6%B0%20Vi%E1%BB%87n%20D%E1%BB%B1%20%C3%81n%20Arduino%20BLKLab/BLKLab_Test_Module_Sim_4G_A7680C)

[Lập trình sử dụng module SIM A7680C A7670C A7600C SIM7600CE để gửi tin nhắn và gọi điện với Arduino](https://maker.tdlogy.com/wiki/firmware-development/programming-guide/lap-trinh-su-dung-module-sim-a7680c-a7670c-a7600c-sim7600ce-de-gui-tin-nhan-va-goi-dien/)

[Module SIM 4G A7680C – Gọi điện thoại, in thông tin SIM/Module, trạng thái 4G](https://iotlabs.vn/lap-trinh-esp32-bai-16-module-sim-4g-a7680c/#respond)

[Github MKE-M21-SIM768x-4G-IoT-Module](https://github.com/makereduvn/MKE-M21-SIM768x-4G-IoT-Module/tree/main)

### Kết nối phần cứng

![Sơ đồ kết nối](./image/Sơ%20đồ%20kết%20nối.png)

Khi Prototype/Test

| ESP32 Devkit V1 | Kết nối đến | Board A7680C | Ghi chú |
| --- | --- | --- | --- |
| 5V(VIN) | <--> | VCC | |
| GND | <--> | GND | Nguyên tắc bất di bất dịch: Luôn phải nối chung GND. |
| GPIO16 (RX2) | <--> | TX | Chân nhận dữ liệu UART2 của ESP32, kết nối với chân TX của module. |
| GPIO17 (TX2) | <--> | RX | Chân truyền dữ liệu UART2 của ESP32, kết nối với chân RX của module. |

Khi dùng thực tế, nên cấp nguồn ngoài cho Module SIM

| Nguồn ngoài 5-12V(2/3A) | Kết nối đến | Board A7680C | Ghi chú |
| --- | --- | --- | --- |
| GND | <--> | GND | Nguyên tắc bất di bất dịch: Luôn phải nối chung GND. |
| 5-12V | <--> | VCC | |

Kiểm tra

- Sau khi kết nối, khoan lắp thẻ SIM, khoan cấp nguồn cho ESP32.

- Cấp nguồn cho Module SIM và quan sát đèn led on board, nếu đèn sáng liên tục thì nghĩa là board đã nhận nguồn.

- Ngắt nguồn, lắp thẻ SIM vào module, bật nguồn lại và quan sát led on board, nếu led sáng chớp tắt thì nghĩa là đã kết nối thẻ SIM thành công.

#### **Lưu ý quan trọng**

- SIM & Antenna: Lắp SIM 4G đã đăng ký dịch vụ thoại/data vào khe SIM, và gắn antenna đi kèm.

- Đăng ký chức năng VoLTE cho cả 2 SIM:

  - Viettel : HDCALL gửi 191

  - Vinaphone : HD CALL gửi 888

### Code test/prototype với ESP32 DEVKIT V1

File `src/main.cpp`

```cpp
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
```

### Code cấu hình file `platformio.ini`

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
monitor_eol = CRLF        ; Đây là dòng "thần thánh" giúp gửi lệnh AT thành công. Tự động thêm \r\n (Carriage Return + Line Feed) mỗi khi bạn nhấn Enter gửi lệnh. Tương đương với chế độ "Both NL & CR" trong Arduino IDE.
```

### Test Module SIM

- Mở Serial Monitor trong PlatformIO

- Gửi lệnh AT "Huyền Thoại": Bắt đầu gõ từng lệnh dưới đây vào khung nhập liệu của Serial Monitor và nhấn "Send" (hoặc Enter).

```bash
AT
```

```bash
AT+CPIN?
```

```bash
AT+CSQ
```

```bash
AT+CREG?
```

```bash
ATI
```

### Chi Tiết Từng Lệnh AT Đã Thực Hiện

| Lệnh AT | Phản hồi từ Module | Ý nghĩa chi tiết (Lời dịch của Senior) |
| --- | --- | --- |
| AT | OK | "Tôi đây, nghe rõ, sẵn sàng nhận lệnh!"<br><br>Lệnh kiểm tra giao tiếp cơ bản nhất. OK xác nhận đường truyền UART giữa ESP32 và module A7680C đã thông suốt, tốc độ baud và các tham số truyền đều chính xác. |
| AT+CPIN? | +CPIN: READY<br>OK | "SIM của cậu đã được nhận diện và không yêu cầu mã PIN."<br><br>READY là trạng thái lý tưởng. Nếu là SIM PIN hoặc SIM PUK, cậu sẽ phải nhập mã để mở khóa. Đây là điều kiện tiên quyết để thực hiện bất kỳ thao tác mạng nào. |
| AT+CSQ | +CSQ: 27,99<br>OK | "Tôi đang bắt sóng rất khỏe! Cường độ tín hiệu là 27/31."<br><br>- Số đầu tiên (27): RSSI (Received Signal Strength Indicator). Thang đo từ 0 đến 31. 27 là mức tín hiệu rất tốt, đảm bảo cuộc gọi và SMS ổn định.<br>- Số thứ hai (99): BER (Bit Error Rate). 99 nghĩa là "không xác định hoặc không có lỗi". Đây là giá trị mặc định khi không có kết nối dữ liệu. |
| AT+CREG? | +CREG: 0,1<br>OK | "Tôi đã đăng ký thành công vào mạng di động nội hạt."<br><br>- Số đầu tiên (0): Chế độ báo cáo (chỉ báo khi được hỏi).<br>- Số thứ hai (1): Trạng thái đăng ký mạng. 1 = Đã đăng ký vào mạng nhà (Home Network). Đây là trạng thái lý tưởng. |
| ATI | Manufacturer: SIMCOM INCORPORATED<br>Model: A7680C-LANS<br>Revision: V6.9.01<br>IMEI: 868508081625165<br>OK | "Đây là 'chứng minh thư' của tôi."<br><br>- Manufacturer: Nhà sản xuất SIMCom (hãng module 4G nổi tiếng).<br>- Model: A7680C-LANS là phiên bản dành cho thị trường Đông Nam Á (LANS = Latin America & South East Asia?). Điều này giải thích tại sao nó hỗ trợ tốt các băng tần 4G tại Việt Nam.<br>- Revision: Phiên bản firmware V6.9.01. Đây là thông tin quan trọng nếu sau này cần cập nhật hoặc tìm lệnh AT tương thích.<br>- IMEI: Số nhận dạng phần cứng duy nhất của module (giống như số khung xe máy). IMEI này khớp với số in trên nhãn dán ở mặt dưới board mà cậu đã gửi ảnh trước đó. Điều này chứng tỏ module là hàng chính hãng, không bị đánh tráo. |

### Xử Lý Sự Cố Thường Gặp

| Vấn đề | Nguyên nhân có thể | Cách khắc phục |
| --- | --- | --- |
| Không nhận được OK | Dây nối TX/RX bị ngược hoặc lỏng | Kiểm tra lại TX (ESP) -> RX (Module) và ngược lại. |
| | Module chưa được bật đúng cách | Kiểm tra chân PEN đã được kéo xuống GND đủ 1.2 giây chưa. |
| | Nguồn không đủ dòng | Dùng nguồn 5V/2A chất lượng cao, không dùng cổng USB máy tính. |
| Serial Monitor hiển thị ký tự lạ | Baudrate không khớp | Kiểm tra lại simSerial.begin() và Serial Monitor đều là 115200. |
| Module phản hồi ERROR | Lệnh AT sai cú pháp hoặc module chưa hỗ trợ | Đây là dấu hiệu kết nối đã ổn, chỉ cần kiểm tra lại lệnh. |
