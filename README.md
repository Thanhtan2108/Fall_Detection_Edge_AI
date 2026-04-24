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

## Module SIM gửi tin nhắn SMS đến số điện thoại được chỉ định

Hình dung module A7680C không chỉ là một con chip truyền nhận tín hiệu vô tuyến đơn thuần. Nó là một chiếc điện thoại di động hoàn chỉnh, nhưng bị cắt bỏ màn hình, bàn phím và pin.

### Bên trong module bao gồm

| Thành phần                        | Chức năng                                                                                                               | Tương đương trên điện thoại                   |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Vi xử lý (CPU)                    | Chạy hệ điều hành riêng (thường là ThreadX hoặc Linux RTOS), quản lý toàn bộ hoạt động.                                 | CPU chính của điện thoại.                     |
| Baseband Processor                | Xử lý tín hiệu vô tuyến, mã hóa/giải mã tín hiệu 4G, giao tiếp với trạm phát sóng (BTS).                                | Chip baseband (Qualcomm, MediaTek).           |
| SIM Controller                    | Giao tiếp với thẻ SIM, xác thực với nhà mạng.                                                                           | Khe SIM.                                      |
| Bộ nhớ Flash/RAM                  | Lưu trữ firmware và dữ liệu tạm.                                                                                        | Bộ nhớ trong của điện thoại.                  |
| Giao thức di động (GSM/LTE Stack) | Phần mềm tích hợp sẵn toàn bộ chồng giao thức từ tầng vật lý đến tầng ứng dụng (L1, L2, L3, NAS, SMS, Call Control...). | Hệ điều hành Android/iOS (phần quản lý mạng). |

### Vai Trò Của ESP32 và UART

Khi gửi lệnh `AT+CMGS="+84xxxxxxxxx"` qua UART, điều gì xảy ra?

- ESP32 chỉ đơn giản gửi một chuỗi ký tự (text) qua chân TX2. Nó không hề biết SMS là gì, sóng 4G là gì.

- Module A7680C nhận chuỗi ký tự đó qua chân RX. Phần mềm `AT Command Parser` bên trong module phân tích cú pháp và hiểu rằng: "À, người dùng muốn tôi gửi một tin nhắn SMS đến số này".

- Sau khi nhận được nội dung và ký tự `Ctrl+Z`, module tự động thực hiện toàn bộ quy trình phức tạp sau đây mà ESP32 không hề hay biết:

  - Đóng gói nội dung vào định dạng `SMS-DELIVER` hoặc `SMS-SUBMIT` (theo chuẩn `3GPP`).

  - Thiết lập kênh tín hiệu với trạm phát sóng (BTS) qua giao thức RRC (Radio Resource Control).

  - Gửi yêu cầu SMS đến SMSC (Trung tâm tin nhắn) của nhà mạng qua kênh điều khiển (SDCCH hoặc NAS trên LTE).

  - Chờ phản hồi CP-ACK từ mạng.

  - Sau khi nhận xác nhận từ SMSC, module trả về `+CMGS: ID` (`ID` là do tổng đàu gán cho tin nhắn) và `OK` cho ESP32 qua chân TX.

Tóm lại: UART chỉ là đường dây điện thoại bàn để cậu "nói" với module. Còn module là người thư ký thông minh, tự mình gọi điện đến nhà mạng và "đọc" nội dung tin nhắn cho họ ghi lại.

### Tại Sao Nhà Sản Xuất Lại Thiết Kế Như Vậy?

Bởi vì nó tuân theo nguyên tắc phân lớp (layering) và tính module hóa trong kỹ thuật điện tử:

- Nhà phát triển ứng dụng (cậu) chỉ cần biết cách "nói chuyện" bằng tập lệnh AT đơn giản. Không cần quan tâm đến mật mã hóa 4G, điều chế QPSK/OFDM, hay thủ tục đăng ký mạng phức tạp.

- Nhà sản xuất module (SIMCom) đã lo liệu hết phần "khó nhằn" nhất, tuân thủ nghiêm ngặt các tiêu chuẩn viễn thông toàn cầu (3GPP, ETSI).

Nếu không có module này, để gửi được một tin nhắn SMS từ ESP32, cậu sẽ phải tự viết code cho toàn bộ chồng giao thức LTE – một công việc mà cả một đội ngũ kỹ sư của Qualcomm hay MediaTek mới làm nổi!

### Gửi Tin Nhắn SMS: Từ AT Command Đến Code

Để gửi một tin nhắn SMS, chúng ta sẽ "dịch" chính xác quy trình gõ lệnh AT bằng tay thành các hàm trong code.

### Các Lệnh AT Cần Nhớ

Quy trình chuẩn để gửi tin nhắn văn bản gồm 4 bước:

- Thiết lập chế độ Text: `AT+CMGF=1`

- Bắt đầu soạn tin: `AT+CMGS="+84xxxxxxxxx"`

- Nhập nội dung: `Xin chao tu A7680C!` (Module sẽ phản hồi bằng dấu `>` để báo sẵn sàng nhận nội dung)

- Gửi tin: Ký tự Ctrl+Z (mã `ASCII 26`, `0x1A`) để kết thúc

### Code sẽ tự động gửi một tin nhắn khi khởi động

```cpp
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
```

Mở Serial Monitor để quan sát log nhé!

### Code gửi SMS khi có 1 sự kiện xảy ra

Tình huống: Sử dụng 1 Push Button để gửi SMS

Mục tiêu:

- Khi chưa nhấn nút: ESP32 vẫn hoạt động như một "cầu nối UART" để cậu có thể gõ lệnh AT kiểm tra bất cứ lúc nào.

- Khi nhấn nút (nối GPIO13 xuống GND): ESP32 sẽ gửi ngay một tin nhắn SMS đến số điện thoại đã cài đặt.

- Chống dội (debounce) để mỗi lần nhấn chỉ gửi một tin nhắn duy nhất.

```cpp
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
```

## Module SIM thực hiện cuộc gọi VoLTE

### Bước 1: "Giấy Thông Hành" - Điều Kiện Tiên Quyết Để Gọi Được VoLTE

Không giống như gửi SMS, chức năng gọi thoại trên nền 4G yêu cầu một "tấm vé" đặc biệt.

- SIM Phải Kích Hoạt VoLTE: Có thể dùng chính số điện thoại đang lắp trong module để soạn tin nhắn:

  - Viettel: `HDCALL` gửi `191`

  - VinaPhone: `HDCALL` gửi `888`

- Module Phải Đăng Ký Mạng Thành Công: Dùng lại lệnh `AT+CREG?` kiểm tra. Phản hồi `+CREG: 0,1` là tín hiệu tốt, nghĩa là module đã sẵn sàng.

- Chuẩn Bị Antenna & Nguồn: Antenna phải được gắn chặt và nguồn 5V/3A phải thật ổn định. Cuộc gọi là lúc module tiêu thụ dòng điện lớn nhất, nguồn yếu có thể khiến module tự ngắt hoặc reset ngay lập tức.

### Bước 2: "Ngôn Ngữ Giao Tiếp" - Giải Mã Các Lệnh AT Điều Khiển Cuộc Gọi

Giống như SMS, việc gọi điện cũng được điều khiển qua UART bằng các lệnh AT đặc biệt.

| Lệnh AT | Mô tả chức năng | Ví dụ minh họa |
| --- | --- | --- |
| ATD\<number\>; | Lệnh quay số để bắt đầu cuộc gọi. Lưu ý: Dấu chấm phẩy ; ở cuối là bắt buộc. | ATD+84327524504; |
| ATH | Lệnh kết thúc cuộc gọi hiện tại (gác máy). | ATH |
| ATA | Lệnh trả lời khi có một cuộc gọi đến. | ATA |
| AT+CHUP | Một lệnh khác cũng để kết thúc cuộc gọi, đảm bảo ngắt kết nối. | AT+CHUP |
| AT+CLCC | Lệnh "kiểm tra phòng khám". Nó liệt kê tất cả các cuộc gọi đang hoạt động và trạng thái của chúng. Rất hữu ích để debug. | AT+CLCC |

Cơ Chế Hoạt Động Đằng Sau Một Lệnh `ATD`

Khi gửi lệnh `ATD+84327524504;`, module A7680C sẽ tự động thực hiện một loạt thao tác phức tạp:

- Nó sẽ yêu cầu nhà mạng thiết lập một kênh thoại VoLTE chất lượng cao, ưu tiên hơn hẳn kênh dữ liệu thông thường.

- Điện thoại được liên lạc sẽ đổ chuông.

- Nếu bắt máy, module sẽ báo về OK và trạng thái cuộc gọi sẽ là "active". Nếu không, nó sẽ báo lỗi `NO CARRIER` hoặc `BUSY`.

### Bước 3: Code Mẫu - Lắp Ráp Mọi Thứ Lại Với Nhau

Dựa trên code có nút nhấn gửi SMS ở giai đoạn trước, chúng ta sẽ nâng cấp để nút nhấn thực hiện luân phiên cả hai chức năng: Nhấn lần 1 để gọi, nhấn lần 2 để gác máy.

File `src/main.cpp`

```cpp
#include <Arduino.h>
#include <HardwareSerial.h>

// ========== CẤU HÌNH PHẦN CỨNG ==========
#define SIM_RX_PIN 16
#define SIM_TX_PIN 17
#define BUTTON_PIN 13
#define PHONE_NUMBER "+84327524504"

// ========== KHAI BÁO HÀM ==========
bool waitForModuleReady(unsigned long timeoutMs);
String readResponse(unsigned long timeoutMs);
void makeCall(String number);
void hangUpCall();

// ========== BIẾN TOÀN CỤC ==========
HardwareSerial simSerial(2);
bool callInProgress = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastButtonState = HIGH;
int buttonState = HIGH;
bool moduleReadyForCall = false;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("ESP32 Call Manager (Simple & Reliable)");
  Serial.println("Dang cho module san sang cho cuoc goi...");

  if (waitForModuleReady(60000)) {
    moduleReadyForCall = true;
    Serial.println(">> Module da san sang cho cuoc goi VoLTE.\n");
  } else {
    Serial.println(">> Canh bao: Module khong san sang sau 60s. Van co the thu goi.\n");
  }
}

// ========== LOOP ==========
void loop() {
  // Xử lý nút nhấn
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        if (!callInProgress) {
          Serial.println("\n>> Nut duoc nhan! Dang thuc hien cuoc goi...");
          makeCall(PHONE_NUMBER);
        } else {
          Serial.println("\n>> Nut duoc nhan! Dang ngat cuoc goi...");
          hangUpCall();
        }
      }
    }
  }
  lastButtonState = reading;

  // Cầu nối UART
  if (simSerial.available()) {
    while (simSerial.available()) Serial.write(simSerial.read());
  }
  if (Serial.available()) {
    simSerial.print(Serial.readString());
  }
}

// ========== CHỜ MODULE SẴN SÀNG ==========
bool waitForModuleReady(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    simSerial.println("AT+CREG?");
    String resp = readResponse(500);
    if (resp.indexOf("+CREG: 0,1") >= 0 || resp.indexOf("+CREG: 0,5") >= 0) {
      return true;
    }
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  return false;
}

// ========== ĐỌC PHẢN HỒI ==========
String readResponse(unsigned long timeoutMs) {
  String resp;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (simSerial.available()) {
      char c = simSerial.read();
      resp += c;
      Serial.write(c);
    }
    delay(10);
  }
  return resp;
}

// ========== THỰC HIỆN CUỘC GỌI ==========
void makeCall(String number) {
  if (!moduleReadyForCall) {
    Serial.println("Module chua san sang, dang kiem tra lai...");
    if (waitForModuleReady(30000)) {
      moduleReadyForCall = true;
    } else {
      Serial.println("Loi: Khong the ket noi mang. Huy cuoc goi.");
      return;
    }
  }

  String cmd = "ATD" + number + ";";
  simSerial.println(cmd);
  Serial.println("Lenh AT: " + cmd);

  String resp = readResponse(10000);
  if (resp.indexOf("OK") >= 0) {
    Serial.println(">> Dang quay so...");
    callInProgress = true;
  } else {
    Serial.println(">> Loi ATD: " + resp);
    callInProgress = false;
  }
}

// ========== KẾT THÚC CUỘC GỌI ==========
void hangUpCall() {
  simSerial.println("ATH");
  Serial.println("Lenh AT: ATH");
  delay(2000);
  callInProgress = false;
  Serial.println(">> Da ngat cuoc goi.\n");
}
```

Log trên Monitor:

```bash
ESP32 UART Bridge + SMS/Call Button da san sang.
Nhan nut (GPIO13 -> GND) de thuc hien:
- Neu chua goi: Se goi den so +84327524504
- Neu dang goi: Se ngat cuoc goi.
Ban van co the go lenh AT thu cong nhu binh thuong.

>> Module A7680C da san sang.


>> Nut duoc nhan! Dang thuc hien cuoc goi...
Lenh AT da gui: ATD+84327524504;
Trang thai cuoc goi (AT+CLCC):
ATD+84327524504;
OK

+CGEV: NW ACT 8,10

+CLCC: 1,0,2,0,0,"+84327524504",145,""

+CGEV: NW MODIFY 10,3

+CLCC: 1,0,3,0,0,"+84327524504",145,""
AT+CLCC
+CLCC: 1,0,3,0,0,"+84327524504",145,""

OK

>> Cuoc goi da duoc thuc hien. Nhan nut lan nua de ngat.


VOICE CALL: BEGIN

+CLCC: 1,0,0,0,0,"+84327524504",145,""

+COLP: "+84327524504",145

+CGEV: NW DEACT 8,10

+CLCC: 1,0,6,0,0,"+84327524504",145,""

VOICE CALL: END: 000004

NO CARRIER

>> Nut duoc nhan! Dang ngat cuoc goi...
Lenh AT da gui: ATH
>> Cuoc goi da ket thuc.

ATH
OK

>> Nut duoc nhan! Dang thuc hien cuoc goi...
Lenh AT da gui: ATD+84327524504;
Trang thai cuoc goi (AT+CLCC):
ATD+84327524504;
OK

+CGEV: NW ACT 8,10

+CLCC: 1,0,2,0,0,"+84327524504",145,""

+CGEV: NW MODIFY 10,3

+CLCC: 1,0,3,0,0,"+84327524504",145,""
AT+CLCC
+CLCC: 1,0,3,0,0,"+84327524504",145,""

OK

>> Cuoc goi da duoc thuc hien. Nhan nut lan nua de ngat.


>> Nut duoc nhan! Dang ngat cuoc goi...
Lenh AT da gui: ATH
>> Cuoc goi da ket thuc.

ATH
OK

+CGEV: NW DEACT 8,10

+CLCC: 1,0,6,0,0,"+84327524504",145,""

VOICE CALL: END

NO CARRIER.
```

Trên đây là phiên bản code test sơ cơ bản để thực hiện xem cuộc gọi VoLTE đã được thực hiện chưa. Người dùng cần:

- Tắt WiFi, bật dữ liệu di động 4G/5G

- Thử mở 1 video youtube để phát

Lúc này hãy thử nhấn nút để Module SIM thực hiện cuộc gọi

- Nếu điện thoại nhận được cuộc gọi và video youtube vẫn tiếp tục phát mà không bị dừng lại thì nghĩa là Module SIM đã thực hiện thành công cuộc gọi VoLTE

Nhưng **code vẫn tồn tại 1 số vấn đề**

Mình thực hiện 2 tính huống test như sau:

- Lần 1: nhấn nút nhấn, Module SIM thực hiện cuộc gọi đến số điện thoại được chỉ định, điện thoại đã đổ chuông, người dùng bắt máy, sau đó người dùng gác máy kết thúc cuộc gọi nhưng Module hình như chưa kết thúc mà phải chờ nhấn nút nhấn lần nữa mới kết thúc.

- Lần 2: nhấn nút nhấn, module thực hiện cuộc gọi đến số điện thoại được chỉ định, điện thoại đổ chuông nhưng người dùng chưa bắt máy, lúc này nhấn nút nhấn lại 1 lần nữa để tắt cuộc gọi từ Module SIM nhưng điện thoại của người dùng vẫn còn đổ chuông.

Mình thử tìm hiểu về việc quản lý trạng thái cuộc gọi VoLTE khi làm việc với Module SIM thì được biết là **do logic code hiện tại chưa đồng bộ với cách module A7680C phản hồi sự kiện.**

Từ log trên Monitor, hãy phân tích

#### Đọc Log: "Module Đang Nói Gì?"

Module đã gửi rất nhiều tín hiệu (`URC - Unsolicited Result Code`) quan trọng mà code hiện tại đang bỏ qua hoàn toàn.

```bash
VOICE CALL: BEGIN          ← Module báo: Cuộc gọi đã thực sự bắt đầu (người dùng bắt máy)

+CLCC: 1,0,0,0,0,"+84...   ← Trạng thái cuộc gọi: '0' = ACTIVE (đang nói chuyện)

NO CARRIER                 ← Module báo: Cuộc gọi đã kết thúc (người dùng gác máy)
```

Vấn đề cốt lõi: Code hiện tại chỉ gửi lệnh và `delay()`, không có cơ chế lắng nghe và phân tích những `URC` này. Module "hét vào tai" ESP32 là "Người ta cúp máy rồi này!" nhưng ESP32 "giả vờ không nghe thấy", vẫn giữ biến `callInProgress = true`.

#### Giải Thích Sâu Hơn Về Từng Vấn Đề

Vấn Đề 1: "Điện Thoại Gác Máy Nhưng Module Vẫn Tưởng Đang Gọi"

- Nguyên nhân: Khi gác máy, module A7680C đã biết cuộc gọi kết thúc và phát ra `URC` `NO CARRIER` và `VOICE CALL: END: ...`. Code không có hàm `readResponse()` để bắt sự kiện này, nên biến `callInProgress` không tự động cập nhật về `false`. Module đã kết thúc cuộc gọi, nhưng "trí nhớ" của ESP32 vẫn nghĩ là đang gọi.

- Kết quả: Phải nhấn nút thêm một lần nữa để gửi lệnh `ATH`, nhưng lệnh này không có tác dụng vì module không còn trong cuộc gọi nào cả. Đây là lý do thấy `OK` sau `ATH` nhưng không có gì thay đổi.

Vấn Đề 2: "Module Tắt Máy Nhưng Điện Thoại Vẫn Đổ Chuông"

- Nguyên nhân: Nhấn nút khi điện thoại đang đổ chuông. Lệnh `ATH` được gửi đi, module nhận và trả về `OK`. Tuy nhiên, mạng VoLTE đã "bắt tay" xong với điện thoại của cậu. Việc module gửi tín hiệu hủy cuộc gọi có thể bị mạng từ chối hoặc chậm trễ, dẫn đến điện thoại vẫn tiếp tục đổ chuông.

- Kết quả: Module nghĩ nó đã hủy cuộc gọi (trả về `OK`), nhưng phía điện thoại chưa nhận được tín hiệu hủy từ nhà mạng. Đây là một "hành vi lạ" thường gặp với lệnh `ATH` khi cuộc gọi chưa được thiết lập hoàn toàn (chưa có trạng thái `ACTIVE`).

### Giải Pháp : Làm Cho ESP32 Biết Lắng Nghe

Cần cải tiến code từ "gửi và quên" thành "gửi và lắng nghe".

#### 1. Hàm Đọc URC Đáng Tin Cậy

Thay vì đọc một lần, cần một vòng lặp đọc liên tục trong một khoảng thời gian ngắn.

```cpp
String readURC(unsigned long timeoutMs = 2000) {
    String response = "";
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        while (simSerial.available()) {
            char c = simSerial.read();
            response += c;
        }
        delay(10); // Cho module thở
    }
    return response;
}
```

#### 2. Đặt Cờ `callInProgress` Bằng Cách Nghe `URC`

Khi gọi lệnh `ATD`, cần nghe module phản hồi để biết cuộc gọi có thành công hay không.

```cpp
void makeCall(String number) {
  String cmd = "ATD" + number + ";";
  simSerial.println(cmd);
  
  // Nghe module trả lời trong 10 giây
  String urc = readURC(10000);
  
  if (urc.indexOf("OK") != -1) {
    Serial.println(">> Dang quay so...");
    // Chờ thêm để xem có CONNECT hoặc NO CARRIER không
    urc = readURC(30000); // Chờ 30 giây để người dùng bắt máy
    if (urc.indexOf("VOICE CALL: BEGIN") != -1 || urc.indexOf("CONNECT") != -1) {
      Serial.println(">> Cuoc goi da duoc ket noi.");
      callInProgress = true;
    } else {
      Serial.println(">> Cuoc goi khong thanh cong (NO CARRIER/BUSY).");
      callInProgress = false;
    }
  } else {
    Serial.println(">> Loi ATD: " + urc);
    callInProgress = false;
  }
}
```

#### 3. Kiểm Tra URC Định Kỳ Trong `loop()`

Cần một cơ chế chạy nền để cập nhật trạng thái `callInProgress` khi module tự động báo `NO CARRIER`.

```cpp
void checkCallState() {
  if (simSerial.available()) {
    String urc = simSerial.readString();
    Serial.print("URC: " + urc); // In ra để debug
    
    if (urc.indexOf("NO CARRIER") != -1 || urc.indexOf("VOICE CALL: END") != -1) {
      if (callInProgress) {
        Serial.println(">> Phat hien URC: Cuoc goi ket thuc.");
        callInProgress = false;
      }
    }
    // Có thể thêm xử lý URC khác ở đây
  }
}
```

Trong hàm `loop()`, sẽ gọi `checkCallState()` thường xuyên thay vì chỉ đọc UART một lần.

#### 4. Code Xử Lý URC và Đồng Bộ Trạng Thái

Tóm lại vấn đề chính đang nằm trong code là **thiếu đồng bộ trạng thái giữa ESP32 và Module SIM**. Module đã kết thúc cuộc gọi (`NO CARRIER`) nhưng ESP32 vẫn nghĩ `callInProgress = true`. Còn khi đang đổ chuông, `ATH` có thể không ngắt được ngay.

```cpp
#include <Arduino.h>
#include <HardwareSerial.h>

// ======================== PHẦN CỨNG ========================
#define SIM_RX_PIN         16      // ESP32 RX2  -> Module TX
#define SIM_TX_PIN         17      // ESP32 TX2  -> Module RX
#define BUTTON_PIN         13      // Nút nhấn active LOW
#define PHONE_NUMBER       "+84833538486"

// ======================== THAM SỐ HOẠT ĐỘNG ========================
constexpr unsigned long MODULE_BOOT_DELAY   = 10000;
constexpr unsigned long DEBOUNCE_DELAY_MS   = 50;
constexpr unsigned long AT_RESPONSE_TIMEOUT = 1000;
constexpr unsigned long CALL_SETUP_TIMEOUT  = 10000;
constexpr unsigned long HANGUP_TIMEOUT      = 3000;
constexpr unsigned long NETWORK_READY_TIMEOUT = 60000;

// ======================== TRẠNG THÁI CUỘC GỌI ========================
enum CallState : int {
  ACTIVE = 0,
  HELD = 1,
  DIALING = 2,
  ALERTING = 3,
  INCOMING = 4,
  DISCONNECT = 6
};

// ======================== BIẾN TOÀN CỤC ========================
HardwareSerial simSerial(2);

struct Button {
  const uint8_t pin;
  bool lastState;
  bool currentState;
  unsigned long lastDebounceTime;
  const unsigned long debounceDelay;

  Button(uint8_t p, unsigned long delayMs)
    : pin(p), lastState(HIGH), currentState(HIGH), lastDebounceTime(0), debounceDelay(delayMs) {}
} button(BUTTON_PIN, DEBOUNCE_DELAY_MS);

bool callInProgress = false;
bool moduleReadyForCall = false;
String urcLineBuffer = "";

// ======================== KHAI BÁO HÀM ========================
void cleanSerialBuffer();
String readModuleResponse(unsigned long timeoutMs);
bool waitForNetworkReady();
void handleUnsolicitedCode(const String& urcLine);
bool isCallActiveFromCLCC(const String& clccResponse);
bool isCallActive();
void makeCall(const String& number);
void hangUpCall();
bool isButtonPressed();

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  pinMode(button.pin, INPUT_PULLUP);

  Serial.println(F("ESP32 Smart Call Manager (Final)"));
  Serial.println(F("Nhan nut (GPIO13 -> GND):"));
  Serial.println(F("  - Chua goi -> Goi den " PHONE_NUMBER));
  Serial.println(F("  - Dang goi -> Ngat cuoc goi."));
  Serial.println(F("Ban van co the go lenh AT thu cong.\n"));

  Serial.print(F("Dang cho module san sang..."));
  moduleReadyForCall = waitForNetworkReady();
  if (moduleReadyForCall) {
    Serial.println(F("\n>> Module A7680C da san sang cho cuoc goi VoLTE.\n"));
  } else {
    Serial.println(F("\n>> Canh bao: Module khong the dang ky mang sau 60s. Van co the thu goi.\n"));
  }
  cleanSerialBuffer();
}

// ======================== LOOP ========================
void loop() {
  // 1. Đọc và xử lý URC từ module
  while (simSerial.available()) {
    char c = simSerial.read();
    urcLineBuffer += c;
    Serial.write(c);
    if (c == '\n') {
      handleUnsolicitedCode(urcLineBuffer);
      urcLineBuffer = "";
    }
  }

  // 2. Xử lý nút nhấn
  if (isButtonPressed()) {
    // 🔥 Cập nhật trạng thái thực tế từ module trước khi quyết định
    bool activeNow = isCallActive();

    if (!activeNow) {
      Serial.println(F("\n>> Goi di..."));
      makeCall(PHONE_NUMBER);
    } else {
      Serial.println(F("\n>> Ngat cuoc goi..."));
      hangUpCall();
    }
  }

  // 3. Cầu nối UART thủ công
  if (Serial.available()) {
    String command = Serial.readString();
    simSerial.print(command);
  }
}

// ======================== CHỜ MẠNG SẴN SÀNG ========================
bool waitForNetworkReady() {
  unsigned long start = millis();
  bool gsmOk = false, lteOk = false;
  while (millis() - start < NETWORK_READY_TIMEOUT) {
    simSerial.println("AT+CREG?");
    String resp = readModuleResponse(AT_RESPONSE_TIMEOUT);
    if (resp.indexOf("+CREG: 0,1") >= 0 || resp.indexOf("+CREG: 0,5") >= 0) gsmOk = true;

    simSerial.println("AT+CEREG?");
    resp = readModuleResponse(AT_RESPONSE_TIMEOUT);
    if (resp.indexOf("+CEREG: 0,1") >= 0 || resp.indexOf("+CEREG: 0,5") >= 0) lteOk = true;

    if (gsmOk && lteOk) return true;
    delay(1000);
    Serial.print(".");
  }
  return false;
}

// ======================== TIỆN ÍCH ========================
void cleanSerialBuffer() {
  while (simSerial.available()) simSerial.read();
  urcLineBuffer = "";
}

String readModuleResponse(unsigned long timeoutMs) {
  String response;
  response.reserve(128);
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (simSerial.available()) {
      char c = simSerial.read();
      response += c;
      Serial.write(c);
    }
    delay(5);
  }
  return response;
}

// ======================== XỬ LÝ URC ========================
void handleUnsolicitedCode(const String& urcLine) {
  String line = urcLine;
  line.trim();
  if (line.isEmpty()) return;

  if (line.indexOf("VOICE CALL: BEGIN") >= 0) {
    Serial.println(F(">> URC: Cuoc goi da ket noi."));
    callInProgress = true;
  }
  else if (line.indexOf("VOICE CALL: END") >= 0 || line.indexOf("NO CARRIER") >= 0) {
    Serial.println(F(">> URC: Cuoc goi da ket thuc."));
    callInProgress = false;
  }
  else if (line.indexOf("+CLCC:") >= 0) {
    if (line.indexOf(",0,") >= 0) callInProgress = true;
    else if (line.indexOf(",6,") >= 0) callInProgress = false;
  }
  else if (line.indexOf("BUSY") >= 0) {
    Serial.println(F(">> URC: May ban."));
    callInProgress = false;
  }
  else if (line.indexOf("NO ANSWER") >= 0) {
    Serial.println(F(">> URC: Khong ai bat may."));
    callInProgress = false;
  }
}

// ======================== KIỂM TRA CUỘC GỌI ========================
bool isCallActiveFromCLCC(const String& clccResponse) {
  int pos = clccResponse.indexOf("+CLCC:");
  if (pos == -1) return false;

  int commaCount = 0, stateStart = -1;
  for (unsigned i = pos; i < clccResponse.length(); ++i) {
    if (clccResponse[i] == ',') {
      ++commaCount;
      if (commaCount == 2) { stateStart = i + 1; break; }
    }
  }
  if (stateStart == -1 || stateStart >= (int)clccResponse.length()) return false;

  int state = clccResponse[stateStart] - '0';
  return (state == CallState::ACTIVE || state == CallState::DIALING || state == CallState::ALERTING);
}

bool isCallActive() {
  simSerial.println("AT+CLCC");
  String resp = readModuleResponse(AT_RESPONSE_TIMEOUT);
  bool active = isCallActiveFromCLCC(resp);
  callInProgress = active;  // Đồng bộ với trạng thái thực tế
  return active;
}

// ======================== THỰC HIỆN CUỘC GỌI ========================
void makeCall(const String& number) {
  if (!moduleReadyForCall) {
    Serial.println(F(">> Module chua san sang, dang kiem tra lai..."));
    moduleReadyForCall = waitForNetworkReady();
    if (!moduleReadyForCall) {
      Serial.println(F(">> Loi: Module khong co ket noi mang."));
      return;
    }
  }

  cleanSerialBuffer();
  String cmd = "ATD" + number + ";";
  simSerial.println(cmd);
  Serial.println("Lenh AT: " + cmd);

  String resp = readModuleResponse(CALL_SETUP_TIMEOUT);
  if (resp.indexOf("OK") >= 0) {
    Serial.println(F(">> Dang quay so..."));
    callInProgress = true;  // 🔥 Đặt cờ ngay để ngăn nhấn nút gọi lại
  } else {
    Serial.println(">> Loi ATD: " + resp);
    callInProgress = false;
  }
}

// ======================== KẾT THÚC CUỘC GỌI ========================
void hangUpCall() {
  Serial.println(F(">> Dang ngat cuoc goi..."));
  cleanSerialBuffer();

  // Gửi ATH trước
  simSerial.println("ATH");
  String resp = readModuleResponse(HANGUP_TIMEOUT);
  if (resp.indexOf("OK") >= 0) {
    Serial.println(F(">> Da gui ATH."));
  } else {
    // Thử AT+CHUP nếu ATH thất bại
    simSerial.println("AT+CHUP");
    resp = readModuleResponse(HANGUP_TIMEOUT);
    if (resp.indexOf("OK") >= 0) {
      Serial.println(F(">> Da gui AT+CHUP."));
    }
  }

  delay(2000); // Đợi module xử lý

  // Kiểm tra lại xem cuộc gọi đã kết thúc chưa
  if (isCallActive()) {
    Serial.println(F(">> Cuoc goi van chua ket thuc, thu lai AT+CHUP..."));
    simSerial.println("AT+CHUP");
    readModuleResponse(1000);
  }

  callInProgress = false;  // Đặt lại cờ, URC sẽ xác nhận nếu cần
  Serial.println(F(">> Da dat lai trang thai.\n"));
}

// ======================== NÚT NHẤN CHỐNG DỘI ========================
bool isButtonPressed() {
  int reading = digitalRead(button.pin);
  if (reading != button.lastState) button.lastDebounceTime = millis();
  if ((millis() - button.lastDebounceTime) > button.debounceDelay) {
    if (reading != button.currentState) {
      button.currentState = reading;
      if (button.currentState == LOW) return true;
    }
  }
  button.lastState = reading;
  return false;
}
```

## Code Gửi SMS và Gọi

```cpp
#include <Arduino.h>
#include <HardwareSerial.h>

// ======================== PHẦN CỨNG ========================
#define SIM_RX_PIN         16      // ESP32 RX2  -> Module TX
#define SIM_TX_PIN         17      // ESP32 TX2  -> Module RX
#define BUTTON_PIN         13      // Nút nhấn active LOW
#define PHONE_NUMBER       "+84833538486"

// ======================== THAM SỐ HOẠT ĐỘNG ========================
constexpr unsigned long MODULE_BOOT_DELAY   = 10000;
constexpr unsigned long DEBOUNCE_DELAY_MS   = 50;
constexpr unsigned long AT_RESPONSE_TIMEOUT = 1000;
constexpr unsigned long CALL_SETUP_TIMEOUT  = 10000;
constexpr unsigned long HANGUP_TIMEOUT      = 3000;
constexpr unsigned long NETWORK_READY_TIMEOUT = 60000;
constexpr unsigned long SMS_TIMEOUT         = 15000; // Thời gian tối đa chờ gửi SMS

// ======================== TRẠNG THÁI CUỘC GỌI ========================
enum CallState : int {
  ACTIVE = 0,
  HELD = 1,
  DIALING = 2,
  ALERTING = 3,
  INCOMING = 4,
  DISCONNECT = 6
};

// ======================== BIẾN TOÀN CỤC ========================
HardwareSerial simSerial(2);

struct Button {
  const uint8_t pin;
  bool lastState;
  bool currentState;
  unsigned long lastDebounceTime;
  const unsigned long debounceDelay;

  Button(uint8_t p, unsigned long delayMs)
    : pin(p), lastState(HIGH), currentState(HIGH), lastDebounceTime(0), debounceDelay(delayMs) {}
} button(BUTTON_PIN, DEBOUNCE_DELAY_MS);

bool callInProgress = false;
bool smsInProgress = false;       // Cờ đang gửi SMS
bool moduleReadyForCall = false;
String urcLineBuffer = "";

// ======================== KHAI BÁO HÀM ========================
void cleanSerialBuffer();
String readModuleResponse(unsigned long timeoutMs);
bool waitForNetworkReady();
void handleUnsolicitedCode(const String& urcLine);
bool isCallActiveFromCLCC(const String& clccResponse);
bool isCallActive();
void makeCall(const String& number);
void hangUpCall();
bool sendSMS(const String& number, const String& message);
bool isButtonPressed();

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  pinMode(button.pin, INPUT_PULLUP);

  Serial.println(F("ESP32 Smart SMS + Call Manager"));
  Serial.println(F("Nhan nut (GPIO13 -> GND):"));
  Serial.println(F("  - Neu khong co cuoc goi: Gui SMS -> Goi den " PHONE_NUMBER));
  Serial.println(F("  - Neu dang goi: Ngat cuoc goi."));
  Serial.println(F("Ban van co the go lenh AT thu cong.\n"));

  Serial.print(F("Dang cho module san sang..."));
  moduleReadyForCall = waitForNetworkReady();
  if (moduleReadyForCall) {
    Serial.println(F("\n>> Module A7680C da san sang.\n"));
  } else {
    Serial.println(F("\n>> Canh bao: Module khong the dang ky mang sau 60s. Van co the thu.\n"));
  }
  cleanSerialBuffer();
}

// ======================== LOOP ========================
void loop() {
  // 1. Đọc và xử lý URC từ module
  while (simSerial.available()) {
    char c = simSerial.read();
    urcLineBuffer += c;
    Serial.write(c);
    if (c == '\n') {
      handleUnsolicitedCode(urcLineBuffer);
      urcLineBuffer = "";
    }
  }

  // 2. Xử lý nút nhấn
  if (isButtonPressed()) {
    // Nếu đang có cuộc gọi -> ưu tiên ngắt
    if (callInProgress) {
      Serial.println(F("\n>> Ngat cuoc goi..."));
      hangUpCall();
    }
    // Nếu không có cuộc gọi và không đang gửi SMS
    else if (!smsInProgress) {
      Serial.println(F("\n>> Nut duoc nhan! Bat dau gui SMS..."));
      
      // Gửi SMS
      if (sendSMS(PHONE_NUMBER, "Canh bao: Co nguoi nhan nut!")) {
        Serial.println(F(">> SMS da gui thanh cong. Dang thuc hien cuoc goi..."));
        delay(1000); // Nghỉ 1 giây trước khi gọi
        makeCall(PHONE_NUMBER);
      } else {
        Serial.println(F(">> Loi gui SMS. Huy cuoc goi."));
      }
    } else {
      Serial.println(F(">> Dang gui SMS, vui long doi..."));
    }
  }

  // 3. Cầu nối UART thủ công
  if (Serial.available()) {
    String command = Serial.readString();
    simSerial.print(command);
  }
}

// ======================== CHỜ MẠNG SẴN SÀNG ========================
bool waitForNetworkReady() {
  unsigned long start = millis();
  bool gsmOk = false, lteOk = false;
  while (millis() - start < NETWORK_READY_TIMEOUT) {
    simSerial.println("AT+CREG?");
    String resp = readModuleResponse(AT_RESPONSE_TIMEOUT);
    if (resp.indexOf("+CREG: 0,1") >= 0 || resp.indexOf("+CREG: 0,5") >= 0) gsmOk = true;

    simSerial.println("AT+CEREG?");
    resp = readModuleResponse(AT_RESPONSE_TIMEOUT);
    if (resp.indexOf("+CEREG: 0,1") >= 0 || resp.indexOf("+CEREG: 0,5") >= 0) lteOk = true;

    if (gsmOk && lteOk) return true;
    delay(1000);
    Serial.print(".");
  }
  return false;
}

// ======================== TIỆN ÍCH ========================
void cleanSerialBuffer() {
  while (simSerial.available()) simSerial.read();
  urcLineBuffer = "";
}

String readModuleResponse(unsigned long timeoutMs) {
  String response;
  response.reserve(128);
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (simSerial.available()) {
      char c = simSerial.read();
      response += c;
      Serial.write(c);
    }
    delay(5);
  }
  return response;
}

// ======================== XỬ LÝ URC ========================
void handleUnsolicitedCode(const String& urcLine) {
  String line = urcLine;
  line.trim();
  if (line.isEmpty()) return;

  if (line.indexOf("VOICE CALL: BEGIN") >= 0) {
    Serial.println(F(">> URC: Cuoc goi da ket noi."));
    callInProgress = true;
  }
  else if (line.indexOf("VOICE CALL: END") >= 0 || line.indexOf("NO CARRIER") >= 0) {
    Serial.println(F(">> URC: Cuoc goi da ket thuc."));
    callInProgress = false;
  }
  else if (line.indexOf("+CLCC:") >= 0) {
    if (line.indexOf(",0,") >= 0) callInProgress = true;
    else if (line.indexOf(",6,") >= 0) callInProgress = false;
  }
  else if (line.indexOf("BUSY") >= 0) {
    Serial.println(F(">> URC: May ban."));
    callInProgress = false;
  }
  else if (line.indexOf("NO ANSWER") >= 0) {
    Serial.println(F(">> URC: Khong ai bat may."));
    callInProgress = false;
  }
}

// ======================== KIỂM TRA CUỘC GỌI ========================
bool isCallActiveFromCLCC(const String& clccResponse) {
  int pos = clccResponse.indexOf("+CLCC:");
  if (pos == -1) return false;

  int commaCount = 0, stateStart = -1;
  for (unsigned i = pos; i < clccResponse.length(); ++i) {
    if (clccResponse[i] == ',') {
      ++commaCount;
      if (commaCount == 2) { stateStart = i + 1; break; }
    }
  }
  if (stateStart == -1 || stateStart >= (int)clccResponse.length()) return false;

  int state = clccResponse[stateStart] - '0';
  return (state == CallState::ACTIVE || state == CallState::DIALING || state == CallState::ALERTING);
}

bool isCallActive() {
  simSerial.println("AT+CLCC");
  String resp = readModuleResponse(AT_RESPONSE_TIMEOUT);
  bool active = isCallActiveFromCLCC(resp);
  callInProgress = active;
  return active;
}

// ======================== GỬI SMS ========================
bool sendSMS(const String& number, const String& message) {
  smsInProgress = true;
  bool success = false;

  // Bước 1: Chọn chế độ Text
  simSerial.println("AT+CMGF=1");
  if (readModuleResponse(1000).indexOf("OK") == -1) {
    Serial.println(F("Loi: Khong the chon Text Mode"));
    smsInProgress = false;
    return false;
  }

  // Bước 2: Gửi lệnh AT+CMGS
  String cmd = "AT+CMGS=\"" + number + "\"";
  simSerial.println(cmd);
  
  // Chờ prompt '>'
  String resp = readModuleResponse(2000);
  if (resp.indexOf('>') == -1) {
    Serial.println(F("Loi: Khong nhan duoc prompt '>'"));
    smsInProgress = false;
    return false;
  }

  // Bước 3: Gửi nội dung
  simSerial.print(message);
  delay(100);
  
  // Bước 4: Gửi Ctrl+Z
  simSerial.write(0x1A);
  
  // Chờ phản hồi +CMGS: hoặc ERROR
  resp = readModuleResponse(SMS_TIMEOUT);
  if (resp.indexOf("+CMGS:") >= 0) {
    Serial.println(F(">> SMS da duoc gui."));
    success = true;
  } else {
    Serial.println(F(">> Loi gui SMS."));
  }

  smsInProgress = false;
  return success;
}

// ======================== THỰC HIỆN CUỘC GỌI ========================
void makeCall(const String& number) {
  if (!moduleReadyForCall) {
    Serial.println(F(">> Module chua san sang, dang kiem tra lai..."));
    moduleReadyForCall = waitForNetworkReady();
    if (!moduleReadyForCall) {
      Serial.println(F(">> Loi: Module khong co ket noi mang."));
      return;
    }
  }

  cleanSerialBuffer();
  String cmd = "ATD" + number + ";";
  simSerial.println(cmd);
  Serial.println("Lenh AT: " + cmd);

  String resp = readModuleResponse(CALL_SETUP_TIMEOUT);
  if (resp.indexOf("OK") >= 0) {
    Serial.println(F(">> Dang quay so..."));
    callInProgress = true;
  } else {
    Serial.println(">> Loi ATD: " + resp);
    callInProgress = false;
  }
}

// ======================== KẾT THÚC CUỘC GỌI ========================
void hangUpCall() {
  Serial.println(F(">> Dang ngat cuoc goi..."));
  cleanSerialBuffer();

  simSerial.println("ATH");
  String resp = readModuleResponse(HANGUP_TIMEOUT);
  if (resp.indexOf("OK") >= 0) {
    Serial.println(F(">> Da gui ATH."));
  } else {
    simSerial.println("AT+CHUP");
    resp = readModuleResponse(HANGUP_TIMEOUT);
    if (resp.indexOf("OK") >= 0) {
      Serial.println(F(">> Da gui AT+CHUP."));
    }
  }

  delay(2000);

  if (isCallActive()) {
    Serial.println(F(">> Cuoc goi van chua ket thuc, thu lai AT+CHUP..."));
    simSerial.println("AT+CHUP");
    readModuleResponse(1000);
  }

  callInProgress = false;
  Serial.println(F(">> Da dat lai trang thai.\n"));
}

// ======================== NÚT NHẤN CHỐNG DỘI ========================
bool isButtonPressed() {
  int reading = digitalRead(button.pin);
  if (reading != button.lastState) button.lastDebounceTime = millis();
  if ((millis() - button.lastDebounceTime) > button.debounceDelay) {
    if (reading != button.currentState) {
      button.currentState = reading;
      if (button.currentState == LOW) return true;
    }
  }
  button.lastState = reading;
  return false;
}
```
