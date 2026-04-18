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
#define PHONE_NUMBER "+84342165945"

// ========== BIẾN TOÀN CỤC ==========
HardwareSerial simSerial(2);

bool smsInProgress = false;           // Cờ báo đang gửi SMS, tránh gửi chồng lệnh

unsigned long lastDebounceTime = 0;   // Thời điểm thay đổi trạng thái nút gần nhất
unsigned long debounceDelay = 50;     // Khoảng thời gian chống dội (50ms)
int lastButtonState = HIGH;           // Trạng thái trước đó của nút
int buttonState = HIGH;               // Trạng thái hiện tại của nút (đã được ổn định)

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
