# Tài liệu hiểu sâu về module SIM và lệnh AT trong dự án Fall Detection Edge AI

Tài liệu này ghi lại cách module SIM được dùng trong dự án, cách giao tiếp qua lệnh AT, ý nghĩa của từng lệnh AT, ý nghĩa của từng hàm và luồng hoạt động của hàm `simTask()`.

---

## 1. Mục tiêu của module SIM trong dự án

Module SIM được dùng để thực hiện hai hành động cảnh báo khi hệ thống phát hiện té ngã:

1. Gửi SMS cảnh báo tới số điện thoại đã cài sẵn.
2. Gọi số điện thoại đó để cảnh báo bằng cuộc gọi.

Trong code, chuỗi cảnh báo này được kích hoạt từ hàm `triggerAlert()` khi AI phát hiện "Fall".

---

## 2. Cách module SIM làm việc trong hệ thống

### 2.1 Giao tiếp qua UART

Module SIM không giao tiếp bằng cách gọi hàm trực tiếp như một thiết bị GPIO bình thường. Nó giao tiếp qua cổng UART serial.

Trong code:

```cpp
HardwareSerial simSerial(2);
```

Và giao tiếp được mở ở chân:

```cpp
#define SIM_RX_PIN 16
#define SIM_TX_PIN 17
```

Baudrate dùng là:

```cpp
simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
```

Ý nghĩa:

- `115200`: tốc độ baud
- `SERIAL_8N1`: 8 bit dữ liệu, không parity, 1 bit stop
- `SIM_RX_PIN`, `SIM_TX_PIN`: chân nối với module SIM

### 2.2 Cấu trúc giao tiếp AT command

Module SIM hiểu các lệnh chuẩn gọi là lệnh AT. Khi MCU gửi một lệnh AT, module SIM sẽ phản hồi bằng:

- `OK`: lệnh thực hiện thành công
- `ERROR`: lệnh thất bại
- các thông báo tự động (URC): như báo cuộc gọi bắt đầu, kết thúc, bận, không có người nghe, vv.

Ví dụ:

```cpp
simSerial.println("AT+CMGF=1");
```

Lệnh này được gửi tới module SIM. Module SIM sẽ phản hồi, thường là `OK` nếu hoạt động bình thường.

---

## 3. Các biến quan trọng trong code liên quan đến SIM

### 3.1 Trạng thái SIM

Code dùng enum để quản lý trạng thái chuyển tiếp của module SIM:

```cpp
enum SimState {
    SIM_IDLE,
    SIM_SMS_INIT,
    SIM_SMS_SEND,
    SIM_SMS_BODY,
    SIM_SMS_WAIT,
    SIM_CALL_DIAL,
    SIM_CALL_ACTIVE,
    SIM_HANGUP
};
```

Ý nghĩa từng trạng thái:

- `SIM_IDLE`: đang rảnh, không làm gì
- `SIM_SMS_INIT`: bắt đầu gửi lệnh thiết lập SMS
- `SIM_SMS_SEND`: gửi lệnh `AT+CMGS="..."`
- `SIM_SMS_BODY`: gửi nội dung SMS rồi kết thúc bằng `Ctrl+Z`
- `SIM_SMS_WAIT`: trạng thái chờ phản hồi SMS
- `SIM_CALL_DIAL`: gửi lệnh gọi điện `ATD...;`
- `SIM_CALL_ACTIVE`: đang trong cuộc gọi
- `SIM_HANGUP`: gửi `ATH` để cúp máy

### 3.2 Các biến điều khiển luồng

```cpp
SimState simState = SIM_IDLE;
unsigned long simStateStartTime = 0;
String simRxBuffer = "";
String urcLineBuffer = "";
bool moduleReady = false;
bool callInProgress = false;
bool simCmdSent = false;
```

- `simState`: trạng thái hiện tại của state machine
- `simStateStartTime`: thời điểm bắt đầu trạng thái hiện tại để kiểm tra timeout
- `simRxBuffer`: vùng lưu dữ liệu phản hồi từ module SIM
- `urcLineBuffer`: vùng lưu dòng dữ liệu URC để xử lý từng dòng
- `moduleReady`: trạng thái đăng ký mạng đã sẵn sàng hay chưa
- `callInProgress`: đang có cuộc gọi đang diễn ra
- `simCmdSent`: đã gửi lệnh cho trạng thái hiện tại chưa

---

## 4. Các lệnh AT được dùng trong code

### 4.1 `AT+CMGF=1`

Mục đích:

- Thiết lập chế độ SMS dạng text (text mode), không phải PDU mode.

Tại sao cần:

- Module SIM cần biết mình sẽ gửi SMS theo dạng text để dễ dùng trong code.

Trong code:

```cpp
simSerial.println("AT+CMGF=1");
```

Phản hồi mong đợi:

- `OK`

### 4.2 `AT+CMGS="<số điện thoại>"`

Mục đích:

- Bắt đầu quá trình gửi SMS đến một số điện thoại cụ thể.

Khi gửi lệnh này, module SIM thường trả về ký tự `>` để báo rằng nó đang chờ nội dung SMS.

Trong code:

```cpp
simSerial.println("AT+CMGS=\"" PHONE_NUMBER "\"");
```

Phản hồi mong đợi:

- `>`
- sau đó gửi nội dung SMS

### 4.3 Nội dung SMS + `Ctrl+Z` (`0x1A`)

Sau khi nhận được dấu `>`, MCU sẽ gửi nội dung SMS và kết thúc bằng ký tự điều khiển `Ctrl+Z` (mã hex `0x1A`).

Trong code:

```cpp
simSerial.print(SMS_MESSAGE);
delay(50);
simSerial.write(0x1A);
```

Mục đích:

- Gửi nội dung tin nhắn
- Báo cho module SIM rằng đã kết thúc body SMS

Phản hồi mong đợi:

- `+CMGS:` nếu gửi thành công
- `ERROR` nếu có lỗi

### 4.4 `ATD<number>;`

Mục đích:

- Gọi điện tới số điện thoại được chỉ định.

Trong code:

```cpp
simSerial.println("ATD" PHONE_NUMBER ";");
```

Phản hồi mong đợi:

- `OK` nếu lệnh gọi được chấp nhận

Lưu ý:

- `;` ở cuối có nghĩa là “gọi kiểu voice call” và không dùng dữ liệu.

### 4.5 `ATH`

Mục đích:

- Cúp máy cuộc gọi hiện tại.

Trong code:

```cpp
simSerial.println("ATH");
```

Phản hồi mong đợi:

- `OK`
- hoặc timeout nếu module không phản hồi kịp

### 4.6 `AT+CREG?`

Mục đích:

- Kiểm tra trạng thái đăng ký mạng GSM.

Trong code dùng trong `waitForNetworkReady()`:

```cpp
simSerial.println("AT+CREG?");
```

Phản hồi mong đợi:

- `+CREG: 0,1` hoặc `+CREG: 0,5`: đã đăng ký mạng thành công hoặc đang đăng ký

### 4.7 `AT+CEREG?`

Mục đích:

- Kiểm tra trạng thái đăng ký mạng LTE/4G (nếu module hỗ trợ).

Trong code:

```cpp
simSerial.println("AT+CEREG?");
```

Phản hồi mong đợi:

- `+CEREG: 0,1` hoặc `+CEREG: 0,5`

---

## 5. Các hàm chính trong code và chức năng của từng hàm

### 5.1 `resetSimToIdle()`

```cpp
void resetSimToIdle() {
    simCmdSent = false;
    simRxBuffer = "";
    callInProgress = false;
    simState = SIM_IDLE;
}
```

Mục đích:

- Reset toàn bộ trạng thái SIM về trạng thái rảnh.
- Dùng khi kết thúc cuộc gọi, xảy ra lỗi hoặc timeout.

Lợi ích:

- Tránh việc hệ thống bị kẹt trong một trạng thái cũ.
- Đảm bảo `simCmdSent` và `callInProgress` được reset đồng bộ.

### 5.2 `triggerAlert()`

```cpp
void triggerAlert() {
    if (!buzzer_on) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzer_on = true;
    }

    if (simState == SIM_IDLE) {
        simRxBuffer = "";
        simStateStartTime = millis();
        simState = SIM_SMS_INIT;
    }
}
```

Mục đích:

- Kích hoạt còi báo động
- Bắt đầu chuỗi cảnh báo SIM nếu hệ thống đang rảnh

### 5.3 `simTask()`

Đây là “trái tim” của toàn bộ logic module SIM.

Nó chạy mỗi lần trong `loop()` và làm cho state machine đi từng bước một.

### 5.4 `handleURC(const String& raw)`

```cpp
void handleURC(const String& raw) {
    String line = raw;
    line.trim();
    if (line.isEmpty()) return;

    if (line.startsWith("VOICE CALL: BEGIN")) {
        callInProgress = true;
    }
    else if (line.startsWith("VOICE CALL: END") || line == "NO CARRIER") {
        if (simState == SIM_CALL_ACTIVE) resetSimToIdle();
    }
    else if (line == "BUSY") {
        if (simState == SIM_CALL_ACTIVE) resetSimToIdle();
    }
    else if (line == "NO ANSWER") {
        if (simState == SIM_CALL_ACTIVE) resetSimToIdle();
    }
}
```

Mục đích:

- Xử lý các thông báo tự động từ module SIM
- Cập nhật trạng thái cuộc gọi đang diễn ra hay không

### 5.5 `handleButton()`

Mục đích:

- Nếu người dùng bấm nút, tắt còi
- Nếu đang có cuộc gọi thì cúp máy bằng cách chuyển sang `SIM_HANGUP`

### 5.6 `cleanSerialBuffer()`

```cpp
void cleanSerialBuffer() {
    while (simSerial.available()) simSerial.read();
    simRxBuffer = "";
    urcLineBuffer = "";
}
```

Mục đích:

- Xóa bộ đệm serial trước khi bắt đầu làm việc với module SIM

### 5.7 `readModuleResponse(unsigned long timeoutMs)`

```cpp
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
```

Mục đích:

- Đọc phản hồi từ module trong setup
- Dùng cho các lệnh blocking, thường là trong `setup()`

### 5.8 `waitForNetworkReady()`

```cpp
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
    }
    return false;
}
```

Mục đích:

- Đợi module SIM đăng ký mạng trước khi dùng chức năng SMS/call
- Nếu không đăng ký được thì hệ thống vẫn có thể tiếp tục chạy nhưng cảnh báo có thể không thành công

---

## 6. Luồng hoạt động trong `loop()`

Trong `loop()` có 4 bước chính:

1. Đọc dữ liệu serial từ module SIM
2. Gọi `simTask()` để tiến bộ state machine
3. Xử lý nút bấm
4. Chạy AI và phát hiện té ngã

Phần quan trọng là:

```cpp
while (simSerial.available()) {
    char c = simSerial.read();
    simRxBuffer += c;
    urcLineBuffer += c;
    if (c == '\n') {
        handleURC(urcLineBuffer);
        urcLineBuffer = "";
    }
}
```

Điều này làm cho code:

- đọc dữ kiện từ module SIM liên tục
- không chờ module phản hồi
- xử lý từng dòng phản hồi hoặc URC

---

## 7. Luồng hoạt động của `simTask()` theo từng `switch case`

### 7.1 `SIM_SMS_INIT`

```cpp
case SIM_SMS_INIT:
    if (!simCmdSent) {
        simRxBuffer = "";
        simSerial.println("AT+CMGF=1");
        simStateStartTime = now;
        simCmdSent = true;
    }
    if (simRxBuffer.indexOf("OK") >= 0) {
        simState = SIM_SMS_SEND;
    } else if (now - simStateStartTime > 3000) {
        simCmdSent = false;
    }
```

Hoạt động:

- Gửi `AT+CMGF=1`
- Chờ phản hồi `OK`
- Nếu có `OK` thì chuyển sang gửi SMS
- Nếu quá 3s mà chưa có `OK` thì retry

### 7.2 `SIM_SMS_SEND`

```cpp
case SIM_SMS_SEND:
    if (!simCmdSent) {
        simRxBuffer = "";
        simSerial.println("AT+CMGS=\"" PHONE_NUMBER "\"");
        simStateStartTime = now;
        simCmdSent = true;
    }
    if (simRxBuffer.indexOf('>') >= 0) {
        simState = SIM_SMS_BODY;
    } else if (now - simStateStartTime > 5000) {
        simState = SIM_CALL_DIAL;
    }
```

Hoạt động:

- Gửi `AT+CMGS="<số>"`
- Chờ module trả về dấu `>`
- Nếu có `>` thì chuyển sang bước gửi nội dung SMS
- Nếu timeout 5s mà không thấy `>` thì bỏ qua SMS và chuyển sang gọi điện

### 7.3 `SIM_SMS_BODY`

```cpp
case SIM_SMS_BODY:
    if (!simCmdSent) {
        simSerial.print(SMS_MESSAGE);
        delay(50);
        simSerial.write(0x1A);
        simStateStartTime = now;
        simCmdSent = true;
    }
    if (simRxBuffer.indexOf("+CMGS:") >= 0) {
        simState = SIM_CALL_DIAL;
    } else if (simRxBuffer.indexOf("ERROR") >= 0 ||
               now - simStateStartTime > SIM_STATE_TIMEOUT_SMS) {
        simState = SIM_CALL_DIAL;
    }
```

Hoạt động:

- Gửi nội dung SMS
- Gửi `Ctrl+Z` để kết thúc
- Nếu module báo `+CMGS:` thì coi như SMS gửi thành công
- Nếu lỗi hoặc timeout thì vẫn chuyển sang gọi điện để vẫn cảnh báo

### 7.4 `SIM_CALL_DIAL`

```cpp
case SIM_CALL_DIAL:
    if (!simCmdSent) {
        simRxBuffer = "";
        simSerial.println("ATD" PHONE_NUMBER ";");
        simStateStartTime = now;
        simCmdSent = true;
    }
    if (simRxBuffer.indexOf("OK") >= 0) {
        simState = SIM_CALL_ACTIVE;
    } else if (now - simStateStartTime > SIM_STATE_TIMEOUT_ATD) {
        resetSimToIdle();
    }
```

Hoạt động:

- Gửi `ATD<phone>;`
- Nếu nhận `OK` thì coi như cuộc gọi được bắt đầu
- Nếu quá 10s mà không nhận `OK` thì reset về idle

### 7.5 `SIM_CALL_ACTIVE`

```cpp
case SIM_CALL_ACTIVE:
    if (now - simStateStartTime > 60000UL) {
        simState = SIM_HANGUP;
    }
```

Hoạt động:

- Chờ cuộc gọi xảy ra hoặc kết thúc
- Nếu quá 60s thì tự động chuyển sang cúp máy

### 7.6 `SIM_HANGUP`

```cpp
case SIM_HANGUP:
    if (!simCmdSent) {
        simSerial.println("ATH");
        simStateStartTime = now;
        simCmdSent = true;
    }
    if (simRxBuffer.indexOf("OK") >= 0 ||
        now - simStateStartTime > SIM_STATE_TIMEOUT_ATH) {
        resetSimToIdle();
    }
```

Hoạt động:

- Gửi `ATH` để cúp máy
- Nếu nhận được `OK` hoặc timeout thì quay về idle

### 7.7 `SIM_IDLE`

Không làm gì.

---

## 8. Mối quan hệ giữa `simRxBuffer` và URC

Module SIM gửi hai loại dữ liệu:

1. Phản hồi lệnh AT
   - ví dụ: `OK`, `ERROR`, `+CMGS: 123`
2. URC (Unsolicited Result Code)
   - ví dụ: `VOICE CALL: BEGIN`, `VOICE CALL: END`, `NO CARRIER`, `BUSY`, `NO ANSWER`

Code dùng `simRxBuffer` để lưu toàn bộ dữ liệu nhận được, rồi `handleURC()` xử lý các dòng URC.

Điều này cho phép hệ thống biết được:

- cuộc gọi có bắt đầu hay không
- cuộc gọi kết thúc hay bị từ chối
- có thể cúp máy đúng lúc

---

## 9. Luồng cảnh báo toàn bộ

Khi hệ thống phát hiện té ngã:

1. `run_inference()` phát hiện nhãn `Fall`
2. `triggerAlert()` được gọi
3. Còi bật lên
4. Nếu `simState == SIM_IDLE` thì bắt đầu chuỗi SIM
5. `simTask()` chạy lần lượt:
   - `AT+CMGF=1`
   - `AT+CMGS="<số>"`
   - gửi nội dung SMS + `Ctrl+Z`
   - `ATD<phone>;`
   - `ATH` khi kết thúc hoặc timeout

---

## 10. Điểm cần lưu ý khi đọc code

### 10.1 Đây là state machine đơn giản, không phải full GSM stack

Code này chỉ dùng module SIM cho các tác vụ rất cơ bản:

- SMS
- gọi điện
- kiểm tra mạng

Không có các chức năng như:

- GPRS/HTTP
- MQTT
- TCP/UDP
- nhận/đọc SMS incoming

### 10.2 Dùng `println()` và `print()` thay vì thư viện AT chuyên sâu

Code sử dụng giao tiếp UART thuần túy. Đây là cách đơn giản và phù hợp cho việc học và triển khai nhanh.

### 10.3 Có timeout cho mỗi bước

Mỗi trạng thái đều có timeout để tránh kẹt forever:

- `SIM_STATE_TIMEOUT_SMS`
- `SIM_STATE_TIMEOUT_ATD`
- `SIM_STATE_TIMEOUT_ATH`

### 10.4 `simCmdSent` rất quan trọng

Biến này ngăn việc gửi cùng một lệnh nhiều lần trong cùng một vòng lặp. Nếu không có nó, module SIM có thể nhận bị lặp lệnh.

### 10.5 `delay(50)` trong gửi SMS là cần thiết

Sau khi gửi lệnh `AT+CMGS`, module SIM cần thời gian để chuẩn bị. Delay nhỏ giúp tránh việc gửi nội dung quá sớm.

---

## 11. Mẹo debug khi kiểm tra module SIM

Khi chạy thực tế, nên theo dõi các log sau:

- `>> SIM: AT+CMGF=1 sent`
- `>> SIM: CMGF OK`
- `>> SIM: AT+CMGS sent, waiting '>'`
- `>> SIM: Got '>'`
- `>> SIM: SMS body + Ctrl+Z sent, waiting +CMGS`
- `>> SIM: ATD sent, dialing...`
- `>> URC: Call connected.`
- `>> URC: Call ended.`

Nếu không thấy log nào, có thể do:

- chân RX/TX chưa nối đúng
- baudrate không đúng
- module SIM chưa đăng ký mạng
- module SIM không hỗ trợ đúng lệnh AT mong đợi

---

## 12. Tóm tắt ngắn gọn

Trong dự án này, module SIM được điều khiển bằng một state machine rất rõ ràng:

- Bắt đầu cảnh báo → gửi SMS → gọi điện → cúp máy
- Mỗi bước dùng các lệnh AT chuẩn
- `simTask()` là nơi điều phối toàn bộ trình tự
- `handleURC()` xử lý các thông báo tự động từ modem
- `resetSimToIdle()` giúp hệ thống quay về trạng thái nghỉ an toàn

Nếu bạn muốn hiểu sâu hơn, cách tốt nhất là đọc lần lượt 3 phần:

1. `triggerAlert()` để thấy khi nào chuỗi SIM bắt đầu
2. `simTask()` để thấy từng `switch case`
3. `handleURC()` để hiểu khi nào cuộc gọi được coi là bắt đầu hoặc kết thúc
