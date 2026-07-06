# Tài liệu hiểu sâu về cách hệ thống hoạt động bằng máy trạng thái và vòng lặp tuần tự

Tài liệu này giải thích cách hệ thống này vẫn chạy ổn định, không bị treo và có thể xử lý nhiều tác vụ cùng lúc dù không dùng hệ điều hành thời gian thực (RTOS).

---

## 1. Vấn đề cốt lõi

Trong firmware Arduino/ESP32, thường chỉ có một luồng thực thi chính: hàm `loop()`.

Điều này có nghĩa là:

- hệ thống không chạy đa nhiệm thật sự như Linux hoặc RTOS
- không có scheduler phân chia thời gian cho nhiều task một cách tự động
- nếu một đoạn code chạy quá lâu, toàn bộ hệ thống sẽ bị chặn

Vì vậy, để chương trình vẫn “như đang làm nhiều việc cùng lúc”, ta cần dùng kỹ thuật gọi là:

- máy trạng thái (state machine)
- lập lịch hợp tác (cooperative scheduling)
- viết code không chặn (non-blocking)

---

## 2. Ý tưởng chính: không để chương trình “ngồi chờ” quá lâu

Nếu một tác vụ cần chờ dữ liệu từ module SIM, từ cảm biến, hoặc từ mạng, thì ta không nên viết kiểu:

```cpp
while (something_not_ready) {
    // chờ mãi
}
```

Cách đó sẽ khiến CPU bị kẹt ở đó, nên:

- nút bấm không phản hồi
- AI không chạy tiếp
- UART không được xử lý
- hệ thống giống bị treo

Thay vào đó, ta chia công việc thành từng bước nhỏ, mỗi lần `loop()` chỉ làm một phần, rồi trả về.

---

## 3. Hệ thống này hoạt động như thế nào trong thực tế

### 3.1 `loop()` là trung tâm điều phối

Hàm `loop()` đóng vai trò như một “bộ lập lịch đơn giản”:

```cpp
void loop() {
    // 1. Đọc dữ liệu từ SIM
    // 2. Chạy state machine SIM
    // 3. Xử lý nút bấm
    // 4. Thu thập mẫu cảm biến và chạy AI
}
```

Mỗi lần `loop()` chạy xong, nó lại quay đầu vòng mới.

Như vậy, hệ thống không phải “đợi” một tác vụ hoàn tất rồi mới làm tác vụ khác, mà là:

- mỗi vòng lặp làm một ít việc
- rồi chuyển sang vòng lặp tiếp theo

Đây là kiểu lập lịch hợp tác.

---

## 4. Máy trạng thái là gì?

Máy trạng thái là cách tổ chức chương trình theo các trạng thái cụ thể.

Ví dụ với module SIM, ta có:

```cpp
enum SimState {
    SIM_IDLE,
    SIM_SMS_INIT,
    SIM_SMS_SEND,
    SIM_SMS_BODY,
    SIM_CALL_DIAL,
    SIM_CALL_ACTIVE,
    SIM_HANGUP
};
```

Mỗi trạng thái có một nhiệm vụ riêng:

- `SIM_IDLE`: nghỉ, không làm gì
- `SIM_SMS_INIT`: gửi lệnh `AT+CMGF=1`
- `SIM_SMS_SEND`: gửi `AT+CMGS="..."`
- `SIM_SMS_BODY`: gửi nội dung SMS
- `SIM_CALL_DIAL`: gửi `ATD...;`
- `SIM_CALL_ACTIVE`: chờ cuộc gọi diễn ra
- `SIM_HANGUP`: gửi `ATH`

Khi một trạng thái hoàn thành một bước, hệ thống chuyển sang trạng thái tiếp theo.

---

## 5. Vì sao máy trạng thái giúp hệ thống không bị treo?

Vì mỗi lần chỉ thực hiện một bước nhỏ của một công việc lớn.

Ví dụ khi gửi SMS:

1. đầu tiên gửi lệnh `AT+CMGF=1`
2. sau đó chờ phản hồi
3. tiếp theo gửi lệnh `AT+CMGS="..."`
4. sau đó đợi dấu `>`
5. rồi gửi nội dung SMS
6. rồi chờ phản hồi `+CMGS:`

Nếu viết toàn bộ quá trình này trong một hàm dài, chương trình có thể bị treo ở một chỗ chờ lâu.

Nhưng với máy trạng thái, mỗi lần `simTask()` chỉ làm một phần của quá trình, rồi kết thúc. Vòng lặp sau sẽ tiếp tục.

---

## 6. Cách `simTask()` hoạt động trong hệ thống tuần tự

Trong code, `simTask()` được gọi mỗi lần trong `loop()`:

```cpp
void loop() {
    while (simSerial.available()) {
        // đọc phản hồi từ SIM
    }

    simTask();
    handleButton();
    // chạy AI và cảm biến
}
```

Điều này có nghĩa là:

- mỗi lần `loop()` chạy, nó tiến một bước trong toàn bộ hệ thống
- `simTask()` không làm toàn bộ việc trong một lần gọi
- nó chỉ xử lý đúng “bước hiện tại” của state machine

Ví dụ:

- nếu đang ở `SIM_SMS_INIT`, nó sẽ gửi `AT+CMGF=1` một lần
- lần gọi tiếp theo, nó sẽ kiểm tra phản hồi
- nếu đã nhận `OK`, nó chuyển sang trạng thái kế tiếp

Như vậy, công việc được chia nhỏ và thực hiện luân phiên.

---

## 7. Tại sao không dùng `delay()` quá nhiều?

`delay()` làm cho hệ thống ngừng xử lý mọi thứ trong một khoảng thời gian.

Ví dụ:

```cpp
delay(1000);
```

Trong 1 giây đó, chương trình không làm gì khác cả.

Do đó, trong firmware thời gian thực, nên tránh dùng `delay()` lâu. Thay vào đó dùng:

- `millis()` để theo dõi thời gian
- timeout để biết khi nào nên chuyển trạng thái
- state machine để tiến bộ từng bước

Trong code này, người ta dùng:

```cpp
unsigned long now = millis();
```

và so sánh thời gian đã trôi qua để biết:

- lệnh AT có hết thời gian chờ chưa
- SMS đã timeout chưa
- cuộc gọi đã quá lâu chưa

Đây là cách làm “không chặn” rất quan trọng.

---

## 8. Vai trò của `millis()`

`millis()` cho phép chương trình biết thời gian đã trôi qua mà không làm chương trình đứng yên.

Ví dụ:

```cpp
if (now - simStateStartTime > 3000) {
    // timeout, retry
}
```

Điều này cho phép hệ thống:

- không cần chờ blocking
- có thể kiểm tra timeout trong mỗi vòng lặp
- có thể tiếp tục xử lý nút bấm, AI, UART trong lúc chờ

---

## 9. Đây là kiểu lập lịch hợp tác, không phải đa nhiệm thật

Trong hệ thống này, không có nhiều task chạy song song như RTOS.

Thay vào đó, có 3 “mảnh công việc” được luân phiên xử lý trong cùng một luồng:

1. Xử lý giao tiếp SIM
2. Xử lý nút bấm
3. Thu thập mẫu cảm biến và chạy AI

Mỗi lần `loop()` làm một chút từng phần:

- đọc byte từ SIM
- chạy tiếp state machine SIM
- xử lý nút bấm
- nếu đến lúc thì thu mẫu và chạy inference

Đây là một dạng cooperative multitasking rất phù hợp cho MCU nhỏ.

---

## 10. Ví dụ về luồng xử lý tuần tự

Giả sử hệ thống phát hiện té ngã:

1. AI kết luận là `Fall`
2. `triggerAlert()` được gọi
3. hệ thống chuyển sang trạng thái `SIM_SMS_INIT`
4. `simTask()` trong các vòng lặp tiếp theo sẽ:
   - gửi `AT+CMGF=1`
   - đợi `OK`
   - gửi `AT+CMGS="..."`
   - đợi `>`
   - gửi nội dung SMS
   - gửi `Ctrl+Z`
   - chuyển sang gọi điện

Trong suốt thời gian này:

- nút bấm vẫn có thể được xử lý
- dữ liệu từ SIM vẫn được đọc
- AI vẫn có thể tiếp tục chạy ở các chu kỳ phù hợp

Đây là lý do hệ thống không bị “kẹt” dù có một tác vụ chờ lâu.

---

## 11. Tại sao hệ thống vẫn “có vẻ như làm nhiều việc cùng lúc”?

Vì mỗi vòng lặp rất ngắn. Trong mỗi vòng lặp, CPU làm ít việc nhưng làm liên tục.

Nếu mỗi vòng lặp mất khoảng vài millisecond, thì trong 1 giây có thể chạy hàng nghìn vòng.

Nhờ vậy, người ta có cảm giác:

- SIM đang xử lý
- nút bấm phản hồi ngay
- AI vẫn chạy liên tục

Thực chất là CPU đang luân phiên xử lý các phần nhỏ của nhiều tác vụ, chứ không phải chạy thật song song.

---

## 12. Điểm cần ghi nhớ

### 12.1 Máy trạng thái làm cho logic rõ ràng

Thay vì viết một hàm dài khó theo dõi, ta chia workflow thành các trạng thái rõ ràng.

### 12.2 Không chặn là nguyên tắc quan trọng

Không nên để chương trình dừng lâu ở một chỗ.

### 12.3 `millis()` là công cụ chính để quản lý thời gian

Dùng nó thay vì `delay()` khi có thể.

### 12.4 Hệ thống này là cooperative, không phải preemptive

Tức là mỗi task phải chủ động trả lại quyền điều khiển.

---

## 13. Tóm tắt ngắn gọn

Hệ thống này không dùng RTOS, nhưng vẫn có thể hoạt động ổn định nhờ ba nguyên tắc:

1. `loop()` làm vai trò bộ điều phối chính
2. state machine chia một tác vụ lớn thành nhiều bước nhỏ
3. mỗi bước chỉ chạy một phần rồi trả về, nên hệ thống không bị treo

Đây là cách rất phổ biến để xây dựng firmware trên MCU khi muốn giữ code đơn giản nhưng vẫn đáp ứng nhiều tác vụ.
