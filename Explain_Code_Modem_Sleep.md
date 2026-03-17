#

## Tiết kiệm năng lượng

- `setCpuFrequencyMhz(80)` trong `setup()`: Giảm tốc độ CPU từ 240MHz xuống 80MHz, giúp giảm đáng kể dòng tiêu thụ mà vẫn đủ xử lý AI.

- Chế độ Modem Sleep: WiFi được tắt hoàn toàn (`WiFi.mode(WIFI_OFF)`) khi không cần gửi dữ liệu. Chỉ bật lên khi phát hiện té ngã.

## Quản lý WiFi thông minh

- `enableWiFi()` và `disableWiFi()`: Bật/tắt modem WiFi một cách an toàn, tránh bật/tắt nhiều lần không cần thiết.

- `connectWiFi()`: Kết nối với timeout (10 giây) để tránh treo vô thời hạn.

- `disconnectWiFi()`: Ngắt kết nối nhưng chưa tắt modem, dùng riêng.

## Tránh gửi cảnh báo trùng lặp

- Biến `alert_sent_for_current_fall` đảm bảo chỉ gửi một lần cho mỗi sự kiện té ngã (vì có thể nhiều frame liên tiếp đều cho kết quả Fall).

- Reset biến này khi phát hiện trạng thái khác (ví dụ Standing) với confidence > 50%, giúp sẵn sàng cho lần ngã tiếp theo.

## Không làm thay đổi logic AI hiện tại

- Toàn bộ phần xử lý buffer, sliding window, inference vẫn giữ nguyên.

- Chỉ thêm các hàm WiFi và điều kiện gửi cảnh báo trong `run_inference()`.
