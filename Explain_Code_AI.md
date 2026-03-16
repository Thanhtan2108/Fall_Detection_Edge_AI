# Phần AI trong code của bạn có thể được chia thành 4 mảng chính. Tôi sẽ giải thích từng mảng, chức năng của nó và tại sao nó lại cần thiết

## Mảng 1: Các hằng số và cấu hình liên quan đến AI (Đầu file)

```c
#include <Fall_Detection_inferencing.h>  // (1) Thư viện AI

// ...

#define WINDOW_FLOATS       EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // 150 (2)

// ...

float         features[WINDOW_FLOATS];  // (3) Bộ đệm chính
int           samples_collected = 0;
bool          buffer_full       = false;
int           stride_count      = 0;
```

- (1) `#include <Fall_Detection_inferencing.h>`: Đây là thư viện do Edge Impulse tạo ra. Nó chứa:

  - Định nghĩa của mô hình AI (các lớp, trọng số).

  - Các hằng số quan trọng như `EI_CLASSIFIER_LABEL_COUNT` (số lượng nhãn), `EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE` (kích thước đầu vào).

  - Hàm `run_classifier()` - "trái tim" của việc chạy suy luận.

- (2) `EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE`: Đây là con số 150 mà bạn thấy. Nó có nghĩa là mô hình AI của bạn yêu cầu đầu vào là một mảng gồm 150 số thực (float) để đưa ra dự đoán. Tại sao là 150? Bởi vì:

  - Bạn đã cấu hình trên Edge Impulse rằng mỗi mẫu dữ liệu (window) để huấn luyện dài 5 giây.

  - Bạn lấy mẫu cảm biến ở tần số 25 Hz (25 mẫu mỗi giây).

  - Mỗi mẫu (sample) có 3 giá trị (x, y, z).

  - Tổng số giá trị = 5 giây x 25 Hz x 3 = 375. **Sai rồi?** Code bạn lại là 150.

  - Giải thích: Con số 150 này cho thấy trên Edge Impulse, bạn (hoặc nền tảng) đã thực hiện một bước tiền xử lý (DSP - Digital Signal Processing). Cụ thể, thay vì đưa 375 giá trị thô vào mạng nơ-ron, bạn đã dùng khối Spectral Analysis để trích xuất các đặc trưng (features) như công suất phổ, tần số trung bình,... Biến 375 giá trị thô thành 150 giá trị đặc trưng. 150 chính là số lượng đặc trưng mà mô hình của bạn cần.

  - **Làm sao biết được con số 150 và căn cứ để thay đổi?**

    - Con số 150 (`WINDOW_FLOATS = 150`) mà bạn thấy trong code thực chất là giá trị của macro `EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE`. Macro này được định nghĩa tự động trong file `model-parameters/model_metadata.h` - một phần của thư viện Arduino do Edge Impulse tạo ra sau khi bạn huấn luyện xong mô hình.

      - Nguồn gốc của con số 150: Con số này là kích thước đầu ra của khối xử lý tín hiệu (DSP block) mà bạn đã chọn trong quá trình xây dựng Impulse trên Edge Impulse. Nó không phải là số mẫu dữ liệu thô (raw data) từ cảm biến, mà là số lượng đặc trưng (features) đã được trích xuất từ cửa sổ dữ liệu thô đó .

        - Dữ liệu thô bạn thu thập (accX, accY, accZ) được đưa vào khối DSP.

        - Khối DSP này (ví dụ: Spectral Features ) sẽ thực hiện các phép tính như FFT (Fast Fourier Transform), lọc nhiễu, tính các giá trị thống kê (RMS, skewness, kurtosis)... để biến đổi dữ liệu thô thành một tập hợp các đặc trưng nhỏ gọn và hữu ích hơn cho việc học.

        - Số lượng đặc trưng đầu ra này phụ thuộc hoàn toàn vào các tham số bạn cài đặt cho khối DSP. Ví dụ: bạn chọn loại filter nào, tần số cắt (cut-off frequency) bao nhiêu, độ dài FFT (FFT length) là bao nhiêu... Tất cả các tham số này sẽ quyết định số lượng đặc trưng cuối cùng (trong trường hợp của bạn là 150).

      - Căn cứ để thay đổi giá trị này: Nếu bạn muốn thay đổi con số 150, bạn không được sửa trực tiếp trong code Arduino. Thay vào đó, bạn phải quay lại dự án trên Edge Impulse và điều chỉnh các tham số trong khối xử lý (processing block) của bạn.

    - 1/ Vào lại dự án trên Edge Impulse Studio.

    - 2/ Đi đến phần "Impulse design".

    - 3/ Nhấp vào khối xử lý bạn đang sử dụng (rất có thể là "Spectral Features" cho dữ liệu gia tốc) .

    - 4/ Thay đổi các tham số như `FFT length`, bật/tắt filter, điều chỉnh `Cut-off frequency`, v.v. .

    - 5/ Sau khi thay đổi, bạn cần "Generate features" lại toàn bộ dữ liệu.

    - 6/ Huấn luyện lại mô hình (retrain).

    - 7/ Tạo lại thư viện Arduino (Deploy) và tải về. Lúc này, file `model_metadata.h` mới sẽ có giá trị `EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE` được cập nhật tương ứng với cấu hình mới của bạn .

- (3) `float features[WINDOW_FLOATS];`: Đây là một vùng đệm (buffer) hình tròn. Nhiệm vụ của nó là luôn giữ 150 giá trị đặc trưng mới nhất để sẵn sàng cho mô hình AI.

## Mảng 2: Cơ chế callback để cung cấp dữ liệu cho AI

```c
int get_signal_data(size_t offset, size_t length, float* out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return EIDSP_OK;
}
```

- Chức năng: Đây là một hàm "cầu nối". Khi bạn gọi `run_classifier()`, nó sẽ không tự nhiên mà biết dữ liệu ở đâu. Nó sẽ gọi hàm `get_signal_data` này nhiều lần để "xin" dữ liệu.

- Cách hoạt động:

  - `offset` và `length` giống như nó nói: "Tôi cần `length` số float, bắt đầu từ vị trí thứ `offset` trong mảng `features` của anh. Hãy chép chúng vào vùng nhớ `out_ptr` cho tôi."

  - Bạn chỉ cần dùng `memcpy` để đáp ứng yêu cầu đó.

- Tại sao phải làm phức tạp vậy? Đây là một kỹ thuật lập trình để tách biệt việc quản lý dữ liệu (bạn) và việc xử lý dữ liệu (thư viện AI). Nó cho phép thư viện AI làm việc với nhiều nguồn dữ liệu khác nhau mà không cần quan tâm dữ liệu được lưu trữ cụ thể ra sao.

## Mảng 3: Logic tích lũy dữ liệu vào bộ đệm (trong `loop()`)

Đây là phần phức tạp nhất, nhưng bạn đã có nền tảng hiểu về sliding window.

**Giai đoạn 1: Lần đầu tiên (`buffer_full = false`)**

```c
if (!buffer_full) {
    int idx = samples_collected * 3;
    features[idx]     = x;
    features[idx + 1] = y;
    features[idx + 2] = z;
    samples_collected++;

    if (samples_collected >= WINDOW_FLOATS / 3) { // 50 samples
        buffer_full = true;
        stride_count = 0;
        run_inference();
    }
    return;
}
```

- Mục đích: Khi mới bắt đầu, buffer `features` đang rỗng. Bạn cần thu thập đủ 50 mẫu cảm biến đầu tiên (50 samples * 3 trục = 150 giá trị) để lấp đầy buffer.

- `samples_collected` đếm số mẫu (sample) đã thu, không phải số float.

- Khi đủ 50 mẫu, bạn đặt cờ `buffer_full = true` và chạy suy luận lần đầu tiên (`run_inference()`). Lúc này, buffer đã có 50 mẫu đầu tiên (tương ứng 2 giây dữ liệu, vì 50 mẫu / 25 Hz = 2 giây). Chờ đã, 5 giây đâu rồi?

  - Giải thích: Mô hình của bạn yêu cầu một window dài 5 giây (125 mẫu) để chạy. Nhưng code bạn đang tích lũy 50 mẫu (2 giây) đã chạy inference. Điều này cho thấy có thể `WINDOW_FLOATS` (150) là số đặc trưng sau DSP, không phải số mẫu thô. Việc tính toán "50 mẫu" ở đây là dựa trên giả định mỗi mẫu thô tạo ra 3 đặc trưng? Điều này hơi mâu thuẫn. Nhưng không sao, logic vận hành là đúng: Bạn đang tích lũy dữ liệu thô và khi đủ một lượng nhất định (tương ứng với window mà mô hình cần), bạn sẽ chạy inference. Con số 50 là do cách bạn cấu hình trên Edge Impulse.

  - **Con số 50 được cấu hình ở mục nào trên Edge Impulse?**

    - Câu nói "Con số 50 là do cách bạn cấu hình trên Edge Impulse" thực chất đang đề cập đến cấu hình cho cửa sổ dữ liệu thô (raw window) và tần số lấy mẫu. Số 50 (mà bạn cho là số mẫu) thực chất là EI_CLASSIFIER_RAW_SAMPLE_COUNT, được tính toán từ hai tham số chính trong quá trình tạo Impulse.

    - Cụ thể, bạn cần tìm đến khu vực "Time series data" trong bước "Create Impulse" . Ở đó có hai ô bạn cần quan tâm:

      - Window size (ms): Đây là độ dài thời gian của một cửa sổ dữ liệu mà bạn muốn mô hình xem xét mỗi lần để đưa ra dự đoán . Với thiết bị phát hiện té ngã, con số này thường là 2000 ms (2 giây) hoặc 5000 ms (5 giây).

      - Frequency (Hz): Đây là tần số lấy mẫu bạn đã đặt khi thu thập dữ liệu (ví dụ: 25 Hz như trong code của bạn) .

    - Công thức để tính tổng số mẫu thô (`raw_sample_count`) trong một cửa sổ là:
`EI_CLASSIFIER_RAW_SAMPLE_COUNT = (Window size / 1000) * Frequency`.

    - Ví dụ với cấu hình trong code của bạn:

    `Window size = 2000 ms` và `Frequency = 25 Hz`

    Thì: (2000 / 1000) x 25 = 2 x 25 = 50. Đó chính là nguồn gốc của con số 50 mẫu thô (samples) . Mỗi mẫu thô này bao gồm 3 giá trị (accX, accY, accZ), nên tổng số giá trị thô đầu vào cho khối DSP sẽ là `50 x 3 = 150` (giá trị này là `EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE` mà chúng ta đã nói ở trên).

    - Vậy, để thay đổi con số 50 này, bạn sẽ điều chỉnh `Window size` hoặc `Frequency` trong bước `"Create Impulse"` trên Edge Impulse . Tuy nhiên, cần hết sức thận trọng vì `Frequency` nên khớp với tần số lấy mẫu thực tế của phần cứng, còn `Window size` là một siêu tham số (hyperparameter) quan trọng ảnh hưởng trực tiếp đến chất lượng của mô hình.

**Giai đoạn 2: Chế độ sliding window (`buffer_full = true`)**

```c
// Lưu tạm vào cuối buffer
int write_idx = WINDOW_FLOATS - STRIDE_FLOATS + stride_count * 3;
features[write_idx]     = x;
features[write_idx + 1] = y;
features[write_idx + 2] = z;
stride_count++;

if (stride_count >= STRIDE_SAMPLES) {
    // Đủ 5 samples mới → slide buffer
    memmove(features, features + STRIDE_FLOATS, (WINDOW_FLOATS - STRIDE_FLOATS) * sizeof(float));
    stride_count = 0;
    run_inference();
}
```

- Mục đích: Duy trì buffer luôn chứa 150 giá trị mới nhất, và cứ mỗi khi có thêm 5 mẫu cảm biến mới (STRIDE_SAMPLES) thì lại chạy inference một lần.

- Cách hoạt động chi tiết:

  - 1/ Ghi đè vào vùng cuối: Với mỗi mẫu cảm biến mới (x,y,z), nó tính toán vị trí để ghi đè vào phần cuối của buffer. Phần cuối này tương ứng với phần dữ liệu "cũ nhất" sắp bị loại bỏ. Nó ghi đè dần dần cho đến khi ghi đủ 5 mẫu (STRIDE_SAMPLES).

  - 2/ Dịch chuyển buffer: Khi đã có đủ 5 mẫu mới được ghi vào cuối (`stride_count >= 5`), nó thực hiện lệnh `memmove`. Lệnh này giống như "kéo" toàn bộ phần dữ liệu cũ (trừ 5 mẫu mới nhất) lên đầu buffer. Kết quả là:

    - 5 mẫu cũ nhất ở đầu buffer bị mất đi.

    - 5 mẫu mới nhất (vừa ghi đè) bây giờ nằm ở cuối buffer.

    - Buffer lại chứa 150 giá trị, nhưng là 150 giá trị mới nhất (đã loại bỏ 5 mẫu cũ, thêm 5 mẫu mới).

  - 3/ Chạy inference: Sau khi buffer đã được cập nhật xong, nó gọi `run_inference()` để phân tích trên bộ dữ liệu mới này.

- Tư duy đằng sau: Đây là cách xử lý luồng dữ liệu thời gian thực (real-time streaming data) một cách hiệu quả. Bạn không thể chờ thu thập đủ 5 giây mỗi lần rồi mới phân tích. Thay vào đó, bạn phân tích từng đoạn chồng lấn (overlapping windows). Ở đây, mỗi lần phân tích, bạn bỏ qua 5 mẫu cũ và thêm 5 mẫu mới (stride). Tần suất phân tích là 5 mẫu / 25 Hz = 0.2 giây/lần. Vậy là mỗi 200ms, bạn có một kết quả dự đoán mới.

## Mảng 4: Hàm chạy suy luận (`run_inference()`)

```c
void run_inference() {
    signal_t signal;                                 // (1) Tạo cấu trúc tín hiệu
    signal.total_length = WINDOW_FLOATS;
    signal.get_data     = &get_signal_data;

    ei_impulse_result_t result = {0};                // (2) Nơi chứa kết quả
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false); // (3) Gọi AI

    if (err != EI_IMPULSE_OK) { ... }

    // (4) Xử lý kết quả
    float       max_val   = 0;
    const char* max_label = "";
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_val) {
            max_val   = result.classification[i].value;
            max_label = result.classification[i].label;
        }
    }

    // (5) In kết quả
    Serial.print("→ ");
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("%s: %.0f%%  ", result.classification[i].label, result.classification[i].value * 100);
    }
    Serial.printf("| %s\n", max_label);

    // (6) Kích hoạt buzzer nếu là FALL
    if (strcmp(max_label, "Fall") == 0 && max_val >= FALL_THRESHOLD) {
        // ... bật buzzer ...
    }
}
```

- (1) `signal_t signal`: Đây là một "đối tượng tín hiệu" mà thư viện Edge Impulse yêu cầu. Nó đóng gói:

  - `total_length`: Kích thước dữ liệu (150).

  - `get_data`: Con trỏ đến hàm callback `get_signal_data` mà bạn đã định nghĩa. Nó như một "giao diện" để thư viện AI có thể lấy dữ liệu từ buffer của bạn.

- (2) `ei_impulse_result_t result`: Một cấu trúc được định nghĩa sẵn trong thư viện, dùng để lưu trữ kết quả sau khi chạy AI. Nó chứa một mảng `classification`, mỗi phần tử là một nhãn (`label`) và độ tin cậy (`value`).

- (3) `run_classifier(...)`: Đây là lệnh quan trọng nhất! Dòng code này thực sự "đánh thức" mô hình AI dậy, đưa dữ liệu từ `signal` vào, chạy các phép tính toán phức tạp, và điền kết quả vào biến `result`. Tham số `false` cuối cùng có nghĩa là không in debug ra Serial.

- (4) & (5) Xử lý và in kết quả: Sau khi có kết quả, bạn làm một vòng lặp đơn giản để tìm ra nhãn nào có độ tin cậy (`value`) cao nhất. Đây là cách hiểu kết quả từ mô hình phân loại (`classification`). Sau đó, nó in ra màn hình tất cả các giá trị để bạn theo dõi.

- (6) Hành động dựa trên kết quả: Cuối cùng, nó kiểm tra nếu nhãn có độ tin cậy cao nhất là `"Fall"` và độ tin cậy đó vượt qua ngưỡng `FALL_THRESHOLD (70%)` thì mới thực sự kích hoạt còi báo động. Đây là cầu nối giữa thế giới AI và thế giới phần cứng.
