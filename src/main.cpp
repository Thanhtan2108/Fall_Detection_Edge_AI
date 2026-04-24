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
