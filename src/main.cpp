#include <Arduino.h>

#define BUTTON_PIN 5
#define BUZZER_PIN 14

// ================= BUTTON =================
bool lastButtonState = HIGH;
bool stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_TIME = 50;

// ================= BUZZER =================
bool buzzerState = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;

// ================= FUNCTION =================
void updateButton();
void startBuzzer(unsigned long duration);
void updateBuzzer();

void setup() {
    Serial.begin(115200);
    delay(100);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
    updateButton();
    updateBuzzer();
}

// ================= BUTTON HANDLE =================
void updateButton() {
    unsigned long currentTime = millis();
    bool currentState = digitalRead(BUTTON_PIN);

    if (currentState != lastButtonState) {
        lastDebounceTime = currentTime;
    }

    if (currentTime - lastDebounceTime > DEBOUNCE_TIME) {
        if (currentState != stableButtonState) {
            stableButtonState = currentState;

            if (stableButtonState == LOW) {
                Serial.println(">>> PRESS BUTTON");
                startBuzzer(1000); // kêu 1 giây
            } else {
                Serial.println(">>> RELEASE BUTTON");
            }
        }
    }

    lastButtonState = currentState;
}

// ================= BUZZER HANDLE =================
void startBuzzer(unsigned long duration) {
    buzzerState = true;
    buzzerStartTime = millis();
    buzzerDuration = duration;

    digitalWrite(BUZZER_PIN, HIGH);
}

void updateBuzzer() {
    unsigned long buzzerCurrentTime = millis();
    
    if (buzzerState) {
        if (buzzerCurrentTime - buzzerStartTime >= buzzerDuration) {
            buzzerState = false;
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
}
