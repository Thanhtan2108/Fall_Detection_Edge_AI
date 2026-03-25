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

// ================= FUNCTION =================
void updateButton();
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
                buzzerState = true;
            } else {
                Serial.println(">>> RELEASE BUTTON");
                buzzerState = false;
            }
        }
    }

    lastButtonState = currentState;
}

// ================= BUZZER HANDLE =================
void updateBuzzer() {
    if (buzzerState) {
        digitalWrite(BUZZER_PIN, HIGH);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
}
