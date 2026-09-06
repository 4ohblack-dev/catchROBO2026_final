#include <Arduino.h>

// =========================
// LED ピン
// =========================
const int LED1_PIN = 16;
const int LED2_PIN = 17;
const int LED3_PIN = 5;

const int INTERVAL = 500;

void setup() {

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);

    // 最初は全部OFF
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
}

void loop() {

    // GPIO16
    digitalWrite(LED1_PIN, HIGH);
    delay(INTERVAL);
    digitalWrite(LED1_PIN, LOW);

    // GPIO17
    digitalWrite(LED2_PIN, HIGH);
    delay(INTERVAL);
    digitalWrite(LED2_PIN, LOW);

    // GPIO5
    digitalWrite(LED3_PIN, HIGH);
    delay(INTERVAL);
    digitalWrite(LED3_PIN, LOW);
}