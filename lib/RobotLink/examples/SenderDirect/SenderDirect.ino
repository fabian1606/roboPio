#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Direktsteuerung (Mode 1)
// Jeder Poti steuert direkt einen Servo.
// ─────────────────────────────────────────────────────────────────────────────

// GPIO-Pin pro Servo
const int servo1Pin = 17;  // Basisrotation
const int servo2Pin = 5;   // Schulter
const int servo3Pin = 6;   // Ellbogen
const int servo4Pin = 7;   // Unterarm-Drehen
const int servo5Pin = 15;  // Handgelenk
const int servo6Pin = 16;  // Greifer

// Smoothing-Stärke (0 = aus, 1 = kaum, 50 = stark)
const int smoothing = 10;

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 5000) delay(10);

    robotLink.setAxisSmoothing(0, smoothing);
    robotLink.setAxisSmoothing(1, smoothing);
    robotLink.setAxisSmoothing(2, smoothing);
    robotLink.setAxisSmoothing(3, smoothing);
    robotLink.setAxisSmoothing(4, smoothing);
    robotLink.setAxisSmoothing(5, smoothing);

    // COORD_DIRECT ist der Standard — kein setCoordMode() nötig
    if (!robotLink.beginSender()) {
        Serial.println("Verbindung konnte nicht hergestellt werden");
    }
    Serial.printf("[Sender] Direkt — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button

    robotLink.setAxisValue(0, analogRead(servo1Pin));
    robotLink.setAxisValue(1, analogRead(servo2Pin));
    robotLink.setAxisValue(2, analogRead(servo3Pin));
    robotLink.setAxisValue(3, analogRead(servo4Pin));
    robotLink.setAxisValue(4, analogRead(servo5Pin));
    robotLink.setAxisValue(5, analogRead(servo6Pin));

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
