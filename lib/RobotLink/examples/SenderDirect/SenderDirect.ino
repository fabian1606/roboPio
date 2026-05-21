#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Direktsteuerung (Mode 1)
// Jeder Poti steuert direkt einen Servo.
// ─────────────────────────────────────────────────────────────────────────────

// GPIO-Pin pro Achse / Servo
//   Index:       0    1    2    3     4     5
//   Servo:       1    2    3    4     5     6
const int AXIS_PINS[6] = { 17,   5,   6,   7,   15,   16 };

// Smoothing-Stärke pro Achse (0 = aus, 1 = kaum, 50 = stark)
const int SMOOTHING[6] = { 10,  10,  10,  10,  10,  10 };

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 5000) delay(10);

    for (int i = 0; i < 6; i++) {
        if (SMOOTHING[i] > 0) robotLink.setAxisSmoothing(i, SMOOTHING[i]);
    }

    // COORD_DIRECT ist der Standard — kein setCoordMode() nötig
    if (!robotLink.beginSender()) {
        Serial.println("Verbindung konnte nicht hergestellt werden");
    }
    Serial.printf("[Sender] Direkt — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button
    for (int i = 0; i < 6; i++) {
        robotLink.setAxisValue(i, analogRead(AXIS_PINS[i]));
    }
    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
