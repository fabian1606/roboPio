#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Kartesische Koordinaten (Mode 3)
//
// Poti-Belegung:
//   Poti 0 (GPIO 4)  → X            (vorne/hinten, mm)
//   Poti 1 (GPIO 7)  → Y            (links/rechts, mm)
//   Poti 2 (GPIO 6)  → Z            (hoch/runter, mm)
//   Poti 3 (GPIO 5)  → Tool-Pitch η (Endeffektor-Neigung, rad)
//   Poti 4 (GPIO 15) → Wrist-Rotate (Servo 5, direkt)
//   Poti 5 (GPIO 16) → Greifer      (Servo 6, direkt)
//
// Die Kinematik (IK) läuft auf dem Receiver — dieser Sender schickt nur
// die rohen Poti-Werte. CoordMode wird im Frame mitgesendet.
// ─────────────────────────────────────────────────────────────────────────────

const uint8_t AXIS_PINS[6] = { 4,   7,   6,   5,   15,   16 };

// Smoothing-Stärke pro Achse (0 = aus, 1 = kaum, 50 = stark)
const uint8_t SMOOTHING[6] = { 10,  10,  10,  10,   0,   10 };

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 5000) delay(10);

    for (int i = 0; i < 6; i++) {
        if (SMOOTHING[i] > 0) robotLink.setAxisSmoothing(i, SMOOTHING[i]);
    }

    robotLink.setCoordMode(COORD_CARTESIAN);

    if (!robotLink.beginSender()) {
        Serial.println("[Sender] FEHLER: ESP-NOW konnte nicht gestartet werden");
    }
    Serial.printf("[Sender] Kartesisch — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button
    for (int i = 0; i < 6; i++) {
        robotLink.setAxisValue(i, analogRead(AXIS_PINS[i]));
    }

    Serial.printf("[X] %4d [Y] %4d [Z] %4d [η] %4d\n",
                  analogRead(AXIS_PINS[0]), analogRead(AXIS_PINS[1]),
                  analogRead(AXIS_PINS[2]), analogRead(AXIS_PINS[3]));

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
