#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Kartesische Koordinaten (Mode 3)
//
// Poti-Belegung:
//   xPot       (GPIO  4) → Achse 0 → X            (vorne/hinten, mm)
//   yPot       (GPIO  7) → Achse 1 → Y            (links/rechts, mm)
//   zPot       (GPIO  6) → Achse 2 → Z            (hoch/runter, mm)
//   pitchPot   (GPIO  5) → Achse 3 → Tool-Pitch   (Endeffektor-Neigung)
//   wristPot   (GPIO 15) → Achse 4 → Wrist-Rotate (Servo 5, direkt)
//   gripperPot (GPIO 16) → Achse 5 → Greifer      (Servo 6, direkt)
//
// Die Kinematik (IK) läuft auf dem Receiver — dieser Sender schickt nur
// die rohen Poti-Werte. CoordMode wird im Frame mitgesendet.
// ─────────────────────────────────────────────────────────────────────────────

const int xPot       = 4;   // Achse 0 → X (vorne/hinten)
const int yPot       = 7;   // Achse 1 → Y (links/rechts)
const int zPot       = 6;   // Achse 2 → Z (hoch/runter)
const int pitchPot   = 5;   // Achse 3 → Tool-Pitch
const int wristPot   = 15;  // Achse 4 → Wrist-Rotate
const int gripperPot = 16;  // Achse 5 → Greifer

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

    robotLink.setCoordMode(COORD_CARTESIAN);

    if (!robotLink.beginSender()) {
        Serial.println("[Sender] FEHLER: ESP-NOW konnte nicht gestartet werden");
    }
    Serial.printf("[Sender] Kartesisch — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button

    robotLink.setAxisValue(0, analogRead(xPot));
    robotLink.setAxisValue(1, analogRead(yPot));
    robotLink.setAxisValue(2, analogRead(zPot));
    robotLink.setAxisValue(3, analogRead(pitchPot));
    robotLink.setAxisValue(4, analogRead(wristPot));
    robotLink.setAxisValue(5, analogRead(gripperPot));

    Serial.printf("[X] %4d [Y] %4d [Z] %4d [Pitch] %4d\n",
                  analogRead(xPot), analogRead(yPot),
                  analogRead(zPot), analogRead(pitchPot));

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
