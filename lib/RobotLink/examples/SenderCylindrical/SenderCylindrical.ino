#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Zylindrische Koordinaten (Mode 2)
//
// Poti-Belegung:
//   Poti 0 (GPIO 17) → Basisrotation  (Servo 1, direkt)
//   Poti 1 (GPIO  5) → Elevation      (Winkel nach oben, 0°–80°)
//   Poti 2 (GPIO  6) → Radius         (lineares Ein-/Ausfahren)
//   Poti 3 (GPIO  7) → Handgelenk     (Servo 5, direkt)
//   Poti 4 (GPIO 15) → (unbenutzt)
//   Poti 5 (GPIO 16) → Greifer        (Servo 6, direkt)
//
// Die Kinematik (IK) läuft auf dem Receiver — dieser Sender schickt nur
// die Koordinaten. Modus muss auf beiden Boards übereinstimmen (Mode 2).
// Doppeltes RST-Drücken wechselt den Modus (1→2→3→4→5→1).
// ─────────────────────────────────────────────────────────────────────────────

// GPIO-Pin pro Eingangsachse
//   Index:       0    1    2    3     4     5
//   Bedeutung:   Rot  Elev Rad  Handg (–)   Greif
// const uint8_t AXIS_PINS[6] = { 4,   5,   6,   7,   15,   16 };
const uint8_t AXIS_PINS[6] = { 4,   7,   6,   5,   15,   16 };

// Smoothing-Stärke pro Achse (0 = aus, 1 = kaum, 50 = stark)
const uint8_t SMOOTHING[6] = { 10,  10,  10,  10,   0,   10 };

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 5000) delay(10);

    for (int i = 0; i < 6; i++) {
        if (SMOOTHING[i] > 0) robotLink.setAxisSmoothing(i, SMOOTHING[i]);
    }

    robotLink.setCoordMode(COORD_CYLINDRICAL);

    if (!robotLink.beginSender()) {
        Serial.println("[Sender] FEHLER: ESP-NOW konnte nicht gestartet werden");
    }
    Serial.printf("[Sender] Zylindrisch — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button
    for (int i = 0; i < 6; i++) {
        robotLink.setAxisValue(i, analogRead(AXIS_PINS[i]));
    }

    // Debug: Elevation und Radius ausgeben
    Serial.printf("[Elev] %4d  [Rad] %4d\n",
                  analogRead(AXIS_PINS[1]), analogRead(AXIS_PINS[2]));

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
