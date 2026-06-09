#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Zylindrische Koordinaten (Mode 2)
//
// Poti-Belegung:
//   basePot    (GPIO  4) → Achse 0 → Basisrotation  (Servo 1, direkt)
//   elevPot    (GPIO  7) → Achse 1 → Elevation      (Winkel nach oben, 0°–80°)
//   radiusPot  (GPIO  6) → Achse 2 → Radius         (lineares Ein-/Ausfahren)
//   wristPot   (GPIO  5) → Achse 3 → Handgelenk     (Servo 5, direkt)
//                        → Achse 4 → unbenutzt
//   gripperPot (GPIO 16) → Achse 5 → Greifer        (Servo 6, direkt)
//
// Die Kinematik (IK) läuft auf dem Receiver — dieser Sender schickt nur
// die Koordinaten. Modus muss auf beiden Boards übereinstimmen (Mode 2).
// Doppeltes RST-Drücken wechselt den Modus (1→2→3→4→5→1).
// ─────────────────────────────────────────────────────────────────────────────

const int basePot    = 4;   // Achse 0 → Basisrotation
const int elevPot    = 7;   // Achse 1 → Elevation
const int radiusPot  = 6;   // Achse 2 → Radius
const int wristPot   = 5;   // Achse 3 → Handgelenk
                            // Achse 4 → unbenutzt
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
    // Achse 4 unbenutzt — kein Smoothing nötig
    robotLink.setAxisSmoothing(5, smoothing);

    robotLink.setCoordMode(COORD_CYLINDRICAL);

    if (!robotLink.beginSender()) {
        Serial.println("[Sender] FEHLER: ESP-NOW konnte nicht gestartet werden");
    }

    // Maximalgeschwindigkeit der Servos (0 = max; z.B. 1500 für sanftere
    // Bewegungen). Wird am Empfänger dauerhaft gespeichert.
    robotLink.setMaxSpeed(0);

    Serial.printf("[Sender] Zylindrisch — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button

    robotLink.setBaseRotation(analogRead(basePot));
    robotLink.setElevation(analogRead(elevPot));
    robotLink.setRadius(analogRead(radiusPot));
    robotLink.setWrist(analogRead(wristPot));
    robotLink.setAxisValue(4, 0);  // Achse 4 unbenutzt
    robotLink.setGripper(analogRead(gripperPot));

    Serial.printf("[Elev] %4d  [Rad] %4d\n",
                  analogRead(elevPot), analogRead(radiusPot));

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
