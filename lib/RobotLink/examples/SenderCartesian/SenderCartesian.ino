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
//   yawPot     (GPIO  3) → Parameter PARAM_YAW_OFFSET (Drehung des
//                           Koordinatensystems um Z, ±90°). Wird NICHT als
//                           Achse, sondern als Laufzeit-Parameter gesendet
//                           (nur bei Änderung) und im Receiver dauerhaft
//                           gespeichert. Pin frei wählbar (ADC-fähig).
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
const int yawPot     = 3;   // Parameter → Yaw-Offset (Koordinatensystem-Drehung)

// Smoothing-Stärke (0 = aus, 1 = kaum, 50 = stark)
const int smoothing = 10;

// Yaw-Bereich (rad) über den Poti-Hub 0…4095.
const float yawMin = -1.5708f;  // -90°
const float yawMax =  1.5708f;  // +90°

// Letzter gesendeter Yaw-Wert + Sende-Schwelle (rad), damit nur bei echter
// Änderung gefunkt wird (hält die Funkstrecke ruhig).
float yawLast      = 0.0f;
bool  yawInit      = false;
const float yawEps = 0.02f;  // ~1.1°

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

    // Maximalgeschwindigkeit der Servos (0 = max; z.B. 1500 für sanftere
    // Bewegungen). Wird am Empfänger dauerhaft gespeichert.
    robotLink.setMaxSpeed(0);

    Serial.printf("[Sender] Kartesisch — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button

    robotLink.setX(analogRead(xPot));
    robotLink.setY(analogRead(yPot));
    robotLink.setZ(analogRead(zPot));
    robotLink.setPitch(analogRead(pitchPot));
    robotLink.setWristRotate(analogRead(wristPot));
    robotLink.setGripper(analogRead(gripperPot));

    // ── Yaw-Offset als Laufzeit-Parameter (nur bei Änderung senden) ──────────
    float yaw = yawMin + (analogRead(yawPot) / 4095.0f) * (yawMax - yawMin);
    if (!yawInit || fabsf(yaw - yawLast) > yawEps) {
        robotLink.sendParam(PARAM_YAW_OFFSET, yaw);
        yawLast = yaw;
        yawInit = true;
        Serial.printf("[Yaw] %.1f°\n", yaw * 180.0f / PI);
    }

    Serial.printf("[X] %4d [Y] %4d [Z] %4d [Pitch] %4d\n",
                  analogRead(xPot), analogRead(yPot),
                  analogRead(zPot), analogRead(pitchPot));

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
