#include <Arduino.h>
#include <RobotLink.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sender: Kartesisch mit entkoppeltem Pitch (COORD_CARTESIAN + Direct-Pitch)
//
// Hybrid aus dem alten Zylindrisch-Controller: X ist fest (Mittelwert), der
// Elevation-Pot fährt die Höhe (Z), der „X"-Pot (GPIO 6) fährt links/rechts (Y),
// und der Basis-Pot dreht das ganze Koordinatensystem um Z (PARAM_YAW_OFFSET).
// Der Pitch bewegt dank setCartDirectPitch(true) NUR das Handgelenk (Servo 4) —
// er dreht den Arm nicht mehr um einen Punkt und lässt die anderen Achsen in Ruhe.
//
// Belegung:
//   basePot     (GPIO  7) → Yaw-Offset   (Drehung des Frames um Z, ±90°, Parameter)
//   elevPot     (GPIO 15) → Achse 2 → Z  (hoch/runter)
//   xPot        (GPIO  6) → Achse 1 → Y  (links/rechts)
//   wristPot    (GPIO  4) → Achse 4 → Handgelenk-Rotation (Servo 5, direkt)
//   pitchPot    (GPIO  5) → Achse 3 → Tool-Pitch η (nur Servo 4, entkoppelt)
//   gripperBtn  (GPIO 16) → Achse 5 → Greifer togglen (Taster, Pulldown)
//   gripperBtn2 (GPIO 17) → Achse 5 → Greifer togglen (Kippschalter, Pulldown)
//   X (Achse 0)           → fest auf Mittelwert (2048)
//
// Beide Buttons hängen am internen PULLDOWN → aktiv = HIGH (gegen 3,3 V
// verdrahten, NICHT gegen GND). Jede Aktivierung (Taster-Druck bzw. Umlegen
// des Kippschalters) schaltet den Greifer um (auf↔zu) — beide tun dasselbe.
//
// Die Kinematik (IK) läuft auf dem Receiver — dieser Sender schickt nur die
// Koordinaten. CoordMode wird im Frame mitgesendet.
// ─────────────────────────────────────────────────────────────────────────────

const int basePot    = 7;   // → Yaw-Offset (Frame-Drehung um Z)
const int elevPot    = 15;  // → Achse 2 → Z (hoch/runter)
const int radiusPot  = 6;   // → Achse 1 → Y (links/rechts)  [der „X"-Pot]
const int wristPot   = 4;   // → Achse 4 → Handgelenk
const int pitchPot   = 5;   // → Achse 3 → Tool-Pitch η
const int gripperBtn  = 16; // → Greifer togglen (Taster, Pulldown)
const int gripperBtn2 = 17; // → Greifer togglen (Kippschalter, Pulldown)

// Smoothing-Stärke (0 = aus, 1 = kaum, 50 = stark)
const int smoothing = 10;

// Fester X-Wert (Mittelwert des 0–4095-Bereichs → mittlerer Vorwärts-Reach).
const uint16_t X_FIXED = 2048;

// Greifer-Endwerte (Button toggelt zwischen den beiden). Bei Bedarf feintunen.
const uint16_t GRIP_OPEN   = 1000;
const uint16_t GRIP_CLOSED = 100;

// Greifer-Toggle: jede Aktivierung eines der beiden Buttons wechselt zu/auf.
// Beide Buttons am internen PULLDOWN → Ruhe = LOW, aktiv = HIGH.
bool gripperClosed       = false;  // Start: offen
int  lastBtn1State       = LOW;    // letzter stabiler Zustand Button 1 (Pulldown: LOW = inaktiv)
int  lastBtn2State       = LOW;    // letzter stabiler Zustand Button 2
unsigned long lastBtn1Ms = 0;      // Zeit der letzten Flanke Button 1 (Entprellung)
unsigned long lastBtn2Ms = 0;      // Zeit der letzten Flanke Button 2
const unsigned long debounceMs = 40;

// Entprellte steigende Flanke (LOW→HIGH = Aktivierung). Gibt true genau einmal
// pro Aktivierung zurück. Verwendet für beide Greifer-Buttons identisch, damit
// Taster und Kippschalter dasselbe tun.
bool gripperActivated(int pin, int &lastState, unsigned long &lastMs) {
    int s = digitalRead(pin);
    if (s != lastState && (millis() - lastMs) > debounceMs) {
        lastMs    = millis();
        lastState = s;
        if (s == HIGH) return true;  // nur beim Aktivieren toggeln
    }
    return false;
}

// Konstanter Wrist-Korrektur-Offset: Handgelenk steht bei Controller-Mitte
// leicht verdreht. Vorzeichen/Betrag nach Augenmaß einstellen.
const int wristOffset = 200;

// Konstanter Pitch-Korrektur-Offset. 0 = aus. Nur setzen falls Tool-Pitch
// bei Controller-Mitte konstant schief steht.
const int pitchOffset = 750;

// ── Yaw-Offset (Frame-Drehung um Z) ─────────────────────────────────────────
// Der Basis-Pot mappt auf einen Winkel in Radiant (Mitte = 0°). PARAM_YAW_OFFSET
// erwartet Radiant, NICHT den rohen 0–4095-Wert. Nur bei Änderung senden, damit
// die Funkstrecke ruhig bleibt (der Parameter ist Session-only und startet bei 0).
const float yawMin = -1.5708f;  // -90°
const float yawMax =  1.5708f;  // +90°
float yawLast      = 0.0f;
bool  yawInit      = false;
const float yawEps = 0.02f;     // ~1.1° Sende-Schwelle

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 5000) delay(10);

    pinMode(gripperBtn,  INPUT_PULLDOWN);  // Buttons gegen 3,3 V, aktiv = HIGH
    pinMode(gripperBtn2, INPUT_PULLDOWN);

    robotLink.setAxisSmoothing(0, smoothing);  // X (fest, Smoothing harmlos)
    robotLink.setAxisSmoothing(1, smoothing);  // Y
    robotLink.setAxisSmoothing(2, smoothing);  // Z
    robotLink.setAxisSmoothing(3, smoothing);  // Tool-Pitch
    robotLink.setAxisSmoothing(4, smoothing);  // Handgelenk
    // Achse 5 (Greifer-Button) — kein Smoothing, soll hart schalten

    robotLink.setCoordMode(COORD_CARTESIAN);

    if (!robotLink.beginSender()) {
        Serial.println("[Sender] FEHLER: ESP-NOW konnte nicht gestartet werden");
    }

    // Maximalgeschwindigkeit der Servos (0 = max). Am Empfänger gespeichert.
    robotLink.setMaxSpeed(0);

    // Pitch steuert NUR das Handgelenk (Servo 4) direkt — keine Kopplung mit
    // Schulter/Ellbogen, X/Y/Z zielen aufs Handgelenk statt die Werkzeugspitze.
    // Wird am Empfänger dauerhaft gespeichert.
    robotLink.setCartDirectPitch(true);

    Serial.printf("[Sender] Kartesisch (Direct-Pitch) — Modus %d\n", robotLink.getMode());
}

void loop() {
    robotLink.update();  // polls mode button

    // Greifer-Toggle: Taster (16) ODER Kippschalter (17) — jede Aktivierung
    // wechselt zu/auf. Beide tun exakt dasselbe.
    if (gripperActivated(gripperBtn,  lastBtn1State, lastBtn1Ms)) gripperClosed = !gripperClosed;
    if (gripperActivated(gripperBtn2, lastBtn2State, lastBtn2Ms)) gripperClosed = !gripperClosed;
    uint16_t gripperVal = gripperClosed ? GRIP_CLOSED : GRIP_OPEN;

    // Poti-Rohbereiche auf vollen 0–4095-Bereich mappen + clampen.
    uint16_t yVal     = constrain(map(analogRead(radiusPot), 250, 4095, 0, 4095), 0, 4095);  // → Y
    uint16_t zVal     = constrain(map(analogRead(elevPot),   130, 1350, 0, 4095), 0, 4095);  // → Z
    uint16_t wristVal = constrain(map(analogRead(wristPot),  800, 2200, 1000, 0) + wristOffset, 0, 4095);
    uint16_t pitchVal = constrain(map(analogRead(pitchPot),    0, 1300, 1000, 3000), 0, 4095);

    // ── Achsen (Kartesisch) ──────────────────────────────────────────────────
    robotLink.setX(X_FIXED);          // Achse 0 — fest auf Mittelwert
    robotLink.setY(yVal);             // Achse 1 — links/rechts
    robotLink.setZ(zVal);             // Achse 2 — hoch/runter
    robotLink.setPitch(pitchVal);     // Achse 3 — Tool-Pitch η (entkoppelt)
    robotLink.setWristRotate(wristVal);// Achse 4 — Handgelenk-Rotation
    robotLink.setGripper(gripperVal); // Achse 5 — Greifer

    // ── Yaw-Offset (Basis-Pot → Frame-Drehung um Z, nur bei Änderung) ────────
    float yaw = yawMin + (analogRead(basePot) / 4095.0f) * (yawMax - yawMin);
    if (!yawInit || fabsf(yaw - yawLast) > yawEps) {
        robotLink.sendParam(PARAM_YAW_OFFSET, yaw);
        yawLast = yaw;
        yawInit = true;
        Serial.printf("[Yaw] %.1f°\n", yaw * 180.0f / PI);
    }

    Serial.printf("[RAW] base(7) %4d  elev(15)->Z %4d  x(6)->Y %4d  wrist(4) %4d  pitch(5) %4d  grip(16) %d  grip2(17) %d  zu=%d\n",
                  analogRead(basePot), analogRead(elevPot), analogRead(radiusPot),
                  analogRead(wristPot), pitchVal,
                  digitalRead(gripperBtn), digitalRead(gripperBtn2), gripperClosed);

    robotLink.sendAllAxes();
    delay(20);  // 50 Hz
}
