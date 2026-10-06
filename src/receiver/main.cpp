#include <Arduino.h>
#include <SCServo.h>
#include <RobotLink.h>

// ─── Servos ───────────────────────────────────────────────────────────────────
// Waveshare "Servo Driver with ESP32" servo bus UART pins (from schematic)
#define SERVO_RXD  18
#define SERVO_TXD  19
#define SERVO_ACC   50   // gentle acceleration to reduce mechanical jerk

// Minimum change in servo units before a new command is issued.
// Prevents redundant writes from ADC noise, but small enough that
// slow fine movements still work (range ~2650 units → 3 units = 0.1%).
#define SERVO_DEADBAND 3

// ─── Überlastschutz (strombasiert) ─────────────────────────────────────────────
// Primärschutz: pro Servo ein Hardware-Drehmoment-Limit (TORQUE_LIMIT[]). Das
// deckelt die PWM → deckelt den Strom → begrenzt die stationäre Temperatur. Auf
// die gewünschte Maximalkraft gesetzt kann der Motor dauerhaft „zupacken", ohne
// durchzubrennen. Sekundärschicht: strombasierte Stall-Erkennung, die ein Gelenk
// nur *sanft einfriert* (Ziel = Ist-Position), wenn es lange sinnlos gegen einen
// Block drückt (Warnung, langsames rotes Blinken). Übertemperatur schaltet den
// Motor aus und blinkt schnell rot.

// Hardware-Drehmoment-Limit pro Servo (0..1000). = maximale „Zupack"-Kraft, die
// zugleich dauerhaft thermisch sicher ist. Schulter/Ellbogen brauchen mehr als
// Handgelenk/Greifer. Empirisch kalibrieren (siehe Monitor-Build). Konservative
// Startwerte — hoch genug zum Heben, gedeckelt gegen Überhitzen.
static const uint16_t TORQUE_LIMIT[6] = { 800, 900, 900, 700, 500, 600 };

// Strom-Stall-Schwellen pro Servo (|ReadCurrent|, ~6,5 mA/LSB). Über diesem Wert
// gilt das Gelenk als „drückt". MUSS kalibriert werden: baseline_max · 1,5…2,0,
// deutlich unter dem gemessenen Blockier-Strom. Startwerte = grobe Schätzung.
static const int STALL_CURRENT[6] = { 300, 350, 350, 250, 200, 250 };

// Positionsfehler (|target - actual|) pro Servo, ab dem „nicht erreicht" gilt.
static const int STALL_POS_TOL[6] = { 20, 20, 20, 20, 20, 20 };

#define SPEED_EPS                 15    // |ReadSpeed| darunter = „steht" (drückt statt fährt)
#define CURRENT_HYST              60    // Strom-Hysterese: Warnung erst darunter zurücksetzen
#define STALL_DURATION_MS         1500  // durchgehender Stall bis zur Force-Warnung (~1–2 s)
#define STARTUP_IGNORE_MS         800   // Einschaltstrom nach Boot ignorieren
#define PROTECT_POLL_INTERVAL_MS  20    // Round-Robin-Schritt (1 Servo/Tick → Runde ≈ 120 ms)
#define TEMP_CRIT_C               70    // °C — zu warm: Motor ausschalten
#define TEMP_RESET_C              55    // °C — Hysterese: wieder einschalten unter diesem Wert
#define FORCE_BLINK_INTERVAL_MS   800   // langsames rotes Blinken bei Force-Überlast (Warnung)
#define OVERTEMP_BLINK_INTERVAL_MS 150  // schnelles rotes Blinken bei Übertemperatur

// ─── Monitoring (Serial-Plotter) ───────────────────────────────────────────────
// Wird nur mit dem Build-Flag -D ROBOLINK_MONITOR aktiviert. Dann wird für ALLE
// 6 Servos (inkl. Greifer = "GRIP") EINE Messgröße im Format des Arduino Serial
// Plotters ausgegeben — nützlich, um Überlast zu erkennen, wenn ein Motor seine
// Zielposition nicht erreicht.
//   PlatformIO:  pio run -e receiver-monitor -t upload -t monitor
//
// WICHTIG: Der Arduino Serial Plotter zeigt max. 8 Serien. 6 Servos × 3 Größen
// wären 18 → abgeschnitten. Deshalb wird pro Build nur EINE Größe geplottet
// (6 Serien). Umschalten per Build-Flag:
//   -D MONITOR_METRIC=MON_CURRENT   (Standard — Strom, das Blockier-Signal)
//   -D MONITOR_METRIC=MON_LOAD      (PWM-Last 0..1000)
//   -D MONITOR_METRIC=MON_TEMP      (Temperatur °C)
#define MON_CURRENT 0
#define MON_LOAD    1
#define MON_TEMP    2
#ifndef MONITOR_METRIC
#define MONITOR_METRIC MON_CURRENT
#endif
#define MONITOR_POLL_INTERVAL_MS  100   // volle Runde über alle 6 Servos je Tick

SMS_STS servos;

struct ServoLimits { int minPos; int maxPos; };

static const ServoLimits LIMITS[6] = {
    {  800, 3450 },  // Servo 1
    {  900, 3200 },  // Servo 2
    { 1000, 3050 },  // Servo 3
    {  900, 3200 },  // Servo 4
    {  180, 3900 },  // Servo 5
    { 1230, 2600 },  // Servo 6
};

// ─── Kinematics config (Mode 2: cylindrical coordinates) ──────────────────────
// jointZeroPos: servo position when joint is at 0 rad (arm pointing straight forward).
// jointScale:   servo units per radian (≈ 2048/π ≈ 651.9 for SMS-STS).
//               Negative = joint moves opposite direction.
// Calibrate by: set arm to reference pose (all joints at 0 rad / straight),
// read each servo's current position, enter it in jointZeroPos.
// Adjust sign of jointScale if a joint moves in the wrong direction.
static const KinematicsConfig KINEMATICS = {
    .L1 = 115.0f, .L2 = 135.0f, .L3 = 165.0f,
    .jointZeroPos = { 2048.0f, 2048.0f, 2048.0f, 2048.0f, 2048.0f, 2048.0f },
    .jointScale   = { 651.9f, -651.9f, -651.9f, -651.9f,  651.9f,  651.9f },
    .limits = {
        {  800, 3450 },  // Servo 1
        {  900, 3200 },  // Servo 2
        { 1000, 3050 },  // Servo 3
        {  900, 3200 },  // Servo 4
        {  180, 3900 },  // Servo 5
        { 1230, 2600 },  // Servo 6
    },
    .rMin    =  40.0f,    // mm — Schulter→Handgelenk-Distanz, knapp über |L1-L2|=21 (α≈5°)
    .rMax    = 240.0f,    // mm — Schulter→Handgelenk-Distanz, sicher unter L1+L2=253
    .elevMin =   0.0f,    // rad — 0° = horizontal
    .elevMax =   1.396f,  // rad — 80°
    .wristMode = WRIST_PARALLEL_GAMMA,  // Fallback, wenn cylUsePitchAxis = false
    .cylUsePitchAxis = true,            // Achse 4 steuert Werkzeug-Pitch η (pitchMin…pitchMax)
    .xMin =   50.0f, .xMax = 350.0f,    // mm — vorne/hinten
    .yMin = -200.0f, .yMax = 200.0f,    // mm — links/rechts
    .zMin = -100.0f, .zMax = 300.0f,    // mm — hoch/runter
    .pitchMin = -1.5708f, .pitchMax = 1.5708f,  // rad — -90°…+90°, Center = 0 (horizontal)
    // Verhalten bei unerreichbarem Ziel:
    //   CART_LIMIT_CLAMP   = pro Gelenk klemmen, so nah wie möglich heranfahren (Default)
    //   CART_LIMIT_HOLD    = letzte gültige Pose halten
    //   CART_LIMIT_PROJECT = auf nächsten erreichbaren Punkt projizieren
    .cartLimitMode = CART_LIMIT_CLAMP,
};

static const uint8_t SERVO_IDS[6] = {1, 2, 3, 4, 5, 6};

int lastSentPos[6];

// ─── Protection state (pro Servo) ──────────────────────────────────────────────
static unsigned long stallStartMs[6]   = {0};     // Beginn des laufenden Stalls (0 = keiner)
static bool          forceStalled[6]   = {false};  // drückt anhaltend gegen Block → rote Warnung
static bool          servoTorqueOff[6] = {false};  // Motor per Übertemp abgeschaltet
static unsigned long lastProtectPollMs = 0;
static unsigned long lastBlinkMs       = 0;
static uint8_t       pollCursor        = 0;
#ifdef ROBOLINK_MONITOR
static unsigned long lastMonitorMs      = 0;
#endif

// Aktuelle Soll-Werte aller 6 Achsen im Frame-Format (0..4095, Gelenkraum).
// Wird von ESP-NOW-Frames UND Serial-Befehlen geschrieben, damit beide Quellen
// nahtlos weitermachen, wo die andere aufgehört hat.
uint16_t axisValue[6] = { 2048, 2048, 2048, 2048, 2048, 2048 };

// Fährt alle Achsen auf values[] (0..4095 → Servo-Limits).
static void driveServos(const uint16_t values[6]) {
    // Build arrays for SyncWritePosEx (writes all 6 servos in one UART packet)
    u8  ids[6];
    s16 positions[6];
    u16 speeds[6];
    u8  accs[6];
    u8  count = 0;

    // Runtime max-speed limit (0 = max/unbegrenzt, der Default). Set via the
    // sender's setMaxSpeed() and persisted on this board.
    u16 maxSpeed = (u16)robotLink.getParam(PARAM_MAX_SPEED);

    for (int i = 0; i < 6; i++) {
        axisValue[i] = values[i];
        int pos = map((int)values[i], 0, 4095,
                      LIMITS[i].minPos, LIMITS[i].maxPos);
        pos = constrain(pos, LIMITS[i].minPos, LIMITS[i].maxPos);

        if (abs(pos - lastSentPos[i]) >= SERVO_DEADBAND) {
            ids[count]       = SERVO_IDS[i];
            positions[count] = (s16)pos;
            speeds[count]    = maxSpeed;
            accs[count]      = SERVO_ACC;
            lastSentPos[i]   = pos;
            count++;
        }
    }

    // Send all changed servos atomically in a single serial packet.
    // This minimises bus time and ensures all servos start moving simultaneously.
    if (count > 0) {
        servos.SyncWritePosEx(ids, count, positions, speeds, accs);
    }
}

// ─── ESP-NOW frame callback ───────────────────────────────────────────────────
// The lib's single-entry queue guarantees this is always the LATEST frame.
// Intermediate positions from fast poti sweeps are automatically discarded —
// the servo never sees them and goes directly to the current target.

void onFrame(uint8_t /*receiverID*/, const uint16_t values[6]) {
    driveServos(values);
}

// ─── Serial-Steuerung ─────────────────────────────────────────────────────────
// Textbefehle über USB-Serial (115200 Baud), eine Zeile pro Befehl, '\n' am Ende.
// Achsen 1..6, Werte 0..4095 (gleiche Skala wie ein Poti/ESP-NOW-Frame).
//
//   M <achse> <delta>      relativ bewegen        z.B. "M 1 +40", "M 6 -100"
//   P <achse> <wert>       eine Achse absolut     z.B. "P 3 2048"
//   A <w1> <w2> ... <w6>   alle Achsen absolut    z.B. "A 2048 2048 2048 2048 2048 2048"
//   H                      Home (alle Achsen auf 2048 = Mitte)
//   ?                      nur aktuelle Werte abfragen
//
// Antwort auf jeden gültigen Befehl:  "POS <w1> <w2> <w3> <w4> <w5> <w6>"
// Bei Fehlern:                        "ERR <grund>"
// Sendet parallel ein Controller per ESP-NOW, gewinnt jeweils der letzte Befehl.

#define SERIAL_CMD_MAXLEN 64

static void printAxisValues() {
    Serial.printf("POS %u %u %u %u %u %u\n",
                  axisValue[0], axisValue[1], axisValue[2],
                  axisValue[3], axisValue[4], axisValue[5]);
}

static void handleSerialCommand(char* line) {
    // Führende Leerzeichen überspringen; leere Zeilen ignorieren.
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    char cmd = toupper(*line);
    char* args = line + 1;
    uint16_t next[6];
    memcpy(next, axisValue, sizeof(next));

    if (cmd == 'M' || cmd == 'P') {
        char* end;
        long axis = strtol(args, &end, 10);
        if (end == args || axis < 1 || axis > 6) { Serial.println("ERR achse 1-6"); return; }
        args = end;
        long val = strtol(args, &end, 10);
        if (end == args) { Serial.println("ERR wert fehlt"); return; }
        long v = (cmd == 'M') ? (long)axisValue[axis - 1] + val : val;
        next[axis - 1] = (uint16_t)constrain(v, 0L, 4095L);
    } else if (cmd == 'A') {
        for (int i = 0; i < 6; i++) {
            char* end;
            long val = strtol(args, &end, 10);
            if (end == args) { Serial.println("ERR 6 werte noetig"); return; }
            next[i] = (uint16_t)constrain(val, 0L, 4095L);
            args = end;
        }
    } else if (cmd == 'H') {
        for (int i = 0; i < 6; i++) next[i] = 2048;
    } else if (cmd != '?') {
        Serial.println("ERR unbekannter befehl (M, P, A, H, ?)");
        return;
    }

    if (cmd != '?') driveServos(next);
    printAxisValues();
}

// Liest verfügbare Zeichen nicht-blockierend und führt jede fertige Zeile aus.
static void pollSerial() {
    static char buf[SERIAL_CMD_MAXLEN];
    static uint8_t len = 0;
    static bool overflow = false;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf[len] = '\0';
            if (overflow) Serial.println("ERR zeile zu lang");
            else          handleSerialCommand(buf);
            len = 0;
            overflow = false;
        } else if (len < SERIAL_CMD_MAXLEN - 1) {
            buf[len++] = c;
        } else {
            overflow = true;
        }
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 5000) delay(10);

    Serial.println("[Receiver] Servo limits:");
    for (int i = 0; i < 6; i++) {
        lastSentPos[i] = -9999;  // force first write
        int start = (LIMITS[i].minPos + LIMITS[i].maxPos) / 2;
        Serial.printf("  S%d: min=%d  max=%d  start=%d\n",
                      i + 1, LIMITS[i].minPos, LIMITS[i].maxPos, start);
    }

    Serial1.begin(1000000, SERIAL_8N1, SERVO_RXD, SERVO_TXD);
    servos.pSerial = &Serial1;
    delay(1000);

    Serial.printf("[Receiver] Servo ping (RXD=%d, TXD=%d):\n", SERVO_RXD, SERVO_TXD);
    for (int id = 1; id <= 6; id++) {
        int r = servos.Ping(id);
        Serial.printf("  Servo %d: %s\n", id, r != -1 ? "OK" : "--");
    }

    // Ist-Positionen einlesen → axisValue, damit der erste Serial-Befehl
    // (z.B. "M 1 +40") von der echten Pose aus fährt statt zur Mitte zu springen.
    for (int i = 0; i < 6; i++) {
        int pos = servos.ReadPos(SERVO_IDS[i]);
        if (pos != -1) {
            pos = constrain(pos, LIMITS[i].minPos, LIMITS[i].maxPos);
            axisValue[i] = (uint16_t)map(pos, LIMITS[i].minPos, LIMITS[i].maxPos, 0, 4095);
        }
    }

    // Primärschutz: Hardware-Drehmoment-Limit pro Servo. Deckelt PWM → Strom →
    // stationäre Temperatur. = maximale „Zupack"-Kraft, dauerhaft thermisch sicher.
    Serial.println("[Receiver] Drehmoment-Limits (Kraft-Deckel):");
    for (int i = 0; i < 6; i++) {
        servos.writeWord(SERVO_IDS[i], SMS_STS_TORQUE_LIMIT_L, TORQUE_LIMIT[i]);
        Serial.printf("  S%d: %d/1000\n", i + 1, TORQUE_LIMIT[i]);
    }

    robotLink.setKinematics(KINEMATICS);

    if (!robotLink.beginReceiver(onFrame)) {
        Serial.println("[Receiver] ESP-NOW init FAILED");
    } else {
        Serial.printf("[Receiver] ESP-NOW ready — Mode %d\n", robotLink.getMode());
    }
    Serial.println("[Receiver] Serial-Befehle: M <achse> <delta> | P <achse> <wert> | A <w1..w6> | H | ?");
    printAxisValues();
}

// ─── Protection polling ───────────────────────────────────────────────────────

// Übertemperatur: Motor ausschalten (Torque AUS), damit er sich nicht weiter
// aufheizt. Das schnelle rote Blinken (siehe pollProtection) signalisiert es.
// ACHTUNG: Ein abgeschaltetes Gelenk hält keine Last mehr — der Arm kann unter
// Schwerkraft absacken. Wieder ein bei Abkühlung unter TEMP_RESET_C (Hysterese).
static void handleTemp(int idx, int temp) {
    if (temp >= TEMP_CRIT_C && !servoTorqueOff[idx]) {
        servos.EnableTorque(SERVO_IDS[idx], 0);
        servoTorqueOff[idx] = true;
        Serial.printf("[S%d] ZU WARM %d°C — Motor AUS\n", idx + 1, temp);
    } else if (servoTorqueOff[idx] && temp <= TEMP_RESET_C) {
        servos.EnableTorque(SERVO_IDS[idx], 1);
        servoTorqueOff[idx] = false;
        Serial.printf("[S%d] auf %d°C abgekühlt — Motor wieder AN\n", idx + 1, temp);
    }
}

#ifdef ROBOLINK_MONITOR
// Liest Temp + Last + Strom aller 6 Servos und gibt sie in Serial-Plotter-Syntax
// aus: "S1_T:<°C> S1_L:<load> S1_C:<|current|> S2_T:… ". Ein FeedBack(id)-Paket
// pro Servo lädt alle Register in den Puffer, danach Read*(-1) ohne weiteren
// Bus-Verkehr — der Strom (das verlässliche Blockier-Signal) kostet nichts extra.
// Mit S*_C werden die STALL_CURRENT-Schwellen kalibriert (siehe Header).
static void pollMonitor() {
    unsigned long now = millis();
    if (now - lastMonitorMs < MONITOR_POLL_INTERVAL_MS) return;
    lastMonitorMs = now;

    // Achsen-Namen für die Plotter-Legende. Servo 6 = Greifer, klar als eigene
    // Achse benannt (sonst geht er als "S6" unter).
    static const char* AXIS_NAME[6] = { "S1", "S2", "S3", "S4", "S5", "GRIP" };
#if   MONITOR_METRIC == MON_LOAD
    const char* suffix = "_L";
#elif MONITOR_METRIC == MON_TEMP
    const char* suffix = "_T";
#else
    const char* suffix = "_C";
#endif

    for (int i = 0; i < 6; i++) {
        int val = -1;
        if (servos.FeedBack(SERVO_IDS[i]) != -1) {
#if   MONITOR_METRIC == MON_LOAD
            val = abs(servos.ReadLoad(-1));
#elif MONITOR_METRIC == MON_TEMP
            val = servos.ReadTemper(-1);
#else
            val = abs(servos.ReadCurrent(-1));
#endif
        }
        // Bei Lesefehler 0 ausgeben, damit die Serie im Plotter nicht springt.
        // 6 Serien (eine je Achse) → passt in das 8-Serien-Limit des Plotters.
        Serial.printf("%s%s:%d ", AXIS_NAME[i], suffix, val < 0 ? 0 : val);
    }
    Serial.println();
}
#endif

// Strombasierte Force-Überlast-Erkennung für ein Gelenk. Überlast = zieht
// anhaltend Strom über der Schwelle, steht dabei (Speed≈0) und erreicht sein
// Ziel nicht → drückt gegen einen Block. Der Zeitfilter (STALL_DURATION_MS)
// blendet Anlauf-/Richtungswechsel-Spitzen aus. Reaktion: nur Warnung
// (langsames rotes Blinken); der Motor greift/hält weiter, thermisch durch das
// Hardware-Drehmoment-Limit gedeckelt. Kein Klemmen/Einfrieren.
static void detectStall(int idx, int current, int actualPos, int speed, unsigned long now) {
    int target = lastSentPos[idx];
    bool pressing = (current >= STALL_CURRENT[idx])
                 && (abs(target - actualPos) > STALL_POS_TOL[idx])
                 && (abs(speed) < SPEED_EPS);

    if (pressing) {
        if (stallStartMs[idx] == 0) {
            stallStartMs[idx] = now;
        } else if (!forceStalled[idx] && (now - stallStartMs[idx]) >= STALL_DURATION_MS) {
            forceStalled[idx] = true;
            Serial.printf("[S%d] Force-Überlast (I=%d) — drückt gegen Block\n", idx + 1, current);
        }
    } else if (current < STALL_CURRENT[idx] - CURRENT_HYST) {
        stallStartMs[idx]  = 0;      // Strom klar gefallen → zurücksetzen (Hysterese)
        forceStalled[idx]  = false;
    }
}

// Ein Servo pro Tick abfragen (Round-Robin). Ein FeedBack(id) lädt Pos/Speed/
// Load/Temp/Strom in einen Puffer; danach Read*(-1) ohne weiteren Bus-Verkehr.
// So bleibt jede loop()-Iteration billig (<1 ms Bus) und der volle Durchlauf
// über alle 6 dauert ~120 ms — mehrfach innerhalb des Stall-Fensters.
static void pollProtection() {
    unsigned long now = millis();
    if (now - lastProtectPollMs < PROTECT_POLL_INTERVAL_MS) return;
    lastProtectPollMs = now;

    int idx = pollCursor;
    pollCursor = (pollCursor + 1) % 6;

    if (servos.FeedBack(SERVO_IDS[idx]) != -1) {
        int actualPos = servos.ReadPos(-1);
        int speed     = servos.ReadSpeed(-1);
        int current   = abs(servos.ReadCurrent(-1));
        int temp      = servos.ReadTemper(-1);

        // Einschaltstrom nach Boot ignorieren (sonst Fehlauslösung).
        if (now >= STARTUP_IGNORE_MS) detectStall(idx, current, actualPos, speed, now);
        handleTemp(idx, temp);
    }

    // Rotes Blinken als Warnung. Übertemperatur (Motor aus) hat Vorrang und
    // blinkt DEUTLICH SCHNELLER; Force-Überlast blinkt langsam.
    bool overtemp = false, forceWarn = false;
    for (int i = 0; i < 6; i++) {
        if (servoTorqueOff[i]) overtemp = true;
        if (forceStalled[i])   forceWarn = true;
    }
    unsigned long interval = overtemp ? OVERTEMP_BLINK_INTERVAL_MS : FORCE_BLINK_INTERVAL_MS;
    if ((overtemp || forceWarn) && (now - lastBlinkMs) >= interval) {
        lastBlinkMs = now;
        robotLink.flashAttention(CRGB(200, 0, 0), overtemp ? 80 : 150);  // rot
    }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
    robotLink.update();  // no delay — process frames as fast as the bus allows
    pollSerial();        // Textbefehle über USB-Serial (siehe "Serial-Steuerung")
    pollProtection();
#ifdef ROBOLINK_MONITOR
    pollMonitor();       // nur mit -D ROBOLINK_MONITOR: Temp+Force aller Servos plotten
#endif
}
