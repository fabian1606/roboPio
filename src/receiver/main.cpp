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

// ─── Overload protection ──────────────────────────────────────────────────────
#define GRIPPER_IDX               5     // index in arrays (servo 6)
#define GRIPPER_TORQUE_LIMIT      600   // 0..1000 — hardware PWM cap on servo 6
#define STALL_LOAD_THRESHOLD      700   // |Load| (0..1023) considered "pressing"
#define STALL_POS_TOLERANCE       20    // |target - actual| to count as "blocked"
#define STALL_DURATION_MS         400   // sustained stall before clamping
#define TEMP_CRIT_C               70    // °C — disable torque above this
#define TEMP_RESET_C              55    // °C — hysteresis: re-enable below this
#define GRIPPER_POLL_INTERVAL_MS  100
#define TEMP_POLL_INTERVAL_MS     200   // round-robin step (full sweep ≈ 1.2 s)
#define CLAMP_BLINK_INTERVAL_MS   800   // blue blink while clamp is active

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

// ─── Protection state ─────────────────────────────────────────────────────────
static unsigned long stallStartMs      = 0;
static bool          gripperClamped    = false;
static int           clampedPos        = 0;
static int           stallDirection    = 0;       // +1 / -1 (sign of target-actual at stall)
static bool          servoTorqueOff[6]  = {false};
static unsigned long lastGripperPollMs  = 0;
static unsigned long lastTempPollMs     = 0;
static unsigned long lastClampBlinkMs   = 0;
static uint8_t       tempPollCursor     = 0;

// ─── ESP-NOW frame callback ───────────────────────────────────────────────────
// The lib's single-entry queue guarantees this is always the LATEST frame.
// Intermediate positions from fast poti sweeps are automatically discarded —
// the servo never sees them and goes directly to the current target.

void onFrame(uint8_t /*receiverID*/, const uint16_t values[6]) {
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
        int pos = map((int)values[i], 0, 4095,
                      LIMITS[i].minPos, LIMITS[i].maxPos);
        pos = constrain(pos, LIMITS[i].minPos, LIMITS[i].maxPos);

        // Gripper overload clamp: don't allow pushing further in the stall direction.
        if (i == GRIPPER_IDX && gripperClamped) {
            bool opening = (stallDirection > 0 && pos < clampedPos - SERVO_DEADBAND) ||
                           (stallDirection < 0 && pos > clampedPos + SERVO_DEADBAND);
            if (opening) {
                gripperClamped = false;
                stallStartMs   = 0;
                Serial.println("[Gripper] clamp released");
            } else if (stallDirection > 0) {
                pos = min(pos, clampedPos);
            } else if (stallDirection < 0) {
                pos = max(pos, clampedPos);
            }
        }

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

    // Hardware PWM cap on the gripper — physically limits stall current.
    servos.writeWord(SERVO_IDS[GRIPPER_IDX], SMS_STS_TORQUE_LIMIT_L, GRIPPER_TORQUE_LIMIT);
    Serial.printf("[Receiver] Gripper torque limit set to %d/1000\n", GRIPPER_TORQUE_LIMIT);

    robotLink.setKinematics(KINEMATICS);

    if (!robotLink.beginReceiver(onFrame)) {
        Serial.println("[Receiver] ESP-NOW init FAILED");
    } else {
        Serial.printf("[Receiver] ESP-NOW ready — Mode %d\n", robotLink.getMode());
    }
}

// ─── Protection polling ───────────────────────────────────────────────────────

static void handleTemp(int idx, int temp) {
    if (temp >= TEMP_CRIT_C && !servoTorqueOff[idx]) {
        servos.EnableTorque(SERVO_IDS[idx], 0);
        servoTorqueOff[idx] = true;
        robotLink.flashAttention();
        Serial.printf("[S%d] OVERTEMP %d°C — torque OFF\n", idx + 1, temp);
    } else if (servoTorqueOff[idx] && temp <= TEMP_RESET_C) {
        servos.EnableTorque(SERVO_IDS[idx], 1);
        servoTorqueOff[idx] = false;
        Serial.printf("[S%d] cooled to %d°C — torque back ON\n", idx + 1, temp);
    }
}

static void pollProtection() {
    unsigned long now = millis();

    // Gripper: poll load + temp every GRIPPER_POLL_INTERVAL_MS.
    if (now - lastGripperPollMs >= GRIPPER_POLL_INTERVAL_MS) {
        lastGripperPollMs = now;
        if (servos.FeedBack(SERVO_IDS[GRIPPER_IDX]) != -1) {
            int load      = abs(servos.ReadLoad(-1));
            int actualPos = servos.ReadPos(-1);
            int temp      = servos.ReadTemper(-1);
            int target    = lastSentPos[GRIPPER_IDX];

            bool blocked = (load >= STALL_LOAD_THRESHOLD)
                        && (abs(target - actualPos) > STALL_POS_TOLERANCE);
            if (blocked) {
                if (stallStartMs == 0) {
                    stallStartMs = now;
                } else if (!gripperClamped && (now - stallStartMs) >= STALL_DURATION_MS) {
                    gripperClamped = true;
                    clampedPos     = actualPos;
                    stallDirection = (target > actualPos) ? +1 : -1;
                    robotLink.flashAttention();
                    Serial.printf("[Gripper] STALL clamp at pos=%d (load=%d, dir=%+d)\n",
                                  actualPos, load, stallDirection);
                }
            } else if (!gripperClamped) {
                stallStartMs = 0;
            }
            handleTemp(GRIPPER_IDX, temp);
        }

        // Blue heartbeat while clamp is active (position not reachable).
        if (gripperClamped && (now - lastClampBlinkMs) >= CLAMP_BLINK_INTERVAL_MS) {
            lastClampBlinkMs = now;
            robotLink.flashAttention(CRGB(0, 0, 200));
        }
    }

    // Other servos: temperature round-robin (skip gripper, already polled above).
    if (now - lastTempPollMs >= TEMP_POLL_INTERVAL_MS) {
        lastTempPollMs = now;
        if (tempPollCursor == GRIPPER_IDX) tempPollCursor = (tempPollCursor + 1) % 6;
        int t = servos.ReadTemper(SERVO_IDS[tempPollCursor]);
        if (t != -1) handleTemp(tempPollCursor, t);
        tempPollCursor = (tempPollCursor + 1) % 6;
    }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
    robotLink.update();  // no delay — process frames as fast as the bus allows
    pollProtection();
}
