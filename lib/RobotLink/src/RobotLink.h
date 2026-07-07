#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>
#include <Smoothed.h>

// ─── Config ───────────────────────────────────────────────────────────────────

#ifndef ROBOTLINK_LED_PIN
#define ROBOTLINK_LED_PIN      38   // default: ESP32-S3-DevKitC-1 onboard LED
#endif
#ifndef ROBOTLINK_BTN_PIN
#define ROBOTLINK_BTN_PIN      -1   // default: no mode button (-1 = disabled)
#endif
#define ROBOTLINK_NUM_LEDS      1
#define ROBOTLINK_NUM_AXES      6
#define ROBOTLINK_DEBOUNCE_MS   40        // button debounce (ignore bouncing edges)
#define ROBOTLINK_WIFI_CHANNEL  1         // fixed channel — avoids scan overhead

// ─── CoordMode ────────────────────────────────────────────────────────────────

enum CoordMode : uint8_t {
    COORD_DIRECT      = 0,  // raw potis → servos (default)
    COORD_CYLINDRICAL = 1,  // (base_rot, elevation, radius, wrist, -, gripper) → IK
    COORD_CARTESIAN   = 2,  // (X, Y, Z, pitch, wrist, gripper) → IK
};

// ─── Runtime parameters ───────────────────────────────────────────────────────
// Generic, low-rate parameters sent out-of-band from the axis stream (see
// RobotParamFrame). Add future runtime-settable parameters here — the protocol,
// persistence, and dispatch are written to scale to new IDs without changes.
enum RobotParam : uint8_t {
    PARAM_YAW_OFFSET = 0,  // float, rad — rotation of the cartesian frame about Z (session-only, resets to 0 on reboot)
    PARAM_MAX_SPEED  = 1,  // servo speed units (steps/s); 0 = max/unbegrenzt
    PARAM_CART_DIRECT_PITCH = 2,  // float 0/1 — Cartesian: 1 = pitch drives servo 4
                                  //   directly (wrist joint, decoupled); 0 = legacy
                                  //   tip-referenced pitch (couples shoulder/elbow). Persisted.
    PARAM_COUNT            // keep last — number of parameters
};

// ─── Kinematics (receiver-side) ───────────────────────────────────────────────

struct JointLimits { int minPos; int maxPos; };

// Wrist tilt strategy for cylindrical IK (axis 4 / servo 4).
enum WristMode : uint8_t {
    WRIST_PARALLEL_GAMMA = 0,  // L3 zeigt entlang Elevations-Linie (radial)
    WRIST_HORIZONTAL     = 1,  // L3 bleibt waagerecht, unabhängig von γ
};

// Behaviour when a cartesian target cannot be reached within the limits.
enum CartesianLimitMode : uint8_t {
    CART_LIMIT_CLAMP   = 0,  // pro Gelenk klemmen — Arm fährt so nah wie möglich
                             //   und bleibt am Anschlag stehen (Default, kann Pose verzerren)
    CART_LIMIT_HOLD    = 1,  // letzte gültige Pose halten, sobald ein Gelenk anschlägt
    CART_LIMIT_PROJECT = 2,  // Ziel auf nächsten erreichbaren Punkt projizieren
                             //   (Reichweite begrenzt, Richtung/Pitch bleiben erhalten)
};

// Configuration for cylindrical IK.
// jointZeroPos[i]: servo position (units) when joint i is at 0 rad.
// jointScale[i]:   servo units per radian. Negative = reversed direction.
//                  Typical SMS-STS value: ±651.9 (= 2048 / PI)
// limits[i]:       hardware min/max positions (same array used in receiver main.cpp).
// rMin/rMax:       shoulder→wrist distance range mapped from poti 2 (mm).
//                  Must satisfy |L1-L2| < rMin and rMax < L1+L2 (triangle inequality).
// elevMin/elevMax: elevation range mapped from poti 1 (radians, 0 = horizontal).
struct KinematicsConfig {
    float L1, L2, L3;          // link lengths in mm (shoulder→elbow, elbow→forearm, forearm→wrist)
    float jointZeroPos[6];     // servo position at 0-rad reference angle, per joint
    float jointScale[6];       // servo units per radian, per joint
    JointLimits limits[6];     // hardware limits per joint
    // Cylindrical
    float rMin;                // minimum shoulder→wrist distance in mm
    float rMax;                // maximum shoulder→wrist distance in mm
    float elevMin;             // minimum elevation in radians
    float elevMax;             // maximum elevation in radians
    WristMode wristMode;       // wrist-tilt fallback when cylUsePitchAxis = false
    bool  cylUsePitchAxis;     // true: axis 4 sets absolute tool pitch η (pitchMin…pitchMax);
                               // false: wrist tilt derived from wristMode (legacy behaviour)
    // Cartesian
    float xMin, xMax;          // X range in mm  (vorne/hinten)
    float yMin, yMax;          // Y range in mm  (links/rechts)
    float zMin, zMax;          // Z range in mm  (hoch/runter)
    float pitchMin, pitchMax;  // Tool-Pitch η range in rad (0 = horizontal).
                               // Used by cartesian axis 3 and cylindrical axis 4 (if cylUsePitchAxis).
    CartesianLimitMode cartLimitMode;  // Verhalten bei unerreichbarem Ziel (Default: CLAMP)
};

// ─── Frame (all axes in one packet, 12-bit packed) ───────────────────────────
// 6 axes × 12 bit = 72 bit = 9 bytes payload.
// receiverID byte layout: [coordMode:5][receiverID:3]
//   receiverID uses 1–5 (fits in 3 bits), coordMode uses upper 5 bits.
// Total on wire: 1 (receiverID+coordMode) + 9 (packed) + 1 (checksum) = 11 bytes.

struct RobotFrame {
    uint8_t receiverID;       // bits[2:0] = target mode, bits[7:3] = CoordMode
    uint8_t packed[9];        // 6 × 12-bit values, big-endian nibble pairs
    uint8_t checksum;         // XOR over all bytes except checksum
} __attribute__((packed));    // 11 bytes total

// Encode 6 values (0–4095) into frame.packed
inline void robotFramePack(RobotFrame& f, const uint16_t v[6]) {
    f.packed[0] =  v[0] >> 4;
    f.packed[1] = (v[0] << 4) | (v[1] >> 8);
    f.packed[2] =  v[1] & 0xFF;
    f.packed[3] =  v[2] >> 4;
    f.packed[4] = (v[2] << 4) | (v[3] >> 8);
    f.packed[5] =  v[3] & 0xFF;
    f.packed[6] =  v[4] >> 4;
    f.packed[7] = (v[4] << 4) | (v[5] >> 8);
    f.packed[8] =  v[5] & 0xFF;
}

// Decode frame.packed into 6 values
inline void robotFrameUnpack(const RobotFrame& f, uint16_t v[6]) {
    v[0] = ((uint16_t)f.packed[0] << 4) | (f.packed[1] >> 4);
    v[1] = ((uint16_t)(f.packed[1] & 0x0F) << 8) | f.packed[2];
    v[2] = ((uint16_t)f.packed[3] << 4) | (f.packed[4] >> 4);
    v[3] = ((uint16_t)(f.packed[4] & 0x0F) << 8) | f.packed[5];
    v[4] = ((uint16_t)f.packed[6] << 4) | (f.packed[7] >> 4);
    v[5] = ((uint16_t)(f.packed[7] & 0x0F) << 8) | f.packed[8];
}

// ─── Parameter frame (out-of-band runtime parameters) ─────────────────────────
// A second, shorter packet carrying a single (paramID, value) pair. It is told
// apart from RobotFrame purely by length (9 ≠ 11 bytes) plus a marker byte.
// Sent at low rate (only on change), so the float payload costs nothing in the
// normal axis stream.
#define ROBOTLINK_PARAM_MARKER  0xA5   // type discriminator for RobotParamFrame

struct RobotParamFrame {
    uint8_t  receiverID;   // bits[2:0] = target mode (same filter as RobotFrame)
    uint8_t  marker;       // ROBOTLINK_PARAM_MARKER — guards against length collisions
    uint8_t  paramID;      // RobotParam
    uint8_t  reserved;     // padding / future flags
    float    value;        // parameter payload
    uint8_t  checksum;     // XOR over all bytes except checksum
} __attribute__((packed)); // 9 bytes total
static_assert(sizeof(RobotParamFrame) != sizeof(RobotFrame),
              "param frame must differ in size from axis frame for len-based routing");

// ─── Callback ─────────────────────────────────────────────────────────────────

// Called on the receiver when a valid frame arrives.
// In COORD_DIRECT mode: values[0..5] are raw 12-bit poti values (0–4095).
// In COORD_CYLINDRICAL mode: values[0..5] are already servo-unit-equivalent
//   values (0–4095 normalised), IK has been applied by the library.
using RobotFrameCallback = void (*)(uint8_t receiverID,
                                    const uint16_t values[ROBOTLINK_NUM_AXES]);

// ─── RobotLink ────────────────────────────────────────────────────────────────

class RobotLink {
public:
    // ── Sender ───────────────────────────────────────────────────────────────

    // Initialises ESP-NOW, LED, and checks for double-reset mode change.
    bool beginSender(const uint8_t* peerMac = nullptr);

    // Set coordinate mode. Call before beginSender().
    // COORD_DIRECT (default): raw poti values are sent as-is.
    // COORD_CYLINDRICAL: sender semantics — axis0=base_rot, axis1=elevation,
    //   axis2=radius, axis3=wrist, axis4=unused, axis5=gripper.
    void setCoordMode(CoordMode mode);

    // Enable smoothing for an axis.
    //   strength: 1 (almost no smoothing) … 50 (heavy smoothing)
    //   Default: no smoothing (raw values passed through).
    void setAxisSmoothing(uint8_t axisIndex, uint8_t strength);

    // Push a raw sensor value. Smoothing (if configured) is applied internally.
    void setAxisValue(uint8_t axisIndex, uint16_t value);

    // ── Semantic axis setters ──────────────────────────────────────────────
    // Thin, self-documenting wrappers around setAxisValue() with the fixed axis
    // index for each mode. All take a raw 0–4095 value (e.g. analogRead()); the
    // mapping to mm / radians happens on the receiver (KinematicsConfig).

    // COORD_CARTESIAN — (X, Y, Z, pitch, wrist-rotate, gripper)
    void setX(uint16_t v)           { setAxisValue(0, v); }  // vorne/hinten
    void setY(uint16_t v)           { setAxisValue(1, v); }  // links/rechts
    void setZ(uint16_t v)           { setAxisValue(2, v); }  // hoch/runter
    void setPitch(uint16_t v)       { setAxisValue(3, v); }  // Werkzeug-Neigung η
    void setWristRotate(uint16_t v) { setAxisValue(4, v); }  // Handgelenk-Rotation

    // COORD_CYLINDRICAL — (base-rot, elevation, radius, wrist, pitch, gripper)
    void setBaseRotation(uint16_t v){ setAxisValue(0, v); }  // Basisdrehung
    void setElevation(uint16_t v)   { setAxisValue(1, v); }  // Elevationswinkel
    void setRadius(uint16_t v)      { setAxisValue(2, v); }  // Ein-/Ausfahren
    void setWrist(uint16_t v)       { setAxisValue(3, v); }  // Handgelenk-Rotation
    void setElevationPitch(uint16_t v){ setAxisValue(4, v); }// Werkzeug-Neigung η (cylUsePitchAxis)

    // Shared (axis 5 in both modes)
    void setGripper(uint16_t v)     { setAxisValue(5, v); }  // Greifer

    // Send all axis values in a single ESP-NOW frame.
    void sendAllAxes();

    // Send a single runtime parameter to the receiver (out-of-band, low rate).
    // Use for values that are set/calibrated rather than streamed every frame,
    // e.g. PARAM_YAW_OFFSET. The receiver persists it across resets.
    // Call only on change to keep the radio quiet.
    void sendParam(RobotParam param, float value);

    // Set the maximum servo speed (target velocity) applied to all servos.
    // Units are SMS-STS speed steps/s; 0 = unbegrenzt (maximale Geschwindigkeit).
    // Persisted on the receiver. Convenience wrapper around sendParam().
    void setMaxSpeed(uint16_t speed) { sendParam(PARAM_MAX_SPEED, (float)speed); }

    // COORD_CARTESIAN: when true, tool pitch (axis 3) drives the wrist joint
    // (servo 4) directly — pitch no longer moves the shoulder/elbow and X/Y/Z
    // target the wrist joint instead of the tool tip. When false (default), the
    // legacy tip-referenced pitch is used. Persisted on the receiver.
    void setCartDirectPitch(bool on) { sendParam(PARAM_CART_DIRECT_PITCH, on ? 1.0f : 0.0f); }

    // ── Receiver ─────────────────────────────────────────────────────────────

    // Initialises ESP-NOW, LED, and restores mode from Preferences.
    bool beginReceiver(RobotFrameCallback cb);

    // Configure inverse kinematics for COORD_CYLINDRICAL mode.
    // Call before or after beginReceiver(). If not called, cylindrical frames
    // are passed through as raw values (same as COORD_DIRECT).
    void setKinematics(const KinematicsConfig& cfg);

    // Call in loop() — drains the RX queue and fires the callback.
    void update();

    // ── Shared ───────────────────────────────────────────────────────────────

    // Returns the current mode (1–5), persisted across resets.
    uint8_t getMode() const;

    // Sets mode, updates LED, and saves to Preferences.
    void setMode(uint8_t mode);

    // Updates only the LED without touching Preferences.
    void setModeLED(uint8_t mode);

    // Returns the current value of a runtime parameter (receiver-side).
    // Reflects the last value received and persisted (or the loaded default).
    float getParam(RobotParam param) const;

    // Brief LED override; auto-restores to mode color after durationMs.
    void flashAttention(CRGB color = CRGB(255, 0, 0), uint16_t durationMs = 150);

private:
    // ── LED ──────────────────────────────────────────────────────────────────
    CRGB _leds[ROBOTLINK_NUM_LEDS];
    unsigned long _flashUntilMs = 0;
    void _initLED();
    void _showRoleAnimation(bool isSender);

    // ── Mode + Preferences ───────────────────────────────────────────────────
    uint8_t _mode = 1;
    void    _loadMode();
    void    _saveMode();

    // ── Frame helpers ────────────────────────────────────────────────────────
    static uint8_t _calcChecksum(const RobotFrame& f);

    // ── Sender state ─────────────────────────────────────────────────────────
    uint16_t        _axisValue[ROBOTLINK_NUM_AXES]    = {};
    bool            _axisSmoothed[ROBOTLINK_NUM_AXES] = {};
    Smoothed<float> _smoother[ROBOTLINK_NUM_AXES];
    uint8_t         _peerMac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    CoordMode       _coordMode  = COORD_DIRECT;

    // ── Receiver state ───────────────────────────────────────────────────────
    RobotFrameCallback _callback     = nullptr;
    volatile bool      _hasFrame     = false;
    RobotFrame         _rxBuf        = {};
    volatile CoordMode _rxCoordMode  = COORD_DIRECT;
    bool               _hasKinematics = false;
    KinematicsConfig   _kin          = {};

    // Runtime parameters (receiver-side), persisted in Preferences.
    float              _params[PARAM_COUNT] = {};
    // Parameter RX hand-off from the ESP-NOW callback to update().
    volatile bool      _hasParam     = false;
    volatile uint8_t   _rxParamID    = 0;
    volatile float     _rxParamValue = 0.0f;
    void   _loadParams();
    void   _applyParam(uint8_t paramID, float value);  // store + persist

    // Last valid cartesian servo output, for CART_LIMIT_HOLD.
    mutable uint16_t   _lastCartOut[ROBOTLINK_NUM_AXES] = {};
    mutable bool       _lastCartValid = false;

    // Applies cylindrical IK to raw values, writes servo-normalised output.
    void _applyCylindricalIK(const uint16_t raw[6], uint16_t out[6]) const;
    // Applies cartesian IK (X, Y, Z, pitch) to raw values.
    void _applyCartesianIK(const uint16_t raw[6], uint16_t out[6]) const;

    // ── Mode button (BOOT button single click) ─────────────────────────────────
    bool          _btnPrev    = false;
    unsigned long _lastEdgeAt = 0;  // last debounced press edge (for debounce)
    void _checkModeButton();

    // ── ISR bridge ───────────────────────────────────────────────────────────
    static RobotLink* _instance;
    static void _onReceive(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len);
};

extern RobotLink robotLink;
