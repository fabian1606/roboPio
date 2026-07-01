#include "RobotLink.h"
#include <Preferences.h>
#include <esp_wifi.h>
#include <math.h>

RobotLink  robotLink;
RobotLink* RobotLink::_instance = nullptr;

// ─── LED ──────────────────────────────────────────────────────────────────────

static const CRGB MODE_COLORS[6] = {
    CRGB::Black,         // 0 — unused
    CRGB( 80,  80,  80), // 1 — white
    CRGB(180,   0,   0), // 2 — red
    CRGB(  0, 180,   0), // 3 — green
    CRGB(  0,   0, 180), // 4 — blue
    CRGB(150,   0, 150), // 5 — magenta
};

void RobotLink::_initLED() {
    FastLED.addLeds<NEOPIXEL, ROBOTLINK_LED_PIN>(_leds, ROBOTLINK_NUM_LEDS);
    FastLED.setBrightness(180);
    _leds[0] = CRGB::Black;
    FastLED.show();
}

void RobotLink::_showRoleAnimation(bool isSender) {
    CRGB color = isSender ? CRGB(0, 180, 180)  // cyan   = sender
                          : CRGB(180, 100, 0);  // orange = receiver
    for (int i = 0; i < 3; i++) {
        _leds[0] = color;       FastLED.show(); delay(150);
        _leds[0] = CRGB::Black; FastLED.show(); delay(150);
    }
}

void RobotLink::setModeLED(uint8_t mode) {
    _leds[0] = (mode >= 1 && mode <= 5) ? MODE_COLORS[mode] : CRGB::Black;
    FastLED.show();
}

void RobotLink::flashAttention(CRGB color, uint16_t durationMs) {
    _leds[0] = color;
    FastLED.show();
    _flashUntilMs = millis() + durationMs;
}

// ─── Mode + Preferences ───────────────────────────────────────────────────────

void RobotLink::_loadMode() {
    Preferences prefs;
    prefs.begin("robotlink", false);
    _mode = prefs.getUChar("mode", 1);
    prefs.end();
}

void RobotLink::_saveMode() {
    Preferences prefs;
    prefs.begin("robotlink", false);
    prefs.putUChar("mode", _mode);
    prefs.end();
}

uint8_t RobotLink::getMode() const { return _mode; }

void RobotLink::setMode(uint8_t mode) {
    _mode = mode;
    _saveMode();
    setModeLED(mode);
}

// ─── Frame checksum ───────────────────────────────────────────────────────────

uint8_t RobotLink::_calcChecksum(const RobotFrame& f) {
    const uint8_t* p = (const uint8_t*)&f;
    uint8_t cs = 0;
    for (size_t i = 0; i < sizeof(RobotFrame) - 1; i++) cs ^= p[i];
    return cs;
}


// ─── WiFi init (shared) ───────────────────────────────────────────────────────

static bool _initWiFi() {
    WiFi.mode(WIFI_STA);
    // Disable power-save mode: removes the ~100 ms beacon-interval wake-up jitter
    esp_wifi_set_ps(WIFI_PS_NONE);
    return esp_now_init() == ESP_OK;
}

// ─── Sender ───────────────────────────────────────────────────────────────────

bool RobotLink::beginSender(const uint8_t* peerMac) {
    _instance = this;

    _initLED();
    _loadMode();
    _showRoleAnimation(true);
    setModeLED(_mode);

#if ROBOTLINK_BTN_PIN >= 0
    pinMode(ROBOTLINK_BTN_PIN, INPUT_PULLUP);
#endif

    if (peerMac) memcpy(_peerMac, peerMac, 6);

    if (!_initWiFi()) return false;

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, _peerMac, 6);
    peer.channel = ROBOTLINK_WIFI_CHANNEL; // fixed channel — no scan needed
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    return true;
}

void RobotLink::setCoordMode(CoordMode mode) {
    _coordMode = mode;
}

void RobotLink::setAxisSmoothing(uint8_t axisIndex, uint8_t strength) {
    if (axisIndex >= ROBOTLINK_NUM_AXES) return;
    _smoother[axisIndex].begin(SMOOTHED_EXPONENTIAL, strength);
    _axisSmoothed[axisIndex] = true;
}

void RobotLink::setAxisValue(uint8_t axisIndex, uint16_t value) {
    if (axisIndex >= ROBOTLINK_NUM_AXES) return;
    if (_axisSmoothed[axisIndex]) {
        _smoother[axisIndex].add((float)value);
        _axisValue[axisIndex] = (uint16_t)_smoother[axisIndex].get();
    } else {
        _axisValue[axisIndex] = value;
    }
}

void RobotLink::sendAllAxes() {
    RobotFrame frame;
    // Encode coordMode in upper 5 bits, receiverID (mode) in lower 3 bits.
    frame.receiverID = ((uint8_t)_coordMode << 3) | (_mode & 0x07);
    robotFramePack(frame, _axisValue);
    frame.checksum = _calcChecksum(frame);
    esp_now_send(_peerMac, (uint8_t*)&frame, sizeof(frame));
}

// XOR over all bytes of a param frame except the trailing checksum byte.
static uint8_t _calcParamChecksum(const RobotParamFrame& f) {
    const uint8_t* p = (const uint8_t*)&f;
    uint8_t cs = 0;
    for (size_t i = 0; i < sizeof(RobotParamFrame) - 1; i++) cs ^= p[i];
    return cs;
}

void RobotLink::sendParam(RobotParam param, float value) {
    RobotParamFrame pf;
    pf.receiverID = _mode & 0x07;
    pf.marker     = ROBOTLINK_PARAM_MARKER;
    pf.paramID    = (uint8_t)param;
    pf.reserved   = 0;
    pf.value      = value;
    pf.checksum   = _calcParamChecksum(pf);
    esp_now_send(_peerMac, (uint8_t*)&pf, sizeof(pf));
}

// ─── Receiver ─────────────────────────────────────────────────────────────────

void RobotLink::_onReceive(const esp_now_recv_info_t* /*info*/,
                            const uint8_t* data, int len) {
    if (!_instance) return;

    // ── Axis frame ───────────────────────────────────────────────────────────
    if (len == (int)sizeof(RobotFrame)) {
        RobotFrame frame;
        memcpy(&frame, data, sizeof(frame));

        if (_calcChecksum(frame) != frame.checksum) return;

        uint8_t id     = frame.receiverID & 0x07;
        CoordMode cmode = (CoordMode)(frame.receiverID >> 3);

        if (id != _instance->_mode) return;

        _instance->_rxBuf       = frame;
        _instance->_rxCoordMode = cmode;
        _instance->_hasFrame    = true;
        return;
    }

    // ── Parameter frame ──────────────────────────────────────────────────────
    if (len == (int)sizeof(RobotParamFrame)) {
        RobotParamFrame pf;
        memcpy(&pf, data, sizeof(pf));

        if (pf.marker != ROBOTLINK_PARAM_MARKER)        return;
        if (_calcParamChecksum(pf) != pf.checksum)      return;
        if ((pf.receiverID & 0x07) != _instance->_mode) return;
        if (pf.paramID >= PARAM_COUNT)                  return;

        _instance->_rxParamID    = pf.paramID;
        _instance->_rxParamValue = pf.value;
        _instance->_hasParam     = true;
        return;
    }
}

bool RobotLink::beginReceiver(RobotFrameCallback cb) {
    _instance = this;
    _callback = cb;

    _initLED();
    _loadMode();
    _loadParams();
    _showRoleAnimation(false);
    setModeLED(_mode);

#if ROBOTLINK_BTN_PIN >= 0
    pinMode(ROBOTLINK_BTN_PIN, INPUT_PULLUP);
#endif

    if (!_initWiFi()) return false;

    esp_now_register_recv_cb(_onReceive);
    return true;
}

// ─── Runtime parameters (receiver-side) ────────────────────────────────────────

// Preferences key per parameter. Short keys (NVS limit 15 chars).
static const char* _paramKey(uint8_t paramID) {
    switch (paramID) {
        case PARAM_YAW_OFFSET: return "p_yaw";
        case PARAM_MAX_SPEED:  return "p_speed";
        default:               return nullptr;
    }
}

void RobotLink::_loadParams() {
    Preferences prefs;
    prefs.begin("robotlink", false);
    // PARAM_YAW_OFFSET is session-only: clear any value persisted by older firmware.
    if (prefs.isKey("p_yaw")) prefs.remove("p_yaw");
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        if (i == PARAM_YAW_OFFSET) continue;  // session-only, always starts at 0
        const char* key = _paramKey(i);
        if (key) _params[i] = prefs.getFloat(key, 0.0f);
    }
    prefs.end();
}

void RobotLink::_applyParam(uint8_t paramID, float value) {
    if (paramID >= PARAM_COUNT) return;
    _params[paramID] = value;
    if (paramID == PARAM_YAW_OFFSET) {  // session-only — not written to NVS
        Serial.printf("[RobotLink] Param %u = %.4f (in-memory only)\n", paramID, value);
        return;
    }
    const char* key = _paramKey(paramID);
    if (key) {
        Preferences prefs;
        prefs.begin("robotlink", false);
        prefs.putFloat(key, value);
        prefs.end();
    }
    Serial.printf("[RobotLink] Param %u = %.4f (persisted)\n", paramID, value);
}

float RobotLink::getParam(RobotParam param) const {
    if ((uint8_t)param >= PARAM_COUNT) return 0.0f;
    return _params[param];
}

// Mode switching via a single click on the BOOT button (GPIO0, active-LOW).
// Each debounced press cycles the mode (1→2→3→4→5→1). This replaces the
// unreliable double-reset timing (which depended on the RTC clock surviving a
// reset) and works on any board that wires ROBOTLINK_BTN_PIN to the BOOT button.
void RobotLink::_checkModeButton() {
#if ROBOTLINK_BTN_PIN >= 0
    bool          pressed = (digitalRead(ROBOTLINK_BTN_PIN) == LOW);
    unsigned long now     = millis();

    // Debounced falling edge → switch mode.
    if (pressed && !_btnPrev && (now - _lastEdgeAt) > ROBOTLINK_DEBOUNCE_MS) {
        _lastEdgeAt = now;
        uint8_t newMode = (_mode % 5) + 1;
        setMode(newMode);
        Serial.printf("[RobotLink] BOOT click → Mode %d\n", newMode);
    }

    _btnPrev = pressed;
#endif
}

void RobotLink::setKinematics(const KinematicsConfig& cfg) {
    _kin = cfg;
    _hasKinematics = true;
}

// ─── Cylindrical IK ───────────────────────────────────────────────────────────
// Joint mapping:
//   out[0] = servo 1 (base rotation)  — direct pass-through
//   out[1] = servo 2 (shoulder)       — β - π/2
//   out[2] = servo 3 (elbow)          — α - π/2
//   out[3] = servo 4 (wrist tilt)     — θ4_rel (WristMode-abhängig)
//   out[4] = servo 5 (wrist rotate)   — direct from raw[3]
//   out[5] = servo 6 (gripper)        — direct from raw[5]
//
// Input poti semantics:
//   raw[0]: base rotation                  (0–4095)
//   raw[1]: elevation γ                    (0–4095 → elevMin…elevMax rad)
//   raw[2]: shoulder→wrist distance l      (0–4095 → rMin…rMax mm)
//   raw[3]: wrist rotate                   (pass-through zu servo 5)
//   raw[4]: tool pitch η                   (pitchMin…pitchMax rad, falls cylUsePitchAxis)
//   raw[5]: gripper                        (pass-through zu servo 6)
//
// Math (triangle S-E-W with sides a=L1, b=L2, l):
//   α    = arccos( (a²+b²-l²)/(2ab) )      Innenwinkel am Ellbogen
//   phi  = arccos( (a²+l²-b²)/(2al) )      Innenwinkel an der Schulter (Cosinussatz,
//                                          eindeutig für alle l in [|a-b|, a+b])
//   β    = phi + γ                          absoluter Schulterwinkel zur Horizontalen
//   δ    = β + α - π                        absoluter Unterarmwinkel zur Horizontalen
//   θ4_abs = η  (cylUsePitchAxis, Achse 4)  |  γ (WRIST_PARALLEL_GAMMA)  |  0 (WRIST_HORIZONTAL)
//   θ4_rel = θ4_abs - δ
//
// Servo zero offsets = mechanische Center-Pose (L1 vertikal, L2 horizontal, L3 colinear):
//   shoulder: β = π/2 → θ_servo = 0
//   elbow:    α = π/2 → θ_servo = 0
//   wrist:    θ4_rel = 0  (gilt im PARALLEL_GAMMA-Modus bei γ = 0)

void RobotLink::_applyCylindricalIK(const uint16_t raw[6], uint16_t out[6]) const {
    const float a = _kin.L1;
    const float b = _kin.L2;

    // ── Map poti values to physical quantities ───────────────────────────────
    float gamma = _kin.elevMin + (raw[1] / 4095.0f) * (_kin.elevMax - _kin.elevMin);
    float l     = _kin.rMin   + (raw[2] / 4095.0f) * (_kin.rMax   - _kin.rMin);

    // ── Triangle inner angles (law of cosines) ──────────────────────────────
    float cosA = (a*a + b*b - l*l) / (2.0f * a * b);
    cosA = constrain(cosA, -1.0f, 1.0f);
    float alpha = acosf(cosA);

    float cosP = (a*a + l*l - b*b) / (2.0f * a * l);
    cosP = constrain(cosP, -1.0f, 1.0f);
    float phi = acosf(cosP);

    // ── Shoulder angle β, forearm angle δ ───────────────────────────────────
    float beta  = phi + gamma;
    float delta = beta + alpha - (float)M_PI;

    // ── Wrist tilt — absolute L3 direction, then relative to L2 ─────────────
    // Pitch-Achse (4) hat Vorrang: η wird vom Bediener frei vorgegeben. Ohne
    // Pitch-Achse fällt es auf die statische WristMode-Logik zurück (Legacy).
    float theta4_abs;
    if (_kin.cylUsePitchAxis) {
        theta4_abs = _kin.pitchMin + (raw[4] / 4095.0f) * (_kin.pitchMax - _kin.pitchMin);
    } else {
        theta4_abs = (_kin.wristMode == WRIST_HORIZONTAL) ? 0.0f : gamma;
    }
    float theta4_rel = theta4_abs - delta;

    // ── Servo joint angles with mechanical center offsets ───────────────────
    float thetaShoulder = beta  - (float)M_PI / 2.0f;
    float thetaElbow    = alpha - (float)M_PI / 2.0f;
    float thetaWrist    = theta4_rel;

    Serial.printf("[IK] γ=%.1f° l=%.1f  α=%.1f° β=%.1f° δ=%.1f° θ4=%.1f°\n",
                  gamma * 180.0f / (float)M_PI, l,
                  alpha * 180.0f / (float)M_PI,
                  beta  * 180.0f / (float)M_PI,
                  delta * 180.0f / (float)M_PI,
                  theta4_rel * 180.0f / (float)M_PI);

    // ── Convert joint angles to servo positions, normalise to 0–4095 ────────
    auto angleToNorm = [&](int jointIdx, float theta) -> uint16_t {
        float pos_f = _kin.jointZeroPos[jointIdx] + theta * _kin.jointScale[jointIdx];
        int   pos   = (int)pos_f;
        pos = constrain(pos, _kin.limits[jointIdx].minPos, _kin.limits[jointIdx].maxPos);
        int range = _kin.limits[jointIdx].maxPos - _kin.limits[jointIdx].minPos;
        if (range <= 0) return 2047;
        int norm = (int)(((float)(pos - _kin.limits[jointIdx].minPos) / (float)range) * 4095.0f);
        return (uint16_t)constrain(norm, 0, 4095);
    };

    out[0] = raw[0];                          // base rotation — direct
    out[1] = angleToNorm(1, thetaShoulder);
    out[2] = angleToNorm(2, thetaElbow);
    out[3] = angleToNorm(3, thetaWrist);
    out[4] = raw[3];                          // wrist rotate — direct
    out[5] = raw[5];                          // gripper — direct
}

// ─── Cartesian IK ──────────────────────────────────────────────────────────────
// Joint mapping:
//   out[0] = servo 1 (base)     — ψ = atan2(Y, X)
//   out[1] = servo 2 (shoulder) — β - π/2  (β = phi + γ)
//   out[2] = servo 3 (elbow)    — α - π/2
//   out[3] = servo 4 (wrist)    — η - δ
//   out[4] = servo 5            — raw[4] direct
//   out[5] = servo 6 (gripper)  — raw[5] direct
//
// Input poti semantics:
//   raw[0]: X (mm)               raw[3]: η Tool-Pitch (rad)
//   raw[1]: Y (mm)               raw[4]: Wrist rotate
//   raw[2]: Z (mm)               raw[5]: Gripper

void RobotLink::_applyCartesianIK(const uint16_t raw[6], uint16_t out[6]) const {
    const float a  = _kin.L1;
    const float b  = _kin.L2;
    const float L3 = _kin.L3;

    // ── Map poti values to physical quantities ───────────────────────────────
    float X   = _kin.xMin     + (raw[0] / 4095.0f) * (_kin.xMax     - _kin.xMin);
    float Y   = _kin.yMin     + (raw[1] / 4095.0f) * (_kin.yMax     - _kin.yMin);
    float Z   = _kin.zMin     + (raw[2] / 4095.0f) * (_kin.zMax     - _kin.zMin);
    float eta = _kin.pitchMin + (raw[3] / 4095.0f) * (_kin.pitchMax - _kin.pitchMin);

    // ── Base rotation + projection into arm plane ───────────────────────────
    // Add the runtime yaw offset → rotates the whole cartesian frame about Z.
    // Only the base angle changes; the horizontal radius r is rotation-invariant.
    float psi = atan2f(Y, X) + _params[PARAM_YAW_OFFSET];
    float r   = sqrtf(X*X + Y*Y);

    // ── Wrist position (back-step L3 along tool-pitch direction) ────────────
    float Wr = r - L3 * cosf(eta);
    float Wz = Z - L3 * sinf(eta);

    // ── 2-link reach check ───────────────────────────────────────────────────
    float l     = sqrtf(Wr*Wr + Wz*Wz);
    const float reachLo = fabsf(a - b);   // closest reachable shoulder→wrist distance
    const float reachHi = a + b;          // farthest reachable distance
    bool clamped = (l > reachHi) || (l < reachLo);

    // CART_LIMIT_PROJECT: pull the wrist target onto the reachable annulus along
    // the same direction, so the pose stays geometrically consistent.
    if (clamped && _kin.cartLimitMode == CART_LIMIT_PROJECT && l > 1e-3f) {
        const float eps = 0.5f;  // stay just inside the boundary to avoid singularities
        float target = (l > reachHi) ? (reachHi - eps) : (reachLo + eps);
        float s = target / l;
        Wr *= s;
        Wz *= s;
        l   = target;
    }

    float gamma = atan2f(Wz, Wr);

    float cosA = (a*a + b*b - l*l) / (2.0f * a * b);
    cosA = constrain(cosA, -1.0f, 1.0f);
    float alpha = acosf(cosA);

    float cosP = (a*a + l*l - b*b) / (2.0f * a * l);
    cosP = constrain(cosP, -1.0f, 1.0f);
    float phi = acosf(cosP);

    float beta  = phi + gamma;
    float delta = beta + alpha - (float)M_PI;
    float theta4_rel = eta - delta;

    // ── Servo joint angles with mechanical center offsets ───────────────────
    float thetaBase     = psi;
    float thetaShoulder = beta  - (float)M_PI / 2.0f;
    float thetaElbow    = alpha - (float)M_PI / 2.0f;
    float thetaWrist    = theta4_rel;

    Serial.printf("[Cart] X=%.0f Y=%.0f Z=%.0f η=%.1f° → ψ=%.1f° β=%.1f° α=%.1f° θ4=%.1f°\n",
                  X, Y, Z, eta * 180.0f / (float)M_PI,
                  psi   * 180.0f / (float)M_PI,
                  beta  * 180.0f / (float)M_PI,
                  alpha * 180.0f / (float)M_PI,
                  theta4_rel * 180.0f / (float)M_PI);

    // Maps a joint angle to a normalised servo value, clamping to hardware
    // limits. Sets `clamped` when the requested angle lay outside the limits.
    auto angleToNorm = [&](int jointIdx, float theta) -> uint16_t {
        float pos_f = _kin.jointZeroPos[jointIdx] + theta * _kin.jointScale[jointIdx];
        int   pos   = (int)pos_f;
        int   lo    = _kin.limits[jointIdx].minPos;
        int   hi    = _kin.limits[jointIdx].maxPos;
        if (pos < lo || pos > hi) clamped = true;
        pos = constrain(pos, lo, hi);
        int range = hi - lo;
        if (range <= 0) return 2047;
        int norm = (int)(((float)(pos - lo) / (float)range) * 4095.0f);
        return (uint16_t)constrain(norm, 0, 4095);
    };

    out[0] = angleToNorm(0, thetaBase);
    out[1] = angleToNorm(1, thetaShoulder);
    out[2] = angleToNorm(2, thetaElbow);
    out[3] = angleToNorm(3, thetaWrist);
    out[4] = raw[4];
    out[5] = raw[5];

    // ── Limit behaviour ──────────────────────────────────────────────────────
    if (_kin.cartLimitMode == CART_LIMIT_HOLD && clamped && _lastCartValid) {
        // Target unreachable → freeze the whole arm at the last valid pose.
        memcpy(out, _lastCartOut, sizeof(_lastCartOut));
        return;
    }
    // CLAMP / PROJECT (and HOLD while still reachable): remember this pose as the
    // last valid one when nothing was clamped.
    if (!clamped) {
        memcpy(_lastCartOut, out, sizeof(_lastCartOut));
        _lastCartValid = true;
    }
}

// ─── update() ─────────────────────────────────────────────────────────────────

void RobotLink::update() {
    if (_flashUntilMs && millis() >= _flashUntilMs) {
        setModeLED(_mode);
        _flashUntilMs = 0;
    }
    _checkModeButton();

    // Drain any pending runtime parameter (set + persist, off the ISR path).
    if (_hasParam) {
        _hasParam = false;
        _applyParam(_rxParamID, _rxParamValue);
    }

    if (!_hasFrame) return;
    _hasFrame = false;
    if (!_callback) return;

    uint16_t raw[ROBOTLINK_NUM_AXES];
    robotFrameUnpack(_rxBuf, raw);

    if (_rxCoordMode == COORD_CYLINDRICAL && _hasKinematics) {
        uint16_t out[ROBOTLINK_NUM_AXES];
        _applyCylindricalIK(raw, out);
        _callback(_rxBuf.receiverID & 0x07, out);
    } else if (_rxCoordMode == COORD_CARTESIAN && _hasKinematics) {
        uint16_t out[ROBOTLINK_NUM_AXES];
        _applyCartesianIK(raw, out);
        _callback(_rxBuf.receiverID & 0x07, out);
    } else {
        _callback(_rxBuf.receiverID & 0x07, raw);
    }
}
