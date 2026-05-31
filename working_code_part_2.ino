/*
 ╔══════════════════════════════════════════════════════════════╗
 ║   HEX-X1+  ·  Hexapod Alien Controller  ·  ESP32            ║
 ║   Hardware : ESP32 + 2×PCA9685 (0x40=Left, 0x41=Right)      ║
 ╠══════════════════════════════════════════════════════════════╣
 ║   BASE = your known-good HEX-X1 sketch (UNCHANGED gaits).    ║
 ║   ADDED two features, nothing else touched:                  ║
 ║                                                              ║
 ║   1) FAULT SIMULATION                                        ║
 ║      Tap any leg button -> that leg folds its femur fully    ║
 ║      UP so the foot leaves the ground and contributes        ║
 ║      nothing to support or motion. Tap again -> it returns.  ║
 ║      Works on all 6 legs, individually.                      ║
 ║                                                              ║
 ║   2) AUTO GAIT ADJUST                                        ║
 ║      With a leg disabled, tap "AUTO ADJUST" ->               ║
 ║        a) the disabled leg's neighbours shift their coxas    ║
 ║           toward the gap (COG re-centre, kills the tilt).    ║
 ║        b) walking switches to a 5-LEG WAVE gait that skips   ║
 ║           the disabled leg. F/B/L/R all use it.              ║
 ║      Tap AUTO ADJUST again (or re-enable the leg) -> back    ║
 ║      to the normal 6-leg tripod.                             ║
 ╠══════════════════════════════════════════════════════════════╣
 ║  Serial / HTTP commands:                                     ║
 ║   W = Wake   F = Fwd  B = Back  L = Left  R = Right  S = Stop ║
 ║   D = Dance  V = Wave   1/2/3 = Speed   + / - = Stride        ║
 ║   a c d e g h = toggle legs L1 L2 L3 R1 R2 R3                 ║
 ║   k = toggle AUTO GAIT ADJUST                                ║
 ╚══════════════════════════════════════════════════════════════╝
*/

#include <Wire.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ══════════════════════════ Network ══════════════════════════════
const char* AP_SSID = "HEX-X1";
const char* AP_PASS = "hexapod123";   // ≥ 8 characters
const char* UI_PASS = "alien";         // Browser login password

const byte  DNS_PORT = 53;
IPAddress   apIP(192, 168, 4, 1);
DNSServer   dnsServer;
WebServer   httpServer(80);

// ══════════════════════════ PCA9685 ══════════════════════════════
Adafruit_PWMServoDriver pcaL(0x40);   // Left  : L1–L9
Adafruit_PWMServoDriver pcaR(0x41);   // Right : R1–R9
static const float PWM_FREQ = 50.0f;

// ══════════════════════ Servo Calibration ════════════════════════
struct ServoCal { uint16_t usP45, usN45; uint8_t brd, ch; };
const ServoCal SVO[18] = {
  // ── Left board 0x40 ──  +45µs   -45µs   brd ch
  {  985, 2010,  0,  0 },  //  0  FL-Coxa
  { 1117, 1989,  0,  1 },  //  1  FL-Femur
  { 1018, 2037,  0,  2 },  //  2  FL-Tibia
  {  973, 2002,  0,  8 },  //  3  ML-Coxa
  { 1007, 1925,  0,  9 },  //  4  ML-Femur
  {  932, 1947,  0,  10 },  //  5  ML-Tibia
  { 1016, 2137,  0,  12 },  //  6  BL-Coxa
  {  956, 1868,  0,  13 },  //  7  BL-Femur
  {  976, 2069,  0,  14 },  //  8  BL-Tibia
  // ── Right board 0x41 ──
  { 1038, 2032,  1,  0 },  //  9  FR-Coxa
  { 1043, 1962,  1,  1 },  // 10  FR-Femur
  {  918, 1962,  1,  2 },  // 11  FR-Tibia
  {  956, 1983,  1,  8 },  // 12  MR-Coxa
  { 1092, 2014,  1,  9 },  // 13  MR-Femur
  {  938, 1990,  1,  10 },  // 14  MR-Tibia
  {  930, 1998,  1,  12 },  // 15  BR-Coxa
  { 1052, 1973,  1,  13 },  // 16  BR-Femur
  {  995, 1978,  1,  14 }   // 17  BR-Tibia
};

// ══════════════════════════ Level Trim ═══════════════════════════
float trimDeg[18] = { 0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0 };

// ══════════════════════════ Leg Layout ═══════════════════════════
struct Leg { uint8_t cx, fm, tb; const char* name; };
const Leg LEG[6] = {
  { 0, 1, 2, "L1" },  // 0 FL  Front Left
  { 3, 4, 5, "L2" },  // 1 ML  Mid   Left
  { 6, 7, 8, "L3" },  // 2 BL  Back  Left
  { 9,10,11, "R1" },  // 3 FR  Front Right
  {12,13,14, "R2" },  // 4 MR  Mid   Right
  {15,16,17, "R3" }   // 5 BR  Back  Right
};

// ─────────────────── PER-JOINT MIRROR DIRECTIONS ───────────────────
// Right-side legs are mounted mirror-image of the left, so a given angle
// turns the servo the OPPOSITE physical way. This table fixes that per
// joint: { coxaDir, femurDir, tibiaDir } for each leg.
//
//   LEFT legs (0,1,2): all +1 (reference orientation, already working).
//   RIGHT legs (3,4,5): coxa -1 (as before), femur -1 and tibia -1 so
//                       "up/in" commands move them up/in like the left.
//
// HOW TO TUNE: if ONE joint on the right still moves the wrong way, flip
// just that one number from -1 to +1 (or vice-versa). Nothing else.
//   col 0 = coxa, col 1 = femur, col 2 = tibia
const int8_t JDIR[6][3] = {
  /* L1 */ { +1, +1, +1 },
  /* L2 */ { +1, +1, +1 },
  /* L3 */ { +1, +1, +1 },
  /* R1 */ { -1, -1, -1 },
  /* R2 */ { -1, -1, -1 },
  /* R3 */ { -1, -1, -1 }
};

// Tripod groups — diagonal triads that lift together
// A: FL(0) + MR(4) + BL(2)    B: FR(3) + ML(1) + BR(5)
const uint8_t GA[3] = { 0, 4, 2 };
const uint8_t GB[3] = { 3, 1, 5 };

// Wave phase offsets per leg [FL, ML, BL, FR, MR, BR] in radians
const float WPH[6] = { 0.0f, 2.094f, 4.189f, 3.1416f, 5.236f, 1.047f };

// Wave-gait visiting order (alternating sides for stability): L1 R1 L2 R2 L3 R3
const uint8_t WAVE_ORDER[6] = { 0, 3, 1, 4, 2, 5 };

// ═══════════════════════ COG-Shift Table ═════════════════════════
// When a leg is OUT, shift the other legs' coxas (body-frame degrees,
// + = forward) so the support polygon re-centres and the body de-tilts.
// Row index = the DISABLED leg. Tune on the bench — starting values.
//          L1     L2     L3     R1     R2     R3
const float COG[6][6] = {
  /* L1 */ {  0.0f, +12.0f, +5.0f, +10.0f, +6.0f,  0.0f },
  /* L2 */ { +7.0f,  0.0f, +7.0f,  +6.0f, +9.0f, +6.0f },
  /* L3 */ { +5.0f, +12.0f,  0.0f,   0.0f, +6.0f, +10.0f },
  /* R1 */ { +10.0f, +6.0f,  0.0f,   0.0f, +12.0f, +5.0f },
  /* R2 */ { +6.0f, +9.0f, +6.0f,  +7.0f,  0.0f, +7.0f },
  /* R3 */ {  0.0f, +6.0f, +10.0f, +5.0f, +12.0f,  0.0f }
};

// ═══════════════════════ Gait Parameters ═════════════════════════
float    SWING    = 18.0f;   // Coxa ± sweep angle (degrees). Stride size.
float    LIFT     = 14.0f;   // Femur lift amplitude (degrees). Leg clearance.
                             //   If legs go DOWN instead of up → flip to -14.0f
float    FM_STAND =  0.0f;   // Femur neutral angle when standing
float    TB_STAND =  0.0f;   // Tibia neutral angle when standing
uint16_t STEP_MS  = 500;     // Half-cycle duration (ms). Lower = faster.
const uint8_t ISTEPS = 50;   // Interpolation sub-steps (smoothness)

// If robot walks backward when 'F' is pressed, flip this to +1
const int8_t DIR_SIGN = -1;

// ─────────────────── DIRECTION CORRECTION ──────────────────────────
// If your movements feel "shuffled" after wiring, fix them here. Each is
// just +1 or -1. Flip ONE at a time and re-test:
//   FWD_SIGN  : if FORWARD button goes backward (and back goes fwd) -> flip
//   TURN_SIGN : if LEFT button turns right (and right turns left)   -> flip
const int8_t FWD_SIGN  = +1;   // forward/back direction
const int8_t TURN_SIGN = +1;   // left/right rotation direction

// 5-leg wave-gait params
float    SWING5     = 16.0f;   // forward reach of the single swinging leg
uint16_t WAVE_STEP_MS = 320;   // per single-leg swing
const int8_t WAVE_DIR = -1;    // flip if 5-leg "forward" walks backward

// Folded-leg pose (foot fully clears the ground). Mirrored automatically
// per side by JDIR, so both left and right fold UP/IN identically.
// Made deliberately large so the motion is obvious; reduce if a leg hits
// the body when it folds.
const float TUCK_FM = -42.0f;  // femur swings up hard
const float TUCK_TB = -40.0f;  // tibia curls up/in (now clearly visible)

// ═══════════════════════════ State ═══════════════════════════════
enum Mode : uint8_t { IDLE, WALK_F, WALK_B, TURN_L, TURN_R, DANCE, WAVE, WAKING };
Mode     curMode  = IDLE;
bool     isActive = false;
bool     wakeFlag = false;   // Queued wake request
bool     danceFlag = false;  // Queued dance request
float    cxPos[6] = {};      // Live coxa positions per leg (logical degrees)
String   sysStatus = "STANDBY";

// ── Fault / auto-adjust state ──
enum LegState : uint8_t { LEG_ACTIVE, LEG_DISABLED };
LegState legState[6]   = { LEG_ACTIVE,LEG_ACTIVE,LEG_ACTIVE,LEG_ACTIVE,LEG_ACTIVE,LEG_ACTIVE };
float    cogOffset[6]  = {0};      // applied coxa offset per leg (COG shift)
bool     autoAdjust    = false;    // is the 5-leg balanced mode engaged?
int8_t   pendingLeg    = -1;       // deferred leg toggle (-1 = none)
bool     pendingAuto   = false;    // deferred auto-adjust toggle

// Auth
IPAddress authIP(0, 0, 0, 0);
uint32_t  authExp = 0;

// ══════════════════════ Forward Declarations ══════════════════════
void svcAll();
void svcDelay(uint32_t ms);
void setStatus(const String& s);
uint8_t countActive();

// ══════════════════════ Low-Level Servo ══════════════════════════
uint16_t usTicks(uint16_t us) {
  return (uint16_t)constrain((us * PWM_FREQ * 4096.0f) / 1e6f + 0.5f, 0, 4095);
}

uint16_t aglUS(uint8_t idx, float deg) {
  deg = constrain(deg + trimDeg[idx], -45.0f, 45.0f);
  const ServoCal& s = SVO[idx];
  return (uint16_t)(s.usN45 + (s.usP45 - s.usN45) * ((deg + 45.0f) / 90.0f) + 0.5f);
}

void wServo(uint8_t idx, float deg) {
  uint16_t t = usTicks(aglUS(idx, deg));
  if (SVO[idx].brd == 0) pcaL.setPWM(SVO[idx].ch, 0, t);
  else                    pcaR.setPWM(SVO[idx].ch, 0, t);
}

// Set all 3 joints of one leg. JDIR mirrors each joint for the right side
// so right legs move the same physical way as the left for a given angle.
void setLeg(uint8_t i, float cx, float fm, float tb) {
  wServo(LEG[i].cx, cx * JDIR[i][0]);
  wServo(LEG[i].fm, fm * JDIR[i][1]);
  wServo(LEG[i].tb, tb * JDIR[i][2]);
}

// Gait-level write: adds the COG coxa offset, then stores the PURE coxa
// position (without the offset) so the gait math stays clean.
// SAFETY: a DISABLED leg is never moved by a gait — it stays folded. This
// means even the 6-leg tripod (auto-adjust OFF) won't un-fold a folded leg.
void setLegGait(uint8_t i, float cx, float fm, float tb) {
  if (legState[i] == LEG_DISABLED) {
    setLeg(i, cogOffset[i], TUCK_FM, TUCK_TB);   // hold the fold
    cxPos[i] = 0.0f;
    return;
  }
  setLeg(i, cx + cogOffset[i], fm, tb);
  cxPos[i] = cx;
}

void standAll() {
  for (uint8_t i = 0; i < 6; i++) {
    if (legState[i] == LEG_DISABLED) {       // keep folded legs folded
      setLeg(i, cogOffset[i], TUCK_FM, TUCK_TB);
      cxPos[i] = 0.0f;
    } else {
      setLegGait(i, 0.0f, FM_STAND, TB_STAND);
    }
  }
}

// Hold neutral on every ACTIVE leg; leave disabled legs folded & untouched.
void holdActive() {
  for (uint8_t i = 0; i < 6; i++)
    if (legState[i] == LEG_ACTIVE) setLegGait(i, 0.0f, FM_STAND, TB_STAND);
}

uint8_t countActive() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 6; i++) if (legState[i] == LEG_ACTIVE) n++;
  return n;
}

// ══════════════════════ Service Layer ════════════════════════════
void svcAll() {
  dnsServer.processNextRequest();
  httpServer.handleClient();
}

void svcDelay(uint32_t ms) {
  uint32_t end = millis() + ms;
  while (millis() < end) { svcAll(); delay(1); }
}

void setStatus(const String& s) {
  sysStatus = s;
  Serial.println(s);
}

// ═══════════════════════ Wake Sequence ═══════════════════════════
void doWake() {
  curMode = WAKING; isActive = true;
  setStatus("INITIALIZING SUBSYSTEMS...");

  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, -35.0f, -22.0f);
  svcDelay(600);

  const uint8_t ORD[6] = { 0, 3, 1, 4, 2, 5 };
  for (uint8_t n = 0; n < 6; n++) {
    uint8_t li = ORD[n];
    setLeg(li, 0.0f, -18.0f, -10.0f); svcDelay(65);
    setLeg(li, 0.0f, -35.0f, -22.0f); svcDelay(65);
    setLeg(li, 0.0f, -10.0f,  -3.0f); svcDelay(55);
    setLeg(li, 0.0f, -35.0f, -22.0f); svcDelay(45);
    svcAll();
  }
  svcDelay(250);

  setStatus("MOTOR SYSTEMS ONLINE...");
  for (uint8_t s = 0; s <= 65; s++) {
    float t   = (float)s / 65.0f;
    float fm  = -35.0f + (FM_STAND + 35.0f) * t;
    float tb  = -22.0f + (TB_STAND + 22.0f) * t;
    float sp  = sinf(t * PI) * 22.0f;
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i < 3) ? -sp : sp;
      setLeg(i, cx, fm, tb);
    }
    svcDelay(12);
  }

  setStatus("CALIBRATING BALANCE...");
  for (uint8_t k = 0; k < 5; k++) {
    for (uint8_t i = 0; i < 6; i++) setLeg(i,  8.0f, FM_STAND, TB_STAND);
    svcDelay(55);
    for (uint8_t i = 0; i < 6; i++) setLeg(i, -8.0f, FM_STAND, TB_STAND);
    svcDelay(55);
  }

  setLeg(0, 0.0f, FM_STAND - 15.0f, TB_STAND);
  setLeg(3, 0.0f, FM_STAND - 15.0f, TB_STAND);
  svcDelay(220);
  setLeg(0, 0.0f, FM_STAND + 10.0f, TB_STAND);
  setLeg(3, 0.0f, FM_STAND + 10.0f, TB_STAND);
  svcDelay(140);

  standAll();
  svcDelay(350);

  setStatus("HEX-X1 ONLINE — AWAITING COMMAND");
  isActive = false;
  curMode  = IDLE;
}

// ═══════════════════════ Tripod Walk (6-leg, ORIGINAL) ════════════
void tripodStep(uint8_t swingId, int8_t dir) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;

  float se  =  (float)dir * SWING;
  float ste = -(float)dir * SWING;

  float ss[3], ts[3];
  for (uint8_t i = 0; i < 3; i++) { ss[i] = cxPos[sg[i]]; ts[i] = cxPos[st[i]]; }

  uint16_t sd = max((uint16_t)1, (uint16_t)(STEP_MS / ISTEPS));

  for (uint8_t s = 0; s <= ISTEPS; s++) {
    svcAll();
    if (!isActive) return;

    float t  = (float)s / (float)ISTEPS;
    float lf = sinf(t * PI);
    float fs = FM_STAND - (LIFT * lf);

    for (uint8_t i = 0; i < 3; i++)
      setLegGait(sg[i], ss[i] + (se  - ss[i]) * t, fs, TB_STAND);
    for (uint8_t i = 0; i < 3; i++)
      setLegGait(st[i], ts[i] + (ste - ts[i]) * t, FM_STAND, TB_STAND);
    svcDelay(sd);
  }
}

void walkCycle(int8_t dir) {
  tripodStep(0, dir);
  if (!isActive) return;
  tripodStep(1, dir);
}

// ═══════════════════════ Tripod Turn (6-leg, ORIGINAL) ════════════
void tripodTurnStep(uint8_t swingId, int8_t td) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;

  float TS = SWING * 0.75f;

  float swTgt[3], stTgt[3];
  for (uint8_t i = 0; i < 3; i++) {
    bool sL = (sg[i] < 3);
    bool gL = (st[i] < 3);
    swTgt[i] = sL ? -(float)td * TS :  (float)td * TS;
    stTgt[i] = gL ?  (float)td * TS : -(float)td * TS;
  }

  float ss[3], ts[3];
  for (uint8_t i = 0; i < 3; i++) { ss[i] = cxPos[sg[i]]; ts[i] = cxPos[st[i]]; }

  uint16_t sd = max((uint16_t)1, (uint16_t)(STEP_MS / ISTEPS));

  for (uint8_t s = 0; s <= ISTEPS; s++) {
    svcAll();
    if (!isActive) return;

    float t  = (float)s / (float)ISTEPS;
    float lf = sinf(t * PI);
    float fs = FM_STAND - (LIFT * lf);

    for (uint8_t i = 0; i < 3; i++)
      setLegGait(sg[i], ss[i] + (swTgt[i] - ss[i]) * t, fs, TB_STAND);
    for (uint8_t i = 0; i < 3; i++)
      setLegGait(st[i], ts[i] + (stTgt[i] - ts[i]) * t, FM_STAND, TB_STAND);
    svcDelay(sd);
  }
}

void turnCycle(int8_t td) {
  tripodTurnStep(0, td);
  if (!isActive) return;
  tripodTurnStep(1, td);
}

// ═══════════════════════ 5-LEG WAVE GAIT ══════════════════════════
// One ACTIVE leg swings at a time -> always ≥4 feet down (statically
// stable). Grounded active legs creep their coxa backward to push the
// body forward. The disabled leg is skipped entirely.
// turn != 0 biases swing/creep for in-place rotation.
void waveCycle5(int8_t dir, int8_t turn) {
  uint8_t nAct = countActive();
  if (nAct < 4) return;                       // safety: too few legs

  float A    = SWING5;
  float fwd  = A * 0.5f;                       // forward-most coxa
  float back = A / (float)nAct;                // per-event backward creep
  uint16_t sd = max((uint16_t)1, (uint16_t)(WAVE_STEP_MS / ISTEPS));

  for (uint8_t o = 0; o < 6; o++) {
    uint8_t sw = WAVE_ORDER[o];
    if (legState[sw] != LEG_ACTIVE) continue;  // skip disabled leg

    float start[6];
    for (uint8_t i = 0; i < 6; i++) start[i] = cxPos[i];

    float swTarget;
    if (turn != 0) {
      bool isL = (sw < 3);
      swTarget = (isL ? -(float)turn : (float)turn) * fwd;
    } else {
      swTarget = (float)dir * (float)WAVE_DIR * fwd;
    }

    for (uint8_t s = 0; s <= ISTEPS; s++) {
      svcAll();
      if (!isActive) return;
      float t = (float)s / (float)ISTEPS;
      for (uint8_t i = 0; i < 6; i++) {
        if (legState[i] != LEG_ACTIVE) continue;
        if (i == sw) {
          float lf = sinf(t * PI);
          float fs = FM_STAND - (LIFT * lf);    // lift & swing forward
          setLegGait(i, start[i] + (swTarget - start[i]) * t, fs, TB_STAND);
        } else {
          float stTarget;
          if (turn != 0) {
            bool isL = (i < 3);
            stTarget = start[i] + (isL ? (float)turn : -(float)turn) * back;
          } else {
            stTarget = start[i] - (float)dir * (float)WAVE_DIR * back;
          }
          stTarget = constrain(stTarget, -A, A);
          setLegGait(i, start[i] + (stTarget - start[i]) * t, FM_STAND, TB_STAND);
        }
      }
      svcDelay(sd);
    }
  }
}

// ═══════════════════════ Wave / Flow Mode (ORIGINAL) ══════════════
void runWave() {
  float t = 0.0f;
  const float AMP = 11.0f;
  const float SPD = 0.075f;

  while (isActive && curMode == WAVE) {
    t += SPD;
    if (t > 2.0f * PI) t -= 2.0f * PI;

    for (uint8_t i = 0; i < 6; i++) {
      if (legState[i] != LEG_ACTIVE) continue;   // skip folded legs
      float fm = FM_STAND + AMP * sinf(t + WPH[i]);
      setLegGait(i, 0.0f, fm, TB_STAND);
    }
    svcAll();
    delay(22);
  }
}

// ═══════════════════════ Dance Mode (ORIGINAL) ════════════════════
void doDance() {
  setStatus("DANCE SEQUENCE ACTIVE");
  auto ok = [&]() { return isActive && curMode == DANCE; };

  for (uint8_t r = 0; r < 4 && ok(); r++) {
    float s = (r % 2 == 0) ? 15.0f : -15.0f;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, (i < 3) ? s : -s, FM_STAND, TB_STAND);
    svcDelay(220);
  }
  standAll(); if (!ok()) return;

  const uint8_t SEQ[6] = { 0, 3, 1, 4, 2, 5 };
  for (uint8_t n = 0; n < 6 && ok(); n++) {
    uint8_t li = SEQ[n];
    setLeg(li, 0.0f, FM_STAND - LIFT - 6.0f, TB_STAND); svcDelay(130);
    setLeg(li, 0.0f, FM_STAND,                TB_STAND); svcDelay(100);
  }
  if (!ok()) return;

  for (uint8_t k = 0; k < 8 && ok(); k++) {
    float fm = (k % 2 == 0) ? FM_STAND - 13.0f : FM_STAND;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, TB_STAND);
    svcDelay(150);
  }
  if (!ok()) return;

  float wt = 0.0f;
  for (uint16_t f = 0; f < 90 && ok(); f++) {
    wt += 0.12f;
    for (uint8_t i = 0; i < 6; i++) {
      float fm = FM_STAND + LIFT * sinf(wt + WPH[i]);
      setLeg(i, 0.0f, fm, TB_STAND);
    }
    svcDelay(20);
  }
  if (!ok()) return;

  for (uint8_t r = 0; r < 2 && ok(); r++) turnCycle(+1);
  if (!ok()) return;

  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 20.0f, TB_STAND);
  svcDelay(350);
  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND +  9.0f, TB_STAND);
  svcDelay(130);
  standAll();

  setStatus("DANCE COMPLETE — STANDING BY");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ═══════════════════════ FAULT: fold / unfold a leg ═══════════════
void foldLeg(uint8_t i) {
  // smooth move from current pose to fully-folded (foot off ground)
  float cs = cxPos[i] + cogOffset[i], fs = FM_STAND, ts = TB_STAND;
  const uint8_t STEPS = 28;
  for (uint8_t s = 0; s <= STEPS; s++) {
    float t  = (float)s / STEPS;
    float et = 0.5f - 0.5f * cosf(t * PI);
    setLeg(i, cs + (cogOffset[i] - cs) * et,
              fs + (TUCK_FM     - fs) * et,
              ts + (TUCK_TB     - ts) * et);
    svcDelay(14);
  }
  cxPos[i] = 0.0f;
}

void unfoldLeg(uint8_t i) {
  float cs = cogOffset[i], fs = TUCK_FM, ts = TUCK_TB;
  const uint8_t STEPS = 28;
  for (uint8_t s = 0; s <= STEPS; s++) {
    float t  = (float)s / STEPS;
    float et = 0.5f - 0.5f * cosf(t * PI);
    setLeg(i, cs + (cogOffset[i] - cs) * et,
              fs + (FM_STAND     - fs) * et,
              ts + (TB_STAND     - ts) * et);
    svcDelay(14);
  }
  setLegGait(i, 0.0f, FM_STAND, TB_STAND);
}

// ═══════════════════════ AUTO ADJUST: COG ramp ════════════════════
// Ramp every leg's coxa offset toward the COG target for the disabled
// leg (engage), or back to zero (disengage). Folded legs are held folded.
void rampCog(uint8_t disLeg, bool engage) {
  float start[6], tgt[6];
  for (uint8_t i = 0; i < 6; i++) {
    start[i] = cogOffset[i];
    tgt[i]   = engage ? COG[disLeg][i] : 0.0f;
  }
  const uint8_t N = 45;
  for (uint8_t s = 0; s <= N; s++) {
    float t  = (float)s / N;
    float et = 0.5f - 0.5f * cosf(t * PI);
    for (uint8_t i = 0; i < 6; i++) cogOffset[i] = start[i] + (tgt[i] - start[i]) * et;
    // re-assert poses so the new offset is visible immediately
    for (uint8_t i = 0; i < 6; i++) {
      if (legState[i] == LEG_DISABLED) setLeg(i, cogOffset[i], TUCK_FM, TUCK_TB);
      else                             setLegGait(i, 0.0f, FM_STAND, TB_STAND);
    }
    svcDelay(22);
  }
}

// First disabled leg index, or -1 if none.
int8_t firstDisabled() {
  for (uint8_t i = 0; i < 6; i++) if (legState[i] == LEG_DISABLED) return i;
  return -1;
}

// ═══════════════════════ Deferred handlers (run in loop) ══════════
void handleLegToggle() {
  uint8_t i = (uint8_t)pendingLeg; pendingLeg = -1;
  isActive = false;                 // stop any motion before re-posing

  if (legState[i] == LEG_ACTIVE) {
    legState[i] = LEG_DISABLED;
    setStatus(String("LEG ") + LEG[i].name + " DISABLED (folded)");
    foldLeg(i);
    // If auto-adjust was already on for a different leg, re-balance to this one.
    if (autoAdjust) {
      int8_t d = firstDisabled();
      if (d >= 0) rampCog((uint8_t)d, true);
    }
  } else {
    setStatus(String("LEG ") + LEG[i].name + " RE-ENABLED");
    // bringing a leg back: if it was the balanced one, undo the COG shift
    if (autoAdjust && firstDisabled() == (int8_t)i) {
      rampCog(i, false);
      autoAdjust = false;
    }
    legState[i] = LEG_ACTIVE;
    unfoldLeg(i);
    standAll();
  }
  curMode = IDLE;
}

void handleAutoToggle() {
  pendingAuto = false;
  isActive = false;

  int8_t d = firstDisabled();
  if (!autoAdjust) {
    if (d < 0) { setStatus("AUTO ADJUST: disable a leg first"); return; }
    setStatus(String("AUTO ADJUST ON — balancing for ") + LEG[d].name);
    rampCog((uint8_t)d, true);          // neighbours shift toward the gap
    autoAdjust = true;
    setStatus(String("5-LEG MODE READY (") + LEG[d].name + " out)");
  } else {
    setStatus("AUTO ADJUST OFF — neighbours returning");
    if (d >= 0) rampCog((uint8_t)d, false);
    else        rampCog(0, false);      // just zero the offsets
    autoAdjust = false;
    standAll();
    setStatus("6-LEG NORMAL");
  }
  curMode = IDLE;
}

// ═══════════════════════ Auth Helpers ════════════════════════════
bool isAuth() {
  if (!httpServer.hasHeader("Cookie")) return false;
  return httpServer.header("Cookie").indexOf("HEX=1") >= 0;
}

bool checkAuth(bool api = false) {
  if (isAuth() || (authExp > 0 && millis() < authExp &&
      httpServer.client().remoteIP() == authIP)) return true;
  if (api) { httpServer.send(401, "text/plain", "401 Unauthorized"); return false; }
  httpServer.sendHeader("Location", "http://192.168.4.1/login", true);
  httpServer.send(302, "text/plain", "");
  return false;
}

// ═══════════════════════ Command Processor ═══════════════════════
void processCmd(char cmd) {
  switch (cmd) {
    case 'W': case 'w':
      if (curMode != WAKING) { isActive = false; wakeFlag = true; setStatus("WAKE: QUEUED"); }
      break;

    case 'F': case 'f':
      wakeFlag = danceFlag = false; curMode = WALK_F; isActive = true;
      setStatus(autoAdjust ? "WALK FWD (5-leg)" : "WALK: FORWARD"); break;
    case 'B': case 'b':
      wakeFlag = danceFlag = false; curMode = WALK_B; isActive = true;
      setStatus(autoAdjust ? "WALK BACK (5-leg)" : "WALK: BACKWARD"); break;
    case 'L': case 'l':
      wakeFlag = danceFlag = false; curMode = TURN_L; isActive = true;
      setStatus(autoAdjust ? "TURN LEFT (5-leg)" : "TURN: LEFT"); break;
    case 'R': case 'r':
      wakeFlag = danceFlag = false; curMode = TURN_R; isActive = true;
      setStatus(autoAdjust ? "TURN RIGHT (5-leg)" : "TURN: RIGHT"); break;

    case 'D': case 'd':
      wakeFlag = false; danceFlag = true; curMode = DANCE; isActive = true;
      setStatus("DANCE: INITIATED"); break;
    case 'V': case 'v':
      wakeFlag = danceFlag = false; curMode = WAVE; isActive = true;
      setStatus("WAVE: FLOW MODE"); break;

    case 'S': case 's':
      isActive = wakeFlag = danceFlag = false; curMode = IDLE;
      standAll(); setStatus("HALT — STANDING BY"); break;

    case '1': STEP_MS = 720; WAVE_STEP_MS = 460; setStatus("SPEED: SLOW");   break;
    case '2': STEP_MS = 500; WAVE_STEP_MS = 320; setStatus("SPEED: NORMAL"); break;
    case '3': STEP_MS = 300; WAVE_STEP_MS = 220; setStatus("SPEED: FAST");   break;

    case '+': SWING = min(SWING + 2.0f, 38.0f); SWING5 = min(SWING5 + 1.5f, 24.0f);
              setStatus("STRIDE: " + String((int)SWING) + " DEG"); break;
    case '-': SWING = max(SWING - 2.0f, 5.0f);  SWING5 = max(SWING5 - 1.5f, 8.0f);
              setStatus("STRIDE: " + String((int)SWING) + " DEG"); break;

    // ── Fault toggles (deferred to loop) ──
    // Leg toggles. L3 is sent as the 2-char token "dd" from the UI and
    // routed in the /cmd handler, so there is NO 'd' case here (that key
    // belongs to Dance above). Serial users: send the UI, or use 'e/g/h'
    // and 'a/c' directly; for L3 over raw serial, type the letter 'p'.
    case 'a': case 'A': pendingLeg = 0; break;   // L1
    case 'c': case 'C': pendingLeg = 1; break;   // L2
    case 'p': case 'P': pendingLeg = 2; break;   // L3 (raw-serial alias)
    case 'e': case 'E': pendingLeg = 3; break;   // R1
    case 'g': case 'G': pendingLeg = 4; break;   // R2
    case 'h': case 'H': pendingLeg = 5; break;   // R3

    // ── Auto gait adjust toggle (deferred) ──
    case 'k': case 'K': pendingAuto = true; break;

    case '\n': case '\r': case ' ': break;
    default: break;
  }
}
void handleSerial() { while (Serial.available()) processCmd((char)Serial.read()); }

// ═══════════════════════ HTML — Login Page ═══════════════════════
const char LOGIN_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>HEX-X1 ACCESS</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 60% at 20% 0%,#001825,transparent 55%),#06090f;
 display:grid;place-items:center;padding:20px;font-family:'Courier New',Courier,monospace;color:#b0ffd0}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:5;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.06) 3px,rgba(0,0,0,0.06) 4px)}
.card{width:min(370px,100%);background:rgba(0,0,0,0.65);border:1px solid rgba(0,255,136,0.38);
 border-radius:3px;padding:22px;position:relative;
 box-shadow:0 0 40px rgba(0,255,136,0.07),inset 0 1px 0 rgba(0,255,136,0.13)}
.card::before,.card::after{content:'';position:absolute;width:14px;height:14px;border-color:#00ff88;border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
h1{font-size:1.35rem;color:#00ff88;letter-spacing:4px;text-shadow:0 0 14px #00ff88;margin-bottom:5px}
p{color:#3a7a5a;font-size:0.78rem;letter-spacing:1.2px;margin-bottom:18px}
.err{color:#ff7777;background:rgba(255,51,51,0.11);border:1px solid rgba(255,51,51,0.42);
 border-radius:2px;padding:9px 11px;margin-bottom:13px;font-size:0.8rem;letter-spacing:0.5px}
input{width:100%;padding:14px 13px;border:1px solid rgba(0,255,136,0.3);border-radius:2px;
 background:rgba(0,255,136,0.04);color:#b0ffd0;font-family:inherit;font-size:1rem;outline:none;letter-spacing:1.5px}
input:focus{border-color:#00ff88;box-shadow:0 0 10px rgba(0,255,136,0.18)}
button{margin-top:11px;width:100%;padding:14px;border:1px solid rgba(0,255,136,0.5);border-radius:2px;
 background:rgba(0,255,136,0.07);color:#00ff88;font-family:inherit;font-size:0.88rem;font-weight:700;
 letter-spacing:2.5px;cursor:pointer;text-transform:uppercase;transition:all 0.13s}
button:hover{background:rgba(0,255,136,0.17);box-shadow:0 0 14px rgba(0,255,136,0.28)}
.hint{margin-top:12px;text-align:center;font-size:0.68rem;color:#2a5a4a;letter-spacing:1px}
</style></head><body>
<div class="card">
 <h1>HEX-X1</h1>
 <p>HEXAPOD CONTROL UNIT · AUTHENTICATION REQUIRED</p>
 %ERR%
 <form action="/login" method="POST">
  <input type="password" name="pw" placeholder="ACCESS CODE" autocomplete="current-password" autofocus/>
  <button type="submit">⚡ UNLOCK SYSTEMS</button>
 </form>
 <div class="hint">WIFI: HEX-X1 · PASS: hexapod123</div>
</div>
</body></html>
)RAW";

// ═══════════════════════ HTML — Control UI ═══════════════════════
const char CTRL_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>HEX-X1 · CONTROL</title>
<style>
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--y:#ffaa00;--bg:#06090f;--t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;
 background:radial-gradient(ellipse 130% 55% at 12% -5%,#001825,transparent 52%),
            radial-gradient(ellipse 80% 40% at 92% 108%,#001508,transparent 46%),var(--bg);
 color:var(--t);font-family:'Courier New',Courier,monospace;display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(455px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;
 padding:13px;position:relative;overflow:hidden;box-shadow:0 0 35px rgba(0,255,136,0.06),inset 0 1px 0 rgba(0,255,136,0.11)}
.card::before,.card::after{content:'';position:absolute;width:16px;height:16px;border-color:var(--g);border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
.hdr{display:flex;align-items:baseline;gap:8px;padding-bottom:8px;border-bottom:1px solid rgba(0,255,136,0.13);margin-bottom:9px}
.logo{font-size:1.3rem;color:var(--g);letter-spacing:3.5px;text-shadow:0 0 12px var(--g)}
.sub{font-size:0.6rem;color:var(--m);letter-spacing:1.5px;margin-left:auto}
.sts{background:rgba(0,0,0,0.65);border:1px solid rgba(0,255,136,0.17);border-radius:2px;
 padding:6px 10px;font-size:0.74rem;color:var(--g);letter-spacing:1px;margin-bottom:9px;
 min-height:28px;display:flex;align-items:center;gap:7px;overflow:hidden;white-space:nowrap}
.sts::before{content:'▶';animation:bl 1.2s step-end infinite;flex-shrink:0;font-size:0.65rem}
@keyframes bl{0%,100%{opacity:1}50%{opacity:0}}
.pills{display:flex;flex-wrap:wrap;gap:4px;margin-bottom:9px;align-items:center}
.pills span:first-child{font-size:0.6rem;color:var(--m);letter-spacing:1.5px}
.pill{padding:2px 8px;border:1px solid rgba(0,255,136,0.18);border-radius:20px;font-size:0.6rem;color:var(--m);letter-spacing:1px;transition:all 0.2s}
.pill.on{border-color:var(--g);color:var(--g);background:rgba(0,255,136,0.11);box-shadow:0 0 7px rgba(0,255,136,0.28)}
.pill.on5{border-color:var(--y);color:var(--y);background:rgba(255,170,0,0.11);box-shadow:0 0 7px rgba(255,170,0,0.28)}
.movepad{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-bottom:9px}
.mp-blank{visibility:hidden}
.mv{padding:16px 0!important;font-size:1.1rem!important;display:flex;flex-direction:column;align-items:center;gap:3px}
.mv small{font-size:0.55rem;color:var(--m);font-weight:400;letter-spacing:1px}
.mv.on{background:rgba(0,255,136,0.16)!important;border-color:var(--g)!important;color:var(--g)!important;box-shadow:0 0 10px rgba(0,255,136,0.38)}
.mv.on small{color:var(--g)}
.mv-stop{border-color:rgba(255,51,68,0.52)!important;color:var(--r)!important}
.mv-stop:hover{background:rgba(255,51,68,0.1)!important;box-shadow:0 0 9px rgba(255,51,68,0.28)!important}
.mv-stop.on{background:rgba(255,51,68,0.18)!important;border-color:var(--r)!important;color:var(--r)!important}
.lbl{font-size:0.59rem;color:var(--m);letter-spacing:2px;text-transform:uppercase;margin:8px 0 4px;display:flex;align-items:center;gap:6px}
.lbl::after{content:'';flex:1;height:1px;background:rgba(0,255,136,0.1)}
.g1{display:grid;grid-template-columns:1fr;gap:5px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:5px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px}
button{all:unset;display:block;width:100%;text-align:center;padding:11px 6px;border:1px solid rgba(0,255,136,0.26);
 border-radius:2px;font-family:inherit;font-size:0.76rem;font-weight:700;letter-spacing:1.5px;color:var(--t);
 background:rgba(0,0,0,0.55);cursor:pointer;touch-action:manipulation;transition:all 0.12s;text-transform:uppercase}
button:hover{background:rgba(0,255,136,0.09);border-color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.2)}
button:active{transform:scale(0.97)}
.btn-wake{border-color:rgba(0,204,255,0.48);color:var(--g2)}
.btn-wake:hover{background:rgba(0,204,255,0.09);box-shadow:0 0 9px rgba(0,204,255,0.28)}
.btn-stop{border-color:rgba(255,51,68,0.52);color:var(--r)}
.btn-stop:hover{background:rgba(255,51,68,0.1);box-shadow:0 0 9px rgba(255,51,68,0.28)}
.btn-mode.on{background:rgba(0,255,136,0.16);border-color:var(--g);color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.28)}
.spd-btn.on{background:rgba(0,204,255,0.12);border-color:rgba(0,204,255,0.6);color:var(--g2);box-shadow:0 0 7px rgba(0,204,255,0.22)}
/* Fault / leg panel */
.legbox{background:rgba(255,170,0,0.04);border:1px solid rgba(255,170,0,0.28);border-radius:3px;padding:10px;margin:4px 0}
.lgttl{font-size:0.64rem;color:var(--y);letter-spacing:1.5px;text-align:center;margin-bottom:8px}
.lgrid{display:grid;grid-template-columns:1fr 52px 1fr;gap:6px;align-items:center}
.lgrid>.body{display:grid;place-items:center;font-size:0.52rem;color:var(--m);border:1px dashed rgba(0,255,136,0.22);border-radius:50%;height:32px}
.lbtn{padding:10px 4px!important;line-height:1.15;font-size:0.7rem!important}
.lbtn small{display:block;font-size:0.5rem;color:var(--m);font-weight:400;margin-top:2px}
.lbtn.dis{background:rgba(255,51,68,0.18)!important;border-color:rgba(255,51,68,0.7)!important;color:var(--r)!important}
.lbtn.dis small{color:rgba(255,119,119,0.8)}
.lghelp{text-align:center;font-size:0.54rem;color:var(--m);margin-top:7px;line-height:1.5}
.btn-auto{border-color:rgba(255,170,0,0.5);color:var(--y)}
.btn-auto:hover{background:rgba(255,170,0,0.1);box-shadow:0 0 9px rgba(255,170,0,0.28)}
.btn-auto.on{background:rgba(255,170,0,0.18);border-color:var(--y);color:var(--y);box-shadow:0 0 11px rgba(255,170,0,0.4)}
</style></head><body>
<div class="card">
 <div class="hdr">
  <div class="logo">HEX-X1</div>
  <div class="sub">HEXAPOD CONTROL UNIT v2.2</div>
 </div>

 <div class="sts" id="sts">STANDBY</div>

 <div class="pills">
  <span>MODE:</span>
  <span class="pill" id="pIdle">IDLE</span>
  <span class="pill" id="pWalk">WALK</span>
  <span class="pill" id="pTurn">TURN</span>
  <span class="pill" id="pDance">DANCE</span>
  <span class="pill" id="pWave">WAVE</span>
  <span class="pill" id="p5">5-LEG</span>
 </div>

 <!-- MOVEMENT BUTTON PAD -->
 <div class="lbl">movement</div>
 <div class="movepad">
  <div class="mp-blank"></div>
  <button class="mv" id="bF" onclick="move('F','bF')">▲<small>FWD</small></button>
  <div class="mp-blank"></div>
  <button class="mv" id="bL" onclick="move('L','bL')">↺<small>LEFT</small></button>
  <button class="mv mv-stop" id="bS" onclick="move('S','bS')">■<small>STOP</small></button>
  <button class="mv" id="bR" onclick="move('R','bR')">↻<small>RIGHT</small></button>
  <div class="mp-blank"></div>
  <button class="mv" id="bB" onclick="move('B','bB')">▼<small>BACK</small></button>
  <div class="mp-blank"></div>
 </div>

 <!-- FAULT SIMULATION + AUTO ADJUST -->
 <div class="lbl">fault simulation</div>
 <div class="legbox">
  <div class="lgttl">TAP A LEG TO FOLD IT UP (DISABLE)</div>
  <div class="lgrid">
   <button class="lbtn" id="lg0" onclick="tg('a')">L1<small>FRONT-L</small></button>
   <div class="body">FRONT</div>
   <button class="lbtn" id="lg3" onclick="tg('e')">R1<small>FRONT-R</small></button>
   <button class="lbtn" id="lg1" onclick="tg('c')">L2<small>MID-L</small></button>
   <div class="body">HEX</div>
   <button class="lbtn" id="lg4" onclick="tg('g')">R2<small>MID-R</small></button>
   <button class="lbtn" id="lg2" onclick="tg('dd')">L3<small>BACK-L</small></button>
   <div class="body">BACK</div>
   <button class="lbtn" id="lg5" onclick="tg('h')">R3<small>BACK-R</small></button>
  </div>
  <button class="btn-auto" id="btnAuto" style="margin-top:9px" onclick="cmd('k')">◎  AUTO GAIT ADJUST</button>
  <div class="lghelp">GREEN=active · RED=folded · AUTO ADJUST shifts neighbours toward the gap &amp; switches to a 5-leg gait</div>
 </div>

 <div class="lbl">special modes</div>
 <div class="g2">
  <button id="btnD" class="btn-mode" onclick="toggleMode('D','btnD')">◈  Dance</button>
  <button id="btnV" class="btn-mode" onclick="toggleMode('V','btnV')">〜  Wave Flow</button>
 </div>

 <div class="lbl">speed preset</div>
 <div class="g3">
  <button id="spd1" class="spd-btn" onclick="setSpeed('1','spd1')">●  Slow</button>
  <button id="spd2" class="spd-btn on" onclick="setSpeed('2','spd2')">●●  Normal</button>
  <button id="spd3" class="spd-btn" onclick="setSpeed('3','spd3')">●●●  Fast</button>
 </div>

 <div class="lbl">stride size</div>
 <div class="g2">
  <button onclick="cmd('+')">Stride  +</button>
  <button onclick="cmd('-')">Stride  −</button>
 </div>

 <div class="lbl">system</div>
 <div class="g1">
  <button class="btn-wake" onclick="cmd('W')">⚡  WAKE SEQUENCE</button>
  <button class="btn-stop" onclick="cmd('S')">■  EMERGENCY HALT</button>
 </div>
 <div class="g2" style="margin-top:5px">
  <button style="font-size:0.68rem" onclick="poll()">↻  Refresh Status</button>
  <button style="font-size:0.68rem" onclick="location='/logout'">⇠  Logout</button>
 </div>
</div>

<script>
const $ = id => document.getElementById(id);
let activeMode=null, activeSpdBtn='spd2';

function setSts(t){ $('sts').textContent=t; }

const PILL={'F':'pWalk','B':'pWalk','L':'pTurn','R':'pTurn','D':'pDance','V':'pWave','W':'pIdle','S':'pIdle'};
let lastPill='pIdle';
function pillOn(c){ $(lastPill)?.classList.remove('on'); const p=PILL[c.toUpperCase()]||'pIdle'; $(p)?.classList.add('on'); lastPill=p; }

const MVBTN={'F':'bF','B':'bB','L':'bL','R':'bR','S':'bS'};
let lastMv=null;
function moveBtnOn(c){ if(lastMv)$(lastMv)?.classList.remove('on'); lastMv=null; const b=MVBTN[c.toUpperCase()]; if(b){$(b)?.classList.add('on');lastMv=b;} }

async function cmd(c){
  if(['F','B','L','R','S'].includes(c.toUpperCase())){ pillOn(c); }
  try{
    const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'});
    if(r.status===401){location='/';return;}
    setSts(await r.text()); setTimeout(poll,180);
  }catch(e){setSts('ERR: '+e.message);}
}

// Movement button: highlight it, clear any special-mode button, send cmd
function move(c, btnId){ moveBtnOn(c); if(c!=='S')clearModeBtn(); cmd(c); }

function tg(c){ cmd(c); }   // leg toggle

function toggleMode(c, btnId){
  const btn=$(btnId);
  if(activeMode===c){ btn.classList.remove('on'); activeMode=null; cmd('S'); }
  else{ if(activeMode){const prev=activeMode==='D'?'btnD':'btnV';$(prev)?.classList.remove('on');} btn.classList.add('on'); activeMode=c; cmd(c); }
}
function setSpeed(c, btnId){ if(activeSpdBtn)$(activeSpdBtn)?.classList.remove('on'); $(btnId)?.classList.add('on'); activeSpdBtn=btnId; cmd(c); }
function clearModeBtn(){ if(activeMode){$(activeMode==='D'?'btnD':'btnV')?.classList.remove('on');activeMode=null;} }

async function poll(){
  try{
    const r=await fetch('/state',{cache:'no-store'});
    if(r.status===401){location='/';return;}
    const j=await r.json();
    setSts(j.status);
    for(let i=0;i<6;i++){const b=$('lg'+i);if(!b)continue;j.legs[i]===1?b.classList.remove('dis'):b.classList.add('dis');}
    const a=$('btnAuto'); if(j.auto){a.classList.add('on');a.textContent='◎  AUTO ADJUST: ON';}else{a.classList.remove('on');a.textContent='◎  AUTO GAIT ADJUST';}
    const p5=$('p5'); if(j.auto){p5.classList.add('on5');}else{p5.classList.remove('on5');}
  }catch(e){}
}

poll();
setInterval(poll,1500);
</script>
</body></html>
)RAW";

// ══════════════════════ Web Server Routes ════════════════════════
void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS, 6, false, 4);
  dnsServer.start(DNS_PORT, "*", apIP);

  const char* hdrs[] = { "Cookie" };
  httpServer.collectHeaders(hdrs, 1);

  httpServer.on("/", HTTP_GET, []() {
    if (!checkAuth()) return;
    httpServer.send_P(200, "text/html", CTRL_HTML);
  });

  httpServer.on("/login", HTTP_GET, []() {
    if (isAuth()) { httpServer.sendHeader("Location", "/", true); httpServer.send(302, "text/plain", ""); return; }
    String p = FPSTR(LOGIN_HTML); p.replace("%ERR%", ""); httpServer.send(200, "text/html", p);
  });

  httpServer.on("/login", HTTP_POST, []() {
    String pw = httpServer.hasArg("pw") ? httpServer.arg("pw") : "";
    if (pw == UI_PASS) {
      authIP  = httpServer.client().remoteIP();
      authExp = millis() + 24UL * 3600UL * 1000UL;
      httpServer.sendHeader("Set-Cookie", "HEX=1; Max-Age=86400; Path=/", true);
      httpServer.sendHeader("Location", "/", true);
      httpServer.send(302, "text/plain", "");
    } else {
      String p = FPSTR(LOGIN_HTML);
      p.replace("%ERR%", "<div class=\"err\">⚠ INCORRECT ACCESS CODE — TRY AGAIN</div>");
      httpServer.send(200, "text/html", p);
    }
  });

  httpServer.on("/logout", HTTP_GET, []() {
    authIP = IPAddress(0, 0, 0, 0); authExp = 0;
    httpServer.sendHeader("Set-Cookie", "HEX=0; Max-Age=0; Path=/", true);
    httpServer.sendHeader("Location", "/login", true);
    httpServer.send(302, "text/plain", "");
  });

  httpServer.on("/cmd", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    if (!httpServer.hasArg("c") || httpServer.arg("c").isEmpty()) {
      httpServer.send(400, "text/plain", "ERR: missing 'c' param"); return;
    }
    // Accept the 2-char tokens 'dd' (L3) and 'k' plus single chars.
    String c = httpServer.arg("c");
    if (c == "dd") pendingLeg = 2;        // L3 (avoids clash with Dance 'D')
    else processCmd(c[0]);
    httpServer.send(200, "text/plain", sysStatus);
  });

  // JSON state: status + per-leg active flags + auto-adjust flag
  httpServer.on("/state", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    String esc = sysStatus; esc.replace("\"", "'");
    String j; j.reserve(180);
    j  = "{\"status\":\"" + esc + "\",\"legs\":[";
    for (uint8_t i = 0; i < 6; i++) { j += (legState[i] == LEG_ACTIVE) ? "1" : "0"; if (i < 5) j += ","; }
    j += "],\"auto\":"; j += autoAdjust ? "true" : "false";
    j += "}";
    httpServer.send(200, "application/json", j);
  });

  httpServer.on("/status", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    httpServer.send(200, "text/plain", sysStatus);
  });

  auto redir = []() {
    httpServer.sendHeader("Location", "http://192.168.4.1/", true);
    httpServer.send(302, "text/plain", "");
  };
  httpServer.on("/generate_204",        HTTP_GET, redir);
  httpServer.on("/hotspot-detect.html", HTTP_GET, redir);
  httpServer.on("/fwlink",              HTTP_GET, redir);
  httpServer.on("/ncsi.txt",            HTTP_GET, []() { httpServer.send(200, "text/plain", "Microsoft NCSI"); });
  httpServer.onNotFound(redir);

  httpServer.begin();

  Serial.println("══════════════════════════════════");
  Serial.println("  WiFi SSID : " + String(AP_SSID));
  Serial.println("  UI URL    : http://192.168.4.1");
  Serial.println("  UI PASS   : " + String(UI_PASS));
  Serial.println("══════════════════════════════════");
}

// ══════════════════════════ Setup ════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  pcaL.begin(); pcaL.setPWMFreq(PWM_FREQ);
  pcaR.begin(); pcaR.setPWMFreq(PWM_FREQ);
  delay(200);

  setupWeb();

  for (uint8_t s = 0; s <= 45; s++) {
    float t = (float)s / 45.0f;
    float fm = -12.0f + (FM_STAND + 12.0f) * t;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, TB_STAND);
    delay(18);
  }
  standAll();

  setStatus("HEX-X1 READY · SEND 'W' TO WAKE");
  Serial.println("Cmds: W F B L R D V S | 1 2 3 | + - | legs a c dd e g h | k=auto-adjust");
}

// ══════════════════════════ Loop ═════════════════════════════════
void loop() {
  svcAll();
  handleSerial();

  // ── Deferred fault / auto-adjust toggles (run their animations here,
  //    never inside an HTTP handler, so the UI never stalls) ──
  if (pendingLeg >= 0) { handleLegToggle();  return; }
  if (pendingAuto)     { handleAutoToggle(); return; }

  // ── Priority: Wake ──
  if (wakeFlag) { wakeFlag = false; doWake(); return; }

  // ── Priority: Dance ──
  if (curMode == DANCE && isActive && danceFlag) { danceFlag = false; doDance(); return; }

  // ── Motion dispatch ──
  // If AUTO ADJUST is engaged AND a leg is disabled -> 5-leg wave gait.
  // Otherwise -> the original 6-leg tripod (UNCHANGED).
  bool fiveLeg = (autoAdjust && firstDisabled() >= 0);

  switch (curMode) {
    case WALK_F:
      if (isActive) { if (fiveLeg) waveCycle5(+1 * FWD_SIGN, 0); else walkCycle(+1 * FWD_SIGN * DIR_SIGN); }
      break;
    case WALK_B:
      if (isActive) { if (fiveLeg) waveCycle5(-1 * FWD_SIGN, 0); else walkCycle(-1 * FWD_SIGN * DIR_SIGN); }
      break;
    case TURN_L:
      if (isActive) { if (fiveLeg) waveCycle5(0, -1 * TURN_SIGN); else turnCycle(-1 * TURN_SIGN); }
      break;
    case TURN_R:
      if (isActive) { if (fiveLeg) waveCycle5(0, +1 * TURN_SIGN); else turnCycle(+1 * TURN_SIGN); }
      break;
    case WAVE:
      if (isActive) runWave();
      break;
    case IDLE:
    default:
      holdActive();
      svcDelay(25);
      break;
  }
}
