/*
 ╔══════════════════════════════════════════════════════════════╗
 ║   HEX-X1  ·  Hexapod Alien Controller  ·  ESP32             ║
 ║   Hardware : ESP32 + 2×PCA9685 (0x40=Left, 0x41=Right)      ║
 ╠══════════════════════════════════════════════════════════════╣
 ║  Serial / HTTP commands:                                     ║
 ║   W = Wake sequence   F = Forward    B = Backward            ║
 ║   L = Turn Left       R = Turn Right  S = Stop               ║
 ║   D = Dance           V = Wave/Flow                          ║
 ║   1 = Slow  2 = Normal  3 = Fast   + / - = Stride            ║
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
struct Leg { uint8_t cx, fm, tb; int8_t dir; };
// dir: +1 = left side, -1 = right side (mirrors coxa direction)
const Leg LEG[6] = {
  { 0, 1, 2, +1 },  // 0 FL  Front Left
  { 3, 4, 5, +1 },  // 1 ML  Mid   Left
  { 6, 7, 8, +1 },  // 2 BL  Back  Left
  { 9,10,11, -1 },  // 3 FR  Front Right
  {12,13,14, -1 },  // 4 MR  Mid   Right
  {15,16,17, -1 }   // 5 BR  Back  Right
};

// Tripod groups — diagonal triads that lift together
// A: FL(0) + MR(4) + BL(2)    B: FR(3) + ML(1) + BR(5)
const uint8_t GA[3] = { 0, 4, 2 };
const uint8_t GB[3] = { 3, 1, 5 };

// Wave phase offsets per leg [FL, ML, BL, FR, MR, BR] in radians
// Creates a smooth ripple from front-left → mid → back, crossing sides
const float WPH[6] = { 0.0f, 2.094f, 4.189f, 3.1416f, 5.236f, 1.047f };

// ═══════════════════════ Gait Parameters ═════════════════════════
// ── Tune these to fit your robot ──────────────────────────────────
float    SWING    = 18.0f;   // Coxa ± sweep angle (degrees). Stride size.
float    LIFT     = 14.0f;   // Femur lift amplitude (degrees). Leg clearance.
                             //   If legs go DOWN instead of up → flip to -14.0f
float    FM_STAND =  0.0f;   // Femur neutral angle when standing
float    TB_STAND =  0.0f;   // Tibia neutral angle when standing
uint16_t STEP_MS  = 500;     // Half-cycle duration (ms). Lower = faster.
const uint8_t ISTEPS = 50;   // Interpolation sub-steps (smoothness)

// If robot walks backward when 'F' is pressed, flip this to +1
const int8_t DIR_SIGN = -1;

// ═══════════════════════════ State ═══════════════════════════════
enum Mode : uint8_t { IDLE, WALK_F, WALK_B, TURN_L, TURN_R, DANCE, WAVE, WAKING };
Mode     curMode  = IDLE;
bool     isActive = false;
bool     wakeFlag = false;   // Queued wake request
bool     danceFlag = false;  // Queued dance request
float    cxPos[6] = {};      // Live coxa positions per leg (logical degrees)
String   sysStatus = "STANDBY";

// Auth
IPAddress authIP(0, 0, 0, 0);
uint32_t  authExp = 0;

// ══════════════════════ Forward Declarations ══════════════════════
void svcAll();
void svcDelay(uint32_t ms);
void setStatus(const String& s);

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

// Set all 3 joints of one leg. coxaDir mirrors right-side coxas.
void setLeg(uint8_t i, float cx, float fm, float tb) {
  wServo(LEG[i].cx, cx * LEG[i].dir);
  wServo(LEG[i].fm, fm);
  wServo(LEG[i].tb, tb);
}

void standAll() {
  for (uint8_t i = 0; i < 6; i++) {
    setLeg(i, 0.0f, FM_STAND, TB_STAND);
    cxPos[i] = 0.0f;
  }
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
// Full alien "boot-up" animation: collapse → twitch → rise → shake → bow → stand
void doWake() {
  curMode = WAKING; isActive = true;
  setStatus("INITIALIZING SUBSYSTEMS...");

  // ── Phase 1: Collapse all legs into a dead heap ──────────────────
  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, -35.0f, -22.0f);
  svcDelay(600);

  // ── Phase 2: Sequential leg twitches (FL→FR→ML→MR→BL→BR) ────────
  // Each leg jolts twice, like muscle spasms waking up
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

  // ── Phase 3: Dramatic rise with outward flare ────────────────────
  // Body lifts while legs fan outward in an arc, then snap to center
  setStatus("MOTOR SYSTEMS ONLINE...");
  for (uint8_t s = 0; s <= 65; s++) {
    float t   = (float)s / 65.0f;
    float fm  = -35.0f + (FM_STAND + 35.0f) * t;
    float tb  = -22.0f + (TB_STAND + 22.0f) * t;
    float sp  = sinf(t * PI) * 22.0f;  // Peak 22° spread at midpoint, 0 at start & end
    for (uint8_t i = 0; i < 6; i++) {
      // Left legs splay backward, right legs splay backward from their side
      float cx = (i < 3) ? -sp : sp;
      setLeg(i, cx, fm, tb);
    }
    svcDelay(12);
  }

  // ── Phase 4: Body tremor (rapid coxa oscillation) ─────────────────
  setStatus("CALIBRATING BALANCE...");
  for (uint8_t k = 0; k < 5; k++) {
    for (uint8_t i = 0; i < 6; i++) setLeg(i,  8.0f, FM_STAND, TB_STAND);
    svcDelay(55);
    for (uint8_t i = 0; i < 6; i++) setLeg(i, -8.0f, FM_STAND, TB_STAND);
    svcDelay(55);
  }

  // ── Phase 5: Bow greeting sequence ───────────────────────────────
  // Front legs dip down, body leans forward, then snaps upright
  setLeg(0, 0.0f, FM_STAND - 15.0f, TB_STAND);  // FL dips
  setLeg(3, 0.0f, FM_STAND - 15.0f, TB_STAND);  // FR dips
  svcDelay(220);
  setLeg(0, 0.0f, FM_STAND + 10.0f, TB_STAND);
  setLeg(3, 0.0f, FM_STAND + 10.0f, TB_STAND);
  svcDelay(140);

  // ── Phase 6: Final stand ─────────────────────────────────────────
  standAll();
  svcDelay(350);

  setStatus("HEX-X1 ONLINE — AWAITING COMMAND");
  isActive = false;
  curMode  = IDLE;
}

// ═══════════════════════ Tripod Walk ═════════════════════════════
// One half-cycle of the tripod gait.
// Swing group: lifts off, arcs over, lands at new position.
// Stance group: stays grounded, pushes body forward.
void tripodStep(uint8_t swingId, int8_t dir) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;

  float se  =  (float)dir * SWING;  // Swing target
  float ste = -(float)dir * SWING;  // Stance target (opposite = push)

  float ss[3], ts[3];
  for (uint8_t i = 0; i < 3; i++) { ss[i] = cxPos[sg[i]]; ts[i] = cxPos[st[i]]; }

  uint16_t sd = max((uint16_t)1, (uint16_t)(STEP_MS / ISTEPS));

  for (uint8_t s = 0; s <= ISTEPS; s++) {
    svcAll();
    if (!isActive) return;

    float t  = (float)s / (float)ISTEPS;
    float lf = sinf(t * PI);                      // 0 → 1 → 0 arc
    float fs = FM_STAND - (LIFT * lf);            // Femur rises mid-swing

    for (uint8_t i = 0; i < 3; i++) {
      uint8_t li = sg[i];
      cxPos[li]  = ss[i] + (se - ss[i]) * t;
      setLeg(li, cxPos[li], fs, TB_STAND);
    }
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t li = st[i];
      cxPos[li]  = ts[i] + (ste - ts[i]) * t;
      setLeg(li, cxPos[li], FM_STAND, TB_STAND);
    }
    svcDelay(sd);
  }
}

void walkCycle(int8_t dir) {
  tripodStep(0, dir);
  if (!isActive) return;
  tripodStep(1, dir);
}

// ═══════════════════════ Tripod Turn ═════════════════════════════
// In-place rotation: left and right legs push in OPPOSITE logical directions.
// Left legs (dir=+1): stance target = +td*TS → sweeps forward
// Right legs (dir=-1): we pass -td*TS to setLeg → coxaDir flips to +td*TS physically
// Net: all legs physically push the same rotational direction → body rotates.
// turnDir: +1 = clockwise (right), -1 = counter-clockwise (left)
// NOTE: If robot turns the wrong way, swap TURN_L/TURN_R commands or flip turnDir signs below.
void tripodTurnStep(uint8_t swingId, int8_t td) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;

  float TS = SWING * 0.75f;  // Slightly smaller amplitude for turns

  // Determine stance/swing targets per leg based on left vs right side
  float swTgt[3], stTgt[3];
  for (uint8_t i = 0; i < 3; i++) {
    bool sL = (sg[i] < 3);  // Is swing leg on the left?
    bool gL = (st[i] < 3);  // Is stance leg on the left?
    // Swing resets to opposite side (ready for next push)
    swTgt[i] = sL ? -(float)td * TS :  (float)td * TS;
    // Stance pushes in rotation direction — opposite sign for left vs right
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

    for (uint8_t i = 0; i < 3; i++) {
      uint8_t li = sg[i];
      cxPos[li]  = ss[i] + (swTgt[i] - ss[i]) * t;
      setLeg(li, cxPos[li], fs, TB_STAND);
    }
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t li = st[i];
      cxPos[li]  = ts[i] + (stTgt[i] - ts[i]) * t;
      setLeg(li, cxPos[li], FM_STAND, TB_STAND);
    }
    svcDelay(sd);
  }
}

void turnCycle(int8_t td) {
  tripodTurnStep(0, td);
  if (!isActive) return;
  tripodTurnStep(1, td);
}

// ═══════════════════════ Wave / Flow Mode ════════════════════════
// All tibias stay static. Femurs undulate in a sinusoidal wave across legs.
// Each leg has a phase offset so the wave travels smoothly along the body.
// The robot appears to breathe and ripple — all 6 feet stay on the ground.
void runWave() {
  float t = 0.0f;
  const float AMP = 11.0f;   // Femur amplitude (degrees). Bigger = more dramatic.
  const float SPD = 0.075f;  // Phase increment per frame. Bigger = faster wave.

  while (isActive && curMode == WAVE) {
    t += SPD;
    if (t > 2.0f * PI) t -= 2.0f * PI;  // Wrap to prevent float drift

    for (uint8_t i = 0; i < 6; i++) {
      float fm = FM_STAND + AMP * sinf(t + WPH[i]);
      setLeg(i, 0.0f, fm, TB_STAND);
    }
    svcAll();
    delay(22);
  }
}

// ═══════════════════════ Dance Mode ══════════════════════════════
// 6-phase pre-programmed routine. Each phase can be interrupted by 'S'.
void doDance() {
  setStatus("DANCE SEQUENCE ACTIVE");
  auto ok = [&]() { return isActive && curMode == DANCE; };

  // ── Phase 1: Side sway (body rocks left-right 4 times) ──────────
  for (uint8_t r = 0; r < 4 && ok(); r++) {
    float s = (r % 2 == 0) ? 15.0f : -15.0f;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, (i < 3) ? s : -s, FM_STAND, TB_STAND);
    svcDelay(220);
  }
  standAll(); if (!ok()) return;

  // ── Phase 2: Tap dance (each leg raises and stamps in sequence) ──
  const uint8_t SEQ[6] = { 0, 3, 1, 4, 2, 5 };
  for (uint8_t n = 0; n < 6 && ok(); n++) {
    uint8_t li = SEQ[n];
    setLeg(li, 0.0f, FM_STAND - LIFT - 6.0f, TB_STAND); svcDelay(130);
    setLeg(li, 0.0f, FM_STAND,                TB_STAND); svcDelay(100);
  }
  if (!ok()) return;

  // ── Phase 3: Body bounce (all legs bob up-down rhythmically) ─────
  for (uint8_t k = 0; k < 8 && ok(); k++) {
    float fm = (k % 2 == 0) ? FM_STAND - 13.0f : FM_STAND;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, TB_STAND);
    svcDelay(150);
  }
  if (!ok()) return;

  // ── Phase 4: Full body wave ripple ───────────────────────────────
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

  // ── Phase 5: Spin burst (2 full tripod-turn cycles) ──────────────
  for (uint8_t r = 0; r < 2 && ok(); r++) turnCycle(+1);
  if (!ok()) return;

  // ── Phase 6: Victory pose — legs rise then slam down ─────────────
  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 20.0f, TB_STAND);
  svcDelay(350);
  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND +  9.0f, TB_STAND);
  svcDelay(130);
  standAll();

  setStatus("DANCE COMPLETE — STANDING BY");
  curMode = IDLE; isActive = false; danceFlag = false;
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
      if (curMode != WAKING) {
        isActive = false;   // Interrupt any ongoing motion gracefully
        wakeFlag = true;
        setStatus("WAKE: QUEUED");
      }
      break;

    case 'F': case 'f':
      wakeFlag = danceFlag = false;
      curMode = WALK_F; isActive = true;
      setStatus("WALK: FORWARD");
      break;

    case 'B': case 'b':
      wakeFlag = danceFlag = false;
      curMode = WALK_B; isActive = true;
      setStatus("WALK: BACKWARD");
      break;

    case 'L': case 'l':
      wakeFlag = danceFlag = false;
      curMode = TURN_L; isActive = true;
      setStatus("TURN: LEFT");
      break;

    case 'R': case 'r':
      wakeFlag = danceFlag = false;
      curMode = TURN_R; isActive = true;
      setStatus("TURN: RIGHT");
      break;

    case 'D': case 'd':
      wakeFlag = false;
      danceFlag = true; curMode = DANCE; isActive = true;
      setStatus("DANCE: INITIATED");
      break;

    case 'V': case 'v':
      wakeFlag = danceFlag = false;
      curMode = WAVE; isActive = true;
      setStatus("WAVE: FLOW MODE");
      break;

    case 'S': case 's':
      isActive = wakeFlag = danceFlag = false;
      curMode = IDLE;
      standAll();
      setStatus("HALT — STANDING BY");
      break;

    case '1': STEP_MS = 720; setStatus("SPEED: SLOW  [720ms]"); break;
    case '2': STEP_MS = 500; setStatus("SPEED: NORMAL [500ms]"); break;
    case '3': STEP_MS = 300; setStatus("SPEED: FAST  [300ms]"); break;

    case '+':
      SWING = min(SWING + 2.0f, 38.0f);
      setStatus("STRIDE: " + String((int)SWING) + " DEG");
      break;
    case '-':
      SWING = max(SWING - 2.0f, 5.0f);
      setStatus("STRIDE: " + String((int)SWING) + " DEG");
      break;

    case '\n': case '\r': case ' ': break;
    default: break;
  }
}

void handleSerial() {
  while (Serial.available()) processCmd((char)Serial.read());
}

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
.card::before,.card::after{content:'';position:absolute;width:14px;height:14px;
 border-color:#00ff88;border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
h1{font-size:1.35rem;color:#00ff88;letter-spacing:4px;text-shadow:0 0 14px #00ff88;margin-bottom:5px}
p{color:#3a7a5a;font-size:0.78rem;letter-spacing:1.2px;margin-bottom:18px}
.err{color:#ff7777;background:rgba(255,51,51,0.11);border:1px solid rgba(255,51,51,0.42);
 border-radius:2px;padding:9px 11px;margin-bottom:13px;font-size:0.8rem;letter-spacing:0.5px}
input{width:100%;padding:14px 13px;border:1px solid rgba(0,255,136,0.3);border-radius:2px;
 background:rgba(0,255,136,0.04);color:#b0ffd0;font-family:inherit;font-size:1rem;
 outline:none;letter-spacing:1.5px}
input:focus{border-color:#00ff88;box-shadow:0 0 10px rgba(0,255,136,0.18)}
button{margin-top:11px;width:100%;padding:14px;border:1px solid rgba(0,255,136,0.5);
 border-radius:2px;background:rgba(0,255,136,0.07);color:#00ff88;font-family:inherit;
 font-size:0.88rem;font-weight:700;letter-spacing:2.5px;cursor:pointer;text-transform:uppercase;
 transition:all 0.13s}
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
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--bg:#06090f;--t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;
 background:radial-gradient(ellipse 130% 55% at 12% -5%,#001825,transparent 52%),
            radial-gradient(ellipse 80% 40% at 92% 108%,#001508,transparent 46%),var(--bg);
 color:var(--t);font-family:'Courier New',Courier,monospace;
 display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(455px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;
 padding:13px;position:relative;overflow:hidden;
 box-shadow:0 0 35px rgba(0,255,136,0.06),inset 0 1px 0 rgba(0,255,136,0.11)}
.card::before,.card::after{content:'';position:absolute;width:16px;height:16px;
 border-color:var(--g);border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
/* Header */
.hdr{display:flex;align-items:baseline;gap:8px;padding-bottom:8px;
 border-bottom:1px solid rgba(0,255,136,0.13);margin-bottom:9px}
.logo{font-size:1.3rem;color:var(--g);letter-spacing:3.5px;text-shadow:0 0 12px var(--g)}
.sub{font-size:0.6rem;color:var(--m);letter-spacing:1.5px;margin-left:auto}
/* Status bar */
.sts{background:rgba(0,0,0,0.65);border:1px solid rgba(0,255,136,0.17);border-radius:2px;
 padding:6px 10px;font-size:0.74rem;color:var(--g);letter-spacing:1px;margin-bottom:9px;
 min-height:28px;display:flex;align-items:center;gap:7px;overflow:hidden;white-space:nowrap}
.sts::before{content:'▶';animation:bl 1.2s step-end infinite;flex-shrink:0;font-size:0.65rem}
@keyframes bl{0%,100%{opacity:1}50%{opacity:0}}
/* Mode pills */
.pills{display:flex;flex-wrap:wrap;gap:4px;margin-bottom:9px;align-items:center}
.pills span:first-child{font-size:0.6rem;color:var(--m);letter-spacing:1.5px}
.pill{padding:2px 8px;border:1px solid rgba(0,255,136,0.18);border-radius:20px;
 font-size:0.6rem;color:var(--m);letter-spacing:1px;transition:all 0.2s}
.pill.on{border-color:var(--g);color:var(--g);background:rgba(0,255,136,0.11);
 box-shadow:0 0 7px rgba(0,255,136,0.28)}
/* Main area: joystick + dpad */
.main{display:grid;grid-template-columns:auto 1fr;gap:10px;align-items:center;margin-bottom:9px}
.jw{display:grid;place-items:center;gap:5px}
/* Joystick */
.jb{width:168px;height:168px;border-radius:50%;border:1px solid var(--br);
 background:radial-gradient(circle at 40% 35%,rgba(0,255,136,0.09),rgba(0,0,0,0.5) 72%);
 position:relative;touch-action:none;cursor:grab;
 box-shadow:0 0 18px rgba(0,255,136,0.07),inset 0 0 22px rgba(0,0,0,0.55)}
.js{width:62px;height:62px;border-radius:50%;
 border:1px solid rgba(0,255,136,0.75);
 background:radial-gradient(circle at 36% 36%,rgba(0,255,136,0.52),rgba(0,255,136,0.16));
 position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
 box-shadow:0 0 14px rgba(0,255,136,0.45)}
.jb.active .js{box-shadow:0 0 24px rgba(0,255,136,0.8)}
.jh{font-size:0.62rem;color:var(--m);text-align:center;max-width:168px;line-height:1.5}
/* D-pad */
.dp{display:grid;grid-template-columns:repeat(3,38px);grid-template-rows:repeat(3,38px);gap:3px}
.dc{width:38px;height:38px;border:1px solid rgba(0,255,136,0.12);border-radius:2px;
 display:grid;place-items:center;font-size:0.85rem;color:var(--m);transition:all 0.15s}
.dc.on{border-color:var(--g);background:rgba(0,255,136,0.17);color:var(--g);
 box-shadow:0 0 8px rgba(0,255,136,0.38)}
.dc-mid{border-color:rgba(0,255,136,0.25)!important}
/* Section label */
.lbl{font-size:0.59rem;color:var(--m);letter-spacing:2px;text-transform:uppercase;
 margin:8px 0 4px;display:flex;align-items:center;gap:6px}
.lbl::after{content:'';flex:1;height:1px;background:rgba(0,255,136,0.1)}
/* Grids */
.g1{display:grid;grid-template-columns:1fr;gap:5px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:5px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px}
/* Buttons — reset then style */
button{all:unset;display:block;width:100%;text-align:center;padding:11px 6px;
 border:1px solid rgba(0,255,136,0.26);border-radius:2px;font-family:inherit;
 font-size:0.76rem;font-weight:700;letter-spacing:1.5px;color:var(--t);
 background:rgba(0,0,0,0.55);cursor:pointer;touch-action:manipulation;
 transition:all 0.12s;text-transform:uppercase}
button:hover{background:rgba(0,255,136,0.09);border-color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.2)}
button:active{transform:scale(0.97)}
.btn-wake{border-color:rgba(0,204,255,0.48);color:var(--g2)}
.btn-wake:hover{background:rgba(0,204,255,0.09);box-shadow:0 0 9px rgba(0,204,255,0.28)}
.btn-stop{border-color:rgba(255,51,68,0.52);color:var(--r)}
.btn-stop:hover{background:rgba(255,51,68,0.1);box-shadow:0 0 9px rgba(255,51,68,0.28)}
.btn-mode.on{background:rgba(0,255,136,0.16);border-color:var(--g);color:var(--g);
 box-shadow:0 0 9px rgba(0,255,136,0.28)}
/* Speed active */
.spd-btn.on{background:rgba(0,204,255,0.12);border-color:rgba(0,204,255,0.6);
 color:var(--g2);box-shadow:0 0 7px rgba(0,204,255,0.22)}
</style></head><body>
<div class="card">
 <div class="hdr">
  <div class="logo">HEX-X1</div>
  <div class="sub">HEXAPOD CONTROL UNIT v2.1</div>
 </div>

 <div class="sts" id="sts">STANDBY</div>

 <div class="pills">
  <span>MODE:</span>
  <span class="pill" id="pIdle">IDLE</span>
  <span class="pill" id="pWalk">WALK</span>
  <span class="pill" id="pTurn">TURN</span>
  <span class="pill" id="pDance">DANCE</span>
  <span class="pill" id="pWave">WAVE</span>
  <span class="pill" id="pWake">WAKING</span>
 </div>

 <!-- Joystick + D-pad -->
 <div class="main">
  <div class="jw">
   <div id="jb" class="jb"><div id="js" class="js"></div></div>
   <div class="jh">Y: FWD / BCK &nbsp;·&nbsp; X: TURN L / R</div>
  </div>
  <div>
   <div class="dp">
    <div class="dc"></div>
    <div class="dc" id="dcF">▲</div>
    <div class="dc"></div>
    <div class="dc" id="dcL">↺</div>
    <div class="dc dc-mid">●</div>
    <div class="dc" id="dcR">↻</div>
    <div class="dc"></div>
    <div class="dc" id="dcB">▼</div>
    <div class="dc"></div>
   </div>
  </div>
 </div>

 <!-- Special Modes -->
 <div class="lbl">special modes</div>
 <div class="g2">
  <button id="btnD" class="btn-mode" onclick="toggleMode('D','btnD')">◈  Dance</button>
  <button id="btnV" class="btn-mode" onclick="toggleMode('V','btnV')">〜  Wave Flow</button>
 </div>

 <!-- Speed -->
 <div class="lbl">speed preset</div>
 <div class="g3">
  <button id="spd1" class="spd-btn" onclick="setSpeed('1','spd1')">●  Slow</button>
  <button id="spd2" class="spd-btn on" onclick="setSpeed('2','spd2')">●●  Normal</button>
  <button id="spd3" class="spd-btn" onclick="setSpeed('3','spd3')">●●●  Fast</button>
 </div>

 <!-- Stride -->
 <div class="lbl">stride size</div>
 <div class="g2">
  <button onclick="cmd('+')">Stride  +</button>
  <button onclick="cmd('-')">Stride  −</button>
 </div>

 <!-- System actions -->
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
const jb = $('jb'), js = $('js');
const R=52, TH=17;
let drag=false, lastCmd='S', activeMode=null, activeSpdBtn='spd2';

function setSts(t){ $('sts').textContent=t; }

/* ── Mode pills ── */
const PILL={'F':'pWalk','B':'pWalk','L':'pTurn','R':'pTurn',
            'D':'pDance','V':'pWave','W':'pWake','S':'pIdle'};
let lastPill='pIdle';
function pillOn(c){
  $(lastPill)?.classList.remove('on');
  const p=PILL[c.toUpperCase()]||'pIdle';
  $(p)?.classList.add('on'); lastPill=p;
}

/* ── D-pad ── */
const DMAP={'F':'dcF','B':'dcB','L':'dcL','R':'dcR'};
let lastDC=null;
function dpadOn(c){
  if(lastDC) $(lastDC)?.classList.remove('on'); lastDC=null;
  const d=DMAP[c.toUpperCase()];
  if(d){ $(d)?.classList.add('on'); lastDC=d; }
}

/* ── HTTP helper ── */
async function cmd(c){
  pillOn(c); dpadOn(c);
  try{
    const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'});
    if(r.status===401){location='/';return;}
    setSts(await r.text());
  }catch(e){setSts('ERR: '+e.message);}
}

/* ── Special mode toggle (D / V) ── */
function toggleMode(c, btnId){
  const btn=$(btnId);
  if(activeMode===c){          // Already active — send Stop
    btn.classList.remove('on');
    activeMode=null; cmd('S');
  } else {
    if(activeMode){
      const prev=activeMode==='D'?'btnD':'btnV';
      $(prev)?.classList.remove('on');
    }
    btn.classList.add('on');
    activeMode=c; cmd(c);
  }
}

/* ── Speed buttons ── */
function setSpeed(c, btnId){
  if(activeSpdBtn) $(activeSpdBtn)?.classList.remove('on');
  $(btnId)?.classList.add('on');
  activeSpdBtn=btnId; cmd(c);
}

/* ── Clear mode buttons on movement/stop ── */
function clearModeBtn(){
  if(activeMode){
    $(activeMode==='D'?'btnD':'btnV')?.classList.remove('on');
    activeMode=null;
  }
}

/* ── Status poll ── */
async function poll(){
  try{
    const r=await fetch('/status',{cache:'no-store'});
    if(r.status===401){location='/';return;}
    setSts(await r.text());
  }catch(e){}
}

/* ── Joystick ── */
function joyPt(cx,cy){
  const rect=jb.getBoundingClientRect();
  let dx=cx-(rect.left+rect.width/2), dy=cy-(rect.top+rect.height/2);
  const d=Math.hypot(dx,dy);
  if(d>R){const k=R/d;dx*=k;dy*=k;}
  return{dx,dy};
}
function joyCmd(dx,dy){
  const ax=Math.abs(dx),ay=Math.abs(dy);
  if(ax<TH&&ay<TH)return'S';
  return ay>=ax?(dy<0?'F':'B'):(dx<0?'L':'R');
}
function applyJoy(dx,dy){
  js.style.transform='translate(calc(-50% + '+dx+'px),calc(-50% + '+dy+'px))';
  const c=joyCmd(dx,dy);
  if(c!==lastCmd){
    lastCmd=c;
    if(c!=='S') clearModeBtn();
    cmd(c);
  }
}
jb.addEventListener('pointerdown',e=>{
  drag=true; jb.setPointerCapture(e.pointerId); jb.classList.add('active');
  const{dx,dy}=joyPt(e.clientX,e.clientY); applyJoy(dx,dy);
},{passive:true});
jb.addEventListener('pointermove',e=>{
  if(!drag)return;
  const{dx,dy}=joyPt(e.clientX,e.clientY); applyJoy(dx,dy);
},{passive:true});
function resetJoy(){
  drag=false; jb.classList.remove('active');
  js.style.transform='translate(-50%,-50%)';
  if(lastCmd!=='S'){lastCmd='S';cmd('S');}
}
jb.addEventListener('pointerup',resetJoy);
jb.addEventListener('pointercancel',resetJoy);
jb.addEventListener('lostpointercapture',resetJoy);

poll();
setInterval(poll,2500);
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

  // ── Root: control page ──────────────────────────────────────────
  httpServer.on("/", HTTP_GET, []() {
    if (!checkAuth()) return;
    httpServer.send_P(200, "text/html", CTRL_HTML);
  });

  // ── Login GET ───────────────────────────────────────────────────
  httpServer.on("/login", HTTP_GET, []() {
    if (isAuth()) {
      httpServer.sendHeader("Location", "/", true);
      httpServer.send(302, "text/plain", "");
      return;
    }
    String p = FPSTR(LOGIN_HTML);
    p.replace("%ERR%", "");
    httpServer.send(200, "text/html", p);
  });

  // ── Login POST ──────────────────────────────────────────────────
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

  // ── Logout ──────────────────────────────────────────────────────
  httpServer.on("/logout", HTTP_GET, []() {
    authIP = IPAddress(0, 0, 0, 0); authExp = 0;
    httpServer.sendHeader("Set-Cookie", "HEX=0; Max-Age=0; Path=/", true);
    httpServer.sendHeader("Location", "/login", true);
    httpServer.send(302, "text/plain", "");
  });

  // ── Command endpoint ────────────────────────────────────────────
  httpServer.on("/cmd", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    if (!httpServer.hasArg("c") || httpServer.arg("c").isEmpty()) {
      httpServer.send(400, "text/plain", "ERR: missing 'c' param");
      return;
    }
    processCmd(httpServer.arg("c")[0]);
    httpServer.send(200, "text/plain", sysStatus);
  });

  // ── Status endpoint ─────────────────────────────────────────────
  httpServer.on("/status", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    httpServer.send(200, "text/plain", sysStatus);
  });

  // ── Captive portal redirects (Android/iOS/Windows probes) ───────
  auto redir = []() {
    httpServer.sendHeader("Location", "http://192.168.4.1/", true);
    httpServer.send(302, "text/plain", "");
  };
  httpServer.on("/generate_204",       HTTP_GET, redir);
  httpServer.on("/hotspot-detect.html",HTTP_GET, redir);
  httpServer.on("/fwlink",             HTTP_GET, redir);
  httpServer.on("/ncsi.txt",           HTTP_GET, []() {
    httpServer.send(200, "text/plain", "Microsoft NCSI");
  });
  httpServer.onNotFound(redir);

  httpServer.begin();

  Serial.println("══════════════════════════════════");
  Serial.println("  WiFi SSID : " + String(AP_SSID));
  Serial.println("  WiFi PASS : " + String(AP_PASS));
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

  // Gentle power-on rise (no dramatic wake, just ensure servos initialize)
  for (uint8_t s = 0; s <= 45; s++) {
    float t = (float)s / 45.0f;
    float fm = -12.0f + (FM_STAND + 12.0f) * t;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, TB_STAND);
    delay(18);
  }
  standAll();

  setStatus("HEX-X1 READY · SEND 'W' TO WAKE");
  Serial.println("Commands: W F B L R D V S  1 2 3  + -");
}

// ══════════════════════════ Loop ═════════════════════════════════
void loop() {
  svcAll();
  handleSerial();

  // ── Priority 1: Wake request ─────────────────────────────────────
  if (wakeFlag) {
    wakeFlag = false;
    doWake();     // Blocking, calls svcAll() internally throughout
    return;
  }

  // ── Priority 2: Dance (one full routine per trigger) ─────────────
  if (curMode == DANCE && isActive && danceFlag) {
    danceFlag = false;
    doDance();    // Blocking, calls svcAll() internally; exits when done or stopped
    return;
  }

  // ── Priority 3: Motion dispatch ──────────────────────────────────
  switch (curMode) {
    case WALK_F:
      if (isActive) walkCycle(+1 * DIR_SIGN);
      break;

    case WALK_B:
      if (isActive) walkCycle(-1 * DIR_SIGN);
      break;

    case TURN_L:
      // If robot turns wrong way, swap to turnCycle(+1)
      if (isActive) turnCycle(-1);
      break;

    case TURN_R:
      // If robot turns wrong way, swap to turnCycle(-1)
      if (isActive) turnCycle(+1);
      break;

    case WAVE:
      if (isActive) runWave();   // Blocks until stopped or mode changes
      break;

    case IDLE:
    default:
      standAll();
      svcDelay(25);
      break;
  }
}
