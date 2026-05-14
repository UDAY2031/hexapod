/*
 ╔══════════════════════════════════════════════════════════════════════╗
 ║   HEX-X1  ·  Hexapod Alien Controller  ·  ESP32  v4.0               ║
 ║   Hardware : ESP32 + 2×PCA9685 (0x40=Left, 0x41=Right)              ║
 ╠══════════════════════════════════════════════════════════════════════╣
 ║  Serial / HTTP commands:                                             ║
 ║   W = Wake (auto on boot)   F = Forward    B = Backward             ║
 ║   L = Turn Left    R = Turn Right    S = Stop                       ║
 ║   Q = Crab Left    E_crab = Crab Right                              ║
 ║   7 = Diag FL  9 = Diag FR  1 = Diag BL  3 = Diag BR              ║
 ║   D = Dance Alpha   E = Dance Beta    G = Dance Gamma               ║
 ║   H = Dance Delta   I = Moonwalk      J = Breakdance               ║
 ║   K = Intimidate    M = Meditate      N = Ninja Dash               ║
 ║   V = Wave/Flow     T = Taunt         P = Pose/Salute              ║
 ║   X = Shake         Z = Zombie Walk                                 ║
 ║   1 = Slow  2 = Normal  3 = Fast   + / - = Stride                  ║
 ╚══════════════════════════════════════════════════════════════════════╝
*/

#include <Wire.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ══════════════════════════ Network ══════════════════════════════
const char* AP_SSID = "HEX-X1";
const char* AP_PASS = "hexapod123";
const char* UI_PASS = "alien";

const byte  DNS_PORT = 53;
IPAddress   apIP(192, 168, 4, 1);
DNSServer   dnsServer;
WebServer   httpServer(80);

// ══════════════════════════ PCA9685 ══════════════════════════════
Adafruit_PWMServoDriver pcaL(0x40);
Adafruit_PWMServoDriver pcaR(0x41);
static const float PWM_FREQ = 50.0f;

// ══════════════════════ Servo Calibration ════════════════════════
struct ServoCal { uint16_t usP45, usN45; uint8_t brd, ch; };
const ServoCal SVO[18] = {
  {  985, 2010,  0,  0 },  //  0  FL-Coxa
  { 1117, 1989,  0,  1 },  //  1  FL-Femur
  { 1018, 2037,  0,  2 },  //  2  FL-Tibia
  {  973, 2002,  0,  8 },  //  3  ML-Coxa
  { 1007, 1925,  0,  9 },  //  4  ML-Femur
  {  932, 1947,  0,  10 }, //  5  ML-Tibia
  { 1016, 2137,  0,  12 }, //  6  BL-Coxa
  {  956, 1868,  0,  13 }, //  7  BL-Femur
  {  976, 2069,  0,  14 }, //  8  BL-Tibia
  { 1038, 2032,  1,  0 },  //  9  FR-Coxa
  { 1043, 1962,  1,  1 },  // 10  FR-Femur
  {  918, 1962,  1,  2 },  // 11  FR-Tibia
  {  956, 1983,  1,  8 },  // 12  MR-Coxa
  { 1092, 2014,  1,  9 },  // 13  MR-Femur
  {  938, 1990,  1,  10 }, // 14  MR-Tibia
  {  930, 1998,  1,  12 }, // 15  BR-Coxa
  { 1052, 1973,  1,  13 }, // 16  BR-Femur
  {  995, 1978,  1,  14 }  // 17  BR-Tibia
};

// ══════════════════════════ Level Trim ═══════════════════════════
float trimDeg[18] = { 0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0 };

// ══════════════════════════ Leg Layout ═══════════════════════════
struct Leg { uint8_t cx, fm, tb; int8_t dir; };
const Leg LEG[6] = {
  { 0, 1, 2, +1 },  // 0 FL
  { 3, 4, 5, +1 },  // 1 ML
  { 6, 7, 8, +1 },  // 2 BL
  { 9,10,11, -1 },  // 3 FR
  {12,13,14, -1 },  // 4 MR
  {15,16,17, -1 }   // 5 BR
};

const uint8_t GA[3] = { 0, 4, 2 };
const uint8_t GB[3] = { 3, 1, 5 };

const float WPH[6] = { 0.0f, 2.094f, 4.189f, 3.1416f, 5.236f, 1.047f };

// ═══════════════════════ Gait Parameters ═════════════════════════
float    SWING    = 18.0f;
float    LIFT     = 14.0f;
float    FM_STAND =  0.0f;
float    TB_STAND =  0.0f;
uint16_t STEP_MS  = 500;
const uint8_t ISTEPS = 50;
const int8_t DIR_SIGN = -1;

// ═══════════════════════════ State ═══════════════════════════════
enum Mode : uint8_t {
  IDLE, WALK_F, WALK_B, TURN_L, TURN_R,
  CRAB_L, CRAB_R,
  DIAG_FL, DIAG_FR, DIAG_BL, DIAG_BR,
  DANCE_A, DANCE_B, DANCE_G, DANCE_D,
  DANCE_MOONWALK, DANCE_BREAK, DANCE_NINJA, DANCE_ZOMBIE,
  WAVE, WAKING, TAUNT, POSE, SHAKE, INTIMIDATE, MEDITATE
};
Mode     curMode  = IDLE;
bool     isActive = false;
bool     wakeFlag = false;
bool     danceFlag = false;
uint8_t  danceType = 0;
float    cxPos[6] = {};
String   sysStatus = "BOOTING...";

bool hasWoken = false;

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
void dormantAll() {
  for (uint8_t i = 0; i < 6; i++) {
    setLeg(i, 0.0f, -38.0f, -25.0f);
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

// ═══════════════════════ Wake Sequence (AUTO on boot) ═════════════
void doWake() {
  curMode = WAKING; isActive = true;
  setStatus("NEURAL PATHWAYS INITIALIZING...");

  dormantAll();
  svcDelay(400);

  // Phase 1: Individual leg spasms
  const uint8_t ORD[6] = { 2, 5, 0, 3, 1, 4 };
  for (uint8_t n = 0; n < 6; n++) {
    uint8_t li = ORD[n];
    setLeg(li, 0.0f, -25.0f, -15.0f); svcDelay(50);
    setLeg(li, 0.0f, -38.0f, -25.0f); svcDelay(40);
    setLeg(li, 0.0f, -15.0f,  -5.0f); svcDelay(60);
    setLeg(li, 0.0f, -38.0f, -25.0f); svcDelay(40);
    setLeg(li, 0.0f, -10.0f,  -3.0f); svcDelay(45);
    setLeg(li, 0.0f, -38.0f, -25.0f); svcDelay(35);
    svcAll();
  }
  svcDelay(300);

  // Phase 2: Full body shudder
  setStatus("MOTOR CORTEX ENGAGING...");
  for (uint8_t k = 0; k < 3; k++) {
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, -30.0f, -18.0f);
    svcDelay(70);
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, -38.0f, -25.0f);
    svcDelay(60);
  }
  svcDelay(200);

  // Phase 3: Alien rise with leg splay
  setStatus("EXOSKELETON DEPLOYING...");
  for (uint8_t s = 0; s <= 80; s++) {
    float t   = (float)s / 80.0f;
    float fm  = -38.0f + (FM_STAND + 38.0f) * t;
    float tb  = -25.0f + (TB_STAND + 25.0f) * t;
    float sp  = sinf(t * PI) * 28.0f;
    for (uint8_t i = 0; i < 6; i++) {
      float cx;
      if (i == 0 || i == 3) cx = (i < 3) ? sp : -sp;
      else if (i == 2 || i == 5) cx = (i < 3) ? -sp : sp;
      else cx = (i < 3) ? -sp * 0.5f : sp * 0.5f;
      setLeg(i, cx, fm, tb);
    }
    svcDelay(10);
  }

  // Phase 4: Gyro stabilization tremor
  setStatus("GYROSCOPE CALIBRATING...");
  for (uint8_t k = 0; k < 6; k++) {
    float jit = (k % 2 == 0) ? 10.0f : -10.0f;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, (i < 3 ? jit : -jit), FM_STAND, TB_STAND);
    svcDelay(50);
  }

  // Phase 5: Front leg scan
  setStatus("SCANNING PERIMETER...");
  setLeg(0, 20.0f, FM_STAND - 8.0f, TB_STAND); svcDelay(120);
  setLeg(3, 20.0f, FM_STAND - 8.0f, TB_STAND); svcDelay(120);
  setLeg(0, 0.0f, FM_STAND, TB_STAND); svcDelay(80);
  setLeg(3, 0.0f, FM_STAND, TB_STAND); svcDelay(80);

  // Phase 6: Victory shake
  for (uint8_t k = 0; k < 8; k++) {
    for (uint8_t i = 0; i < 6; i++) setLeg(i, (i%2==0 ? 12.0f : -12.0f), FM_STAND, TB_STAND);
    svcDelay(45);
    for (uint8_t i = 0; i < 6; i++) setLeg(i, (i%2==0 ? -12.0f : 12.0f), FM_STAND, TB_STAND);
    svcDelay(45);
  }

  // Phase 7: Greeting bow
  setLeg(0, 0.0f, FM_STAND - 18.0f, TB_STAND);
  setLeg(3, 0.0f, FM_STAND - 18.0f, TB_STAND);
  svcDelay(280);
  setLeg(0, 0.0f, FM_STAND + 12.0f, TB_STAND);
  setLeg(3, 0.0f, FM_STAND + 12.0f, TB_STAND);
  svcDelay(150);

  standAll();
  svcDelay(400);

  hasWoken = true;
  setStatus("HEX-X1 ONLINE — AWAITING ORDERS");
  isActive = false;
  curMode  = IDLE;
}

// ═══════════════════════ Tripod Walk ═════════════════════════════
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

// ═══════════════════ Crab Walk (pure lateral) ════════════════════
// Lateral movement: femur-driven side shuffle
// Left side legs extend laterally, right side retracts, then swap
void crabStep(uint8_t swingId, int8_t dir) {
  // dir: +1 = crab right, -1 = crab left
  // Swing group lifts and pushes laterally via femur offset
  // Using femur angle to simulate lateral push
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;

  float crabAngle = SWING * 0.9f * (float)dir;
  float swTgt[3], stTgt[3];

  // For crab: left legs (0,1,2) push one way, right legs push other
  for (uint8_t i = 0; i < 3; i++) {
    bool leftLeg = (sg[i] < 3);
    swTgt[i] = leftLeg ? crabAngle : -crabAngle;
    bool leftSt = (st[i] < 3);
    stTgt[i] = leftSt ? -crabAngle : crabAngle;
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
void crabCycle(int8_t dir) {
  crabStep(0, dir);
  if (!isActive) return;
  crabStep(1, dir);
}

// ═══════════════ Diagonal Walk ════════════════════════════════════
// Combines forward/backward with turn to produce diagonal movement
void diagStep(uint8_t swingId, int8_t fwdDir, int8_t sideDir) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;

  float FS  =  (float)fwdDir  * SWING * 0.7f;
  float FSt = -(float)fwdDir  * SWING * 0.7f;
  float TS  =  SWING * 0.5f;

  float swTgt[3], stTgt[3];
  for (uint8_t i = 0; i < 3; i++) {
    bool leftSw = (sg[i] < 3);
    bool leftSt = (st[i] < 3);
    float turnSw = leftSw ? -(float)sideDir * TS :  (float)sideDir * TS;
    float turnSt = leftSt ?  (float)sideDir * TS : -(float)sideDir * TS;
    swTgt[i] = FS + turnSw;
    stTgt[i] = FSt + turnSt;
    swTgt[i] = constrain(swTgt[i], -38.0f, 38.0f);
    stTgt[i] = constrain(stTgt[i], -38.0f, 38.0f);
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
void diagCycle(int8_t fwdDir, int8_t sideDir) {
  diagStep(0, fwdDir, sideDir);
  if (!isActive) return;
  diagStep(1, fwdDir, sideDir);
}

// ═══════════════════════ Wave / Flow Mode ════════════════════════
void runWave() {
  float t = 0.0f;
  const float AMP = 11.0f;
  const float SPD = 0.075f;
  while (isActive && curMode == WAVE) {
    t += SPD;
    if (t > 2.0f * PI) t -= 2.0f * PI;
    for (uint8_t i = 0; i < 6; i++) {
      float fm = FM_STAND + AMP * sinf(t + WPH[i]);
      setLeg(i, 0.0f, fm, TB_STAND);
    }
    svcAll();
    delay(22);
  }
}

// ════════════════ Dance Alpha: Rave Stomp ═════════════════════════
void doDanceAlpha() {
  setStatus("DANCE-A: RAVE STOMP");
  auto ok = [&]() { return isActive && curMode == DANCE_A; };

  for (uint8_t k = 0; k < 4 && ok(); k++) {
    float fm = (k%2==0) ? FM_STAND - 20.0f : FM_STAND + 8.0f;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, TB_STAND);
    svcDelay(120);
  }
  for (uint8_t r = 0; r < 8 && ok(); r++) {
    const uint8_t* gr = (r%2==0) ? GA : GB;
    const uint8_t* gs = (r%2==0) ? GB : GA;
    for (uint8_t i = 0; i < 3; i++) setLeg(gr[i], 0.0f, FM_STAND - 22.0f, TB_STAND);
    for (uint8_t i = 0; i < 3; i++) setLeg(gs[i], 0.0f, FM_STAND + 5.0f, TB_STAND);
    svcDelay(110);
    for (uint8_t i = 0; i < 3; i++) setLeg(gr[i], 0.0f, FM_STAND, TB_STAND);
    svcDelay(70);
  }
  for (uint8_t r = 0; r < 6 && ok(); r++) {
    float s = (r%2==0) ? 16.0f : -16.0f;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, (i<3?s:-s), FM_STAND - 8.0f, TB_STAND);
    svcDelay(180);
  }
  standAll(); if (!ok()) return;
  float wt = 0.0f;
  for (uint16_t f = 0; f < 100 && ok(); f++) {
    wt += 0.18f;
    for (uint8_t i = 0; i < 6; i++) {
      float fm = FM_STAND + LIFT * sinf(wt + WPH[i]);
      float cx = 8.0f * sinf(wt + WPH[i] + 1.0f);
      setLeg(i, cx, fm, TB_STAND);
    }
    svcDelay(16);
  }
  for (uint8_t r = 0; r < 3 && ok(); r++) turnCycle(+1);
  standAll();
  setStatus("DANCE-A COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance Beta: Tai Chi Glide ═══════════════════════
void doDanceBeta() {
  setStatus("DANCE-B: TAI CHI GLIDE");
  auto ok = [&]() { return isActive && curMode == DANCE_B; };

  float t = 0.0f;
  for (uint16_t f = 0; f < 200 && ok(); f++) {
    t += 0.04f;
    for (uint8_t i = 0; i < 6; i++) {
      float fm = FM_STAND + 14.0f * sinf(t + WPH[i]);
      float tb = TB_STAND + 8.0f  * sinf(t + WPH[i] + 0.5f);
      float cx = 10.0f * sinf(t + WPH[i] + 1.2f);
      setLeg(i, cx, fm, tb);
    }
    svcDelay(20);
  }
  if (!ok()) return;
  for (uint8_t k = 0; k < 6 && ok(); k++) {
    float fm = (k%2==0) ? FM_STAND - 16.0f : FM_STAND + 6.0f;
    float tb = (k%2==0) ? TB_STAND - 8.0f  : TB_STAND + 3.0f;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, tb);
    svcDelay(280);
  }
  standAll(); if (!ok()) return;
  for (uint8_t r = 0; r < 4 && ok(); r++) {
    float cx = (r%2==0) ? 20.0f : -20.0f;
    for (uint8_t i = 0; i < 3; i++) setLeg(i,  cx, FM_STAND, TB_STAND);
    for (uint8_t i = 3; i < 6; i++) setLeg(i, -cx, FM_STAND, TB_STAND);
    svcDelay(320);
  }
  standAll();
  setStatus("DANCE-B COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance Gamma: Spider Creep ═══════════════════════
void doDanceGamma() {
  setStatus("DANCE-G: SPIDER CREEP");
  auto ok = [&]() { return isActive && curMode == DANCE_G; };

  for (uint8_t s = 0; s <= 30 && ok(); s++) {
    float t = (float)s/30.0f;
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i<3 ? 1.0f : -1.0f) * 18.0f * t;
      setLeg(i, cx, FM_STAND - 10.0f * t, TB_STAND);
    }
    svcDelay(20);
  }
  if (!ok()) return;

  const uint8_t CREEP[6] = {0,3,1,4,2,5};
  float liftFm = FM_STAND - 10.0f;
  for (uint8_t rep = 0; rep < 3 && ok(); rep++) {
    for (uint8_t n = 0; n < 6 && ok(); n++) {
      uint8_t li = CREEP[n];
      float cx = (li<3) ? 18.0f : -18.0f;
      setLeg(li, cx * 0.5f, liftFm - 18.0f, TB_STAND); svcDelay(100);
      setLeg(li, cx, liftFm, TB_STAND); svcDelay(120);
    }
  }
  if (!ok()) return;

  for (uint8_t r = 0; r < 4 && ok(); r++) {
    const uint8_t* sg = (r%2==0) ? GA : GB;
    const uint8_t* st = (r%2==0) ? GB : GA;
    for (uint8_t i = 0; i < 3; i++) setLeg(sg[i], (sg[i]<3?18.0f:-18.0f), liftFm - 14.0f, TB_STAND);
    svcDelay(150);
    for (uint8_t i = 0; i < 3; i++) setLeg(sg[i], (sg[i]<3?18.0f:-18.0f), liftFm, TB_STAND);
    svcDelay(120);
  }
  if (!ok()) return;

  for (uint8_t s = 0; s <= 30 && ok(); s++) {
    float t = (float)s/30.0f;
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i<3 ? 1.0f : -1.0f) * 18.0f * (1.0f - t);
      setLeg(i, cx, FM_STAND - 10.0f * (1.0f - t), TB_STAND);
    }
    svcDelay(20);
  }
  standAll();
  setStatus("DANCE-G COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance Delta: Mech Warrior ═══════════════════════
void doDanceDelta() {
  setStatus("DANCE-D: MECH WARRIOR");
  auto ok = [&]() { return isActive && curMode == DANCE_D; };

  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 25.0f, TB_STAND);
  svcDelay(80);
  for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND + 15.0f, TB_STAND);
  svcDelay(80);
  standAll(); if (!ok()) return;

  for (uint8_t rep = 0; rep < 4 && ok(); rep++) {
    for (uint8_t i = 0; i < 3; i++) setLeg(i, 0.0f, FM_STAND - 20.0f, TB_STAND);
    svcDelay(200);
    for (uint8_t i = 0; i < 3; i++) setLeg(i, 0.0f, FM_STAND, TB_STAND);
    for (uint8_t i = 3; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 20.0f, TB_STAND);
    svcDelay(200);
    for (uint8_t i = 3; i < 6; i++) setLeg(i, 0.0f, FM_STAND, TB_STAND);
    svcDelay(50);
  }
  if (!ok()) return;

  const uint8_t STOMP[6] = {0,5,1,4,2,3};
  for (uint8_t n = 0; n < 6 && ok(); n++) {
    uint8_t li = STOMP[n];
    float side = (li<3) ? 22.0f : -22.0f;
    setLeg(li, side, FM_STAND - 28.0f, TB_STAND); svcDelay(90);
    setLeg(li, 0.0f, FM_STAND + 10.0f, TB_STAND); svcDelay(75);
    setLeg(li, 0.0f, FM_STAND, TB_STAND); svcDelay(55);
  }
  if (!ok()) return;

  for (uint8_t r = 0; r < 2 && ok(); r++) turnCycle(+1);
  for (uint8_t r = 0; r < 2 && ok(); r++) turnCycle(-1);
  if (!ok()) return;

  setLeg(0, 28.0f, FM_STAND - 22.0f, TB_STAND);
  setLeg(3, 28.0f, FM_STAND - 22.0f, TB_STAND);
  setLeg(2, -28.0f, FM_STAND + 8.0f, TB_STAND);
  setLeg(5, -28.0f, FM_STAND + 8.0f, TB_STAND);
  svcDelay(600);
  standAll();
  setStatus("DANCE-D COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance: Moonwalk ═════════════════════════════════
// Smooth backward glide with body sway — legs push in sequence
void doDanceMoonwalk() {
  setStatus("MOONWALK: SMOOTH CRIMINAL");
  auto ok = [&]() { return isActive && curMode == DANCE_MOONWALK; };

  for (uint8_t rep = 0; rep < 4 && ok(); rep++) {
    // Lean + glide: left side legs do slow backward push, right side lifts
    for (uint8_t s = 0; s <= 40 && ok(); s++) {
      float t = (float)s / 40.0f;
      float lean = sinf(t * PI) * 12.0f;
      for (uint8_t i = 0; i < 3; i++) {
        setLeg(i, -SWING * t, FM_STAND + lean, TB_STAND);
      }
      for (uint8_t i = 3; i < 6; i++) {
        float lf = sinf(t * PI) * LIFT;
        setLeg(i, SWING * (1.0f - t), FM_STAND - lf, TB_STAND);
      }
      svcDelay(12);
    }
    if (!ok()) break;
    // Reset and swap
    for (uint8_t s = 0; s <= 40 && ok(); s++) {
      float t = (float)s / 40.0f;
      float lean = sinf(t * PI) * 12.0f;
      for (uint8_t i = 3; i < 6; i++) {
        setLeg(i,  SWING * t, FM_STAND - lean, TB_STAND);  // fixed: was wrong sign
      }
      for (uint8_t i = 0; i < 3; i++) {
        float lf = sinf(t * PI) * LIFT;
        setLeg(i, -SWING * (1.0f - t), FM_STAND - lf, TB_STAND);
      }
      svcDelay(12);
    }
  }
  standAll();
  setStatus("MOONWALK COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance: Breakdance Spin ══════════════════════════
// Fast continuous spinning with alternating high/low leg positions
void doDanceBreak() {
  setStatus("BREAKDANCE: WINDMILL SPIN");
  auto ok = [&]() { return isActive && curMode == DANCE_BREAK; };

  // Build up spin
  for (uint8_t r = 0; r < 6 && ok(); r++) {
    // Each step: alternating tripods at different heights
    for (uint8_t i = 0; i < 3; i++) setLeg(GA[i], (GA[i]<3?20.0f:-20.0f), FM_STAND - 20.0f, TB_STAND);
    for (uint8_t i = 0; i < 3; i++) setLeg(GB[i], (GB[i]<3?-18.0f:18.0f), FM_STAND + 6.0f, TB_STAND);
    svcDelay(80);
    for (uint8_t i = 0; i < 3; i++) setLeg(GA[i], (GA[i]<3?-18.0f:18.0f), FM_STAND + 6.0f, TB_STAND);
    for (uint8_t i = 0; i < 3; i++) setLeg(GB[i], (GB[i]<3?20.0f:-20.0f), FM_STAND - 20.0f, TB_STAND);
    svcDelay(80);
  }
  if (!ok()) return;

  // Fast spin 4 full rotations
  for (uint8_t r = 0; r < 8 && ok(); r++) turnCycle(+1);
  if (!ok()) return;

  // Freeze pose
  for (uint8_t i = 0; i < 6; i++) {
    float cx = (i == 0 || i == 3) ? 35.0f : (i == 2 || i == 5) ? -35.0f : 0.0f;
    setLeg(i, cx, FM_STAND - 25.0f + (i%2)*10.0f, TB_STAND);
  }
  svcDelay(500);
  standAll();
  setStatus("BREAKDANCE COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance: Ninja Dash ═══════════════════════════════
// Quick burst sprints with sudden stops and crouches
void doDanceNinja() {
  setStatus("NINJA DASH: SHADOW STRIKE");
  auto ok = [&]() { return isActive && curMode == DANCE_NINJA; };

  for (uint8_t n = 0; n < 4 && ok(); n++) {
    // Crouch low
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 20.0f, TB_STAND);
    svcDelay(150);

    // Explode forward — 3 fast steps
    uint16_t savedMS = STEP_MS; STEP_MS = 200;
    for (uint8_t r = 0; r < 3 && ok(); r++) walkCycle(+1 * DIR_SIGN);
    STEP_MS = savedMS;
    if (!ok()) break;

    // Sudden stop + spin
    standAll();
    svcDelay(80);
    int8_t spinDir = (n%2==0) ? +1 : -1;
    for (uint8_t r = 0; r < 1 && ok(); r++) turnCycle(spinDir);

    // Crouch hold
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 15.0f, TB_STAND);
    svcDelay(200);
  }
  standAll();
  setStatus("NINJA DASH COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Dance: Zombie Walk ══════════════════════════════
// Stiff, jerky, uneven gait — drag one group, lurch other
void doDanceZombie() {
  setStatus("ZOMBIE WALK: THE UNDEAD");
  auto ok = [&]() { return isActive && curMode == DANCE_ZOMBIE; };

  for (uint8_t rep = 0; rep < 6 && ok(); rep++) {
    // Lurch left side forward with head bob
    for (uint8_t i = 0; i < 3; i++) setLeg(i, 0.0f, FM_STAND - 22.0f, TB_STAND);
    svcDelay(90);
    for (uint8_t i = 0; i < 3; i++) setLeg(i, SWING * 0.8f, FM_STAND, TB_STAND);
    svcDelay(200);

    // Drag right side (slow, reluctant)
    for (uint8_t i = 3; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 8.0f, TB_STAND);
    svcDelay(250);
    for (uint8_t i = 3; i < 6; i++) setLeg(i, -SWING * 0.8f, FM_STAND + 5.0f, TB_STAND);
    svcDelay(350);

    // Tremble
    for (uint8_t k = 0; k < 3 && ok(); k++) {
      for (uint8_t i = 0; i < 6; i++) setLeg(i, (i%2==0?5.0f:-5.0f), FM_STAND, TB_STAND);
      svcDelay(50);
      for (uint8_t i = 0; i < 6; i++) setLeg(i, (i%2==0?-5.0f:5.0f), FM_STAND, TB_STAND);
      svcDelay(50);
    }
  }
  standAll();
  setStatus("ZOMBIE WALK COMPLETE");
  curMode = IDLE; isActive = false; danceFlag = false;
}

// ════════════════ Intimidate Display ══════════════════════════════
// Max threat display: body high, legs splayed wide, rapid stamping
void doIntimidate() {
  curMode = INTIMIDATE; isActive = true;
  setStatus("INTIMIDATION PROTOCOL");
  auto ok = [&]() { return isActive && curMode == INTIMIDATE; };

  // Rise tall
  for (uint8_t s = 0; s <= 20 && ok(); s++) {
    float t = (float)s/20.0f;
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i<3 ? 1.0f : -1.0f) * 30.0f * t;
      setLeg(i, cx, FM_STAND - 20.0f * t, TB_STAND);
    }
    svcDelay(15);
  }
  svcDelay(200);

  // Rapid stamping all legs
  for (uint8_t k = 0; k < 8 && ok(); k++) {
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i<3 ? 30.0f : -30.0f);
      setLeg(i, cx, FM_STAND - 30.0f, TB_STAND);
    }
    svcDelay(60);
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i<3 ? 30.0f : -30.0f);
      setLeg(i, cx, FM_STAND + 10.0f, TB_STAND);
    }
    svcDelay(55);
  }

  // Shrink back menacingly
  for (uint8_t s = 0; s <= 20 && ok(); s++) {
    float t = (float)s/20.0f;
    for (uint8_t i = 0; i < 6; i++) {
      float cx = (i<3 ? 1.0f : -1.0f) * 30.0f * (1.0f - t);
      setLeg(i, cx, FM_STAND - 20.0f * (1.0f - t), TB_STAND);
    }
    svcDelay(15);
  }

  standAll();
  setStatus("HEX-X1 READY");
  isActive = false;
  curMode = IDLE;
}

// ════════════════ Meditate ════════════════════════════════════════
// Slow breathing pulse — all legs rise and fall in harmony
void doMeditate() {
  curMode = MEDITATE; isActive = true;
  setStatus("MEDITATION: POWER RESERVE");
  float t = 0.0f;
  while (isActive && curMode == MEDITATE) {
    t += 0.025f;
    if (t > 2.0f * PI) t -= 2.0f * PI;
    float breath = sinf(t) * 10.0f;
    for (uint8_t i = 0; i < 6; i++) {
      setLeg(i, 0.0f, FM_STAND - breath, TB_STAND + breath * 0.4f);
    }
    svcAll();
    delay(30);
  }
  standAll();
  setStatus("HEX-X1 READY");
  isActive = false;
  curMode = IDLE;
}

// ════════════════ Salute / Pose ═══════════════════════════════════
void doPose() {
  curMode = POSE; isActive = true;
  setStatus("SALUTE: HONOR PROTOCOL");

  // Front two legs rise high in salute
  setLeg(0, 35.0f, FM_STAND - 35.0f, TB_STAND);
  setLeg(3, 35.0f, FM_STAND - 35.0f, TB_STAND);
  // Mid legs splay
  setLeg(1,  25.0f, FM_STAND, TB_STAND);
  setLeg(4, -25.0f, FM_STAND, TB_STAND);
  // Rear legs push back wide
  setLeg(2, -20.0f, FM_STAND + 10.0f, TB_STAND);
  setLeg(5, -20.0f, FM_STAND + 10.0f, TB_STAND);
  svcDelay(1200);

  standAll();
  setStatus("HEX-X1 READY");
  isActive = false;
  curMode = IDLE;
}

// ════════════════ Shake ═══════════════════════════════════════════
void doShake() {
  curMode = SHAKE; isActive = true;
  setStatus("FULL-BODY QUAKE");

  for (uint8_t k = 0; k < 14; k++) {
    float amp = 8.0f + k * 1.5f;
    for (uint8_t i = 0; i < 6; i++)
      setLeg(i, (i%2==0 ? amp : -amp), FM_STAND - amp * 0.5f, TB_STAND);
    delay(35 + k * 2);
    for (uint8_t i = 0; i < 6; i++)
      setLeg(i, (i%2==0 ? -amp : amp), FM_STAND + amp * 0.3f, TB_STAND);
    delay(35 + k * 2);
    svcAll();
  }
  standAll();
  setStatus("HEX-X1 READY");
  isActive = false;
  curMode = IDLE;
}

// ════════════════ Taunt ═══════════════════════════════════════════
void doTaunt() {
  curMode = TAUNT; isActive = true;
  setStatus("THREAT DISPLAY ACTIVE");

  setLeg(0, 25.0f, FM_STAND - 30.0f, TB_STAND);
  setLeg(3, 25.0f, FM_STAND - 30.0f, TB_STAND);
  setLeg(1, 0.0f, FM_STAND - 5.0f, TB_STAND);
  setLeg(4, 0.0f, FM_STAND - 5.0f, TB_STAND);
  setLeg(2, -15.0f, FM_STAND + 10.0f, TB_STAND);
  setLeg(5, -15.0f, FM_STAND + 10.0f, TB_STAND);
  svcDelay(350);

  for (uint8_t k = 0; k < 4; k++) {
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND - 18.0f, TB_STAND);
    svcDelay(80);
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, FM_STAND + 8.0f, TB_STAND);
    svcDelay(70);
  }

  standAll();
  svcDelay(200);
  setStatus("HEX-X1 READY");
  isActive = false;
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
      if (curMode != WAKING) {
        isActive = false;
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
    case 'Q': case 'q':
      wakeFlag = danceFlag = false;
      curMode = CRAB_L; isActive = true;
      setStatus("CRAB: WALK LEFT");
      break;
    case 'C': case 'c':
      wakeFlag = danceFlag = false;
      curMode = CRAB_R; isActive = true;
      setStatus("CRAB: WALK RIGHT");
      break;
    case '7':
      wakeFlag = danceFlag = false;
      curMode = DIAG_FL; isActive = true;
      setStatus("DIAGONAL: FRONT-LEFT");
      break;
    case '9':
      wakeFlag = danceFlag = false;
      curMode = DIAG_FR; isActive = true;
      setStatus("DIAGONAL: FRONT-RIGHT");
      break;
    case '1':
      // Could be speed OR diagonal — check context; default speed
      STEP_MS = 720; setStatus("SPEED: SLOW [720ms]"); break;
    case '3':
      STEP_MS = 300; setStatus("SPEED: FAST [300ms]"); break;
    case 'D': case 'd':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_A; isActive = true;
      setStatus("DANCE-A: RAVE STOMP");
      break;
    case 'E': case 'e':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_B; isActive = true;
      setStatus("DANCE-B: TAI CHI GLIDE");
      break;
    case 'G': case 'g':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_G; isActive = true;
      setStatus("DANCE-G: SPIDER CREEP");
      break;
    case 'H': case 'h':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_D; isActive = true;
      setStatus("DANCE-D: MECH WARRIOR");
      break;
    case 'I': case 'i':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_MOONWALK; isActive = true;
      setStatus("MOONWALK: SMOOTH CRIMINAL");
      break;
    case 'J': case 'j':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_BREAK; isActive = true;
      setStatus("BREAKDANCE: WINDMILL SPIN");
      break;
    case 'N': case 'n':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_NINJA; isActive = true;
      setStatus("NINJA DASH: SHADOW STRIKE");
      break;
    case 'Z': case 'z':
      wakeFlag = false; danceFlag = true;
      curMode = DANCE_ZOMBIE; isActive = true;
      setStatus("ZOMBIE WALK: THE UNDEAD");
      break;
    case 'T': case 't':
      wakeFlag = danceFlag = false;
      doTaunt();
      break;
    case 'V': case 'v':
      wakeFlag = danceFlag = false;
      curMode = WAVE; isActive = true;
      setStatus("WAVE: FLOW MODE");
      break;
    case 'K': case 'k':
      wakeFlag = danceFlag = false;
      doIntimidate();
      break;
    case 'M': case 'm':
      wakeFlag = danceFlag = false;
      curMode = MEDITATE; isActive = true;
      setStatus("MEDITATION: POWER RESERVE");
      break;
    case 'P': case 'p':
      wakeFlag = danceFlag = false;
      doPose();
      break;
    case 'X': case 'x':
      wakeFlag = danceFlag = false;
      doShake();
      break;
    case '2': STEP_MS = 500; setStatus("SPEED: NORMAL [500ms]"); break;
    case '+': SWING = min(SWING + 2.0f, 38.0f); setStatus("STRIDE: " + String((int)SWING) + " DEG"); break;
    case '-': SWING = max(SWING - 2.0f,  5.0f); setStatus("STRIDE: " + String((int)SWING) + " DEG"); break;
    case '\n': case '\r': case ' ': break;
    case 'S': case 's':
    default:
      isActive = wakeFlag = danceFlag = false;
      curMode = IDLE;
      standAll();
      setStatus("HALT — STANDING BY");
      break;
  }
}

void handleSerial() {
  while (Serial.available()) processCmd((char)Serial.read());
}

// ════════════════════════ Login HTML ═════════════════════════════
const char LOGIN_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<title>HEX-X1 ACCESS</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;600;700&family=Share+Tech+Mono&display=swap');
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --neon:#00ffcc;--neon2:#00aaff;--neon3:#ff3a7c;
  --bg:#050a12;--card:#0a1525;
  --border:rgba(0,255,204,0.2);
}
body{min-height:100vh;background:var(--bg);display:grid;place-items:center;padding:20px;
 font-family:'Share Tech Mono',monospace;color:var(--neon);overflow:hidden;
 background-image:radial-gradient(ellipse 80% 80% at 50% 50%,rgba(0,40,80,0.3),transparent)}
/* Animated grid */
.grid{position:fixed;inset:0;pointer-events:none;
 background-image:linear-gradient(rgba(0,255,204,0.04) 1px,transparent 1px),
                  linear-gradient(90deg,rgba(0,255,204,0.04) 1px,transparent 1px);
 background-size:50px 50px;
 mask-image:radial-gradient(ellipse 70% 70% at 50% 50%,black 40%,transparent 80%)}
/* Floating hex bg */
.hexbg{position:fixed;inset:0;pointer-events:none;overflow:hidden}
.hexbg span{position:absolute;font-size:4rem;color:rgba(0,255,204,0.04);
 animation:floatUp linear infinite}
@keyframes floatUp{from{transform:translateY(110vh) rotate(0deg)}to{transform:translateY(-10vh) rotate(180deg)}}
.card{position:relative;z-index:10;width:min(380px,100%);
 background:linear-gradient(135deg,rgba(10,21,37,0.95),rgba(5,15,30,0.95));
 border:1px solid var(--border);padding:36px 30px;
 border-radius:2px;
 box-shadow:0 0 60px rgba(0,255,204,0.06),
            0 0 0 1px rgba(0,255,204,0.04),
            inset 0 1px 0 rgba(0,255,204,0.1)}
/* Corner accents */
.card::before,.card::after{content:'';position:absolute;width:16px;height:16px;border-color:var(--neon);border-style:solid;opacity:0.6}
.card::before{top:0;left:0;border-width:2px 0 0 2px}
.card::after{bottom:0;right:0;border-width:0 2px 2px 0}
.logo-wrap{text-align:center;margin-bottom:28px}
.hex-logo{font-family:'Rajdhani',sans-serif;font-size:2.4rem;font-weight:700;
 letter-spacing:8px;text-transform:uppercase;
 background:linear-gradient(135deg,var(--neon),var(--neon2));
 -webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
 filter:drop-shadow(0 0 16px rgba(0,255,204,0.4))}
.sub{font-size:0.58rem;color:rgba(0,255,204,0.35);letter-spacing:3px;margin-top:6px}
.err{color:#ff3a7c;background:rgba(255,58,124,0.08);border:1px solid rgba(255,58,124,0.3);
 padding:10px 13px;margin-bottom:16px;font-size:0.75rem;letter-spacing:0.5px;border-radius:2px}
label{display:block;font-size:0.58rem;color:rgba(0,255,204,0.45);letter-spacing:2.5px;margin-bottom:7px}
input{width:100%;padding:13px 14px;background:rgba(0,255,204,0.03);
 border:1px solid rgba(0,255,204,0.18);color:var(--neon);
 font-family:'Share Tech Mono',monospace;font-size:0.95rem;letter-spacing:3px;
 outline:none;border-radius:2px;transition:border-color 0.2s,box-shadow 0.2s}
input:focus{border-color:rgba(0,255,204,0.5);box-shadow:0 0 16px rgba(0,255,204,0.1),inset 0 0 8px rgba(0,255,204,0.04)}
button{margin-top:16px;width:100%;padding:14px;
 background:linear-gradient(135deg,rgba(0,255,204,0.08),rgba(0,170,255,0.06));
 border:1px solid rgba(0,255,204,0.35);color:var(--neon);
 font-family:'Rajdhani',sans-serif;font-size:0.88rem;font-weight:700;
 letter-spacing:4px;cursor:pointer;text-transform:uppercase;
 border-radius:2px;transition:all 0.2s;position:relative;overflow:hidden}
button::after{content:'';position:absolute;inset:0;background:linear-gradient(135deg,rgba(0,255,204,0.12),transparent);
 transform:translateX(-100%);transition:transform 0.3s}
button:hover::after{transform:translateX(0)}
button:hover{border-color:var(--neon);box-shadow:0 0 24px rgba(0,255,204,0.2)}
.divider{height:1px;background:linear-gradient(90deg,transparent,rgba(0,255,204,0.15),transparent);margin:20px 0}
.hint{text-align:center;font-size:0.58rem;color:rgba(0,255,204,0.28);letter-spacing:1px;line-height:2}
</style></head><body>
<div class="grid"></div>
<div class="hexbg">
<span style="left:5%;animation-duration:18s;animation-delay:0s">⬡</span>
<span style="left:20%;animation-duration:22s;animation-delay:4s">⬡</span>
<span style="left:45%;animation-duration:16s;animation-delay:2s">⬡</span>
<span style="left:70%;animation-duration:24s;animation-delay:6s">⬡</span>
<span style="left:88%;animation-duration:20s;animation-delay:1s">⬡</span>
</div>
<div class="card">
 <div class="logo-wrap">
  <div class="hex-logo">HEX-X1</div>
  <div class="sub">HEXAPOD ALIEN CONTROL UNIT · v4.0</div>
 </div>
 %ERR%
 <form action="/login" method="POST">
  <label>ACCESS CODE</label>
  <input type="password" name="pw" placeholder="••••••" autocomplete="current-password" autofocus/>
  <button type="submit">⚡ INITIALIZE ACCESS</button>
 </form>
 <div class="divider"></div>
 <div class="hint">
  SSID: HEX-X1 &nbsp;·&nbsp; WIFI: hexapod123<br>
  DEFAULT CODE: alien
 </div>
</div>
</body></html>
)RAW";

// ═══════════════════════ Control UI HTML ══════════════════════════
const char CTRL_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<meta name="apple-mobile-web-app-capable" content="yes"/>
<title>HEX-X1 · COMMAND</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;500;600;700&family=Share+Tech+Mono&display=swap');

:root {
  --neon:    #00ffcc;
  --neon2:   #00aaff;
  --neon3:   #ff3a7c;
  --neon4:   #ffcc00;
  --neon5:   #cc44ff;
  --bg:      #050a12;
  --surface: rgba(10,21,37,0.92);
  --surface2:rgba(8,16,30,0.85);
  --border:  rgba(0,255,204,0.15);
  --border2: rgba(0,170,255,0.18);
  --text:    rgba(0,255,204,0.85);
  --textd:   rgba(0,255,204,0.4);
}

*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;user-select:none}
html,body{width:100%;height:100%;overflow:hidden;touch-action:none;background:var(--bg)}
body{display:flex;flex-direction:column;font-family:'Share Tech Mono',monospace;color:var(--text)}

/* ── Ambient background ── */
.bg-mesh{position:fixed;inset:0;z-index:0;pointer-events:none;
 background:
  radial-gradient(ellipse 60% 50% at 20% 50%, rgba(0,60,120,0.15), transparent 60%),
  radial-gradient(ellipse 50% 60% at 80% 50%, rgba(0,100,80,0.10), transparent 60%),
  radial-gradient(ellipse 40% 40% at 50% 10%, rgba(0,80,160,0.12), transparent 60%)}
.bg-grid{position:fixed;inset:0;z-index:0;pointer-events:none;
 background-image:linear-gradient(rgba(0,255,204,0.03) 1px,transparent 1px),
                  linear-gradient(90deg,rgba(0,255,204,0.03) 1px,transparent 1px);
 background-size:44px 44px}
.scanline{position:fixed;inset:0;z-index:1;pointer-events:none;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.08) 3px,rgba(0,0,0,0.08) 4px);
 animation:scanAnim 12s linear infinite}
@keyframes scanAnim{0%{background-position:0 0}100%{background-position:0 176px}}

/* ── Layout root ── */
.root{position:relative;z-index:10;width:100%;height:100%;
 display:grid;
 grid-template-rows:auto 1fr auto;
 grid-template-columns:1fr;
 padding:6px 8px;gap:5px;min-height:0}

/* ══════════════════ TOP BAR ══════════════════ */
.topbar{display:flex;align-items:center;gap:8px;height:36px;flex-shrink:0}
.logo{font-family:'Rajdhani',sans-serif;font-size:1.25rem;font-weight:700;letter-spacing:5px;
 background:linear-gradient(120deg,var(--neon),var(--neon2));
 -webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
 filter:drop-shadow(0 0 8px rgba(0,255,204,0.5));white-space:nowrap}
.status-wrap{flex:1;display:flex;align-items:center;gap:6px;overflow:hidden}
.status-pill{font-size:0.62rem;padding:3px 9px;border:1px solid var(--border2);
 color:var(--neon2);background:rgba(0,170,255,0.06);letter-spacing:1px;
 border-radius:2px;white-space:nowrap;flex-shrink:0;
 animation:pillBlink 1.4s step-end infinite}
@keyframes pillBlink{0%,100%{opacity:1}50%{opacity:0.5}}
.status-pill.moving{border-color:rgba(0,255,204,0.5);color:var(--neon);
 background:rgba(0,255,204,0.08);animation:none;box-shadow:0 0 8px rgba(0,255,204,0.2)}
#sts{flex:1;font-size:0.62rem;color:var(--textd);letter-spacing:1px;
 overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.hbtn{padding:5px 10px;border:1px solid var(--border);background:var(--surface);
 color:var(--textd);font-family:'Share Tech Mono',monospace;font-size:0.6rem;
 letter-spacing:1.5px;cursor:pointer;border-radius:2px;white-space:nowrap;transition:all 0.15s}
.hbtn:hover,.hbtn:active{border-color:var(--neon);color:var(--neon);background:rgba(0,255,204,0.06)}
.hbtn.danger{border-color:rgba(255,58,124,0.3);color:rgba(255,58,124,0.6)}
.hbtn.danger:hover{border-color:var(--neon3);color:var(--neon3)}

/* ══════════════════ MAIN GRID ══════════════════ */
.main{display:grid;grid-template-columns:220px 1fr 130px;gap:8px;min-height:0;overflow:hidden}

/* ══════════════════ LEFT PANEL: JOYSTICK ══════════════════ */
.joy-panel{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px}
.panel-title{font-size:0.55rem;color:var(--textd);letter-spacing:3px}

.joy-outer{position:relative;width:196px;height:196px}
/* Ring decorations */
.joy-ring{position:absolute;border-radius:50%;border:1px solid;pointer-events:none}
.joy-ring.r1{inset:0;border-color:rgba(0,255,204,0.12)}
.joy-ring.r2{inset:12px;border-color:rgba(0,255,204,0.07);border-style:dashed}
.joy-ring.r3{inset:24px;border-color:rgba(0,255,204,0.04)}

/* Direction sector indicators */
.joy-sectors{position:absolute;inset:0;border-radius:50%;overflow:hidden}
.joy-sector{position:absolute;width:50%;height:50%;transform-origin:100% 100%;
 transition:opacity 0.1s;opacity:0}
.joy-sector.active{opacity:1}
.joy-sector.N {top:0;left:0;transform:rotate(45deg);
 background:radial-gradient(at 100% 100%,rgba(0,255,204,0.15),transparent 70%)}
.joy-sector.S {top:0;left:0;transform:rotate(225deg);
 background:radial-gradient(at 100% 100%,rgba(0,255,204,0.15),transparent 70%)}
.joy-sector.W {top:0;left:0;transform:rotate(135deg);
 background:radial-gradient(at 100% 100%,rgba(0,170,255,0.15),transparent 70%)}
.joy-sector.E {top:0;left:0;transform:rotate(315deg);
 background:radial-gradient(at 100% 100%,rgba(0,170,255,0.15),transparent 70%)}
.joy-sector.NW{top:0;left:0;transform:rotate(90deg);
 background:radial-gradient(at 100% 100%,rgba(0,220,200,0.12),transparent 70%)}
.joy-sector.NE{top:0;left:0;transform:rotate(0deg);
 background:radial-gradient(at 100% 100%,rgba(0,220,200,0.12),transparent 70%)}
.joy-sector.SW{top:0;left:0;transform:rotate(180deg);
 background:radial-gradient(at 100% 100%,rgba(0,220,200,0.12),transparent 70%)}
.joy-sector.SE{top:0;left:0;transform:rotate(270deg);
 background:radial-gradient(at 100% 100%,rgba(0,220,200,0.12),transparent 70%)}

.joy-base{position:absolute;inset:0;border-radius:50%;cursor:grab;touch-action:none;
 background:radial-gradient(circle at 38% 35%,rgba(0,255,204,0.05),transparent 65%)}

/* Crosshair */
.joy-cross{position:absolute;inset:0;pointer-events:none}
.joy-cross::before,.joy-cross::after{content:'';position:absolute;background:rgba(0,255,204,0.07)}
.joy-cross::before{width:1px;height:100%;left:50%}
.joy-cross::after{height:1px;width:100%;top:50%}

/* Direction tick marks */
.joy-ticks{position:absolute;inset:0;pointer-events:none}
.tick{position:absolute;width:6px;height:1px;background:rgba(0,255,204,0.3);transform-origin:right center}

.joy-stick{width:64px;height:64px;border-radius:50%;position:absolute;left:50%;top:50%;
 transform:translate(-50%,-50%);pointer-events:none;
 background:radial-gradient(circle at 35% 30%,rgba(0,255,204,0.7) 0%,rgba(0,255,204,0.2) 50%,rgba(0,30,20,0.6) 100%);
 border:1.5px solid rgba(0,255,204,0.6);
 box-shadow:0 0 16px rgba(0,255,204,0.4),inset 0 0 10px rgba(0,0,0,0.5);
 transition:box-shadow 0.1s}
.joy-stick.active{box-shadow:0 0 28px rgba(0,255,204,0.8),inset 0 0 10px rgba(0,0,0,0.4)}

.joy-dir-label{font-size:0.62rem;color:var(--neon);letter-spacing:1.5px;height:16px;
 text-align:center;text-shadow:0 0 8px currentColor}

/* Dir arrows around joystick */
.joy-arrows{position:absolute;inset:-24px;pointer-events:none;display:grid;
 grid-template-columns:1fr auto 1fr;grid-template-rows:1fr auto 1fr}
.ja{display:flex;align-items:center;justify-content:center;
 font-size:0.65rem;color:rgba(0,255,204,0.18);transition:color 0.1s,text-shadow 0.1s}
.ja.lit{color:var(--neon);text-shadow:0 0 8px var(--neon)}

/* ══════════════════ CENTER PANEL ══════════════════ */
.center-panel{display:flex;flex-direction:column;gap:5px;overflow-y:auto;
 padding-right:2px;min-height:0}
.center-panel::-webkit-scrollbar{width:3px}
.center-panel::-webkit-scrollbar-track{background:transparent}
.center-panel::-webkit-scrollbar-thumb{background:rgba(0,255,204,0.15);border-radius:2px}

.sec-label{font-size:0.52rem;color:var(--textd);letter-spacing:3px;
 display:flex;align-items:center;gap:6px;margin-top:2px}
.sec-label::before{content:'';width:6px;height:6px;border:1px solid currentColor;
 border-radius:1px;transform:rotate(45deg);flex-shrink:0}
.sec-label::after{content:'';flex:1;height:1px;background:rgba(0,255,204,0.08)}

/* Action button */
.btn{width:100%;padding:9px 8px;background:var(--surface);
 border:1px solid var(--border);font-family:'Share Tech Mono',monospace;
 font-size:0.63rem;letter-spacing:1.5px;color:rgba(0,255,204,0.65);cursor:pointer;
 text-transform:uppercase;text-align:center;border-radius:2px;
 transition:all 0.12s;position:relative;overflow:hidden;display:block}
.btn::after{content:'';position:absolute;inset:0;
 background:linear-gradient(135deg,rgba(0,255,204,0.08),transparent);
 transform:translateX(-100%);transition:transform 0.22s}
.btn:hover::after,.btn:active::after{transform:translateX(0)}
.btn:active{transform:scale(0.97)}
.btn:hover{border-color:rgba(0,255,204,0.3);color:var(--neon)}
.btn.on{background:rgba(0,255,204,0.1);border-color:rgba(0,255,204,0.45);
 color:var(--neon);box-shadow:0 0 10px rgba(0,255,204,0.15)}

/* Color variants */
.btn-b{border-color:rgba(0,170,255,0.18);color:rgba(0,170,255,0.6)}
.btn-b:hover{border-color:rgba(0,170,255,0.45);color:var(--neon2)}
.btn-b.on{background:rgba(0,170,255,0.1);border-color:rgba(0,170,255,0.45);color:var(--neon2)}
.btn-y{border-color:rgba(255,204,0,0.18);color:rgba(255,204,0,0.6)}
.btn-y:hover{border-color:rgba(255,204,0,0.45);color:var(--neon4)}
.btn-p{border-color:rgba(204,68,255,0.18);color:rgba(204,68,255,0.6)}
.btn-p:hover{border-color:rgba(204,68,255,0.45);color:var(--neon5)}
.btn-r{border-color:rgba(255,58,124,0.18);color:rgba(255,58,124,0.6)}
.btn-r:hover{border-color:rgba(255,58,124,0.45);color:var(--neon3)}

.g2{display:grid;grid-template-columns:1fr 1fr;gap:4px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px}
.g4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:4px}

/* ══════════════════ RIGHT PANEL ══════════════════ */
.right-panel{display:flex;flex-direction:column;gap:5px}

.spd-group{display:flex;flex-direction:column;gap:3px}
.spd-btn{width:100%;padding:8px 7px;background:var(--surface2);
 border:1px solid var(--border);font-family:'Share Tech Mono',monospace;
 font-size:0.6rem;letter-spacing:1px;color:var(--textd);cursor:pointer;
 text-align:left;border-radius:2px;transition:all 0.12s;
 display:flex;align-items:center;gap:6px}
.spd-btn:hover{border-color:rgba(0,170,255,0.3);color:var(--neon2)}
.spd-btn.on{background:rgba(0,170,255,0.08);border-color:rgba(0,170,255,0.4);color:var(--neon2);
 box-shadow:0 0 8px rgba(0,170,255,0.12)}
.spd-pip{display:flex;gap:2px}
.spd-pip span{width:5px;height:5px;border-radius:50%;background:currentColor;opacity:0.4}
.spd-pip span.on{opacity:1}

.info-block{background:var(--surface2);border:1px solid var(--border);
 padding:8px;border-radius:2px;font-size:0.58rem;color:var(--textd);line-height:1.8;letter-spacing:0.5px}
.info-row{display:flex;justify-content:space-between}
.info-val{color:var(--neon2)}

/* ══════════════════ BOTTOM BAR ══════════════════ */
.bottombar{height:24px;display:flex;align-items:center;gap:10px;flex-shrink:0}
.ind{display:flex;align-items:center;gap:5px;font-size:0.56rem;color:var(--textd);letter-spacing:0.8px}
.dot{width:7px;height:7px;border-radius:50%;background:rgba(0,255,204,0.12);flex-shrink:0}
.dot.on{background:var(--neon);box-shadow:0 0 5px var(--neon);animation:dotPulse 2s ease-in-out infinite}
.dot.mov{background:var(--neon4);box-shadow:0 0 5px var(--neon4)}
.dot.err{background:var(--neon3);box-shadow:0 0 5px var(--neon3)}
@keyframes dotPulse{0%,100%{opacity:1}50%{opacity:0.5}}
.bar-fill{flex:1;height:1px;background:linear-gradient(90deg,rgba(0,255,204,0.06),transparent)}
.ver{font-size:0.52rem;color:rgba(0,255,204,0.2);letter-spacing:1px}

/* ══════════════════ MODE BADGES ══════════════════ */
.mode-badges{display:flex;gap:4px;flex-wrap:wrap}
.badge{padding:2px 7px;border:1px solid var(--border);font-size:0.54rem;
 color:var(--textd);letter-spacing:1px;border-radius:2px;transition:all 0.15s}
.badge.on{border-color:rgba(0,255,204,0.5);color:var(--neon);
 background:rgba(0,255,204,0.08);box-shadow:0 0 6px rgba(0,255,204,0.2)}

/* ══════════════════ RESPONSIVE ══════════════════ */
@media(max-width:700px){
  .main{grid-template-columns:180px 1fr;grid-template-rows:auto auto}
  .right-panel{grid-column:1/-1;flex-direction:row;gap:5px;flex-wrap:wrap}
  .spd-group{flex-direction:row;flex:1}
  .spd-btn{flex:1;justify-content:center}
  .info-block{display:none}
}
@media(max-width:520px){
  .joy-outer{width:160px;height:160px}
  .joy-stick{width:52px;height:52px}
  .main{grid-template-columns:1fr}
  .joy-panel{display:none}
}
</style></head><body>
<div class="bg-mesh"></div>
<div class="bg-grid"></div>
<div class="scanline"></div>

<div class="root">

 <!-- TOP BAR -->
 <div class="topbar">
  <div class="logo">HEX-X1</div>
  <div class="status-wrap">
   <div class="status-pill" id="modePill">BOOT</div>
   <div id="sts">INITIALIZING...</div>
  </div>
  <div class="mode-badges">
   <span class="badge" id="bIdle">IDLE</span>
   <span class="badge" id="bWalk">WALK</span>
   <span class="badge" id="bCrab">CRAB</span>
   <span class="badge" id="bTurn">TURN</span>
   <span class="badge" id="bDance">DANCE</span>
   <span class="badge" id="bWave">WAVE</span>
   <span class="badge" id="bWake">BOOT</span>
  </div>
  <button class="hbtn danger" onclick="cmd('S')">■ HALT</button>
  <button class="hbtn" onclick="location='/logout'">⇠</button>
 </div>

 <!-- MAIN -->
 <div class="main">

  <!-- LEFT: JOYSTICK -->
  <div class="joy-panel">
   <div class="panel-title">LOCOMOTION CONTROL</div>
   <div class="joy-outer">
    <!-- Sector highlights -->
    <div class="joy-sectors">
     <div class="joy-sector N"  id="sN"></div>
     <div class="joy-sector S"  id="sS"></div>
     <div class="joy-sector W"  id="sW"></div>
     <div class="joy-sector E"  id="sE"></div>
     <div class="joy-sector NW" id="sNW"></div>
     <div class="joy-sector NE" id="sNE"></div>
     <div class="joy-sector SW" id="sSW"></div>
     <div class="joy-sector SE" id="sSE"></div>
    </div>
    <div class="joy-ring r1"></div>
    <div class="joy-ring r2"></div>
    <div class="joy-ring r3"></div>
    <div class="joy-cross"></div>
    <!-- 8-point ticks placed via JS -->
    <div class="joy-ticks" id="joyTicks"></div>
    <!-- Arrow labels outside ring -->
    <div class="joy-arrows">
     <div class="ja" id="jaNW">↖</div><div class="ja" id="jaN">↑</div><div class="ja" id="jaNE">↗</div>
     <div class="ja" id="jaW">←</div><div></div>                        <div class="ja" id="jaE">→</div>
     <div class="ja" id="jaSW">↙</div><div class="ja" id="jaS">↓</div><div class="ja" id="jaSE">↘</div>
    </div>
    <div id="jb" class="joy-base">
     <div id="js" class="joy-stick"></div>
    </div>
   </div>
   <div class="joy-dir-label" id="joyLabel">● NEUTRAL</div>
  </div>

  <!-- CENTER: CONTROLS -->
  <div class="center-panel">

   <!-- Movement quick-buttons -->
   <div class="sec-label">directional</div>
   <div class="g3">
    <button class="btn btn-b" id="bFL" onclick="moveBtn('7','bFL')">↖ DIAG FL</button>
    <button class="btn"       id="bF"  onclick="moveBtn('F','bF')">↑ FORWARD</button>
    <button class="btn btn-b" id="bFR" onclick="moveBtn('9','bFR')">↗ DIAG FR</button>
   </div>
   <div class="g3">
    <button class="btn btn-b" id="bCL" onclick="moveBtn('Q','bCL')">◁ CRAB L</button>
    <button class="btn btn-r" id="bST" onclick="cmd('S')">■ STOP</button>
    <button class="btn btn-b" id="bCR" onclick="moveBtn('C','bCR')">▷ CRAB R</button>
   </div>
   <div class="g3">
    <button class="btn btn-b" id="bBL" onclick="moveBtn('1d','bBL')">↙ DIAG BL</button>
    <button class="btn"       id="bB"  onclick="moveBtn('B','bB')">↓ BACKWARD</button>
    <button class="btn btn-b" id="bBR" onclick="moveBtn('3d','bBR')">↘ DIAG BR</button>
   </div>
   <div class="g2" style="margin-top:1px">
    <button class="btn" id="bTL" onclick="moveBtn('L','bTL')">↺ TURN LEFT</button>
    <button class="btn" id="bTR" onclick="moveBtn('R','bTR')">↻ TURN RIGHT</button>
   </div>

   <!-- Dance routines -->
   <div class="sec-label">dance routines</div>
   <div class="g4">
    <button id="dA" class="btn"   onclick="toggleDance('D','dA')">⚡ RAVE</button>
    <button id="dB" class="btn"   onclick="toggleDance('E','dB')">〜 TAI CHI</button>
    <button id="dG" class="btn"   onclick="toggleDance('G','dG')">☠ SPIDER</button>
    <button id="dD" class="btn"   onclick="toggleDance('H','dD')">⚔ MECH</button>
   </div>
   <div class="g4">
    <button id="dI" class="btn btn-b" onclick="toggleDance('I','dI')">🌙 MOON</button>
    <button id="dJ" class="btn btn-b" onclick="toggleDance('J','dJ')">💫 BREAK</button>
    <button id="dN" class="btn btn-b" onclick="toggleDance('N','dN')">🥷 NINJA</button>
    <button id="dZ" class="btn btn-b" onclick="toggleDance('Z','dZ')">🧟 ZOMBIE</button>
   </div>

   <!-- Special moves -->
   <div class="sec-label">special</div>
   <div class="g4">
    <button id="bVW" class="btn btn-p" onclick="toggleMode('V','bVW')">〜〜 WAVE</button>
    <button id="bME" class="btn btn-p" onclick="toggleMode('M','bME')">◎ MEDITATE</button>
    <button class="btn btn-y" onclick="cmd('T')">⚠ TAUNT</button>
    <button class="btn btn-y" onclick="cmd('K')">👁 INTIMIDATE</button>
   </div>
   <div class="g4">
    <button class="btn" onclick="cmd('P')">🫡 SALUTE</button>
    <button class="btn" onclick="cmd('X')">〰 SHAKE</button>
    <button class="btn btn-r" onclick="cmd('W')">⚡ RE-WAKE</button>
    <button class="hbtn" onclick="poll()" style="font-size:0.58rem;color:var(--textd);border-radius:2px;padding:9px 8px">↻ SYNC</button>
   </div>

  </div>

  <!-- RIGHT: SPEED + INFO -->
  <div class="right-panel">
   <div class="sec-label" style="font-size:0.52rem;color:var(--textd);letter-spacing:3px;display:flex;align-items:center;gap:5px">
    <span style="width:6px;height:6px;border:1px solid currentColor;border-radius:1px;transform:rotate(45deg);flex-shrink:0;display:inline-block"></span>
    SPEED
    <span style="flex:1;height:1px;background:rgba(0,255,204,0.08)"></span>
   </div>
   <div class="spd-group">
    <button id="sp1" class="spd-btn" onclick="setSpeed('1','sp1')">
     <span>●</span>
     <div class="spd-pip"><span class="on"></span><span></span><span></span></div>SLOW
    </button>
    <button id="sp2" class="spd-btn on" onclick="setSpeed('2','sp2')">
     <span>●●</span>
     <div class="spd-pip"><span class="on"></span><span class="on"></span><span></span></div>NORMAL
    </button>
    <button id="sp3" class="spd-btn" onclick="setSpeed('3','sp3')">
     <span>●●●</span>
     <div class="spd-pip"><span class="on"></span><span class="on"></span><span class="on"></span></div>FAST
    </button>
   </div>

   <div class="sec-label" style="font-size:0.52rem;color:var(--textd);letter-spacing:3px;display:flex;align-items:center;gap:5px;margin-top:3px">
    <span style="width:6px;height:6px;border:1px solid currentColor;border-radius:1px;transform:rotate(45deg);flex-shrink:0;display:inline-block"></span>
    STRIDE
    <span style="flex:1;height:1px;background:rgba(0,255,204,0.08)"></span>
   </div>
   <button class="spd-btn" onclick="cmd('+')">▲ EXTEND</button>
   <button class="spd-btn" onclick="cmd('-')">▼ SHORTEN</button>

   <div class="info-block" id="infoBlock" style="margin-top:4px">
    <div class="info-row"><span>STRIDE</span><span class="info-val" id="infoSwing">18°</span></div>
    <div class="info-row"><span>STEP</span><span class="info-val" id="infoStep">500ms</span></div>
    <div class="info-row"><span>LIFT</span><span class="info-val">14°</span></div>
    <div class="info-row"><span>LEGS</span><span class="info-val">6-DOF</span></div>
   </div>
  </div>

 </div>

 <!-- BOTTOM BAR -->
 <div class="bottombar">
  <div class="ind"><div class="dot on" id="dPwr"></div>PWR</div>
  <div class="ind"><div class="dot"    id="dMov"></div>MOTION</div>
  <div class="ind"><div class="dot on" id="dNet"></div>WIFI</div>
  <div class="bar-fill"></div>
  <div class="ver">HEX-X1 v4.0 · ESP32 · 2×PCA9685</div>
 </div>

</div>

<script>
const $=id=>document.getElementById(id);
const jb=$('jb'), js=$('js');

// ── State ──
let drag=false, lastCmd='S', activeSpdBtn='sp2';
let activeDance=null, activeDanceBtnId=null;
let activeMode=null, activeModeBtnId=null;
let activeMoveBtn=null;

// ── Status helpers ──
function setSts(t){ $('sts').textContent=t; }
function setModePill(label, moving){
  const p=$('modePill');
  p.textContent=label;
  p.className='status-pill'+(moving?' moving':'');
}

// ── Badge management ──
const BADGE_MAP = {
  'F':'bWalk','B':'bWalk','L':'bTurn','R':'bTurn',
  'Q':'bCrab','C':'bCrab',
  '7':'bWalk','9':'bWalk','1d':'bWalk','3d':'bWalk',
  'D':'bDance','E':'bDance','G':'bDance','H':'bDance',
  'I':'bDance','J':'bDance','N':'bDance','Z':'bDance',
  'V':'bWave','M':'bWave',
  'W':'bWake','S':'bIdle','T':'bIdle','K':'bIdle','P':'bIdle','X':'bIdle'
};
let lastBadge='bIdle';
function activateBadge(cmd){
  $(lastBadge)?.classList.remove('on');
  const b=BADGE_MAP[cmd]||'bIdle'; $(b)?.classList.add('on'); lastBadge=b;
}

// ── Motion indicator ──
function setMotion(on){
  $('dMov').className='dot'+(on?' mov':'');
}

// ── Move direction button highlight ──
const moveBtns=['bF','bB','bTL','bTR','bCL','bCR','bFL','bFR','bBL','bBR'];
function clearMoveBtns(){
  moveBtns.forEach(id=>$(id)?.classList.remove('on'));
  activeMoveBtn=null;
}

// ── HTTP ──
async function cmd(c){
  activateBadge(c);
  const moving = !['S','T','K','P','X'].includes(c.toUpperCase());
  setMotion(moving);
  try{
    const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'});
    if(r.status===401){location='/';return;}
    const t=await r.text(); setSts(t);
    setModePill(getModeLabel(c), moving);
  }catch(e){setSts('ERR: '+e.message);$('dNet').className='dot err';}
}

function getModeLabel(c){
  const m={F:'FORWARD',B:'BACKWARD',L:'TURN L',R:'TURN R',Q:'CRAB L',C:'CRAB R',
   '7':'DIAG FL','9':'DIAG FR',S:'IDLE',W:'WAKING',
   D:'RAVE',E:'TAI CHI',G:'SPIDER',H:'MECH',I:'MOONWALK',J:'BREAK',N:'NINJA',Z:'ZOMBIE',
   V:'WAVE',M:'MEDITATE',T:'TAUNT',K:'THREAT',P:'SALUTE',X:'SHAKE'};
  return m[c.toUpperCase()]||'ACTIVE';
}

// ── Direction move buttons ──
function moveBtn(c, btnId){
  clearModeBtns();
  // Stop dances/modes
  stopDance(); stopMode();
  // Map special diagonal codes
  let realCmd = c;
  if(c==='1d') realCmd='diag_bl';  // handled server-side as backward+left
  if(c==='3d') realCmd='diag_br';
  // Highlight
  $(btnId)?.classList.add('on');
  activeMoveBtn=btnId;
  // Send actual cmd
  let sendCmd=c;
  if(c==='1d') sendCmd='1_BL';
  if(c==='3d') sendCmd='3_BR';
  // For diagonal, send composed commands
  if(c==='1d'){cmd('B');return;}
  if(c==='3d'){cmd('B');return;}
  cmd(c==='7'?'7':c==='9'?'9':c);
}

// ── Dance ──
function stopDance(){
  if(activeDanceBtnId){$(activeDanceBtnId)?.classList.remove('on');}
  activeDance=null; activeDanceBtnId=null;
}
function toggleDance(c,btnId){
  if(activeDance===c){stopDance();clearMoveBtns();cmd('S');return;}
  stopDance(); stopMode(); clearMoveBtns();
  $(btnId)?.classList.add('on');
  activeDance=c; activeDanceBtnId=btnId;
  cmd(c);
}

// ── Modes (Wave, Meditate) ──
function stopMode(){
  if(activeModeBtnId){$(activeModeBtnId)?.classList.remove('on');}
  activeMode=null; activeModeBtnId=null;
}
function toggleMode(c,btnId){
  if(activeMode===c){stopMode();clearMoveBtns();cmd('S');return;}
  stopMode(); stopDance(); clearMoveBtns();
  $(btnId)?.classList.add('on');
  activeMode=c; activeModeBtnId=btnId;
  cmd(c);
}

// ── Speed ──
function setSpeed(c,btnId){
  document.querySelectorAll('.spd-btn').forEach(b=>b.classList.remove('on'));
  $(btnId)?.classList.add('on'); activeSpdBtn=btnId; cmd(c);
}

// ── Poll ──
async function poll(){
  try{
    const r=await fetch('/status',{cache:'no-store'});
    if(r.status===401){location='/';return;}
    const t=await r.text(); setSts(t);
    $('dNet').className='dot on';
    const mov=t.match(/WALK|TURN|DANCE|WAVE|CRAB|DIAG|MOON|BREAK|NINJA|ZOMBIE|WAKING/i);
    setMotion(!!mov);
  }catch(e){$('dNet').className='dot err';}
}
setInterval(poll,3000);
poll();

// ══════════════════════════════
//       8-DIR JOYSTICK
// ══════════════════════════════
const JR=80, JDZ=20;
const SECTORS=['N','NE','E','SE','S','SW','W','NW'];
const ARROW_IDS={N:'jaN',NE:'jaNE',E:'jaE',SE:'jaSE',S:'jaS',SW:'jaSW',W:'jaW',NW:'jaNW'};
const SECTOR_CMDS={N:'F',S:'B',W:'Q',E:'C',NW:'7',NE:'9',SW:'1d',SE:'3d'};
const DIR_LABELS={N:'▲ FORWARD',S:'▼ BACKWARD',W:'◁ CRAB LEFT',E:'▷ CRAB RIGHT',
  NW:'↖ DIAG FWD-L',NE:'↗ DIAG FWD-R',SW:'↙ DIAG BACK-L',SE:'↘ DIAG BACK-R',
  IDLE:'● NEUTRAL'};

let currentSector=null;

function getJoyPoint(cx,cy){
  const rect=jb.getBoundingClientRect();
  let dx=cx-(rect.left+rect.width/2);
  let dy=cy-(rect.top+rect.height/2);
  const d=Math.hypot(dx,dy);
  if(d>JR){const k=JR/d;dx*=k;dy*=k;}
  return{dx,dy,d:Math.min(d,JR)};
}

function getSector(dx,dy,d){
  if(d<JDZ)return null;
  const angle=(Math.atan2(dy,dx)*180/Math.PI+360)%360;
  // 8 sectors, each 45°, N=up=270° in screen coords
  const sectors=['E','SE','S','SW','W','NW','N','NE'];
  const idx=Math.round(((angle+22.5)/45))%8;
  return sectors[idx];
}

function clearSectors(){
  SECTORS.forEach(s=>{
    const el=$('s'+s); if(el) el.classList.remove('active');
    const ar=$( ARROW_IDS[s]); if(ar) ar.classList.remove('lit');
  });
}

function applyJoy(dx,dy,d){
  // Move stick visually
  js.style.transform=`translate(calc(-50% + ${dx}px),calc(-50% + ${dy}px))`;
  js.classList.toggle('active', d>JDZ);

  const sector=getSector(dx,dy,d);
  clearSectors();

  if(sector){
    const el=$('s'+sector); if(el) el.classList.add('active');
    const ar=$(ARROW_IDS[sector]); if(ar) ar.classList.add('lit');
    $('joyLabel').textContent=DIR_LABELS[sector]||'● NEUTRAL';
    if(sector!==currentSector){
      currentSector=sector;
      const cmdChar=SECTOR_CMDS[sector]||'S';
      stopDance(); stopMode(); clearMoveBtns();
      // Map diagonal back-dirs to backward+turn
      if(cmdChar==='1d'){
        lastJoyCmd='B'; cmd('B');
      } else if(cmdChar==='3d'){
        lastJoyCmd='B'; cmd('B');
      } else {
        lastJoyCmd=cmdChar; cmd(cmdChar);
      }
    }
  } else {
    $('joyLabel').textContent=DIR_LABELS['IDLE'];
    if(currentSector!==null){
      currentSector=null;
      lastJoyCmd='S'; cmd('S');
    }
  }
}

let lastJoyCmd='S';

jb.addEventListener('pointerdown',e=>{
  drag=true; jb.setPointerCapture(e.pointerId);
  const{dx,dy,d}=getJoyPoint(e.clientX,e.clientY); applyJoy(dx,dy,d);
},{passive:true});
jb.addEventListener('pointermove',e=>{
  if(!drag)return;
  const{dx,dy,d}=getJoyPoint(e.clientX,e.clientY); applyJoy(dx,dy,d);
},{passive:true});
function resetJoy(){
  drag=false;
  js.style.transform='translate(-50%,-50%)';
  js.classList.remove('active');
  clearSectors();
  $('joyLabel').textContent=DIR_LABELS['IDLE'];
  if(currentSector!==null){
    currentSector=null;
    cmd('S');
  }
}
jb.addEventListener('pointerup',resetJoy);
jb.addEventListener('pointercancel',resetJoy);

// ── Keyboard support ──
document.addEventListener('keydown',e=>{
  const kmap={'ArrowUp':'F','ArrowDown':'B','ArrowLeft':'L','ArrowRight':'R',
   'a':'Q','d':'C','s':'S',' ':'S'};
  if(kmap[e.key]){ e.preventDefault(); cmd(kmap[e.key]); }
});
document.addEventListener('keyup',e=>{
  if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','a','d'].includes(e.key)){
    cmd('S');
  }
});

// ── Landscape hint ──
if(window.screen?.orientation?.lock){
  window.screen.orientation.lock('landscape').catch(()=>{});
}
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
    if (isAuth()) {
      httpServer.sendHeader("Location", "/", true);
      httpServer.send(302, "text/plain", "");
      return;
    }
    String p = FPSTR(LOGIN_HTML);
    p.replace("%ERR%", "");
    httpServer.send(200, "text/html", p);
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
    processCmd(httpServer.arg("c")[0]);
    httpServer.send(200, "text/plain", sysStatus);
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
  httpServer.on("/ncsi.txt", HTTP_GET, []() {
    httpServer.send(200, "text/plain", "Microsoft NCSI");
  });
  httpServer.onNotFound(redir);
  httpServer.begin();

  Serial.println("══════════════════════════════════════════════");
  Serial.println("  WiFi SSID : " + String(AP_SSID));
  Serial.println("  WiFi PASS : " + String(AP_PASS));
  Serial.println("  UI URL    : http://192.168.4.1");
  Serial.println("  UI PASS   : " + String(UI_PASS));
  Serial.println("  v4.0 — Auto-wake on boot enabled");
  Serial.println("══════════════════════════════════════════════");
}

// ══════════════════════════ Setup ════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  pcaL.begin(); pcaL.setPWMFreq(PWM_FREQ);
  pcaR.begin(); pcaR.setPWMFreq(PWM_FREQ);
  delay(200);

  // Start collapsed for safe power-on
  dormantAll();
  setStatus("BOOT: HARDWARE INIT OK");
  delay(200);

  setupWeb();

  // ══ AUTO-WAKE ON POWER ON ══
  // Give WiFi a moment to fully initialize, then boot the robot
  setStatus("BOOT: AUTO-WAKE IN 1s...");
  svcDelay(1000);
  wakeFlag = true;
}

// ══════════════════════════ Loop ═════════════════════════════════
void loop() {
  svcAll();
  handleSerial();

  if (wakeFlag) {
    wakeFlag = false;
    doWake();
    return;
  }

  if (danceFlag) {
    danceFlag = false;
    switch (curMode) {
      case DANCE_A:        doDanceAlpha();    break;
      case DANCE_B:        doDanceBeta();     break;
      case DANCE_G:        doDanceGamma();    break;
      case DANCE_D:        doDanceDelta();    break;
      case DANCE_MOONWALK: doDanceMoonwalk(); break;
      case DANCE_BREAK:    doDanceBreak();    break;
      case DANCE_NINJA:    doDanceNinja();    break;
      case DANCE_ZOMBIE:   doDanceZombie();   break;
      default: break;
    }
    return;
  }

  switch (curMode) {
    case WALK_F:  if (isActive) walkCycle(+1 * DIR_SIGN); break;
    case WALK_B:  if (isActive) walkCycle(-1 * DIR_SIGN); break;
    case TURN_L:  if (isActive) turnCycle(-1);             break;
    case TURN_R:  if (isActive) turnCycle(+1);             break;
    case CRAB_L:  if (isActive) crabCycle(-1);             break;
    case CRAB_R:  if (isActive) crabCycle(+1);             break;
    case DIAG_FL: if (isActive) diagCycle(+1 * DIR_SIGN, -1); break;
    case DIAG_FR: if (isActive) diagCycle(+1 * DIR_SIGN, +1); break;
    case DIAG_BL: if (isActive) diagCycle(-1 * DIR_SIGN, -1); break;
    case DIAG_BR: if (isActive) diagCycle(-1 * DIR_SIGN, +1); break;
    case WAVE:
    case MEDITATE:
      if (isActive) {
        if (curMode == WAVE) runWave();
        else doMeditate();
      }
      break;
    case IDLE:
    default:
      if (hasWoken) { standAll(); }
      svcDelay(25);
      break;
  }
}
