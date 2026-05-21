/*
 ╔════════════════════════════════════════════════════════════════════╗
 ║   HEX-X1  ·  Fault-Tolerant Hexapod Controller  ·  v3.2            ║
 ║   ESP32 + 2× PCA9685 (0x40, 0x41) + MPU6050 (0x68) + 18× Servos    ║
 ╠════════════════════════════════════════════════════════════════════╣
 ║   NEW IN v3.2                                                      ║
 ║     • MPU6050 integrated on shared I²C bus                         ║
 ║     • Complementary filter (gyro + accel fusion)                   ║
 ║     • Live telemetry page at /telemetry                            ║
 ║     • Auto-calibration on boot + manual /mpu/zero                  ║
 ║     • Streaming JSON endpoints for laptop logging                  ║
 ║                                                                    ║
 ║   PRIOR FEATURES (v3.1)                                            ║
 ║     • Write-on-change PWM cache  → zero jitter on inactive servos  ║
 ║     • Per-leg LIFT_SIGN[]        → handles right-side mirror mount ║
 ║     • I²C health scan at boot    → diagnoses missing boards        ║
 ╚════════════════════════════════════════════════════════════════════╝
*/

#include <Wire.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>
#include <math.h>

// ════════════════════════ 1. NETWORK ════════════════════════════════
const char* AP_SSID = "HEX-X1";
const char* AP_PASS = "hexapod123";
const char* UI_PASS = "alien";

const byte DNS_PORT = 53;
IPAddress  apIP(192, 168, 4, 1);
DNSServer  dnsServer;
WebServer  httpServer(80);

// ════════════════════════ 2. I²C DEVICES ════════════════════════════
Adafruit_PWMServoDriver pcaL(0x40);
Adafruit_PWMServoDriver pcaR(0x41);
MPU6050 mpu;                      // default address 0x68

static const float PWM_FREQ = 50.0f;
bool pcaL_ok = false, pcaR_ok = false, mpu_ok = false;

// MPU runtime state
float mpu_roll  = 0.0f;
float mpu_pitch = 0.0f;
float mpu_yaw   = 0.0f;          // gyro-integrated yaw (drifts without magnetometer)
float mpu_roll_offset  = 0.0f;
float mpu_pitch_offset = 0.0f;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
uint32_t mpu_last_us = 0;
const float COMP_ALPHA = 0.96f;  // complementary filter weight (gyro vs accel)

// Telemetry ring buffer (last N samples) for the live plot page
const uint16_t TELEM_LEN = 120;   // ~2.4 sec at 50Hz
float telem_roll[TELEM_LEN]  = {0};
float telem_pitch[TELEM_LEN] = {0};
uint16_t telem_idx = 0;

// ════════════════════════ 3. SERVO CALIBRATION ══════════════════════
struct ServoCal { uint16_t usP45, usN45; uint8_t brd, ch; };
const ServoCal SVO[18] = {
  {  985, 2010, 0,  0 },  //  0 FL-Coxa   (L1)
  { 1117, 1989, 0,  1 },  //  1 FL-Femur
  { 1018, 2037, 0,  2 },  //  2 FL-Tibia
  {  973, 2002, 0,  8 },  //  3 ML-Coxa   (L2)
  { 1007, 1925, 0,  9 },  //  4 ML-Femur
  {  932, 1947, 0, 10 },  //  5 ML-Tibia
  { 1016, 2137, 0, 12 },  //  6 BL-Coxa   (L3)
  {  956, 1868, 0, 13 },  //  7 BL-Femur
  {  976, 2069, 0, 14 },  //  8 BL-Tibia
  { 1038, 2032, 1,  0 },  //  9 FR-Coxa   (R1)
  { 1043, 1962, 1,  1 },  // 10 FR-Femur
  {  918, 1962, 1,  2 },  // 11 FR-Tibia
  {  956, 1983, 1,  8 },  // 12 MR-Coxa   (R2)
  { 1092, 2014, 1,  9 },  // 13 MR-Femur
  {  938, 1990, 1, 10 },  // 14 MR-Tibia
  {  930, 1998, 1, 12 },  // 15 BR-Coxa   (R3)
  { 1052, 1973, 1, 13 },  // 16 BR-Femur
  {  995, 1978, 1, 14 }   // 17 BR-Tibia
};
float trimDeg[18] = { 0 };

// ════════════════════════ 4. LEG LAYOUT ═════════════════════════════
struct Leg { uint8_t cx, fm, tb; int8_t cxDir; const char* name; };
const Leg LEG[6] = {
  { 0,  1,  2, +1, "L1" },
  { 3,  4,  5, +1, "L2" },
  { 6,  7,  8, +1, "L3" },
  { 9, 10, 11, -1, "R1" },
  {12, 13, 14, -1, "R2" },
  {15, 16, 17, -1, "R3" }
};

int8_t LIFT_SIGN[6] = { -1, -1, -1, +1, +1, +1 };

const float DIS_FM_MAG = 38.0f;
const float DIS_TB_MAG = 25.0f;
const float DIS_CX     =  0.0f;

const uint8_t GA[3] = { 0, 4, 2 };
const uint8_t GB[3] = { 3, 1, 5 };

// ════════════════════════ 5. WALK PARAMETERS ════════════════════════
float    SWING    = 28.0f;
float    LIFT     = 22.0f;
float    FM_STAND =  0.0f;
float    TB_STAND =  0.0f;
uint16_t STEP_MS  = 450;
const uint8_t ISTEPS = 40;
const int8_t  DIR_SIGN = +1;

// ════════════════════════ 6. STATE ══════════════════════════════════
enum Mode      : uint8_t { IDLE, WALK_F, WALK_B, TURN_L, TURN_R };
enum LegState  : uint8_t { LEG_ACTIVE, LEG_DISABLED };
enum LegAction : uint8_t { ACT_NONE, ACT_DISABLE, ACT_ENABLE };

Mode      curMode  = IDLE;
Mode      lastMode = IDLE;
bool      needHoldIdle = false;
bool      isActive = false;

LegState  legState[6]  = { LEG_ACTIVE, LEG_ACTIVE, LEG_ACTIVE,
                            LEG_ACTIVE, LEG_ACTIVE, LEG_ACTIVE };
LegAction legAction[6] = { ACT_NONE, ACT_NONE, ACT_NONE,
                            ACT_NONE, ACT_NONE, ACT_NONE };
bool      actionBusy   = false;

float     cxPos[6] = { 0 };
float     fmPos[6] = { 0 };
float     tbPos[6] = { 0 };

String    sysStatus = "BOOTING";
IPAddress authIP(0, 0, 0, 0);
uint32_t  authExp = 0;

uint16_t lastTicks[18];

// Forward decls
void svcAll();
void svcDelay(uint32_t ms);
void setStatus(const String& s);
void processPendingLegActions();
void executeDisableLeg(uint8_t i);
void executeEnableLeg(uint8_t i);
void readMPU();
void calibrateMPU();

// ════════════════════════ 7. SERVO LAYER ════════════════════════════
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
  if (t == lastTicks[idx]) return;
  lastTicks[idx] = t;
  if (SVO[idx].brd == 0) pcaL.setPWM(SVO[idx].ch, 0, t);
  else                    pcaR.setPWM(SVO[idx].ch, 0, t);
}
void setLeg(uint8_t i, float cx, float fm, float tb) {
  wServo(LEG[i].cx, cx * LEG[i].cxDir);
  wServo(LEG[i].fm, fm);
  wServo(LEG[i].tb, tb);
  cxPos[i] = cx; fmPos[i] = fm; tbPos[i] = tb;
}

// ════════════════════════ 8. SERVICE LAYER ══════════════════════════
void svcAll() {
  dnsServer.processNextRequest();
  httpServer.handleClient();
  readMPU();                 // ★ keep MPU updated even during gait loops
}
void svcDelay(uint32_t ms) {
  uint32_t end = millis() + ms;
  while (millis() < end) { svcAll(); delay(1); }
}
void setStatus(const String& s) { sysStatus = s; Serial.println(s); }

// ════════════════════════ 9. I²C HEALTH CHECK ═══════════════════════
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
void runI2CDiagnostics() {
  Serial.println();
  Serial.println("+--- I2C BUS DIAGNOSTIC ---+");
  pcaL_ok = i2cPing(0x40);
  pcaR_ok = i2cPing(0x41);
  bool mpu_addr = i2cPing(0x68);
  Serial.print("  Left  PCA @ 0x40: ");  Serial.println(pcaL_ok ? "FOUND" : "*** MISSING ***");
  Serial.print("  Right PCA @ 0x41: ");  Serial.println(pcaR_ok ? "FOUND" : "*** MISSING ***");
  Serial.print("  MPU6050   @ 0x68: ");  Serial.println(mpu_addr ? "FOUND" : "*** MISSING ***");
  Serial.println("  ------------------------");
  Serial.print  ("  Full bus scan: ");
  uint8_t count = 0;
  for (uint8_t a = 1; a < 127; a++) if (i2cPing(a)) count++;
  Serial.print(count); Serial.println(" device(s)");
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPing(a)) {
      Serial.print("    - 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
    }
  }
  Serial.println("+--------------------------+");
}

// ════════════════════════ 10. MPU FUSION ════════════════════════════
// Complementary filter:
//   alpha * (previous + gyro_rate*dt) + (1-alpha) * accel_tilt
// Gyro is trusted short-term (smooth), accel long-term (drift correction).
void readMPU() {
  if (!mpu_ok) return;
  uint32_t now = micros();
  if (now - mpu_last_us < 20000) return;     // 50 Hz cap
  float dt = (mpu_last_us == 0) ? 0.02f : (now - mpu_last_us) / 1e6f;
  mpu_last_us = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axg = ax / 16384.0f;
  float ayg = ay / 16384.0f;
  float azg = az / 16384.0f;
  float accel_roll  = atan2f(ayg, azg) * 180.0f / PI;
  float accel_pitch = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;

  // ±250°/s range → sensitivity 131 LSB per deg/s
  float gx_dps = (gx - gyro_bias_x) / 131.0f;
  float gy_dps = (gy - gyro_bias_y) / 131.0f;
  float gz_dps = (gz - gyro_bias_z) / 131.0f;

  mpu_roll  = COMP_ALPHA * (mpu_roll  + gx_dps * dt) + (1.0f - COMP_ALPHA) * accel_roll;
  mpu_pitch = COMP_ALPHA * (mpu_pitch + gy_dps * dt) + (1.0f - COMP_ALPHA) * accel_pitch;
  mpu_yaw  += gz_dps * dt;

  // Push offset-corrected values into the telemetry ring buffer
  telem_roll[telem_idx]  = mpu_roll  - mpu_roll_offset;
  telem_pitch[telem_idx] = mpu_pitch - mpu_pitch_offset;
  telem_idx = (telem_idx + 1) % TELEM_LEN;
}

// Sample ~1.5s of "still" data: averages accel for level reference + gyro for bias.
void calibrateMPU() {
  if (!mpu_ok) return;
  Serial.println("MPU: calibrating, hold robot still for 1.5s...");
  const int N = 100;
  float r_sum = 0, p_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float axg = ax / 16384.0f, ayg = ay / 16384.0f, azg = az / 16384.0f;
    r_sum += atan2f(ayg, azg) * 180.0f / PI;
    p_sum += atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    delay(15);
  }
  mpu_roll_offset  = r_sum / N;
  mpu_pitch_offset = p_sum / N;
  gyro_bias_x = gx_sum / N;
  gyro_bias_y = gy_sum / N;
  gyro_bias_z = gz_sum / N;
  mpu_roll = mpu_pitch = mpu_yaw = 0.0f;
  Serial.print("MPU offsets — roll: ");  Serial.print(mpu_roll_offset, 2);
  Serial.print("  pitch: ");             Serial.print(mpu_pitch_offset, 2);
  Serial.print("  gyro_bias: ");
  Serial.print(gyro_bias_x, 0); Serial.print(", ");
  Serial.print(gyro_bias_y, 0); Serial.print(", ");
  Serial.println(gyro_bias_z, 0);
}

inline float roll_deg()  { return mpu_roll  - mpu_roll_offset; }
inline float pitch_deg() { return mpu_pitch - mpu_pitch_offset; }
inline float yaw_deg()   { return mpu_yaw; }

// ════════════════════════ 11. POSES ═════════════════════════════════
void holdIdle() {
  for (uint8_t i = 0; i < 6; i++) {
    if (legState[i] == LEG_ACTIVE) setLeg(i, 0.0f, FM_STAND, TB_STAND);
  }
}
void smoothStartup() {
  for (uint8_t i = 0; i < 18; i++) lastTicks[i] = 0xFFFF;
  for (uint8_t s = 0; s <= 45; s++) {
    float t  = (float)s / 45.0f;
    float fm = -15.0f + (FM_STAND + 15.0f) * t;
    float tb = -10.0f + (TB_STAND + 10.0f) * t;
    for (uint8_t i = 0; i < 6; i++) setLeg(i, 0.0f, fm, tb);
    delay(20);
  }
}

// ════════════════════════ 12. LEG FAULT MANAGER ═════════════════════
void disableLeg(uint8_t i) {
  if (i >= 6) return;
  legAction[i] = ACT_DISABLE;
  setStatus(String("FAULT QUEUED: ") + LEG[i].name);
}
void enableLeg(uint8_t i) {
  if (i >= 6) return;
  legAction[i] = ACT_ENABLE;
  setStatus(String("RECOVER QUEUED: ") + LEG[i].name);
}
void toggleLeg(uint8_t i) {
  if (i >= 6) return;
  if (legState[i] == LEG_ACTIVE) disableLeg(i);
  else                            enableLeg(i);
}

void executeDisableLeg(uint8_t i) {
  if (legState[i] == LEG_DISABLED) return;
  float fmTgt = LIFT_SIGN[i] * DIS_FM_MAG;
  float tbTgt = LIFT_SIGN[i] * DIS_TB_MAG;
  setStatus(String("LIFTING ") + LEG[i].name);
  float cxStart = cxPos[i], fmStart = fmPos[i], tbStart = tbPos[i];
  const uint8_t STEPS = 30;
  for (uint8_t s = 0; s <= STEPS; s++) {
    float t  = (float)s / STEPS;
    float et = 0.5f - 0.5f * cosf(t * PI);
    setLeg(i,
      cxStart + (DIS_CX - cxStart) * et,
      fmStart + (fmTgt - fmStart) * et,
      tbStart + (tbTgt - tbStart) * et);
    delay(14);
  }
  legState[i] = LEG_DISABLED;
  setStatus(String("LEG ") + LEG[i].name + " DISABLED");
}
void executeEnableLeg(uint8_t i) {
  if (legState[i] == LEG_ACTIVE) return;
  setStatus(String("RESTORING ") + LEG[i].name);
  float cxStart = cxPos[i], fmStart = fmPos[i], tbStart = tbPos[i];
  const uint8_t STEPS = 30;
  for (uint8_t s = 0; s <= STEPS; s++) {
    float t  = (float)s / STEPS;
    float et = 0.5f - 0.5f * cosf(t * PI);
    setLeg(i,
      cxStart + (0.0f     - cxStart) * et,
      fmStart + (FM_STAND - fmStart) * et,
      tbStart + (TB_STAND - tbStart) * et);
    delay(14);
  }
  legState[i] = LEG_ACTIVE;
  setStatus(String("LEG ") + LEG[i].name + " ACTIVE");
}
void processPendingLegActions() {
  if (actionBusy) return;
  actionBusy = true;
  for (uint8_t i = 0; i < 6; i++) {
    LegAction a = legAction[i];
    if (a == ACT_NONE) continue;
    legAction[i] = ACT_NONE;
    if      (a == ACT_DISABLE) executeDisableLeg(i);
    else if (a == ACT_ENABLE)  executeEnableLeg(i);
  }
  actionBusy = false;
}

// ════════════════════════ 13. GAIT ENGINE ═══════════════════════════
static inline void runHalfCycle(const uint8_t* sg, const uint8_t* st,
                                const float swTgt[3], const float stTgt[3]) {
  float ss[3], ts[3];
  for (uint8_t k = 0; k < 3; k++) { ss[k] = cxPos[sg[k]]; ts[k] = cxPos[st[k]]; }
  const uint16_t sd = max((uint16_t)1, (uint16_t)(STEP_MS / ISTEPS));
  for (uint8_t s = 0; s <= ISTEPS; s++) {
    svcAll();
    processPendingLegActions();
    if (!isActive) return;
    float t  = (float)s / (float)ISTEPS;
    float lf = sinf(t * PI);
    float fs = FM_STAND - (LIFT * lf);
    for (uint8_t k = 0; k < 3; k++) {
      uint8_t li = sg[k];
      if (legState[li] != LEG_ACTIVE) continue;
      setLeg(li, ss[k] + (swTgt[k] - ss[k]) * t, fs, TB_STAND);
    }
    for (uint8_t k = 0; k < 3; k++) {
      uint8_t li = st[k];
      if (legState[li] != LEG_ACTIVE) continue;
      setLeg(li, ts[k] + (stTgt[k] - ts[k]) * t, FM_STAND, TB_STAND);
    }
    svcDelay(sd);
  }
}
void tripodStep(uint8_t swingId, int8_t dir) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;
  float swTgt[3], stTgt[3];
  for (uint8_t k = 0; k < 3; k++) { swTgt[k] = (float)dir * SWING; stTgt[k] = -(float)dir * SWING; }
  runHalfCycle(sg, st, swTgt, stTgt);
}
void tripodTurnStep(uint8_t swingId, int8_t td) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;
  const float TS = SWING * 0.75f;
  float swTgt[3], stTgt[3];
  for (uint8_t k = 0; k < 3; k++) {
    bool sL = (sg[k] < 3), gL = (st[k] < 3);
    swTgt[k] = sL ? -(float)td * TS :  (float)td * TS;
    stTgt[k] = gL ?  (float)td * TS : -(float)td * TS;
  }
  runHalfCycle(sg, st, swTgt, stTgt);
}
void walkCycle(int8_t dir) { tripodStep(0, dir); if (isActive) tripodStep(1, dir); }
void turnCycle(int8_t td)  { tripodTurnStep(0, td); if (isActive) tripodTurnStep(1, td); }

void walkForward()  { curMode = WALK_F; isActive = true; setStatus("WALK FORWARD"); }
void walkBackward() { curMode = WALK_B; isActive = true; setStatus("WALK BACKWARD"); }
void turnLeft()     { curMode = TURN_L; isActive = true; setStatus("TURN LEFT"); }
void turnRight()    { curMode = TURN_R; isActive = true; setStatus("TURN RIGHT"); }
void stopWalking()  { isActive = false; curMode = IDLE; setStatus("HALT"); }

// ════════════════════════ 14. COMMAND PROCESSOR ═════════════════════
void processCmd(char cmd) {
  switch (cmd) {
    case 'F': case 'f': turnLeft();     break;   // was walkForward
    case 'B': case 'b': turnRight();    break;   // was walkBackward
    case 'L': case 'l': walkForward();  break;   // was turnLeft
    case 'R': case 'r': walkBackward(); break;   // was turnRight

    case 'S': case 's': stopWalking();  break;
    case '1': STEP_MS = 700; setStatus("SPEED SLOW");   break;
    case '2': STEP_MS = 450; setStatus("SPEED NORMAL"); break;
    case '3': STEP_MS = 280; setStatus("SPEED FAST");   break;
    case '+': SWING = min(SWING + 2.0f, 40.0f); setStatus("STRIDE " + String((int)SWING)); break;
    case '-': SWING = max(SWING - 2.0f, 8.0f);  setStatus("STRIDE " + String((int)SWING)); break;
    case 'a': case 'A': toggleLeg(0); break;
    case 'c': case 'C': toggleLeg(1); break;
    case 'd': case 'D': toggleLeg(2); break;
    case 'e': case 'E': toggleLeg(3); break;
    case 'g': case 'G': toggleLeg(4); break;
    case 'h': case 'H': toggleLeg(5); break;
    case 'z': case 'Z': calibrateMPU(); break;
    default: break;
  }
}
void handleSerial() { while (Serial.available()) processCmd((char)Serial.read()); }

// ════════════════════════ 15. AUTH HELPERS ══════════════════════════
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

// ════════════════════════ 16. HTML — LOGIN ══════════════════════════
const char LOGIN_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>HEX-X1 ACCESS</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 60% at 20% 0%,#001825,transparent 55%),#06090f;
 display:grid;place-items:center;padding:20px;font-family:'Courier New',monospace;color:#b0ffd0}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:5;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.06) 3px,rgba(0,0,0,0.06) 4px)}
.card{width:min(370px,100%);background:rgba(0,0,0,0.65);border:1px solid rgba(0,255,136,0.38);
 border-radius:3px;padding:22px;position:relative}
.card::before,.card::after{content:'';position:absolute;width:14px;height:14px;border-color:#00ff88;border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
h1{font-size:1.35rem;color:#00ff88;letter-spacing:4px;text-shadow:0 0 14px #00ff88;margin-bottom:5px}
p{color:#3a7a5a;font-size:0.78rem;letter-spacing:1.2px;margin-bottom:18px}
.err{color:#ff7777;background:rgba(255,51,51,0.11);border:1px solid rgba(255,51,51,0.42);border-radius:2px;padding:9px 11px;margin-bottom:13px;font-size:0.8rem}
input{width:100%;padding:14px 13px;border:1px solid rgba(0,255,136,0.3);border-radius:2px;
 background:rgba(0,255,136,0.04);color:#b0ffd0;font-family:inherit;font-size:1rem;outline:none;letter-spacing:1.5px}
input:focus{border-color:#00ff88;box-shadow:0 0 10px rgba(0,255,136,0.18)}
button{margin-top:11px;width:100%;padding:14px;border:1px solid rgba(0,255,136,0.5);border-radius:2px;
 background:rgba(0,255,136,0.07);color:#00ff88;font-family:inherit;font-size:0.88rem;font-weight:700;letter-spacing:2.5px;cursor:pointer;text-transform:uppercase}
button:hover{background:rgba(0,255,136,0.17);box-shadow:0 0 14px rgba(0,255,136,0.28)}
.hint{margin-top:12px;text-align:center;font-size:0.68rem;color:#2a5a4a;letter-spacing:1px}
</style></head><body>
<div class="card">
 <h1>HEX-X1</h1>
 <p>FAULT-TOLERANT HEXAPOD · v3.2</p>
 %ERR%
 <form action="/login" method="POST">
  <input type="password" name="pw" placeholder="ACCESS CODE" autocomplete="current-password" autofocus/>
  <button type="submit">UNLOCK SYSTEMS</button>
 </form>
 <div class="hint">WIFI: HEX-X1 · PASS: hexapod123</div>
</div>
</body></html>
)RAW";

// ════════════════════════ 17. HTML — CONTROL ════════════════════════
const char CTRL_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>HEX-X1 · CONTROL</title>
<style>
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--y:#ffaa00;--bg:#06090f;
 --t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 55% at 12% -5%,#001825,transparent 52%),var(--bg);
 color:var(--t);font-family:'Courier New',monospace;display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(465px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;padding:13px;position:relative}
.card::before,.card::after{content:'';position:absolute;width:16px;height:16px;border-color:var(--g);border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
.hdr{display:flex;align-items:baseline;gap:8px;padding-bottom:8px;border-bottom:1px solid rgba(0,255,136,0.13);margin-bottom:10px}
.logo{font-size:1.3rem;color:var(--g);letter-spacing:3.5px;text-shadow:0 0 12px var(--g)}
.sub{font-size:0.6rem;color:var(--m);letter-spacing:1.5px;margin-left:auto}
.tabs{display:flex;gap:5px;margin-bottom:9px}
.tab{flex:1;padding:7px;text-align:center;font-size:0.7rem;letter-spacing:1.5px;text-decoration:none;color:var(--m);border:1px solid rgba(0,255,136,0.2);border-radius:2px;text-transform:uppercase;background:rgba(0,0,0,0.4)}
.tab.active{background:rgba(0,255,136,0.13);color:var(--g);border-color:var(--g)}
.sts{background:rgba(0,0,0,0.65);border:1px solid rgba(0,255,136,0.17);border-radius:2px;padding:7px 11px;font-size:0.74rem;color:var(--g);letter-spacing:1px;margin-bottom:10px;min-height:30px;display:flex;align-items:center;gap:7px;overflow:hidden;white-space:nowrap}
.sts::before{content:'>';animation:bl 1.2s step-end infinite;flex-shrink:0;font-size:0.65rem}
@keyframes bl{0%,100%{opacity:1}50%{opacity:0}}
.diag{background:rgba(255,170,0,0.06);border:1px solid rgba(255,170,0,0.3);border-radius:2px;padding:5px 9px;font-size:0.62rem;margin-bottom:9px;display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;color:var(--m)}
.diag b{color:var(--y);letter-spacing:1px}
.diag .ok{color:var(--g)} .diag .bad{color:var(--r);font-weight:700}
.lbl{font-size:0.59rem;color:var(--m);letter-spacing:2px;text-transform:uppercase;margin:9px 0 5px;display:flex;align-items:center;gap:6px}
.lbl::after{content:'';flex:1;height:1px;background:rgba(0,255,136,0.1)}
button{all:unset;display:block;width:100%;text-align:center;padding:11px 6px;border:1px solid rgba(0,255,136,0.26);border-radius:2px;font-family:inherit;font-size:0.78rem;font-weight:700;letter-spacing:1.5px;color:var(--t);background:rgba(0,0,0,0.55);cursor:pointer;touch-action:manipulation;transition:all 0.12s;text-transform:uppercase}
button:hover{background:rgba(0,255,136,0.09);border-color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.2)}
button:active{transform:scale(0.97)}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:5px;margin-bottom:5px}
.dpad button{padding:16px 0;font-size:0.9rem}
.dpad .blank{visibility:hidden}
.dpad button.on{background:rgba(0,255,136,0.16);border-color:var(--g);color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.32)}
.btn-stop{border-color:rgba(255,51,68,0.55)!important;color:var(--r)!important}
.btn-stop:hover{background:rgba(255,51,68,0.1)!important;box-shadow:0 0 9px rgba(255,51,68,0.3)!important}
.legbox{background:rgba(255,170,0,0.04);border:1px solid rgba(255,170,0,0.28);border-radius:3px;padding:11px;margin:8px 0 4px}
.lgttl{font-size:0.7rem;color:var(--y);letter-spacing:2.5px;text-align:center;margin-bottom:9px;text-shadow:0 0 6px rgba(255,170,0,0.5)}
.lgrid{display:grid;grid-template-columns:1fr 56px 1fr;gap:6px;align-items:center}
.lgrid > .body{display:grid;place-items:center;font-size:0.55rem;color:var(--m);border:1px dashed rgba(0,255,136,0.22);border-radius:50%;height:34px}
.lbtn{padding:11px 4px!important;line-height:1.15;font-size:0.7rem!important}
.lbtn small{display:block;font-size:0.52rem;color:var(--m);font-weight:400;letter-spacing:1px;margin-top:2px}
.lbtn.dis{background:rgba(255,51,68,0.18)!important;border-color:rgba(255,51,68,0.7)!important;color:var(--r)!important;box-shadow:0 0 9px rgba(255,51,68,0.35)!important}
.lbtn.dis small{color:rgba(255,51,68,0.7)}
.lghelp{text-align:center;font-size:0.58rem;color:var(--m);letter-spacing:1px;margin-top:8px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:5px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px}
.spd-btn.on{background:rgba(0,204,255,0.13);border-color:rgba(0,204,255,0.6);color:var(--g2);box-shadow:0 0 7px rgba(0,204,255,0.25)}
</style></head><body>
<div class="card">
 <div class="hdr">
  <div class="logo">HEX-X1</div>
  <div class="sub">CONTROL v3.2</div>
 </div>
 <div class="tabs">
  <a class="tab active" href="/">CONTROL</a>
  <a class="tab" href="/telemetry">TELEMETRY</a>
 </div>
 <div class="sts" id="sts">STANDBY</div>
 <div class="diag">
  <span><b>L-PCA:</b> <span id="diagL">-</span></span>
  <span><b>R-PCA:</b> <span id="diagR">-</span></span>
  <span><b>MPU:</b> <span id="diagM">-</span></span>
 </div>

 <div class="lbl">movement</div>
 <div class="dpad">
  <div class="blank"></div><button id="bF" onclick="mv('F','bF')">FWD</button><div class="blank"></div>
  <button id="bL" onclick="mv('L','bL')">LEFT</button>
  <button class="btn-stop" id="bS" onclick="mv('S','bS')">STOP</button>
  <button id="bR" onclick="mv('R','bR')">RIGHT</button>
  <div class="blank"></div><button id="bB" onclick="mv('B','bB')">BACK</button><div class="blank"></div>
 </div>

 <div class="legbox">
  <div class="lgttl">LEG FAULT SIMULATION</div>
  <div class="lgrid">
   <button class="lbtn" id="lg0" onclick="toggleLeg(0)">L1<small>FRONT</small></button>
   <div class="body">FRONT</div>
   <button class="lbtn" id="lg3" onclick="toggleLeg(3)">R1<small>FRONT</small></button>
   <button class="lbtn" id="lg1" onclick="toggleLeg(1)">L2<small>MIDDLE</small></button>
   <div class="body">HEX</div>
   <button class="lbtn" id="lg4" onclick="toggleLeg(4)">R2<small>MIDDLE</small></button>
   <button class="lbtn" id="lg2" onclick="toggleLeg(2)">L3<small>BACK</small></button>
   <div class="body">BACK</div>
   <button class="lbtn" id="lg5" onclick="toggleLeg(5)">R3<small>BACK</small></button>
  </div>
  <div class="lghelp">TAP TO TOGGLE - GREEN=ACTIVE - RED=DISABLED</div>
 </div>

 <div class="lbl">speed</div>
 <div class="g3">
  <button id="spd1" class="spd-btn" onclick="setSpeed('1','spd1')">Slow</button>
  <button id="spd2" class="spd-btn on" onclick="setSpeed('2','spd2')">Normal</button>
  <button id="spd3" class="spd-btn" onclick="setSpeed('3','spd3')">Fast</button>
 </div>

 <div class="lbl">stride</div>
 <div class="g2">
  <button onclick="sendCmd('+')">Stride +</button>
  <button onclick="sendCmd('-')">Stride -</button>
 </div>

 <div class="g2" style="margin-top:9px">
  <button style="font-size:0.7rem" onclick="poll()">Refresh</button>
  <button style="font-size:0.7rem" onclick="location='/logout'">Logout</button>
 </div>
</div>

<script>
const $ = id => document.getElementById(id);
let activeMv='bS', activeSpd='spd2';
async function sendCmd(c){try{const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'});if(r.status===401){location='/';return;}$('sts').textContent=await r.text();}catch(e){$('sts').textContent='ERR: '+e.message;}}
function mv(c,b){if(activeMv)$(activeMv)?.classList.remove('on');$(b)?.classList.add('on');activeMv=b;sendCmd(c);}
function setSpeed(c,b){if(activeSpd)$(activeSpd)?.classList.remove('on');$(b)?.classList.add('on');activeSpd=b;sendCmd(c);}
async function toggleLeg(i){try{const r=await fetch('/leg?i='+i,{cache:'no-store'});if(r.status===401){location='/';return;}$('sts').textContent=await r.text();setTimeout(poll,250);}catch(e){$('sts').textContent='ERR: '+e.message;}}
async function poll(){try{const r=await fetch('/state',{cache:'no-store'});if(r.status===401){location='/';return;}const j=await r.json();$('sts').textContent=j.status;for(let i=0;i<6;i++){const b=$('lg'+i);if(!b)continue;if(j.legs[i]===1)b.classList.remove('dis');else b.classList.add('dis');}$('diagL').textContent=j.pcaL?'OK':'X';$('diagR').textContent=j.pcaR?'OK':'X';$('diagM').textContent=j.mpu?'OK':'X';$('diagL').className=j.pcaL?'ok':'bad';$('diagR').className=j.pcaR?'ok':'bad';$('diagM').className=j.mpu?'ok':'bad';}catch(e){}}
poll();setInterval(poll,1800);
</script>
</body></html>
)RAW";

// ════════════════════════ 18. HTML — TELEMETRY ══════════════════════
const char TELEM_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>HEX-X1 · TELEMETRY</title>
<style>
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--y:#ffaa00;--bg:#06090f;
 --t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 55% at 12% -5%,#001825,transparent 52%),var(--bg);
 color:var(--t);font-family:'Courier New',monospace;display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(465px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;padding:13px;position:relative}
.card::before,.card::after{content:'';position:absolute;width:16px;height:16px;border-color:var(--g);border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
.hdr{display:flex;align-items:baseline;gap:8px;padding-bottom:8px;border-bottom:1px solid rgba(0,255,136,0.13);margin-bottom:10px}
.logo{font-size:1.3rem;color:var(--g);letter-spacing:3.5px;text-shadow:0 0 12px var(--g)}
.sub{font-size:0.6rem;color:var(--m);letter-spacing:1.5px;margin-left:auto}
.tabs{display:flex;gap:5px;margin-bottom:9px}
.tab{flex:1;padding:7px;text-align:center;font-size:0.7rem;letter-spacing:1.5px;text-decoration:none;color:var(--m);border:1px solid rgba(0,255,136,0.2);border-radius:2px;text-transform:uppercase;background:rgba(0,0,0,0.4)}
.tab.active{background:rgba(0,255,136,0.13);color:var(--g);border-color:var(--g)}
.lbl{font-size:0.59rem;color:var(--m);letter-spacing:2px;text-transform:uppercase;margin:9px 0 5px;display:flex;align-items:center;gap:6px}
.lbl::after{content:'';flex:1;height:1px;background:rgba(0,255,136,0.1)}
.gauges{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px}
.gauge{background:rgba(0,0,0,0.55);border:1px solid var(--br);border-radius:2px;padding:8px;text-align:center}
.gauge .lab{font-size:0.55rem;color:var(--m);letter-spacing:2px;margin-bottom:3px}
.gauge .val{font-size:1.6rem;color:var(--g);font-weight:700;text-shadow:0 0 8px rgba(0,255,136,0.4)}
.gauge .unit{font-size:0.55rem;color:var(--m);letter-spacing:1px}
.gauge.warn .val{color:var(--y);text-shadow:0 0 8px rgba(255,170,0,0.5)}
.gauge.danger .val{color:var(--r);text-shadow:0 0 8px rgba(255,51,68,0.6)}
.horizon{width:100%;height:140px;background:rgba(0,0,0,0.55);border:1px solid var(--br);border-radius:2px;position:relative;overflow:hidden;margin-bottom:10px}
.horizon canvas{width:100%;height:100%;display:block}
.plot{width:100%;height:200px;background:rgba(0,0,0,0.55);border:1px solid var(--br);border-radius:2px;margin-bottom:6px}
.plot canvas{width:100%;height:100%;display:block}
.legend{display:flex;justify-content:center;gap:14px;font-size:0.65rem;color:var(--m);margin-top:0;margin-bottom:10px}
.legend span{display:inline-flex;align-items:center;gap:5px}
.legend i{display:inline-block;width:14px;height:2px}
button{all:unset;display:block;width:100%;text-align:center;padding:11px 6px;border:1px solid rgba(0,255,136,0.26);border-radius:2px;font-family:inherit;font-size:0.78rem;font-weight:700;letter-spacing:1.5px;color:var(--t);background:rgba(0,0,0,0.55);cursor:pointer;text-transform:uppercase}
button:hover{background:rgba(0,255,136,0.09);border-color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.2)}
button:active{transform:scale(0.97)}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:5px}
.info{font-size:0.62rem;color:var(--m);padding:6px 9px;background:rgba(0,255,136,0.04);border:1px solid rgba(0,255,136,0.13);border-radius:2px;margin-top:9px;line-height:1.45}
</style></head><body>
<div class="card">
 <div class="hdr">
  <div class="logo">HEX-X1</div>
  <div class="sub">TELEMETRY v3.2</div>
 </div>
 <div class="tabs">
  <a class="tab" href="/">CONTROL</a>
  <a class="tab active" href="/telemetry">TELEMETRY</a>
 </div>

 <div class="lbl">orientation (deg)</div>
 <div class="gauges">
  <div class="gauge" id="gRoll"><div class="lab">ROLL</div><div class="val" id="vRoll">0.0</div><div class="unit">deg</div></div>
  <div class="gauge" id="gPitch"><div class="lab">PITCH</div><div class="val" id="vPitch">0.0</div><div class="unit">deg</div></div>
  <div class="gauge" id="gYaw"><div class="lab">YAW</div><div class="val" id="vYaw">0.0</div><div class="unit">deg</div></div>
 </div>

 <div class="lbl">attitude indicator</div>
 <div class="horizon"><canvas id="horizon"></canvas></div>

 <div class="lbl">history (~2.4 sec)</div>
 <div class="plot"><canvas id="plot"></canvas></div>
 <div class="legend">
  <span><i style="background:#00ff88"></i> ROLL</span>
  <span><i style="background:#00ccff"></i> PITCH</span>
 </div>

 <div class="lbl">controls</div>
 <div class="g2">
  <button onclick="zeroMPU()">Zero MPU</button>
  <button onclick="location='/'">Back</button>
 </div>

 <div class="info">
  <b>Zero MPU</b>: place the robot on a flat surface, then tap. This recalibrates
  the level reference and gyro bias. Repeat after any major hardware change.
 </div>
</div>

<script>
const $ = id => document.getElementById(id);
const horizon = $('horizon'), plot = $('plot');
const hCtx = horizon.getContext('2d'), pCtx = plot.getContext('2d');

function resize(){
  for (const c of [horizon, plot]){
    const r = c.getBoundingClientRect();
    c.width = r.width * devicePixelRatio;
    c.height = r.height * devicePixelRatio;
  }
}
window.addEventListener('resize', resize); resize();
let history = [];

function drawHorizon(roll, pitch){
  const w = horizon.width, h = horizon.height;
  hCtx.clearRect(0, 0, w, h);
  hCtx.save();
  hCtx.translate(w/2, h/2);
  hCtx.rotate(-roll * Math.PI / 180);
  const pitchOffset = pitch * 2 * devicePixelRatio;
  hCtx.fillStyle = '#1a3050'; hCtx.fillRect(-w, -h + pitchOffset, w*2, h);
  hCtx.fillStyle = '#4a3020'; hCtx.fillRect(-w, pitchOffset, w*2, h);
  hCtx.strokeStyle = '#00ff88'; hCtx.lineWidth = 2 * devicePixelRatio;
  hCtx.beginPath(); hCtx.moveTo(-w, pitchOffset); hCtx.lineTo(w, pitchOffset); hCtx.stroke();
  hCtx.strokeStyle = 'rgba(255,255,255,0.4)'; hCtx.lineWidth = 1 * devicePixelRatio;
  hCtx.font = (10 * devicePixelRatio) + "px monospace";
  hCtx.fillStyle = 'rgba(255,255,255,0.6)';
  for (let p = -30; p <= 30; p += 10){
    if (p === 0) continue;
    const y = pitchOffset - p * 2 * devicePixelRatio;
    const len = (p % 20 === 0) ? 40 : 20;
    hCtx.beginPath(); hCtx.moveTo(-len * devicePixelRatio, y); hCtx.lineTo(len * devicePixelRatio, y); hCtx.stroke();
    if (p % 20 === 0) hCtx.fillText(p + 'deg', 50 * devicePixelRatio, y + 4 * devicePixelRatio);
  }
  hCtx.restore();
  hCtx.strokeStyle = '#ffaa00'; hCtx.lineWidth = 2.5 * devicePixelRatio;
  hCtx.beginPath();
  hCtx.moveTo(w/2 - 40*devicePixelRatio, h/2); hCtx.lineTo(w/2 - 10*devicePixelRatio, h/2);
  hCtx.moveTo(w/2 + 10*devicePixelRatio, h/2); hCtx.lineTo(w/2 + 40*devicePixelRatio, h/2);
  hCtx.moveTo(w/2, h/2 - 6*devicePixelRatio); hCtx.lineTo(w/2, h/2 + 6*devicePixelRatio);
  hCtx.stroke();
}

function drawPlot(){
  const w = plot.width, h = plot.height;
  pCtx.clearRect(0, 0, w, h);
  pCtx.strokeStyle = 'rgba(0,255,136,0.08)'; pCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i++){
    const y = h * i / 4;
    pCtx.beginPath(); pCtx.moveTo(0, y); pCtx.lineTo(w, y); pCtx.stroke();
  }
  pCtx.strokeStyle = 'rgba(0,255,136,0.25)'; pCtx.lineWidth = 1;
  pCtx.beginPath(); pCtx.moveTo(0, h/2); pCtx.lineTo(w, h/2); pCtx.stroke();
  pCtx.fillStyle = 'rgba(0,255,136,0.5)'; pCtx.font = (10 * devicePixelRatio) + "px monospace";
  pCtx.fillText('0', 4 * devicePixelRatio, h/2 - 4 * devicePixelRatio);
  pCtx.fillText('+30', 4 * devicePixelRatio, 12 * devicePixelRatio);
  pCtx.fillText('-30', 4 * devicePixelRatio, h - 4 * devicePixelRatio);

  if (history.length < 2) return;
  const scale = h / 60;
  const dx = w / (history.length - 1);

  pCtx.strokeStyle = '#00ff88'; pCtx.lineWidth = 2 * devicePixelRatio;
  pCtx.beginPath();
  for (let i = 0; i < history.length; i++){
    const x = i * dx, y = h/2 - history[i].roll * scale;
    if (i === 0) pCtx.moveTo(x, y); else pCtx.lineTo(x, y);
  }
  pCtx.stroke();

  pCtx.strokeStyle = '#00ccff'; pCtx.lineWidth = 2 * devicePixelRatio;
  pCtx.beginPath();
  for (let i = 0; i < history.length; i++){
    const x = i * dx, y = h/2 - history[i].pitch * scale;
    if (i === 0) pCtx.moveTo(x, y); else pCtx.lineTo(x, y);
  }
  pCtx.stroke();
}

function setGauge(elId, val, warn=5, danger=15){
  $('v' + elId.slice(1)).textContent = val.toFixed(1);
  const g = $(elId);
  g.classList.remove('warn','danger');
  if (Math.abs(val) >= danger)      g.classList.add('danger');
  else if (Math.abs(val) >= warn)   g.classList.add('warn');
}

async function tick(){
  try{
    const r = await fetch('/telemetry.json',{cache:'no-store'});
    if (r.status === 401){ location = '/'; return; }
    const j = await r.json();
    setGauge('gRoll',  j.roll);
    setGauge('gPitch', j.pitch);
    setGauge('gYaw',   j.yaw);
    history = j.history || history;
    drawHorizon(j.roll, j.pitch);
    drawPlot();
  }catch(e){}
}

async function zeroMPU(){
  try{ await fetch('/mpu/zero',{cache:'no-store'}); }catch(e){}
  setTimeout(tick, 1800);
}

tick();
setInterval(tick, 100);
</script>
</body></html>
)RAW";

// ════════════════════════ 19. WEB ROUTES ════════════════════════════
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
  httpServer.on("/telemetry", HTTP_GET, []() {
    if (!checkAuth()) return;
    httpServer.send_P(200, "text/html", TELEM_HTML);
  });
  httpServer.on("/login", HTTP_GET, []() {
    if (isAuth()) { httpServer.sendHeader("Location", "/", true);
                    httpServer.send(302, "text/plain", ""); return; }
    String p = FPSTR(LOGIN_HTML); p.replace("%ERR%", "");
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
      p.replace("%ERR%", "<div class=\"err\">INCORRECT ACCESS CODE</div>");
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
      httpServer.send(400, "text/plain", "ERR: missing c"); return;
    }
    processCmd(httpServer.arg("c")[0]);
    httpServer.send(200, "text/plain", sysStatus);
  });
  httpServer.on("/leg", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    if (!httpServer.hasArg("i")) { httpServer.send(400, "text/plain", "ERR: missing i"); return; }
    int i = httpServer.arg("i").toInt();
    if (i < 0 || i >= 6) { httpServer.send(400, "text/plain", "ERR: invalid i"); return; }
    toggleLeg((uint8_t)i);
    httpServer.send(200, "text/plain", sysStatus);
  });
  httpServer.on("/state", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    String esc = sysStatus; esc.replace("\"", "'");
    String j = "{\"status\":\"" + esc + "\",\"legs\":[";
    for (uint8_t i = 0; i < 6; i++) {
      j += (legState[i] == LEG_ACTIVE) ? "1" : "0";
      if (i < 5) j += ",";
    }
    j += "],\"pcaL\":";  j += pcaL_ok ? "true" : "false";
    j += ",\"pcaR\":";   j += pcaR_ok ? "true" : "false";
    j += ",\"mpu\":";    j += mpu_ok  ? "true" : "false";
    j += ",\"roll\":";   j += String(roll_deg(),  2);
    j += ",\"pitch\":";  j += String(pitch_deg(), 2);
    j += ",\"yaw\":";    j += String(yaw_deg(),   2);
    j += "}";
    httpServer.send(200, "application/json", j);
  });

  // Single-shot MPU read (for laptop logging)
  httpServer.on("/mpu", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    String j = "{";
    j += "\"ok\":";     j += mpu_ok ? "true" : "false";
    j += ",\"roll\":";  j += String(roll_deg(),  3);
    j += ",\"pitch\":"; j += String(pitch_deg(), 3);
    j += ",\"yaw\":";   j += String(yaw_deg(),   3);
    j += ",\"t_ms\":";  j += String(millis());
    j += "}";
    httpServer.send(200, "application/json", j);
  });

  // Current + ring-buffer history (for live plot)
  httpServer.on("/telemetry.json", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    String j = "{";
    j += "\"roll\":";   j += String(roll_deg(),  2);
    j += ",\"pitch\":"; j += String(pitch_deg(), 2);
    j += ",\"yaw\":";   j += String(yaw_deg(),   2);
    j += ",\"mpu\":";   j += mpu_ok ? "true" : "false";
    j += ",\"history\":[";
    for (uint16_t k = 0; k < TELEM_LEN; k++) {
      uint16_t i = (telem_idx + k) % TELEM_LEN;
      j += "{\"roll\":";  j += String(telem_roll[i],  2);
      j += ",\"pitch\":"; j += String(telem_pitch[i], 2);
      j += "}";
      if (k < TELEM_LEN - 1) j += ",";
    }
    j += "]}";
    httpServer.send(200, "application/json", j);
  });

  httpServer.on("/mpu/zero", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    calibrateMPU();
    httpServer.send(200, "text/plain", "MPU zeroed");
  });

  auto redir = []() {
    httpServer.sendHeader("Location", "http://192.168.4.1/", true);
    httpServer.send(302, "text/plain", "");
  };
  httpServer.on("/generate_204",        HTTP_GET, redir);
  httpServer.on("/hotspot-detect.html", HTTP_GET, redir);
  httpServer.on("/fwlink",              HTTP_GET, redir);
  httpServer.on("/ncsi.txt",            HTTP_GET, []() {
    httpServer.send(200, "text/plain", "Microsoft NCSI");
  });
  httpServer.onNotFound(redir);
  httpServer.begin();
}

// ════════════════════════ 20. SETUP / LOOP ══════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("================================");
  Serial.println("  HEX-X1 v3.2 - Hexapod + MPU");
  Serial.println("================================");

  Wire.begin();
  Wire.setClock(400000);
  delay(50);

  runI2CDiagnostics();

  pcaL.begin(); pcaL.setPWMFreq(PWM_FREQ);
  pcaR.begin(); pcaR.setPWMFreq(PWM_FREQ);
  delay(200);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  delay(100);

// Read WHO_AM_I manually — clone chips return 0x72/0x70 instead of 0x68
  Wire.beginTransmission(0x68);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  uint8_t whoami = Wire.read();
  Serial.print("MPU6050 WHO_AM_I register: 0x");
  Serial.println(whoami, HEX);

// Accept 0x68 (genuine) or 0x72/0x70 (common clones)
  mpu_ok = (whoami == 0x68 || whoami == 0x72 || whoami == 0x70);
  Serial.print("MPU6050 status: ");
  Serial.println(mpu_ok ? "CONNECTED" : "*** NOT FOUND at 0x68 ***");
  if (mpu_ok) {
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    delay(200);
    calibrateMPU();
  }

  setupWeb();
  smoothStartup();
  lastMode = curMode;

  setStatus("READY - 6 LEGS ACTIVE");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  Walk:   F  B  L  R  S");
  Serial.println("  Speed:  1  2  3");
  Serial.println("  Stride: +  -");
  Serial.println("  Leg toggle: a=L1 c=L2 d=L3 e=R1 g=R2 h=R3");
  Serial.println("  Zero MPU:   z");
  Serial.println("--------------------------------");
  Serial.println("UI: http://192.168.4.1");
  Serial.println("  Control page:   /");
  Serial.println("  Telemetry page: /telemetry");
  Serial.println("--------------------------------");
}

void loop() {
  svcAll();
  handleSerial();
  processPendingLegActions();

  if (curMode != lastMode) {
    if (curMode == IDLE) needHoldIdle = true;
    lastMode = curMode;
  }

  switch (curMode) {
    case WALK_F: if (isActive) walkCycle(+1 * DIR_SIGN); break;
    case WALK_B: if (isActive) walkCycle(-1 * DIR_SIGN); break;
    case TURN_L: if (isActive) turnCycle(-1); break;
    case TURN_R: if (isActive) turnCycle(+1); break;
    case IDLE:
    default:
      if (needHoldIdle) { holdIdle(); needHoldIdle = false; }
      svcDelay(25);
      break;
  }
}
