/*
 ╔════════════════════════════════════════════════════════════════════╗
 ║   CHITTI 2.0  ·  Autonomous Fault-Tolerant Hexapod Controller       ║
 ║   ESP32 + 2× PCA9685 (0x40, 0x41) + MPU6050 (0x68) + 18× Servos    ║
 ║   Auto-Detection + Dynamic Gait Adjustment + Recovery Rules         ║
 ╠════════════════════════════════════════════════════════════════════╣
 ║   ADVANCED FEATURES v2.0                                           ║
 ║     • Autonomous leg fault detection via MPU signatures             ║
 ║     • Rule-based recovery offset tables (5 per leg)                 ║
 ║     • Dynamic gait adjustment (tripod → 5-leg asymmetric gait)      ║
 ║     • COG shift via coxa rotation (neighboring legs)                ║
 ║     • Auto-recovery when MPU returns to normal                      ║
 ║     • Signature calibration from live data                          ║
 ║     • Confidence scoring for fault classification                   ║
 ║     • Telemetry streaming for validation                            ║
 ║     • MLP-ready architecture (rule-based + optimization layer)      ║
 ║                                                                    ║
 ║   PHYSICS                                                          ║
 ║     Left/Right: distinguished by ROLL sign (+ = left, - = right)   ║
 ║     Front/Mid/Back: distinguished by PITCH sign                    ║
 ║     Mid-legs (L2/R2) produce larger tilt (most weight)             ║
 ║     Corner legs (L1/L3/R1/R3) produce smaller tilt                 ║
 ╚════════════════════════════════════════════════════════════════════╝
*/

#include <Wire.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>
#include <math.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ════════════════════════ 1. NETWORK ════════════════════════════════
const char* AP_SSID = "CHITTI-2.0";
const char* AP_PASS = "hexapod123";
const char* UI_PASS = "alien";

const byte DNS_PORT = 53;
IPAddress  apIP(192, 168, 4, 1);
DNSServer  dnsServer;
WebServer  httpServer(80);

// ════════════════════════ 2. I²C DEVICES ════════════════════════════
Adafruit_PWMServoDriver pcaL(0x40);
Adafruit_PWMServoDriver pcaR(0x41);
MPU6050 mpu;

static const float PWM_FREQ = 50.0f;
bool pcaL_ok = false, pcaR_ok = false, mpu_ok = false;

// MPU runtime state
float mpu_roll  = 0.0f;
float mpu_pitch = 0.0f;
float mpu_yaw   = 0.0f;
float mpu_roll_offset  = 0.0f;
float mpu_pitch_offset = 0.0f;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
uint32_t mpu_last_us = 0;
const float COMP_ALPHA = 0.96f;

// Telemetry ring buffer
const uint16_t TELEM_LEN = 120;
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
  { 0,  1,  2, +1, "L1" },  // Front-left
  { 3,  4,  5, +1, "L2" },  // Mid-left
  { 6,  7,  8, +1, "L3" },  // Back-left
  { 9, 10, 11, -1, "R1" },  // Front-right
  {12, 13, 14, -1, "R2" },  // Mid-right
  {15, 16, 17, -1, "R3" }   // Back-right
};

int8_t LIFT_SIGN[6] = { -1, -1, -1, +1, +1, +1 };

const float DIS_FM_MAG = 38.0f;
const float DIS_TB_MAG = 25.0f;
const float DIS_CX     =  0.0f;

// Normal tripod gait groups
const uint8_t GA[3] = { 0, 4, 2 };  // L1, R2, L3
const uint8_t GB[3] = { 3, 1, 5 };  // R1, L2, R3

// ════════════════════════ 5. FAULT DETECTION & RECOVERY ══════════════
enum FaultState { FAULT_NONE, FAULT_DETECTED, FAULT_RECOVERING };

struct LegSignature {
  float roll_mean, roll_std;
  float pitch_mean, pitch_std;
  uint16_t count;
};

// Signature lookup table (populated from real captures)
LegSignature sig[6] = {
  { 8.0f, 1.5f,  5.0f, 1.2f, 0 },   // L1: front-left ++
  { 15.0f, 2.0f,  0.5f, 1.5f, 0 },  // L2: mid-left +≈0
  { 8.5f, 1.4f, -4.8f, 1.3f, 0 },   // L3: back-left +-
  { -8.2f, 1.5f,  4.9f, 1.2f, 0 },  // R1: front-right -+
  { -15.5f, 2.1f,  0.3f, 1.6f, 0 }, // R2: mid-right -≈0
  { -8.6f, 1.3f, -5.1f, 1.4f, 0 }   // R3: back-right --
};

// Recovery offset table: [coxa_offset_L1, L2, L3, R1, R2, R3, femur_bias, tibia_bias, stride_scale, turn_bias]
// These are initial values from physics; MLP will refine during operation
struct RecoveryRule {
  float cx_offset[6];      // Coxa position adjustments for COG shift
  float fm_bias;           // Femur bias for stance adjustment
  float tb_bias;           // Tibia bias for stance adjustment
  float stride_scale;      // Stride length modifier (0.6 - 1.0 for failed leg compensation)
  float turn_bias;         // Turn bias to keep robot straight
};

// Recovery rules for each failed leg
RecoveryRule recovery[6] = {
  // L1 fails (front-left): body tilts forward-left
  // Compensate: R1 rotates inward, L2 rotates forward, L3 shifts back
  { { 0.0f, +8.0f, +12.0f, -5.0f, +10.0f, -6.0f }, +2.5f, +1.8f, 0.75f, +3.5f },
  
  // L2 fails (mid-left): body tilts left (largest effect)
  // Compensate: L1/L3 shift inward, R1/R2 push outward
  { { +6.0f, 0.0f, +7.0f, -8.0f, +12.0f, -9.0f }, +3.0f, +2.2f, 0.80f, +5.0f },
  
  // L3 fails (back-left): body tilts back-left
  // Compensate: L1 forward, L2 forward, R3 inward
  { { +10.0f, +6.0f, 0.0f, -4.0f, +8.0f, -10.0f }, +2.0f, +1.5f, 0.78f, +4.0f },
  
  // R1 fails (front-right): body tilts forward-right
  // Compensate: L1 rotates inward, R2 rotates forward, R3 shifts back
  { { -5.0f, +10.0f, -6.0f, 0.0f, +8.0f, +12.0f }, +2.5f, +1.8f, 0.75f, -3.5f },
  
  // R2 fails (mid-right): body tilts right (largest effect)
  // Compensate: R1/R3 shift inward, L1/L2 push outward
  { { -8.0f, +12.0f, -9.0f, +6.0f, 0.0f, +7.0f }, +3.0f, +2.2f, 0.80f, -5.0f },
  
  // R3 fails (back-right): body tilts back-right
  // Compensate: R1 forward, R2 forward, L3 inward
  { { -4.0f, +8.0f, -10.0f, +10.0f, +6.0f, 0.0f }, +2.0f, +1.5f, 0.78f, -4.0f }
};

FaultState faultState = FAULT_NONE;
uint8_t faultedLeg = 255;  // 255 = no fault
uint32_t faultDetectedTime = 0;
float faultConfidence = 0.0f;
const float FAULT_CONFIDENCE_THRESH = 0.72f;
const uint32_t FAULT_HYSTERESIS_MS = 500;  // Require fault signature for 500ms before triggering

// ════════════════════════ 6. STATE ══════════════════════════════════
enum Mode      : uint8_t { IDLE, WALK_F, WALK_B, TURN_L, TURN_R };
enum LegState  : uint8_t { LEG_ACTIVE, LEG_DISABLED };
enum LegAction : uint8_t { ACT_NONE, ACT_DISABLE, ACT_ENABLE };

Mode      curMode  = IDLE;
Mode      lastMode = IDLE;
bool      needHoldIdle = false;
bool      isActive = false;
bool      autoRecoveryEnabled = true;

LegState  legState[6]  = { LEG_ACTIVE, LEG_ACTIVE, LEG_ACTIVE,
                            LEG_ACTIVE, LEG_ACTIVE, LEG_ACTIVE };
LegAction legAction[6] = { ACT_NONE, ACT_NONE, ACT_NONE,
                            ACT_NONE, ACT_NONE, ACT_NONE };
bool      actionBusy   = false;

float     cxPos[6] = { 0 };
float     fmPos[6] = { 0 };
float     tbPos[6] = { 0 };
float     cxRecoveryOffset[6] = { 0 };  // Dynamic offsets during recovery

String    sysStatus = "BOOTING";
IPAddress authIP(0, 0, 0, 0);
uint32_t  authExp = 0;

uint16_t lastTicks[18];

// Walk parameters (dynamic)
float    SWING    = 28.0f;
float    LIFT     = 22.0f;
float    FM_STAND =  0.0f;
float    TB_STAND =  0.0f;
uint16_t STEP_MS  = 450;
const uint8_t ISTEPS = 40;
const int8_t  DIR_SIGN = +1;

// Forward decls
void svcAll();
void svcDelay(uint32_t ms);
void setStatus(const String& s);
void processPendingLegActions();
void executeDisableLeg(uint8_t i);
void executeEnableLeg(uint8_t i);
void readMPU();
void calibrateMPU();
void updateFaultDetection();
void applyRecoveryOffsets(uint8_t leg);
void clearRecoveryOffsets();

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
  readMPU();
  updateFaultDetection();  // ★ Continuous fault monitoring
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
  uint8_t count = 0;
  for (uint8_t a = 1; a < 127; a++) if (i2cPing(a)) count++;
  Serial.print  ("  Full bus scan: ");
  Serial.print(count); Serial.println(" device(s)");
  Serial.println("+--------------------------+");
}

// ════════════════════════ 10. MPU FUSION ════════════════════════════
void readMPU() {
  if (!mpu_ok) return;
  uint32_t now = micros();
  if (now - mpu_last_us < 20000) return;
  float dt = (mpu_last_us == 0) ? 0.02f : (now - mpu_last_us) / 1e6f;
  mpu_last_us = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axg = ax / 16384.0f;
  float ayg = ay / 16384.0f;
  float azg = az / 16384.0f;
  float accel_roll  = atan2f(ayg, azg) * 180.0f / PI;
  float accel_pitch = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;

  float gx_dps = (gx - gyro_bias_x) / 131.0f;
  float gy_dps = (gy - gyro_bias_y) / 131.0f;
  float gz_dps = (gz - gyro_bias_z) / 131.0f;

  mpu_roll  = COMP_ALPHA * (mpu_roll  + gx_dps * dt) + (1.0f - COMP_ALPHA) * accel_roll;
  mpu_pitch = COMP_ALPHA * (mpu_pitch + gy_dps * dt) + (1.0f - COMP_ALPHA) * accel_pitch;
  mpu_yaw  += gz_dps * dt;

  telem_roll[telem_idx]  = mpu_roll  - mpu_roll_offset;
  telem_pitch[telem_idx] = mpu_pitch - mpu_pitch_offset;
  telem_idx = (telem_idx + 1) % TELEM_LEN;
}

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
  Serial.println();
}

inline float roll_deg()  { return mpu_roll  - mpu_roll_offset; }
inline float pitch_deg() { return mpu_pitch - mpu_pitch_offset; }
inline float yaw_deg()   { return mpu_yaw; }

// ════════════════════════ 11. FAULT DETECTION ════════════════════════
// Identify which leg failed based on (roll, pitch) signature
uint8_t identifyLeg(float roll, float pitch, float& confidence) {
  uint8_t bestLeg = 255;
  float bestScore = 0.0f;
  
  for (uint8_t leg = 0; leg < 6; leg++) {
    // Mahalanobis distance (simplified): how many std-devs away?
    float droll = fabsf(roll - sig[leg].roll_mean) / (sig[leg].roll_std + 0.5f);
    float dpitch = fabsf(pitch - sig[leg].pitch_mean) / (sig[leg].pitch_std + 0.5f);
    float dist = sqrtf(droll*droll + dpitch*dpitch);
    
    // Confidence = gaussian-like penalty (closer = higher confidence)
    float score = expf(-dist * dist / 2.0f);
    
    if (score > bestScore) {
      bestScore = score;
      bestLeg = leg;
    }
  }
  
  confidence = bestScore;
  return bestLeg;
}

// Main fault detection loop (runs in svcAll)
void updateFaultDetection() {
  if (!autoRecoveryEnabled || !mpu_ok || faultState == FAULT_RECOVERING) return;
  
  float roll = roll_deg();
  float pitch = pitch_deg();
  
  // Threshold: any significant tilt suggests a fault
  float tiltMagnitude = sqrtf(roll*roll + pitch*pitch);
  const float TILT_THRESH = 3.5f;  // degrees
  
  if (tiltMagnitude < TILT_THRESH) {
    // No significant tilt detected
    if (faultState == FAULT_DETECTED) {
      // Recovery phase: check if robot returned to normal
      if (millis() - faultDetectedTime > 3000) {  // 3 sec hysteresis
        Serial.println(">>> FAULT RESOLVED - AUTO-RECOVERY CLEARING");
        faultState = FAULT_NONE;
        faultedLeg = 255;
        clearRecoveryOffsets();
        setStatus("FAULT CLEARED - NORMAL GAIT RESUMED");
      }
    }
    return;
  }
  
  // Tilt detected; try to identify which leg
  float conf = 0.0f;
  uint8_t leg = identifyLeg(roll, pitch, conf);
  
  if (leg == 255 || conf < FAULT_CONFIDENCE_THRESH) {
    // Low confidence; keep monitoring
    return;
  }
  
  if (faultState == FAULT_NONE) {
    // First time seeing this leg fault
    faultState = FAULT_DETECTED;
    faultedLeg = leg;
    faultDetectedTime = millis();
    faultConfidence = conf;
    Serial.print(">>> FAULT DETECTED: LEG ");
    Serial.print(LEG[leg].name);
    Serial.print(" (confidence: ");
    Serial.print(conf, 2);
    Serial.println(")");
  } else if (faultState == FAULT_DETECTED && faultedLeg == leg) {
    // Still seeing the same leg fault
    if (millis() - faultDetectedTime > FAULT_HYSTERESIS_MS) {
      // Hysteresis time elapsed; trigger auto-recovery
      Serial.print(">>> AUTO-RECOVERY TRIGGERED FOR ");
      Serial.println(LEG[leg].name);
      faultState = FAULT_RECOVERING;
      applyRecoveryOffsets(leg);
      setStatus(String("AUTO-RECOVERY: ") + LEG[leg].name);
    }
  } else if (faultedLeg != leg) {
    // Signature changed; reset detection (possible transient)
    faultState = FAULT_DETECTED;
    faultedLeg = leg;
    faultDetectedTime = millis();
    faultConfidence = conf;
  }
}

// Apply recovery: coxa offsets + gait adjustments
void applyRecoveryOffsets(uint8_t leg) {
  if (leg >= 6) return;
  
  RecoveryRule& rule = recovery[leg];
  
  // Store recovery offsets (applied during gait)
  for (uint8_t i = 0; i < 6; i++) {
    cxRecoveryOffset[i] = rule.cx_offset[i];
  }
  
  // Adjust global walk parameters for asymmetric gait
  SWING = 24.0f * rule.stride_scale;  // Shorter strides for failed leg side
  LIFT = 20.0f;
  FM_STAND = rule.fm_bias;
  TB_STAND = rule.tb_bias;
  STEP_MS = 520;  // Slower to maintain stability
  
  Serial.print("Recovery: SWING=");  Serial.print(SWING, 1);
  Serial.print(" LIFT="); Serial.print(LIFT, 1);
  Serial.print(" FM_STAND="); Serial.print(FM_STAND, 2);
  Serial.println();
}

void clearRecoveryOffsets() {
  for (uint8_t i = 0; i < 6; i++) {
    cxRecoveryOffset[i] = 0.0f;
  }
  SWING = 28.0f;
  LIFT = 22.0f;
  FM_STAND = 0.0f;
  TB_STAND = 0.0f;
  STEP_MS = 450;
}

// ════════════════════════ 12. POSES ═════════════════════════════════
void holdIdle() {
  for (uint8_t i = 0; i < 6; i++) {
    if (legState[i] == LEG_ACTIVE) {
      float cx = cxRecoveryOffset[i];
      setLeg(i, cx, FM_STAND, TB_STAND);
    }
  }
}
void smoothStartup() {
  for (uint8_t i = 0; i < 18; i++) lastTicks[i] = 0xFFFF;
  for (uint8_t s = 0; s <= 45; s++) {
    float t  = (float)s / 45.0f;
    float fm = -15.0f + (FM_STAND + 15.0f) * t;
    float tb = -10.0f + (TB_STAND + 10.0f) * t;
    for (uint8_t i = 0; i < 6; i++) {
      float cx = cxRecoveryOffset[i];
      setLeg(i, cx, fm, tb);
    }
    delay(20);
  }
}

// ════════════════════════ 13. LEG FAULT MANAGER ════════════════════════
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

// ════════════════════════ 14. GAIT ENGINE ════════════════════════════
static inline void runHalfCycle(const uint8_t* sg, const uint8_t* st,
                                const float swTgt[3], const float stTgt[3]) {
  float ss[3], ts[3];
  for (uint8_t k = 0; k < 3; k++) { 
    ss[k] = cxPos[sg[k]] + cxRecoveryOffset[sg[k]]; 
    ts[k] = cxPos[st[k]] + cxRecoveryOffset[st[k]]; 
  }
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
      float cx_offset = cxRecoveryOffset[li];
      setLeg(li, cx_offset + ss[k] + (swTgt[k] - ss[k]) * t, fs, TB_STAND);
    }
    
    for (uint8_t k = 0; k < 3; k++) {
      uint8_t li = st[k];
      if (legState[li] != LEG_ACTIVE) continue;
      float cx_offset = cxRecoveryOffset[li];
      setLeg(li, cx_offset + ts[k] + (stTgt[k] - ts[k]) * t, FM_STAND, TB_STAND);
    }
    
    svcDelay(sd);
  }
}

void tripodStep(uint8_t swingId, int8_t dir) {
  const uint8_t* sg = (swingId == 0) ? GA : GB;
  const uint8_t* st = (swingId == 0) ? GB : GA;
  float swTgt[3], stTgt[3];
  for (uint8_t k = 0; k < 3; k++) { 
    swTgt[k] = (float)dir * SWING; 
    stTgt[k] = -(float)dir * SWING; 
  }
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

void walkCycle(int8_t dir) { 
  tripodStep(0, dir); 
  if (isActive) tripodStep(1, dir); 
}
void turnCycle(int8_t td)  { 
  tripodTurnStep(0, td); 
  if (isActive) tripodTurnStep(1, td); 
}

void walkForward()  { curMode = WALK_F; isActive = true; setStatus("WALK FORWARD"); }
void walkBackward() { curMode = WALK_B; isActive = true; setStatus("WALK BACKWARD"); }
void turnLeft()     { curMode = TURN_L; isActive = true; setStatus("TURN LEFT"); }
void turnRight()    { curMode = TURN_R; isActive = true; setStatus("TURN RIGHT"); }
void stopWalking()  { isActive = false; curMode = IDLE; setStatus("HALT"); }

// ════════════════════════ 15. COMMAND PROCESSOR ═════════════════════
void processCmd(char cmd) {
  switch (cmd) {
    case 'F': case 'f': walkForward();  break;
    case 'B': case 'b': walkBackward(); break;
    case 'L': case 'l': turnLeft();     break;
    case 'R': case 'r': turnRight();    break;
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
    case 'x': case 'X': autoRecoveryEnabled = !autoRecoveryEnabled; 
                        setStatus(autoRecoveryEnabled ? "AUTO-RECOVERY ON" : "AUTO-RECOVERY OFF");
                        break;
    default: break;
  }
}
void handleSerial() { while (Serial.available()) processCmd((char)Serial.read()); }

// ════════════════════════ 16. AUTH HELPERS ══════════════════════════
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

// ════════════════════════ 17. HTML — LOGIN ══════════════════════════
const char LOGIN_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>CHITTI 2.0 ACCESS</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 60% at 20% 0%,#001a33,transparent 55%),#06090f;
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
 <h1>CHITTI 2.0</h1>
 <p>AUTONOMOUS FAULT-TOLERANT HEXAPOD</p>
 %ERR%
 <form action="/login" method="POST">
  <input type="password" name="pw" placeholder="ACCESS CODE" autocomplete="current-password" autofocus/>
  <button type="submit">UNLOCK SYSTEMS</button>
 </form>
 <div class="hint">WIFI: CHITTI-2.0 · PASS: hexapod123</div>
</div>
</body></html>
)RAW";

// ════════════════════════ 18. HTML — CONTROL ════════════════════════
const char CTRL_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>CHITTI 2.0 · CONTROL</title>
<style>
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--y:#ffaa00;--bg:#06090f;
 --t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 55% at 12% -5%,#001a33,transparent 52%),var(--bg);
 color:var(--t);font-family:'Courier New',monospace;display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(500px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;padding:13px;position:relative}
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
.fault-box{background:rgba(255,51,68,0.08);border:2px solid rgba(255,51,68,0.5);border-radius:2px;padding:7px 9px;margin-bottom:9px;font-size:0.7rem;color:var(--r);letter-spacing:1px}
.fault-box.active{background:rgba(255,51,68,0.15);border-color:var(--r)}
.fault-box b{color:var(--r);font-weight:700}
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
  <div class="logo">CHITTI 2.0</div>
  <div class="sub">AUTO-RECOVERY v2.0</div>
 </div>
 <div class="tabs">
  <a class="tab active" href="/">CONTROL</a>
  <a class="tab" href="/telemetry">TELEMETRY</a>
  <a class="tab" href="/config">CONFIG·AI</a>
 </div>
 <div class="sts" id="sts">STANDBY</div>
 
 <div class="fault-box" id="faultBox" style="display:none">
  <b>⚠ FAULT DETECTED:</b> <span id="faultText">-</span> (confidence: <span id="faultConf">-</span>%)
 </div>
 
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
  <div class="lgttl">LEG SIMULATION / CONTROL</div>
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

 <div class="lbl">auto-recovery</div>
 <div class="g2">
  <button onclick="toggleAutoRecovery()" id="autoRecBtn">Auto-Recovery: ON</button>
  <button style="font-size:0.7rem" onclick="location='/telemetry'">Telemetry</button>
 </div>

 <div class="g2" style="margin-top:9px">
  <button style="font-size:0.7rem" onclick="poll()">Refresh</button>
  <button style="font-size:0.7rem" onclick="location='/logout'">Logout</button>
 </div>
</div>

<script>
const $ = id => document.getElementById(id);
let activeMv='bS', activeSpd='spd2';
let autoRecoveryOn = true;
async function sendCmd(c){try{const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'});if(r.status===401){location='/';return;}$('sts').textContent=await r.text();}catch(e){$('sts').textContent='ERR: '+e.message;}}
function mv(c,b){if(activeMv)$(activeMv)?.classList.remove('on');$(b)?.classList.add('on');activeMv=b;sendCmd(c);}
function setSpeed(c,b){if(activeSpd)$(activeSpd)?.classList.remove('on');$(b)?.classList.add('on');activeSpd=b;sendCmd(c);}
async function toggleLeg(i){try{const r=await fetch('/leg?i='+i,{cache:'no-store'});if(r.status===401){location='/';return;}$('sts').textContent=await r.text();setTimeout(poll,250);}catch(e){$('sts').textContent='ERR: '+e.message;}}
function toggleAutoRecovery(){sendCmd('x');}
async function poll(){try{const r=await fetch('/state',{cache:'no-store'});if(r.status===401){location='/';return;}const j=await r.json();$('sts').textContent=j.status;for(let i=0;i<6;i++){const b=$('lg'+i);if(!b)continue;if(j.legs[i]===1)b.classList.remove('dis');else b.classList.add('dis');}$('diagL').textContent=j.pcaL?'OK':'X';$('diagR').textContent=j.pcaR?'OK':'X';$('diagM').textContent=j.mpu?'OK':'X';$('diagL').className=j.pcaL?'ok':'bad';$('diagR').className=j.pcaR?'ok':'bad';$('diagM').className=j.mpu?'ok':'bad';if(j.fault>=0){$('faultBox').style.display='block';const legNames=['L1','L2','L3','R1','R2','R3'];$('faultText').textContent=legNames[j.fault];$('faultConf').textContent=(j.faultConf*100).toFixed(0);}else{$('faultBox').style.display='none';}$('autoRecBtn').textContent=j.autoRecovery?'Auto-Recovery: ON':'Auto-Recovery: OFF';}catch(e){}}
poll();setInterval(poll,1800);
</script>
</body></html>
)RAW";

// ════════════════════════ 19. HTML — TELEMETRY ══════════════════════
const char TELEM_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>CHITTI 2.0 · TELEMETRY</title>
<style>
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--y:#ffaa00;--bg:#06090f;
 --t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 55% at 12% -5%,#001a33,transparent 52%),var(--bg);
 color:var(--t);font-family:'Courier New',monospace;display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(500px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;padding:13px;position:relative}
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
  <div class="logo">CHITTI 2.0</div>
  <div class="sub">TELEMETRY v2.0</div>
 </div>
 <div class="tabs">
  <a class="tab" href="/">CONTROL</a>
  <a class="tab active" href="/telemetry">TELEMETRY</a>
  <a class="tab" href="/config">CONFIG·AI</a>
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
  the level reference and gyro bias. The fault detection signatures use this zero as reference.
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
  hCtx.fillStyle = '#1a3a50'; hCtx.fillRect(-w, -h + pitchOffset, w*2, h);
  hCtx.fillStyle = '#4a3a20'; hCtx.fillRect(-w, pitchOffset, w*2, h);
  hCtx.strokeStyle = '#00ff88'; hCtx.lineWidth = 2 * devicePixelRatio;
  hCtx.beginPath(); hCtx.moveTo(-w, pitchOffset); hCtx.lineTo(w, pitchOffset); hCtx.stroke();
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

// ════════════════════════ 19. HTML — CONFIG·AI ══════════════════════════
const char CONFIG_HTML[] PROGMEM = R"RAW(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>CHITTI 2.0 · CONFIG·AI</title>
<style>
:root{--g:#00ff88;--g2:#00ccff;--r:#ff3344;--y:#ffaa00;--b:#5588ff;--bg:#06090f;
 --t:#c8ffe8;--m:#3a7a5a;--br:rgba(0,255,136,0.3)}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:radial-gradient(ellipse 130% 55% at 12% -5%,#001a33,transparent 52%),var(--bg);
 color:var(--t);font-family:'Courier New',monospace;display:grid;place-items:start center;padding:9px 7px 30px}
body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:100;
 background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.05) 3px,rgba(0,0,0,0.05) 4px)}
.card{width:min(1000px,100%);background:rgba(0,0,0,0.6);border:1px solid var(--br);border-radius:3px;padding:13px;position:relative}
.card::before,.card::after{content:'';position:absolute;width:16px;height:16px;border-color:var(--g);border-style:solid}
.card::before{top:5px;left:5px;border-width:1px 0 0 1px}
.card::after{bottom:5px;right:5px;border-width:0 1px 1px 0}
.hdr{display:flex;align-items:baseline;gap:8px;padding-bottom:8px;border-bottom:1px solid rgba(0,255,136,0.13);margin-bottom:10px}
.logo{font-size:1.3rem;color:var(--g);letter-spacing:3.5px;text-shadow:0 0 12px var(--g)}
.sub{font-size:0.6rem;color:var(--m);letter-spacing:1.5px;margin-left:auto}
.tabs{display:flex;gap:5px;margin-bottom:9px;flex-wrap:wrap}
.tab{flex:1;min-width:80px;padding:7px;text-align:center;font-size:0.7rem;letter-spacing:1.5px;text-decoration:none;color:var(--m);border:1px solid rgba(0,255,136,0.2);border-radius:2px;text-transform:uppercase;background:rgba(0,0,0,0.4)}
.tab.active{background:rgba(0,255,136,0.13);color:var(--g);border-color:var(--g)}
.lbl{font-size:0.59rem;color:var(--m);letter-spacing:2px;text-transform:uppercase;margin:9px 0 5px;display:flex;align-items:center;gap:6px}
.lbl::after{content:'';flex:1;height:1px;background:rgba(0,255,136,0.1)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-bottom:10px}
.box{background:rgba(0,0,0,0.55);border:1px solid var(--br);border-radius:2px;padding:9px}
.box-title{font-size:0.65rem;color:var(--g2);letter-spacing:2px;font-weight:700;margin-bottom:6px;text-transform:uppercase}
.sig-table{width:100%;font-size:0.65rem;border-collapse:collapse}
.sig-table th{text-align:left;color:var(--y);border-bottom:1px solid rgba(0,255,136,0.2);padding:4px 3px}
.sig-table td{padding:4px 3px;border-bottom:1px solid rgba(0,255,136,0.1)}
.sig-table tr:hover{background:rgba(0,255,136,0.06)}
.leg-diagram{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:8px 0}
.leg-item{background:rgba(0,0,0,0.55);border:2px solid rgba(0,255,136,0.3);border-radius:3px;padding:8px;text-align:center;transition:all 0.2s}
.leg-item.active{border-color:var(--g);background:rgba(0,255,136,0.08);box-shadow:0 0 8px rgba(0,255,136,0.3)}
.leg-item.fault{border-color:var(--r);background:rgba(255,51,68,0.15);box-shadow:0 0 8px rgba(255,51,68,0.4)}
.leg-name{font-size:0.75rem;color:var(--g2);font-weight:700;letter-spacing:1.5px}
.leg-status{font-size:0.55rem;color:var(--m);margin-top:3px}
.conf-bar{display:flex;align-items:center;gap:4px;margin:3px 0}
.conf-val{font-size:0.6rem;color:var(--g2);min-width:35px;text-align:right}
.conf-bar-bg{flex:1;height:12px;background:rgba(0,0,0,0.6);border:1px solid rgba(0,255,136,0.2);border-radius:2px;overflow:hidden}
.conf-bar-fill{height:100%;background:linear-gradient(90deg,var(--b),var(--g2));transition:width 0.2s;min-width:2px}
.recovery-params{display:grid;grid-template-columns:repeat(2,1fr);gap:6px;font-size:0.6rem}
.param-item{background:rgba(0,0,0,0.4);border-left:3px solid var(--y);padding:4px 6px}
.param-label{color:var(--m);font-size:0.55rem}
.param-value{color:var(--g2);font-weight:700;margin-top:2px}
.state-box{background:rgba(0,0,0,0.55);border:2px solid var(--b);border-radius:2px;padding:9px;margin:8px 0;text-align:center}
.state-title{font-size:0.6rem;color:var(--m);letter-spacing:1.5px;text-transform:uppercase}
.state-current{font-size:0.85rem;color:var(--b);font-weight:700;margin:4px 0;text-shadow:0 0 6px rgba(85,136,255,0.4)}
.state-arrow{color:var(--m);font-size:0.55rem;margin:2px 0}
button{all:unset;display:block;width:100%;text-align:center;padding:9px;border:1px solid rgba(0,255,136,0.26);border-radius:2px;font-family:inherit;font-size:0.7rem;font-weight:700;letter-spacing:1.5px;color:var(--t);background:rgba(0,0,0,0.55);cursor:pointer;text-transform:uppercase;transition:all 0.12s}
button:hover{background:rgba(0,255,136,0.09);border-color:var(--g);box-shadow:0 0 9px rgba(0,255,136,0.2)}
.info-box{background:rgba(0,255,136,0.04);border:1px solid rgba(0,255,136,0.13);border-radius:2px;padding:6px 8px;font-size:0.6rem;color:var(--m);line-height:1.4;margin:8px 0}
.model-badge{display:inline-block;background:rgba(85,136,255,0.15);border:1px solid rgba(85,136,255,0.5);border-radius:2px;padding:3px 6px;font-size:0.55rem;color:var(--b);letter-spacing:1px;font-weight:700;margin:2px 2px 2px 0}
</style></head><body>
<div class="card">
 <div class="hdr">
  <div class="logo">CHITTI 2.0</div>
  <div class="sub">CONFIG·AI v2.0</div>
 </div>
 <div class="tabs">
  <a class="tab" href="/">CONTROL</a>
  <a class="tab" href="/telemetry">TELEMETRY</a>
  <a class="tab active" href="/config">CONFIG·AI</a>
 </div>

 <div class="lbl">system state machine</div>
 <div class="state-box">
  <div class="state-title">Current State</div>
  <div class="state-current" id="stateVal">IDLE</div>
  <div class="state-arrow">↓</div>
  <div style="font-size:0.58rem;color:var(--m)">Detecting · Recovering · Nominal</div>
 </div>

 <div class="grid2">
  <div class="box">
   <div class="box-title">📊 Signature Lookup Table</div>
   <table class="sig-table">
    <thead><tr>
     <th>Leg</th><th>Roll (μ)</th><th>Pitch (μ)</th><th>Σ Roll</th><th>Σ Pitch</th>
    </tr></thead>
    <tbody id="sigTable"></tbody>
   </table>
  </div>
  
  <div class="box">
   <div class="box-title">🦵 Leg Status Diagram</div>
   <div class="leg-diagram" id="legDiagram"></div>
  </div>
 </div>

 <div class="lbl">neural prediction & confidence</div>
 <div class="box">
  <div class="box-title">Classification Scores (Mahalanobis Distance)</div>
  <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px" id="confScores"></div>
 </div>

 <div class="lbl">active recovery model</div>
 <div class="grid2">
  <div class="box">
   <div class="box-title">🧠 Model Selection</div>
   <div style="padding:6px 0">
    <div class="model-badge" id="modelBadge">Rule-Based v2</div>
    <div style="font-size:0.6rem;color:var(--m);margin:6px 0">
     Active: <span id="modelName">Fault Detection Engine</span>
    </div>
    <div style="font-size:0.55rem;color:var(--m);line-height:1.5;margin:6px 0">
     Hybrid rule + feedback loop<br/>
     Confidence threshold: 72%<br/>
     Hysteresis: 500ms
    </div>
   </div>
  </div>
  
  <div class="box">
   <div class="box-title">⚙️ Recovery Offsets (Active)</div>
   <div class="recovery-params" id="recoveryParams"></div>
  </div>
 </div>

 <div class="lbl">gait adjustment parameters</div>
 <div class="box">
  <table class="sig-table">
   <thead><tr>
    <th>Parameter</th><th>Normal Mode</th><th>Recovery Mode</th><th>Unit</th>
   </tr></thead>
   <tbody>
    <tr><td>SWING (stride)</td><td id="swingNorm">28.0</td><td id="swingRec">28.0</td><td>deg</td></tr>
    <tr><td>LIFT (height)</td><td id="liftNorm">22.0</td><td id="liftRec">22.0</td><td>deg</td></tr>
    <tr><td>FM_STAND (femur)</td><td id="fmNorm">0.0</td><td id="fmRec">0.0</td><td>deg</td></tr>
    <tr><td>TB_STAND (tibia)</td><td id="tbNorm">0.0</td><td id="tbRec">0.0</td><td>deg</td></tr>
    <tr><td>STEP_MS (timing)</td><td id="stepNorm">450</td><td id="stepRec">450</td><td>ms</td></tr>
   </tbody>
  </table>
 </div>

 <div class="lbl">real-time detection metrics</div>
 <div class="grid3">
  <div class="box">
   <div class="box-title">📈 Current MPU</div>
   <div style="font-size:0.75rem;padding:4px 0">
    <div style="margin:3px 0"><span style="color:var(--m)">Roll:</span> <span style="color:var(--g);" id="mpuRoll">0.0</span>°</div>
    <div style="margin:3px 0"><span style="color:var(--m)">Pitch:</span> <span style="color:var(--g);" id="mpuPitch">0.0</span>°</div>
    <div style="margin:3px 0"><span style="color:var(--m)">Tilt:</span> <span style="color:var(--g);" id="mpuTilt">0.0</span>°</div>
   </div>
  </div>

  <div class="box">
   <div class="box-title">🎯 Top Prediction</div>
   <div style="font-size:0.75rem;padding:4px 0">
    <div style="margin:3px 0"><span style="color:var(--m)">Leg:</span> <span style="color:var(--b);" id="predLeg">-</span></div>
    <div style="margin:3px 0"><span style="color:var(--m)">Confidence:</span> <span style="color:var(--b);" id="predConf">0%</span></div>
    <div style="margin:3px 0"><span style="color:var(--m)">Status:</span> <span style="color:var(--y);" id="predStatus">NOMINAL</span></div>
   </div>
  </div>

  <div class="box">
   <div class="box-title">🔄 Recovery Progress</div>
   <div style="font-size:0.75rem;padding:4px 0">
    <div style="margin:3px 0"><span style="color:var(--m)">State:</span> <span style="color:var(--g);" id="recState">NONE</span></div>
    <div style="margin:3px 0"><span style="color:var(--m)">Active Leg:</span> <span style="color:var(--g);" id="recLeg">-</span></div>
    <div style="margin:3px 0"><span style="color:var(--m)">Time:</span> <span style="color:var(--g);" id="recTime">0s</span></div>
   </div>
  </div>
 </div>

 <div class="info-box">
  <b>💡 How it works:</b><br/>
  1) MPU fuses gyro + accel → roll/pitch estimate<br/>
  2) Signature lookup matches (roll, pitch) to leg via Mahalanobis distance<br/>
  3) Confidence scoring: if &gt;72% for 500ms → trigger recovery<br/>
  4) Apply coxa offsets + gait adjustments from rule table<br/>
  5) Monitor until MPU returns normal (&gt;3s) → clear recovery<br/>
  Rule-based fallback ensures robustness; neural refinement ready.
 </div>

 <button onclick="location='/'">Back to Control</button>
</div>

<script>
const legNames = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3'];
let lastState = {};

async function update(){
  try{
    const r = await fetch('/config.json', {cache:'no-store'});
    if(r.status === 401){ location='/'; return; }
    const j = await r.json();
    
    // State machine
    const stateMap = {0:'NOMINAL', 1:'FAULT_DETECTED', 2:'RECOVERING'};
    $('stateVal').textContent = stateMap[j.faultState] || 'UNKNOWN';
    
    // Signature table
    if(!lastState.sigs){
      let html = '';
      for(let i=0;i<6;i++){
        html += '<tr><td>'+legNames[i]+'</td>';
        html += '<td>'+j.sig[i].roll_m.toFixed(1)+'</td>';
        html += '<td>'+j.sig[i].pitch_m.toFixed(1)+'</td>';
        html += '<td>'+j.sig[i].roll_s.toFixed(2)+'</td>';
        html += '<td>'+j.sig[i].pitch_s.toFixed(2)+'</td></tr>';
      }
      $('sigTable').innerHTML = html;
      lastState.sigs = true;
    }
    
    // Leg diagram
    let legHtml = '';
    for(let i=0;i<6;i++){
      const isActive = j.legs[i] === 1;
      const isFault = j.fault === i;
      const cls = isFault ? 'fault' : (isActive ? 'active' : '');
      legHtml += '<div class="leg-item '+cls+'">';
      legHtml += '<div class="leg-name">'+legNames[i]+'</div>';
      legHtml += '<div class="leg-status">'+(isFault?'⚠ FAULT':(isActive?'✓ ACTIVE':'✗ OFF'))+'</div>';
      legHtml += '</div>';
    }
    $('legDiagram').innerHTML = legHtml;
    
    // Confidence scores
    let confHtml = '';
    for(let i=0;i<6;i++){
      const conf = j.conf[i] * 100;
      const fillW = Math.min(conf, 100);
      confHtml += '<div style="margin:6px 0">';
      confHtml += '<div style="font-size:0.6rem;color:var(--g2);margin-bottom:2px">'+legNames[i]+'</div>';
      confHtml += '<div class="conf-bar">';
      confHtml += '<div class="conf-bar-bg"><div class="conf-bar-fill" style="width:'+fillW+'%"></div></div>';
      confHtml += '<div class="conf-val">'+conf.toFixed(0)+'%</div>';
      confHtml += '</div></div>';
    }
    $('confScores').innerHTML = confHtml;
    
    // Recovery params
    let paramHtml = '';
    const labels = ['CX L1', 'CX L2', 'CX L3', 'CX R1', 'CX R2', 'CX R3'];
    for(let i=0;i<6;i++){
      paramHtml += '<div class="param-item">';
      paramHtml += '<div class="param-label">'+labels[i]+'</div>';
      paramHtml += '<div class="param-value">'+j.recovery_cx[i].toFixed(1)+'°</div>';
      paramHtml += '</div>';
    }
    $('recoveryParams').innerHTML = paramHtml;
    
    // Gait params
    $('swingNorm').textContent = '28.0';
    $('swingRec').textContent = j.swing.toFixed(1);
    $('liftNorm').textContent = '22.0';
    $('liftRec').textContent = j.lift.toFixed(1);
    $('fmNorm').textContent = '0.0';
    $('fmRec').textContent = j.fm_bias.toFixed(2);
    $('tbNorm').textContent = '0.0';
    $('tbRec').textContent = j.tb_bias.toFixed(2);
    $('stepNorm').textContent = '450';
    $('stepRec').textContent = j.step_ms;
    
    // Real-time metrics
    $('mpuRoll').textContent = j.roll.toFixed(2);
    $('mpuPitch').textContent = j.pitch.toFixed(2);
    const tilt = Math.sqrt(j.roll*j.roll + j.pitch*j.pitch);
    $('mpuTilt').textContent = tilt.toFixed(2);
    
    // Prediction
    if(j.fault >= 0){
      $('predLeg').textContent = legNames[j.fault];
      $('predConf').textContent = (j.faultConf * 100).toFixed(0) + '%';
      $('predStatus').textContent = ['DETECTING','RECOVERING','CLEARED'][j.faultState];
    } else {
      $('predLeg').textContent = '✓ None';
      $('predConf').textContent = '0%';
      $('predStatus').textContent = 'NOMINAL';
    }
    
    // Recovery
    if(j.faultState === 0){
      $('recState').textContent = 'NONE';
      $('recLeg').textContent = '-';
      $('recTime').textContent = '0s';
    } else {
      $('recState').textContent = j.faultState === 1 ? 'DETECTING' : 'ACTIVE';
      $('recLeg').textContent = j.fault >= 0 ? legNames[j.fault] : '-';
      $('recTime').textContent = j.recTime + 's';
    }
  }catch(e){
    console.error(e);
  }
}

const $ = id => document.getElementById(id);
update();
setInterval(update, 400);
</script>
</body></html>
)RAW";

// ════════════════════════ 20. WEB ROUTES ════════════════════════════
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
  httpServer.on("/config", HTTP_GET, []() {
    if (!checkAuth()) return;
    httpServer.send_P(200, "text/html", CONFIG_HTML);
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
    j += "],\"pcaL\":";   j += pcaL_ok ? "true" : "false";
    j += ",\"pcaR\":";    j += pcaR_ok ? "true" : "false";
    j += ",\"mpu\":";     j += mpu_ok  ? "true" : "false";
    j += ",\"roll\":";    j += String(roll_deg(),  2);
    j += ",\"pitch\":";   j += String(pitch_deg(), 2);
    j += ",\"yaw\":";     j += String(yaw_deg(),   2);
    j += ",\"fault\":";   j += (faultState != FAULT_NONE) ? String((int)faultedLeg) : "-1";
    j += ",\"faultConf\":"; j += String(faultConfidence, 3);
    j += ",\"autoRecovery\":"; j += autoRecoveryEnabled ? "true" : "false";
    j += "}";
    httpServer.send(200, "application/json", j);
  });

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

  httpServer.on("/config.json", HTTP_GET, []() {
    if (!checkAuth(true)) return;
    String j = "{";
    
    // Fault state machine
    j += "\"faultState\":"; j += String((int)faultState);
    j += ",\"fault\":"; j += (faultedLeg < 6) ? String((int)faultedLeg) : "-1";
    j += ",\"faultConf\":"; j += String(faultConfidence, 3);
    j += ",\"recTime\":"; j += String((millis() - faultDetectedTime) / 1000);
    
    // Leg states
    j += ",\"legs\":[";
    for(uint8_t i=0;i<6;i++){
      j += (legState[i] == LEG_ACTIVE) ? "1" : "0";
      if(i<5) j += ",";
    }
    j += "]";
    
    // Signatures
    j += ",\"sig\":[";
    for(uint8_t i=0;i<6;i++){
      j += "{\"roll_m\":"; j += String(sig[i].roll_mean, 2);
      j += ",\"pitch_m\":"; j += String(sig[i].pitch_mean, 2);
      j += ",\"roll_s\":"; j += String(sig[i].roll_std, 3);
      j += ",\"pitch_s\":"; j += String(sig[i].pitch_std, 3);
      j += "}";
      if(i<5) j += ",";
    }
    j += "]";
    
    // Confidence scores (calculated for all legs)
    j += ",\"conf\":[";
    for(uint8_t i=0;i<6;i++){
      float droll = fabsf(roll_deg() - sig[i].roll_mean) / (sig[i].roll_std + 0.5f);
      float dpitch = fabsf(pitch_deg() - sig[i].pitch_mean) / (sig[i].pitch_std + 0.5f);
      float dist = sqrtf(droll*droll + dpitch*dpitch);
      float conf = expf(-dist * dist / 2.0f);
      j += String(conf, 3);
      if(i<5) j += ",";
    }
    j += "]";
    
    // Recovery offsets currently active
    j += ",\"recovery_cx\":[";
    for(uint8_t i=0;i<6;i++){
      j += String(cxRecoveryOffset[i], 2);
      if(i<5) j += ",";
    }
    j += "]";
    
    // Current gait params
    j += ",\"swing\":"; j += String(SWING, 2);
    j += ",\"lift\":"; j += String(LIFT, 2);
    j += ",\"fm_bias\":"; j += String(FM_STAND, 3);
    j += ",\"tb_bias\":"; j += String(TB_STAND, 3);
    j += ",\"step_ms\":"; j += String(STEP_MS);
    
    // MPU readings
    j += ",\"roll\":"; j += String(roll_deg(), 3);
    j += ",\"pitch\":"; j += String(pitch_deg(), 3);
    j += ",\"yaw\":"; j += String(yaw_deg(), 3);
    
    j += "}";
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

// ════════════════════════ 21. SETUP / LOOP ══════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║        CHITTI 2.0 - v2.0 BOOT        ║");
  Serial.println("║   AUTO-RECOVERY HEXAPOD CONTROLLER     ║");
  Serial.println("╚════════════════════════════════════════╝");

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

  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  uint8_t whoami = Wire.read();
  Serial.print("MPU6050 WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  mpu_ok = (whoami == 0x68 || whoami == 0x72 || whoami == 0x70);
  Serial.print("MPU6050 status: ");
  Serial.println(mpu_ok ? "✓ CONNECTED" : "✗ NOT FOUND");
  if (mpu_ok) {
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    delay(200);
    calibrateMPU();
  }

  setupWeb();
  smoothStartup();
  lastMode = curMode;

  setStatus("READY - AUTO-RECOVERY ARMED");
  Serial.println();
  Serial.println("╔════════ COMMANDS ════════╗");
  Serial.println("║ Walk:   F  B  L  R  S    ║");
  Serial.println("║ Speed:  1  2  3          ║");
  Serial.println("║ Stride: +  -             ║");
  Serial.println("║ Legs:   a c d e g h      ║");
  Serial.println("║ Auto-Recovery: X         ║");
  Serial.println("║ Calibrate: Z             ║");
  Serial.println("╚═════════════════════════╝");
  Serial.println("UI: http://192.168.4.1");
  Serial.println("  Control:    /");
  Serial.println("  Telemetry:  /telemetry");
  Serial.println("════════════════════════════");
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
