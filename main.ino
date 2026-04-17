/**
 * ╔══════════════════════════════════════════════════════╗
 * ║        Hexapod Tripod Gait Controller                ║
 * ║  Hardware : 2x PCA9685  (0x40=Left | 0x41=Right)    ║
 * ║  Legs     : FL, ML, BL (left) | FR, MR, BR (right)  ║
 * ║  Joints   : Coxa (yaw) | Femur (lift) | Tibia (ext) ║
 * ╠══════════════════════════════════════════════════════╣
 * ║  Serial 115200 baud commands:                        ║
 * ║    F = Forward    B = Backward    S = Stop           ║
 * ║    1 = Slow       2 = Normal      3 = Fast           ║
 * ║    + = Bigger stride    - = Smaller stride           ║
 * ║    H = Help                                          ║
 * ╚══════════════════════════════════════════════════════╝
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─────────────────────────────── PCA9685 ─────────────────────────────────────
Adafruit_PWMServoDriver pcaLeft(0x40);    // L1–L9  → channels 0–8
Adafruit_PWMServoDriver pcaRight(0x41);   // R1–R9  → channels 0–8
static const float PWM_FREQ = 50.0f;

// ─────────────────────────── Calibration Data ────────────────────────────────
struct ServoCal {
  uint16_t usAtPos45;   // pulse width (µs) at +45°
  uint16_t usAtNeg45;   // pulse width (µs) at −45°
  uint8_t  board;       // 0 = left PCA (0x40) | 1 = right PCA (0x41)
  uint8_t  channel;     // PCA9685 channel
  const char* name;
};

//                         +45µs   −45µs  brd  ch   name        index  joint
ServoCal servos[] = {
  // ── Left board (0x40) ─────────────────────────────────────────────────────
  {  985, 2010,  0,  0, "L1" },  //  0   FL-Coxa
  { 1117, 1989,  0,  1, "L2" },  //  1   FL-Femur
  { 1018, 2037,  0,  2, "L3" },  //  2   FL-Tibia
  {  973, 2002,  0,  3, "L4" },  //  3   ML-Coxa
  { 1007, 1925,  0,  4, "L5" },  //  4   ML-Femur
  {  932, 1947,  0,  5, "L6" },  //  5   ML-Tibia
  { 1016, 2137,  0,  6, "L7" },  //  6   BL-Coxa
  {  956, 1868,  0,  7, "L8" },  //  7   BL-Femur
  {  976, 2069,  0,  8, "L9" },  //  8   BL-Tibia
  // ── Right board (0x41) ────────────────────────────────────────────────────
  { 1038, 2032,  1,  0, "R1" },  //  9   FR-Coxa
  { 1043, 1962,  1,  1, "R2" },  // 10   FR-Femur
  {  918, 1962,  1,  2, "R3" },  // 11   FR-Tibia
  {  956, 1983,  1,  3, "R4" },  // 12   MR-Coxa
  { 1092, 2014,  1,  4, "R5" },  // 13   MR-Femur
  {  938, 1990,  1,  5, "R6" },  // 14   MR-Tibia
  {  930, 1998,  1,  6, "R7" },  // 15   BR-Coxa
  { 1052, 1973,  1,  7, "R8" },  // 16   BR-Femur
  {  995, 1978,  1,  8, "R9" }   // 17   BR-Tibia
};

const size_t SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// ───────────────────────────── Level Trim ────────────────────────────────────
float servoTrimDeg[18] = {
  // L1   L2   L3   L4   L5   L6   L7   L8   L9
     0,   0,   0,   0,   0,   0,   0,   0,   0,
  // R1   R2   R3   R4   R5   R6   R7   R8   R9
     0,   0,   0,   0,   0,   0,   0,   0,   0
};

// ─────────────────────────── Leg Definitions ─────────────────────────────────
struct Leg {
  uint8_t coxaIdx;    // servo index into servos[]
  uint8_t femurIdx;
  uint8_t tibiaIdx;
  int8_t  coxaDir;    // +1 for left legs, −1 for right legs (mirror symmetry)
  const char* name;
};

//          coxa  femur  tibia  dir   name        leg index
Leg legs[6] = {
  {  0,   1,   2,  +1, "FL" },  // 0 — Front Left
  {  3,   4,   5,  +1, "ML" },  // 1 — Mid   Left
  {  6,   7,   8,  +1, "BL" },  // 2 — Back  Left
  {  9,  10,  11,  -1, "FR" },  // 3 — Front Right
  { 12,  13,  14,  -1, "MR" },  // 4 — Mid   Right
  { 15,  16,  17,  -1, "BR" }   // 5 — Back  Right
};

// ── Tripod groups ─────────────────────────────────────────────────────────────
//   Group A: FL(0), MR(4), BL(2)   — lift & swing together
//   Group B: FR(3), ML(1), BR(5)   — lift & swing together
//   When A is airborne, B is on the ground pushing the body, and vice-versa.
const uint8_t GROUP_A[3] = { 0, 4, 2 };   // FL, MR, BL
const uint8_t GROUP_B[3] = { 3, 1, 5 };   // FR, ML, BR

// ─────────────────────────── Gait Parameters ─────────────────────────────────
// ── Tune these to dial in your robot's motion ──
float    COXA_SWING_DEG  = 18.0f;  // ±coxa sweep (degrees) — controls stride length
float    FEMUR_LIFT_DEG  = 14.0f;  // Lift height in degrees
                                    //   (+) subtracts from stand angle → lifts up
                                    //   if legs go DOWN instead of up, flip to (-)
float    FEMUR_STAND_DEG =  0.0f;  // Neutral femur angle when standing
float    TIBIA_STAND_DEG =  0.0f;  // Neutral tibia angle when standing
uint16_t STEP_MS         = 500;    // Half-cycle duration (ms) — lower = faster walk
const uint8_t INTERP_STEPS = 50;   // Sub-steps per half-cycle (smoothness)

// ─────────────────────────── Runtime State ───────────────────────────────────
float  coxaPos[6] = { 0 };   // Tracked coxa angle per leg (logical degrees)
int8_t walkDir    = 0;        // 0 = stopped | +1 = forward | −1 = backward
bool   isWalking  = false;

// ═════════════════════════ Low-Level Helpers ══════════════════════════════════

uint16_t usToTicks(uint16_t us) {
  float ticks = (us * PWM_FREQ * 4096.0f) / 1e6f;
  return (uint16_t)constrain(ticks + 0.5f, 0.0f, 4095.0f);
}

uint16_t pulseForAngle(const ServoCal& s, float deg) {
  deg = constrain(deg, -45.0f, 45.0f);
  float t  = (deg + 45.0f) / 90.0f;
  float us = s.usAtNeg45 + (s.usAtPos45 - s.usAtNeg45) * t;
  return (uint16_t)(us + 0.5f);
}

void writeServo(uint8_t idx, float deg) {
  deg += servoTrimDeg[idx];
  uint16_t ticks = usToTicks(pulseForAngle(servos[idx], deg));
  if (servos[idx].board == 0) pcaLeft.setPWM(servos[idx].channel,  0, ticks);
  else                         pcaRight.setPWM(servos[idx].channel, 0, ticks);
}

// Set all three joints of one leg.  coxaDir mirroring is applied here.
void setLeg(uint8_t i, float coxa, float femur, float tibia) {
  writeServo(legs[i].coxaIdx,  coxa * legs[i].coxaDir);
  writeServo(legs[i].femurIdx, femur);
  writeServo(legs[i].tibiaIdx, tibia);
}

// ═══════════════════════════ Stand Helpers ════════════════════════════════════

void standAll() {
  for (uint8_t i = 0; i < 6; i++) {
    setLeg(i, 0.0f, FEMUR_STAND_DEG, TIBIA_STAND_DEG);
    coxaPos[i] = 0.0f;
  }
}

// Smoothly raise the robot from a crouched femur angle to stand
void smoothRiseToStand(float fromFemur, float toFemur, uint16_t ms) {
  const uint8_t steps = 60;
  for (uint8_t s = 0; s <= steps; s++) {
    float t     = (float)s / (float)steps;
    float femur = fromFemur + (toFemur - fromFemur) * t;
    for (uint8_t i = 0; i < 6; i++)
      setLeg(i, 0.0f, femur, TIBIA_STAND_DEG);
    delay(ms / steps);
  }
}

// ═══════════════════════════ Tripod Gait Core ═════════════════════════════════

/**
 * tripodStep — one half-cycle of the tripod gait
 *
 *  swingId : 0 → Group A is airborne,  1 → Group B is airborne
 *  dir     : +1 = walk forward,       −1 = walk backward
 *
 *  SWING group  → lifted off the ground, coxa sweeps to new position
 *  STANCE group → firmly on the ground, coxa sweeps opposite direction
 *                 (friction with floor moves body forward)
 *
 *  Vertical profile: sinusoidal arc (0 → peak → 0) for smooth foot clearance
 */
void tripodStep(uint8_t swingId, int8_t dir) {
  const uint8_t* swingGrp  = (swingId == 0) ? GROUP_A : GROUP_B;
  const uint8_t* stanceGrp = (swingId == 0) ? GROUP_B : GROUP_A;

  // Coxa target positions for this half-cycle
  float swingEnd  =  (float)dir * COXA_SWING_DEG;   // swing foot → forward
  float stanceEnd = -(float)dir * COXA_SWING_DEG;   // stance pushes → backward

  // Record starting positions
  float swingStart[3], stanceStart[3];
  for (uint8_t i = 0; i < 3; i++) {
    swingStart[i]  = coxaPos[swingGrp[i]];
    stanceStart[i] = coxaPos[stanceGrp[i]];
  }

  uint16_t stepDelay = STEP_MS / INTERP_STEPS;
  if (stepDelay < 1) stepDelay = 1;

  for (uint8_t s = 0; s <= INTERP_STEPS; s++) {
    float t = (float)s / (float)INTERP_STEPS;

    // ── Sinusoidal vertical arc ───────────────────────────────────────────
    //    sin curve gives 0 at start/end, 1.0 at midpoint → smooth liftoff/touchdown
    float liftFactor = sin(t * PI);
    float femurSwing = FEMUR_STAND_DEG - (FEMUR_LIFT_DEG * liftFactor);
    //    ↑ subtract → femur angle decreases → leg rises
    //    If your femur lifts the other way, change '-' to '+'

    // ── Swing group (airborne) ────────────────────────────────────────────
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t li = swingGrp[i];
      coxaPos[li] = swingStart[i] + (swingEnd - swingStart[i]) * t;
      setLeg(li, coxaPos[li], femurSwing, TIBIA_STAND_DEG);
    }

    // ── Stance group (grounded, pushing body forward) ─────────────────────
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t li = stanceGrp[i];
      coxaPos[li] = stanceStart[i] + (stanceEnd - stanceStart[i]) * t;
      setLeg(li, coxaPos[li], FEMUR_STAND_DEG, TIBIA_STAND_DEG);
    }

    delay(stepDelay);
  }
}

// One full walk cycle = two half-cycles (Group A swings, then Group B swings)
void walkCycle(int8_t dir) {
  tripodStep(0, dir);   // Group A airborne
  tripodStep(1, dir);   // Group B airborne
}

// ═══════════════════════ Serial Command Handler ═══════════════════════════════

void printHelp() {
  Serial.println(F("─────────────────────────────────────"));
  Serial.println(F("  F / B     Forward / Backward"));
  Serial.println(F("  S         Stop & return to stand"));
  Serial.println(F("  1 / 2 / 3 Slow / Normal / Fast"));
  Serial.println(F("  + / -     Increase / Decrease stride"));
  Serial.println(F("  H         Show this help"));
  Serial.println(F("─────────────────────────────────────"));
}

void handleSerial() {
  if (!Serial.available()) return;
  char cmd = (char)Serial.read();

  switch (cmd) {
    // ── Direction ──────────────────────────────────────────────────────────
    case 'f': case 'F':
      walkDir   = +1;
      isWalking = true;
      Serial.println(F("[CMD] ▶ Forward"));
      break;

    case 'b': case 'B':
      walkDir   = -1;
      isWalking = true;
      Serial.println(F("[CMD] ◀ Backward"));
      break;

    case 's': case 'S':
      walkDir   = 0;
      isWalking = false;
      standAll();
      Serial.println(F("[CMD] ■ Stop → standing"));
      break;

    // ── Speed presets ──────────────────────────────────────────────────────
    case '1':
      STEP_MS = 700;
      Serial.println(F("[SPD] Slow  (700 ms/half-step)"));
      break;
    case '2':
      STEP_MS = 500;
      Serial.println(F("[SPD] Normal (500 ms/half-step)"));
      break;
    case '3':
      STEP_MS = 320;
      Serial.println(F("[SPD] Fast  (320 ms/half-step)"));
      break;

    // ── Stride size ────────────────────────────────────────────────────────
    case '+':
      COXA_SWING_DEG = min(COXA_SWING_DEG + 2.0f, 38.0f);
      Serial.print(F("[STR] Stride = "));
      Serial.print(COXA_SWING_DEG, 0);
      Serial.println(F("°"));
      break;
    case '-':
      COXA_SWING_DEG = max(COXA_SWING_DEG - 2.0f, 5.0f);
      Serial.print(F("[STR] Stride = "));
      Serial.print(COXA_SWING_DEG, 0);
      Serial.println(F("°"));
      break;

    // ── Help ───────────────────────────────────────────────────────────────
    case 'h': case 'H':
      printHelp();
      break;

    default: break;
  }
}

// ═══════════════════════════════ Setup ═══════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  pcaLeft.begin();   pcaLeft.setPWMFreq(PWM_FREQ);
  pcaRight.begin();  pcaRight.setPWMFreq(PWM_FREQ);
  delay(200);

  // Smoothly rise from crouched position to stand
  smoothRiseToStand(-10.0f, FEMUR_STAND_DEG, 1200);

  Serial.println(F("╔═══════════════════════════════╗"));
  Serial.println(F("║  Hexapod Tripod Gait — Ready  ║"));
  Serial.println(F("╚═══════════════════════════════╝"));
  printHelp();
}

// ══════════════════════════════ Main Loop ═════════════════════════════════════

void loop() {
  handleSerial();

  if (isWalking && walkDir != 0) {
    walkCycle(walkDir);
  } else {
    standAll();
    delay(50);
  }
}
