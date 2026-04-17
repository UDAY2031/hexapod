#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== PCA9685 =====================
Adafruit_PWMServoDriver pcaLeft(0x40);   // L1..L9 -> channels 0..8
Adafruit_PWMServoDriver pcaRight(0x41);  // R1..R9 -> channels 0..8

static const float PWM_FREQ = 50.0f; // Servo frequency

// ===================== Calibration Data =====================
struct ServoCal {
  uint16_t usAtPos45; // pulse at +45 deg
  uint16_t usAtNeg45; // pulse at -45 deg
  uint8_t board;      // 0=left PCA, 1=right PCA
  uint8_t channel;    // PCA channel
  const char* name;
};

ServoCal servos[] = {
  // Left side
  {  985, 2010, 0, 0, "L1" },
  { 1117, 1989, 0, 1, "L2" },
  { 1018, 2037, 0, 2, "L3" },
  {  973, 2002, 0, 3, "L4" },
  { 1007, 1925, 0, 4, "L5" },
  {  932, 1947, 0, 5, "L6" },
  { 1016, 2137, 0, 6, "L7" },
  {  956, 1868, 0, 7, "L8" },
  {  976, 2069, 0, 8, "L9" },

  // Right side
  { 1038, 2032, 1, 0, "R1" },
  { 1043, 1962, 1, 1, "R2" },
  {  918, 1962, 1, 2, "R3" },
  {  956, 1983, 1, 3, "R4" },
  { 1092, 2014, 1, 4, "R5" },
  {  938, 1990, 1, 5, "R6" },
  {  930, 1998, 1, 6, "R7" },
  { 1052, 1973, 1, 7, "R8" },
  {  995, 1978, 1, 8, "R9" }
};

const size_t SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// ===================== LEVEL TRIM =====================
// Change ONLY this array to level robot.
// Right side is slightly lifted by small +trim.
float servoTrimDeg[] = {
  // L1..L9
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  // R1..R9 (increased a bit more to lift right side)
  0, 0, 0, 0, 0, 0, 0, 0, 0/
  
};

// ===================== Helpers =====================
uint16_t usToTicks(uint16_t us) {
  float ticks = (us * PWM_FREQ * 4096.0f) / 1000000.0f;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)(ticks + 0.5f);
}

uint16_t pulseForAngle(const ServoCal& s, float angleDeg) {
  if (angleDeg < -45.0f) angleDeg = -45.0f;
  if (angleDeg >  45.0f) angleDeg =  45.0f;

  // t=0 at -45, t=1 at +45
  float t = (angleDeg + 45.0f) / 90.0f;
  float us = s.usAtNeg45 + (s.usAtPos45 - s.usAtNeg45) * t;
  return (uint16_t)(us + 0.5f);
}

void writeServoUS(const ServoCal& s, uint16_t us) {
  uint16_t ticks = usToTicks(us);
  if (s.board == 0) pcaLeft.setPWM(s.channel, 0, ticks);
  else              pcaRight.setPWM(s.channel, 0, ticks);
}

void setAllToAngle(float angleDeg) {
  for (size_t i = 0; i < SERVO_COUNT; i++) {
    float correctedAngle = angleDeg + servoTrimDeg[i];
    uint16_t us = pulseForAngle(servos[i], correctedAngle);
    writeServoUS(servos[i], us);
  }
}

void moveAllSmooth(float startDeg, float endDeg, uint16_t durationMs, uint8_t steps = 50) {
  for (uint8_t i = 0; i <= steps; i++) {
    float a = (float)i / (float)steps;
    float deg = startDeg + (endDeg - startDeg) * a;
    setAllToAngle(deg);
    delay(durationMs / steps);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  pcaLeft.begin();
  pcaRight.begin();
  pcaLeft.setPWMFreq(PWM_FREQ);
  pcaRight.setPWMFreq(PWM_FREQ);
  delay(200);

  // Move to default pose with trim applied
  moveAllSmooth(-10.0f, 0.0f, 1200);

  Serial.println("Default pose reached with leveling trim.");
}

void loop() {
  // Hold pose
  setAllToAngle(0.0f);
  delay(200);
}
