#include <Wire.h>

// MPU6500/6050 Register Map
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize I2C on your pins (18/19)
  Wire.begin(18, 19);
  Wire.setClock(400000); // Fast mode

  Serial.println("\n--- Phase 2: Raw IMU Data Stream ---");

  // Wake up the sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); 
  if (Wire.endTransmission() != 0) {
    Serial.println("Connection Failed. Check SDA(18)/SCL(19) and 3.3V.");
    while(1) delay(1000); // Avoid Watchdog reset with a delay
  }
  
  Serial.println("MPU6500 (0x70) Awake. Plotting Data...");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H); // Start reading from Accel X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Read 14 bytes (Accel, Temp, Gyro)

  // Read 16-bit values (combine High and Low bytes)
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t temp = (Wire.read() << 8) | Wire.read(); // Skip or use temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Print format for Serial Plotter
  Serial.print("AccX:"); Serial.print(ax); Serial.print(",");
  Serial.print("AccY:"); Serial.print(ay); Serial.print(",");
  Serial.print("AccZ:"); Serial.print(az); Serial.print(",");
  Serial.print("GyrX:"); Serial.print(gx); Serial.print(",");
  Serial.print("GyrY:"); Serial.print(gy); Serial.print(",");
  Serial.print("GyrZ:"); Serial.println(gz);

  delay(20); // 50Hz sampling for gait analysis
}
