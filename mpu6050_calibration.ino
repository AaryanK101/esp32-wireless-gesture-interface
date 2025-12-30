#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake up
  Wire.endTransmission();
}

bool readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                 // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)MPU_ADDR, 14, true) != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read();         // temp (ignore)

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1200); // gives you time to open Serial Monitor

  Wire.begin(21, 22);
  Wire.setClock(100000);

  wakeMPU();
  Serial.println("MPU6050 ready. Printing A(ax,ay,az) and G(gx,gy,gz)...");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  if (readMPU(ax, ay, az, gx, gy, gz)) {
    Serial.print("A: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az);
    Serial.print(" | G: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);
  } else {
    Serial.println("MPU read failed");
  }

  delay(100);
}
