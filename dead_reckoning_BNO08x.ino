#include <Adafruit_BNO08x.h>
#include <SimpleKalmanFilter.h>
#include <QRM3100.h>
#include <Wire.h>

#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

QRM3100 mag;

SimpleKalmanFilter kalman_r(0.5, 0.5, 0.01);
SimpleKalmanFilter kalman_i(0.5, 0.5, 0.01);
SimpleKalmanFilter kalman_j(0.5, 0.5, 0.01);
SimpleKalmanFilter kalman_k(0.5, 0.5, 0.01);
SimpleKalmanFilter kalman_x(2, 2, 0.01);
SimpleKalmanFilter kalman_y(2, 2, 0.01);
SimpleKalmanFilter kalman_z(2, 2, 0.01);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println("Adafruit BNO08x test!");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }
  setReports();

  Serial.println("Reading events");
  delay(100);

  if (mag.readRegister(0x36) != 0x22) {
    Serial.println("REVID NOT CORRECT!");
    Serial.print("REVID: ");
    Serial.println(mag.readRegister(0x36), HEX);
  } else {
    Serial.println("RM3100 Detected Properly");
    Serial.print("REVID: ");
    Serial.println(mag.readRegister(0x36), HEX);
  }
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, float &pitch, float &roll, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    pitch *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;
  }
}

void rotateVectorByQuaternion(float x, float y, float z, float qr, float qi, float qj, float qk, float &xr, float &yr, float &zr) {
  float q0 = qr;
  float q1 = qi;
  float q2 = qj;
  float q3 = qk;

  // Quaternion conjugate
  float qc0 = q0;
  float qc1 = -q1;
  float qc2 = -q2;
  float qc3 = -q3;

  // q * v
  float w1 = -q1 * x - q2 * y - q3 * z;
  float x1 = q0 * x + q2 * z - q3 * y;
  float y1 = q0 * y + q3 * x - q1 * z;
  float z1 = q0 * z + q1 * y - q2 * x;

  // (q * v) * q_conjugate
  xr = w1 * qc1 + x1 * qc0 + y1 * qc3 - z1 * qc2;
  yr = w1 * qc2 - x1 * qc3 + y1 * qc0 + z1 * qc1;
  zr = w1 * qc3 + x1 * qc2 - y1 * qc1 + z1 * qc0;
}

float computeYawFromMagnetometer(float mx, float my) {
  float yaw = atan2(my, mx);  // Standard atan2(y, x)
  yaw *= RAD_TO_DEG;
  if (yaw < 0) yaw += 360;  // Normalize to [0, 360]
  return yaw;
}

void loop() {
  delay(20);
  float r, i, j, k;
  int32_t x, y, z;
  int32_t mx, my, mz;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

    case SH2_GAME_ROTATION_VECTOR:
      r = kalman_r.updateEstimate(sensorValue.un.gameRotationVector.real);
      i = kalman_i.updateEstimate(sensorValue.un.gameRotationVector.i);
      j = kalman_j.updateEstimate(sensorValue.un.gameRotationVector.j);
      k = kalman_k.updateEstimate(sensorValue.un.gameRotationVector.k);

      float norm = sqrt(r * r + i * i + j * j + k * k);
      if (abs(norm - 1.0) > 0.1) {
        Serial.println("Warning: Invalid quaternion");
        return;  // Skip this loop iteration
      }
      break;
  }

  float pitch = 0;
  float roll = 0;
  quaternionToEuler(r, i, j, k, pitch, roll, true);

  static uint32_t lastMagTime = 0;
  if (millis() - lastMagTime > 100) {  // 10Hz
    mag.writeRegister(0x00, 0x70);
    lastMagTime = millis();
  }

  while ((mag.readRegister(0x34) & 0x80) != 0x80) {}
  // uint32_t timeout = millis() + 100;  // wait up to 100 ms
  // while ((mag.readRegister(0x34) & 0x80) != 0x80 && millis() < timeout) {}


  uint8_t mx2 = mag.readRegister(0x24);
  uint8_t mx1 = mag.readRegister(0x25);
  uint8_t mx0 = mag.readRegister(0x26);
  uint8_t my2 = mag.readRegister(0x27);
  uint8_t my1 = mag.readRegister(0x28);
  uint8_t my0 = mag.readRegister(0x29);
  uint8_t mz2 = mag.readRegister(0x2A);
  uint8_t mz1 = mag.readRegister(0x2B);
  uint8_t mz0 = mag.readRegister(0x2C);

  x = ((int32_t)(mx2 * 256 * 256) | (mx1 * 256) | mx0) / 75;
  // int32_t combine24(uint8_t h, uint8_t m, uint8_t l) {
  //int32_t val = ((int32_t)h << 24) | ((int32_t)m << 16) | ((int32_t)l << 8);
  //return val / 256; // Sign-extended 24-bit
  y = ((int32_t)(my2 * 256 * 256) | (my1 * 256) | my0) / 75;
  z = ((int32_t)(mz2 * 256 * 256) | (mz1 * 256) | mz0) / 75;

  mx = kalman_x.updateEstimate(x);
  my = kalman_y.updateEstimate(y);
  mz = kalman_z.updateEstimate(z);

  float mx_earth, my_earth, mz_earth;
  rotateVectorByQuaternion(mx, my, mz, r, i, j, k, mx_earth, my_earth, mz_earth);
  float yaw = computeYawFromMagnetometer(mx_earth, my_earth);

  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.print("°, Roll: ");
  Serial.print(roll, 2);
  Serial.println("°");
  Serial1.print("Yaw: ");
  Serial1.print(yaw, 2);
  Serial1.print("°  ");

  delay(500);
}
