/*
TODO: 
  * need to adjst code for better initial offset calibration routine
  * Need to somehow get Yaw position (z axis rotation) using a different sensor?
  * Convert into coordinate system for astronomy
  * Test on telescope
*/




#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

// Kalman filter variables (one filter per axis)
struct Kalman {
  float angle;      // Estimated angle
  float bias;       // Gyro bias
  float P[2][2];    // Error covariance matrix
};

Kalman kalmanRoll, kalmanPitch;

// Timing
unsigned long lastTime;
float dt;

// Raw sensor values
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// Scaled values
float ax, ay, az;
float gx, gy;

// Output angles
float roll, pitch, roll_offset, pitch_offset;

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);        // Power management
  Wire.write(0x00);        // Wake up MPU6050
  Wire.endTransmission(true);

  // Gyro config ±250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Accel config ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX  = Wire.read() << 8 | Wire.read();
  accY  = Wire.read() << 8 | Wire.read();
  accZ  = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temperature
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  // Scale values
  ax = accX / 16384.0;
  ay = accY / 16384.0;
  az = accZ / 16384.0;

  gx = gyroX / 131.0;
  gy = gyroY / 131.0;
}

// Calculate Accelerometer Roll angle in degrees (X axis Roll)
// Roll is irrelevant
float accRoll() {
  return atan2(ay, az) * RAD_TO_DEG;
}

// Calculate accelerometer Pitch angle in degrees (Y axis Pitch)
// Pitch will allow us to set the inclination of the scope
float accPitch() {
  return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}

// TODO: Need to get a magnetometer for correct Yaw (rotation about the x axis)


float kalmanUpdate(Kalman &k, float newAngle, float newRate) {
  // Prediction
  k.angle += dt * (newRate - k.bias);

  k.P[0][0] += dt * (dt * k.P[1][1] - k.P[0][1] - k.P[1][0] + 0.001);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += 0.003 * dt;

  // Update
  float S = k.P[0][0] + 0.03;
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  float y = newAngle - k.angle;
  k.angle += K0 * y;
  k.bias  += K1 * y;

  float P00 = k.P[0][0];
  float P01 = k.P[0][1];

  k.P[0][0] -= K0 * P00;
  k.P[0][1] -= K0 * P01;
  k.P[1][0] -= K1 * P00;
  k.P[1][1] -= K1 * P01;

  return k.angle;
}


void setup() {
  delay(1000);
  digitalWrite(13,1);
  Serial.begin(115200);
  Wire.begin();
  initMPU();

  lastTime = millis();
  // Get offsets:
  readMPU();
  float accRollAngle  = accRoll();
  float accPitchAngle = accPitch();
  roll_offset  = kalmanUpdate(kalmanRoll, accRollAngle, gx);
  pitch_offset = kalmanUpdate(kalmanPitch, accPitchAngle, gy);

}

void loop() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  readMPU();

  float accRollAngle  = accRoll();
  float accPitchAngle = accPitch();

  roll  = kalmanUpdate(kalmanRoll, accRollAngle, gx) - roll_offset;
  pitch = kalmanUpdate(kalmanPitch, accPitchAngle, gy) - pitch_offset;
  
  //TODO:
  // yaw - kalmanUpdate(kalmanYaw, accYawAngle, gz))

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("  Pitch: ");
  Serial.print(pitch);
  Serial.print("  Roll Offset: ");
  Serial.print(roll_offset);
  Serial.print("  Pitch Offset: ");
  Serial.println(pitch_offset);
  delay(10);
}


