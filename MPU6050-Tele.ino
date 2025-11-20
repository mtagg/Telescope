#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing MPU6050...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(10);
    }
  }

  // Optional: Configure settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 ready!");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.println("---- Sensor Readings ----");
  Serial.print("Accel (m/s^2): ");
  Serial.print(accel.acceleration.x);
  Serial.print(", ");
  Serial.print(accel.acceleration.y);
  Serial.print(", ");
  Serial.println(accel.acceleration.z);

  Serial.print("Gyro (rad/s):  ");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.println(gyro.gyro.z);

  Serial.print("Temp (C):      ");
  Serial.println(temp.temperature);

  Serial.println("-------------------------");
  Serial.println();

  delay(1000);  // Read once per second
}
