#include <Wire.h>
#include <math.h>

#define MPU6050_ADDR (uint8_t) 0x68 
// #define QMC5883L_ADDR (uint8_t) 0x0D
#define QMC5883P_ADDR (uint8_t) 0x2C

#define RAD2DEG 57.2957795f
#define DEG2RAD 0.0174532925f


// Raw sensor data
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;

// Scaled data
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// Orientation
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// Timing
unsigned long now;
unsigned long lastMicros;
float dt;


void writeRegister(uint8_t dev, uint8_t reg, uint8_t value){
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  }


void initMPU6050() {
  uint8_t reg_pwr_mgmt = 0x6B;
  uint8_t reg_gyro_cfg = 0x1B;
  uint8_t reg_acc_cfg = 0x1C;
  
  // Wake up MPU
  writeRegister(MPU6050_ADDR,reg_pwr_mgmt,0x00);
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x6B); // PWR_MGMT_1 Register
  // Wire.write(0x00);
  // Wire.endTransmission();

  // Gyro ±250 
  writeRegister(MPU6050_ADDR,reg_gyro_cfg,0x00);
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x1B); // GYRO_CONFIG Register
  // Wire.write(0x00);
  // Wire.endTransmission();

  // Accel ±2g
  writeRegister(MPU6050_ADDR,reg_acc_cfg,0x00);
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x1C); // ACCEL_CONFIG Register
  // Wire.write(0x00);
  // Wire.endTransmission();
  }

void initQMC5883P() {
  uint8_t qmc_reg_ctrl1 = 0x0A;
  uint8_t qmc_reg_ctrl2 = 0x0B;

  // Control Register 2:
  // Soft Reset
  writeRegister(QMC5883P_ADDR, qmc_reg_ctrl2, 0x01);
  delay(10);


  // Control register:
  // 0b0101_0001
  // [7:6] OSR = 128
  // [5:4] RNG = ±2G
  // [3:2] ODR = 100Hz
  // [1:0] MODE = Continuous
  writeRegister(QMC5883P_ADDR, qmc_reg_ctrl1, 0b01010001);

  // // Control Register 1:
  // // OSR2 = 1 
  // // OSR1 = 8
  // // ODR = 100Hz
  // // MODE = Continuous
  // uint8_t QMC5883L_CTRL1_SETTING = 0b00001011;
  // writeRegister(QMC5883P_ADDR, qmc_reg_ctrl1, QMC5883L_CTRL1_SETTING);

  delay(10);

  // // Control Register 2:
  // // RNG = ±2G
  // // SET/RESET MODE = 00
  // writeRegister(QMC5883P_ADDR, qmc_reg_ctrl2, 0b00001100);
  // delay(10);
  }

void readAccelGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();  // temp
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  ax = accX / 16384.0f;
  ay = accY / 16384.0f;
  az = accZ / 16384.0f;

  gx = gyroX / 131.0f;
  gy = gyroY / 131.0f;
  gz = gyroZ / 131.0f;
  }

bool readQMC5883P() {
  uint8_t qmc_reg_x_lsb = 0x01;
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(qmc_reg_x_lsb);                 // Start at X LSB register
  if (Wire.endTransmission(false) != 0)
    return false;                   // I2C error

  Wire.requestFrom(QMC5883P_ADDR, (uint8_t)6);
  if (Wire.available() < 6)
    return false;

  uint8_t xL = Wire.read();
  uint8_t xH = Wire.read();
  uint8_t yL = Wire.read();
  uint8_t yH = Wire.read();
  uint8_t zL = Wire.read();
  uint8_t zH = Wire.read();

  magX = (int16_t)(xH << 8 | xL);
  magY = (int16_t)(yH << 8 | yL);
  magZ = (int16_t)(zH << 8 | zL);

  return true;
}


float accRoll() {
  return atan2f(ay, az) * RAD2DEG;
}

float accPitch() {
  return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

float computeYaw() {
  float rollRad = roll * DEG2RAD;
  float pitchRad = pitch * DEG2RAD;

  float Xh = mx * cosf(pitchRad) + mz * sinf(pitchRad);
  float Yh = mx * sinf(rollRad) * sinf(pitchRad)
             + my * cosf(rollRad)
             - mz * sinf(rollRad) * cosf(pitchRad);

  float heading = atan2f(-Yh, Xh) * RAD2DEG;

  return (heading + 360) % 360;
}

void updateOrientation() {
  // Integrate gyro
  roll += gx * dt;
  pitch += gy * dt;
  yaw += gz * dt;

  // Accel correction
  roll = 0.98f * roll + 0.02f * accRoll();
  pitch = 0.98f * pitch + 0.02f * accPitch();

  // Magnetometer correction
  float yawMag = computeYaw();
  yaw = 0.95f * yaw + 0.05f * yawMag;
}

void scanI2C() {
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(i, HEX);
    }
  }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting Telescope Positioning Program...\n");
    delay(500);
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(400000);
    initMPU6050();
    initQMC5883P();


    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    scanI2C();
    delay(1000);
    // // Test Code:
    // while(1){
    //   digitalWrite(LED_BUILTIN, HIGH);
    //   delay(1000);
    //   digitalWrite(LED_BUILTIN, LOW);
    //   delay(1000);
    //   // scanI2C();
    //   if (!readQMC5883P()){
    //     Serial.println("Failed to read Mag Sensor!");
    //   }else{
    //     Serial.print("MX: ");
    //     Serial.print(magX);
    //     Serial.print(" MY: ");
    //     Serial.print(magY);
    //     Serial.print(" MZ: ");
    //     Serial.println(magZ);
    //   }
    //   delay(200);
    // }

  lastMicros = micros();

}

void loop() {

  while ((now-lastMicros) < 10000){
    now = micros();
  }
  dt = (now - lastMicros) /1000000;
  lastMicros = now;


  readAccelGyro();
  readQMC5883P();
  updateOrientation();

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);

}
