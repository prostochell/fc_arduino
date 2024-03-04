#define COMPL_K 0.09
#include "Wire.h"
#include "Kalman.h"
Kalman kalmanX;
Kalman kalmanY;
uint8_t IMUAddress = 0x68;
/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
float accXangle; // Angle calculate using the accelerometer
float accYangle;
float temp;
float gyroXangle = 0; // Angle calculate using the gyro
float gyroYangle = 0;
float kalAngleX = 0; // Calculate the angle using a Kalman filter
float kalAngleY = 0;


uint32_t timer;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  kalmanX.setAngle(0); // Set starting angle
  kalmanY.setAngle(0);
  timer = micros();
  Serial.print("accXangle");
  Serial.print(",");
  Serial.print("gyroXangle");
  Serial.print(",");
  Serial.print("kalmanXacc");
  Serial.println();
  
}
void loop() {
  getValues();
  calculateAngles();

  Serial.print(accXangle); Serial.print(",");
  Serial.print(gyroXangle); Serial.print(",");
  Serial.print(kalAngleX);  Serial.println();

  delay(20); // The accelerometer's maximum samples rate is 1kHz
}

//Function for get vaules with iic
void getValues() {
  /* Update all the values */
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 14, true); // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */


  //Calculate acc angle without any filter
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG-180;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG-180;
  float gyroXrate = (float)gyroX / 131.0;
  float gyroYrate = -((float)gyroY / 131.0);
  
  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * ((float)(micros() - timer) / 1000000);
  gyroYangle += gyroYrate * ((float)(micros() - timer) / 1000000);

  // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (float)(micros() - timer) / 1000000);
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (float)(micros() - timer) / 1000000);
  timer = micros();

  
}
