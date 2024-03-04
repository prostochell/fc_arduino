#define COMPL_K 0.09
#define BUFFER_SIZE 100

#include "Wire.h"
#include "Kalman.h"
#include "MPU6050.h"
#include <Servo.h>

Servo esc_12;
Servo esc_11; 
Servo esc_10;
Servo esc_9; 

MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;


int16_t accX;
int16_t accY;
int16_t accZ;

int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

float accXangle; // Angle calculate using the accelerometer
float accYangle;

float gyroXangle = 0; // Angle calculate using the gyro
float gyroYangle = 0;
float kalAngleX = 0; // Calculate the angle using a Kalman filter
float kalAngleY = 0;


uint32_t timer;
void setup() {

  esc_9.attach(9); 
  esc_10.attach(10);   
  esc_9.attach(11); 
  esc_10.attach(12); 

  
  mpu.initialize();
  int offsets[6];
  Serial.begin(9600);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);


  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  calibration();

  delay(1000);
  
  kalmanX.setAngle(0); // Set starting angle
  kalmanY.setAngle(0);
  timer = micros();

  esc_9.writeMicroseconds (2300);  
  esc_10.writeMicroseconds (2300); 
  esc_11.writeMicroseconds (2300);  
  esc_12.writeMicroseconds (2300);  
  delay (2000);
  esc_9.writeMicroseconds (800);
  esc_10.writeMicroseconds (800);
  esc_11.writeMicroseconds (800);
  esc_12.writeMicroseconds (800);
  delay(2000); 
  
  Serial.print("accXangle");
  Serial.print(",");
  Serial.print("kalmanXacc");
  Serial.println();
  
}
void loop() {
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  calculateAngles();

  Serial.print(accXangle);  Serial.print(",");
  Serial.print(kalAngleX);  Serial.println();

  
  esc_9.write(2250);
  esc_10.write(2250);  
  esc_11.write(2250);
  esc_12.write(2250);

  delay(20); // The accelerometer's maximum samples rate is 1kHz
}

//Function for get vaules with iic

void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */


  //Calculate acc angle without any filter
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG-180;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG-180;
  float gyroXrate = (float)gyroX / 131.0;
  float gyroYrate = -((float)gyroY / 131.0);
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - timer) / 1000000.0;
  timer = currentTime;

  // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, elapsedTime);
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, elapsedTime);
  
}

void calibration() {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];
  // используем стандартную точность
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // обнуляем оффсеты
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
  for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
    for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) {
      // делаем BUFFER_SIZE измерений для усреднения
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      // пропускаем первые 99 измерений
      if (i >= 99) {
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];    // записываем в калибровочный массив
        }
      }
    }
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
      if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
      offsetsOld[i] = offsets[i];
    }
    // ставим новые оффсеты
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);

  }
}
