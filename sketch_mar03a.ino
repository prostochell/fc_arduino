#define COMPL_K 0.09
#define BUFFER_SIZE 100
#define START_BYTE 1010

#include "Wire.h"
#include "Kalman.h"
#include "MPU6050.h"
#include <EEPROM.h>

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
  
  mpu.initialize();
  int offsets[6];
  Serial.begin(9600);
  EEPROM.get(START_BYTE, offsets);
  calibration();
  
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
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  calculateAngles();

  Serial.print(accXangle); Serial.print(",");
  Serial.print(gyroXangle); Serial.print(",");
  Serial.print(kalAngleX);  Serial.println();

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
  
  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * elapsedTime;
  gyroYangle += gyroYrate * elapsedTime;

  // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, elapsedTime);
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, elapsedTime);
  
}

// =======  ФУНКЦИЯ КАЛИБРОВКИ И ЗАПИСИ В ЕЕПРОМ =======
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
  delay(5);

  for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
    for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) {
      // делаем BUFFER_SIZE измерений для усреднения
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);

      if (i >= 99) {                         // пропускаем первые 99 измерений
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];   // записываем в калибровочный массив
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

  // пересчитываем для хранения
  for (byte i = 0; i < 6; i++) {
    if (i < 3) offsets[i] /= 8;
    else offsets[i] /= 4;
  }

  // запись в память
  EEPROM.put(START_BYTE, offsets);
}
