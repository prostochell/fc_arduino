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

//Value for PiD control
float Kp_r = 1.0; //Roll control
float Kd_r = 0.5;
float Ki_r = 0.1;
float Kp_p = 1.0; //Pitch control
float Kd_p = 0.5;
float Ki_p = 0.1;
float Kp_y = 1.0; //Yaw control
float Kd_y = 0.5;
float Ki_y = 0.1;

int T1;  //Value for first motor
int T2;  //Value for second motor
int T3;  //Value for third motor
int T4;  //Value for fourth motor

float base_throttle = 900;
float pid_roll, pid_pitch, pid_yaw, error_roll, error_pitch, error_yaw, integral_roll, desired_roll, desired_pitch, desired_yaw;
float current_roll, current_pitch, current_yaw;
float  integral_yaw, integral_pitch = 0;



uint32_t timer;

void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */


  //Calculate acc angle without any filter
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG - 180;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG - 180;
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
  // use standard accuracy
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
  for (byte n = 0; n < 10; n++) {     // 10 calibration iterations
    for (byte j = 0; j < 6; j++) {    // reset the calibration array
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) {
      // make BUFFER_SIZE of measurements for averaging
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      // skip the first 99 measurements
      if (i >= 99) {
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];    // write to the calibration array
        }
      }
    }
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // take into account previous calibration
      if (i == 2) offsets[i] += 16384;                               // if Z axis, calibrate to 16384
      offsetsOld[i] = offsets[i];
    }
    // new offsets
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);

  }
}


double calc_pid(double input_angle, double set_angle, double  Kp, double  Ki, double  Kd, double  dt) {
  double err = set_angle - input_angle;
  double integral_ = 0.0;
  double prev_err = 0.0;
  integral_ += err * dt;
  double D = (err - prev_err) / dt;
  prev_err = err;
  return (err * Kp + integral_ * Ki + D * Kd);
}


void calculatePID() {

  float previous_error_roll = 0;
  float  previous_error_pitch = 0;
  float previous_error_yaw = 0;

  error_roll = desired_roll - current_roll;
  error_pitch = desired_pitch - current_pitch;
  error_yaw = desired_yaw - current_yaw;


  integral_roll += error_roll;
  pid_roll = Kp_r * error_roll + Ki_r * integral_roll + Kd_r * (error_roll - previous_error_roll);
  previous_error_roll = error_roll;


  integral_pitch += error_pitch;
  pid_pitch = Kp_p * error_pitch + Ki_p * integral_pitch + Kd_p * (error_pitch - previous_error_pitch);
  previous_error_pitch = error_pitch;


  integral_yaw += error_yaw;
  pid_yaw = Kp_y * error_yaw + Ki_y * integral_yaw + Kd_y * (error_yaw - previous_error_yaw);
  previous_error_yaw = error_yaw;
}






void setup() {
  int offsets[6];

  esc_9.attach(6);
  esc_10.attach(8);
  esc_11.attach(10);
  esc_12.attach(12);


  mpu.initialize();
  Serial.begin(115200);

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

  /*
    Serial.print("T1, ");
    Serial.print("T2, ");
    Serial.print("T3, ");
    Serial.print("T4, ");
    Serial.println();
    Serial.print("accXangle");
    Serial.print(" ");
    Serial.print("kalmanXacc");
    Serial.println();*/


}
void loop() {
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  calculateAngles();

  if (Serial.available() > 0) {
    base_throttle = (float)Serial.parseInt();
    Serial.println(base_throttle);
  }

  desired_roll = 0;
  desired_pitch = 0;
  current_pitch = kalAngleX;
  current_roll = kalAngleY;

  double pid_roll_1 = calc_pid(current_roll, desired_roll, Kp_r, Ki_r, Kd_r, 20);
  double pid_pitch_1 = calc_pid(current_pitch, desired_roll, Kp_r, Ki_r, Kd_r, 20);
  T1 = base_throttle - pid_pitch_1 + pid_roll_1;
  T2 = base_throttle + pid_pitch_1 + pid_roll_1 ;
  T3 = base_throttle + pid_pitch_1 - pid_roll_1 ;
  T4 = base_throttle - pid_pitch_1 - pid_roll_1;

  //String s = Serial.readString();
  //Serial.println(s);

  /*
    Serial.print(current_pitch);  Serial.print(" ");
    Serial.print(current_roll);   Serial.print(" ");
    Serial.print(desired_roll);   Serial.print(" ");
    Serial.print(desired_pitch);   Serial.println();


  */



  Serial.print(T1);
  Serial.print(", ");
  Serial.print(T2);
  Serial.print(", ");
  Serial.print(T3);
  Serial.print(", ");
  Serial.print(T4);
  Serial.println();

  esc_9.write(T1);
  esc_10.write(T2);
  esc_11.write(T3);
  esc_12.write(T4);

  delay(20); // The accelerometer's maximum samples rate is 1kHz
}
