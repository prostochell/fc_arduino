# FC_Arduino 🚁

**Project Status:** On Pause ⏸️

This project is a custom-built, Arduino-based flight controller for a quadcopter. The primary goal was to create proprietary stabilization logic from scratch, read data from an Inertial Measurement Unit (IMU), and control brushless DC motors via ESCs.

The core challenges and main focus areas of this development were filtering the noise from the MPU6050 accelerometer/gyroscope and writing/tuning a PID controller for Roll and Pitch stabilization.

---

## 🛠 Hardware & Setup

* **Microcontroller:** Arduino-compatible board (Nano/Uno/Mega)
* **IMU Sensor:** MPU6050 (Accelerometer + Gyroscope)
* **ESCs & Motors:** 4x (connected to pins 6, 8, 10, 12)
* **Dependencies:** `Wire.h`, `MPU6050.h`, `Kalman.h`, `Servo.h`

---

## 🎛 Noise Filtering & MPU6050 Data Processing

Raw data from the MPU6050 is highly susceptible to high-frequency vibrational noise (accelerometer) and drift over time (gyroscope). To obtain clean and accurate tilt angles, the project implements the following comprehensive approach:

### 1. Deep Calibration (`calibration` function)
Instead of relying on standard offsets, a custom calibration routine was written:
* It runs **10 iterations**, taking **100 measurements** per iteration (discarding the first 99 to allow the buffer to stabilize).
* It calculates the arithmetic mean to zero out the errors on the X and Y axes.
* For the accelerometer's Z-axis, Earth's gravity is taken into account (adding `16384`, which corresponds to 1G at the selected `MPU6050_ACCEL_FS_2` range).

### 2. Kalman Filter (Sensor Fusion)
To accurately fuse the sensor data, a Kalman Filter is used instead of a simpler complementary filter.
* **The Accelerometer** provides accurate long-term orientation but is extremely noisy due to motor vibrations (calculated via `atan2`).
* **The Gyroscope** reacts very quickly and accurately to sudden changes, but its values "drift" over time.
* **The Kalman Filter** (`kalmanX.getAngle`, `kalmanY.getAngle`) dynamically adjusts the trust weight between the two sensors based on noise variance, outputting the smoothest and most accurate angle possible (`kalAngleX`, `kalAngleY`) based on the loop's `elapsedTime` (dt).

---

## ⚖️ PID Controller & Stabilization Algorithm

A classic PID (Proportional-Integral-Derivative) controller is used to keep the drone level.

### Tuning Methodology
The code features base coefficients (`Kp = 1.0`, `Kd = 0.5`, `Ki = 0.1`). The classic tuning algorithm applied to this project is:
1.  **P (Proportional) — `Kp`:** Tuned first. It determines the strength of the reaction to an angle error. Increased until the drone starts to rapidly oscillate (wobble).
2.  **D (Derivative) — `Kd`:** Added for dampening. Acts as a "shock absorber," smoothing out sharp movements and stopping the oscillations caused by a high P-gain.
3.  **I (Integral) — `Ki`:** Tuned last. Corrects accumulated errors over time (e.g., if the drone constantly drifts to one side due to an off-center center of gravity or wind).

### Motor Mixing
The calculated PID values are added to or subtracted from the base throttle (`base_throttle`) depending on the motor's position in an X-configuration:
* `esc_1` (Front Right) = Throttle + Pitch - Roll
* `esc_2` (Front Left) = Throttle + Pitch + Roll
* `esc_3` (Rear Left) = Throttle - Pitch - Roll
* `esc_4` (Rear Right) = Throttle - Pitch + Roll

*(Note: The code uses `Servo.write()` to send signals, which may require value mapping depending on the specific ESC protocol).*

---

## ⚠️ Known Issues (Why the project is on pause)

If you plan to unpause and fork this project, pay attention to the following architectural flaws that prevent stable flight:

1.  **Scope Bug in `calc_pid`:** The variables `integral_` and `prev_err` are declared locally inside the function. They reset to `0.0` on every single loop iteration. Because of this, the Integral (I) and Derivative (D) terms **do not work**. *Solution: declare them as `static` or move them to the global scope.*
2.  **Unused Yaw Control:** The PID for the Yaw axis is calculated in `calculatePID()`, but it is never integrated into the final motor mixing math in the `loop()` function.
3.  **Loop Time Limitations:** The main loop uses a `delay(20)`, limiting the update frequency to ~50 Hz. Modern flight controllers run at 250 Hz to 1000+ Hz for stable flight.
4.  **Using `Servo.h` for ESCs:** The `Servo` library generates a 50Hz PWM signal. For quadcopters, it is much better to use direct hardware PWM generation or faster protocols like OneShot for rapid motor response.

---

## 🚀 Setup & Testing

1. Connect the MPU6050 (SDA to A4, SCL to A5).
2. Connect the ESCs to pins D6, D8, D10, D12. **ALWAYS remove propellers** during bench testing!
3. Upload the sketch to your Arduino.
4. Open the Serial Monitor (115200 baud).
5. The controller will perform a multi-second calibration on startup. Do not move the drone during this time!
6. Send a numeric value via the Serial Monitor to set the `base_throttle`.