#include <Wire.h>
#include <MPU9250.h>

// =============================================================================
// WHEELED REVERSE PENDULUM - SELF BALANCING CONTROLLER  (v2)
// =============================================================================
// IMU-only startup balancing (no encoders yet). GY-91 (MPU9250) + IBT-2 driver.
//
// Changes vs v1 (the cause of the overshoot / head-bashing):
//   * REMOVED the per-cycle slew limiter (MAX_PWM_STEP). It added ~half a
//     fall-time-constant of phase lag and made the motors unable to reverse
//     in time -> limit cycle.
//   * ADDED motor deadband feed-forward so small corrections actually move
//     the geared motors instead of falling into the stiction dead band.
//   * RAISED the PWM ceiling (real authority). Watch motor heat & current.
//   * Empirical BALANCE_SETPOINT (the angle where the bot truly balances on
//     its wheels), and a dead-zone moved OFF the sensor-noise floor.
//   * Fixed-rate control loop (constant dt) for predictable gains.
//   * Safety cutoff if it has clearly fallen.
// =============================================================================

MPU9250 imu;

// ---- Motor driver pins (IBT-2 H-Bridge) ------------------------------------
#define LEFT_MOTOR_FORWARD_PIN  5
#define LEFT_MOTOR_REVERSE_PIN  6
#define RIGHT_MOTOR_FORWARD_PIN 9
#define RIGHT_MOTOR_REVERSE_PIN 10

// ---- IMU calibration -------------------------------------------------------
const float GYRO_X_OFFSET     = 3.89f;   // deg/s, removes gyro bias (well-calibrated per logs)
const float PITCH_ZERO_OFFSET = -9.20f;  // deg, makes accel-pitch ~0 at mechanical level

// ---- Control setpoint ------------------------------------------------------
// IMPORTANT: this is the angle (after offset) where the bot actually balances
// on its wheels with NO tendency to tip. From your static log, rest sat at
// ~+0.76 deg, so 0.0 was biased. Find the true value on the wheels (see the
// tuning notes) and put it here.
float BALANCE_SETPOINT = 0.76f;          // deg  <-- CALIBRATE on the wheels

// ---- PID gains -------------------------------------------------------------
// Start as a PD controller (Ki = 0). Add a tiny Ki only after PD balances.
float Kp = 1.6f;   // proportional  (deg      -> PWM)
float Kd = 1.8f;   // derivative    (deg/s    -> PWM)
float Ki = 0.0f;   // integral      (deg*s    -> PWM)  keep 0 until PD works

// ---- Output mapping --------------------------------------------------------
const int   MAX_PWM        = 100;   // ceiling, 0-255. Raise CAREFULLY (heat/current). v1 was 50.
const int   MOTOR_DEADBAND = 28;    // <-- MEASURE: lowest PWM that just spins the loaded wheels
const float OUT_DEADZONE   = 0.5f;  // ignore PD outputs smaller than this (PWM units)

// ---- Angle dead-zone (moved off the noise floor: rest jitter is ~0.15 deg) -
const float ANGLE_DEADZONE = 0.30f; // deg : below this error AND slow -> coast
const float RATE_DEADZONE  = 5.0f;  // deg/s

// ---- Safety ----------------------------------------------------------------
const float FALL_CUTOFF_DEG = 30.0f; // |pitch| beyond this = it fell, stop fighting

// ---- Fixed-rate loop -------------------------------------------------------
const unsigned long CONTROL_PERIOD_US = 5000;   // 5 ms = 200 Hz
const float DT = CONTROL_PERIOD_US * 1e-6f;     // constant dt for the controller

// ---- State -----------------------------------------------------------------
float pitch = 0.0f;
float integral = 0.0f;
unsigned long lastControlMicros = 0;

// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE_PIN, OUTPUT);
  motorRaw(0);

  Wire.begin();
  if (!imu.setup(0x68)) {
    Serial.println("ERROR: IMU init failed");
    while (1) { motorRaw(0); delay(1000); }
  }

  delay(500);
  while (!imu.update()) {}

  // Seed the filter from the accelerometer so we don't start from 0.
  pitch = accelPitch();
  lastControlMicros = micros();

  Serial.println("BALANCE_V2_READY");
}

// =============================================================================
// Motor helpers
// =============================================================================
void motorRaw(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, pwm);  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, pwm); analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  } else if (pwm < 0) {
    pwm = -pwm;
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);  analogWrite(LEFT_MOTOR_REVERSE_PIN, pwm);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0); analogWrite(RIGHT_MOTOR_REVERSE_PIN, pwm);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0); analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  }
}

// Map a control effort 'u' (PWM-ish units) to PWM, jumping past the motor
// dead band so even small efforts produce motion. No slew limiting here.
void driveControl(float u) {
  int pwm = 0;
  if (fabs(u) > OUT_DEADZONE) {
    float mag = MOTOR_DEADBAND + fabs(u);
    pwm = (int)constrain(mag, 0.0f, (float)MAX_PWM);
    if (u < 0) pwm = -pwm;
  }
  motorRaw(pwm);
}

// =============================================================================
// Angle estimation
// =============================================================================
float accelPitch() {
  float a = atan2(imu.getAccX(), imu.getAccZ()) * 180.0f / PI;
  return a - PITCH_ZERO_OFFSET;
}

// =============================================================================
// MAIN LOOP - fixed 200 Hz cadence
// =============================================================================
void loop() {
  imu.update();   // refresh latest sample (non-blocking)

  unsigned long now = micros();
  if ((now - lastControlMicros) < CONTROL_PERIOD_US) return;  // wait for next tick
  lastControlMicros += CONTROL_PERIOD_US;

  // --- Estimate ---
  float gyroRate = imu.getGyroX() - GYRO_X_OFFSET;            // deg/s about pitch axis
  float aPitch   = accelPitch();
  pitch = 0.99f * (pitch + gyroRate * DT) + 0.01f * aPitch;   // complementary filter

  float error = pitch - BALANCE_SETPOINT;

  // --- Safety: it has fallen ---
  if (fabs(pitch) > FALL_CUTOFF_DEG) {
    integral = 0.0f;
    motorRaw(0);
    telemetry(error, gyroRate, 0);
    return;
  }

  // --- Integral (with anti-windup); harmless while Ki = 0 ---
  integral += error * DT;
  integral = constrain(integral, -40.0f, 40.0f);   // clamp; tune with Ki

  // --- PID ---  (sign matches v1: positive lean -> drive to catch it)
  float u = -(Kp * error + Kd * gyroRate + Ki * integral);

  // --- Angle dead-zone: coast only when truly settled ---
  if (fabs(error) < ANGLE_DEADZONE && fabs(gyroRate) < RATE_DEADZONE) {
    u = 0.0f;
    integral = 0.0f;   // don't accumulate while parked
  }

  driveControl(u);
  telemetry(error, gyroRate, (int)u);
}

// =============================================================================
void telemetry(float error, float rate, int u) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();
  Serial.print("PITCH "); Serial.print(pitch, 2);
  Serial.print(" ERR ");  Serial.print(error, 2);
  Serial.print(" RATE "); Serial.print(rate, 2);
  Serial.print(" U ");    Serial.println(u);
}
