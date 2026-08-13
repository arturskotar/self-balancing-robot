#include <Wire.h>
#include <MPU9250.h>
#include "crsf.h"     // ELRS/CRSF radio on Serial1: arm kill-switch + live gain tuning

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
// Teensy 4.1 map (see MIGRATION_TEENSY.md §3.1): LEFT = 6/9 (FlexPWM2.2 A/B),
// RIGHT = 22/23 (FlexPWM4.0 / FlexPWM4.1). Configure all four pins explicitly.
#define LEFT_MOTOR_FORWARD_PIN   6
#define LEFT_MOTOR_REVERSE_PIN   9
#define RIGHT_MOTOR_FORWARD_PIN  22
#define RIGHT_MOTOR_REVERSE_PIN  23
const int MOTOR_PWM_HZ = 20000;  // IBT-2/BTS7960 supports PWM up to 25 kHz

// ---- Encoder pins (Waveshare DCGM-3865, connector silkscreen "M V A B G M")-
// Teensy 4.1 map (see MIGRATION_TEENSY.md §3.1). Per motor: A = Hall A (green),
// B = Hall B (yellow). Still using attachInterrupt on A + reading B for direction
// (half-quadrature: ~546 counts / output-shaft rev) -- works on ANY Teensy pin,
// no D2/D3 limit anymore. Pins 2/3/4/5 are ALSO hardware-QuadEncoder-capable, so
// the future x4 upgrade (2184 counts/rev) reuses these same pins.
// POWER (Teensy!): Encoder V (blue) -> 3.3V, G (black) -> GND. NOT 5V -- the
// built-in pull-up would then drive 5V into a 3.3V pin and damage the Teensy.
#define LEFT_ENC_A   2   // green
#define LEFT_ENC_B   3   // dir (yellow)
#define RIGHT_ENC_A  4   // green
#define RIGHT_ENC_B  5   // dir (yellow)

// Per-side count direction. Validated on this chassis: physical forward logs as
// negative wheel motion, physical reverse as positive. Keep both wheels using
// the same chassis convention so the velocity loop can command forward as a
// negative target and reverse as a positive target.
#define ENC_LEFT_DIR   (+1)
#define ENC_RIGHT_DIR  (-1)

// Encoder geometry (for when we convert ticks -> wheel velocity):
//   13 PPR base (26-pole magnet ring) * 42:1 gearbox = 546 rising edges of A
//   per output-shaft revolution. (Full 4x quadrature would be 2184, but that
//   needs interrupts on both channels of both motors -- more than the Uno's 2.)
const long ENC_COUNTS_PER_REV = 546;

// ---- Encoder test mode -----------------------------------------------------
// Set to 1 to bench-check the encoder wiring: motors are held OFF so the wheels
// spin freely, and the loop streams just the counts (and the implied number of
// revolutions). Roll a wheel one full turn -> its count should change by ~546
// and the "rev" figure by ~1.00. Set back to 0 to restore normal balancing.
#define ENCODER_TEST 0

// ---- Motor deadband test ---------------------------------------------------
// Set to 1 to MEASURE the stiction PWM (the lowest PWM that actually turns each
// loaded wheel). Put the robot on a stand so the wheels spin free, flash, and
// watch the monitor: it ramps PWM up and reports "first-move L@<pwm> R@<pwm>".
// Set LEFT_DEADBAND/RIGHT_DEADBAND from their respective results, then disable.
#define DEADBAND_TEST 0

// ---- IMU sign/axis test ----------------------------------------------------
// Set to 1 to verify the gyro vs accelerometer convention (motors OFF). Hold the
// robot and SLOWLY tilt it forward then back. Expected (consistent) result:
//   * aPitch RISES when you tilt FORWARD, falls when you tilt BACK
//   * gX (the pitch-axis gyro) goes POSITIVE while tilting forward, negative back
//   * gX is the axis that moves most; gY/gZ stay near 0
// If gX has the OPPOSITE sign to aPitch's motion -> the gyro sign is flipped
// (negate gyroRate in the code). If gY or gZ is the one that responds instead ->
// the IMU library remapped axes (use that getter). Set back to 0 when done.
#define IMU_TEST 0

// ---- IMU calibration -------------------------------------------------------
// PITCH-AXIS GYRO: the MPU9250 lib (0.4.8) reports pitch-axis rotation on the Y
// gyro, inverted vs the accel-pitch convention (confirmed with IMU_TEST: tilting
// forward, aPitch rises while gY goes negative; gX stays ~0). So the rate is
// -getGyroY(). Measure its zero-rate bias at startup while the motors are off.
const float GYRO_Y_BIAS_FALLBACK = -9.0f;
const int   GYRO_CAL_SAMPLES = 400;
const float GYRO_CAL_MAX_STDDEV = 2.0f;  // reject calibration if the robot is moving
float gyroYBias = GYRO_Y_BIAS_FALLBACK;
const float PITCH_ZERO_OFFSET = -9.20f;  // deg, makes accel-pitch ~0 at mechanical level

// ---- Control setpoint ------------------------------------------------------
// IMPORTANT: this is the angle (after offset) where the bot actually balances
// on its wheels with NO tendency to tip. From your static log, rest sat at
// ~+0.76 deg, so 0.0 was biased. Find the true value on the wheels (see the
// tuning notes) and put it here.
float BALANCE_SETPOINT = -3.22f;         // deg : measured after rigidly mounting the IMU (now steady &
                                         // repeatable, ~0.2deg wander). Fine-tune: forward lower, back raise.

// ---- PID gains -------------------------------------------------------------
// Start as a PD controller (Ki = 0). Add a tiny Ki only after PD balances.
float Kp = 2.00f;  // proportional  (deg -> PWM)  raised 0.85 -> 2.0: at low Kp a few-degree lean
                   // produced a PWM below wheel stiction, so it never caught itself. Re-tune only
                   // after measuring LEFT_DEADBAND and RIGHT_DEADBAND with the deadband test.
float Kd = 0.3f;   // derivative    (deg/s    -> PWM)  RE-TUNED FROM SCRATCH 2026-06-30: old 3.5-4.5 was
                   // tuned vs a DEAD gyro. With the clean rate, RAISING Kd 0.5->1.0 made the ring WORSE,
                   // not better -> we're past peak damping into D-DESTABILIZATION: the 20 Hz on-chip
                   // filter lags a fast (several-Hz) component enough that D pumps it. So Kd goes DOWN.
                   // 0.3 to get below the ring. If a low-Kd ring persists, the culprit is angle-estimate
                   // lag (revert gyro DLPF 20->41 Hz + remove the phantom bias in software instead).
float Ki = 0.0f;   // integral      (deg*s    -> PWM)  keep 0 until PD works

// ---- Cascade outer loop (encoders -> desired LEAN) -------------------------
// RC drive asks directly for a lean setpoint, and the inner pitch loop drives
// the wheels to catch that lean. When not driving, a small position spring
// returns to the current home spot. This keeps forward/backward motion in the
// balance loop and avoids direct common-mode motor feedforward.
//   driveLean = -driveStick * driveMaxLean
//   leanRaw   = driveLean + idlePositionLean
//   error   = pitch - (BALANCE_SETPOINT + leanCmd)
// TUNING: start gentle. driveMaxLean is the full-stick lean request; Kpos is
// the idle return-home spring.
// If it limit-cycles, SLOW the outer loop (raise LEAN_LPF) before cutting gains.
float Kpos = 1.0f;               // wheel position (rev from home -> deg of lean)
float driveMaxLean = 2.0f;       // deg at full drive stick; tune with SC=2 + S1.
const float LEAN_CLAMP = 8.0f;   // deg : cap the commanded lean so the outer loop can't tip it over
const float LEAN_LPF   = 0.99f;  // EMA on leanCmd (~500 ms). Higher = slower, safer outer loop.
                                 // Keep direct stick-to-lean changes slower than the pitch loop so
                                 // the robot eases into a commanded lean instead of stepping the
                                 // balance target abruptly.

// ---- Auto PID sweep (for recording) ----------------------------------------
// Cycles Kp/Kd through a grid, holding each set for SWEEP_HOLD_MS so you can
// record the robot's behavior set-by-set. A banner prints on every change, and
// every telemetry line carries the active set number ("SET n").
// Set SWEEP_ENABLED to 0 to disable the sweep and use the fixed Kp/Kd above.
#define SWEEP_ENABLED 0
const unsigned long SWEEP_HOLD_MS = 30000;   // 30 s per set
struct Gains { float kp; float kd; };
const Gains SWEEP[] = {        // higher-Kp (proportional-dominant) regime, paired with damping
  {1.50f, 2.50f}, {1.50f, 3.25f},
  {2.25f, 2.50f}, {2.25f, 3.25f},
  {3.00f, 2.50f}, {3.00f, 3.25f},
};
const int SWEEP_N = sizeof(SWEEP) / sizeof(SWEEP[0]);
int sweepIdx = -1;                 // active set (-1 = not started yet)
unsigned long sweepLastMs = 0;

// ---- Output mapping --------------------------------------------------------
const int   MAX_PWM        = 110;   // ceiling, 0-255. Raised 80->110 for authority to recover large
                                    // backward falls. WATCH motor heat/current as you push this up.
const float OUT_DEADZONE   = 0.5f;  // ignore PD outputs smaller than this (PWM units)

// ---- Per-side stiction kick -------------------------------------------------
// Compensate only for the measured per-wheel dead zone. A larger permanent floor
// turns small PID sign changes into full-power reversals and creates a limit cycle.
// Any extra loaded breakaway authority must be a short pulse, not a steady floor.
const int   LEFT_DEADBAND  = 11;    // pins 6/9; keep sides equal until loaded response is measured
const int   RIGHT_DEADBAND = 11;    // pins 22/23
const int   BREAKAWAY_PWM  = 35;    // short loaded-wheel kick; never a steady minimum
const unsigned long BREAKAWAY_TIME_US  = 70000;   // 70 ms kick
const unsigned long BREAKAWAY_REARM_US = 120000;  // must be idle 120 ms before another kick

struct BreakawayState {
  bool active = false;
  int kickSign = 0;
  unsigned long idleSinceUs = 0;
  unsigned long kickUntilUs = 0;
};
BreakawayState leftBreakaway, rightBreakaway;

// ---- RC drive (radio -> motion) --------------------------------------------
const float TURN_AUTHORITY = 25.0f; // PWM-effort differential at full turn stick
const float DRIVE_STICK_DEADZONE = 0.15f;
const float TURN_STICK_DEADZONE  = 0.30f; // measured cross-axis reaches ~0.25 during straight drive
const float DRIVE_SIGN     = +1.0f; // flip to -1 if the drive stick drives the wrong way
const float TURN_SIGN      = +1.0f; // flip to -1 if the turn stick steers the wrong way

// ---- Coast band (motor protection): rest the motors near balance -----------
// Within ANGLE_DEADZONE of the (leaned) target AND wheels stopped -> command 0.
// WHY 0.15 -> 1.0 (2026-06-30): the residual jitter is motor vibration on the
// gyro (RATE swings +/-35 while pitch moves +/-1). With a tight 0.15 band the
// motors corrected EVERY cycle -> always energized at the deadband floor ->
// constant current, heat, battery drain, wear. A 1.0 deg band lets the bot
// COAST (motors fully off) when essentially balanced; it only kicks when it
// drifts past 1 deg. Trade-off: a slow ~+/-1 deg rock instead of a constant
// buzz. Raise toward 1.5 if it still buzzes; lower if the rock gets lurchy.
const float ANGLE_DEADZONE = 1.0f;  // deg : |error| below this (and wheels stopped) -> coast
const float RATE_DEADZONE  = 5.0f;  // deg/s : NO LONGER gates the coast (gyro vibration would keep it
                                    // from ever engaging). Kept for reference / future use.

// ---- Safety ----------------------------------------------------------------
const float FALL_CUTOFF_DEG = 30.0f; // trip the fall latch: |pitch| beyond this = it fell, stop fighting
const float FALL_REARM_DEG  = 8.0f;  // re-arm ONLY when the absolute accel angle is back within this of
                                     // the setpoint -- a gyro-drifting pitch estimate on its back can't
                                     // restart the motors, only physically standing it up does.

// Stall cutoff: if the controller is pushing hard (|u| big) but the wheels are
// NOT turning (|VF| ~ 0) for a sustained time, the bot is wedged/stuck and the
// motors are just stalling (heat, current). Cut them until it frees up.
// DISABLED 2026-06-30: the premise "a balancing wheel always moves while u is
// large" is FALSE for the cascade -- it leans and HOLDS, i.e. moderate u with
// near-zero wheel motion is normal. So it false-tripped at PITCH -1.25 (U~9,
// VF~0). Worse, it DEADLOCKS: once cut, VF stays 0 and the bot falls, so |u|
// grows, the trip condition stays true forever -> guaranteed topple. Re-add
// later with a saturation-level threshold (|u| near MAX_PWM) AND a non-latching
// design that re-arms off the absolute accel angle, like the fall latch does.
#define STALL_CUTOFF 0
const float         STALL_U_MIN   = 5.0f;     // |u| above this counts as "pushing hard"
const float         STALL_VEL_MAX = 0.04f;    // rev/s : below this the wheels are "not moving"
const unsigned long STALL_TIME_US = 1000000;  // both conditions must persist this long (1 s)

// ---- Fixed-rate loop -------------------------------------------------------
const unsigned long CONTROL_PERIOD_US = 5000;   // 5 ms = 200 Hz
const float DT = CONTROL_PERIOD_US * 1e-6f;     // constant dt for the controller

// ---- Velocity estimation ---------------------------------------------------
// Per-tick Δticks is coarse (at 5 ms, slow rolling is <1 count), so the raw
// rev/s is heavily low-pass filtered. VEL_LPF closer to 1 = smoother but laggier
// (0.90 -> ~50 ms time constant). VEL_DEADZONE keeps the coast dead-zone from
// engaging while the robot is still drifting (so velocity feedback can act).
const float VEL_LPF      = 0.90f;   // EMA weight on the previous velocity estimate
const float VEL_DEADZONE = 0.05f;   // rev/s : below this the wheels count as "stopped"

// Low-pass on the D-term gyro rate ONLY (the angle estimate still uses raw rate).
// Knocks down single-sample gyro spikes that a high Kd would turn into kicks.
// Higher = smoother D but more phase lag (less effective damping). 0.0 = no filter.
const float D_LPF        = 0.60f;   // EMA weight on the previous filtered rate (~7 ms time constant).
                                    // 0.80 was a REGRESSION (2026-06-30): the extra D-path lag weakened
                                    // damping -> "oscillates a lot". This plant is lag-sensitive (same
                                    // reason high Kd destabilizes). Back to 0.60 = the known-good inner
                                    // loop. The residual fast wiggle is real gyro vibration; the genuine
                                    // cure is a foam/rubber soft-mount under the IMU, not more filtering.

// ---- State -----------------------------------------------------------------
float pitch = 0.0f;
float integral = 0.0f;
float dRateFilt = 0.0f;   // low-pass-filtered gyro rate for the D term (angle est. uses the raw rate)
float dTermLog = 0.0f;    // derivative contribution, for telemetry
float leanCmd   = 0.0f;   // cascade outer-loop output: the desired lean (deg) added to BALANCE_SETPOINT
unsigned long lastControlMicros = 0;
int controlHz = 0;   // measured control-loop frequency (should read ~200; if lower, we're falling behind)

// Encoder tick counts since boot (written in ISRs -> must be volatile).
// Sign is arbitrary until calibrated: if a wheel counts DOWN when rolling the
// robot forward, swap that motor's A/B pins or negate in the read below.
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// Wheel-velocity state (rev/s, low-pass filtered). forwardVel is the average of
// the two wheels = the chassis's forward speed; that's what the controller damps.
long  velLastTicksL = 0, velLastTicksR = 0;   // counts at the previous control tick
float wheelVelL = 0.0f, wheelVelR = 0.0f;     // per-wheel rev/s (filtered)
float forwardVel = 0.0f;                      // chassis rev/s = mean of the two
float rotationVel = 0.0f;                     // differential wheel speed; yaw/rotation proxy
float positionRev = 0.0f;                     // avg wheel position, revs from home
float driveLeanLog = 0.0f;                    // latest drive-requested lean angle
int   motorPwmL = 0, motorPwmR = 0;           // signed PWM actually sent to each IBT-2
long  homeTicksSum = 0;                        // (leftTicks+rightTicks) defining "home" (0 = boot spot)
bool  stalled = false;                         // true while the stall cutoff has the motors off
bool  fallen = false;                          // true while the fall latch has the motors off

// =============================================================================
void setup() {
  Serial.begin(115200); // Monitor with `screen /dev/ttyUSB0 115200` -- arduino-cli's monitor
                        // doesn't reliably apply --config baudrate=, which looked like garbled output.
  delay(1500);

  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE_PIN, OUTPUT);
  analogWriteFrequency(LEFT_MOTOR_FORWARD_PIN, MOTOR_PWM_HZ);
  analogWriteFrequency(LEFT_MOTOR_REVERSE_PIN, MOTOR_PWM_HZ);
  analogWriteFrequency(RIGHT_MOTOR_FORWARD_PIN, MOTOR_PWM_HZ);
  analogWriteFrequency(RIGHT_MOTOR_REVERSE_PIN, MOTOR_PWM_HZ);
  motorRaw(0);

  // Encoders: inputs + interrupt on each A channel (Hall outputs are push-pull,
  // PULLUP is just insurance against a floating pin if a wire is loose).
  pinMode(LEFT_ENC_A,  INPUT_PULLUP);
  pinMode(LEFT_ENC_B,  INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR, RISING);

  Wire.begin();
  Wire.setClock(400000);   // 400 kHz I2C (default is 100 kHz) -> ~4x faster IMU reads, fresher data

  // On-chip low-pass: drop the gyro DLPF from the lib default 41 Hz to 20 Hz (accel 45->21 Hz).
  // WHY: motor/gearbox buzz couples into the gyro and RECTIFIES into a DC bias (~+14 deg/s seen
  // while the wheels were frozen and pitch steady -- a phantom, not real rotation). That phantom
  // offsets the complementary filter AND the D term, keeping the motors energized at the stiction
  // floor (buzz). Filtering at the sensor kills the vibration band before it can rectify. 20 Hz is
  // far above the balance dynamics (<5 Hz) so it adds negligible phase lag. Go DLPF_10HZ if needed.
  MPU9250Setting imuSetting;
  imuSetting.gyro_dlpf_cfg  = GYRO_DLPF_CFG::DLPF_20HZ;
  imuSetting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_21HZ;
  if (!imu.setup(0x68, imuSetting)) {
    Serial.println("ERROR: IMU init failed");
    while (1) { motorRaw(0); delay(1000); }
  }

  delay(500);
  if (calibrateGyroBias()) {
    Serial.print("Gyro Y bias calibrated: "); Serial.println(gyroYBias, 3);
  } else {
    Serial.print("Gyro calibration rejected; using fallback: ");
    Serial.println(gyroYBias, 3);
  }
  while (!imu.update()) {}

  // Seed the filter from the accelerometer so we don't start from 0.
  pitch = accelPitch();
  lastControlMicros = micros();
  readEncoders(velLastTicksL, velLastTicksR);   // baseline so the first velocity sample isn't a spike

  crsf::begin();   // ELRS receiver on Serial1 (pins 0/1). Robot only runs when armed (SB up).

  Serial.println("BALANCE_V2_READY");
#if ENCODER_TEST
  Serial.println("ENCODER_TEST mode: motors OFF. Roll each wheel by hand "
                 "(1 turn = ~546 counts = 1.00 rev).");
#elif DEADBAND_TEST
  Serial.println("DEADBAND_TEST mode: wheels OFF THE GROUND. Ramping PWM; "
                 "set LEFT_DEADBAND and RIGHT_DEADBAND from their own first-move values.");
#elif IMU_TEST
  Serial.println("IMU_TEST mode: motors OFF. Hold + slowly tilt FORWARD then BACK. "
                 "aPitch & gX should rise together tilting forward (consistent sign).");
#endif
}

// =============================================================================
// Motor helpers
// =============================================================================
void motorRaw(int pwm) {
  pwm = constrain(pwm, -255, 255);
  motorPwmL = pwm;
  motorPwmR = pwm;
  if (pwm == 0) {
    unsigned long now = micros();
    if (leftBreakaway.active) leftBreakaway.idleSinceUs = now;
    if (rightBreakaway.active) rightBreakaway.idleSinceUs = now;
    leftBreakaway.active = rightBreakaway.active = false;
    leftBreakaway.kickUntilUs = rightBreakaway.kickUntilUs = 0;
  }
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

// Change direction through coast so RPWM and LPWM are never active together.
// The blanking delay is used only for a true powered reversal, not every update.
void motorWriteWheel(int forwardPin, int reversePin, int pwm, int &lastSign) {
  int sign = (pwm > 0) - (pwm < 0);
  if (sign != 0 && lastSign != 0 && sign != lastSign) {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
    delayMicroseconds(2);
  }

  if (pwm > 0) {
    analogWrite(reversePin, 0);
    analogWrite(forwardPin, pwm);
  } else if (pwm < 0) {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, -pwm);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
  }
  lastSign = sign;
}

// Per-wheel PWM (left/right differ for steering). Same polarity as motorRaw(+).
void motorPerWheel(int pwmL, int pwmR) {
  static int lastSignL = 0, lastSignR = 0;
  pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
  pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);
  motorPwmL = pwmL;
  motorPwmR = pwmR;
  motorWriteWheel(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_REVERSE_PIN, pwmL, lastSignL);
  motorWriteWheel(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_REVERSE_PIN, pwmR, lastSignR);
}

// Map per-wheel efforts (balance +/- turn) to PWM, each past its own stiction
// dead band. Below OUT_DEADZONE a wheel is left at 0 (lets the coast band rest it).
int mapEffortToPwm(float effort, int deadband, BreakawayState &state, unsigned long now) {
  if (fabs(effort) <= OUT_DEADZONE) {
    if (state.active) state.idleSinceUs = now;
    state.active = false;
    state.kickUntilUs = 0;
    return 0;
  }

  int sign = effort > 0.0f ? 1 : -1;
  if (!state.active) {
    if (now - state.idleSinceUs >= BREAKAWAY_REARM_US) {
      state.kickSign = sign;
      state.kickUntilUs = now + BREAKAWAY_TIME_US;
    }
    state.active = true;
  }

  int pwm = (int)constrain(deadband + fabs(effort), 0.0f, (float)MAX_PWM);
  bool kickActive = (long)(state.kickUntilUs - now) > 0;
  if (kickActive && sign == state.kickSign) pwm = max(pwm, BREAKAWAY_PWM);
  if (sign != state.kickSign) state.kickUntilUs = 0;
  return sign * pwm;
}

void driveControlDiff(float uL, float uR) {
  unsigned long now = micros();
  int pwmL = mapEffortToPwm(uL, LEFT_DEADBAND, leftBreakaway, now);
  int pwmR = mapEffortToPwm(uR, RIGHT_DEADBAND, rightBreakaway, now);
  motorPerWheel(pwmL, pwmR);
}

// Live gain tuning from the radio. SC (CH7) picks Kp/Kd/driveMaxLean; the S1 knob (CH8)
// sets it, mapped to a SAFE band centered on the shipped value (knob center =
// shipped). "Pickup" style: switching SC does NOT jump a gain -- the knob only
// takes over once you actually MOVE it -- so flipping SC never disturbs a gain.
// Read the sweet spot off telemetry, then bake it into the #-defaults above.
void applyLiveTune() {
  static uint8_t lastSel  = 255;
  static float   lastKnob = -1.0f;
  uint8_t sel = crsf::gainSel();
  float   k   = crsf::knob01();
  if (sel != lastSel) { lastSel = sel; lastKnob = k; return; }  // just switched: wait for movement
  if (fabsf(k - lastKnob) < 0.01f) return;                      // knob idle: leave gain as-is
  lastKnob = k;
  float x = (k - 0.5f) * 2.0f;                                  // -1..+1 about center
  switch (sel) {
    case 0: Kp   = 2.0f + 0.6f * x; break;   // 1.4 .. 2.6  (shipped 2.0)
    case 1: Kd   = 0.3f + 0.2f * x; break;   // 0.1 .. 0.5  (shipped 0.3)
    case 2: driveMaxLean = 2.0f + 1.0f * x; break;   // 1.0 .. 3.0 deg (shipped 2.0)
  }
}

// =============================================================================
// Encoders (half-quadrature: count RISING edges of A, read B for direction)
// =============================================================================
// Keep ISRs tiny: one digitalRead + one increment. On the Uno a digitalRead is
// a few microseconds, fine at these pulse rates. B gives the raw direction;
// ENC_*_DIR flips it per side so both wheels share the chassis convention:
// physical forward = negative, physical reverse = positive.
void leftEncISR()  { leftTicks  += ENC_LEFT_DIR  * (digitalRead(LEFT_ENC_B)  ? -1 : +1); }
void rightEncISR() { rightTicks += ENC_RIGHT_DIR * (digitalRead(RIGHT_ENC_B) ? -1 : +1); }

// Atomic snapshot of both counters (a long is multi-byte on the Uno, so a read
// can tear if an interrupt fires mid-copy -- briefly mask interrupts).
void readEncoders(long &l, long &r) {
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();
}

// Encoder-only bench test: motors stay off, print counts + revolutions @ 10 Hz.
void encoderTestLoop() {
  motorRaw(0);                       // keep the H-bridges quiet so wheels roll free
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();
  long l, r; readEncoders(l, r);
  Serial.print("ENC_TEST  LENC "); Serial.print(l);
  Serial.print(" RENC ");          Serial.print(r);
  Serial.print("  | rev L ");      Serial.print(l / (float)ENC_COUNTS_PER_REV, 2);
  Serial.print(" R ");             Serial.println(r / (float)ENC_COUNTS_PER_REV, 2);
}

// Deadband measurement: slowly ramp PWM, report the PWM at which each wheel
// first turns (its stiction threshold). Run with the wheels off the ground.
void deadbandTestLoop() {
  static bool seeded = false;
  static long baseL = 0, baseR = 0;
  static int  pwm = 0, movedL = 0, movedR = 0;     // 0 = wheel hasn't moved yet
  static unsigned long lastStep = 0;
  if (!seeded) { readEncoders(baseL, baseR); seeded = true; lastStep = millis(); }
  if (millis() - lastStep < 200) return;           // step every 200 ms
  lastStep = millis();

  long l, r; readEncoders(l, r);
  if (!movedL && labs(l - baseL) > 5) movedL = pwm; // record the PWM that broke stiction
  if (!movedR && labs(r - baseR) > 5) movedR = pwm;

  Serial.print("DB_TEST PWM ");        Serial.print(pwm);
  Serial.print(" dL ");                Serial.print(l - baseL);
  Serial.print(" dR ");                Serial.print(r - baseR);
  Serial.print("  | first-move L@");   Serial.print(movedL);
  Serial.print(" R@");                 Serial.println(movedR);

  if ((movedL && movedR) || pwm >= MAX_PWM) { motorRaw(0); return; }  // done: stop ramping
  pwm++;
  motorRaw(pwm);
}

// IMU sign/axis check: motors off, stream the accel-pitch and all three gyro
// axes @ 10 Hz so you can hand-tilt and verify the convention (see IMU_TEST).
void imuTestLoop() {
  motorRaw(0);
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();
  imu.update();
  Serial.print("IMU_TEST aPitch "); Serial.print(accelPitch(), 2);
  Serial.print(" | gX ");           Serial.print(imu.getGyroX(), 2);
  Serial.print(" gY ");             Serial.print(imu.getGyroY(), 2);  // <- pitch-axis rate (used, negated)
  Serial.print(" gZ ");             Serial.print(imu.getGyroZ(), 2);
  Serial.print(" | aX ");           Serial.print(imu.getAccX(), 2);
  Serial.print(" aZ ");             Serial.println(imu.getAccZ(), 2);
}

// Update filtered wheel velocities from the counters. Call once per control
// tick (uses the fixed DT). rev/s = Δticks / counts-per-rev / dt, then EMA.
void updateVelocity() {
  long l, r; readEncoders(l, r);
  float vL_raw = (l - velLastTicksL) / (float)ENC_COUNTS_PER_REV / DT;
  float vR_raw = (r - velLastTicksR) / (float)ENC_COUNTS_PER_REV / DT;
  velLastTicksL = l;
  velLastTicksR = r;
  wheelVelL = VEL_LPF * wheelVelL + (1.0f - VEL_LPF) * vL_raw;
  wheelVelR = VEL_LPF * wheelVelR + (1.0f - VEL_LPF) * vR_raw;
  forwardVel = 0.5f * (wheelVelL + wheelVelR);
  rotationVel = 0.5f * (wheelVelL - wheelVelR);
  positionRev = 0.5f * ((l + r) - homeTicksSum) / (float)ENC_COUNTS_PER_REV;  // revs from home
}

#if STALL_CUTOFF
// Stall detector. Returns true (and latches `stalled`) once the controller has
// been pushing hard (|u| > STALL_U_MIN) while the wheels sit still (|VF| <
// STALL_VEL_MAX) for STALL_TIME_US. Clears as soon as either condition lifts
// (wheels move again, or it stops fighting), so recovery is automatic. Logs the
// edges so the cause is visible in the serial stream.
bool updateStall(float u, unsigned long now) {
  static unsigned long stallStartUs = 0;
  bool fighting = fabs(u) > STALL_U_MIN && fabs(forwardVel) < STALL_VEL_MAX;
  bool wasStalled = stalled;
  if (fighting) {
    if (stallStartUs == 0) stallStartUs = now;
    if (now - stallStartUs > STALL_TIME_US) stalled = true;
  } else {
    stallStartUs = 0;
    stalled = false;
  }
  if (stalled && !wasStalled) Serial.println("STALL: wheels not moving under load -> motors cut");
  if (!stalled && wasStalled) Serial.println("STALL cleared -> resuming");
  return stalled;
}
#endif

// =============================================================================
// Angle estimation
// =============================================================================
float accelPitch() {
  float a = atan2(imu.getAccX(), imu.getAccZ()) * 180.0f / PI;
  return a - PITCH_ZERO_OFFSET;
}

bool calibrateGyroBias() {
  float sum = 0.0f, sumSq = 0.0f;
  int samples = 0;
  unsigned long startedMs = millis();
  while (samples < GYRO_CAL_SAMPLES && millis() - startedMs < 3000) {
    if (!imu.update()) continue;
    float sample = imu.getGyroY();
    sum += sample;
    sumSq += sample * sample;
    samples++;
  }
  if (samples < GYRO_CAL_SAMPLES) return false;

  float mean = sum / samples;
  float variance = max(0.0f, sumSq / samples - mean * mean);
  if (sqrtf(variance) > GYRO_CAL_MAX_STDDEV) return false;
  gyroYBias = mean;
  return true;
}

// =============================================================================
// Auto PID sweep: swap in the next gain set every SWEEP_HOLD_MS (for recording)
// =============================================================================
void updateSweep() {
#if SWEEP_ENABLED
  unsigned long nowMs = millis();
  if (sweepIdx < 0 || (nowMs - sweepLastMs) >= SWEEP_HOLD_MS) {
    sweepIdx = (sweepIdx + 1) % SWEEP_N;
    sweepLastMs = nowMs;
    Kp = SWEEP[sweepIdx].kp;
    Kd = SWEEP[sweepIdx].kd;
    integral = 0.0f;               // start each set clean
    Serial.print("=== SET "); Serial.print(sweepIdx + 1);
    Serial.print("/");        Serial.print(SWEEP_N);
    Serial.print("  Kp=");    Serial.print(Kp, 2);
    Serial.print(" Kd=");     Serial.print(Kd, 2);
    Serial.println("  (30s) ===");
  }
#endif
}

// =============================================================================
// MAIN LOOP - fixed 200 Hz cadence
// =============================================================================
void loop() {
#if ENCODER_TEST
  encoderTestLoop();   // motors off; just stream encoder counts (roll wheels by hand)
  return;
#elif DEADBAND_TEST
  deadbandTestLoop();  // ramp PWM, report each wheel's stiction threshold
  return;
#elif IMU_TEST
  imuTestLoop();       // motors off; stream accel-pitch + gyro axes (hand-tilt to check signs)
  return;
#endif

  imu.update();    // refresh latest sample (non-blocking)
  crsf::update();  // service the radio every loop pass (not just on control ticks)

  unsigned long now = micros();
  if ((now - lastControlMicros) < CONTROL_PERIOD_US) return;  // wait for next tick
  lastControlMicros += CONTROL_PERIOD_US;

  // --- Measure the achieved control-loop rate (ticks per second) ---
  static unsigned long tickCount = 0, lastHzMs = 0;
  tickCount++;
  if (millis() - lastHzMs >= 1000) { controlHz = tickCount; tickCount = 0; lastHzMs = millis(); }

  updateSweep();   // swap in the next gain set every 30 s (no-op if SWEEP_ENABLED is 0)
  updateVelocity(); // refresh filtered wheel velocities (every tick, before the safety return)

  // --- Estimate ---
  float gyroRate = -(imu.getGyroY() - gyroYBias);             // deg/s, forward-lean +. Pitch rate is on
                                                             // the Y gyro (negated) -- see gyroYBias.
  float aPitch   = accelPitch();
  pitch = 0.99f * (pitch + gyroRate * DT) + 0.01f * aPitch;   // complementary filter (raw rate)
  dRateFilt = D_LPF * dRateFilt + (1.0f - D_LPF) * gyroRate;  // smoothed rate for the D term only

  // --- KILL SWITCH: disarmed (SB down) OR link lost -> cut everything ---------
  // A balancer whose IMU is moved/knocked can command full power. SB(down) or the
  // radio off stops the motors instantly. Keep the pitch filter running so it's
  // accurate the moment you re-arm, and re-home while idle so re-arming doesn't
  // lurch back toward an old position.
  if (!crsf::armed()) {
    motorRaw(0);
    integral = 0.0f;
    dTermLog = 0.0f;
    leanCmd  = 0.0f;
    driveLeanLog = 0.0f;
    long hl, hr; readEncoders(hl, hr);
    homeTicksSum = hl + hr;
    telemetry(pitch - BALANCE_SETPOINT, gyroRate, 0);
    return;
  }

  // --- Live gain tuning from the radio (SC selects, S1 knob sets) -------------
  applyLiveTune();

  // --- RC drive/turn command from the radio ----------------------------------
  // Drive stick -> target lean angle; turn stick -> differential. CH3 throttle
  // caps both. With BOTH sticks centered these are 0 and the balancer behaves
  // exactly as before.
  float cap       = 0.35f + 0.65f * crsf::speed();                    // CH3 speed cap 0.35..1.0
  float driveIn   = crsf::drive();
  float turnIn    = crsf::turn();
  if (fabs(driveIn) < DRIVE_STICK_DEADZONE) driveIn = 0.0f;
  if (fabs(turnIn) < TURN_STICK_DEADZONE) turnIn = 0.0f;
  float driveLean = -DRIVE_SIGN * driveIn * cap * driveMaxLean;       // deg; DRV -1 -> positive forward lean
  float turnCmd   = TURN_SIGN  * turnIn  * cap * TURN_AUTHORITY;      // per-wheel PWM-effort diff
  driveLeanLog = driveLean;

  // While driving, RELEASE the position hold: re-home under the robot so the
  // station-keeping term (Kpos*positionRev) can't fight the pilot. positionRev
  // stays ~0, leaving the outer loop as pure drive-lean control below. On release
  // it holds wherever it stopped. (No moving-reference runaway.)
  bool driving = fabs(driveLean) > 0.01f;
  if (driving) {
    long dl, dr; readEncoders(dl, dr);
    homeTicksSum = dl + dr;
  }

  float error = pitch - BALANCE_SETPOINT;

  // --- Safety: fall latch (hysteresis) ---
  // Trip on the filtered pitch; re-arm ONLY when the absolute accel angle is back
  // near the setpoint. On its back the gyro-based pitch drifts and can dip under
  // the cutoff -> without the latch the motors flail. Keying re-arm off the
  // accelerometer means only physically standing it up restarts them.
  if (!fallen && fabs(pitch - BALANCE_SETPOINT) > FALL_CUTOFF_DEG) fallen = true;
  if (fallen && fabs(aPitch - BALANCE_SETPOINT) < FALL_REARM_DEG) {
    fallen = false;
    pitch = aPitch;                          // reseed the filter from the true angle
    long hl, hr; readEncoders(hl, hr);
    homeTicksSum = hl + hr;                  // re-home here so it doesn't drive off to the old spot
    integral = 0.0f;
    leanCmd = 0.0f;                          // drop any stale lean from before the fall
  }
  if (fallen) {
    integral = 0.0f;
    dTermLog = 0.0f;
    motorRaw(0);
    telemetry(error, gyroRate, 0);
    return;
  }

  // --- Cascade OUTER loop: drive stick -> target LEAN command ---
  // No velocity PI here: this test isolates the simple balancer behavior where
  // holding forward means holding a forward lean until the pilot releases it.
  float positionLean = driving ? 0.0f : Kpos * positionRev;
  float leanRaw = driveLean + positionLean;
  leanRaw = constrain(leanRaw, -LEAN_CLAMP, LEAN_CLAMP);
  leanCmd = LEAN_LPF * leanCmd + (1.0f - LEAN_LPF) * leanRaw;

  // Inner-loop error now chases the LEANED target instead of the bare setpoint.
  error = pitch - (BALANCE_SETPOINT + leanCmd);

  // --- Integral (with anti-windup); harmless while Ki = 0 ---
  integral += error * DT;
  integral = constrain(integral, -40.0f, 40.0f);   // clamp; tune with Ki

  // --- PID INNER loop (D on the low-passed rate so gyro spikes don't kick) ---
  // No direct Kv/Kx anymore -- the encoders act through leanCmd above.
  dTermLog = Kd * dRateFilt;
  float u = -(Kp * error + dTermLog + Ki * integral);

  // --- Coast band: rest the motors when essentially balanced ---
  // Gate on the FILTERED pitch (error) and wheel velocity (VF) -- the trustworthy
  // "settled" signals -- NOT on gyroRate, which is corrupted by motor vibration
  // (it would never let the coast engage). If a real tilt develops, error leaves
  // the band within a cycle or two and full control resumes before it can fall.
  // Only coast when truly IDLE -- not while the pilot commands drive/turn, or the
  // small drive lean (often < ANGLE_DEADZONE) gets zeroed here and nothing moves.
  bool commanding = (fabs(driveLean) > 0.01f) || (fabs(turnCmd) > OUT_DEADZONE);
  if (!commanding && fabs(error) < ANGLE_DEADZONE && fabs(forwardVel) < VEL_DEADZONE) {
    u = 0.0f;
    integral = 0.0f;   // don't accumulate while parked
  }

#if STALL_CUTOFF
  // --- Stall cutoff: pushing hard but wheels not turning -> motors off ---
  if (updateStall(u, now)) {
    motorRaw(0);
    telemetry(error, gyroRate, 0);
    return;
  }
#endif

  driveControlDiff(u + turnCmd, u - turnCmd);   // balance effort +/- steering (drive is via the lean)
  telemetry(error, gyroRate, u);
}

// =============================================================================
void telemetry(float error, float rate, float u) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();
  Serial.print("PITCH "); Serial.print(pitch, 2);
  Serial.print(" ERR ");  Serial.print(error, 2);
  Serial.print(" RATE "); Serial.print(rate, 2);
  Serial.print(" DFILT "); Serial.print(dRateFilt, 2);
  Serial.print(" DTERM "); Serial.print(dTermLog, 2);
  Serial.print(" U ");    Serial.print(u, 2);
  Serial.print(" PWML "); Serial.print(motorPwmL);
  Serial.print(" PWMR "); Serial.print(motorPwmR);
  Serial.print(" SET ");  Serial.print(sweepIdx + 1);  // active sweep set (0 if sweep disabled)
  Serial.print(" HZ ");   Serial.print(controlHz);     // achieved loop rate; expect ~200
  long lEnc, rEnc; readEncoders(lEnc, rEnc);            // wheel ticks since boot (sanity-check wiring)
  Serial.print(" LENC "); Serial.print(lEnc);
  Serial.print(" RENC "); Serial.print(rEnc);
  Serial.print(" VL ");   Serial.print(wheelVelL, 2);     // left wheel rev/s (filtered)
  Serial.print(" VR ");   Serial.print(wheelVelR, 2);     // right wheel rev/s (filtered)
  Serial.print(" VF ");   Serial.print(forwardVel, 2);    // chassis forward rev/s (mean)
  Serial.print(" VROT "); Serial.print(rotationVel, 2);    // differential rev/s (yaw proxy)
  Serial.print(" DLEAN "); Serial.print(driveLeanLog, 2);  // drive-stick requested lean angle
  Serial.print(" LEAN "); Serial.print(leanCmd, 2);       // cascade outer-loop lean command (deg)
  Serial.print(" POS ");  Serial.print(positionRev, 2);   // avg wheel position, revs from home
  Serial.print(" | ARM "); Serial.print(crsf::armed() ? "Y" : "-");  // radio kill-switch state
  Serial.print(" Kp ");    Serial.print(Kp, 2);           // live gains (tune with SC + S1 knob)
  Serial.print(" Kd ");    Serial.print(Kd, 2);
  Serial.print(" GBIAS "); Serial.print(gyroYBias, 2);
  Serial.print(" Dmax ");  Serial.print(driveMaxLean, 2);
  Serial.print(" DRV ");   Serial.print(crsf::drive(), 2);   // CH2 drive stick (should move when pushed)
  Serial.print(" TRN ");   Serial.println(crsf::turn(), 2);  // CH1 turn stick
}
