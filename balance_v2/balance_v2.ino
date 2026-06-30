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

// ---- Encoder pins (Waveshare DCGM-3865, connector silkscreen "M V A B G M")-
// Per motor: A = Hall A (green wire), B = Hall B (yellow wire). The A channel
// goes on an external-interrupt pin so every pulse is caught; B is read inside
// the ISR to get direction (half-quadrature: ~546 counts / output-shaft rev).
// The Uno has only TWO interrupt pins (D2, D3), so each motor's A takes one.
// Encoder V (blue) -> 5V, G (black) -> GND (shared with logic + driver ground).
// NOTE: these counts are plumbing/telemetry only for now -- the control loop is
// still IMU-only. Wire them, confirm the counts move, then add velocity feedback.
#define LEFT_ENC_A   2   // INT0  (green)
#define LEFT_ENC_B   4   // dir   (yellow)
#define RIGHT_ENC_A  3   // INT1  (green)
#define RIGHT_ENC_B  7   // dir   (yellow)

// Per-side count direction. The right motor/encoder is mirror-mounted, so its
// raw counts run opposite the left; flip its sign so BOTH wheels count UP when
// the robot rolls forward. Change these if you re-mount or rewire a side.
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
// Set MOTOR_DEADBAND to ~the larger of those two. Set back to 0 when done.
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
// -getGyroY(). GYRO_Y_BIAS = the raw getGyroY() reading when the robot is DEAD
// STILL (~-9 here). RECALIBRATE per unit: run IMU_TEST, hold still, read gY.
const float GYRO_Y_BIAS = -9.0f;   // deg/s : raw getGyroY() at rest
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
                   // produced a PWM below the wheels' stiction (see MOTOR_DEADBAND), so it never
                   // caught itself. Re-tune AFTER setting MOTOR_DEADBAND from the deadband test.
float Kd = 0.3f;   // derivative    (deg/s    -> PWM)  RE-TUNED FROM SCRATCH 2026-06-30: old 3.5-4.5 was
                   // tuned vs a DEAD gyro. With the clean rate, RAISING Kd 0.5->1.0 made the ring WORSE,
                   // not better -> we're past peak damping into D-DESTABILIZATION: the 20 Hz on-chip
                   // filter lags a fast (several-Hz) component enough that D pumps it. So Kd goes DOWN.
                   // 0.3 to get below the ring. If a low-Kd ring persists, the culprit is angle-estimate
                   // lag (revert gyro DLPF 20->41 Hz + remove the phantom bias in software instead).
float Ki = 0.0f;   // integral      (deg*s    -> PWM)  keep 0 until PD works

// ---- Velocity & position feedback (encoders, DIRECT) -----------------------
// Added straight to the motor command (NOT via a lean setpoint). The cascade
// (outer loop -> lean) is the textbook structure and gives stronger braking in
// principle, but on this non-minimum-phase plant it was unstable to flash-tune
// (limit-cycled / ran away). This direct form was the known-stable one: it
// balances, holds position, and recovers from nudges; its only weakness is
// gentle braking on a slow drag. Revisit the cascade later with a real model/LQR.
//   u += Kv*forwardVel   brakes wheel velocity (viscous; verified sign)
//   u -= Kx*positionRev  returns toward home  (-= : non-minimum-phase, clamped)
float Kv = 5.0f;            // forward wheel velocity (rev/s -> PWM)  raise to damp drift harder
float Kx = 4.0f;            // wheel position (rev from home -> PWM)  raise to return home harder
const float POS_CLAMP = 20; // PWM cap on the position term so it can't overpower the angle loop

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
const int   MOTOR_DEADBAND = 11;    // feed-forward kick past stiction (free-spin floor ~11 L / ~13 R).
                                    // Dropped 14->11: the larger kick made a ~28-PWM jump through zero
                                    // that fed a sustained limit cycle. 11 softens that bang-bang while
                                    // still moving the wheels (Kp adds on top for real corrections).
                                    // Raise if small corrections stall; lower if it still bang-bangs.
const float OUT_DEADZONE   = 0.5f;  // ignore PD outputs smaller than this (PWM units)

// ---- Angle dead-zone (moved off the noise floor: rest jitter is ~0.15 deg) -
const float ANGLE_DEADZONE = 0.15f; // deg : below this error AND slow -> coast. Tightened 0.30->0.15
                                    // so it starts correcting earlier -> smaller gap before the kick.
const float RATE_DEADZONE  = 5.0f;  // deg/s

// ---- Safety ----------------------------------------------------------------
const float FALL_CUTOFF_DEG = 30.0f; // trip the fall latch: |pitch| beyond this = it fell, stop fighting
const float FALL_REARM_DEG  = 8.0f;  // re-arm ONLY when the absolute accel angle is back within this of
                                     // the setpoint -- a gyro-drifting pitch estimate on its back can't
                                     // restart the motors, only physically standing it up does.

// Stall cutoff: if the controller is pushing hard (|u| big) but the wheels are
// NOT turning (|VF| ~ 0) for a sustained time, the bot is wedged/stuck and the
// motors are just stalling (heat, current). Cut them until it frees up. A real
// balancing wheel always moves while u is large, so only a genuine stall trips
// this. Set STALL_CUTOFF to 0 to disable.
#define STALL_CUTOFF 1
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
const float D_LPF        = 0.80f;   // EMA weight on the previous filtered rate (~20 ms time constant).
                                    // Raised 0.60->0.80 (2026-06-30): residual ~+/-1 deg sway near level
                                    // is the D term chasing a fast vibration component (RATE swung +/-45
                                    // while pitch only swung +/-1). Smoothing the D PATH ONLY cuts that
                                    // without weakening slow-motion damping or lagging the angle estimate.

// ---- State -----------------------------------------------------------------
float pitch = 0.0f;
float integral = 0.0f;
float dRateFilt = 0.0f;   // low-pass-filtered gyro rate for the D term (angle est. uses the raw rate)
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
float positionRev = 0.0f;                     // avg wheel position, revs from home
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
  while (!imu.update()) {}

  // Seed the filter from the accelerometer so we don't start from 0.
  pitch = accelPitch();
  lastControlMicros = micros();
  readEncoders(velLastTicksL, velLastTicksR);   // baseline so the first velocity sample isn't a spike

  Serial.println("BALANCE_V2_READY");
#if ENCODER_TEST
  Serial.println("ENCODER_TEST mode: motors OFF. Roll each wheel by hand "
                 "(1 turn = ~546 counts = 1.00 rev).");
#elif DEADBAND_TEST
  Serial.println("DEADBAND_TEST mode: wheels OFF THE GROUND. Ramping PWM; "
                 "note 'first-move L@/R@' -> set MOTOR_DEADBAND to ~the larger.");
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
// Encoders (half-quadrature: count RISING edges of A, read B for direction)
// =============================================================================
// Keep ISRs tiny: one digitalRead + one increment. On the Uno a digitalRead is
// a few microseconds, fine at these pulse rates. B gives the raw direction;
// ENC_*_DIR flips it per side so forward = positive on both wheels.
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

  imu.update();   // refresh latest sample (non-blocking)

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
  float gyroRate = -(imu.getGyroY() - GYRO_Y_BIAS);          // deg/s, forward-lean +. Pitch rate is on
                                                             // the Y gyro (negated) -- see GYRO_Y_BIAS.
  float aPitch   = accelPitch();
  pitch = 0.99f * (pitch + gyroRate * DT) + 0.01f * aPitch;   // complementary filter (raw rate)
  dRateFilt = D_LPF * dRateFilt + (1.0f - D_LPF) * gyroRate;  // smoothed rate for the D term only

  float error = pitch - BALANCE_SETPOINT;

  // --- Safety: fall latch (hysteresis) ---
  // Trip on the filtered pitch; re-arm ONLY when the absolute accel angle is back
  // near the setpoint. On its back the gyro-based pitch drifts and can dip under
  // the cutoff -> without the latch the motors flail. Keying re-arm off the
  // accelerometer means only physically standing it up restarts them.
  if (!fallen && fabs(pitch) > FALL_CUTOFF_DEG) fallen = true;
  if (fallen && fabs(aPitch - BALANCE_SETPOINT) < FALL_REARM_DEG) {
    fallen = false;
    pitch = aPitch;                          // reseed the filter from the true angle
    long hl, hr; readEncoders(hl, hr);
    homeTicksSum = hl + hr;                  // re-home here so it doesn't drive off to the old spot
    integral = 0.0f;
  }
  if (fallen) {
    integral = 0.0f;
    motorRaw(0);
    telemetry(error, gyroRate, 0);
    return;
  }

  // --- Integral (with anti-windup); harmless while Ki = 0 ---
  integral += error * DT;
  integral = constrain(integral, -40.0f, 40.0f);   // clamp; tune with Ki

  // --- PID inner loop (D on the low-passed rate so gyro spikes don't kick) ---
  float u = -(Kp * error + Kd * dRateFilt + Ki * integral);

  // --- Velocity damping (direct): brake wheel drift. +Kv*VF, verified sign. ---
  u += Kv * forwardVel;

  // --- Position hold (direct): return toward home. -= for the non-minimum-phase
  //     plant (to go home the wheels first go the "wrong" way). Clamped. ---
  u -= constrain(Kx * positionRev, -POS_CLAMP, POS_CLAMP);

  // --- Angle dead-zone: coast only when truly settled (and not drifting) ---
  if (fabs(error) < ANGLE_DEADZONE && fabs(gyroRate) < RATE_DEADZONE
      && fabs(forwardVel) < VEL_DEADZONE) {
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
  Serial.print(" U ");    Serial.print(u);
  Serial.print(" SET ");  Serial.print(sweepIdx + 1);  // active sweep set (0 if sweep disabled)
  Serial.print(" HZ ");   Serial.print(controlHz);     // achieved loop rate; expect ~200
  long lEnc, rEnc; readEncoders(lEnc, rEnc);            // wheel ticks since boot (sanity-check wiring)
  Serial.print(" LENC "); Serial.print(lEnc);
  Serial.print(" RENC "); Serial.print(rEnc);
  Serial.print(" VL ");   Serial.print(wheelVelL, 2);     // left wheel rev/s (filtered)
  Serial.print(" VR ");   Serial.print(wheelVelR, 2);     // right wheel rev/s (filtered)
  Serial.print(" VF ");   Serial.print(forwardVel, 2);    // chassis forward rev/s (mean)
  Serial.print(" POS ");  Serial.println(positionRev, 2); // avg wheel position, revs from home
}
