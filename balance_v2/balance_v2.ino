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
//
// DRIVE ARCHITECTURE (rebuilt 2026-08-13 on branch cascade-setpoint-drive):
//   The pilot drives the outer loop's SETPOINT, never the lean and never the
//   motors. Stick -> wheel velocity -> integrated into a position setpoint ->
//   the always-closed outer loop derives the lean -> the inner pitch loop
//   chases it. This is the librobotcontrol / rc_balance (EduMiP) structure.
//   Everything that failed before failed because it bypassed this: direct lean
//   commands are ACCELERATION commands (constant speed needs zero lean), and
//   motor feedforward gets cancelled by the balance loop.
//
// TWO HARDWARE FACTS THAT DROVE THIS REVISION (BTS7960 datasheet, rev 1.1):
//   1. PWM FREQUENCY IS A CONTROL PARAMETER. The driver's turn-on delay swallows
//      short pulses whole -- see the MOTOR_PWM_HZ comment. 20 kHz multiplied the
//      effective motor deadband by 4.5x and was the real "stiction wall".
//   2. 3.3 V LOGIC IS FINE. V_IN(H) is max 2.0 V (INH max 2.15 V), absolute and
//      NOT ratiometric to Vcc, over -40..150 C. The planned 74HCT244 buffer is
//      unnecessary and would not have fixed the drive problem.
// =============================================================================

MPU9250 imu;

// ---- Motor driver pins (IBT-2 H-Bridge) ------------------------------------
// Teensy 4.1 map (see MIGRATION_TEENSY.md §3.1): LEFT = 6/9 (FlexPWM2.2 A/B),
// RIGHT = 22/23 (FlexPWM4.0 / FlexPWM4.1). Configure all four pins explicitly.
#define LEFT_MOTOR_FORWARD_PIN   6
#define LEFT_MOTOR_REVERSE_PIN   9
#define RIGHT_MOTOR_FORWARD_PIN  22
#define RIGHT_MOTOR_REVERSE_PIN  23
// PWM FREQUENCY IS A CONTROL PARAMETER HERE, NOT A COSMETIC ONE. The BTS7960
// has a large input->output TURN-ON DELAY (datasheet 4.2.3): with the IBT-2's
// ~10k slew-rate resistor, tdr(HS) ~5.5 us + tr ~2.5 us ~= 8 us before the
// output even reaches the rail. Any PWM pulse shorter than that produces NOTHING.
//   @ 4482 Hz (223 us period): 1 PWM count = 0.875 us -> ~9 counts to clear 8 us
//   @ 20000 Hz ( 50 us period): 1 PWM count = 0.196 us -> ~41 counts to clear 8 us
// That matches both bench measurements exactly: the deadband measured 11 at
// 4482 Hz, and the wheels needed ~40 to roll after someone set 20 kHz. Running
// 20 kHz multiplied the electrical deadband by 4.46x and ate the entire low-PWM
// band that smooth driving lives in. 4482 Hz is the Teensy default and the
// frequency the known-good balance + the 11/13 deadbands were measured at.
// Set explicitly on all four pins so the value is documented, not inherited.
const int MOTOR_PWM_HZ = 4482;   // do NOT raise without re-running DEADBAND_TEST
const int MOTOR_PWM_BITS = 8;    // analogWrite range is explicitly 0..255 on every channel

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

// Per-side count direction. The right motor/encoder is mirror-mounted, so its
// raw counts run opposite the left. VALIDATED with the motors unpowered on
// 2026-08-13: manually pushing the complete chassis physically forward produced
// negative counts on both wheels with +1/-1. Flip both so:
//   >>> PHYSICAL FORWARD ROLL = POSITIVE COUNTS on both wheels. <<<
// Everything downstream (positionRev, forwardVel, targetVel, posSetpoint) now
// uses "+ = forward" and the control law reads as written. If a wheel counts the
// wrong way after a re-mount/rewire, flip that side here and NOWHERE else.
#define ENC_LEFT_DIR   (-1)
#define ENC_RIGHT_DIR  (+1)

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
// Measures the lowest PWM that actually turns each wheel, ramping 1 count every
// 200 ms and reporting "first-move L@<pwm> R@<pwm>".
//
// >>> RUN IT LOADED. WHEELS ON THE FLOOR, ROBOT UPRIGHT, BEARING ITS OWN WEIGHT.
// >>> Steady it by hand at the TOP so it cannot tip, but let the wheels roll.
//
// The old instruction here said "wheels off the ground", which measures the
// FREE-SPIN floor (~11 L / 13 R on this chassis). That is the wrong number and it
// has been wrong all along: through the 42:1 gearbox, breakaway under the robot's
// own weight is 2-3x higher. The 2026-08-13 drive log brackets it -- PWM 15/17
// produced ZERO encoder counts for seconds, while 34/39 (during a turn) rolled
// the wheels freely. So the loaded value is somewhere in 17..34.
//
// This matters because the balance loop lives in exactly that band: once the
// inner loop reaches its commanded lean, ERR ~ 0.7 deg -> u ~ -1.4 -> PWM ~ 12/14,
// which is BELOW breakaway. The robot then leans and sticks instead of rolling.
//
// This test measures the STATIC breakaway, so set LEFT_DEADBAND_STATIC and
// RIGHT_DEADBAND_STATIC from each wheel's own first-move value. It does NOT
// measure the *_MOVING floors -- those are the free-spin figures, taken with the
// wheels off the ground. Then set this back to 0.
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
// ⚠️ WORTH RE-MEASURING — 2026-08-13 telemetry. With the motors OFF (coast band) the
// robot sat at PITCH ~ -4.03 for 4.5 s, vs this setpoint of -3.22: a 0.8 deg forward
// bias. Inside the coast band that is invisible (|ERR| 0.81 < ANGLE_DEADZONE 1.0,
// motors off), so it only bites once something knocks it out of the band -- then the
// controller chases a target 0.8 deg forward of balance, i.e. creeps forward.
// CAVEAT: that log was a hand-held test, so the robot may have been supported rather
// than free-standing, which would make -4.03 meaningless. Re-measure deliberately:
// FREE-STANDING, disarmed, motors off, undisturbed -- then set this to the rest PITCH.

// ---- PID gains -------------------------------------------------------------
// Start as a PD controller (Ki = 0). Add a tiny Ki only after PD balances.
float Kp = 2.00f;  // proportional  (deg -> PWM)  raised 0.85 -> 2.0: at low Kp a few-degree lean
                   // produced a PWM below wheel stiction, so it never caught itself. Re-tune only
                   // after measuring the per-side deadbands with the deadband test.
float Kd = 0.3f;   // derivative    (deg/s    -> PWM)  RE-TUNED FROM SCRATCH 2026-06-30: old 3.5-4.5 was
                   // tuned vs a DEAD gyro. With the clean rate, RAISING Kd 0.5->1.0 made the ring WORSE,
                   // not better -> we're past peak damping into D-DESTABILIZATION: the 20 Hz on-chip
                   // filter lags a fast (several-Hz) component enough that D pumps it. So Kd goes DOWN.
                   // 0.3 to get below the ring. If a low-Kd ring persists, the culprit is angle-estimate
                   // lag (revert gyro DLPF 20->41 Hz + remove the phantom bias in software instead).
float Ki = 0.0f;   // integral      (deg*s    -> PWM)  keep 0 until PD works
// Translation needs turn-class common-mode authority, but neutral balance is
// already tuned. Blend toward this proportional gain with commanded drive only.
const float DRIVE_KP = 4.0f;
// Preserve enough rate damping for the measured ~5 Hz drive mode. At the logged
// 1.5 deg / 47 deg/s oscillation, Kd=0.20 makes D comparable to the drive-mode
// proportional term instead of leaving a broad proportional-only damping hole.
// Larger tracking errors still restore the known-good neutral Kd.
const float DRIVE_KD = 0.20f;
const float DRIVE_D_NEAR_TARGET_LIMIT = 1.5f; // soften near-target D so rate noise cannot kick the static floor
const float DRIVE_KD_RESTORE_START_ERROR = 1.5f;
const float DRIVE_KD_RESTORE_FULL_ERROR  = 3.0f;

// ---- Cascade outer loop (position/velocity SETPOINT -> desired LEAN) -------
// THE PILOT DRIVES THE SETPOINT, NEVER THE LEAN AND NEVER THE MOTORS.
// This is the structure every working reference uses (librobotcontrol's
// rc_balance / EduMiP is the canonical one):
//   posSetpoint += targetVel * DT                       <- stick integrates in
//   leanRaw = Kpos*(posSetpoint - positionRev)
//           + Kvel*(targetVel  - forwardVel)            <- loop ALWAYS closed
//   error   = pitch - (BALANCE_SETPOINT + leanCmd)      <- inner loop untouched
//
// WHY NOT COMMAND LEAN DIRECTLY (the thing that kept failing): on flat ground a
// constant forward VELOCITY needs ZERO lean -- lean commands ACCELERATION. So a
// held drive-lean is a held acceleration command: it runs away until it hits the
// clamp, a wall, or the floor. There is no lean value that means "cruise". The
// outer loop must DERIVE whatever lean is needed. Same reason motor feedforward
// failed: the balance loop cancels any common-mode wheel command.
//
// SAFETY PROPERTY: with targetVel = 0 and posSetpoint parked, this reduces
// EXACTLY to the known-good baseline law  leanRaw = -(Kpos*positionRev +
// Kvel*forwardVel). Standing still, this is the balancer that already worked.
//
// NOTE these gains have never actually run: the encoders were dead (wiring) for
// the whole Teensy era, so positionRev/forwardVel were pinned at 0 and the outer
// loop contributed nothing. Treat Kpos/Kvel as UNVALIDATED starting points.
// If it limit-cycles, SLOW the outer loop (raise LEAN_LPF) before cutting gains.
// >>> DIAGNOSTIC SWITCH (bisection tool, not a fix) <<<
// 0 = outer loop OFF: leanCmd forced to 0, so TARGET == BALANCE_SETPOINT. That is
//     EXACTLY the known-good baseline controller (which also ran with the outer
//     loop inert, since the encoders were dead) but at the corrected 4482 Hz.
// 1 = outer loop ON (normal cascade). DEFAULT.
// Use this to bisect ANY future instability: if it misbehaves with 1 and is clean
// with 0, the fault is in the outer loop; if it misbehaves with both, the fault is
// in the inner loop / motor direction.
// NOTE: all four sign chains (encoder, inner loop, motor polarity, outer loop) were
// verified correct against the 2026-08-13 hand-tilt telemetry -- see git log.
#define OUTER_LOOP 1

float Kpos = 1.0f;               // wheel position error (rev -> deg of lean)
float Kvel = 3.0f;               // wheel-velocity damping; enough drive lean without sitting on the pitch ring
const float KVEL_I = 0.6f;       // slow velocity-error memory (rev/s*s -> deg lean)
const float VEL_I_CLAMP = 0.8f;  // deg; enough to bias through stiction without sustaining a hard lean
// Keep drive velocity feedback at the neutral-loop gain. Doubling it drove
// leanRaw into the +/-5 deg clamp for most held-stick samples, so encoder ripple
// could only move the command away from saturation and was fed back asymmetrically.
const float DRIVE_KVEL_MULTIPLIER = 1.0f;
const float LEAN_CLAMP = 5.0f;   // deg : known-good outer-loop safety cap
const float LEAN_LPF   = 0.99f;  // EMA on leanCmd (~500 ms). Higher = slower, safer outer loop.
                                 // 0.95 (~100 ms) RAN AWAY: faster than the pendulum's non-minimum-
                                 // phase "wrong-way" transient (~100-300 ms), so the outer loop
                                 // re-commanded lean while the inner loop was still driving the
                                 // wheels the wrong way -> positive feedback. KEEP THIS SLOW: it is
                                 // what stops the drive setpoint from exciting the same runaway.
// Anti-windup: never let the setpoint run further than this ahead of where the
// robot actually is. Without it, blocked wheels wind the setpoint up forever and
// the robot lurches when they free up.
// TIGHTENED 2.0 -> 0.5 after the 2026-08-13 stick log. With the wheels unable to
// break away, the setpoint integrated open-loop to PSET 1.003 and STAYED there
// once the stick was released. At Kpos = 1.0 that left a PERMANENT +1.3 deg
// forward lean bias -- TARGET sat at -2.00 against a true balance near -3.3 --
// so the robot would creep forward forever with the sticks centred. Classic
// integrator windup; 2.0 rev of slack was far too much to leave unbounded.
// TRADE-OFF: this also caps the position term at Kpos * POS_ERROR_CLAMP degrees,
// so station-keeping can only correct 0.5 rev of displacement (~10 cm) and any
// release-lurch is bounded to the same. Raise it if station-keeping feels weak
// against a shove -- but only once the wheels reliably move, or the windup and
// the permanent lean bias come straight back.
const float POS_ERROR_CLAMP = 0.5f;   // rev

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

// ---- Per-side stiction compensation ----------------------------------------
// Every non-zero PID effort gets a static floor so the wheel actually moves.
// STILL STATIC, NOT TIME-DEPENDENT: do not reintroduce breakaway pulses into the
// pitch loop -- they turn small post-coast corrections into kicks (that was the
// abandoned `balancing-tuned` branch).
//
// The two wheels do NOT need the same floor. Evidence from the 2026-08-13 log:
// summing per-window encoder deltas, the LEFT wheel ran 21% ahead of the right
// below ~23 PWM, falling to 4% above ~30 PWM -- i.e. the RIGHT wheel needs more
// PWM to break away, and the gap closes once both are moving. That is exactly a
// deadband difference, and it matches the old bench measurement of ~11 L / 13 R.
// Symptom if this is wrong: it VEERS while creeping but tracks straight at speed.
// NOTE this is the opposite side from the comment in rc_drive.ino, which claims
// the pins-6/9 (left) motor is stiffer. The measurement wins; rc_drive is stale.
// TWO floors per wheel, selected by whether that wheel is already turning. This
// is ordinary Coulomb-vs-static friction compensation, and it reconciles two
// measurements that looked contradictory:
//   MOVING (11 / 13) - the free-spin figures from DEADBAND_TEST. They were never
//     wrong, they were just the wrong PARAMETER: they describe a wheel that is
//     already rotating, where only Coulomb friction remains.
//   STATIC (18) - the breakaway needed to get a stopped wheel rolling under the
//     robot's own weight through the 42:1 gearbox.
//
// Measured 2026-08-13, stick-engaged log. Drive commanded 11/13..13/15 and the
// encoders did not move at all; the turn commanded 14..25 on one side and that
// side rolled. Stick-slip is severe -- PWML 19 gave DL 0 one window, then PWML
// 16 gave DL -6 the next -- so treat 18 as approximate, not a clean threshold.
//
// WHY SPLIT INSTEAD OF JUST RAISING THE FLOOR: a single static 18 would make
// EVERY small balance correction jump to 18, and a ~28-PWM swing through zero is
// what fed a limit cycle historically. Splitting keeps small corrections at the
// low floor (the wheels are nearly always dithering while balancing) and spends
// the big kick only where it is actually needed -- breaking a stopped wheel free.
//
// ASYMMETRY RESOLVED by a loaded DEADBAND_TEST, 2026-08-13. The right wheel
// needs nearly DOUBLE the left to break away:
//     PWM 14 -> dL -1   dR -1
//     PWM 15 -> dL -6   dR -1     <- LEFT breaks free, then accelerates cleanly
//     PWM 20 -> dL -39  dR -3
//     PWM 26 -> dL -95  dR -4     <- right still dead
//     PWM 27 -> dL -119 dR -8     <- RIGHT finally breaks free
// The previous equal floors of 18/18 straddled that gap: the left was above its
// threshold and the right below its own. That single fact explains the whole
// drive history -- commands of 13..15 moved nothing, the stick-slip oscillation
// was the LEFT wheel intermittently breaking free near 15..18 while the right
// stayed locked, and turning worked only because the differential pushes one
// side past 27.
//
// ⚠️ 15 vs 27 is NOT normal unit-to-unit variation -- suspect a MECHANICAL fault
// on the right drivetrain (over-tight hub, rubbing tire, misaligned motor mount
// side-loading the shaft, or the gearbox itself). These values compensate for it,
// they do not fix it, and the cost of compensating is that every correction hands
// the right wheel 12 more PWM than the left. If a mechanical fix brings the right
// side back near 15, re-run this test and lower it.
//
// CAVEAT: DEADBAND_TEST ramps positive PWM only, so these are one-direction
// figures. Breakaway can differ by direction (backlash, gearbox preload), and the
// turn logs hint that it does. Worth measuring both ways eventually.
const int LEFT_DEADBAND_MOVING  = 11;
const int RIGHT_DEADBAND_MOVING = 13;
const int LEFT_DEADBAND_STATIC  = 15;
const int RIGHT_DEADBAND_STATIC = 27;
// A wheel counts as "moving" only if it makes NET progress: at least
// WHEEL_MOVE_COUNTS ticks in one direction across a WHEEL_WINDOW_TICKS window.
// NOT "any tick recently" -- that was the first attempt and it was wrong. Under
// stick-slip the encoder emits a steady stream of +1/-1 ticks that sum to zero;
// an any-tick test reads that as "moving" and drops to the low Coulomb floor at
// exactly the moment the wheel is stuck and needs the static breakaway kick.
// Confirmed in the 2026-08-13 oscillation log: DL alternated -1/+1 for four
// seconds, ENC L moved -21 to -31 total, and DBL read 11M the whole time.
// 20 ticks = 100 ms; 3 counts over that window is ~0.055 rev/s of real rolling.
const int WHEEL_WINDOW_TICKS = 20;
const int WHEEL_MOVE_COUNTS  = 3;
// Worst-case floor, used for the saturation-latch effort ceiling so the latch
// can't false-trip on whichever wheel has the smaller usable effort range.
const int MAX_DEADBAND = (LEFT_DEADBAND_STATIC > RIGHT_DEADBAND_STATIC)
                       ? LEFT_DEADBAND_STATIC : RIGHT_DEADBAND_STATIC;

// ---- RC drive (radio -> motion) --------------------------------------------
// Drive stick sets a wheel VELOCITY target (rev/s), which is integrated into the
// outer loop's position setpoint. crsf::drive() is +1 for stick UP (fixed at the
// source in crsf.h on 2026-08-13 -- it used to return -1), and forward is +counts,
// so DRIVE_SIGN = +1 gives stick-up -> forward. Keep DRIVE_SIGN as a pure
// CHASSIS-wiring knob; if the radio axis is ever inverted again, fix crsf.h, not
// this, so the two negations can't quietly cancel.
// STILL UNVERIFIED: the TURN direction. Push the stick RIGHT and confirm the robot
// yaws right; if not, flip TURN_SIGN (turn is a separate channel with its own TX
// direction setting, so it does not follow from the drive fix).
const float DRIVE_MAX_VEL  = 0.6f;  // rev/s at full stick. Conservative on purpose: one encoder
                                    // count per 5 ms tick is already 0.366 rev/s (546 counts/rev),
                                    // so 0.6 rev/s is only ~1.6 counts/tick of resolution. Raise
                                    // this after the x4 hardware QuadEncoder upgrade (2184 cpr).
const float TURN_AUTHORITY = 25.0f; // PWM-effort differential at full turn stick
const float DRIVE_STICK_DEADZONE = 0.15f;
const float TURN_STICK_DEADZONE  = 0.30f; // measured cross-axis reaches ~0.25 during straight drive
const float DRIVE_SIGN     = +1.0f; // flip to -1 if the drive stick drives the wrong way
const float TURN_SIGN      = +1.0f; // flip to -1 if the turn stick steers the wrong way

// ---- Drive launch assist ----------------------------------------------------
// A hand push proves the cascade drives correctly once the wheels are rolling,
// while a stopped launch reaches its 5 deg target and merely rocks around it.
// Wait until that lean is established and nearly stationary, then apply one
// bounded 200 ms axle pulse to cross static friction. Unlike the retired
// velocity-effort path, this cannot persist: every exit enters LAUNCH_DONE and
// a fresh pulse is impossible until the stick is released or changes direction.
#define DRIVE_LAUNCH_ASSIST 1
const float         LAUNCH_ASSIST_EFFORT     = 20.0f; // -35/-47 PWM with measured static floors
const float         LAUNCH_MIN_TARGET_VEL    = 0.12f; // allow launch at the low CH3 speed cap (0.35*Vmax ~= 0.21)
const float         LAUNCH_READY_LEAN_DEG    = 2.5f;  // max required lean; scaled down for smaller requested speeds
const float         LAUNCH_READY_ERROR_DEG   = 1.25f; // body must be close to the leaned target
const float         LAUNCH_READY_RATE_DPS    = 10.0f; // do not launch in the fast part of a pitch swing
const float         LAUNCH_ABORT_ERROR_DEG   = 2.5f;  // immediately return authority to balance
const float         LAUNCH_ABORT_LEAN_DEG    = 2.0f;  // pulse stops if the established lean is lost
const float         LAUNCH_MAX_START_VEL     = 0.06f; // already rolling means the pulse is unnecessary
const float         LAUNCH_RELEASE_VEL       = 0.12f; // rev/s toward command means breakaway succeeded
const unsigned int  LAUNCH_RELEASE_TICKS     = 20;    // require 100 ms at 200 Hz; reject oscillation spikes
const unsigned int  LAUNCH_ASSIST_TICKS      = 40;    // one bounded 200 ms window at 200 Hz

// Gentle post-PD velocity trim. The outer loop still owns drive authority via
// lean; this only prevents a held leaned target from settling with U ~= 0 while
// VERR remains large. Keep it small: the old 8 PWM version made the inner loop
// fight it by building several degrees of angle error.
#define DRIVE_VELOCITY_EFFORT 1
const float DRIVE_VELOCITY_EFFORT_GAIN  = 4.0f; // rev/s error -> PWM effort
const float DRIVE_VELOCITY_EFFORT_LIMIT = 3.0f; // small trim, well below launch floor
const float DRIVE_VELOCITY_HOLD_LEAN_DEG = 0.75f; // hysteresis after the strict 1.5 deg engagement

// ---- Soft start + saturation latch (from the rc_balance reference) ---------
// Soft start: ramp control authority in over SOFT_START_SEC after arming so a
// mid-air / mid-tilt arm doesn't slam the motors to full.
const float SOFT_START_SEC = 0.7f;
// Saturation latch: if the INNER loop sits pinned at its ceiling this long, the
// robot is not recovering -- stop cooking the motors and latch. Re-arm is keyed
// off the absolute accel angle (same as the fall latch), so it CANNOT deadlock
// the way the old STALL_CUTOFF did: only physically standing it up restarts it.
#define SATURATION_LATCH 1
const float         SAT_EFFORT_FRAC  = 0.95f;  // fraction of usable effort range that counts as pinned
const unsigned long SAT_TIMEOUT_TICKS = 100;   // 0.5 s at 200 Hz

// ---- Coast band (motor protection): rest the motors near balance -----------
// Within ANGLE_DEADZONE of the (leaned) target AND wheels stopped -> command 0.
// WHY 0.15 -> 1.0 (2026-06-30): the residual jitter is motor vibration on the
// gyro (RATE swings +/-35 while pitch moves +/-1). With a tight 0.15 band the
// motors corrected EVERY cycle -> always energized at the deadband floor ->
// constant current, heat, battery drain, wear. A 1.0 deg band lets the bot
// COAST (motors fully off) when essentially balanced; it only kicks when it
// drifts past 1 deg. Trade-off: a slow ~+/-1 deg rock instead of a constant
// buzz. Raise toward 1.5 if it still buzzes; lower if the rock gets lurchy.
// This setting has no effect in STATE DRIVE, where coasting is disabled.
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
float velocityLeanI = 0.0f;
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
float positionRev = 0.0f;                     // avg wheel position, revs from home (+ = forward)
float posSetpoint = 0.0f;                     // OUTER-LOOP TARGET position (rev). The drive stick
                                              // integrates into this; the loop always chases it.
float targetVelLog = 0.0f;                    // latest commanded wheel velocity (rev/s)
float turnCmdLog = 0.0f;                      // post-deadzone differential effort
int   motorPwmL = 0, motorPwmR = 0;           // signed PWM actually sent to each IBT-2
long  homeTicksSum = 0;                        // (leftTicks+rightTicks) defining "home" (0 = boot spot)
bool  stalled = false;                         // true while the stall cutoff has the motors off
bool  fallen = false;                          // true while the fall latch has the motors off
bool  wheelMovingL = false, wheelMovingR = false;  // wheel turned within WHEEL_STILL_TICKS
bool  armedPrev = false;                       // edge-detect the arm transition (soft start)
bool  driveCommandPrev = false;                // edge-detect a fresh non-zero drive-stick command
bool  driveVelocityEffortLatched = false;      // strict engage, wider safety limits while active
enum LaunchAssistState : uint8_t { LAUNCH_IDLE, LAUNCH_WAIT_LEAN, LAUNCH_PUSH, LAUNCH_DONE };
LaunchAssistState launchAssistState = LAUNCH_IDLE;
int8_t launchAssistDirection = 0;              // +1 forward target, -1 reverse target
unsigned int launchAssistTicksRemaining = 0;   // wall-clock ticks remaining in the bounded push window
unsigned int launchRollingTicks = 0;            // sustained commanded-direction motion confirmation
unsigned long armedAtMs = 0;                   // millis() at the moment we armed
unsigned long satTicks = 0;                    // consecutive ticks with the inner loop pinned

struct ControlTelemetry {
  float accelPitch = 0.0f;
  float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
  float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
  float targetPitch = BALANCE_SETPOINT;
  float pTerm = 0.0f, iTerm = 0.0f, dTermRaw = 0.0f;
  float effectiveKp = 0.0f, effectiveKd = 0.0f;
  float effortL = 0.0f, effortR = 0.0f;
  float positionLean = 0.0f, velocityLean = 0.0f, velocityLeanIntegral = 0.0f, leanRaw = 0.0f;
  float effectiveKvel = 0.0f;
  float speedInput = 0.0f, commandCap = 0.35f;
  float driveRaw = 0.0f, turnRaw = 0.0f;
  float driveCmd = 0.0f, turnCmd = 0.0f;
  float wheelVelRawL = 0.0f, wheelVelRawR = 0.0f;
  long encoderDeltaL = 0, encoderDeltaR = 0;
  float posError = 0.0f, velError = 0.0f;   // outer-loop tracking errors
  float softStart = 0.0f;                   // 0..1 authority ramp after arming
  float launchBoost = 0.0f;                 // signed change needed to enforce the launch floor
  float driveVelocityEffort = 0.0f;         // persistent, safety-gated velocity torque
  bool dTermLimited = false;
  bool driveVelocityEffortActive = false;
  bool driving = false;
  char launchState = '-';                   // '-' idle, W waiting, P pulse, C completed until release
  char launchEvent = '-';                   // P pulse, T timeout, R rolling, E/B unsafe, N neutral
  bool coasting = true;
};
ControlTelemetry controlLog;

// =============================================================================
void setup() {
  Serial.begin(115200); // Monitor with `screen /dev/ttyUSB0 115200` -- arduino-cli's monitor
                        // doesn't reliably apply --config baudrate=, which looked like garbled output.
  delay(1500);

  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE_PIN, OUTPUT);
  analogWriteResolution(MOTOR_PWM_BITS);
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
  // 20 Hz -> 10 Hz (2026-08-13). The 20 Hz corner sat directly ON the chassis
  // resonance the motor kicks excite, so it passed straight through into the D
  // term. Reconciling the drive-oscillation log: RATE/DFILT swung +/-35..45 deg/s
  // and AP swung -15.5..+8.8 while the FILTERED pitch moved only +/-1 deg -- that
  // is a real but tiny (~0.3 deg) oscillation at roughly 20 Hz, not body motion.
  // It made |D| reach 11.3 against a P term of 0.4..3.2, i.e. 3x to 25x, so the
  // output was essentially pure derivative and reversed sign every sample. The
  // wheels then never got a sustained push in one direction and could not roll
  // even at PWM 40, well past their measured 27 breakaway.
  // COST: DLPF_10HZ adds ~14 ms of group delay vs ~9 ms at 20 Hz. At the balance
  // bandwidth (~1-2 Hz) that is a few degrees of phase, which this plant should
  // tolerate -- but it IS the lag-sensitive direction that made Kd 0.5->1.0 and
  // D_LPF 0.60->0.80 regressions before. If balance degrades, this is the change
  // to back out first, and the honest fix is the IMU soft-mount instead.
  // Accel 21 -> 10 Hz too: it feeds the complementary filter at only 1% weight,
  // but the fall-latch re-arm reads aPitch DIRECTLY, and +/-15 deg of vibration
  // noise on that is enough to re-arm the motors spuriously.
  MPU9250Setting imuSetting;
  imuSetting.gyro_dlpf_cfg  = GYRO_DLPF_CFG::DLPF_10HZ;
  imuSetting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ;
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
  Serial.print("PWM_CFG HZ "); Serial.print(MOTOR_PWM_HZ);
  Serial.print(" BITS "); Serial.print(MOTOR_PWM_BITS);
  Serial.println(" | L 6/9 FlexPWM2_2 A/B | R 22 FlexPWM4_0A 23 FlexPWM4_1A");
  Serial.println("SIGN_CFG TARGET_FWD + ENC_FWD + PWM_FWD -");
#if ENCODER_TEST
  Serial.println("ENCODER_TEST mode: motors OFF. Roll each wheel by hand "
                 "(1 turn = ~546 counts = 1.00 rev).");
#elif DEADBAND_TEST
  Serial.println("DEADBAND_TEST mode: wheels OFF THE GROUND. Ramping PWM; "
                 "set LEFT/RIGHT_DEADBAND_STATIC from 'first-move L@' and 'R@'.");
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

// Map a wheel's effort to PWM through the appropriate friction floor: the STATIC
// (breakaway) figure if that wheel is stopped, the lower MOVING (Coulomb) figure
// once it is already turning. Below OUT_DEADZONE the wheel rests.
int mapEffortToPwm(float effort, bool moving, int dbMoving, int dbStatic) {
  if (fabs(effort) <= OUT_DEADZONE) return 0;
  int deadband = moving ? dbMoving : dbStatic;
  int pwm = (int)constrain(deadband + fabs(effort), 0.0f, (float)MAX_PWM);
  return effort > 0.0f ? pwm : -pwm;
}

void driveControlDiff(float uL, float uR) {
  int pwmL = mapEffortToPwm(uL, wheelMovingL, LEFT_DEADBAND_MOVING,  LEFT_DEADBAND_STATIC);
  int pwmR = mapEffortToPwm(uR, wheelMovingR, RIGHT_DEADBAND_MOVING, RIGHT_DEADBAND_STATIC);
  motorPerWheel(pwmL, pwmR);
}

// Live gain tuning from the radio. SC (CH7) picks Kp/Kd/Kvel; the S1 knob (CH8)
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
    case 2: Kvel = 2.0f + 2.0f * x; break;   // 0.0 .. 4.0 -- range shifted DOWN so the knob can
                                             // reach ZERO and dial the outer loop out live while
                                             // the robot is running. Knob fully CCW = outer
                                             // velocity feedback off.
  }
}

// =============================================================================
// Encoders (half-quadrature: count RISING edges of A, read B for direction)
// =============================================================================
// Keep ISRs tiny: one digitalRead + one increment. On the Uno a digitalRead is
// a few microseconds, fine at these pulse rates. B gives the raw direction;
// ENC_*_DIR flips it per side so both wheels share the chassis convention:
// physical forward = positive, physical reverse = negative.
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
  long deltaL = l - velLastTicksL;
  long deltaR = r - velLastTicksR;
  float vL_raw = deltaL / (float)ENC_COUNTS_PER_REV / DT;
  float vR_raw = deltaR / (float)ENC_COUNTS_PER_REV / DT;
  velLastTicksL = l;
  velLastTicksR = r;
  // Per-wheel "is it turning?" for the friction-floor selection, judged on NET
  // displacement across a window so stick-slip dither (which sums to zero) is not
  // mistaken for rolling. See WHEEL_WINDOW_TICKS.
  static long winStartL = 0, winStartR = 0;
  static int  winTicks  = 0;
  if (++winTicks >= WHEEL_WINDOW_TICKS) {
    wheelMovingL = labs(l - winStartL) >= WHEEL_MOVE_COUNTS;
    wheelMovingR = labs(r - winStartR) >= WHEEL_MOVE_COUNTS;
    winStartL = l;
    winStartR = r;
    winTicks  = 0;
  }

  controlLog.encoderDeltaL += deltaL;
  controlLog.encoderDeltaR += deltaR;
  controlLog.wheelVelRawL = vL_raw;
  controlLog.wheelVelRawR = vR_raw;
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
  controlLog.gyroX = imu.getGyroX();
  controlLog.gyroY = imu.getGyroY();
  controlLog.gyroZ = imu.getGyroZ();
  controlLog.accelX = imu.getAccX();
  controlLog.accelY = imu.getAccY();
  controlLog.accelZ = imu.getAccZ();
  float gyroRate = -(imu.getGyroY() - gyroYBias);             // deg/s, forward-lean +. Pitch rate is on
                                                             // the Y gyro (negated) -- see gyroYBias.
  float aPitch   = accelPitch();
  controlLog.accelPitch = aPitch;
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
    velocityLeanI = 0.0f;
    turnCmdLog = 0.0f;
    controlLog.targetPitch = BALANCE_SETPOINT;
    controlLog.pTerm = controlLog.iTerm = 0.0f;
    controlLog.effectiveKp = Kp;
    controlLog.effectiveKd = Kd;
    controlLog.dTermRaw = 0.0f;
    controlLog.dTermLimited = false;
    controlLog.launchEvent = '-';
    controlLog.effortL = controlLog.effortR = 0.0f;
    controlLog.driveRaw = crsf::drive();
    controlLog.turnRaw = crsf::turn();
    controlLog.speedInput = crsf::speed();
    controlLog.commandCap = 0.35f + 0.65f * controlLog.speedInput;
    controlLog.driveCmd = controlLog.turnCmd = 0.0f;
    controlLog.positionLean = controlLog.velocityLean = controlLog.velocityLeanIntegral = controlLog.leanRaw = 0.0f;
    controlLog.effectiveKvel = Kvel;
    controlLog.posError = controlLog.velError = controlLog.softStart = 0.0f;
    controlLog.launchBoost = 0.0f;
    controlLog.driveVelocityEffort = 0.0f;
    controlLog.driveVelocityEffortActive = false;
    controlLog.driving = false;
    controlLog.launchState = '-';
    controlLog.coasting = true;
    long hl, hr; readEncoders(hl, hr);
    homeTicksSum = hl + hr;      // re-home under the robot while it's parked...
    posSetpoint  = 0.0f;         // ...and park the outer-loop target there too, so
    targetVelLog = 0.0f;         // re-arming never lurches toward a stale setpoint.
    satTicks     = 0;
    armedPrev    = false;        // next armed tick re-triggers the soft start
    driveCommandPrev = false;
    driveVelocityEffortLatched = false;
    launchAssistState = LAUNCH_IDLE;
    launchAssistDirection = 0;
    launchAssistTicksRemaining = 0;
    launchRollingTicks = 0;
    telemetry(pitch - BALANCE_SETPOINT, gyroRate, 0);
    return;
  }

  // --- Arm transition: start the soft-start ramp from a clean slate -----------
  if (!armedPrev) {
    armedPrev   = true;
    armedAtMs   = millis();
    posSetpoint = 0.0f;      // homeTicksSum was just reset, so positionRev ~ 0
    integral    = 0.0f;
    leanCmd     = 0.0f;
    satTicks    = 0;
    driveCommandPrev = false;
    driveVelocityEffortLatched = false;
    launchAssistState = LAUNCH_IDLE;
    launchAssistDirection = 0;
    launchAssistTicksRemaining = 0;
    launchRollingTicks = 0;
  }
  float softStart = constrain((millis() - armedAtMs) / (SOFT_START_SEC * 1000.0f), 0.0f, 1.0f);
  controlLog.softStart = softStart;

  // --- Live gain tuning from the radio (SC selects, S1 knob sets) -------------
  applyLiveTune();

  // --- RC drive/turn command from the radio ----------------------------------
  // Drive stick -> target wheel VELOCITY (rev/s), which feeds the outer loop's
  // setpoint below. Turn stick -> differential effort (balance is yaw-blind, so
  // a pure differential passes straight through). CH3 throttle caps both. With
  // BOTH sticks centered these are 0 and the balancer behaves exactly as before.
  float speedIn   = crsf::speed();
  float cap       = 0.35f + 0.65f * speedIn;                          // CH3 speed cap 0.35..1.0
  float driveRaw  = crsf::drive();
  float turnRaw   = crsf::turn();
  float driveIn   = driveRaw;
  float turnIn    = turnRaw;
  if (fabs(driveIn) < DRIVE_STICK_DEADZONE) driveIn = 0.0f;
  if (fabs(turnIn) < TURN_STICK_DEADZONE) turnIn = 0.0f;
  float targetVel = DRIVE_SIGN * driveIn * cap * DRIVE_MAX_VEL;       // rev/s, + = forward
  float turnCmd   = TURN_SIGN  * turnIn  * cap * TURN_AUTHORITY;      // per-wheel PWM-effort diff
  targetVelLog = targetVel;
  turnCmdLog = turnCmd;
  controlLog.speedInput = speedIn;
  controlLog.commandCap = cap;
  controlLog.driveRaw = driveRaw;
  controlLog.turnRaw = turnRaw;
  controlLog.driveCmd = driveIn;
  controlLog.turnCmd = turnIn;

  // NOTE: the position hold is NO LONGER released while driving. That was the
  // core bug -- with Kpos and Kvel both zeroed, nothing closed the loop on
  // translation and drive was open-loop acceleration. Driving now moves the
  // TARGET (posSetpoint) and the loop keeps chasing it, exactly as rc_balance
  // does. On release, re-anchor the target at the measured position. Otherwise
  // a failed drive leaves PERR pinned at POS_ERROR_CLAMP and keeps commanding a
  // lean after the pilot lets go instead of returning to neutral balance.
  bool driving = fabs(targetVel) > 0.001f;
  controlLog.driving = driving;
  int8_t driveDirection = targetVel > 0.0f ? +1 : -1;
  if (driving && (!driveCommandPrev || driveDirection != launchAssistDirection)) {
    // Start pilot control from the measured wheel position. A manual push can
    // leave the parked target behind the robot, making position hold oppose the
    // requested direction until the velocity setpoint catches up.
    posSetpoint = positionRev;
    velocityLeanI = 0.0f;
    driveVelocityEffortLatched = false;
#if DRIVE_LAUNCH_ASSIST
    // RC input ramps through small values on the way to full stick. Arm now and
    // let WAIT_LEAN hold until LAUNCH_MIN_TARGET_VEL is actually reached; marking
    // a small first sample DONE would suppress the pulse for the whole gesture.
    launchAssistState = LAUNCH_WAIT_LEAN;
#else
    launchAssistState = LAUNCH_IDLE;
#endif
    launchAssistDirection = driveDirection;
    launchAssistTicksRemaining = DRIVE_LAUNCH_ASSIST ? LAUNCH_ASSIST_TICKS : 0;
    launchRollingTicks = 0;
    controlLog.launchEvent = '-';
  } else if (!driving) {
    if (driveCommandPrev) posSetpoint = positionRev;
    driveVelocityEffortLatched = false;
    launchAssistState = LAUNCH_IDLE;
    launchAssistDirection = 0;
    launchAssistTicksRemaining = 0;
    launchRollingTicks = 0;
  }
  driveCommandPrev = driving;

  // Confirm real rolling over a window instead of trusting a one-tick encoder
  // burst. Sustained motion retires the pulse before its fixed budget expires.
#if DRIVE_LAUNCH_ASSIST
  float driveVelTowardTarget = driveDirection * forwardVel;
  if (driving && driveVelTowardTarget >= LAUNCH_RELEASE_VEL) {
    if (launchRollingTicks < LAUNCH_RELEASE_TICKS) launchRollingTicks++;
  } else {
    launchRollingTicks = 0;
  }
  bool driveRollingConfirmed = launchRollingTicks >= LAUNCH_RELEASE_TICKS;
#endif

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
    positionRev = 0.0f;                      // homeTicksSum just moved; don't use the stale value
    posSetpoint = 0.0f;                      // and park the outer-loop target on top of it
    integral = 0.0f;
    leanCmd = 0.0f;                          // drop any stale lean from before the fall
    velocityLeanI = 0.0f;
    satTicks = 0;
    driveVelocityEffortLatched = false;
    launchAssistState = LAUNCH_IDLE;
    launchAssistDirection = 0;
    launchAssistTicksRemaining = 0;
    launchRollingTicks = 0;
    armedAtMs = millis();                    // soft-start the recovery too
  }
  if (fallen) {
    integral = 0.0f;
    dTermLog = 0.0f;
    velocityLeanI = 0.0f;
    controlLog.targetPitch = BALANCE_SETPOINT;
    controlLog.pTerm = controlLog.iTerm = 0.0f;
    controlLog.effectiveKp = Kp;
    controlLog.effectiveKd = Kd;
    controlLog.dTermRaw = 0.0f;
    controlLog.dTermLimited = false;
    controlLog.effortL = controlLog.effortR = 0.0f;
    controlLog.positionLean = controlLog.velocityLean = controlLog.velocityLeanIntegral = controlLog.leanRaw = 0.0f;
    controlLog.effectiveKvel = Kvel;
    controlLog.posError = controlLog.velError = 0.0f;
    controlLog.launchBoost = 0.0f;
    controlLog.driveVelocityEffort = 0.0f;
    controlLog.driveVelocityEffortActive = false;
    controlLog.launchState = '-';
    controlLog.launchEvent = '-';
    controlLog.coasting = true;
    posSetpoint = positionRev;   // park the target under the robot while it's down
    motorRaw(0);
    telemetry(error, gyroRate, 0);
    return;
  }

  // --- Cascade OUTER loop: position/velocity SETPOINT -> target LEAN ---------
  // Always closed. The stick integrates into posSetpoint; the loop derives
  // whatever lean is needed to get there. Sign check: posError > 0 means the
  // target is AHEAD of us -> we must travel forward -> lean FORWARD (+deg).
  // With targetVel = 0 and posSetpoint parked this is identically the baseline
  // law  leanRaw = -(Kpos*positionRev + Kvel*forwardVel).
  posSetpoint += targetVel * DT;
  // Anti-windup: clamp the setpoint to a bounded lead over the true position, so
  // blocked wheels can't wind it up and lurch when they free.
  posSetpoint = constrain(posSetpoint, positionRev - POS_ERROR_CLAMP,
                                       positionRev + POS_ERROR_CLAMP);

  float posError = posSetpoint - positionRev;   // rev   : + = must travel forward
  float velError = targetVel   - forwardVel;    // rev/s : + = must speed up forward
#if OUTER_LOOP
  float effectiveKvel = Kvel * (driving ? DRIVE_KVEL_MULTIPLIER : 1.0f);
  float positionLean = Kpos * posError;
  float velocityLean = effectiveKvel * velError;
  if (driving) {
    velocityLeanI += KVEL_I * velError * DT;
    velocityLeanI = constrain(velocityLeanI, -VEL_I_CLAMP, VEL_I_CLAMP);
  } else {
    velocityLeanI = 0.0f;
  }
  float velocityLeanIntegral = velocityLeanI;
#else
  float effectiveKvel = 0.0f;
  float positionLean = 0.0f;   // DIAGNOSTIC: outer loop disabled -> pure baseline inner loop
  float velocityLean = 0.0f;
  velocityLeanI = 0.0f;
  float velocityLeanIntegral = 0.0f;
#endif
  float leanRaw = positionLean + velocityLean + velocityLeanIntegral;
  leanRaw = constrain(leanRaw, -LEAN_CLAMP, LEAN_CLAMP);
  leanCmd = LEAN_LPF * leanCmd + (1.0f - LEAN_LPF) * leanRaw;
  controlLog.posError = posError;
  controlLog.velError = velError;
  controlLog.positionLean = positionLean;
  controlLog.velocityLean = velocityLean;
  controlLog.velocityLeanIntegral = velocityLeanIntegral;
  controlLog.effectiveKvel = effectiveKvel;
  controlLog.leanRaw = leanRaw;
  controlLog.targetPitch = BALANCE_SETPOINT + leanCmd;

  // Inner-loop error now chases the LEANED target instead of the bare setpoint.
  error = pitch - (BALANCE_SETPOINT + leanCmd);

  // --- Integral (with anti-windup); harmless while Ki = 0 ---
  integral += error * DT;
  integral = constrain(integral, -40.0f, 40.0f);   // clamp; tune with Ki

  // --- PID INNER loop (D on the low-passed rate so gyro spikes don't kick) ---
  // Encoder position/velocity act through leanCmd; no direct motor feedforward.
  // The outer loop owns only the angle target; the inner loop remains the sole
  // source of common-mode motor power. Drive-specific gains are blended with the
  // pilot command, leaving the known-good neutral balance gains unchanged.
  // The speed knob limits the requested velocity, not the balance loop's ability
  // to catch the requested lean. Full stick must retain full DRIVE_KP authority
  // even when the pilot deliberately selects a low speed cap.
  float driveAuthority = driving ? constrain(fabs(driveIn), 0.0f, 1.0f) : 0.0f;

  controlLog.dTermRaw = Kd * dRateFilt;
  float kdSafetyBlend = constrain(
      (fabs(error) - DRIVE_KD_RESTORE_START_ERROR) /
      (DRIVE_KD_RESTORE_FULL_ERROR - DRIVE_KD_RESTORE_START_ERROR),
      0.0f, 1.0f);
  float driveKdFloor = Kd < DRIVE_KD ? Kd : DRIVE_KD;
  float driveKdTarget = driveKdFloor + kdSafetyBlend * (Kd - driveKdFloor);
  controlLog.effectiveKd = Kd + driveAuthority * (driveKdTarget - Kd);
  dTermLog = controlLog.effectiveKd * dRateFilt;
  if (driveAuthority > 0.0f && kdSafetyBlend < 1.0f) {
    float dLimit = DRIVE_D_NEAR_TARGET_LIMIT +
                   kdSafetyBlend * (fabs(dTermLog) - DRIVE_D_NEAR_TARGET_LIMIT);
    dTermLog = constrain(dTermLog, -dLimit, dLimit);
  }
  controlLog.dTermLimited = fabs(controlLog.effectiveKd - Kd) > 0.001f;

  float driveKpTarget = Kp < DRIVE_KP ? DRIVE_KP : Kp;
  controlLog.effectiveKp = Kp + driveAuthority * (driveKpTarget - Kp);
  controlLog.pTerm = controlLog.effectiveKp * error;
  controlLog.iTerm = Ki * integral;
  float u = -(controlLog.pTerm + dTermLog + controlLog.iTerm);
  u *= softStart;   // ramp authority in over SOFT_START_SEC after arming

  // --- Coast band: rest the motors when essentially balanced ---
  // Gate on the FILTERED pitch (error) and wheel velocity (VF) -- the trustworthy
  // "settled" signals -- NOT on gyroRate, which is corrupted by motor vibration
  // (it would never let the coast engage). If a real tilt develops, error leaves
  // the band within a cycle or two and full control resumes before it can fall.
  // Only coast when truly IDLE -- not while the pilot commands drive/turn, or the
  // small drive lean (often < ANGLE_DEADZONE) gets zeroed here and nothing moves.
  bool commanding = driving || (fabs(turnCmd) > OUT_DEADZONE);
  controlLog.coasting = false;
  if (!commanding && fabs(error) < ANGLE_DEADZONE && fabs(forwardVel) < VEL_DEADZONE) {
    u = 0.0f;
    integral = 0.0f;   // don't accumulate while parked
    controlLog.iTerm = 0.0f;
    controlLog.coasting = true;
  }

#if STALL_CUTOFF
  // --- Stall cutoff: pushing hard but wheels not turning -> motors off ---
  if (updateStall(u, now)) {
    motorRaw(0);
    telemetry(error, gyroRate, 0);
    return;
  }
#endif

#if SATURATION_LATCH
  // --- Inner-loop saturation latch -------------------------------------------
  // mapEffortToPwm() tops out at MAX_PWM, so the usable effort range is
  // (MAX_PWM - MAX_DEADBAND). Sitting at 95% of that for half a second means
  // the inner loop is pinned and losing -- latch instead of stalling the motors.
  if (fabs(u) > SAT_EFFORT_FRAC * (float)(MAX_PWM - MAX_DEADBAND)) satTicks++;
  else                                                             satTicks = 0;
  if (satTicks > SAT_TIMEOUT_TICKS) {
    satTicks = 0;
    fallen = true;   // re-arms off the ABSOLUTE accel angle, so this cannot deadlock
    Serial.println("SAT: inner loop pinned -> latched (stand it up to re-arm)");
  }
#endif

  // --- Bounded drive-launch assist ------------------------------------------
  // WAIT_LEAN lets the normal non-minimum-phase response establish the requested
  // lean. PUSH may strengthen an already matching PD effort for at most 200 ms,
  // but never reverses an opposing balance correction. Angle and lean gates abort
  // it, and LAUNCH_DONE prevents repeated kicks during a held command.
  controlLog.launchBoost = 0.0f;
  controlLog.launchEvent = '-';
  controlLog.driveVelocityEffort = 0.0f;
  controlLog.driveVelocityEffortActive = false;
#if DRIVE_LAUNCH_ASSIST
  float launchEffortSign = -(float)launchAssistDirection; // SIGN_CFG: physical forward is negative PWM effort
#endif
#if DRIVE_LAUNCH_ASSIST || DRIVE_VELOCITY_EFFORT
  float actualLean = pitch - BALANCE_SETPOINT;
#endif
#if DRIVE_VELOCITY_EFFORT
  bool velocityNeedsDrive = driveDirection * velError > 0.0f;
  bool driveVelocityEffortReady = driving &&
                                  velocityNeedsDrive &&
                                  driveDirection * actualLean >= LAUNCH_READY_LEAN_DEG &&
                                  fabs(error) <= LAUNCH_READY_ERROR_DEG &&
                                  fabs(dRateFilt) <= LAUNCH_READY_RATE_DPS;
  bool driveVelocityEffortSafeToHold = driving &&
                                       velocityNeedsDrive &&
                                       driveDirection * actualLean >= DRIVE_VELOCITY_HOLD_LEAN_DEG &&
                                       fabs(error) <= LAUNCH_ABORT_ERROR_DEG &&
                                       fabs(dRateFilt) <= LAUNCH_ABORT_RATE_DPS;
  if (!driveVelocityEffortSafeToHold) driveVelocityEffortLatched = false;
  else if (driveVelocityEffortReady)   driveVelocityEffortLatched = true;
#else
  driveVelocityEffortLatched = false;
#endif
#if DRIVE_LAUNCH_ASSIST
  if (launchAssistState == LAUNCH_WAIT_LEAN) {
    float launchReadyLean = min(LAUNCH_READY_LEAN_DEG, 0.75f * fabs(leanRaw));
    bool leanReady = launchAssistDirection * actualLean >= launchReadyLean;
    bool bodyReady = fabs(error) <= LAUNCH_READY_ERROR_DEG &&
                     fabs(dRateFilt) <= LAUNCH_READY_RATE_DPS;
    bool strongCommand = fabs(targetVel) >= LAUNCH_MIN_TARGET_VEL;
    bool stopped = fabs(forwardVel) <= LAUNCH_MAX_START_VEL;
    if (driveRollingConfirmed) {
      controlLog.launchEvent = 'R';
      launchAssistState = LAUNCH_DONE;
    } else if (!driving) {
      controlLog.launchEvent = 'N';
      launchAssistState = LAUNCH_DONE;
    } else if (strongCommand && softStart >= 0.99f && leanReady && bodyReady && stopped) {
      controlLog.launchEvent = 'P';
      launchAssistState = LAUNCH_PUSH;
    }
  }
  if (launchAssistState == LAUNCH_PUSH) {
    bool unsafeError = fabs(error) > LAUNCH_ABORT_ERROR_DEG;
    float launchAbortLean = min(LAUNCH_ABORT_LEAN_DEG, 0.50f * fabs(leanRaw));
    bool unsafeLean = launchAssistDirection * actualLean < launchAbortLean;
    bool strongCommand = fabs(targetVel) >= LAUNCH_MIN_TARGET_VEL;
    if (driveRollingConfirmed) {
      controlLog.launchEvent = 'R';
      launchAssistState = LAUNCH_DONE;
    } else if (unsafeError) {
      controlLog.launchEvent = 'E';
      launchAssistState = LAUNCH_DONE;
    } else if (unsafeLean) {
      controlLog.launchEvent = 'B';
      launchAssistState = LAUNCH_DONE;
    } else if (!driving || !strongCommand) {
      controlLog.launchEvent = 'N';
      launchAssistState = LAUNCH_DONE;
    } else if (launchAssistTicksRemaining == 0) {
      controlLog.launchEvent = 'T';
      launchAssistState = LAUNCH_DONE;
    } else {
      launchAssistTicksRemaining--;
      // Physical forward PWM is negative while encoder/target forward is positive.
      float launchFloor = launchEffortSign * LAUNCH_ASSIST_EFFORT;
      // The inner loop owns safety. Apply breakaway compensation only when it is
      // already trying to move in the requested direction; never force it through
      // an opposite-sign catch like the observed +23.3 -> -20 launch reversal.
      if (launchEffortSign * u > OUT_DEADZONE &&
          launchEffortSign * u < LAUNCH_ASSIST_EFFORT) {
        controlLog.launchBoost = launchFloor - u;
        u = launchFloor;
      }
    }
  }
#endif

  // Legacy post-PD velocity effort retained behind a switch for controlled A/B
  // comparison only. Normal drive authority comes from the outer-loop lean.
#if DRIVE_VELOCITY_EFFORT
  if (launchAssistState != LAUNCH_PUSH && driveVelocityEffortLatched) {
    controlLog.driveVelocityEffort = constrain(-DRIVE_VELOCITY_EFFORT_GAIN * velError,
                                                -DRIVE_VELOCITY_EFFORT_LIMIT,
                                                 DRIVE_VELOCITY_EFFORT_LIMIT);
    controlLog.driveVelocityEffortActive =
        fabs(controlLog.driveVelocityEffort) > OUT_DEADZONE;
    u += controlLog.driveVelocityEffort;
  }
#endif

  controlLog.launchState = launchAssistState == LAUNCH_WAIT_LEAN ? 'W' :
                           launchAssistState == LAUNCH_PUSH ? 'P' :
                           launchAssistState == LAUNCH_DONE ? 'C' : '-';

  controlLog.effortL = u + turnCmd;
  controlLog.effortR = u - turnCmd;
  driveControlDiff(controlLog.effortL, controlLog.effortR);   // balance +/- steering
  telemetry(error, gyroRate, u);
}

// =============================================================================
void telemetry(float error, float rate, float u) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();
  bool armed = crsf::armed();
  const char *state = !armed ? "DISARM" : fallen ? "FALL" :
                      controlLog.coasting ? "COAST" : controlLog.driving ? "DRIVE" : "BAL";
  long lEnc, rEnc; readEncoders(lEnc, rEnc);

  Serial.print("MS "); Serial.print(lastPrint);
  Serial.print(" STATE "); Serial.print(state);
  Serial.print(" LINK "); Serial.print(crsf::linkUp() ? "Y" : "-");
  Serial.print(" ARM "); Serial.print(armed ? "Y" : "-");
  Serial.print(" HZ "); Serial.print(controlHz);

  Serial.print(" | IMU AP "); Serial.print(controlLog.accelPitch, 2);
  Serial.print(" PITCH "); Serial.print(pitch, 2);
  Serial.print(" TARGET "); Serial.print(controlLog.targetPitch, 2);
  Serial.print(" ERR "); Serial.print(error, 2);
  Serial.print(" GX "); Serial.print(controlLog.gyroX, 2);
  Serial.print(" GY "); Serial.print(controlLog.gyroY, 2);
  Serial.print(" GZ "); Serial.print(controlLog.gyroZ, 2);
  Serial.print(" RATE "); Serial.print(rate, 2);
  Serial.print(" DFILT "); Serial.print(dRateFilt, 2);
  Serial.print(" AX "); Serial.print(controlLog.accelX, 2);
  Serial.print(" AY "); Serial.print(controlLog.accelY, 2);
  Serial.print(" AZ "); Serial.print(controlLog.accelZ, 2);

  Serial.print(" | PID P "); Serial.print(controlLog.pTerm, 2);
  Serial.print(" I "); Serial.print(controlLog.iTerm, 2);
  Serial.print(" D0 "); Serial.print(controlLog.dTermRaw, 2);
  Serial.print(" D "); Serial.print(dTermLog, 2);
  Serial.print(" DLIM "); Serial.print(controlLog.dTermLimited ? "Y" : "-");
  Serial.print(" U "); Serial.print(u, 2);
  Serial.print(" EL "); Serial.print(controlLog.effortL, 2);
  Serial.print(" ER "); Serial.print(controlLog.effortR, 2);
  Serial.print(" PWML "); Serial.print(motorPwmL);
  Serial.print(" PWMR "); Serial.print(motorPwmR);

  Serial.print(" | ENC L "); Serial.print(lEnc);
  Serial.print(" R "); Serial.print(rEnc);
  Serial.print(" DL "); Serial.print(controlLog.encoderDeltaL);
  Serial.print(" DR "); Serial.print(controlLog.encoderDeltaR);
  Serial.print(" VRL "); Serial.print(controlLog.wheelVelRawL, 2);
  Serial.print(" VRR "); Serial.print(controlLog.wheelVelRawR, 2);
  Serial.print(" VL "); Serial.print(wheelVelL, 2);
  Serial.print(" VR "); Serial.print(wheelVelR, 2);
  Serial.print(" VF "); Serial.print(forwardVel, 2);
  Serial.print(" VROT "); Serial.print(rotationVel, 2);
  Serial.print(" POS "); Serial.print(positionRev, 3);

  // Outer-loop tracking: TVEL is what the stick asked for, PSET/PERR is where the
  // setpoint is vs the robot. On a forward stick TVEL must be POSITIVE and PERR
  // must go positive -- if TVEL is negative on stick-up, flip DRIVE_SIGN.
  Serial.print(" | TRK TVEL "); Serial.print(targetVelLog, 2);
  Serial.print(" PSET "); Serial.print(posSetpoint, 3);
  Serial.print(" PERR "); Serial.print(controlLog.posError, 3);
  Serial.print(" VERR "); Serial.print(controlLog.velError, 2);
  Serial.print(" SOFT "); Serial.print(controlLog.softStart, 2);
  Serial.print(" START "); Serial.print(controlLog.launchState);
  Serial.print(" EXIT "); Serial.print(controlLog.launchEvent);
  Serial.print(" BOOST "); Serial.print(controlLog.launchBoost, 1);
  Serial.print(" BTK "); Serial.print(launchAssistTicksRemaining);
  Serial.print(" VTRQ "); Serial.print(controlLog.driveVelocityEffort, 2);
  Serial.print(" VTG "); Serial.print(controlLog.driveVelocityEffortActive ? "Y" : "-");

  Serial.print(" | LEAN ACT "); Serial.print(pitch - BALANCE_SETPOINT, 2);
  Serial.print(" POS "); Serial.print(controlLog.positionLean, 2);
  Serial.print(" VEL "); Serial.print(controlLog.velocityLean, 2);
  Serial.print(" VELI "); Serial.print(controlLog.velocityLeanIntegral, 2);
  Serial.print(" RAW "); Serial.print(controlLog.leanRaw, 2);
  Serial.print(" CMD "); Serial.print(leanCmd, 2);

  Serial.print(" | RC SPD "); Serial.print(controlLog.speedInput, 2);
  Serial.print(" CAP "); Serial.print(controlLog.commandCap, 2);
  Serial.print(" DRAW "); Serial.print(controlLog.driveRaw, 2);
  Serial.print(" TRAW "); Serial.print(controlLog.turnRaw, 2);
  Serial.print(" DCMD "); Serial.print(controlLog.driveCmd, 2);
  Serial.print(" TCMD "); Serial.print(controlLog.turnCmd, 2);
  Serial.print(" TEFF "); Serial.print(turnCmdLog, 2);

  Serial.print(" | CFG Kp "); Serial.print(Kp, 2);
  Serial.print(" Kpeff "); Serial.print(controlLog.effectiveKp, 2);
  Serial.print(" Kd "); Serial.print(Kd, 2);
  Serial.print(" Kdeff "); Serial.print(controlLog.effectiveKd, 2);
  Serial.print(" Kpos "); Serial.print(Kpos, 2);
  Serial.print(" Kvel "); Serial.print(Kvel, 2);
  Serial.print(" Kveff "); Serial.print(controlLog.effectiveKvel, 2);
  Serial.print(" Vmax "); Serial.print(DRIVE_MAX_VEL, 2);
  Serial.print(" Bstart "); Serial.print(LAUNCH_ASSIST_EFFORT, 1);
  Serial.print(" GBIAS "); Serial.print(gyroYBias, 2);
  // Floors actually in force this tick (M = moving/Coulomb, S = static breakaway).
  Serial.print(" DBL "); Serial.print(wheelMovingL ? LEFT_DEADBAND_MOVING : LEFT_DEADBAND_STATIC);
  Serial.print(wheelMovingL ? "M" : "S");
  Serial.print(" DBR "); Serial.print(wheelMovingR ? RIGHT_DEADBAND_MOVING : RIGHT_DEADBAND_STATIC);
  Serial.print(wheelMovingR ? "M" : "S");
  Serial.print(" PWMHZ "); Serial.print(MOTOR_PWM_HZ);
  Serial.print(" SET "); Serial.println(sweepIdx + 1);
  controlLog.encoderDeltaL = 0;
  controlLog.encoderDeltaR = 0;
}
