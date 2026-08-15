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
// SWEEP IN PROGRESS 2026-08-14 -- 4482 measured free-spin L@13 R@12 and L@14 R@13 on a
// repeat, so repeatability is +/-1 count. Now at 2000; 1200 next. RESTORE TO 4482 unless
// the sweep says otherwise, and never raise it (see the note above about turn-on delay).
// SWEEP RESULTS (free-spin breakaway, wheels off the ground, positive PWM = backward):
//         L     R
//   4482  13    12    (repeat 14 / 13, so repeatability is +/-1 count)
//   2000  11     9
//   1200  21     8    <- R lands exactly on the predicted 8; L is NOT monotonic
// A floor cannot RISE as frequency falls, so L@21 is a bad sample, not a measurement.
// UNRESOLVED, and it blocks interpreting any of this: at 1200 the log shows dL -14
// (barely moved) and dR -358 (spun freely), but the pilot saw the RIGHT wheel barely
// move. If the encoder channels are mislabelled then every left/right constant in this
// file is attached to the wrong wheel. DO NOT TUNE DEADBANDS UNTIL THAT IS SETTLED.
// Conclusions that survive either way:
//   - the 15/27 LOADED asymmetry is not electrical. Off the ground both wheels break
//     within a count of each other at every frequency, so no PWM change narrows it.
//   - fitting dt = 1/(f*256) gives t_on ~4-5.5 us, not the ~8 us assumed from the
//     datasheet, so frequency buys ~3 counts of floor rather than ~7. The remaining
//     6-9 counts are real unloaded mechanical friction. FLOOR_KNEE 2.5 stays.
// SWEEP CONCLUDED: 2000 Hz. 1200 REPRODUCIBLY BROKE THE LEFT CHANNEL (L@21 twice, with
// dL -25 against dR -360) -- a floor cannot RISE as frequency falls, so that is not the
// turn-on-delay model, it is the left driver misbehaving at 1200. Note the hardware
// split in the boot banner: L is 6/9 on ONE submodule (FlexPWM2_2 A/B) while R is 22/23
// on TWO (FlexPWM4_0A, 4_1A). A left-only, frequency-dependent fault fits that exactly.
// DO NOT GO BELOW 2000 without re-checking the left channel.
const int MOTOR_PWM_HZ = 2000;   // do NOT raise without re-running DEADBAND_TEST
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
// ENABLED 2026-08-14 to measure the right-vs-left breakaway asymmetry (27 vs 15) that
// is the root of the rear stall -- see FLOOR_KNEE. Run at MOTOR_PWM_HZ 4482, then 2000,
// then 1200, and fit: deadband = t_on * f_pwm * 256 + mechanical. Predicted driver-delay
// term at t_on ~8 us is 9.2 / 4.1 / 2.5 counts respectively, so if the floors fall by
// roughly that much the gap is BTS7960 turn-on delay and a lower PWM frequency fixes it;
// if they barely move it is real friction and the answer is mechanical.
// SET BACK TO 0 BEFORE FLYING -- this replaces the balance loop entirely.
// Post-EN-fix re-measure is COMPLETE (moving 10/10, static 18/18) and the flag is back
// to 0. deadbandDetect changed after that run, so the next loaded run is a confirmation
// with the twitch artefact removed -- worth one pass before trusting 18 to a decimal.
#define DEADBAND_TEST 0
// +1 = positive PWM = physical BACKWARD (all existing deadband figures). -1 = FORWARD.
// This is the sign of the FIRST pass only; the test alternates every pass. See the
// caveat at LEFT_DEADBAND_MOVING.
const int DEADBAND_TEST_SIGN = 1;
// MULTI-PASS, 2026-08-15. One flash and one power-up runs DEADBAND_TEST_PASSES ramps
// with the sign alternating each pass, then prints a summary table.
//
// IT HAS ALREADY ANSWERED THE QUESTION IT WAS BUILT FOR -- see LEFT_DEADBAND_MOVING for
// the numbers. Direction is not a variable and the free-spin floor is ~10 on both
// wheels. Kept because any future deadband claim needs a distribution, not a sample.
//
// WHY REPEATS: repeatability stopped being an assumption and became the measurement.
// A +1 run came back L@21 R@8 where the 2000 Hz sweep recorded L@11 R@9 -- and
// L@21 / R@8 / dR ~ -360 is a replay of the 1200 Hz line that ac0b305 discarded as a
// bad sample on the grounds that a floor cannot rise as frequency falls. Two events
// looked like a bimodal left channel, which would have made the "+/-1 count" premise
// under every deadband figure in this file false. Six passes said otherwise: L@21 did
// not reproduce, and each wheel's spread is 2-4 counts. A single ramp cannot tell an
// outlier from a regime. Three per direction can.
//
// WHY ALTERNATE rather than run a block of + then a block of -: a block comparison
// confounds direction with anything that drifts across the session (motor warming,
// battery sag). Alternating puts both directions on the same drift.
//
// Each pass re-seeds after the rotor has coasted to a stop, so the passes also sample
// different rotor rest positions -- which is the standing hypothesis for the left
// channel: a commutator dead spot, where breakaway depends on where the rotor stopped.
// If that is what this is, the answer is neither a deadband constant nor a mechanical
// hunt, and per-direction constants would be fitting a number to a coin flip.
//
// >>> RESOLVE THE ENCODER CHANNEL LABELS FIRST (ENCODER_TEST). ac0b305 is still
// >>> blocking: the pilot saw the RIGHT wheel barely moving while the log said dL
// >>> stuck / dR spinning, and this run reproduced that too. If the channels are
// >>> swapped, every L/R constant here is on the wrong wheel and these passes will
// >>> just mislabel a real result.
const int DEADBAND_TEST_PASSES   = 6;      // 3 per direction
const int DEADBAND_TEST_DWELL_MS = 1500;   // coast-down between passes, motors off

// ---- Idle-brake experiment --------------------------------------------------
// The two bridges idle in OPPOSITE states -- left coasts, right brakes; see the
// retraction block at LEFT_DEADBAND_MOVING. The BTS7960 enable pins are not wired to
// the MCU, so the RIGHT cannot be made to coast. The reverse IS reachable in firmware:
// driving BOTH inputs of a BTS7960 HIGH puts both outputs at VCC, shorting the motor
// just as both-LOW shorts it to ground. Either way it is a brake, and it works whether
// that module's enables are strapped high or follow the inputs -- which is what makes
// this testable without knowing the left module's jumpering.
// NOT shoot-through: each half-bridge drives one FET, the halves are independent, and
// a stationary motor generates no back-EMF so a braked idle draws no current.
//
// SUPERSEDED 2026-08-15: the EN wiring was found and fixed, both bridges now idle in
// the same state, and the asymmetry this was built to explain is gone. Kept because it
// is still the only way to A/B brake-vs-coast from firmware -- if a limit cycle wants
// passive damping later, or if the two wheels now both COAST and breakaway needs help,
// this is the lever. The original framing follows.
//
// THE TEST: set to 1, run DEADBAND_TEST LOADED, and read the LEFT static figures.
//   climbs from ~13 toward the right's ~24 -> brake drag is worth ~11 counts of
//     breakaway, the 13-vs-24 asymmetry is explained, and nothing is mechanically wrong
//   does not move                          -> the brake is not the mechanism and the
//     24 still needs an explanation
// A second effect rides along and this test cannot separate it: with the enables
// following the inputs the left also uses FAST decay during PWM off-time while the
// right uses SLOW decay, which changes torque per unit duty independently of anything
// at rest. If the left static moves but not all the way, suspect both are in play.
//
// EXPERIMENT ONLY, SET BACK TO 0. Braking both wheels IS symmetric and would kill the
// veer, but it doubles down on the stiction wall this whole file is fighting. With the
// enables unreachable the shipped mitigation stays the per-side constants.
#define LEFT_IDLE_BRAKE 0
const int MOTOR_PWM_FULL = 255;            // MOTOR_PWM_BITS 8 -> full-scale count

// ---- Burst logger ----------------------------------------------------------
// 100 ms telemetry cannot resolve the drive oscillation: clean alternation every
// sample is equally consistent with 5, 15, 25 or 35 Hz, and those need opposite
// fixes (loop gain/phase vs structural resonance). This captures the inner loop
// at the FULL tick rate into RAM with no serial I/O during capture, so it cannot
// perturb what it is measuring, then dumps once disarmed.
//
// Arms automatically on the drive-command rising edge; dumps 20 lines per pass
// while DISARMED (motors already cut there, and chunking avoids a long block).
// Read from the dump: the true ring frequency, and whether U LEADS pitch (the D
// term is driving the oscillation) or LAGS it (D is damping, something else
// drives it). Either answer picks the next fix; neither is knowable from the
// 100 ms telemetry.
#define BURST_LOG 1
const int BURST_N = 400;             // 2.0 s at 200 Hz; 400 * 20 B = 8 KB of 1 MB

// ---- Static lean test ------------------------------------------------------
// Bypasses the outer loop entirely and hands the inner loop a FIXED lean target,
// held for as long as the stick is held. Isolates the single question everything
// else has been confounded by: given a sustained lean command, does the inner
// loop produce sustained wheel torque?
//   * robot drives away and keeps going  -> inner loop is fine, fault is entirely
//                                           in the outer loop's lean trajectory
//   * robot shakes in place at a fixed lean -> the inner loop cannot convert a
//                                           standing lean into net wheel travel
// ARMED BY DEFAULT: the lean is applied continuously the whole time the robot is
// armed, with no stick input required, so the body holds a lean it should be
// falling out of for as long as you let it. SB (the kill switch) is the bail-out.
//
// Side effects this mode deliberately forces, both needed for the test to mean
// anything:
//   * the COAST BAND is suppressed. Without the stick, `commanding` would be
//     false, so the moment the body reached the lean (|error| < ANGLE_DEADZONE,
//     wheels stopped) the coast band would zero u and cut the motors -- exactly
//     the instant the measurement starts.
//   * gains are the NEUTRAL set (Kp 2.0 / Kd 0.30), not the drive set, because
//     driveAuthority is 0 with no stick. That is the known-good balance tuning,
//     which is the right baseline for an inner-loop question.
// The burst capture arms on the ARM transition in this mode (not the drive edge),
// so SB up gives soft-start + 2 s of full-rate capture, and SB down dumps it.
// RESULT, first run at 3.0 deg: the robot DID travel -- POS 0.016 -> 0.261, about
// 0.245 rev -- but only during the stand-up overshoot, while the ACTUAL lean was
// 6.5-7.6 deg. As the inner loop settled the pitch onto the commanded 3.0, speed
// decayed with it and motion stopped at LEAN ACT ~3.4:
//     LEAN ACT 7.6 -> VF 0.20      LEAN ACT 4.1 -> VF 0.06
//     LEAN ACT 6.5 -> VF 0.18      LEAN ACT 3.4 -> VF 0
//     LEAN ACT 5.0 -> VF 0.11
// So BREAKAWAY LEAN IS ~3.4 deg and 3.0 sat just under it. Raised to 6.0 as the
// falsifiable check: comfortably above threshold, it should roll continuously and
// never enter the stalled stick-slip cycle.
// RESULT at 6.0 deg, and the reason this test is now RETIRED -- it answered all
// three questions it was built for:
//   1. THE DRIVETRAIN WORKS. One run produced POS 0.000 -> 2.134, i.e. 2.13 rev of
//      sustained monotonic travel at up to VF 0.65 rev/s. That question is closed.
//   2. Breakaway is ~6-7 deg (not the 3.4 deg first inferred; that figure came
//      from a decelerating coast, see below).
//   3. THE REAL OBSTACLE IS THE STATIC/KINETIC FRICTION GAP. The SAME 6.0 deg
//      command produced opposite outcomes on consecutive runs:
//          run A: stalled, POS frozen, LEAN ACT held ~6.2
//          run B: broke away, ran away to LEAN ACT 18.9 with ERR 12.9
//      and run B did BOTH -- ran away, then settled back and sat stalled at
//      LEAN ACT ~6 for its last 3 s. Above breakaway 6 deg over-accelerates
//      without limit; below it, 6 deg holds the robot stopped. NO SINGLE LEAN
//      VALUE BOTH STARTS AND CRUISES THIS CHASSIS.
// The runaway is the test working, not a fault: overriding leanCmd removes the
// velocity feedback, and a fixed lean is a fixed ACCELERATION command, so once it
// breaks away nothing backs it off. Bridging that gap -- high lean to break away,
// then collapse it as VF rises -- is precisely the job of the outer loop's
// velocity term. Set to 1 only to re-measure breakaway on a changed chassis.
#define STATIC_LEAN_TEST 0
const float STATIC_LEAN_DEG = 6.0f;  // deg of commanded lean, held continuously

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
// >>> DIAGNOSTIC SWITCH (bisection tool, not a fix) <<<
// 0 = schedule OFF: driveAuthority is forced to 0, so effectiveKp/effectiveKd stay
//     at the neutral Kp/Kd for ANY stick position (this also survives live tuning,
//     which the DRIVE_KP/DRIVE_KD values below do not). Drive then runs the exact
//     inner loop that balances quietly. Telemetry confirms it: DLIM reads '-' and
//     Kpeff/Kdeff track Kp/Kd for the whole drive.
// 1 = schedule ON (blend toward DRIVE_KP / DRIVE_KD with commanded drive).
//
// TESTED 2026-08-14 WITH THIS AT 0: NULL RESULT, RESTORED TO 1. The bench run
// confirmed the switch took (Kpeff 2.00 / Kdeff 0.30 / DLIM '-' for the whole drive)
// and the stall was IDENTICAL -- POS moved 0.029 rev in 4.2 s while LEAN CMD ramped
// to 7.47 and VELI sat on its clamp, PWM alternating +/-20 about a mean of -2.6.
// The schedule is NOT the cause. Worse, setting it to 0 raises Kd during drive to
// 0.30, and the 2026-06-30 note at Kd already records that RAISING Kd makes a
// several-Hz ring worse (D-destabilization via rate-filter lag) -- at 9 Hz the D term
// is not damping, it is the whole loop gain (0.30 * 56 = 17). Leave this at 1.
// Kept as a switch because the burst evidence below is still the best description of
// the oscillation, and this is the cheapest way to re-test the gains against it.
//
// The 200 Hz burst capture (400 samples, armed on the drive rising edge) showed:
//   * The stall is a clean 8.5 Hz LIMIT CYCLE, not chatter and not gyro noise.
//     Positive-run starts at i = 202,226,250,273,296,319,343,366 -> deltas
//     24,24,23,23,23,24,23 = 23.4 samples = 117 ms. u is a smooth sinusoid swinging
//     +8 -> -8 over ~12 ticks each way; PWM tracks it faithfully on the friction
//     floor. Because the swing is SYMMETRIC its mean is ~0, so the wheels dither in
//     place and the robot never breaks away -- see the note at DRIVE_MAX_VEL.
//   * The BODY is really rocking, the gyro is not lying: pitch swings ~+/-0.55 deg,
//     and 2*pi*8.5*0.55 = 29 deg/s, which is exactly the observed rate peak. So this
//     is NOT an IMU-mount artefact and NOT the 20 Hz DLPF.
//   * It starts WITH the schedule. i=0..6 are the first ticks of DRIVE: pitch is dead
//     flat (-3.560,-3.560,-3.559,-3.556) at rate 0.02. By i=9 it is oscillating --
//     under 50 ms, exactly as Kpeff ramps 2.00->2.86->3.74->4.00 and Kdeff 0.30->0.20.
//   * It dies WITHOUT the loop. i=374..387 have u = 0.000 and motors off: pitch runs
//     -2.331 -> -1.187 and rate decays -14.29 -> 4.35, smooth and monotonic with no
//     ringing. A structural resonance would keep ringing; this does not.
// The gains do not set it; the FLOOR does. The cycle settles at |u| = 8-10 against a
// floor of 15 -- i.e. exactly where the floor stops dominating the PWM. Below that the
// map has enormous incremental gain (u = 0.1 -> PWM 15), which is the describing-
// function recipe for a limit cycle, and the amplitude it settles at is the signature.
// That also explains the thing that never fit: balance is quiet, drive oscillates. At
// rest the robot sits in the COAST BAND with the motors off, so no floor is applied.
// Driving holds them energized every tick. It was never a drive problem -- the coast
// band was hiding it. Fix lives at DRIVE_FRICTION_FF, not in these gains.
#define DRIVE_GAIN_SCHEDULE 1

// Translation needs turn-class common-mode authority, but neutral balance is
// already tuned. Blend toward this proportional gain with commanded drive only.
const float DRIVE_KP = 4.0f;
// Kd=0.20 was chosen to damp what 100 ms telemetry looked like a ~5 Hz drive mode.
// That frequency was an ALIAS -- the real cycle is 8.5 Hz (see DRIVE_GAIN_SCHEDULE),
// and cutting Kd is what let it grow. Left at 0.20 only so the schedule can be
// switched back on unchanged for an A/B; it is not a defended value.
const float DRIVE_KD = 0.20f;
// REMOVED DRIVE_D_NEAR_TARGET_LIMIT (was 1.5). A clamped derivative is not a
// damper, it is a relay: above |dTerm| = limit it outputs a constant magnitude
// carrying only the SIGN of the rate, with no proportionality and no phase
// information. In the 2026-08-14 log D0 reached +/-11.45 while D was pinned at
// +/-1.50 on essentially every sample (DLIM Y throughout), so ~80% of the
// derivative was discarded exactly when the rate was highest -- i.e. exactly
// when damping was needed. PITCH ring amplitude grew across the window
// (+/-0.6 -> +/-2.4) as a direct result. Kd is set by DRIVE_KD; let it act.
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

// 1.0 -> 4.0 -> 2.0. The raise to 4.0 was meant to lift the outer loop's lean
// ceiling past the friction threshold; the bench test refuted that theory (see
// LEAN_CLAMP) and 4.0 walked the robot to 12 deg of pitch under a held stick.
// 2.0 keeps some of the extra station-keeping authority without making a stalled
// drive dangerous to hold. With POS_ERROR_CLAMP 1.5 the position term now tops
// out at 3.0 deg.
// 1.0 -> 4.0 -> 2.0 -> 4.0. Back to 4.0 now that breakaway is MEASURED at ~6-7 deg
// (see STATIC_LEAN_DEG). At 2.0 the outer loop's component sum was
//   Kpos 2.0 * POS_ERROR_CLAMP 1.5 = 3.00
//   Kvel 3.0 * DRIVE_MAX_VEL 0.25  = 0.75   <- shrank when Vmax was cut to 0.25
//   VEL_I_CLAMP                     = 2.50
//                                    ------
//                                     6.25  -- just UNDER the threshold, again.
// With Kpos 4.0 and VEL_I_CLAMP 4.0 the sum is 10.75, clear of breakaway and
// inside LEAN_CLAMP 12. The earlier 4.0 was reverted because it walked the robot
// to 12 deg, but that was with LEAN_CLAMP at 10 and no conditional integration
// engaging; LEAN_CLAMP 12 now binds below the component sum, so the anti-windup
// actually fires and bounds the excursion.
// 4.0 -> 6.0 (2026-08-14, after the two-phase friction FF made drive work at all).
// This term does double duty: it sets 3/5 of the lean RAMP RATE (posError grows at
// targetVel, so this contributes Kpos*Vmax deg/s) and Kpos*POS_ERROR_CLAMP of the
// CEILING. Pilot report: "hold the stick longer and the drive actually is initiated,
// but very slowly" -- the mechanism works, the ramp was just too slow to use.
float Kpos = 6.0f;               // wheel position error (rev -> deg of lean)
float Kvel = 3.0f;               // wheel-velocity damping; enough drive lean without sitting on the pitch ring
// 0.6 -> 4.0. This term sets TIME TO BREAKAWAY, and 0.6 made that time absurd.
// posError grows at targetVel, so the lean ramps at Kpos*Vmax = 4.0*0.25 = 1.0
// deg/s from position and KVEL_I*Vmax = 0.6*0.25 = 0.15 deg/s from here -- about
// 1.15 deg/s toward a ~6-7 deg threshold, i.e. SIX SECONDS of held stick before
// the wheels can break loose. At 4.0 the integral contributes 1.0 deg/s and the
// total is ~2 deg/s, so breakaway lands near 3 s. Still bounded by VEL_I_CLAMP
// and still discarded at the stall -> rolling edge, so nothing carries into the
// cruise. Observed 2026-08-14: forward drive that was ALREADY rolling worked
// fine; it was only a fresh standing start that timed out.
// 4.0 -> 8.0. The other half of the ramp rate: this contributes KVEL_I*Vmax deg/s.
// Measured on the 2026-08-14 bench log, LEAN CMD went 6.12 -> 8.46 over 1.3 s = 1.8
// deg/s, matching the predicted (Kpos + KVEL_I)*Vmax = 8*0.25 = 2.0 deg/s. Reaching a
// ~7 deg breakaway therefore took ~3.5 s of held stick. See LEAN_CLAMP for the new sum.
// 8 -> 16 (2026-08-14). AGGRESSIVE LEAN, requested after reverting the whole
// launch-overdrive line of work back to 28badb1. Authority now comes from the LEAN,
// which the cascade is built around, instead of from a feedforward push fighting the
// inner loop. Ramp is (Kpos + KVEL_I)*Vmax = (6 + 16)*0.40 = 8.8 deg/s, up from 5.6.
// 16 -> 24. Measured ramp at 16 was 8.5 deg/s (MS 137170->138675), matching the 8.8
// design, so the ramp law is right -- it is just not fast enough. Now 24*0.40 = 9.6
// plus Kpos*Vmax 2.4 = 12.0 deg/s, so LEAN_CLAMP 18 is reached in ~1.5 s.
// This also fixes "too slow to react to reverse": a direction change re-anchors
// posSetpoint and zeroes velocityLeanI, so a reversal re-runs the entire ramp from
// zero. Reversal latency IS ramp time.
const float KVEL_I = 24.0f;      // velocity-error memory (rev/s*s -> deg lean)
// RAISED 0.8 -> 2.5. In the 2026-08-14 log this integral ramped to 0.80 in 0.6 s
// and then sat there for the rest of the hold: saturated, and contributing a
// constant. An integral that saturates below the disturbance it is integrating
// against is just an offset. 2.5 lets it actually search for the friction level.
// 0.8 -> 2.5 -> 4.0. Raised again with Kpos: breakaway is ~6-7 deg and the whole
// component sum has to clear it, not just this term.
// 5 -> 9. This sets how long the FAST part of the ramp lasts: once the integral
// saturates only Kpos*Vmax = 2.4 deg/s is left and the last degrees crawl, which is
// what "ramp up is slow" was. At KVEL_I 16 the integral now runs 1.4 s before it pins.
// 9 -> 14. THE CRAWL WAS HERE. At 9 the integral pinned after 1.5 s and the ramp fell
// to Kpos*Vmax = 2.4 deg/s for the last 2.4 deg, which took 1.1 s -- over 40% of the
// total time to breakaway (measured MS 138675->139775). Sized so the integral no longer
// saturates before LEAN_CLAMP binds: 14/9.6 = 1.46 s, and the clamp binds at 1.5 s. The
// component sum is 9 + 1.2 + 14 = 24.2, well over LEAN_CLAMP 18, so that clamp stays the
// single saturation point with the conditional integration behind it.
const float VEL_I_CLAMP = 14.0f; // deg; must exceed the stiction lean, not sit under it
// The integral may only TRIM A CRUISE, never fight stiction. While the wheels are
// not actually rolling, velError reports a stalled drivetrain rather than a speed
// shortfall, and integrating that just winds to the clamp -- so the term arrives
// at breakaway already saturated and then overshoots. Below this speed the
// integral FREEZES (holds its value); it is only zeroed when the stick releases.
// 0.10 -> 0.20. At 0.10 this sat INSIDE the encoder quantization noise: one count
// at 200 Hz is 0.366 rev/s raw, and VRL/VRR show those +/-0.37 spikes all through
// a stick-slip stall. So single wheel twitches were tripping the breakaway edge
// detector and wiping the integral -- the one term meant to accumulate THROUGH a
// stall was being reset by the stall itself. Seen 2026-08-14: VELI -0.06 -> -0.01
// and -0.03 -> -0.00 mid-stall with VF never reading above 0.05. 0.20 requires a
// real roll, clear of a single-count spike.
const float VEL_I_ROLL_MIN = 0.20f;  // rev/s : below this the integral is frozen
// FINDING 2026-08-14, bench: gating the integral OFF below this speed was a
// CATCH-22. It required rolling before the integral could build, but building was
// what would start the roll -- so on a stalled drive VELI sat at 0.01 forever.
// Measured ceiling with it frozen: LEAN POS 3.00 (pinned at POS_ERROR_CLAMP *
// Kpos) + LEAN VEL 1.26 (Kvel * TVEL 0.42 at a 0.70 speed cap) + VELI 0.01
// = 4.27 deg, RAW read 4.10-4.47, LEAN_CLAMP 6.0 never binding.
// RESOLVED: this threshold is now the BREAKAWAY EDGE DETECTOR, not an integrate
// gate. The integral accumulates freely while stalled, and is discarded on the
// stall -> rolling transition -- which keeps the anti-windup property (it never
// carries stiction-fighting wind into the cruise) without blocking the only term
// that could grow. Ramp rate is KVEL_I * velError ~ 0.25 deg/s at full stick, so
// it takes several seconds to matter; raise KVEL_I if that is too slow.

// COAST BRAKING (2026-08-14, "a bit lazy break").
// The integral was zeroed the instant the stick centred (`if (!driving)`), which
// collapsed the loop's braking authority from LEAN_CLAMP 18 deg to whatever
// Kpos*posError + Kvel*velError happened to be -- measured at 1.8 deg on release,
// BELOW the ~6-7 deg breakaway lean. So the robot could not brake at all: it
// coasted on rolling friction alone. Measured from the log, stick centred at
// 0.44 rev/s: 0.35 rev of travel over 1.7 s, ending parked with a permanent
// 0.25 rev position error it never recovered (POS -0.397 against PSET -0.146,
// held by static friction against a 1.68 deg lean).
// Fix: keep integrating while the wheels are still ROLLING, stick or no stick.
// With targetVel 0, velError = -forwardVel, so the counter-lean ramps at
// KVEL_I*|v| = 24*0.44 = 10.6 deg/s and clears breakaway in ~0.6 s instead of
// never. VEL_I_ROLL_MIN is reused as the rest detector, and below it the integral
// is ZEROED, not frozen: a frozen integral parked just above breakaway would
// creep, stop, creep again -- the standstill stick-slip hunt this project already
// knows about (see the 2026-08-14 burst finding in MIGRATION_TEENSY.md).
// This does NOT touch the stick-reversal brake, which is a separate path and is
// authority- and LEAN_LPF-limited rather than dead. See LEAN_LPF.
#define COAST_BRAKE 1

// Keep drive velocity feedback at the neutral-loop gain. Doubling it drove
// leanRaw into the +/-5 deg clamp for most held-stick samples, so encoder ripple
// could only move the command away from saturation and was fed back asymmetrically.
const float DRIVE_KVEL_MULTIPLIER = 1.0f;
// 5.0 -> 10.0 -> 6.0.
//
// THEORY THAT FAILED, recorded so nobody re-derives it: earlier logs showed the
// robot sitting motionless at leans of +0.50 and -3.7 deg, which was read as a
// friction threshold near 3.7 deg that the old 3.10 deg lean ceiling could never
// reach. Raising the ceiling to 10 falsified it. The bench run reached a
// COMMANDED lean of 8.4 deg and an ACTUAL pitch of 12 deg with the wheels still
// producing net +7 encoder counts over 3.1 s -- in the wrong direction. A
// stationary robot at lean X proves the threshold is at least X, not equal to X.
// There is no lean value in this range that breaks the wheels loose.
//
// WHY MORE LEAN CANNOT HELP: commanding lean does not command wheel torque. The
// inner loop drives the wheels only in proportion to (pitch - target), and it
// tracks the ramping target to within +/-1.4 deg throughout. Lean is supposed to
// become motion THROUGH GRAVITY -- lean, body falls, pitch error opens, inner
// loop catches it, and that catch IS the translation. Something is absorbing the
// gravity moment, so the chain never starts, and the outer loop just integrates
// forever against a disturbance it cannot move.
//
// RESOLVED 2026-08-14 by STATIC_LEAN_TEST + the 200 Hz burst capture. The missing
// number was never the ceiling, it was the FLOOR: breakaway lean is ~3.4 deg (see
// STATIC_LEAN_DEG for the lean/velocity table). Every drive attempt had a MEAN
// achieved lean of ~2.9-3.0 -- a few tenths under threshold. The commanded value
// looked adequate; the achieved mean never was.
//
// 6.0 -> 12.0. Has to clear 3.4 with room for transients and for the outer loop
// to modulate above it, not merely exceed it. Still under rc_balance's
// THETA_REF_MAX (0.33 rad = 18.9 deg).
// 12 -> 14 (2026-08-14). The old component sum was Kpos*POS_ERROR_CLAMP + Kvel*Vmax +
// VEL_I_CLAMP = 6.0 + 0.75 + 4.0 = 10.75, BELOW LEAN_CLAMP 12 -- so this clamp never
// fired and the conditional integration hung off it was dead code; the real ceiling
// was an accident of three unrelated limits. New sum is 9.0 + 1.2 + 5.0 = 15.2, so
// LEAN_CLAMP binds again and is once more the single saturation point it claims to be.
// THIS IS THE RISKY NUMBER of the set: the robot was already in a slow topple at ~10
// deg. Back this off first if it starts falling forward instead of driving.
// 14 -> 18. "Allow the bot to fall": the clamp is no longer sized to keep it upright,
// only to stop a runaway setpoint. 18 is essentially rc_balance's THETA_REF_MAX
// (0.33 rad = 18.9 deg), so this is the reference implementation's own ceiling rather
// than an arbitrary one. Component sum is 6*1.5 + 3*0.4 + 9 = 19.2, so this still binds
// and the conditional integration behind it still fires.
// IT WILL FALL FORWARD if the wheels do not break away -- that is the accepted trade.
// 18 -> 12 (2026-08-14). THIS IS NOW A MECHANICAL LIMIT, NOT A TUNING CHOICE.
// Manual tilt sweep, disarmed, measured on the bench:
//     forward rest  pitch +27.31  =  LEAN ACT +30.53   (held 1.2 s)
//     rear settle   pitch -18.89  =  LEAN ACT -15.67   (damped settle, steady)
//     rear hard     pitch -27.29  =  LEAN ACT -24.07   (held 2.0 s)
// The chassis has 30.5 deg of FORWARD lean and only 15.7 deg of REAR lean before it
// sits down on its rear contact. At 18 the commanded pitch is -21.22, i.e. 2.3 deg
// PAST the angle where the robot stops being an inverted pendulum and becomes a
// tripod -- so every full-reverse command drove it onto its own backstop and parked.
// Signature of that deadlock, seen three times: LEAN RAW pinned at -18, encoders
// frozen for 1.2-3.0 s, pitch held at -20.6..-21.5 (the free rest at -18.89 plus
// ~1.8 deg of motor reaction torque, since PWM 7-15 is under the 15/27 breakaway so
// the wheel never turns but the motor still pushes), and u stuck at +0.4..+0.6 deg of
// error so the inner loop kept asking for MORE backward rotation -- which it can only
// get by driving the wheels FORWARD. That is the "reverse commands wheels forward"
// report: correct control law, unreachable target.
// 12 leaves 3.7 deg of margin under the 15.67 sit-down, and forward drive never
// exceeded 9.43 deg of lean even at Vmax 0.90, so nothing that currently works loses
// authority. Braking should IMPROVE rather than suffer: a hard stop used to command
// -18 and put the robot on its backstop, which killed the brake it was asking for.
// Symmetric on purpose. An asymmetric clamp (~26 forward / 12 rear) would recover the
// unused forward range, but nothing needs it yet and one number is easier to reason
// about. The real fix is mechanical: move the rear contact back or up and the rear
// range grows toward the forward 30.5, at which point this can rise again.
// NOTE the fall/recovery latches cannot fire on this chassis -- FALL_CUTOFF_DEG 45 and
// RECOVERY_GIVEUP_DEG 32 are both ABOVE the mechanical maxima (30.53 fwd, 24.07 rear),
// so the robot can never tilt far enough to trip either. That is why it pushed into
// the stop for 3 s instead of giving up. Left alone for now: fixing it properly needs
// per-direction thresholds, and this clamp keeps it away from the stop in the first place.
// SPLIT PER DIRECTION (2026-08-14), immediately after the symmetric 12 shipped.
// A symmetric clamp has to be sized for the WORSE side, and here the two sides differ
// by 2x, so 12 starved forward to fix rear. Measured in the very next drive log:
// LEAN RAW pinned at exactly 12.00 for 4+ s of held forward stick while VELI kept
// climbing (7.42 -> 7.63) and PERR stayed open at 0.55-0.73 -- the loop asking for
// more lean than it was allowed, in the direction that has 30.5 deg of physical room.
// Forward goes back to 18, which is not a guess: it is the value in every good
// forward-drive log to date, and forward has never actually exceeded 9.43 deg of lean
// under it. Rear stays at 12, which is the measured sit-down limit and non-negotiable.
// Gating on sign(leanRaw) is safe here in a way the old friction-FF gates were not:
// both limits are far from zero, so a misclassification near the crossing changes
// nothing (|leanRaw| ~ 0 is nowhere near either bound), and the two branches differ
// only in HOW MUCH authority is allowed -- never in which direction it points.
// CORRECTION, same day: the log cited above was CONTAMINATED -- the USB tether was
// fouling the right wheel. That drag is what held forwardVel at 0.15 against a target
// of 0.51, and a permanently open velError is exactly what winds VELI into the clamp.
// So "forward pinned at 12" is an artifact of the tether, not proof that clean-ground
// forward needs more than 12. Do not cite that log as evidence for anything.
// THE SPLIT STILL STANDS, on the geometry rather than on that log:
//   - the chassis is measurably asymmetric (30.53 fwd vs 15.67 rear), so ONE number
//     has to be sized for the worse side and is wrong-shaped for the better one;
//   - 18 forward is not speculative, it is the value every good forward-drive log ran
//     at, so this restores known-good forward while keeping the new measured rear cap;
//   - it costs nothing measurable: forward has never exceeded 9.43 deg on clean ground,
//     so 12 vs 18 should be invisible until something actually loads the wheels.
// Corollary worth remembering: a dragging wheel and a too-low clamp look IDENTICAL in
// telemetry (RAW pinned, VELI at its ceiling, velError open, robot slower than asked).
// Distinguish them by LEAN ACT vs speed -- a real 13 deg lean must produce g*tan(13) =
// 2.3 m/s^2, and if the robot is not accelerating, something external is eating it.
// Headroom check: 18 forward leaves 12.5 deg to the +30.53 stop; 12 rear leaves 3.7 deg
// to the -15.67 sit-down. If forward pins at 18 under Vmax 0.90 there is room to ~24
// before the geometry bites, but do not go there without a CLEAN log showing 18 saturating.
// 18 -> 24, 2026-08-15. The forward cap was the binding constraint and it was binding against
// nothing: the 2026-08-15 flight held LEAN RAW 18.00 = CMD 18.00 for seconds at a stretch --
// RAW equalling CMD exactly means the request was being truncated -- while TARGET sat at
// BALANCE_SETPOINT + 18 = 14.78 against a forward stop at +30.53. Half the forward range was
// never used. Raising DRIVE_MAX_VEL 0.65 -> 1.00 made this worse by asking for more velocity,
// hence more lean, into the same ceiling.
// 24 puts TARGET at 20.78 and leaves ~9.7 deg to the stop. Forward overshoot is small in
// practice (PITCH tracks TARGET within ~1 deg while driving, peaking ~16 against a 14.78
// target), so that margin is real, unlike the rear where overshoot eats most of it.
// NOT symmetric with the rear, on purpose: the chassis has 30.5 deg forward and ~19 to the rear
// soft contact, and LEAN_CLAMP_REAR 14 already puts the rear TARGET at -17.2, i.e. nearly on
// the stop. Forward has room to give; rear does not. See LEAN_CLAMP_REAR.
const float LEAN_CLAMP_FWD  = 24.0f; // deg : forward authority cap (stop is at +30.53)
// 12 -> 18 (2026-08-14). THE -15.67 SIT-DOWN WAS NOT REAL. A second manual sweep,
// disarmed, back-to-front, measured the rear rest at PITCH -28.5 = LEAN ACT -25.3, held
// steady for a full second -- and swept straight THROUGH -18.9 on the way up at 34-42
// deg/s with no pause. The -15.67 that this clamp was sized against was a dynamic settle
// in the earlier sweep, taken during the period when the tether was fouling the right
// wheel, not a hard contact. Forward agrees across both sweeps (+30.53 / +29.97), which
// is what made the rear disagreement easy to miss.
// Corroborating evidence I should have weighted higher at the time: 18 ran for weeks of
// logs without ever sitting the robot down, and the reported regression after cutting to
// 12 was exactly "it falls and can't recover" plus "dead angles are too narrow".
// Restored symmetric with LEAN_CLAMP_FWD. Rear room is ~25 deg, so 18 leaves ~7 deg of
// margin for the 2.78 deg of measured overshoot -- comfortable, and the same margin the
// forward side has always run with.
// LESSON: a rest angle reached by RELEASING the chassis is not a limit. It is wherever
// the torques happened to balance on that entry. A limit is where it STOPS on a slow
// deliberate push, and it must reproduce across sweeps before anything gets sized to it.
// 18 -> 14. THERE ARE TWO REAR CONTACTS and the sweeps each found a different one:
//   ~-19 to -21  soft/compliant -- the first sweep SETTLED at -18.89 here; the second
//                swept through at 34-42 deg/s without pausing, which is what a
//                compliant contact does to a fast hand sweep. Real, just not hard.
//   -25.3        hard rest, found by the slow sweep, held steady for a full second.
// At 18 the commanded pitch is -3.22 - 18 = -21.22, landing inside the soft band, and
// EVERY logged reverse stall parked at -20.6..-21.5 with the encoders frozen for 1-3 s.
// That is the loop driving itself onto the soft contact: pitch reaches target, ERR falls
// to ~+0.5, u drops to ~2, PWM ~8 which is under the 15/27 breakaway, and nothing moves.
// Reads as "reverse commands the wheels forward" because the inner loop still wants a
// few tenths more backward rotation and rolling forward is how you get it.
// 14 puts the command at -17.22; add the 2.78 deg of measured overshoot and the peak is
// -20.0, just clear of the -20.6 where the stalls begin. That is ~0.6 deg of margin, so
// this is a CEILING not a comfortable setting -- if reverse still parks, go to 13 (-19.0
// peak) before looking anywhere else.
// Braking is barely affected: g*tan(14) = 2.45 vs 3.19 m/s^2 at 18, and DRIVE_MAX_VEL
// 0.65 was sized against 12, so the stopping distance is still better than the config
// that braked acceptably.
// WHAT WOULD SETTLE IT: a SLOW deliberate push through the -17..-23 band, watching for
// where resistance first appears, rather than a full-range sweep that flies past it.
// Both previous sweeps were too fast in exactly this region.
// 14 -> 16, 2026-08-15. Pilot reports the same truncation-and-jitter in reverse as forward,
// and the log agrees: LEAN RAW -14.00 = CMD -14.00, pinned. Raised only 2 deg, not the 6 that
// LEAN_CLAMP_FWD got, because the rear budget is not comparable:
//   BALANCE_SETPOINT -3.22 SUBTRACTS from forward reach and ADDS to rear reach, so the same
//   clamp number is worth 3.2 deg less one way and 3.2 deg more the other. At 24/14 the
//   commanded pitch was +20.78 / -17.22 against limits of +30.53 / ~-19, i.e. 9.7 deg of
//   forward margin and 1.8 of rear.
// 16 puts the rear TARGET at -19.2, essentially ON the documented soft contact.
// ⚠️ THE CONTACT FIGURE IS THE REAL UNCERTAINTY AND IT IS UNRESOLVED. This file says soft
// contact ~-19; the 2026-06 chassis measurement says the tail sits down at -15.7. Those cannot
// both be right, and the pilot reports visible clearance at the current lean, which fits
// neither. Everything above is arithmetic on a number nobody has re-measured.
// MEASURE IT BEFORE GOING FURTHER: disarmed, tip the chassis back by hand until the tail
// touches, read PITCH off the telemetry. If it is past -22 this can go to 18-19 and match the
// forward change; if it is near -16 then 14 was already too much and the earlier rear-stall
// diagnosis (chassis resting on its tail under full reverse) applies at this setting too.
const float LEAN_CLAMP_REAR = 16.0f; // deg : rear cap; soft contact ~-19 (DISPUTED), hard rest -25.3
// 0.99 -> 0.97 (tau 0.50 s -> 0.17 s, pole 2 -> 6 rad/s). The velocity loop
// crosses over near 2.5 rad/s, so the old 2 rad/s pole sat essentially ON the
// crossover and ate ~51 deg of phase exactly where it hurt. That lag is what
// turns velocity regulation into the observed slow hunt: lean builds, the robot
// accelerates, velError closes, the lean collapses, friction decays the speed,
// and the loop needs half a second to notice and rebuild -- "reaches the lean,
// forgets it, balances, again and again". At 0.17 s the pole contributes ~23 deg
// instead, recovering ~28 deg of margin.
// The old warning that 0.95 "RAN AWAY" was recorded against the DIRECT-LEAN
// structure that predates the cascade, where a held lean was a held acceleration
// command with nothing closing the loop on translation. In the always-closed
// cascade the outer loop derives lean from measured position/velocity error, so
// that failure mode does not carry over unchanged -- but this is still the knob
// to slow down first if the outer loop starts oscillating.
// 0.97 -> 0.95 (2026-08-14): "when I try to brake, it takes too much time to take
// effect and the bot crashes sometimes". This is the LAG half of that complaint.
// tau = DT/(1-alpha): 0.005/0.03 = 167 ms at 0.97, 0.005/0.05 = 100 ms at 0.95.
// Measured cost at 0.97 on a full reversal: leanRaw reached the clamp in 400 ms, then
// leanCmd needed another 700 ms to follow it (-13.42 -> -17.94, which is 4.3 tau and
// matched the model to two decimals). At 0.95 that tail is ~410 ms, so ~290 ms comes
// off a maneuver that is running out of room.
// Not lower yet: the outer-loop corner goes 0.95 Hz -> 1.6 Hz against an inner loop
// closing around 3-5 Hz. Cascade rule of thumb wants the outer loop 3-5x slower than
// the inner, and 1.6 Hz is already only ~2-3x. 0.94 is the next step if 0.95 is clean;
// past that the two loops start arguing. This is still the first knob to RAISE if the
// outer loop oscillates -- the old "0.95 ran away" note predates the cascade and does
// not carry over unchanged, but it is not nothing either.
// THIS DOES NOT FIX THE ASYMMETRY, and cannot: forward->backward braking needs REAR
// lean, capped at 12 by the chassis, while backward->forward braking gets the forward
// 18. That is 2.08 vs 3.19 m/s^2 of available deceleration (g*tan), i.e. the good
// direction has 53% more, which is exactly the reported "way faster and better".
// Software cannot close that gap; moving the rear contact can. See LEAN_CLAMP_REAR.
// WATCH: a faster command means a faster body rotation into the lean, and rotational
// momentum means pitch overshoots leanCmd by more, not less. Rear margin is only
// 3.7 deg (clamp 12 vs sit-down 15.67) and overshoot was already ~1.3 deg in steady
// drive. If LEAN ACT touches -15.7 during a hard stop the robot is ON its backstop
// mid-brake, which removes the braking entirely -- and that is the likeliest
// explanation for "crashes sometimes". If that shows up, this goes back to 0.97 and
// the answer is mechanical, not a gain.
// 0.95 -> 0.97, reverted same day. 0.95 was never flown; it was traded away as soon as
// the overshoot budget became clear. Rear margin is the scarce resource, not response
// time: pitch overshoot past leanCmd is what consumes the 3.7 deg between LEAN_CLAMP_REAR
// 12 and the -15.67 sit-down, and a faster command rotates the body faster into the lean,
// so it overshoots MORE. 0.97 is also the only alpha where the overshoot is actually
// measured (2.78 deg peak, rear side, MS 32705 of the good brake log) -- going faster
// invalidated that number in the dangerous direction.
// EXCHANGE RATE, still unmeasured: slower leanCmd -> less overshoot -> more of the 15.67
// usable -> a higher LEAN_CLAMP_REAR, which is worth ~8.5% deceleration per degree. That
// may well beat the lag it costs, but it is a guess until the overshoot is measured at
// this alpha. Do that before going to 0.98 (tau 250 ms), or the trade is blind in both
// directions: giving up known response time for unknown headroom.
const float LEAN_LPF   = 0.97f;  // EMA on leanCmd (~170 ms). Higher = slower outer loop.
// HARD BACKSTOP ONLY. The primary anti-windup is now conditional integration
// against LEAN_CLAMP, applied in the outer loop below.
//
// WHY THIS STOPPED BEING THE ANTI-WINDUP MECHANISM -- the 2026-08-14 log. Held
// full stick for 1.9 s, POS flat at 0.535 +/- 0.005 (zero travel against 1.14 rev
// commanded), and every one of the three lean terms was independently capped:
//     LEAN POS  0.50  <- PERR pinned at POS_ERROR_CLAMP 0.5 * Kpos 1.0
//     LEAN VEL  1.80  <- Kvel 3.0 * velError, and with the wheels STOPPED
//                        velError is identically targetVel, so this term is
//                        capped at Kvel * DRIVE_MAX_VEL by construction
//     LEAN VELI 0.80  <- saturated at VEL_I_CLAMP after 0.6 s
//     -------------
//     total     3.10 deg -- a hard arithmetic ceiling the loop could not exceed
//                           however long the stick was held.
// That ceiling was real. The INFERENCE drawn from it -- that lifting it would let
// the robot break away -- was wrong; see the failed-theory note at LEAN_CLAMP.
// The structural argument below still stands on its own merits.
// LEAN_CLAMP was 5.0 and 1.9 deg of it went permanently unused. The loop could
// not ask for more lean no matter how long the stick was held -- not a tuning
// shortfall but an arithmetic ceiling, because the only two terms that can
// ACCUMULATE were both clamped below the disturbance.
//
// THE STRUCTURAL FIX: clamp the OUTPUT, not the input. Saturating the position
// ERROR caps the loop's DC gain at Kpos*POS_ERROR_CLAMP forever; saturating the
// output preserves full authority right up to the limit. This is what
// librobotcontrol's rc_balance does -- setpoint.phi - phi is never clamped, only
// D2's theta_ref output is (at THETA_REF_MAX). Raised 0.5 -> 1.5 so this stays a
// safety backstop against a runaway setpoint rather than the binding constraint.
const float POS_ERROR_CLAMP = 1.5f;   // rev : backstop, NOT the anti-windup path

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
// Effort over which the stiction floor ramps in from 0 to full, instead of stepping.
// Sized against the observed ring: u swings +/-9 during a drive, so 5.0 softens more
// than half of that amplitude while leaving the floor fully applied for real drive
// efforts (9-30). Raise it if the 8 Hz ring persists; lower it if breakaway suffers.
// 5.0 -> 2.5. THE REAR BREAKAWAY DEAD ZONE. Clean tether-free reverse log: the stall
// angle FOLLOWS THE COMMAND (clamp 18 parked at pitch -20.6, clamp 14 parks at -17.2),
// which rules out the fixed rear contact I chased for several commits. What actually
// happens is that the loop reaches its commanded lean, ERR falls to ~0, and there is
// no effort left to break static friction with.
// The arithmetic: pwm = deadband*(|u|/FLOOR_KNEE) + |u|, so at FLOOR_KNEE 5 the RIGHT
// wheel needs |u| >= 3.7 just to reach its own 27-count static floor. In the stall |u|
// oscillates 0.15..3.7 and flips sign every tick or two -- peak PWM measured 23 against
// a breakaway of 27, never once reached. A ramped floor below the knee is by
// construction incapable of breaking away: it is a dead zone of +/-3.7 effort.
// Forward escapes it by breaking away early during the lean ramp-in, when ERR is still
// large, after which the MOVING deadband is only 11/13.
// 2.5 halves the dead zone to |u| >= 1.9 for the right wheel. NOT lower: the knee exists
// because a floor applied as a STEP at the zero crossing has infinite incremental gain
// and generated an 8 Hz limit cycle (see the note at DRIVE_FRICTION_FF). 2.5 keeps a
// ramp, just a steeper one. If an 8 Hz buzz reappears at standstill, go back up.
// The right wheel's 27 vs the left's 15 is the real asymmetry underneath this -- the
// deadband sweep (DEADBAND_TEST, wheels off the ground) is what would decompose that
// into BTS7960 turn-on delay vs actual friction, and it has still never been run.
// 2.5 -> 1.0, 2026-08-15. The ramp existed to cut incremental gain at the zero crossing
// and starve the 8 Hz limit cycle. That cycle turned out to be the CHATTERING FLOOR (see
// mapEffortToPwm) and died with it: post-fix burst shows PWM holding one sign for tens of
// samples, |u| ~1.5 not 4-6, RATE +/-3 not +/-30, and no periodicity at all. With the
// cycle gone the ramp is pure loss, and it was the thing blocking breakaway.
// Measured, same flight: 5.7 s at full stick moved the wheels 17 counts while LEAN CMD
// wound to 16.6 deg. pwm = floor*(|u|/KNEE) + |u|, checked against the log --
//     U 1.06 -> 17.9*0.424 + 1.06 =  8.6 -> PWML  8
//     U 1.88 -> 17.9*0.752 + 1.88 = 15.3 -> PWML 15
// Sustained effort ran |u| 0.5..1.9, so PWM sat at 8..15 against a breakaway of 18. The
// wheel COULD NOT move, however long the stick was held; the inner loop tracked its lean
// happily the whole time (ERR ~0.2) because the failure is downstream of it.
// At 1.0 the same |u| 1.06 gives 17.9 + 1.06 = 19.0 and clears breakaway. Chosen as the
// value where the OBSERVED sustained efforts clear the MEASURED floor -- not tuned by feel.
// NOTE this is the opposite direction from what the old comment here implied, and lower
// KNEE means HIGHER incremental gain near zero (floor/KNEE + 1 = 18.9, was 8.2). If a
// limit cycle returns, do NOT simply raise this back: the principled fix is to ramp on the
// LOW-PASSED |u| instead of the instantaneous one, so a sustained demand gets the whole
// floor while a dithering one averages to nothing. That machinery already exists at
// DRIVE_FRICTION_FF and is currently disabled.
// ⚠️ REVERTED 1.0 -> 2.5, 2026-08-15, SAME DAY. KNEE 1.0 was strictly worse: it brought
// the limit cycle back BIGGER and did not fix breakaway either. Flight burst at 1.0:
//   * u peaks every ~27 samples at 200 Hz = 7.4 Hz, |u| up to 10 (was ~1.5 at KNEE 2.5)
//   * PWM slams 0 -> +/-18..28 with a zero crossing every ~10-13 samples. At KNEE 1.0 the
//     ramp saturates by |u| = 1.0, so the map is a near-RELAY: B133 u -0.61 -> -11,
//     B134 u 0.78 -> +14, B135 u 2.40 -> +19. Incremental gain 18.9, as predicted.
//   * ENC deltas still 0-3 counts. The wheel gets +20 for ~65 ms then -20 for ~65 ms and
//     nets zero, which is the ORIGINAL failure with a louder amplitude.
// THE LESSON, and it invalidates the reasoning that led to 1.0: what blocks breakaway is
// not the PWM magnitude at a given |u|, it is that u itself DITHERS THROUGH ZERO. Any
// memoryless map of |u| to PWM alternates with it and nets ~0 thrust; raising the floor
// gain only makes the dither more violent. The map is the wrong place to fix this.
// The fix has to use TIME: ramp on the LOW-PASSED |u| so a sustained demand earns the
// whole floor and a dithering one earns none. That machinery is at DRIVE_FRICTION_FF and
// is disabled. Do not touch FLOOR_KNEE again as a breakaway lever.
const float FLOOR_KNEE     = 2.5f;  // PWM-effort units

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
// DIRECTION IS NOT A VARIABLE. Settled 2026-08-15, 6 interleaved free-spin passes:
//               L                    R
//   back(+1)    12, 11, 10  -> 11.0   9, 11, 9  -> 9.7
//   fwd (-1)     9,  8, 12  ->  9.7   9, 10, 9  -> 9.3
// All 12 samples lie in 8..12. Each wheel's OWN spread (2-4 counts) exceeds the
// fwd/rev difference (1.3 on L, 0.3 on R) and the ranges overlap in both directions,
// so the fwd/rev gap is not resolvable above the noise. mapEffortToPwm keeps ONE
// constant per wheel for both signs; a per-sign floor would be fitting a constant to
// noise. Do not reopen this without a measurement that beats +/-2 counts.
// The old caveat here (backlash/gearbox preload, "the turn logs hint that it does")
// and the mirrored-motor argument in a06ce6c are both answered: NO.
//
// L@21 DID NOT REPRODUCE. Six passes, max L is 12. The two historical L@21 events
// (1200 Hz sweep, and one 2000 Hz run) are a rare excursion -- 2 in 8 backward ramps,
// 0 in 3 here -- not a bimodal constant. Nothing to compensate; noted so the next
// person who meets one knows it is a known rare event and not a new regime.
//
// LEFT AND RIGHT ARE THE SAME WHEEL OFF THE GROUND: ~10 counts, both channels, both
// directions, now across three independent free-spin datasets. The 9/10 MOVING split
// below is BELOW the resolution of the measurement -- it is not evidence of a real
// difference, and nothing should be built on top of it.
// So the loaded 13-vs-24 asymmetry is LOAD-DEPENDENT: it appears only with the
// robot's weight through the gearbox and vanishes free-spinning. Not electrical
// (frequency sweep), not directional (this test), and not the rig -- pilot confirms
// no hand side-load during the loaded runs (2026-08-15).
//
// ⚠️ NOT MECHANICAL. RETRACTED 2026-08-15, and the 2026-08-13 "suspect a MECHANICAL
// fault on the right drivetrain" warning above goes with it. Pilot, hands on the
// hardware: the right wheel is tighter to turn by hand ONLY WHEN THE ROBOT IS POWERED.
// Power off, both wheels spin the same. Friction does not switch off with the battery.
//
// That is ELECTROMAGNETIC BRAKING. A motor spun by hand is a generator; short its
// terminals and the current produces drag, leave them open and it coasts. On a BTS7960
// both inputs LOW pulls both outputs low, shorting the motor -- brake. Kill the supply
// and both bridges go high-impedance, so both wheels coast and feel identical. Exactly
// the reported behaviour.
//
// The firmware idle state is SYMMETRIC -- motorRaw(0) and motorWriteWheel(pwm 0) both
// write 0 to all four pins -- so the asymmetry is downstream of this file: enable (INH)
// wiring, a bridge module difference, or a supply/ground fault on one channel.
//
// ISOLATED 2026-08-15, pilot by hand: LEFT had no drag either powered or unpowered.
// RIGHT dragged only when powered. Two identical bridges idling in OPPOSITE states.
// Both inputs low brakes ONLY if the enable pins are held high, so one module's
// R_EN/L_EN were not doing what the other's were.
//
// ✅ FOUND AND FIXED 2026-08-15 (pilot): it was the EN wiring. Both wheels now behave
// identically when powered.
//
// ⚠️⚠️ THEREFORE EVERY DEADBAND CONSTANT BELOW IS STALE. All of them -- 9/10 moving,
// 13/24 static, and the 11/13 and 15/27 figures they descend from -- were measured
// with one bridge braking at idle and the other coasting. The measurements were not
// wrong, but the machine they described no longer exists.
// The 13-vs-24 split is the urgent one: it now hands the right wheel 11 counts of PWM
// to overcome a brake that has been removed. That is a real differential injected into
// a balancing robot, and it will veer. DO NOT FLY until both are re-measured.
// It also retires the last of the mechanical-fault thread: there was never anything
// wrong with the right drivetrain, and the 2026-08-13 warning about a "MECHANICAL
// fault ... over-tight hub, rubbing tire, misaligned motor mount" was chasing a jumper.
//
// RE-MEASURE BOTH, 6-pass -- BOTH DONE 2026-08-15:
//   1. ✅ OFF THE GROUND -> *_MOVING, both 10.
//   2. ✅ LOADED         -> *_STATIC, both 18. The 13-vs-24 split did not survive.
// Consequences to walk through before flying, because three things were tuned against
// an asymmetry that is no longer in the machine:
//   - MAX_DEADBAND drops 24 -> 18, so the saturation-latch ceiling moves:
//     SAT_EFFORT_FRAC*(MAX_PWM-MAX_DEADBAND) goes 81.7 -> 87.4, and the 78.85 quoted
//     at FALL_CUTOFF_DEG was staler still (it dated from RIGHT_DEADBAND_STATIC 27).
//     A DIFFERENT trip point, on top of the Kp-vs-Kpeff bug already open in that latch.
//   - FLOOR_KNEE 2.5 sets incremental gain deadband/FLOOR_KNEE + 1, which was 10.6 on
//     the right wheel and is now 8.2 on both. It was chosen against the limit cycle the
//     old asymmetry produced.
//   - driveControlDiff's one-floor-for-both-wheels fix exists because per-wheel floors
//     injected a differential. With the floors now EQUAL that differential is zero and
//     the max() is a no-op -- still correct, but no longer load-bearing.
//
// THE L@21 ANOMALY IS **NOT** CLOSED. It briefly looked closed: post-fix free-spin put
// the LEFT at spread 1 in both directions, against 8..12 with 21-count outliers before,
// and the obvious story was an enable that was not reliably asserted leaving the
// half-bridge undriven at low duty. That story is WRONG, or at least incomplete.
// The 2026-08-15 loaded CONFIRMATION run, taken after the EN fix, pass 1:
//     dL EXACTLY 0 for 32 consecutive steps while dR broke at 18 and reached -496.
//     L finally moved at 33.
// Same signature as every historical event -- left stuck, right spinning freely -- so
// whatever this is survived the EN repair.
//
// IT IS THE USB TETHER, almost certainly. A fourth loaded run with a BETTER TETHER put
// the dropout in PASS 2 (dL exactly 0 from PWM 0 to 26, then 37 at 27), killing the
// "always the first ramp after power-up" pattern. And this is a KNOWN rig artefact,
// recorded 2026-06-30: the tether drags the LEFT wheel, and it was misread as a
// friction asymmetry that time too. It accounts for every feature at once -- always the
// left, EXACTLY zero counts (a wheel physically held, not a noisy sensor), intermittent,
// and clearing once the ramp has enough PWM to drag the snag free.
// So this is not an encoder dropout and not a drive fault. Before spending another
// session on it: watch the left wheel during a stall and see whether the cable is
// holding it. The measurement that actually settles it is an UNTETHERED run.
//
// THE TETHER DOES NOT INFLATE THE BASELINE -- RETRACTED. Three runs read L 18/17/16 and
// R 18.5/17.5/16.5 as the tether improved, which looked monotonic and got committed in
// 16272f6 as a ~2-count bias. A fourth run on the same tether came back L 18 / R 19.
// Three points were not a trend. The tether causes discrete SNAGS; there is no evidence
// it shifts the baseline. (Reading a trend out of three samples is the exact error this
// whole thread has been about -- it does not stop being tempting once you have named it.)
//
// WATCH, NOT YET ACTIONABLE: post-fix R backward reads 13, 12, 9. Across both sessions
// R-back is 9,9,9,11,12,13 against R-fwd 9,9,9,10,10,10 -- an upward tail on one side
// only. Its own spread swamps the fwd/rev gap, so it is not a direction effect by this
// file's own rule, and 3 samples per cell cannot carry a constant. Re-check if the
// loaded run shows the same one-sided tail.
// Expect FLOOR_KNEE 2.5 to need revisiting too -- it was tuned for the limit cycle
// that the old asymmetry produced.
//
// WHY THIS IS THE MOST PROMISING LEAD IN THE FILE: brake drag on ONE channel opposes
// breakaway from rest on that wheel and nothing else, which is the shape of the data --
// the STATIC figures differ by 11 counts while the MOVING figures differ by 1. If the
// right bridge brakes at zero command and the left coasts, the 13-vs-24 split is an
// artefact of the electronics and there is nothing wrong with the drivetrain at all.
// It would also be FIXABLE rather than compensated: coast (high-Z) at zero command
// instead of braking. That needs the INH pins under firmware control; they are not
// wired to the Teensy today.
// DO NOT touch these constants until it is localised -- they are currently the only
// thing making that wheel usable, whatever the cause turns out to be.
// RESCALED for MOTOR_PWM_HZ 4482 -> 2000 (2026-08-14). These were all measured at 4482.
// The free-spin sweep gave 4482 L13/R12 -> 2000 L11/R9, i.e. deltas of -2 and -3, and
// the BTS7960 turn-on delay is an ADDITIVE pwm-count offset that does not depend on
// mechanical load -- so the same deltas transfer exactly to the loaded figures. This is
// arithmetic, not extrapolation.
// Still worth a loaded re-measure to confirm: hold the robot upright, wheels on the
// ground, DEADBAND_TEST 1. If the right static comes back near 24 this was right.
// MOVING: re-measured 2026-08-15 AFTER the EN fix, 6 free-spin passes, median of 3
// samples per wheel per direction. L 9,9,10,10,10,9 and R 9,9,10,10,12,13 -- median 10
// on both, and no systematic L/R difference survives, so they are SYMMETRIC now. The
// old 9/10 split expressed a difference the data never supported; equal floors also
// stop injecting a differential into a command that should have none.
const int LEFT_DEADBAND_MOVING  = 10;   // free-spin median, post-EN-fix 2026-08-15
const int RIGHT_DEADBAND_MOVING = 10;   // ditto -- keep these two equal without evidence
// STATIC: re-measured LOADED 2026-08-15 after the EN fix, 6 passes.
//     back(+1)   L 16, 11, 18     R 18, 17, 19
//     fwd (-1)   L 21, 18, 19     R 22, 18, 19
// Medians 18 and 18.5; per-pass L-vs-R differences are 2, 1, 6, 0, 1, 0. The 13-vs-24
// split is GONE and both wheels take 18. (The L@11 in pass 3 is a detector artefact --
// the wheel twitched 6 counts and stalled until 16; deadbandDetect now rejects that.
// Reading it as 16 puts L at 16,16,18 / 21,18,19.)
//
// HONEST CAVEAT ON WHY IT WENT AWAY: both wheels converged on ~18, which is close to
// the mean of the old 13 and 24 -- and the original 2026-08-13 estimate was 18 before a
// single ramp "corrected" it to 15/27. The loaded spread is wide (L 11..21, R 17..22),
// so one ramp reading L@15 R@27 is fully consistent with both wheels having been ~18
// all along. The pre-fix loaded test was never run 6-pass, so "the EN fix removed the
// asymmetry" and "the asymmetry was a single-sample artefact" cannot be separated.
// Both fit. What is certain is that it is not there now, and that neither story
// supports carrying an 11-count differential.
//
// CONFIRMED by a second loaded 6-pass run with the deadbandDetect twitch fix in place:
//     back(+1)  L 33, 18, 17    R 18, 17, 19
//     fwd (-1)  L 16, 20, 16    R 17, 21, 17
// Discarding the pass-1 L@33 dropout (see the L@21 note above -- it is a channel fault,
// not a breakaway), L reads 16,18,20,17,16 and R reads 17,17,18,19,21,17. Both median
// 17-18 across two independent loaded runs, so 18/18 stands.
//
// FOUR loaded runs now, 24 samples per wheel, artifacts excluded:
//     run 3  L 17,27*,15,18,16,16   R 20,16,16,19,17,16   (*tether snag)
//     run 4  L 19,19,14,17,17,19    R 20,19,16,19,17,20   (clean, no dropout)
// POOLED across all four: 21 valid L samples median 17, 24 valid R samples median 18.
// Same as the three-run pool, so the figure is stable and 18/18 stays. R runs about a
// count above L consistently, which is not enough to split them -- and splitting would
// reintroduce exactly the differential this file spent a session removing.
//
// STOP MEASURING. Four runs agree; further ramps buy noise, not precision. Erring high
// is the safe side anyway: this project's failure mode has always been a wheel that
// will not break away, never one that kicks too hard.
//
// These are MEDIANS OF A WIDE DISTRIBUTION, not thresholds. Half the samples sit above
// 18; loaded stick-slip is severe and always has been. Do not treat 18 as the PWM at
// which the wheel is guaranteed to move.
const int LEFT_DEADBAND_STATIC  = 18;   // loaded median, post-EN-fix 2026-08-15
const int RIGHT_DEADBAND_STATIC = 18;   // keep equal without evidence to split them
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
// 0.6 -> 0.25. Measured 2026-08-14: above the ~3.4 deg breakaway the chassis does
// about 0.05 rev/s per extra degree of lean, so 0.6 rev/s would need ~15 deg --
// unreachable. An unreachable target keeps velError permanently open, which pinned
// the outer loop in saturation in every earlier drive log. 0.25 rev/s corresponds
// to roughly 8-9 deg of lean, inside LEAN_CLAMP with margin.
// 0.25 -> 0.40 (2026-08-14). This is a THIRD ramp-rate lever, not just a speed cap:
// both outer-loop ramp terms scale with it, so the total is (Kpos + KVEL_I)*Vmax.
// It also improves the velocity feedback rather than hurting it -- at 0.25 rev/s the
// target sat BELOW one encoder count per tick (0.366 rev/s raw), so the loop was
// regulating against a signal that was mostly quantization. 0.40 is still well under
// the old 0.6 and gets a fuller x1 count per tick.
// 0.40 -> 0.60 (2026-08-14), from v2-drive-works. NOTE THIS IS NOT JUST A SPEED CAP:
// both outer-loop ramp terms scale with it, so the lean ramp goes (Kpos + KVEL_I)*Vmax
// = 30*0.60 = 18 deg/s, up from 12, and LEAN_CLAMP 18 is now reached in ~1.0 s instead
// of ~1.5 s. Expect it to lean into a command noticeably harder, not only run faster.
// Encoder resolution improves too: at 0.25 the target sat BELOW one count (0.366 rev/s
// at 200 Hz x1), at 0.40 it was ~1.1 counts, at 0.60 it is ~1.6 -- the velocity loop
// finally has real signal rather than mostly quantization.
// Ceiling is unchanged: 6*1.5 + 3*0.60 + 14 = 24.8, still clamped by LEAN_CLAMP 18.
// 0.60 -> 0.90 (2026-08-14), "too slow, too sluggish, cannot overcome minor bumps".
// EVIDENCE THIS IS THE RIGHT KNOB and not LEAN_CLAMP: in the 0.60 drive log the loop
// was TRACKING its target, not straining against a ceiling -- TVEL 0.56 vs VF 0.55,
// VERR 0.01, and LEAN RAW peaked at 9.43 against a clamp of 18 that never bound on
// forward acceleration. The robot delivered exactly the speed it was asked for, so
// the speed request is what was low. Raising LEAN_CLAMP would have done nothing here.
// Ramp goes (Kpos + KVEL_I)*Vmax = 30*0.90 = 27 deg/s, up from 18, so LEAN_CLAMP is
// reachable in 0.67 s instead of 1.0 -- it answers "sluggish" as well as "slow".
// Bumps ride on the same term: a bump drops forwardVel, and the bigger the target the
// bigger the resulting velError, so VELI winds at KVEL_I*velError and converts the
// shortfall into lean (hence torque) faster than it did at 0.60.
// DRIVE-ONLY BY CONSTRUCTION -- no gate needed. targetVel is identically 0 with the
// drive stick centred, so every term this scales vanishes in plain balance and the
// balancer keeps its tuning untouched. CH3 still scales it (cap 0.35..1.0), so this
// raises the top of the throttle range rather than the whole range.
// Ceiling invariant holds: 6*1.5 + 3*0.90 + 14 = 25.7, still over LEAN_CLAMP 18, so
// that clamp remains the single saturation point behind the conditional integration.
// Encoder resolution keeps improving: 0.90/0.366 = ~2.5 counts per tick at full stick.
// WATCH: if LEAN RAW now starts pinning at +/-18 during forward acceleration (it did
// not at 0.60), the clamp has become the binding constraint and IT is the next lever
// -- and it would then need a drive-only gate, which this constant does not.
// 0.90 -> 0.65 (2026-08-14). SIZED BY STOPPING DISTANCE, not by feel. Reported: braking
// from forward takes too long and the robot crashes; the pilot's own read was "it keeps
// driving forward to reach the lean, can't reach it, and crashes" -- which is the
// non-minimum-phase surge (wheels must roll FORWARD to plant a backward lean; measured
// at VF 0.65 -> 1.01 before the turnaround) failing because there is not enough room or
// wheel authority left at speed.
// The arithmetic I should have run when raising this. Stopping distance goes as
//     d  ~  v^2 / (g * tan(theta_rear))
// and the last few commits moved BOTH terms the wrong way at once:
//     was:  Vmax 0.60, rear lean 18 deg -> a = 9.81*tan(18) = 3.19 m/s^2   (braked fine)
//     then: Vmax 0.90, rear lean 12 deg -> a = 9.81*tan(12) = 2.09 m/s^2
//     ratio = (0.90/0.60)^2 * (3.19/2.09) = 2.25 * 1.53 = 3.4x the stopping distance.
// The 12 is not negotiable -- the chassis sits down at 15.67 deg of rear lean, see
// LEAN_CLAMP_REAR -- so the speed is the only term left to move.
// Reference points at the 12 deg rear cap, relative to the config that stopped well:
//     0.49 = same distance    0.60 = 1.5x    0.69 = 2x    0.90 = 3.4x
// 0.65 lands at ~1.8x: still quicker than the old 0.60 setup but stoppable. Go to 0.60
// or 0.49 if it is still running out of room. Ramp comes down with it, (Kpos+KVEL_I)*Vmax
// = 30*0.65 = 19.5 deg/s, so the forward clamp is reached in ~0.9 s.
// THE REAL FIX IS MECHANICAL. Every degree recovered at the rear contact buys back
// deceleration directly, and 18 deg rear would allow 0.90 again at this same distance.
// TARGET VELOCITY SLEW (2026-08-14). The loop had no concept of BRAKING: it only ever
// tracked a setpoint, so when momentum opposed the command all three outer terms
// saturated at once and stayed there. Measured on a real forward->reverse slam
// (MS 31590): LEAN POS -3.24, VEL -4.37, VELI -10.86, RAW -18.47 -> clamped. The
// INTEGRAL was doing 59% of the braking -- a term that exists to trim a cruise had
// become the primary brake authority, and it is the slowest thing in the loop.
// velError sat at -1.25 rev/s for the whole maneuver, so VELI wound at KVEL_I*1.25 =
// 30 deg/s and pinned in ~0.4 s. Consequences, all bad:
//   1. the brake is bang-bang, not proportional -- max lean from 0.4 s until the
//      velocity crosses zero, equally committed at 0.6 rev/s and at 0.05;
//   2. a pinned maximum command is exactly what drives pitch past the -15.67 sit-down,
//      because the demand never tapers so neither does the body rotation;
//   3. at the crossing VELI is still at -13 and must unwind from saturation, so it
//      over-reverses on the far side.
// Fix is setpoint shaping, not another output clamp: ramp targetVel at a rate the robot
// can actually follow, so velError stays proportional to the REMAINING deceleration
// need. VELI then tracks instead of saturating and the lean bleeds off with the speed.
// RATE, derived not guessed: the good brake shed 1.01 -> 0 rev/s in ~1.0 s at 18 deg of
// rear lean = ~1.0 rev/s^2. Scaling by tan(12)/tan(18) = 0.655 for the current rear cap
// gives ~0.7 rev/s^2, i.e. this asks for almost exactly the deceleration the chassis can
// produce and no more.
// ASYMMETRIC ON PURPOSE, and this is the part that matters: motion AWAY from zero is
// rate-limited, motion TOWARD zero is instant. So releasing the stick still zeroes the
// demand immediately (no added coast), backing off is immediate, and a full reversal
// snaps the target to 0 first -- halving the initial velError spike from -1.25 to -0.6 --
// and only then ramps out the far side at a plantable rate.
// Why the sign test is not the kind of gate that killed the friction FF: it fires once
// per direction change, its only effect is to move the target to ZERO (a value between
// the old and new ones, never outside them), and it can never reverse the sign of
// anything. A spurious fire costs a momentary drop in demand, not a full-amplitude PWM
// flip. It also cannot chatter in the deadzone: DRIVE_STICK_DEADZONE makes driveIn
// exactly 0 there, and 0 * x is never < 0.
// Starts are not the cost they look like: 0.65 rev/s at 0.7 rev/s^2 is 0.93 s, and the
// existing lean ramp is (Kpos + KVEL_I)*Vmax = 19.5 deg/s = ~0.9 s to the forward clamp.
// The velocity ramp lands on top of a ramp that was already there, so it does not become
// the binding constraint. Set TARGET_VEL_SLEW_LIMIT 0 to A/B it on the same flash.
#define TARGET_VEL_SLEW_LIMIT 1
const float TARGET_VEL_SLEW = 0.70f; // rev/s^2 : cap on how fast the velocity DEMAND grows
// 0.65 -> 1.00, 2026-08-15, pilot asked for more speed. The old "conservative on purpose"
// note below was about ENCODER RESOLUTION, and resolution gets BETTER as speed rises: one
// count per 5 ms tick is 0.366 rev/s, so 0.65 is ~1.8 counts/tick and 1.00 is ~2.7. The
// resolution argument constrains the LOW end, not this. The machine already demonstrates
// the headroom -- VF hit 0.86 during aggressive drive on 2026-08-15 while the target was
// capped at 0.65, i.e. it overshoots the old cap under its own power.
const float DRIVE_MAX_VEL  = 1.00f; // rev/s at full stick. Was 0.65. Original note: one encoder
                                    // count per 5 ms tick is already 0.366 rev/s (546 counts/rev),
                                    // so 0.6 rev/s is only ~1.6 counts/tick of resolution. Raise
                                    // this after the x4 hardware QuadEncoder upgrade (2184 cpr).
// FLOOR ON THE COMMANDED SPEED, 2026-08-15, pilot request: more rpm and a NARROWER rpm range
// on the throttle, with the lower bound raised. The stick used to map linearly from zero, so
// just past the deadzone it asked for ~0.09 rev/s -- inside the regime where the wheels dither
// without rolling (2026-08-15 flight: sustained |u| 0.5..1.9 gave PWM 8..15 against a measured
// breakaway of 18, and the encoders moved 17 counts in 5.7 s). Anything the pilot could ask for
// down there was unachievable, so the bottom of the stick travel was dead.
// Now the stick maps its live travel onto DRIVE_MIN_VEL..DRIVE_MAX_VEL, so the smallest command
// that is not zero is one the machine can actually execute. Observed rolling speeds are 0.2..0.5
// rev/s and stuck ones are 0..0.1, so 0.30 sits clearly in the rolling regime.
// The CH3 cap still scales BOTH ends, so slow mode stays slow -- the floor is a fraction of full
// authority, not an absolute minimum speed. At cap 0.10 the floor is 0.03 rev/s and inert.
// This does NOT fix breakaway, it stops ASKING for motion below it. If the wheels still refuse
// at minimum stick, raise this; if it lurches off the centre detent, lower it.
const float DRIVE_MIN_VEL  = 0.30f; // rev/s just outside the stick deadzone (scaled by the cap)
// TURN MIXER PRIORITY (2026-08-14). Turning is differential -- effortL = u + turnCmd,
// effortR = u - turnCmd -- so the fore-aft force, which is the SUM, is exactly 2u and
// the turn cancels out of the pitch axis entirely. That is why turning has never upset
// the balancer. BUT THE CANCELLATION ONLY HOLDS WHILE NEITHER WHEEL SATURATES.
// The moment one wheel hits MAX_PWM, the clip is ONE-SIDED: u + turnCmd is truncated
// and u - turnCmd is not, the sum stops being 2u, and the turn differential leaks into
// COMMON MODE -- i.e. becomes a direct pitch disturbance, arriving exactly when the loop
// has no authority left to reject it. Wheel RPM saturation does the same thing: past the
// motor's speed limit extra PWM buys no extra torque, so the outer wheel silently stops
// following its command. Reported symptom: spinning near max revs leaves no wheel speed
// to plant a lean, so a reverse command cannot tip the body back at all.
// Budget arithmetic that says this is real, not theoretical: DRIVE_MAX_VEL 0.65 plus the
// ~0.5 rev/s of rotation seen in VROT traces asks the outer wheel for 1.15 rev/s, and the
// highest wheel speed in ANY log is ~1.0 (the brake surge). Over-subscribed on paper.
// Fix is mixer priority, the same rule flight controllers use for attitude vs yaw:
//   (a) PWM budget  -- turn is clamped to whatever MAX_PWM headroom u has not taken, so
//       the one-sided clip that breaks the cancellation can never happen;
//   (b) speed budget -- turn fades as forward speed rises, so rotation cannot spend the
//       RPM a lean needs. At rest the fade is 1.0, so spin-in-place is untouched.
// Both only ever REDUCE |turnCmd| toward zero. Turn is never reversed, never boosted, and
// yaw stays a stable decoupled axis -- so this cannot become the friction-FF failure.
#define TURN_HEADROOM_LIMIT 1
const float TURN_SPEED_FADE = 0.50f; // fraction of turn authority given up at full speed
// 25 -> 16, 2026-08-15: pilot reports turns are still faster than the drive. Measured on that
// flight, full turn stick produced TEFF up to +/-22 and VROT up to 0.61 rev/s while VF during
// normal driving ran 0.4..0.5 -- so yaw rate was roughly 1.5x the fore-aft rate. 16 scales the
// differential by 0.64 and brings the two into the same neighbourhood.
// Note this is now the SECOND lever pulled in the same direction: DRIVE_MAX_VEL went 0.65 ->
// 1.00 at the same time, so the drive/turn ratio moves by both. If turning ends up too slow,
// undo this one before touching the speed.
const float TURN_AUTHORITY = 16.0f; // PWM-effort differential at full turn stick
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
// ---- Friction feedforward: the launch assist, made continuous ---------------
// 2026-08-14. The bench log finally showed the ONLY mechanism that has ever moved
// this robot, and it is this assist -- not the cascade. MS 33390-33590:
//     MS      DL  DR    VF    START  BTK
//     33290   -1   0   0.01     W     40
//     33390   +1  +3   0.06     W     40
//     33490   +1  +5   0.09     P     36   <- assist PUSH fires
//     33590   +4  +4   0.12     P     16
//     33690    0  +1   0.03     C     11   <- released early on driveRollingConfirmed
//     33790   -1  -2  -0.04     C     11
// It rolls while the pulse is on and dies within 200 ms of it ending. The assist
// exited at BTK 11 with ticks still on the clock, i.e. it saw 0.12 rev/s, declared
// breakaway, and handed back to a cascade whose u averages -0.78 -- far under the
// 11/13 kinetic floor needed to KEEP a wheel turning. So the push works and we
// deliberately switch it off. MAX_PWM is not the limit: peak PWM in that run was 37
// of 110, and the saturation latch needs |u| > 78.
//
// This replaces the 200 ms timer with a floor that fades on DEMAND, not a clock:
//   * Aimed by a slow-filtered u.
//
//     FIRST ATTEMPT AIMED IT AT velError AND THE SIGN WAS WRONG. Bench 2026-08-14:
//     the oscillation vanished (u smooth, |rate| <= 9, nothing at 9 Hz -- the floor
//     really was what drove the limit cycle) but the robot could not move at all,
//     and the burst says why. At i=399, u = +20.4 with FFB = -0.97:
//         left :  15 * (-0.97) + 20.4 = +5.9   -> pwmL  +6
//         right:  27 * (-0.97) + 20.4 = -5.8   -> pwmR  -4
//     The bias and the PD cancelled, and because the floors differ by 12 the two
//     wheels came out with OPPOSITE SIGNS -- a pure yaw command with no drive.
//     The sign error: at i=18..40 the bias pushed the wheels forward and pitch went
//     -3.229 -> -4.148, i.e. the body rotated BACKWARD. That is correct
//     non-minimum-phase physics, and it is also why the PD was commanding the wheels
//     BACKWARD (u positive): to establish a FORWARD lean the wheels must first roll
//     back. LEAN ACT never left -2.4 while LEAN CMD reached 6.5 -- the body never
//     tilted forward, because the motion that would tilt it was the motion the bias
//     was fighting. velError points at where we want to END UP; the stiction that
//     needs breaking at launch resists the opposite motion. u carries the right
//     intent in BOTH phases, so aim at u.
//     (The backward lean was introduced by that bias, not pre-existing: every log
//     before it shows the body leaning the commanded way, to +6.8 and -12.8 deg.)
//
//     Low-passing u keeps the property that mattered -- the ~9 Hz swing gets no
//     floor, only the sustained demand does. Magnitude check: u settles near +20,
//     and 20 + the floors gives 35/47, which is exactly what LAUNCH_ASSIST_EFFORT
//     20 produced. Fixing the sign also fixes the opposite-sign wheels, since the
//     bias and u now ADD instead of cancelling.
//   * Magnitude capped at the per-wheel deadband (11/13 moving, 15/27 static). That
//     is BY DEFINITION the amount that produces no motion, so while the wheel is
//     stuck friction absorbs it and net body torque is ~0. This is what makes it a
//     friction compensator and not the retired DRIVE_VELOCITY_EFFORT torque command
//     that the balance loop had to fight.
//   * It also takes the floor OFF the oscillating path: a +/-9 u now yields +/-9 PWM,
//     under breakaway, so no buzz and no energy injected at the zero crossings.
//   * Drive only. With no stick the old sign-of-u floor is untouched, so standing
//     balance is exactly as before.
// LAUNCH ASSIST DISABLED so this is the sole breakaway mechanism and the next log is
// unambiguous. Running both would confound it the way DRIVE_VELOCITY_EFFORT did.
// OFF 2026-08-14. SETTLED: the two-phase feedforward is not salvageable. LAUNCH and
// CRUISE push in OPPOSITE directions, so any gate between them turns a misclassification
// into a full-amplitude PWM reversal -- and every gate tried chattered on a ringing
// LEAN ACT: 3.0/1.5 hysteresis, a one-way latch, a -2.0 loss threshold, and a rate lead.
// Final run: FFP alternating C,L,C,L,C,C,L,C,L,... every 100 ms sample with FFB flipping
// +0.94, -1.00, +0.92, -1.00, +0.49, -1.00; burst shows +/-3.7 deg of pitch at ~5.1 Hz
// (peaks 39-40 samples apart) and RATE reaching +/-118 deg/s. No threshold fixes this:
// the ring amplitude exceeds any band narrow enough to be useful.
// Slew-limiting the floor (97d6e55) made it WORSE, because it held the floor high and
// steady while the SIGN kept inverting underneath it.
// With DRIVE_FRICTION_FF 0 the ffBias path in mapEffortToPwm is inert and the plain
// sign-of-u floor takes over -- the per-wheel deadband only ever augments |effort| and
// can never oppose it, which is the property the FF path never had.
// The lean no longer needs a launch push anyway: at Kpos 6 / KVEL_I 24 / LEAN_CLAMP 18
// it reaches 17-18 deg on its own, which was not true when the FF was designed.
// Anything rebuilt here must be SINGLE-SIGNED. Do not reintroduce opposing phases.
#define DRIVE_FRICTION_FF 0
const float FRICTION_FF_LPF    = 0.98f; // ~0.64 Hz corner at 200 Hz: passes the demand,
                                        // rejects the ring (14x down at 9 Hz).
const float FRICTION_FF_KNEE_U = 3.0f;  // effort units for ~76% of the floor. u reached
                                        // +20 while stalled, so this saturates early and
                                        // the floor is effectively full during breakaway.
// TWO PHASES, gated on whether the lean is actually up. Bench 2026-08-14 with the
// u-aimed bias alone: LEAN ACT reached 6.4 -> 9.9 deg tracking LEAN CMD 6.1 -> 8.5
// with |ERR| <= 1.5 -- the FIRST time the body ever went where it was told (every
// earlier run had LEAN ACT pinned at -2.4 while the command climbed to 6.5). But POS
// stayed at 0.21 for the whole 1.3 s window: lean up, wheels still stopped.
// The burst says why. u crosses zero at i = 46, 148, 241 -> ~195 samples = ~1 Hz, and
// over the whole capture u has essentially NO DC, it just swings +/-7. FRICTION_FF_LPF
// sits at 0.64 Hz, right on that, so FFB chased the swing (-0.62..+0.65) instead of
// extracting a bias, and pwmR peaked at 22 against a 27 static breakaway. A low-pass
// cannot extract a DC component that is not there.
// The two aims were each right for one phase and I kept applying one to both:
//   LAUNCH  (lean not yet up): wheels must roll BACKWARD to tip the body forward.
//                              u points that way. This is the part that now works.
//   CRUISE  (lean up):         wheels must roll FORWARD to convert lean into motion.
//                              velError points that way and fades as speed arrives.
// The velError-only version never got the lean up; the u-only version gets it up and
// then has nothing left to push with, which is exactly "almost moved". Once the lean
// is established u has no job, so it oscillates and a u-aimed bias oscillates with it
// -- that is the 1 Hz hunt.
// Checked against that log: at burst i=0 the lean toward the command is 0.68 deg
// (LAUNCH); by MS 31199 it is 6.45 deg (CRUISE), so the forward push would engage
// exactly where the robot sat and did nothing.
const float FRICTION_FF_KNEE_V    = 0.12f; // rev/s of velError for ~76% of the floor in CRUISE
// Breakaway overdrive, re-added NARROWLY after 268a8f7's version had to be reverted.
// That one applied in BOTH phases, and since LAUNCH and CRUISE push OPPOSITE ways it
// amplified them fighting each other. This one is gated three ways so it cannot
// misclassify: CRUISE only (direction comes from velError, which is unambiguous),
// wheels actually STOPPED, and common-mode so it adds no yaw. It can only ever add
// magnitude to a push whose sign is already correct.
// 27 + 20 = 47 is the same number LAUNCH_ASSIST_EFFORT produced, the one mechanism
// that reliably moved this robot. It disappears the moment either wheel turns.
const float FRICTION_FF_BOOST     = 20.0f; // extra PWM on the floor while stalled in CRUISE
// 0.15 -> 0.40, AND the result is now slew-limited. 0.15 did not work and the reason is
// worth writing down, because it is the third instance of the same mistake this session:
// ONE ENCODER COUNT IS 0.366 rev/s (546 cpr at 200 Hz, x1 decode). VRL/VRR step in +/-0.37
// quanta all through the logs. So a 0.15 rev/s blend window is 2.4x SMALLER than the
// quantum -- ffRoll still slammed 0->1 on a single count and ffFloor still stepped 13<->47.
// Bench 2026-08-14 burst, u essentially unchanged across the tick:
//     i=72  u 1.834   PWM 10        i=145 u 16.139  PWM  1
//     i=73  u 1.801   PWM 47        i=146 u 16.555  PWM 29
// Filtering the INPUT cannot fix this when the input's quantum exceeds the window. The
// filter has to be on the OUTPUT, which is immune to whatever the velocity signal does.
const float FRICTION_FF_ROLL_VEL  = 0.40f; // rev/s for a full blend; = DRIVE_MAX_VEL, and
                                           // comfortably more than one 0.366 rev/s count
const float FF_FLOOR_LPF          = 0.99f; // ~0.32 Hz at 200 Hz (tau 0.5 s). A 34-count
                                           // floor step becomes a 0.5 s ramp, which cannot
                                           // drive a limit cycle no matter how ffRoll jumps.
const float FRICTION_FF_LEAN_DEG  = 3.0f;  // deg toward the command: LAUNCH -> CRUISE
const float FRICTION_FF_LEAN_DROP = 1.5f;  // hysteresis back to LAUNCH; wide enough that the
                                           // +/-1.5 deg tracking ripple cannot chatter the phase

#define DRIVE_LAUNCH_ASSIST 0
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
// FINDING 2026-08-14: this is ON despite the comment above calling it A/B-only,
// and it is a CONFOUND in every log analysed so far -- u is not a clean PD output
// while it runs. Worse, it is a relay gated on a FAST variable: engagement needs
// |dRateFilt| <= DRIVE_VELOCITY_HOLD_RATE_DPS (18), and dRateFilt swings +/-38 at
// the ring frequency, so the term switches on and off in step with the
// oscillation. Bench sample, four consecutive telemetry lines:
//     DFILT  -5.11 -> VTG Y (VTRQ -2.11)
//     DFILT +36.44 -> VTG -
//     DFILT  -7.58 -> VTG -
//     DFILT -17.73 -> VTG Y (VTRQ -1.99)
// Same class of error as scheduling Kd on |error|: a gain that switches with the
// signal it acts on is a nonlinear feedback path, not a schedule.
// DISABLED 2026-08-14 so u is a clean PD output and the burst capture is
// interpretable. Turn it back on only for a deliberate A/B, never for flight.
#define DRIVE_VELOCITY_EFFORT 0
const float DRIVE_VELOCITY_EFFORT_GAIN  = 5.5f; // rev/s error -> PWM effort
const float DRIVE_VELOCITY_EFFORT_LIMIT = 4.0f; // small trim, well below launch floor
const float DRIVE_VELOCITY_HOLD_LEAN_DEG = 0.75f; // hysteresis after the strict 1.5 deg engagement
const float DRIVE_VELOCITY_HOLD_ERROR_DEG = 0.85f; // do not keep trim through a balance recovery swing
const float DRIVE_VELOCITY_HOLD_RATE_DPS  = 18.0f; // let the inner loop catch pitch snaps by itself

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
// 30 -> 45 (2026-08-14). The logged "laying" pose is LEAN ACT 31.60 deg -- 1.6 deg past
// the old cutoff -- so the latch tripped and the robot was forbidden from even trying.
// At 31.6 deg of error the PD asks for Kp*31.6 = 63 PWM against a 110 ceiling, which is
// authority it already has; the cutoff was the only thing in the way. Beyond ~45 deg the
// accel pitch gets ambiguous and it is genuinely on its back, so that is the new line.
const float FALL_CUTOFF_DEG = 45.0f; // trip the fall latch: |pitch| beyond this = it fell, stop fighting
// A stand-up attempt must be BOUNDED or it cooks the motors. Two backstops now cover the
// whole range: above ~39 deg of error the PD exceeds the saturation latch's threshold
// (SAT_EFFORT_FRAC*(MAX_PWM-MAX_DEADBAND) = 87.4 since the 2026-08-15 re-measure took
// MAX_DEADBAND 24 -> 18; it was 81.7 before that and the 78.85 written here previously
// was staler still, from the 27 era -- i.e. 43.7 deg at Kp 2.0) and it latches
// in 0.5 s. BELOW that, between 25 and 39 deg, u sits under the sat threshold and the old
// code would have pushed at ~60 PWM indefinitely against an obstacle -- which is exactly
// the band this 31.6 deg pose lives in. Hence the explicit timeout.
// 25 deg is chosen to clear normal driving: LEAN_CLAMP caps the COMMANDED lean at 14 and
// the worst observed ACTUAL overshoot was 18.5 deg, so this cannot false-trip on a drive.
// 25 -> 32. Had to move with LEAN_CLAMP: a commanded 18 deg overshoots in practice
// (~1.4x observed), so 25 would false-trip the give-up timeout during a normal
// aggressive drive and latch the motors off mid-run. Still well under FALL_CUTOFF_DEG.
const float         RECOVERY_GIVEUP_DEG   = 32.0f; // deg of error that counts as "still down"
const unsigned long RECOVERY_GIVEUP_TICKS = 400;   // 2 s at 200 Hz, then give up and latch
// 8 -> 12 (2026-08-14): "it disables and doesn't read commands at all until I physically
// push it". This is the angle that made it stay dead. Once `fallen` latches, NOTHING
// re-arms until the accel angle is back inside this window, and 8 deg is tight enough
// that the robot has to be held almost perfectly upright to clear it. 12 still requires
// a deliberate stand-up -- the rear sit-down is 15.67 and the forward stop 30.53, so a
// robot that is actually down reads far outside 12 and cannot self-clear by lying there.
const float FALL_REARM_DEG  = 12.0f; // re-arm ONLY when the absolute accel angle is back within this of
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
float targetRateFilt = 0.0f;         // d(leanCmd)/dt, filtered like dRateFilt, for D-on-ERROR
const float LEAN_RATE_FF_MAX = 60.0f; // deg/s : bound on the setpoint-rate term (a full LEAN_CLAMP
                                      // swing in 100 ms). Stops a leanCmd STEP becoming a PWM kick.
float dTermLog = 0.0f;    // derivative contribution, for telemetry
float leanCmd   = 0.0f;   // cascade outer-loop output: the desired lean (deg) added to BALANCE_SETPOINT
float velocityLeanI = 0.0f;
float frictionUSlow = 0.0f;   // low-passed u; aims the friction floor (DRIVE_FRICTION_FF)
bool  frictionLeanUp = false; // latched LAUNCH -> CRUISE phase for the friction floor
float ffFloorFilt = 47.0f;    // slew-limited FF floor; starts at the stopped value
float plainFloorFilt = 18.0f; // same, for the no-FF path; starts at the STATIC figure so
                              // the first breakaway from rest is not under-compensated
bool  integralRollingPrev = false;            // edge-detect stall -> rolling for the integral reset
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
float targetVelSlewed = 0.0f;                 // rate-limited copy of it; see TARGET_VEL_SLEW
float turnCmdLog = 0.0f;                      // post-deadzone differential effort
int   motorPwmL = 0, motorPwmR = 0;           // signed PWM actually sent to each IBT-2

#if BURST_LOG
// Full-rate capture buffer. Written from the control tick only; printed only
// while disarmed, so nothing here can stretch a 5 ms tick.
struct BurstSample {
  float   pitch, target, rate, u;
  int16_t pwmL, pwmR;
};
BurstSample burstBuf[BURST_N];
int  burstFill   = 0;        // samples captured so far
bool burstArmed  = false;    // capturing now
bool burstReady  = false;    // buffer full, waiting to be dumped
int  burstCursor = 0;        // dump position

// Print a bounded chunk per call so the dump never blocks for long. Only ever
// called from the disarmed path, where the motors are already cut.
void burstDumpChunk() {
  if (burstCursor == 0) {
    Serial.print("BURST N="); Serial.print(BURST_N);
    Serial.println(" HZ=200 COLS i,pitch,target,rate,u,pwmL,pwmR");
  }
  int end = burstCursor + 20;
  if (end > BURST_N) end = BURST_N;
  for (; burstCursor < end; burstCursor++) {
    const BurstSample &s = burstBuf[burstCursor];
    Serial.print("B ");  Serial.print(burstCursor);
    Serial.print(' ');   Serial.print(s.pitch, 3);
    Serial.print(' ');   Serial.print(s.target, 3);
    Serial.print(' ');   Serial.print(s.rate, 2);
    Serial.print(' ');   Serial.print(s.u, 3);
    Serial.print(' ');   Serial.print(s.pwmL);
    Serial.print(' ');   Serial.println(s.pwmR);
  }
  if (burstCursor >= BURST_N) {
    Serial.println("BURST END");
    burstReady = false;
    burstFill  = 0;
    burstCursor = 0;
  }
}
#endif
long  homeTicksSum = 0;                        // (leftTicks+rightTicks) defining "home" (0 = boot spot)
bool  stalled = false;                         // true while the stall cutoff has the motors off
bool  fallen = false;                          // true while the fall latch has the motors off
unsigned long recoverTicks = 0;                // ticks spent above RECOVERY_GIVEUP_DEG
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
  float frictionFF = 0.0f;   // -1..+1 : how much of the deadband floor drive is aiming
  bool  frictionLeanUp = false;  // false = LAUNCH phase, true = CRUISE phase
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
  Serial.print("DEADBAND_TEST mode: OFF THE GROUND measures MOVING, ON THE FLOOR "
               "measures STATIC. ");
  Serial.print(DEADBAND_TEST_PASSES);
  Serial.println(" passes, direction alternating, summary at the end. "
                 "Do not touch the wheels between passes.");
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
#if LEFT_IDLE_BRAKE
    // Both inputs HIGH -> both outputs at VCC -> left motor shorted -> brake, matching
    // the state the right bridge sits in on its own. See LEFT_IDLE_BRAKE.
    analogWrite(LEFT_MOTOR_FORWARD_PIN, MOTOR_PWM_FULL);
    analogWrite(LEFT_MOTOR_REVERSE_PIN, MOTOR_PWM_FULL);
#else
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
#endif
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
#if LEFT_IDLE_BRAKE
    if (forwardPin == LEFT_MOTOR_FORWARD_PIN) {   // see LEFT_IDLE_BRAKE: both HIGH = brake
      analogWrite(forwardPin, MOTOR_PWM_FULL);
      analogWrite(reversePin, MOTOR_PWM_FULL);
      lastSign = 0;                               // leaving brake needs no reversal blanking
      return;
    }
#endif
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
// ffBias (-1..+1, see DRIVE_FRICTION_FF) aims the floor at the outer loop's SUSTAINED
// demand instead of at the sign of this tick's effort. At 0 this is the original map
// exactly, so standing balance is unchanged.
// plainFloor is the CONTINUOUS blend of the static and moving figures (see
// driveControlDiff). It replaces the binary `moving ? dbMoving : dbStatic` this function
// used to do for itself -- that gate chattered at the loop rate and square-waved the
// floor. 2026-08-15 flight, consecutive telemetry samples with the wheels barely turning:
//     DBL 10M DBR 10M  ->  DBL 18S DBR 18S  ->  DBL 10M DBR 18S  ->  DBL 18S DBR 10M
// an 8-count step in the floor every tick, on both wheels, independently. The continuous
// ffRoll blend that fixes exactly this was already built -- but only the ffBias branch
// below ever used it, and with DRIVE_FRICTION_FF 0 that branch is inert, so the chattering
// gate was the live path the whole time. Same bug as the one fixed at FF_FLOOR_LPF, in the
// other half of the function.
int mapEffortToPwm(float effort, float ffBias, float ffFloor, float plainFloor) {
  if (ffBias != 0.0f) {
    // One-sided friction compensation plus the raw PD effort. An effort that averages
    // to zero now averages to the bias, not to a relay twice its own amplitude.
    float cmd = ffFloor * ffBias + effort;
    if (fabs(cmd) <= OUT_DEADZONE) return 0;
    return (int)constrain(cmd, -(float)MAX_PWM, (float)MAX_PWM);
  }
  if (fabs(effort) <= OUT_DEADZONE) return 0;
  // RAMP THE FLOOR IN over the first FLOOR_KNEE of |effort| instead of stepping to it.
  // With the feedforward off this is the ONLY nonlinearity left in the loop, and as a
  // STEP it has infinite incremental gain exactly at the zero crossing -- a textbook
  // limit-cycle generator, and the burst shows the cycle: u peaks at i = 21, 46, 70, 94,
  // 117, 142, 168, 193, 219, 245, 271, 296 (intervals 23-26 samples = ~125 ms = 8 Hz),
  // with PWM stepping 0 -> -16/-28 and 0 -> +15/+27 on every crossing.
  // Ramped, the incremental gain near zero drops from infinite to deadband/FLOOR_KNEE + 1
  // (about 6.4 for the 27 floor), which is what takes the energy out of the cycle.
  // Full friction compensation is still there wherever it matters: |effort| >= FLOOR_KNEE
  // gets the whole floor, and drive efforts run 9-30.
  float ramp = fabs(effort) / FLOOR_KNEE;
  if (ramp > 1.0f) ramp = 1.0f;
  int pwm = (int)constrain(plainFloor * ramp + fabs(effort), 0.0f, (float)MAX_PWM);
  return effort > 0.0f ? pwm : -pwm;
}

// ffRoll (0..1) replaces BOTH discrete gates that used to set the FF floor: the
// static/moving choice and the boost enable. Both keyed off wheelMoving, which toggles
// at encoder-noise level -- DBR alternates 27S/13M essentially every telemetry sample --
// so the floor was being square-waved. Bench 2026-08-14, consecutive ticks:
//     MS 17360  u = -10.13  FFB 0.98  DBL 15S DBR 27S -> floor 27+20=47 -> PWM +36
//     MS 17460  u = -37.06  FFB 0.96  DBL 15S DBR 13M -> floor      15 -> PWM -22
// A 32-count step in the floor, compounded by u, giving a full-amplitude PWM reversal
// every tick and a ~2.5 Hz limit cycle in the burst. Blending on measured speed instead
// is continuous: no gate, nothing to chatter, and the boost still lands where it is
// needed (stopped) and fades where it is not (rolling).
void driveControlDiff(float uL, float uR, float ffBias, float ffRoll) {
  // ONE FLOOR FOR BOTH WHEELS in the FF path (re-applied from 4013030; it was a real
  // fix that got thrown out with that revert). The bias is a COMMON-MODE drive command,
  // so per-wheel floors inject a differential into a command that should have none --
  // and because the FF can OPPOSE u, that differential strands a wheel at zero. Bench
  // 2026-08-14, u = -26.84, FFB 1.00, DBL 15S / DBR 27S:
  //     left :  15 * 1.00 - 26.84 = -11.8  -> PWML -11
  //     right:  27 * 1.00 - 26.84 =  +0.2  -> PWMR   0
  // Self-reinforcing too: the stopped wheel keeps the larger STATIC floor, cancels more
  // of u, and stays stopped. Max means neither side is under-driven.
  const float fStatic = (float)(LEFT_DEADBAND_STATIC > RIGHT_DEADBAND_STATIC
                                ? LEFT_DEADBAND_STATIC : RIGHT_DEADBAND_STATIC);
  const float fMoving = (float)(LEFT_DEADBAND_MOVING > RIGHT_DEADBAND_MOVING
                                ? LEFT_DEADBAND_MOVING : RIGHT_DEADBAND_MOVING);
  // Stopped -> static floor + breakaway boost (27+20 = 47). Rolling -> kinetic floor (13).
  // SLEW-LIMITED: the blend input is quantized far coarser than the blend window (see
  // FRICTION_FF_ROLL_VEL), so the target still steps. Filtering here bounds how fast the
  // floor can move regardless, which is what actually stops the ringing.
  float ffFloorTarget = (fStatic + FRICTION_FF_BOOST) * (1.0f - ffRoll) + fMoving * ffRoll;
  ffFloorFilt = FF_FLOOR_LPF * ffFloorFilt + (1.0f - FF_FLOOR_LPF) * ffFloorTarget;
  float ffFloor = ffFloorFilt;
  // THE PLAIN PATH GETS A CONTINUOUS FLOOR TOO (2026-08-15). The old claim here was that
  // wheelMoving could keep governing this path because "the floor only augments |effort|
  // and a step in it cannot reverse the command". True in isolation, and still beside the
  // point: the floor is multiplied by the ramp and added to |effort|, so an 8-count step
  // in it is an 8-count step in PWM at constant effort, and the gate chatters every tick.
  // Combined with the sign flip of a limit cycle that is a +/-21 square wave into a wheel
  // trying to break away -- see mapEffortToPwm for the log. Blend on measured speed like
  // the FF path does: same ffRoll, same LPF, no gate, nothing to chatter.
  // ONE FLOOR FOR BOTH WHEELS is now exactly right rather than a compromise: the deadbands
  // were re-measured symmetric (10/10 moving, 18/18 static), so a common floor injects no
  // differential at all.
  float plainTarget = fStatic * (1.0f - ffRoll) + fMoving * ffRoll;
  plainFloorFilt = FF_FLOOR_LPF * plainFloorFilt + (1.0f - FF_FLOOR_LPF) * plainTarget;
  int pwmL = mapEffortToPwm(uL, ffBias, ffFloor, plainFloorFilt);
  int pwmR = mapEffortToPwm(uR, ffBias, ffFloor, plainFloorFilt);
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

// A wheel has broken free when it ADVANCES by DB_STEP_COUNTS within one 200 ms step AND
// advances again on the following step. CUMULATIVE displacement is not enough: under
// load a wheel takes up backlash, jumps a few counts and STALLS. Pass 3 of the
// 2026-08-15 loaded run had dL reach 6 at PWM 11 then sit at exactly 6 through PWM 15
// before really breaking at 16 -- the old bare "cumulative > 5" test recorded 11, five
// counts early, and it was the single widest sample in that run. Requiring a second
// advance rejects the twitch, and recording the FIRST of the two steps keeps a clean
// break from being reported late.
const int DB_STEP_COUNTS = 5;

void deadbandDetect(int pwm, long d, long prevD, int &cand, long &candD, int &moved) {
  if (moved) return;
  if (cand) {
    if (d >= candD + DB_STEP_COUNTS) { moved = cand; return; }  // advanced again: confirmed
    cand = 0;                                                   // stalled: it was a twitch
  }
  if (d >= prevD + DB_STEP_COUNTS) { cand = pwm; candD = d; }   // provisional
}

// Per-wheel min/max across the passes that ran in one direction. Ignores 0 entries:
// 0 means that wheel never broke free before MAX_PWM, which is not a threshold and must
// not be averaged in as if it were one. Returns the number of valid samples.
int deadbandSpread(const int *res, const int *sgn, int n, int want, int &lo, int &hi) {
  int seen = 0;
  for (int i = 0; i < n; i++) {
    if (sgn[i] != want || res[i] == 0) continue;
    if (!seen || res[i] < lo) lo = res[i];
    if (!seen || res[i] > hi) hi = res[i];
    seen++;
  }
  return seen;
}

// One line per wheel per direction: the range, not a single number. The range IS the
// result -- if L comes back 11..21 while R comes back 8..9, the left wheel has no
// deadband to tune and the next question is why it is bimodal, not what to set it to.
void deadbandPrintDirection(const int *resL, const int *resR, const int *sgn,
                            int n, int want) {
  int lo = 0, hi = 0;
  Serial.print(want > 0 ? "  back(+1)  L " : "  fwd (-1)  L ");
  int seen = deadbandSpread(resL, sgn, n, want, lo, hi);
  if (!seen) Serial.print("none");
  else { Serial.print(lo); Serial.print(".."); Serial.print(hi);
         Serial.print(" spread "); Serial.print(hi - lo); }
  Serial.print("   R ");
  lo = 0; hi = 0;
  seen = deadbandSpread(resR, sgn, n, want, lo, hi);
  if (!seen) Serial.println("none");
  else { Serial.print(lo); Serial.print(".."); Serial.print(hi);
         Serial.print(" spread "); Serial.println(hi - lo); }
}

void deadbandTestSummary(const int *resL, const int *resR, const int *sgn, int n) {
  Serial.println();
  Serial.println("==== DB_TEST SUMMARY ====");
  Serial.println("pass  dir        L     R");
  for (int i = 0; i < n; i++) {
    Serial.print("  ");   Serial.print(i + 1);
    Serial.print("   ");  Serial.print(sgn[i] > 0 ? "back(+1)  " : "fwd (-1)  ");
    if (resL[i]) { Serial.print(resL[i]); } else { Serial.print("--"); }
    Serial.print("    ");
    if (resR[i]) { Serial.println(resR[i]); } else { Serial.println("--"); }
  }
  Serial.println("-- means the wheel never moved below MAX_PWM.");
  deadbandPrintDirection(resL, resR, sgn, n, +1);
  deadbandPrintDirection(resL, resR, sgn, n, -1);
  // Read it in this order. Spread first, direction second: a per-direction deadband is
  // only meaningful if each direction is tight enough to have a value at all.
  Serial.println("READ: spread within a direction FIRST. If a wheel's own spread is as");
  Serial.println("big as the fwd/rev gap, direction is not the variable and per-direction");
  Serial.println("deadbands would be fitting a constant to noise. Only if both directions");
  Serial.println("are tight (<=1) and differ does mapEffortToPwm need a per-sign floor.");
  Serial.println("=========================");
}

// Deadband measurement: slowly ramp PWM, report the PWM at which each wheel first turns
// (its stiction threshold). Run with the wheels off the ground.
//
// Runs DEADBAND_TEST_PASSES ramps back to back, alternating direction, then prints the
// summary and stops. No reset or reflash between samples -- see DEADBAND_TEST_PASSES for
// why repeats and interleaving are the point.
void deadbandTestLoop() {
  static bool seeded = false;
  static long baseL = 0, baseR = 0;
  static int  pwm = 0, movedL = 0, movedR = 0;     // 0 = wheel hasn't moved yet
  static int  candL = 0, candR = 0;                // provisional breakaway, not yet confirmed
  static long candDL = 0, candDR = 0, prevDL = 0, prevDR = 0;
  static int  pass = 0;
  static int  sign = DEADBAND_TEST_SIGN;
  static bool coasting = false, finished = false;
  static unsigned long lastStep = 0;
  static int  resL[DEADBAND_TEST_PASSES], resR[DEADBAND_TEST_PASSES];
  static int  resSign[DEADBAND_TEST_PASSES];

  if (finished) { motorRaw(0); return; }

  if (!seeded) {
    readEncoders(baseL, baseR);
    seeded   = true;
    lastStep = millis();
    Serial.print("DB_TEST pass ");  Serial.print(pass + 1);
    Serial.print("/");              Serial.print(DEADBAND_TEST_PASSES);
    Serial.println(sign > 0 ? "  sign +1 (backward)" : "  sign -1 (forward)");
  }

  // Between passes: motors off, and WAIT for the rotor to coast to a stop before
  // re-seeding the baseline. Re-seeding while it still turns carries the leftover
  // rotation into the next pass and trips the >5-count move test at pwm 0, which would
  // report a deadband of 0 and look like a spectacular result.
  if (coasting) {
    if (millis() - lastStep < (unsigned long)DEADBAND_TEST_DWELL_MS) return;
    coasting = false;
    seeded   = false;
    pwm = 0; movedL = 0; movedR = 0;
    candL = 0; candR = 0; candDL = 0; candDR = 0; prevDL = 0; prevDR = 0;
    return;
  }

  if (millis() - lastStep < 200) return;           // step every 200 ms
  lastStep = millis();

  long l, r; readEncoders(l, r);
  long dl = labs(l - baseL), dr = labs(r - baseR);
  deadbandDetect(pwm, dl, prevDL, candL, candDL, movedL);   // see deadbandDetect: a
  deadbandDetect(pwm, dr, prevDR, candR, candDR, movedR);   // twitch is not a breakaway
  prevDL = dl; prevDR = dr;

  Serial.print("DB_TEST p");           Serial.print(pass + 1);
  Serial.print(sign > 0 ? "+" : "-");
  Serial.print(" PWM ");               Serial.print(pwm);
  Serial.print(" dL ");                Serial.print(l - baseL);
  Serial.print(" dR ");                Serial.print(r - baseR);
  Serial.print("  | first-move L@");   Serial.print(movedL);
  Serial.print(" R@");                 Serial.println(movedR);

  if ((movedL && movedR) || pwm >= MAX_PWM) {      // pass done
    motorRaw(0);
    resL[pass] = movedL; resR[pass] = movedR; resSign[pass] = sign;
    pass++;
    if (pass >= DEADBAND_TEST_PASSES) {
      deadbandTestSummary(resL, resR, resSign, DEADBAND_TEST_PASSES);
      finished = true;
      return;
    }
    sign     = -sign;                              // interleave the directions
    coasting = true;
    lastStep = millis();
    return;
  }
  pwm++;
  // sign +1 ramps positive PWM = physical BACKWARD (see motorRaw). Every deadband figure
  // in this file was taken that way. mapEffortToPwm applies ONE constant per wheel to
  // BOTH signs, so if breakaway differs by direction -- backlash, gearbox preload -- one
  // direction is mis-compensated with no way to express it. Mirrored motors make that a
  // robot-level fwd/rev asymmetry, because each motor runs in its opposite local
  // direction between the two. The printed numbers stay magnitudes either way.
  motorRaw(sign * pwm);
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
    frictionUSlow = 0.0f;
    frictionLeanUp = false;
    ffFloorFilt = (float)RIGHT_DEADBAND_STATIC + FRICTION_FF_BOOST;  // re-arm stopped
    plainFloorFilt = (float)RIGHT_DEADBAND_STATIC;                   // ditto, no boost
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
    controlLog.driveCmd = controlLog.turnCmd = controlLog.frictionFF = 0.0f;
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
    targetVelSlewed = 0.0f;      // ...including the slew state, so the ramp starts at rest
    satTicks     = 0;
    armedPrev    = false;        // next armed tick re-triggers the soft start
    driveCommandPrev = false;
    driveVelocityEffortLatched = false;
    launchAssistState = LAUNCH_IDLE;
    launchAssistDirection = 0;
    launchAssistTicksRemaining = 0;
    launchRollingTicks = 0;
#if BURST_LOG
    // Motors are already cut above, so this is the safe place to read the buffer
    // out. Suppress the normal telemetry line while dumping so the two do not
    // interleave in the capture file.
    if (burstReady) { burstDumpChunk(); return; }
#endif
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
    targetRateFilt = 0.0f;
#if BURST_LOG && STATIC_LEAN_TEST
    // No stick in this mode, so there is no drive edge to arm on. Capture the
    // arming instead: SB up gives soft-start + 2 s of full-rate data, SB down
    // dumps it.
    if (!burstReady) { burstFill = 0; burstCursor = 0; burstArmed = true; }
#endif
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
  // 0.35 + 0.65*SPD -> 0.10 + 0.90*SPD (2026-08-14, "give 90 percent of the rpm to the
  // throttle scale"). CH3 now owns 90 points of the range instead of 65: full stick is
  // still 1.00 x Vmax, but the bottom drops from 0.35 to 0.10 so the knob actually
  // spans the speed range instead of living in its top two thirds. The 0.10 floor is
  // deliberate -- a true zero would make targetVel identically 0 at low throttle, which
  // reads as "drive stick does nothing" rather than "drive slowly", and would also stop
  // `driving` from ever latching. Worth knowing the range moved: the log that prompted
  // this had SPD 0.33 -> cap 0.56; the same stick position is now cap 0.40, so a given
  // throttle setting is SLOWER than before while full stick is unchanged.
  float cap       = 0.10f + 0.90f * speedIn;                          // CH3 speed cap 0.10..1.0
  float driveRaw  = crsf::drive();
  float turnRaw   = crsf::turn();
  float driveIn   = driveRaw;
  float turnIn    = turnRaw;
  if (fabs(driveIn) < DRIVE_STICK_DEADZONE) driveIn = 0.0f;
  if (fabs(turnIn) < TURN_STICK_DEADZONE) turnIn = 0.0f;
  // Map the stick's LIVE travel (deadzone..1) onto DRIVE_MIN_VEL..DRIVE_MAX_VEL instead of
  // scaling from zero, so the first non-zero command is already a speed the wheels can hold.
  // See DRIVE_MIN_VEL. driveIn is exactly 0 inside the deadzone, so a centred stick still
  // commands exactly 0 and the balancer's standing behaviour is untouched.
  float targetVelRaw = 0.0f;
  if (driveIn != 0.0f) {
    float live = (fabs(driveIn) - DRIVE_STICK_DEADZONE) / (1.0f - DRIVE_STICK_DEADZONE);
    live = constrain(live, 0.0f, 1.0f);
    float speed = DRIVE_MIN_VEL + live * (DRIVE_MAX_VEL - DRIVE_MIN_VEL);
    targetVelRaw = DRIVE_SIGN * (driveIn > 0.0f ? 1.0f : -1.0f) * cap * speed;  // rev/s, + = fwd
  }
#if TARGET_VEL_SLEW_LIMIT
  // See TARGET_VEL_SLEW. Away from zero is rate-limited so the demand stays plantable;
  // toward zero is instant so a release still stops asking immediately. A sign flip
  // drops the target to zero first, then ramps out the far side from there.
  if (targetVelRaw * targetVelSlewed < 0.0f) targetVelSlewed = 0.0f;
  if (fabs(targetVelRaw) <= fabs(targetVelSlewed)) {
    targetVelSlewed = targetVelRaw;                                   // toward zero: free
  } else {
    const float step = TARGET_VEL_SLEW * DT;
    targetVelSlewed += constrain(targetVelRaw - targetVelSlewed, -step, step);
  }
  float targetVel = targetVelSlewed;
#else
  targetVelSlewed = targetVelRaw;
  float targetVel = targetVelRaw;
#endif
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
#if BURST_LOG
    // Arm the full-rate capture on the drive rising edge, unless a previous
    // capture is still waiting to be read out.
    if (!burstReady) { burstFill = 0; burstCursor = 0; burstArmed = true; }
#endif
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
    if (driveCommandPrev) {
      posSetpoint = positionRev;
#if COAST_BRAKE
      // Falling edge: drop the cruise trim so the coast brake starts from zero
      // rather than unwinding a forward-signed integral through zero first. VELI
      // read +3.48 at release, which at 10.6 deg/s is ~0.33 s of wrong-signed
      // lean before braking could even begin.
      velocityLeanI = 0.0f;
#endif
    }
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
  // Bounded stand-up attempt: below the saturation latch's reach the PD would otherwise
  // push at ~60 PWM forever against whatever it is lying on. See RECOVERY_GIVEUP_DEG.
  if (!fallen && fabs(error) > RECOVERY_GIVEUP_DEG) {
    if (++recoverTicks > RECOVERY_GIVEUP_TICKS) {
      recoverTicks = 0;
      fallen = true;
      Serial.println("RECOVER: stand-up attempt timed out -> latched (stand it up to re-arm)");
    }
  } else {
    recoverTicks = 0;
  }
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
    recoverTicks = 0;
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
  // Remember both accumulator states so a saturated output can unwind exactly
  // this tick's contribution -- see the conditional integration below.
  float posSetpointPrev   = posSetpoint;
  float velocityLeanIPrev = velocityLeanI;

  posSetpoint += targetVel * DT;
  // Backstop only. This used to be the anti-windup mechanism, which capped the
  // loop's authority at Kpos*POS_ERROR_CLAMP permanently -- see POS_ERROR_CLAMP.
  posSetpoint = constrain(posSetpoint, positionRev - POS_ERROR_CLAMP,
                                       positionRev + POS_ERROR_CLAMP);

  float posError = posSetpoint - positionRev;   // rev   : + = must travel forward
  float velError = targetVel   - forwardVel;    // rev/s : + = must speed up forward
#if OUTER_LOOP
  float effectiveKvel = Kvel * (driving ? DRIVE_KVEL_MULTIPLIER : 1.0f);
  float positionLean = Kpos * posError;
  float velocityLean = effectiveKvel * velError;
  // Accumulate whenever the pilot is asking for speed -- INCLUDING while stalled,
  // which is exactly when the extra lean is wanted. Freezing it there (the first
  // attempt) was a catch-22: it required rolling to build, and building was what
  // started the roll. The windup problem is handled at the other end instead --
  // on the stall -> rolling transition whatever accumulated was fighting stiction,
  // not trimming a cruise, so it is discarded at breakaway and the integral
  // restarts from zero for the cruise it actually exists to trim.
  bool integralRolling = fabs(forwardVel) >= VEL_I_ROLL_MIN;
#if COAST_BRAKE
  // Still rolling counts as well as still commanded -- see COAST_BRAKE. The
  // breakaway edge below cannot misfire during a coast: integralRolling is
  // already true on stick release and stays true until the robot stops.
  bool integralActive = driving || integralRolling;
#else
  bool integralActive = driving;
#endif
  if (!integralActive) {
    velocityLeanI = 0.0f;
  } else {
    if (integralRolling && !integralRollingPrev) velocityLeanI = 0.0f;   // breakaway
    velocityLeanI += KVEL_I * velError * DT;
    velocityLeanI = constrain(velocityLeanI, -VEL_I_CLAMP, VEL_I_CLAMP);
  }
  integralRollingPrev = integralRolling;
  float velocityLeanIntegral = velocityLeanI;
#else
  float effectiveKvel = 0.0f;
  float positionLean = 0.0f;   // DIAGNOSTIC: outer loop disabled -> pure baseline inner loop
  float velocityLean = 0.0f;
  velocityLeanI = 0.0f;
  float velocityLeanIntegral = 0.0f;
#endif
  float leanRaw = positionLean + velocityLean + velocityLeanIntegral;
  // CONDITIONAL INTEGRATION (rc_balance structure). LEAN_CLAMP is the single
  // saturation point: while the requested lean is pinned there, freeze both
  // accumulators so neither can wind past what the loop is allowed to ask for.
  // Below saturation they run free, so the loop keeps searching upward for the
  // lean that actually breaks the wheels loose instead of stalling at a fixed
  // ceiling. On breakaway there is no lurch to unwind: forwardVel rises,
  // velError collapses, and positionRev catches posSetpoint, so all three terms
  // shrink on their own.
  // Asymmetric: the chassis has 30.5 deg of forward lean and 15.7 deg of rear before it
  // sits on its rear contact, so the saturation point differs by direction. + = forward.
  float leanLimit = leanRaw >= 0.0f ? LEAN_CLAMP_FWD : LEAN_CLAMP_REAR;
  if (fabs(leanRaw) >= leanLimit) {
    posSetpoint   = posSetpointPrev;
    velocityLeanI = velocityLeanIPrev;
  }
  leanRaw = constrain(leanRaw, -LEAN_CLAMP_REAR, LEAN_CLAMP_FWD);
  float leanCmdBefore = leanCmd;
  leanCmd = LEAN_LPF * leanCmd + (1.0f - LEAN_LPF) * leanRaw;
#if STATIC_LEAN_TEST
  // Override the whole outer loop with a fixed lean the inner loop must hold for
  // as long as the stick is held. Everything above still runs and still logs, so
  // the telemetry shows what the outer loop WOULD have asked for alongside what
  // the inner loop is actually being given.
  leanCmd = STATIC_LEAN_DEG;   // constant, no stick, no ramp: hold it and watch
  leanRaw = leanCmd;
#endif
  // --- Setpoint rate for D-on-ERROR ------------------------------------------
  // dRateFilt is d(pitch)/dt. The proper derivative for the inner loop is
  // d(error)/dt = d(pitch)/dt - d(target)/dt; without the second term the D term
  // opposes the body rotation the OUTER loop just asked for, damping the lean
  // while it is being established.
  // Two details that matter:
  //   * CLAMP before filtering. leanCmd can step (STATIC_LEAN_TEST arming, or the
  //     posSetpoint re-anchor on a drive edge). A 3 deg step in one 5 ms tick is
  //     600 deg/s, which would be a ~120 PWM one-tick kick. Soft-start happens to
  //     mask it at arming, but that is luck, not protection.
  //   * FILTER IT THE SAME WAY as dRateFilt. Subtracting an unfiltered rate from
  //     a D_LPF-filtered one introduces a phase mismatch and puts noise back into
  //     the term the filter exists to clean.
  float targetRateRaw = constrain((leanCmd - leanCmdBefore) / DT,
                                  -LEAN_RATE_FF_MAX, LEAN_RATE_FF_MAX);
  targetRateFilt = D_LPF * targetRateFilt + (1.0f - D_LPF) * targetRateRaw;

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
#if DRIVE_GAIN_SCHEDULE
  float driveAuthority = driving ? constrain(fabs(driveIn), 0.0f, 1.0f) : 0.0f;
#else
  // Schedule bisected out: both blends below collapse to the neutral Kp/Kd exactly,
  // whatever the live-tune knob has done to them. This is the ONLY consumer of
  // driveAuthority, so zeroing it here disables the schedule completely.
  const float driveAuthority = 0.0f;
#endif

  controlLog.dTermRaw = Kd * dRateFilt;
  float kdSafetyBlend = constrain(
      (fabs(error) - DRIVE_KD_RESTORE_START_ERROR) /
      (DRIVE_KD_RESTORE_FULL_ERROR - DRIVE_KD_RESTORE_START_ERROR),
      0.0f, 1.0f);
  float driveKdFloor = Kd < DRIVE_KD ? Kd : DRIVE_KD;
  float driveKdTarget = driveKdFloor + kdSafetyBlend * (Kd - driveKdFloor);
  controlLog.effectiveKd = Kd + driveAuthority * (driveKdTarget - Kd);
  // No magnitude clamp here: the drive Kd blend above already sets the damping,
  // and clamping the result turns the derivative into a sign-only relay (see the
  // note at DRIVE_KD). DLIM in telemetry now means "drive Kd blend is active",
  // not "D was truncated".
  dTermLog = controlLog.effectiveKd * (dRateFilt - targetRateFilt);
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
#if STATIC_LEAN_TEST
  // The static lean test uses no stick, so `commanding` would be false and the
  // coast band would cut the motors the instant the body reached the commanded
  // lean -- i.e. exactly when the measurement begins. Never coast during it.
  commanding = true;
#endif
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
                                       fabs(error) <= DRIVE_VELOCITY_HOLD_ERROR_DEG &&
                                       fabs(dRateFilt) <= DRIVE_VELOCITY_HOLD_RATE_DPS;
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

  // --- Friction feedforward (see DRIVE_FRICTION_FF) --------------------------
  // Aim the deadband floor at the outer loop's sustained demand. velError holds one
  // sign for seconds while u swings +/-9 about a mean of -0.78, so velError is the
  // only signal here that carries usable DC.
  float frictionFF = 0.0f;
#if DRIVE_FRICTION_FF
  if (driving) {
    // Phase gate: is the commanded lean actually ON THE BODY yet? Latched with
    // hysteresis so the tracking ripple cannot flip it tick to tick.
    float leanToward = driveDirection * (pitch - BALANCE_SETPOINT);
    if      (leanToward >= FRICTION_FF_LEAN_DEG)  frictionLeanUp = true;
    else if (leanToward <  FRICTION_FF_LEAN_DROP) frictionLeanUp = false;

    if (frictionLeanUp) {
      // CRUISE. The lean is established, so the wheels must roll FORWARD to turn it
      // into travel. velError is the sustained one-sided signal here (+0.23..+0.33 for
      // seconds in the bench log) and it fades on its own as the speed is reached.
      // SIGN_CFG: physical forward is NEGATIVE PWM effort, velError is POSITIVE when
      // the robot still owes forward speed -- hence the negation.
      frictionFF = -tanhf(velError / FRICTION_FF_KNEE_V);
      frictionUSlow = 0.0f;   // stale on re-entry to LAUNCH; rebuild it from scratch
    } else {
      // LAUNCH. The lean is not up yet, and it is established by rolling the wheels
      // BACKWARD, which is where u points. No negation -- u already carries the sign.
      frictionUSlow = FRICTION_FF_LPF * frictionUSlow + (1.0f - FRICTION_FF_LPF) * u;
      frictionFF = tanhf(frictionUSlow / FRICTION_FF_KNEE_U);
    }
  } else {
    frictionUSlow  = 0.0f;   // no carry-over into the next drive gesture
    frictionLeanUp = false;  // every gesture starts in LAUNCH
  }
#endif
  controlLog.frictionFF = frictionFF;
  controlLog.frictionLeanUp = frictionLeanUp;

  // Balance has priority over turn; turn spends only what is left. See TURN_AUTHORITY.
  float turnMix = turnCmd;
#if TURN_HEADROOM_LIMIT
  float pwmHeadroom = (float)MAX_PWM - fabs(u);            // (a) keep u + turn off the clip
  if (pwmHeadroom < 0.0f) pwmHeadroom = 0.0f;
  turnMix = constrain(turnMix, -pwmHeadroom, pwmHeadroom);
  float speedFrac = constrain(fabs(forwardVel) / DRIVE_MAX_VEL, 0.0f, 1.0f);
  turnMix *= (1.0f - TURN_SPEED_FADE * speedFrac);         // (b) leave RPM for the lean
#endif
  turnCmdLog = turnMix;   // TEFF reports what was APPLIED, not what the stick asked for
  controlLog.effortL = u + turnMix;
  controlLog.effortR = u - turnMix;
  // Continuous stopped->rolling blend for the FF floor. NOT gated on wheelMoving: that
  // flag toggles at encoder-noise level and square-waved the floor. See driveControlDiff.
  float ffRoll = constrain(fabs(forwardVel) / FRICTION_FF_ROLL_VEL, 0.0f, 1.0f);
  driveControlDiff(controlLog.effortL, controlLog.effortR, frictionFF, ffRoll);

#if BURST_LOG
  // Capture AFTER the motors are written so pwmL/pwmR are this tick's real output.
  if (burstArmed && burstFill < BURST_N) {
    BurstSample &s = burstBuf[burstFill++];
    s.pitch  = pitch;
    s.target = BALANCE_SETPOINT + leanCmd;
    s.rate   = dRateFilt;
    s.u      = u;
    s.pwmL   = (int16_t)motorPwmL;
    s.pwmR   = (int16_t)motorPwmR;
    if (burstFill >= BURST_N) { burstArmed = false; burstReady = true; }
  }
#endif

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
  Serial.print(" FFB "); Serial.print(controlLog.frictionFF, 2);
  Serial.print(" FFP "); Serial.print(controlLog.frictionLeanUp ? 'C' : 'L');
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
  // Was printing LAUNCH_ASSIST_EFFORT under a "Bstart" label -- stale since
  // DRIVE_LAUNCH_ASSIST went to 0, and misleading (it is not a start ANGLE).
  Serial.print(" FALLDEG "); Serial.print(FALL_CUTOFF_DEG, 1);
  Serial.print(" GBIAS "); Serial.print(gyroYBias, 2);
  // DBF is the floor ACTUALLY in force this tick: one continuous blended value for both
  // wheels now, so it can be read as a number rather than decoded from a chattering
  // M/S gate. It should slew between the moving and static figures, never step.
  // wheelMovingL/R are still shown (MV) because they remain the raw evidence of how
  // violently the old gate was chattering -- but nothing in the PWM path reads them.
  Serial.print(" DBF "); Serial.print(plainFloorFilt, 1);
  Serial.print(" MV "); Serial.print(wheelMovingL ? "M" : "S");
  Serial.print(wheelMovingR ? "M" : "S");
  Serial.print(" PWMHZ "); Serial.print(MOTOR_PWM_HZ);
  Serial.print(" SET "); Serial.println(sweepIdx + 1);
  controlLog.encoderDeltaL = 0;
  controlLog.encoderDeltaR = 0;
}
