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
//
// 4482 -> 2000, 2026-08-15. LOWERING the frequency lengthens the period, so a PWM count buys
// more on-time and the electrical deadband shrinks -- the same mechanism as the 20 kHz
// disaster above, run in the useful direction:
//   @ 4482 Hz (223 us period): 1 count = 0.875 us -> ~9 counts to clear the ~8 us turn-on
//   @ 2000 Hz (500 us period): 1 count = 1.953 us -> ~4 counts to clear it
// Adopted from a three-point sweep (4482 / 2000 / 1200), which also revised the turn-on
// estimate down from the datasheet figure. Lower still was not taken: the period cannot grow
// without limit before the current ripple starts to be audible and to heat the bridge.
// THIS AND THE DEADBAND VALUES ARE A MATCHED PAIR. The 10/10 and 18/18 floors below were
// measured AT 2000 Hz; running one without the other means using floors measured under a
// different electrical deadband. Change either and re-run DEADBAND_TEST.
const int MOTOR_PWM_HZ = 2000;   // do NOT change without re-running DEADBAND_TEST
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
// Sign of the FIRST pass only; the test alternates every pass.
const int DEADBAND_TEST_SIGN = 1;
// MULTI-PASS. One flash and one power-up runs DEADBAND_TEST_PASSES ramps with the sign
// alternating each pass, coasting down between, then prints a summary with per-direction
// spreads.
//
// WHY REPEATS: a single ramp cannot tell an outlier from a regime, and this file has twice
// been steered by one. A "+/-1 count repeatability" premise sat under every deadband figure
// here until a run came back L@21 where the sweep had recorded L@11 -- which looked like a
// bimodal left channel and would have invalidated all of them. Six passes said otherwise:
// L@21 did not reproduce and each wheel's spread is 2-4 counts. Any future deadband claim
// needs a distribution, not a sample.
//
// WHY ALTERNATE rather than a block of + then a block of -: a block comparison confounds
// direction with anything that drifts across the session (motor warming, battery sag).
// Alternating puts both directions on the same drift. It has already answered the
// per-direction question -- each wheel's own spread within a direction exceeds the
// forward/reverse gap, so direction is not the variable.
//
// Each pass re-seeds after the rotor has coasted to a stop, so the passes also sample
// different rotor rest positions.
const int DEADBAND_TEST_PASSES   = 6;      // 3 per direction
const int DEADBAND_TEST_DWELL_MS = 1500;   // coast-down between passes, motors off

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

// ---- Lean-limit sweep ------------------------------------------------------
// Set to 1 to MEASURE where the chassis actually stops. Motors off throughout.
//
// WHY THIS EXISTS RATHER THAN JUST READING IMU_TEST BY EYE. The rear limit has been
// "measured" three times and come back -19, -15.7 and -21.0, and firmware was sized to each
// of them in turn. Every one of those was read off a scrolling stream or taken from a
// RELEASE settle -- where the chassis lands when you let go, which is wherever the torques
// happened to balance on that particular release, not where the chassis stops.
//
// The rule this enforces, already stated at LEAN_CLAMP_REAR: a limit is where the chassis
// stops under a SLOW DELIBERATE PUSH, and it must REPRODUCE ACROSS SWEEPS. So this mode
// refuses to report an angle until the body has been genuinely stationary
// (|rate| < LEAN_SWEEP_REST_DPS) for LEAN_SWEEP_REST_MS, and it prints the running
// distribution per direction so the spread is visible instead of inferred.
//
// Pitch comes from the ACCELEROMETER, not the fused estimate: the robot is stationary at the
// moment of measurement, so there is no linear acceleration to corrupt it and no gyro drift
// to accumulate. The gyro is used only to decide "is it holding still".
//
// HOW TO RUN:
//   1. Flash with this at 1. Robot on the floor, disarmed. Motors stay off the whole time.
//   2. Push the chassis SLOWLY forward until it stops on its own. Hold it there, hands
//      steady, until a "rest" line prints. Do not force past the stop.
//   3. Return upright, then repeat for the rear.
//   4. Do BOTH DIRECTIONS AT LEAST TWICE, ideally three times. The summary prints each
//      direction's spread; if a direction's spread is more than ~1 deg, the chassis is not
//      stopping on a hard feature and no single number should be trusted from it.
//   5. Set back to 0.
// A push that is too fast shows up as PEAK well beyond REST -- that is momentum, not travel,
// and the peak is reported separately so it cannot be mistaken for the limit.
//
// ⚠️ KNOWN LIMITATION, FOUND THE FIRST TIME THIS WAS RUN. A rest detector proves the body is
// STATIONARY. It cannot prove the body is SELF-SUPPORTED. A hand holding steady and a chassis
// sitting on its stop are identical to a gyro, so this mode WILL happily record a hand hold
// as a limit -- the first run produced a "rear limit" of -9.53 after the chassis had been to
// -54.56, which was simply the operator holding it near upright at the end of the sweep.
// The peak-vs-rest warning above now catches that case, but the discrimination is ultimately
// TACTILE and belongs to the operator: only push until you feel the frame come up solid, and
// throw away any rest that did not come with that feeling.
//
// ⚠️ accelPitch() IS ONLY VALID WHEN STILL. It reads gravity, so any hand acceleration adds
// straight into the angle. The rest gate (|rate| < LEAN_SWEEP_REST_DPS) exists partly for
// this reason. It also means the LIVE stream during a fast sweep is not trustworthy as an
// angle -- read the REST lines, not the scroll.
#define LEAN_SWEEP 0
const float         LEAN_SWEEP_REST_DPS = 2.0f;   // |deg/s| below this counts as stationary
const unsigned long LEAN_SWEEP_REST_MS  = 400;    // ...held this long = a genuine stop
const float         LEAN_SWEEP_MIN_DEG  = 8.0f;   // ignore rests near upright (that is a hand,
                                                  // not a stop); well inside any real limit
const int           LEAN_SWEEP_MAX_REST = 8;      // per direction, for the spread table

// ---- Back-EMF feedforward measurement --------------------------------------
// Set to 1 to measure K_EMF: the PWM needed per rev/s just to HOLD a wheel speed against
// back-EMF. WHEELS OFF THE GROUND. Motors ARE driven in this mode -- it is the one test mode
// here that spins them, so keep hands clear and let it run to the summary.
// See emfTestLoop for what the number is for and how to sanity-check the fit.
// Set back to 0 when done.
#define EMF_TEST 0
const int EMF_TEST_SIGN       = 1;    // which way to spin; magnitude is what matters
const int EMF_TEST_PWM_START  = 10;   // start at the moving floor, below it nothing turns
const int EMF_TEST_PWM_STEP   = 10;
const int EMF_TEST_PWM_MAX    = 140;  // = MAX_PWM; no point characterising past what we command
const int EMF_TEST_MAX_STEPS  = 16;   // (140-10)/10 + 1 = 14, with headroom
const unsigned long EMF_SETTLE_MS = 700;  // free-spin rotor takes a few hundred ms to settle
const unsigned long EMF_SAMPLE_MS = 400;  // average over this; long enough for many counts

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
// 14 -> 24, 2026-08-15. THIS IS THE CONSTANT THAT ACTUALLY LIMITS LEAN, and it took until
// now to notice. LEAN_CLAMP was raised to 60 fwd / 44 rear to "release the lean" and the
// change was INERT, because the three outer-loop terms are separately capped and their sum
// was the real ceiling:
//     Kpos * POS_ERROR_CLAMP = 6.0 * 1.5 = 9.0
//     Kveff * velError                  ~ 1.2
//     VEL_I_CLAMP                       = 14.0   <-- dominant
//                                         ------
//                                          24.2 deg, and the logs top out at RAW 18.95
// So no LEAN_CLAMP above ~24 could ever bind. At 24 the sum becomes ~34 deg of reachable
// lean, which clears the rear torque null at lean -14.98 with real room on the far side --
// the point of releasing it in the first place. Still inside FALL_CUTOFF_REAR 47 and well
// inside the frame.
// WHY IT IS NEEDED AT ALL, and it is a workaround not a fix: the chassis is only slightly
// top-heavy, so L (CoM height above the axle) is small, and acceleration goes as
// g * L*sin(pitch) / (R + L*cos(pitch)). A small L means a large lean buys little force, so
// the loop has to wind further to get the same acceleration out of a weak pendulum. Raising
// the CoM is the real fix and makes this constant matter less. Pilot is moving mass up.
// THE COST IS WINDUP. The integral can now store 24 deg instead of 14, so anything that
// unwinds it does so from further out. Conditional integration freezes it at saturation and
// the breakaway edge detector zeroes it on the stall->rolling transition, so the designed
// path is covered -- but WATCH THE BREAKAWAY LURCH. If the robot snaps hard when the wheels
// finally free, this is where it came from, and the answer is a lower number here, not more
// damping in the inner loop.
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
// ══ 140 mm WHEELS, LEAN_SWEEP 2026-08-15: GEOMETRY IS NO LONGER THE BINDING LIMIT. ══
// Every lean figure in this file predates the 120 -> 140 mm wheel change and is now stale.
// Re-measured with LEAN_SWEEP (motors off, slow push to a stop the pilot confirmed by feel
// as "hard stop - solid bot frame"), two sweeps:
//     forward  +61.0   run 2 held 60.6..61.6 across ~17 samples at |rate| 0.0-2.0
//     rear     ~-51    run 1 -50.1..-53.4, run 2 -49.4..-51.7; the overlap is -50..-51.7
// Run 1's forward readings (a hold at 49.7, then 55.3) were UNDER-PUSHES, not disagreement:
// the operator held mid-sweep, pushed further, and released before reaching the frame. Only
// run 2 actually arrived. Worth remembering when a repeat "disagrees" -- check whether the
// earlier one ever got there before treating the difference as spread.
// Against the OLD figures of ~+30 / -25, the extra 10 mm of axle height did not widen the
// range so much as remove the feature that was catching.
//
// WHAT NOW BINDS, in order (smallest first) -- the chassis is LAST:
//     saturation latch   ~19.7 deg ACTUAL   <-- tightest, and it is a BUG, see below
//     RECOVERY_GIVEUP    32 deg
//     FALL_CUTOFF        63 fwd / 47 rear   <-- now AT the frame (was a symmetric 45)
//     chassis            ~-51 rear / +61 fwd
// SUPERSEDED: this held only while FALL_CUTOFF was a symmetric 45. It is now AT the frame,
// so the armed robot CAN reach its stops. Old text kept for the arithmetic: 45 fired 6 deg before
// the rear frame and 16 deg before the forward frame, so chassis contact is now reachable
// only with the motors already disarmed. Every "the robot is sitting on its stop" style of
// explanation for a stall is off the table while armed -- that was a live hypothesis at
// 120 mm and it is dead at 140 mm.
// So raising this clamp past ~19 buys nothing today: the saturation latch fires first. That
// latch computes its trip point at Kp 2.0 (78.85 effort = 39.4 deg) but the loop runs at
// Kpeff 4.0 while driving, so it really trips near half that. Fixing that Kp-vs-Kpeff
// mismatch is the prerequisite for using any of the range the new wheels just unlocked --
// it is a real bug, it predates this branch, and it is now the tightest constraint on the
// robot rather than a background annoyance.
// LEFT AT 18 DELIBERATELY. Sizing it to the new geometry before the latch is fixed would
// just move the clamp behind a limit that fires earlier, and this file has enough history
// of numbers sized against the wrong constraint.
// ══ SPLIT PER DIRECTION 2026-08-15, AND THE REAR NUMBER GOES DOWN, NOT UP ══
// LEAN_SWEEP found a STABLE EQUILIBRIUM AT PITCH -18.2 (motors off, 23 consecutive samples
// between -18.09 and -18.34 at |rate| 0.0-0.4, spread 0.25). The pilot had already named it:
// the chassis has distinct points of balance, and "when the lean is similar to those points
// it can't move".
//
// THAT IS THE STALL. Gravity torque about the contact patch is ZERO at an equilibrium, so a
// commanded lean placed there has no drive authority by construction. The loop reaches the
// target, ERR -> 0, u -> 0, PWM -> 0, and the robot sits. It was never failing to track --
// it was arriving. Every "leans well but doesn't move" log this session is a target landing
// on an equilibrium:
//     LEAN_CLAMP_REAR 14 -> TARGET -3.22 - 14 = -17.22, observed PITCH -17.2..-18.2,
//                           ERR ~0, U ~0, PWM 0, POS frozen for seconds
//     and on the 120 mm wheels the same thing forward, at the +20.8 equilibrium:
//                           PITCH 20.78 TARGET 20.78 ERR 0.00 U -0.01 PWM 0 0
// The forward trap is GONE with the 140 mm wheels (pilot: "front disappeared with the new
// wheels"), which is why forward always drove better than reverse this session. The rear one
// remains.
//
// SO THE REAR CLAMP MUST MOVE AWAY FROM -18.2, AND WE SPENT THE SESSION MOVING IT TOWARD.
// 14 -> 16 -> 18 walked the target from 1.0 deg short of the equilibrium to 3.0 deg past it.
// 10 puts TARGET at -13.22, a clear 5.0 deg short.
// WHY THE SHALLOW SIDE: between upright and -18.2 the gravity torque carries the body
// further back, which is the direction that produces reverse drive. Past -18.2 the restoring
// torque reverses and works against the commanded direction. The shallow side is also inside
// every other limit and is ground the robot demonstrably traverses.
// NOT YET CONFIRMED: whether the far side is usable at all. If a nudge past -18.2 continues
// away rather than returning, the equilibrium is not stable in the way assumed here and the
// useful band is the other one. Sizing to the shallow side does not depend on that answer;
// placing the target ON the equilibrium is wrong either way.
//
// FORWARD IS UNCHANGED AT 18 and stays evidence-based: TARGET +14.78 is proven in flight,
// there is no forward equilibrium left to avoid, and the real forward limits are far away
// (RECOVERY_GIVEUP 32, FALL_CUTOFF 45, frame +61). Raise it separately if reverse comes good.
// ══ OPENED RIGHT UP 2026-08-15, ON PILOT'S CALL ══
// "release all lean. Doesn't matter if it's usable or not. I should be controlled by the
// pilot." So the firmware stops deciding which part of the range is worth having. Both caps
// go to 40, symmetric, and the pilot flies whatever is there.
//
// Sized against the FRAME, not against an arbitrary margin -- pilot: "we shouldn't
// overconstrain it." The fall latch now sits at the measured frame contacts
// (FALL_CUTOFF_FWD_DEG 63 / REAR 47, in lean), so these sit just inside those, leaving ~3 deg
// for tracking overshoot so that commanding full stick does not itself trip the latch.
// Two safety checks had to move first, and BOTH would have cut the motors at full stick:
//   * FALL_CUTOFF_DEG was a single symmetric 45, below the frame in both directions.
//     Now split and moved out to the frame.
//   * RECOVERY_GIVEUP_DEG 32 gated on the LEAN, not on tracking error -- I had this
//     backwards and said so out loud. Any commanded lean past 32 held for 2 s latched the
//     robot as fallen. Harmless at LEAN_CLAMP 18, fatal at 44. Now gates on tracking error,
//     which is what "the stand-up attempt timed out" actually means.
//
// WHAT THE PILOT IS ACCEPTING, stated plainly rather than prevented:
//   * The rear TORQUE NULL at lean -14.98 (PITCH -18.2) is now INSIDE the range, not fenced
//     off. Commanding through it will feel dead -- that is the physics, not a fault, and it
//     is the thing the 10 deg cap existed to avoid. See lean-equilibria notes.
//   * Beyond the null the restoring torque opposes the commanded direction, so deep rear
//     lean may drive the wrong way. Unmeasured; now reachable.
//   * At full rear stick the robot is 3 deg from the frame and the fall latch is what stops
//     it. That is the intended behaviour now, not a margin failure.
// Was 18 fwd / 10 rear, then 40/40. If the fall latch fires on full stick, drop these rather
// than pushing FALL_CUTOFF past the frame -- past the frame there is nothing left to catch it.
// LEAN_CLAMP_FWD / _REAR / LEAN_CLAMP REMOVED 2026-08-15 -- the outer loop is uncapped. See
// the note at the leanRaw sum for what bounds it instead (~25 deg from the per-term caps).
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
// 0.97 -> 0.95, 2026-08-15. tau = -DT/ln(a): 164 ms -> 98 ms, so the outer-loop lag roughly
// halves. This value was flown once before and reverted with the note "rear margin is
// scarcer than response time" -- THAT REASON HAS EXPIRED. Rear margin was scarce when the
// fall latch was a symmetric 45 deg of lean and the frame angle was unknown/wrong. The frame
// is now measured (rear contact at lean -47.8) and the latch sits at 47, with reachable lean
// ~34 deg (VEL_I_CLAMP 24). There is ~13 deg of unused rear travel, so trading a little of
// it for response time is no longer the trade it was.
// This is pure lag reduction and costs nothing in windup: the EMA only smooths leanCmd, it
// does not store anything. Contrast VEL_I_CLAMP, where more range does mean more to unwind.
// The 0.95-"RAN AWAY" warning above is older still and predates the cascade entirely; see
// that note for why it does not carry over. This remains the first knob to slow back down if
// the OUTER loop starts oscillating -- distinguishable from the inner-loop ring by frequency,
// outer-loop oscillation is slow (order 1 Hz), the floor-driven limit cycle was ~8 Hz.
const float LEAN_LPF   = 0.95f;  // EMA on leanCmd (~98 ms). Higher = slower outer loop.
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
// 110 -> 140, 2026-08-15, for the 140 mm wheels. Ground force is torque/R, so the 120 -> 140
// wheel change cut the force available at any given PWM by 1/1.167. Holding the ceiling at
// 110 therefore quietly REDUCED the robot's peak accelerating force by 14% relative to the
// build these gains were tuned on. 110 * 1.167 = 128 restores parity; 140 restores it and
// adds ~9% of genuine new headroom, which is what "release some PWM" asks for.
// Still only 55% of the 255 range, so there is more available if it proves useful.
// WHAT THIS DOES AND DOES NOT BUY: it raises the TOP end -- recovery from large angles, and
// peak acceleration. It does NOT help breakaway. Breakaway happens at small commanded
// effort, where PWM is plainFloor*ramp + |u| and the FLOOR dominates; the ceiling is not in
// that expression. Peak PWM across every log this session was 37-50 against a ceiling of
// 110, so this changes nothing in normal operation and only shows up in the transients.
// KNOCK-ON, do not miss it: the saturation latch threshold is
// SAT_EFFORT_FRAC * (MAX_PWM - MAX_DEADBAND), so it moves 87.4 -> 115.9 with this. The
// effort half of that latch is now even less likely to bind, which is fine -- since the
// 2026-08-15 fix the ANGLE half (SAT_TRIP_DEG 39) is what actually decides, and it is
// unaffected by this constant. FALL_CUTOFF (63 fwd / 47 rear, at the frame) is the ceiling.
// WATCH MOTOR HEAT AND CURRENT. This is the constant most likely to cook something; the
// previous note said the same and it still applies with more force at 140.
const int   MAX_PWM        = 140;   // ceiling, 0-255. 80 -> 110 -> 140 (140 mm wheels)
const float OUT_DEADZONE   = 0.5f;  // ignore PD outputs smaller than this (PWM units)
// Effort over which the stiction floor ramps in from 0 to full, instead of stepping.
// Sized against the observed ring: u swings +/-9 during a drive, so 5.0 softens more
// than half of that amplitude while leaving the floor fully applied for real drive
// efforts (9-30). Raise it if the 8 Hz ring persists; lower it if breakaway suffers.
const float FLOOR_KNEE     = 5.0f;  // PWM-effort units

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
// ══ RETRACTED 2026-08-15. THE 15-vs-27 ASYMMETRY WAS NEVER IN THE MACHINE. ══
// Everything above about a "MECHANICAL fault on the right drivetrain" is wrong and is kept
// only so the reasoning that produced it stays visible. The cause was an ENABLE-PIN WIRING
// FAULT on one BTS7960: one bridge braked at idle while the other coasted, so the "stiffer"
// wheel was being electrically braked, not mechanically dragged. The giveaway, from the
// pilot: the right wheel was tighter ONLY WHEN POWERED -- with the robot off, both spun the
// same. That is electromagnetic braking, not friction.
// The pilot repaired the wiring. Re-measured afterwards over FOUR loaded runs and ~45 pooled
// samples, with the multi-pass harness below and its twitch rejection, the wheels are
// SYMMETRIC. Keep the pairs equal unless there is evidence to split them; the old split is
// what sent a whole session chasing a drivetrain fault that did not exist.
//
// The per-direction caveat below is also answered: 12 samples, both directions, three passes
// each way. Each wheel's OWN spread within a direction exceeds the forward/reverse gap, so
// direction is not the variable and a per-sign floor would be fitting a constant to noise.
//
// MEASURED AT MOTOR_PWM_HZ 2000, which is what this file now runs -- the frequency change was
// taken together with these values, as a matched pair. Change one and re-run DEADBAND_TEST.
// The symmetry finding is frequency-independent; only the magnitudes move.
const int LEFT_DEADBAND_MOVING  = 10;   // free-spin median, post-EN-fix 2026-08-15
const int RIGHT_DEADBAND_MOVING = 10;   // keep equal without evidence to split them
const int LEFT_DEADBAND_STATIC  = 18;   // loaded median, post-EN-fix 2026-08-15
const int RIGHT_DEADBAND_STATIC = 18;   // ditto
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
// 0.40 -> 0.65, 2026-08-15. THE RESOLUTION ARGUMENT ABOVE RUNS THE WRONG WAY and should not
// have held this down. One encoder count per 5 ms tick is 0.366 rev/s, so:
//     0.40 rev/s -> 1.1 counts/tick   (worse resolution)
//     0.65 rev/s -> 1.8 counts/tick   (better)
// Going FASTER improves the velocity estimate; it is standing still that starves it. The
// real constraint on top speed is stopping distance, and 0.65 is the value that analysis
// produced (0.90 -> 0.65) and that was flying before the reset to v2-drive-works.
// With 140 mm wheels 0.65 rev/s = 0.29 m/s, up from 0.176.
const float DRIVE_MAX_VEL  = 0.65f; // rev/s at full stick; sized by stopping distance
// Lower bound on the commanded speed once the stick is out of its deadzone. Without it the
// first usable stick position asks for a crawl that sits inside the dither band and never
// breaks away, which is what "can't break away on small throttle values" was. Span is now
// 0.30..0.65 rather than 0..0.40 -- narrower AND faster at the bottom.
const float DRIVE_MIN_VEL  = 0.30f; // rev/s just outside the stick deadzone (scaled by the cap)
// 25 -> 16: "turns are still faster" than the forward drive they are mixed into.
const float TURN_AUTHORITY = 16.0f; // PWM-effort differential at full turn stick
// Fraction of TURN_AUTHORITY still available at ZERO throttle. Turn is scaled by the live
// throttle so rotation cannot out-run translation; this floor keeps a stationary pivot
// possible. 0 = no turning at all unless the drive stick is off centre.
const float TURN_THROTTLE_FLOOR = 0.30f;
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
// FIXED 2026-08-15: the latch used the effort test ALONE, and effort is not an angle.
// Threshold is SAT_EFFORT_FRAC * (MAX_PWM - MAX_DEADBAND) = 0.95 * (110 - 18) = 87.4, and
// |u| ~ Kpeff * |error|, so the angle it really trips at depends on which gain is live:
//     Kpeff 2.0 (balancing) -> 87.4 / 2.0 = 43.7 deg
//     Kpeff 4.0 (driving)   -> 87.4 / 4.0 = 21.9 deg
// A 2x swing in a safety threshold, driven by a gain schedule that has nothing to do with
// safety -- and the tighter half lands while DRIVING, which is when large transient tracking
// errors actually happen. The surrounding comments (see FALL_CUTOFF_DEG) reason about this
// latch as a fixed "above ~39 deg" angle backstop, which it has never been at drive gains.
// This has not been firing because ERR stays near 0 in normal driving, so u stays far below
// 87.4. It becomes a live hazard the moment LEAN_CLAMP goes up: bigger commanded leans mean
// bigger transient ERR through a stick reversal, and 22 deg for half a second is reachable.
// FIX: require BOTH conditions -- the output genuinely pinned AND the angle genuinely
// hopeless. The angle test is on `error` directly, so it cannot be rescaled by a gain.
//     Kpeff 2.0 -> effort trips at 43.7, angle at 39.0 -> latch at 43.7 (effort binds)
//     Kpeff 4.0 -> effort trips at 21.9, angle at 39.0 -> latch at 39.0 (angle binds)
// Gain dependence collapses from 2.0x to 1.12x, and both ends now sit in the 39-44 band the
// design already assumed.
// COST, stated honestly: a motor pinned at less than SAT_TRIP_DEG no longer latches through
// THIS path. That is deliberate -- pinned effort at a recoverable angle is the loop working,
// not failing, and it is what the old test could not distinguish. RECOVERY_GIVEUP_DEG (32,
// with its own timeout) covers the band above it, and FALL_CUTOFF at the frame covers the top.
const float         SAT_TRIP_DEG    = 39.0f;   // deg of |error| that also has to be true

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
// ══ SPLIT AND MOVED TO THE FRAME 2026-08-15, ON PILOT'S CALL ══
// "the hard ceiling should be the rest degree. when the bot leans on the frame."
// Right: below frame contact the robot is still on its wheels and could in principle
// recover, so latching at 45 was giving up with travel still in hand. Measured frame
// contacts (LEAN_SWEEP, 140 mm wheels), converted to LEAN = pitch - BALANCE_SETPOINT:
//     rear     PITCH -51  ->  lean -47.8   -> cutoff 47
//     forward  PITCH +61  ->  lean +64.2   -> cutoff 63
// Asymmetric because the chassis is, and because BALANCE_SETPOINT -3.22 shifts the two
// directions oppositely. A single symmetric number would have to take the rear value and
// would then throw away 17 deg of forward travel.
//
// THE OLD 45 RESTED PARTLY ON A FALSE PREMISE. Its comment said "beyond ~45 deg the accel
// pitch gets ambiguous". It does not: accelPitch() is atan2(ax, az), which is single-valued
// through +/-180, and the LEAN_SWEEP run read +61 and -51 cleanly and stably for seconds.
// There is no sensor reason to stop at 45.
// WHAT IS STILL TRUE: past the frame the robot IS down, and fighting it there just cooks
// motors. That is what these now mark, and nothing else.
const float FALL_CUTOFF_FWD_DEG  = 63.0f; // lean beyond this = on the forward frame, stop fighting
const float FALL_CUTOFF_REAR_DEG = 47.0f; // lean beyond this = on the rear frame, stop fighting
// A stand-up attempt must be BOUNDED or it cooks the motors. Two backstops now cover the
// whole range: above SAT_TRIP_DEG (39) of error, with the output also pinned, the saturation
// latch fires in 0.5 s.
// (The figure quoted here used to be "78.85, i.e. 39.4 deg at Kp 2.0". Both halves were
// wrong: 78.85 was computed with the pre-EN-fix MAX_DEADBAND of 27 and is now 87.4, and the
// "39.4 deg" only ever held at Kp 2.0 -- at the Kpeff 4.0 used while driving the same effort
// meant 21.9 deg. The latch now tests the angle explicitly so this number means what it
// says at both gains. See SAT_TRIP_DEG.) BELOW that, between 25 and 39 deg, u sits under the sat threshold and the old
// code would have pushed at ~60 PWM indefinitely against an obstacle -- which is exactly
// the band this 31.6 deg pose lives in. Hence the explicit timeout.
// 25 deg is chosen to clear normal driving: LEAN_CLAMP caps the COMMANDED lean at 14 and
// the worst observed ACTUAL overshoot was 18.5 deg, so this cannot false-trip on a drive.
// 25 -> 32. Had to move with LEAN_CLAMP: a commanded 18 deg overshoots in practice
// (~1.4x observed), so 25 would false-trip the give-up timeout during a normal
// aggressive drive and latch the motors off mid-run. Still well under FALL_CUTOFF.
const float         RECOVERY_GIVEUP_DEG   = 32.0f; // deg of error that counts as "still down"
const unsigned long RECOVERY_GIVEUP_TICKS = 400;   // 2 s at 200 Hz, then give up and latch
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

// ---- Back-EMF feedforward ---------------------------------------------------
// Supplies the PWM a motor needs just to HOLD its speed against back-EMF, so the inner PD
// no longer has to manufacture it out of a standing pitch error. That standing error was the
// pilot's "bot may escape the commanded lean when it reaches higher velocity": PITCH walks
// away from TARGET as VF rises, because the only thing generating cruise voltage was ERR.
//
// MEASURED, not guessed. EMF_TEST free-spin staircase, 14 steps from PWM 10 to 140:
//     LEFT   K_EMF 43.6 PWM per rev/s, intercept 4.1, n 13
//     RIGHT  K_EMF 45.1 PWM per rev/s, intercept 5.1, n 14
// Per-step slopes held 42-47 across the whole range, so the linear model is right rather
// than merely fitted. The intercept landing at 4-5 rather than at the MOVING deadband of 10
// is expected and is itself a check: 10 is the BREAKAWAY threshold of a stopped wheel, while
// this line describes a wheel already turning, and kinetic friction is below stiction.
//
// ONE GAIN FOR BOTH WHEELS. They differ by 3%, which is not evidence of anything -- and this
// file has already lost a session to a "wheel asymmetry" that turned out to be a wiring
// fault. Use the smaller of the two so the error is on the under-compensating side.
//
// DERATED TO 80% (43.6 -> 35). This term cancels the motor's own velocity damping. At 100%
// the motor becomes very nearly a pure torque source; ABOVE 100% it has NEGATIVE damping and
// wheel speed runs away, because this is velocity feedback on the output. The measurement is
// clean, but battery sag and the loaded case both move the true value around, and there is no
// symmetric penalty for being low: under-compensating just leaves some of the old standing
// offset behind, over-compensating destabilises the wheel. Raise toward 43 once a flight
// shows the offset gone and nothing running away.
//
// WATCH FOR QUANTISATION. One encoder count is 0.366 rev/s, so a single count of velocity
// noise becomes 35 * 0.366 = 12.8 PWM through this gain. VEL_LPF 0.90 is what keeps that
// tolerable. If the wheels buzz at low speed after this goes in, it is this amplification and
// the fix is more velocity filtering, NOT a lower gain -- the gain is measured.
const float EMF_FF_GAIN = 35.0f;   // PWM per rev/s; 80% of the measured 43.6
// SIGN_CFG says TARGET_FWD + ENC_FWD + PWM_FWD -, i.e. physical forward is POSITIVE encoder
// velocity and NEGATIVE PWM. wheelVelL/R come straight off encoder deltas with no flip, so
// sustaining forward motion needs a MORE NEGATIVE pwm as speed rises: hence -1.
// If the robot accelerates uncontrollably the instant it starts rolling, this sign is wrong.
const float EMF_FF_SIGN = -1.0f;
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
const float LEAN_RATE_FF_MAX = 60.0f; // deg/s : bound on the setpoint-rate term (the outer loop
                                      // swing in 100 ms). Stops a leanCmd STEP becoming a PWM kick.
float dTermLog = 0.0f;    // derivative contribution, for telemetry
float leanCmd   = 0.0f;   // cascade outer-loop output: the desired lean (deg) added to BALANCE_SETPOINT
float velocityLeanI = 0.0f;
float frictionUSlow = 0.0f;   // low-passed u; aims the friction floor (DRIVE_FRICTION_FF)
bool  frictionLeanUp = false; // latched LAUNCH -> CRUISE phase for the friction floor
float ffFloorFilt = 38.0f;    // slew-limited FF floor; starts at the stopped value (18 + 20).
                              // Was 47 = the OLD 27 static deadband + boost, stale since the
                              // EN wiring fix made the floors 18/18.
float plainFloorFilt = 18.0f; // same, for the no-FF path. Starts at the STATIC figure because
                              // the wheels are stopped at boot; no boost on this path.
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
  int   emfFFL = 0, emfFFR = 0;  // PWM added by the back-EMF feedforward, per wheel (see EMF_FF_GAIN)
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
#elif LEAN_SWEEP
  Serial.println("LEAN_SWEEP mode: motors OFF, robot on the FLOOR.");
  Serial.println("Push SLOWLY to the forward stop, HOLD steady until a REST line prints,");
  Serial.println("return upright, then repeat to the rear. Do each direction 2-3 times.");
  Serial.println("Only HELD angles are recorded. A release settle is not a limit, and a");
  Serial.println("peak far beyond the rest means you pushed too fast -- redo that one.");
#elif EMF_TEST
  Serial.println("EMF_TEST mode: WHEELS OFF THE GROUND. Free-spin PWM staircase.");
  Serial.println("Do not touch the wheels. Each step settles, then averages speed.");
  Serial.println("Reports a least-squares fit of PWM = K_EMF*rev/s + intercept per wheel.");
  Serial.println("K_EMF is the feedforward gain; the intercept should land near the");
  Serial.println("MOVING deadband (~10). If it does not, the linear model is wrong.");
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
// ffBias (-1..+1, see DRIVE_FRICTION_FF) aims the floor at the outer loop's SUSTAINED
// demand instead of at the sign of this tick's effort. At 0 this is the original map
// exactly, so standing balance is unchanged.
//
// plainFloor replaces the `moving ? dbMoving : dbStatic` this function used to do for itself.
// THAT GATE CHATTERED AT THE LOOP RATE AND SQUARE-WAVED THE FLOOR. wheelMoving toggles at
// encoder-noise level whenever the wheels are barely turning, which is exactly the condition
// the floor exists for. Consecutive telemetry samples, 2026-08-15 flight:
//     DBL 10M DBR 10M  ->  DBL 18S DBR 18S  ->  DBL 10M DBR 18S  ->  DBL 18S DBR 10M
// an 8-count step in the floor every tick, on both wheels, independently. That is a second
// relay sitting on top of the sign relay this ramp was written to tame, and it drove an
// ~8.7 Hz limit cycle: post-fix bursts hold one PWM sign for tens of samples with RATE +/-3
// where it had been +/-25..30.
// The continuous static->moving blend that fixes it (ffRoll, see driveControlDiff) was
// already built -- but ONLY the ffBias branch below consumed it, and with DRIVE_FRICTION_FF 0
// that branch is inert, so the chattering gate was the live path the whole time.
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
  // Ramped, the incremental gain near zero drops from infinite to plainFloor/FLOOR_KNEE + 1
  // (about 4.6 for the 18 floor at FLOOR_KNEE 5.0), which is what takes the energy out of
  // the cycle. This knob is a TRADE-OFF, not a fix: lowering FLOOR_KNEE raises that gain and
  // buys breakaway at the cost of jitter. Flown points -- 5.0 -> 4.6 (quiet), 2.5 -> 8.2
  // (jitters), 1.0 -> 19.0 ("made it worse. To jittery, can't break away"). Do not reach for
  // it to fix breakaway.
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
  // SAME TREATMENT FOR THE PLAIN (no-FF) PATH. The comment that used to sit here claimed
  // wheelMoving was safe on this path because "the floor only augments |effort| and a step in
  // it cannot reverse the command". True but irrelevant: it does not need to reverse anything
  // to do damage. An 8-count step between the 10 and 18 floors, arriving at the loop rate and
  // independently per wheel, is a square wave injected straight into the output -- and with
  // DRIVE_FRICTION_FF 0 this is the ONLY path, so the ffRoll blend above was being computed
  // and thrown away while the gate it replaces stayed live. Same blend, no boost (that is a
  // feedforward-only term), same LPF for the same reason.
  float plainTarget = fStatic * (1.0f - ffRoll) + fMoving * ffRoll;
  plainFloorFilt = FF_FLOOR_LPF * plainFloorFilt + (1.0f - FF_FLOOR_LPF) * plainTarget;
  int pwmL = mapEffortToPwm(uL, ffBias, ffFloor, plainFloorFilt);
  int pwmR = mapEffortToPwm(uR, ffBias, ffFloor, plainFloorFilt);
  // Back-EMF feedforward: the voltage each wheel needs simply to HOLD its current speed.
  // Added AFTER the friction floor because it compensates a different thing -- the floor is
  // Coulomb friction (constant magnitude, sign of the command), this is back-EMF (grows with
  // speed, sign of the MOTION). Per-wheel on each wheel's own measured velocity, so it stays
  // correct through a turn where the two wheels run at different speeds.
  // Naturally zero at standstill, which is why this is safe where the two previous
  // feedforwards were not: it cannot contribute during the non-minimum-phase launch that
  // killed both of them. See EMF_FF_GAIN.
  const int emfL = (int)(EMF_FF_SIGN * EMF_FF_GAIN * wheelVelL);
  const int emfR = (int)(EMF_FF_SIGN * EMF_FF_GAIN * wheelVelR);
  pwmL += emfL;
  pwmR += emfR;
  controlLog.emfFFL = emfL;
  controlLog.emfFFR = emfR;
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
// A wheel has broken free when it ADVANCES by DB_STEP_COUNTS within one 200 ms step AND
// KEEPS ADVANCING on the next one. The single-shot `cumulative > 5` test this replaces
// fires on backlash: loaded pass 3 had dL reach 6 at PWM 11 and then sit at EXACTLY 6
// through PWM 15 before really breaking at 16 -- the take-up of gear lash, not rotation,
// and a 5-count error in the number every floor in this file is sized from.
// Requiring a second advance costs one 200 ms step of resolution and rejects the twitch.
const int DB_STEP_COUNTS = 5;

void deadbandDetect(int pwm, long d, long prevD, int &cand, long &candD, int &moved) {
  if (moved) return;
  if (cand) {
    if (d >= candD + DB_STEP_COUNTS) { moved = cand; return; }  // advanced again: confirmed
    cand = 0;                                                   // stalled: it was a twitch
  }
  if (d >= prevD + DB_STEP_COUNTS) { cand = pwm; candD = d; }   // provisional
}

// Min/max of the passes matching one direction. Returns how many were found; lo/hi are
// only meaningful when that is nonzero.
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
  Serial.println("READ: spread within a direction FIRST. If a wheel's own spread is as");
  Serial.println("big as the fwd/rev gap, direction is not the variable and per-direction");
  Serial.println("deadbands would be fitting a constant to noise. Only if both directions");
  Serial.println("are tight (<=1) and differ does mapEffortToPwm need a per-sign floor.");
  Serial.println("=========================");
}

void deadbandTestLoop() {
  static bool seeded = false;
  static long baseL = 0, baseR = 0;
  static int  pwm = 0, movedL = 0, movedR = 0;     // 0 = wheel hasn't moved yet
  static unsigned long lastStep = 0;
  static int  candL = 0, candR = 0;                // provisional breakaway, not yet confirmed
  static long candDL = 0, candDR = 0, prevDL = 0, prevDR = 0;
  static int  pass = 0;
  static int  sign = DEADBAND_TEST_SIGN;
  static bool coasting = false, finished = false;
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
  // Coast-down between passes, motors off, so the next pass starts from a genuine standstill
  // and from a different rotor rest position.
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

// Lean-limit sweep: report only angles the chassis actually HELD. See LEAN_SWEEP.
void leanSweepPrintSpread(const char *label, const float *v, int n) {
  Serial.print("  "); Serial.print(label); Serial.print(" ");
  if (n == 0) { Serial.println("no rests recorded"); return; }
  float lo = v[0], hi = v[0], sum = 0.0f;
  for (int i = 0; i < n; i++) {
    if (v[i] < lo) lo = v[i];
    if (v[i] > hi) hi = v[i];
    sum += v[i];
    Serial.print(v[i], 2); Serial.print(" ");
  }
  Serial.print(" | n "); Serial.print(n);
  Serial.print("  mean ");   Serial.print(sum / n, 2);
  // The whole point of the mode: say out loud whether the number is usable.
  // n < 2 MUST NOT BE CALLED TIGHT. A single sample has a spread of zero by definition, and
  // the first version printed "spread 0.00 -> TIGHT, usable" off one reading -- which is
  // precisely the fit-a-constant-to-noise error this mode was built to prevent. It said
  // exactly that about a 49.65 that turned out to be a mid-push hand hold 11 deg short of
  // the real stop, and about a -9.53 taken 45 deg away from the peak.
  if (n < 2) {
    Serial.println("  spread n/a  -> ONE SAMPLE, NOT A MEASUREMENT. Sweep this direction again.");
    return;
  }
  Serial.print("  spread "); Serial.print(hi - lo, 2);
  Serial.println((hi - lo) <= 1.0f ? "  -> TIGHT, usable"
                                   : "  -> LOOSE, do not size to this");
}

void leanSweepLoop() {
  motorRaw(0);                                    // motors stay off for the entire mode
  static unsigned long lastPrint = 0, restSince = 0;
  static bool  resting = false, armed = true;
  static float fwd[LEAN_SWEEP_MAX_REST], rear[LEAN_SWEEP_MAX_REST];
  static int   nFwd = 0, nRear = 0;
  static float peakFwd = 0.0f, peakRear = 0.0f;

  imu.update();
  const float p    = accelPitch();
  const float rate = fabs(imu.getGyroY() - gyroYBias);   // magnitude only; sign irrelevant here

  if (p > peakFwd)  peakFwd  = p;                 // peaks include momentum -- reported, not used
  if (p < peakRear) peakRear = p;

  // A rest is |pitch| past the deadzone AND the body holding still for a sustained window.
  // `armed` stops one long hold from printing repeatedly: the chassis must move again
  // (leave the rest condition) before another rest can be recorded.
  const bool still = (rate < LEAN_SWEEP_REST_DPS) && (fabs(p) >= LEAN_SWEEP_MIN_DEG);
  if (still) {
    if (!resting) { resting = true; restSince = millis(); }
    else if (armed && (millis() - restSince) >= LEAN_SWEEP_REST_MS) {
      armed = false;
      float *arr = (p > 0.0f) ? fwd : rear;
      int   &n   = (p > 0.0f) ? nFwd : nRear;
      if (n < LEAN_SWEEP_MAX_REST) arr[n++] = p;
      Serial.println();
      Serial.print("LEAN_SWEEP  REST  ");
      Serial.print(p > 0.0f ? "FWD  pitch +" : "REAR pitch ");
      Serial.print(p, 2);
      const float pk = (p > 0.0f) ? peakFwd : peakRear;
      Serial.print("   peak this direction ");
      Serial.print(pk, 2);
      // BUGFIX: this warning used to be `p > 0.0f ? "..." : ""`, i.e. keyed on DIRECTION
      // rather than on the discrepancy. It therefore fired on every forward rest (even when
      // peak and rest agreed to 0.6 deg) and NEVER on a rear one -- including the rear rest
      // that reported -9.53 after a peak of -54.56, a 45 deg miss, which is the exact case
      // it exists to catch. Key it on the actual gap, both directions.
      if (fabs(pk) - fabs(p) > 2.0f) {
        Serial.print("  <-- PEAK IS ");
        Serial.print(fabs(pk) - fabs(p), 1);
        Serial.print(" DEG BEYOND THIS REST: not a stop. Either you pushed past and came ");
        Serial.print("back, or this is a hand hold. REDO.");
      }
      Serial.println();
      leanSweepPrintSpread("FWD ", fwd,  nFwd);
      leanSweepPrintSpread("REAR", rear, nRear);
      Serial.println();
    }
  } else {
    resting = false;
    if (fabs(p) < LEAN_SWEEP_MIN_DEG) armed = true;   // back near upright: ready for the next
  }

  if (millis() - lastPrint < 100) return;             // live stream at 10 Hz
  lastPrint = millis();
  Serial.print("LEAN_SWEEP pitch "); Serial.print(p, 2);
  Serial.print("  rate ");           Serial.print(rate, 1);
  Serial.print("  ");                Serial.print(still ? (resting ? "HOLDING" : "still") : "moving");
  Serial.print("  | rests F/R ");    Serial.print(nFwd);
  Serial.print("/");                 Serial.println(nRear);
}

// ---- Back-EMF / velocity feedforward measurement (see EMF_TEST) -------------
// Drives a PWM STAIRCASE with the wheels off the ground and records the STEADY free-spin
// speed at each step, then least-squares fits PWM = K_EMF * rev/s + intercept.
//
// WHY THIS NUMBER MATTERS. Nothing in the control path supplies the voltage a motor needs
// just to HOLD a speed against its own back-EMF. The only thing writing PWM is the inner PD,
// which manufactures output from PITCH ERROR -- so at cruise the loop has to produce the
// speed-holding voltage out of a standing pitch offset, and that offset grows with speed.
// That is the "bot escapes the commanded lean at higher velocity" symptom: PITCH walks away
// from TARGET as VF rises and ERR never returns to zero.
// With K_EMF known, `pwm += K_EMF * wheelVel` supplies it directly and the PD goes back to
// correcting disturbances only.
//
// THE FIT IS THE POINT, NOT THE RAW ROWS. Two things to check before trusting K_EMF:
//   * the intercept should land near the MOVING deadband (~10). It is the PWM extrapolated
//     to zero speed, which is exactly what that floor is. If it comes back at 40, the
//     relationship is not linear over this range and a single gain will not model it.
//   * the per-step slopes printed alongside should be roughly constant. Curvature means
//     the motor is saturating (approaching no-load speed) and the fit should use only the
//     lower steps.
// Free-spin, not loaded, on purpose: this isolates back-EMF from load torque. A loaded run
// would fold in rolling resistance and give a gain that is wrong the moment the load changes.
void emfTestFit(const char *label, const float *v, const int *p, int n) {
  // Fit only points that actually turned; a stalled step is not on the line.
  float sv = 0, sp = 0, svp = 0, svv = 0; int m = 0;
  for (int i = 0; i < n; i++) {
    if (v[i] < 0.05f) continue;
    sv += v[i]; sp += (float)p[i]; svp += v[i] * (float)p[i]; svv += v[i] * v[i]; m++;
  }
  Serial.print("  "); Serial.print(label); Serial.print("  ");
  if (m < 3) { Serial.println("too few moving points to fit"); return; }
  float den = m * svv - sv * sv;
  if (fabs(den) < 1e-6f) { Serial.println("degenerate fit"); return; }
  float k = (m * svp - sv * sp) / den;
  float b = (sp - k * sv) / m;
  Serial.print("K_EMF "); Serial.print(k, 1); Serial.print(" PWM per rev/s");
  Serial.print("   intercept "); Serial.print(b, 1);
  Serial.print("   n "); Serial.print(m);
  Serial.println(b > 4.0f && b < 20.0f ? "   (intercept ~ moving deadband: model looks sane)"
                                       : "   <-- intercept is NOT near the moving floor, be suspicious");
}

void emfTestLoop() {
  static bool  seeded = false, finished = false, sampling = false;
  static int   pwm = 0, idx = 0;
  static unsigned long phaseStart = 0;
  static long  encL0 = 0, encR0 = 0;
  static int   pwmRec[EMF_TEST_MAX_STEPS];
  static float velL[EMF_TEST_MAX_STEPS], velR[EMF_TEST_MAX_STEPS];

  if (finished) { motorRaw(0); return; }
  if (!seeded) {
    pwm = EMF_TEST_PWM_START;
    seeded = true;
    sampling = false;
    phaseStart = millis();
    motorRaw(EMF_TEST_SIGN * pwm);
    Serial.println("EMF_TEST  pwm   revL     revR    (settling first step)");
  }

  if (!sampling) {
    if (millis() - phaseStart < EMF_SETTLE_MS) return;   // let the speed settle
    readEncoders(encL0, encR0);                          // then start the measurement window
    sampling   = true;
    phaseStart = millis();
    return;
  }
  if (millis() - phaseStart < EMF_SAMPLE_MS) return;

  long l, r; readEncoders(l, r);
  const float dt = (float)(millis() - phaseStart) / 1000.0f;
  const float vl = fabs((float)(l - encL0)) / (float)ENC_COUNTS_PER_REV / dt;
  const float vr = fabs((float)(r - encR0)) / (float)ENC_COUNTS_PER_REV / dt;

  if (idx < EMF_TEST_MAX_STEPS) {
    pwmRec[idx] = pwm; velL[idx] = vl; velR[idx] = vr;
    Serial.print("EMF_TEST  ");   Serial.print(pwm);
    Serial.print("   ");          Serial.print(vl, 3);
    Serial.print("   ");          Serial.print(vr, 3);
    // Incremental slope against the previous step: constant = linear = a single gain works.
    if (idx > 0) {
      float dvl = vl - velL[idx - 1], dvr = vr - velR[idx - 1];
      float dp  = (float)(pwm - pwmRec[idx - 1]);
      Serial.print("   dPWM/dv L ");
      if (dvl > 0.01f) Serial.print(dp / dvl, 1); else Serial.print("--");
      Serial.print("  R ");
      if (dvr > 0.01f) Serial.print(dp / dvr, 1); else Serial.print("--");
    }
    Serial.println();
    idx++;
  }

  pwm += EMF_TEST_PWM_STEP;
  if (pwm > EMF_TEST_PWM_MAX || idx >= EMF_TEST_MAX_STEPS) {
    motorRaw(0);
    Serial.println();
    Serial.println("==== EMF_TEST FIT ====");
    emfTestFit("LEFT ", velL, pwmRec, idx);
    emfTestFit("RIGHT", velR, pwmRec, idx);
    Serial.println("Use the smaller K_EMF of the two if they differ; an over-large");
    Serial.println("feedforward drives the loop, an under-large one just leaves some");
    Serial.println("of the old standing offset behind.");
    Serial.println("======================");
    finished = true;
    return;
  }
  sampling   = false;
  phaseStart = millis();
  motorRaw(EMF_TEST_SIGN * pwm);
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
#elif LEAN_SWEEP
  leanSweepLoop();     // motors off; push slowly to each stop, reports only HELD angles
  return;
#elif EMF_TEST
  emfTestLoop();       // WHEELS OFF THE GROUND; PWM staircase, logs steady speed per step
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
    plainFloorFilt = (float)RIGHT_DEADBAND_STATIC;                   // ditto, no boost here
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
    controlLog.emfFFL = controlLog.emfFFR = 0;
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
  float cap       = 0.35f + 0.65f * speedIn;                          // CH3 speed cap 0.35..1.0
  float driveRaw  = crsf::drive();
  float turnRaw   = crsf::turn();
  float driveIn   = driveRaw;
  float turnIn    = turnRaw;
  if (fabs(driveIn) < DRIVE_STICK_DEADZONE) driveIn = 0.0f;
  if (fabs(turnIn) < TURN_STICK_DEADZONE) turnIn = 0.0f;
  // DRIVE_MIN_VEL floor: the stick's live range maps to MIN..MAX rather than 0..MAX, so the
  // first usable stick position asks for a speed the robot can actually break away at
  // instead of a crawl inside the dither band. driveIn is already zeroed inside the stick
  // deadzone above, so this cannot creep at neutral.
  float targetVel = 0.0f;
  if (driveIn != 0.0f) {
    float live = (fabs(driveIn) - DRIVE_STICK_DEADZONE) / (1.0f - DRIVE_STICK_DEADZONE);
    live = constrain(live, 0.0f, 1.0f);
    float speed = DRIVE_MIN_VEL + live * (DRIVE_MAX_VEL - DRIVE_MIN_VEL);
    targetVel = DRIVE_SIGN * (driveIn > 0.0f ? 1.0f : -1.0f) * cap * speed;  // rev/s, + = fwd
  }
  // TURN SCALED BY THROTTLE. Turn is an open-loop PWM-effort differential -- it has no
  // velocity feedback, no rate limit and nothing measuring yaw -- while drive has to
  // negotiate with the balance loop and the lean ramp before it produces any motion. That
  // asymmetry is why turns have always felt faster than the driving they are mixed into,
  // and dropping TURN_AUTHORITY 25 -> 16 reduced it without changing its nature.
  // Scaling by the live throttle ties rotation to translation: hard turns need the stick.
  // TURN_THROTTLE_FLOOR keeps a fraction available at zero throttle so the robot can still
  // pivot in place; set it to 0 to forbid stationary turns entirely.
  float turnScale = TURN_THROTTLE_FLOOR
                  + (1.0f - TURN_THROTTLE_FLOOR) * fabs(driveIn);
  if (turnScale > 1.0f) turnScale = 1.0f;
  float turnCmd   = TURN_SIGN * turnIn * cap * turnScale * TURN_AUTHORITY;  // per-wheel PWM diff
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
  // Per-direction, at the measured frame contacts. See FALL_CUTOFF_FWD_DEG.
  if (!fallen && (error > FALL_CUTOFF_FWD_DEG || error < -FALL_CUTOFF_REAR_DEG)) fallen = true;
  // Bounded stand-up attempt: below the saturation latch's reach the PD would otherwise
  // push at ~60 PWM forever against whatever it is lying on. See RECOVERY_GIVEUP_DEG.
  // GATES ON TRACKING ERROR, NOT LEAN. It used the `error` above, which is the LEAN from the
  // balance point -- so any COMMANDED lean past 32 held for 2 s latched the robot as fallen.
  // Harmless while LEAN_CLAMP was 18, fatal the moment the lean is released to the pilot:
  // full stick would cut the motors mid-manoeuvre. What this check is for is "the robot
  // cannot reach the angle it is trying to hold", and that is tracking error. leanCmd is the
  // previous tick's value here (it is computed further down); one tick of lag is nothing
  // against a 2 s timeout.
  if (!fallen && fabs(pitch - (BALANCE_SETPOINT + leanCmd)) > RECOVERY_GIVEUP_DEG) {
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
  if (!driving) {
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
  // ══ LEAN CLAMP REMOVED 2026-08-15, ON PILOT'S CALL: "remove lean clamp. we don't need it." ══
  // No outer-loop lean cap, and no conditional-integration freeze keyed to one -- both existed
  // to stop the accumulators winding past a ceiling that no longer exists.
  //
  // WHAT STILL BOUNDS THE LEAN, because "no clamp" is not "unbounded": each term carries its
  // own cap and their sum is the real ceiling --
  //     Kpos * POS_ERROR_CLAMP = 6.0 * 1.5 = 9.0
  //     Kveff * velError                   ~ 2.0
  //     VEL_I_CLAMP                        = 14.0
  //                                          ----
  //                                          ~25 deg reachable
  // The rear cap of 18 WAS binding (a flight shows RAW -18.00 = CMD -18.00), so this buys
  // about 7 deg more rearward. It does NOT buy motion: 35 deg was flown and did not break the
  // wheels loose. This removes an artificial limit, nothing more.
  //
  // Anti-windup never depended on this clamp. POS_ERROR_CLAMP bounds the position term and
  // VEL_I_CLAMP bounds the integral, both directly, and the breakaway edge detector still
  // discards the integral on the stall->rolling transition. The freeze was a second layer
  // over accumulators that were already individually bounded.
  // Safety does not live here either: FALL_CUTOFF (63 fwd / 47 rear, at the measured frame)
  // and RECOVERY_GIVEUP still apply, and ~25 deg is far inside both.
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
  // the inner loop is pinned -- but pinned is not the same as LOSING, and effort alone
  // cannot tell them apart because |u| ~ Kpeff * |error| and Kpeff doubles while driving.
  // Require the angle to be hopeless too; see the note at SAT_TRIP_DEG for the arithmetic.
  const bool effortPinned  = fabs(u)     > SAT_EFFORT_FRAC * (float)(MAX_PWM - MAX_DEADBAND);
  const bool angleHopeless = fabs(error) > SAT_TRIP_DEG;
  if (effortPinned && angleHopeless) satTicks++;
  else                               satTicks = 0;
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

  controlLog.effortL = u + turnCmd;
  controlLog.effortR = u - turnCmd;
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
  Serial.print(" FALLDEG "); Serial.print(FALL_CUTOFF_FWD_DEG, 0);
  Serial.print("/");         Serial.print(FALL_CUTOFF_REAR_DEG, 0);
  Serial.print(" GBIAS "); Serial.print(gyroYBias, 2);
  // DBF is the floor ACTUALLY IN FORCE this tick -- the blended, filtered value both wheels
  // now share. The old DBL/DBR printed what the moving/static GATE would have chosen, which
  // is exactly the chattering quantity that was removed; printing it back would re-import the
  // thing the fix exists to kill. MV keeps the raw wheelMoving flags visible (L then R) so the
  // gate's behaviour can still be watched -- it is no longer wired to the output.
  Serial.print(" EMF "); Serial.print(controlLog.emfFFL); Serial.print("/"); Serial.print(controlLog.emfFFR);
  Serial.print(" DBF "); Serial.print(plainFloorFilt, 1);
  Serial.print(" MV "); Serial.print(wheelMovingL ? "M" : "S");
  Serial.print(wheelMovingR ? "M" : "S");
  Serial.print(" PWMHZ "); Serial.print(MOTOR_PWM_HZ);
  Serial.print(" SET "); Serial.println(sweepIdx + 1);
  controlLog.encoderDeltaL = 0;
  controlLog.encoderDeltaR = 0;
}
