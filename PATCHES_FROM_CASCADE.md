# Salvage list: `cascade-setpoint-drive` -> `v2-drive-works`

`balance_v2/balance_v2.ino` was reset to tag **`v2-drive-works`** (`a20285b`, 2026-08-14) after
the cascade branch reached a wall: the robot leaned correctly but would not translate at any
throttle value, and 33 commits of tuning had not moved that.

Nothing below is lost — every patch is recoverable with
`git show <sha>:balance_v2/balance_v2.ino` or `git cherry-pick <sha>`. The branch is intact.

**Apply one at a time, fly between each.** Most of the damage on that branch came from stacking
changes and then reasoning about the result from a single log.

---

## Tier 1 — apply these, they are hardware truth or fix a real defect

### 1.1 Deadbands: `11/13` moving, `15/27` static -> `10/10` and `18/18`
`9741e6c`, confirmed `9dadfac` / `4270999`

The tag's numbers were measured while **one BTS7960 had a wiring fault on its enable pin**, so
one bridge braked at idle and the other coasted. The 15-vs-27 asymmetry was never in the
machine. The pilot repaired the wiring; re-measured across four loaded runs and roughly 45
pooled samples the wheels are symmetric: **10/10 moving, 18/18 static**.

This also retires the "right wheel has higher friction" story entirely. Note `MAX_DEADBAND`
drops 27 -> 18, which moves the saturation-latch ceiling.

**Highest confidence item here.** The tag is running numbers taken from a broken bridge.

### 1.2 `mapEffortToPwm` — continuous floor instead of a chattering gate
`d17f3e2`

The tag selects its floor with `moving ? dbMoving : dbStatic`, evaluated every tick. With the
wheels barely turning, `wheelMoving` toggles at encoder-noise level and square-waves the floor
by 8 counts, per wheel, independently. Consecutive telemetry samples from the 2026-08-15
flight:

```
DBL 10M DBR 10M  ->  DBL 18S DBR 18S  ->  DBL 10M DBR 18S  ->  DBL 18S DBR 10M
```

The fix replaces the binary gate with `plainFloorFilt`, a low-passed blend of the static and
moving figures driven by actual speed. It measurably killed an ~8.7 Hz limit cycle: post-fix
bursts held one PWM sign for tens of samples with `RATE` +/-3 where it had been +/-25..30.

Note the continuous-blend machinery already existed in the tag but only the feedforward branch
used it, so with `DRIVE_FRICTION_FF 0` the chattering gate was the live path.

### 1.3 `deadbandDetect` — reject the backlash twitch
part of `a06ce6c`

The tag's `DEADBAND_TEST` declares breakaway on `cumulative > 5` counts, which false-fires on a
single backlash twitch. Observed: `dL` reached 6 at PWM 11 and sat at *exactly* 6 through PWM 15
before really breaking at 16 — a 5-count error in a number everything else is sized from.
`deadbandDetect` requires a second advance to confirm.

Test-harness only, no flight risk.

### 1.4 Multi-pass `DEADBAND_TEST` harness with a summary table
`a06ce6c`

Runs `DEADBAND_TEST_PASSES` ramps per flash with the sign alternating and a coast-down between,
then prints per-direction spreads. Replaces single-ramp measurement, which is what produced
several retracted findings on this branch (a "per-direction deadband difference" that vanished
once each wheel's own spread was measured, and a "tether inflates the loaded numbers" trend
fitted to three run medians and killed by a fourth).

Test-harness only. Worth taking before any future deadband work.

---

## Tier 2 — the pilot asked for these; re-apply on request

### 2.1 `DRIVE_MIN_VEL = 0.30` — throttle lower bound
part of `ae7a7f7`

Requested: "give more rpm to the movement", "reduce the range", "increase the lower bound".
Puts a floor under the commanded velocity just outside the stick deadzone so the slow end is
not in the dither region. Span becomes 0.30..0.65 instead of 0..0.40.

**Do NOT also raise `DRIVE_MAX_VEL`.** Raising the top end widens the range, which is the
opposite of what was asked, and on this branch the larger velocity demand pinned the outer loop
against `LEAN_CLAMP` and cost three commits of misdirected lean-clamp work.

### 2.2 `TURN_AUTHORITY = 25 -> 16`
part of `ae7a7f7`

Requested: "turns are still faster". One constant, no coupling.

### 2.3 Split `LEAN_CLAMP` into `FWD` / `REAR`
`c243546` for the split mechanics; pick the values fresh

The tag has a single symmetric clamp of 18. The chassis is not symmetric and neither is the
setpoint: `BALANCE_SETPOINT -3.22` shifts every target rearward, so the same clamp number buys
3.2 deg less forward reach and 3.2 deg more rear. Pilot's measurements: forward ~32 deg, rear
20+ deg.

Take the *mechanism*, not this branch's numbers — they were sized three times against figures
that never reproduced. See the warning in Tier 4.

### 2.4 `FALL_REARM_DEG = 8 -> 12`
`8447a23`

8 deg was too tight to clear a latched fall by hand. Trivial and independent.

---

## Tier 3 — coupled pair, take both or neither

### 3.1 `MOTOR_PWM_HZ = 4482 -> 2000` **plus** the rescaled deadbands
`38174a0`, `0a1b496`, `674ed2d`

Adopted from a three-point frequency sweep. The deadband figures in Tier 1.1 were measured
**at 2000 Hz**, so the frequency and the floor values belong together — applying 18/18 while
still running 4482 Hz means using floors measured under different conditions.

If Tier 1.1 goes in, this should go with it.

---

## Tier 4 — do NOT re-apply

### 4.1 `DRIVE_FRICTION_FF` in any form
`2a05434`, reverted `1d7de41`, analysed `f1b3eee`

Tried twice, failed twice, and the second attempt failed *harder* than the first.

- The two-phase version (LAUNCH aimed at low-passed `u`, CRUISE aimed at `velError`) cannot be
  gated: the phases push in opposite directions, so any misclassification is a full-amplitude
  PWM reversal. Every gate tried chattered on a ringing `LEAN ACT`.
- The single-signed rebuild (LAUNCH deleted, `velError` only) cancelled against the PD:
  `cmd = 38*(-1.00) + 49.43 = 11`. `LEAN ACT` froze at 11.7 while `LEAN CMD` ran to 24.00 and
  `POS` did not move for four seconds.

**Both aiming signals are now eliminated by experiment:**
- `velError` is wrong at **all** times, not just at launch. Establishing a lean is
  non-minimum-phase — the wheels must roll *backward* to tilt the body *forward* — so a
  `velError`-aimed bias opposes exactly the motion that builds the lean. There is no phase
  boundary to gate this away.
- `u` carries the right intent but has **no usable DC**: across a stalled dither it runs
  `-5.08 +8.21 -5.46 +6.16 -5.57 +7.43 ...`, averaging about **+0.2**.

A sustained-demand feedforward needs a third signal or a different mechanism. Do not spend
another flight on these two.

### 4.2 `TARGET_VEL_SLEW` / `COAST_BRAKE` / `TURN_HEADROOM_LIMIT` / `TURN_SPEED_FADE`
`8d1a173`, `26ba159`, `6e8845a`

Added during the period when drive was already broken, so none of them has ever been observed
on a robot that moves. `TARGET_VEL_SLEW` and the floor LPF together put two lags on the launch
that the working build does not have. Re-introduce only with a specific reason and one at a time.

### 4.3 This branch's lean-clamp values
`92711f4`, `2878e41`, `b1526a7`, `c243546`

Sized four times against three mutually inconsistent rear figures (~-19 in the source, -15.7 in
the 2026-06 notes, -21.0 from a disarmed release settle) and changed in both directions. **A
rest angle reached by releasing the chassis is not a limit** — that is the same class of
measurement that produced the discredited -15.67 sit-down. A limit is where the chassis stops
under a slow deliberate push *and reproduces across sweeps*.

---

## Still open in the tag (pre-existing, not caused by this branch)

**Saturation-latch gain bug.** The latch computes its trip angle at `Kp` 2.0 but the loop runs
at `Kpeff` 4.0 while driving, so it fires at roughly half the intended angle — 78.85 effort
reads as 39.4 deg at `Kp` 2.0 and is really 19.7 deg. A fixed effort limit standing in for an
angle limit, with the gain between them doubling. Untouched all session.

**`FLOOR_KNEE` is a trade-off, not a fix.** The tag's 5.0 is the right end of it. Lowering it
raises the zero-crossing gain (`floor/FLOOR_KNEE + 1`) and buys breakaway at the cost of a limit
cycle: 5.0 -> 4.6, 2.5 -> 8.2 (jitters), 1.0 -> 19.0 (pilot: "made it worse. To jittery, can't
break away"). Do not reach for this knob to fix breakaway; it trades one symptom for the other.
