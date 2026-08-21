# SHARP 2Y0A21 Collision Avoidance — Wiring & Configuration Spec

Status: **design spec, not yet built.** Nothing in `balance_v2.ino` implements
this yet.

## Scope

**One job: stop the robot before it drives into something.**

Not mapping, not SLAM, not a 3-D scan, not telemetry to a PC. Those were
considered and are out of scope — see [§13](#13-scope-note-what-this-is-not) for
why, briefly, so the ground already covered is not re-covered.

Narrowing to collision avoidance removes most of the hard parts. Because the
braking window is 45–70 cm rather than the sensor's full 10–80 cm range, and
because a wrong reading only ever costs speed rather than corrupting a map,
**four subsystems that a mapping build would need are provably unnecessary
here**: range calibration to a curve ([§9.3](#93-no-calibration-curve-two-points-and-a-reciprocal)),
runtime pitch compensation ([§4.3](#43-why-there-is-no-pitch-compensation)),
speed-scheduled sector selection ([§4.1](#41-the-sector-is-fixed-at-15)), and
any reduction of `DRIVE_MAX_VEL` ([§5.3](#53-the-approach-simulation)).

## Table of contents

1. [The part, and the angle](#1-the-part-and-the-angle)
2. [Why it needs a servo](#2-why-it-needs-a-servo)
3. [What "collision avoidance" means here](#3-what-collision-avoidance-means-here)
4. [Geometry: sector, corridor, pitch](#4-geometry-sector-corridor-pitch)
5. [Timing and the approach simulation](#5-timing-and-the-approach-simulation)
6. [Electrical spec](#6-electrical-spec)
7. [Pin budget and the two Teensy traps](#7-pin-budget-and-the-two-teensy-traps)
8. [Mechanical spec](#8-mechanical-spec)
9. [Firmware spec](#9-firmware-spec)
10. [Integration with the cascade](#10-integration-with-the-cascade)
11. [Bench procedures and acceptance tests](#11-bench-procedures-and-acceptance-tests)
12. [Limitations](#12-limitations)
13. [Scope note: what this is not](#13-scope-note-what-this-is-not)

---

## 1. The part, and the angle

**It is not a lidar.** No time of flight, no laser, no scanning. An infrared LED
throws a spot, a position-sensitive detector ~19.5 mm to one side images it, and
the spot's position gives the range by similar triangles. One number per
measurement, no bearing information, ~26 Hz, and a non-monotone output curve.

| | |
|---|---|
| **Part** | Sharp `GP2Y0A21YK0F` (body marking `2Y0A21`) |
| **Rated range** | 10–80 cm |
| **Supply** | 4.5–5.5 V, ~30 mA average |
| **Output** | ~2.3 V @ 10 cm → ~0.4 V @ 80 cm; **non-monotone below 10 cm** |
| **Update period** | 38.3 ms ± 9.6 ms |
| **Connector** | JST PH 3-pin: pin 1 `Vo`, pin 2 `GND`, pin 3 `Vcc` |

**"F25" is a lot code, not a variant.** The datasheet's marking example is
`2Y0A21 F 4Z` — model number, a fixed `F`, then the production lot. There is no
`-F25` ordering part number. It is an ordinary `GP2Y0A21YK0F`.

### 1.1 The angle

**Sharp does not publish one.** The datasheet has range, output curve, timing
and outline dimensions, and no angular characteristic at all. Anyone quoting an
exact FOV is quoting a measurement.

The optics bound it. The emitted spot is ø5–7 cm at 80 cm, and the *effective*
beam is narrower still because a target must be inside the emitter cone **and**
imaged on the PSD:

```
alpha = 2*atan(w / (2*d))
  w = 5 cm @ 80 cm -> 3.6 deg
  w = 7 cm @ 80 cm -> 5.0 deg
```

> **Working assumption: full effective beam angle 4–5°, i.e. ±2–2.5°.**
> [§11.2](#112-beam_test--measure-the-angle) measures it on your unit. Use 5° as
> the pessimistic case for coverage and 4° for step-size, until then.

Spot diameter at range, at 5°:

| Range | 20 cm | 45 cm | 70 cm | 80 cm |
|---|---|---|---|---|
| Spot ø | 1.7 cm | 3.9 cm | 6.1 cm | 7.0 cm |

---

## 2. Why it needs a servo

The robot's collision corridor is its own width. Take the half-width as
**10 cm** (track plus margin — measure yours). The half-angle that must be
covered at range `d` is `atan(w/d)`. A rigidly-mounted sensor covers ±2.5°:

| Range `d` | 20 cm | 30 cm | 45 cm | 70 cm | 80 cm |
|---|---|---|---|---|---|
| Required half-angle | 26.6° | 18.4° | 12.5° | 8.1° | 7.1° |
| **Corridor fraction seen by a fixed sensor** | **9 %** | **13 %** | **20 %** | **31 %** | **35 %** |

Read the bottom row twice. A fixed 2Y0A21 pointed dead ahead sees **under a
fifth of the corridor across the entire braking window**, and coverage gets
*worse* as the obstacle gets closer — backwards from what collision avoidance
needs. A table leg, a door frame, a chair foot: all of them sit 10–20 cm
off-axis at the range where there was still time to stop, and all of them are
invisible.

Sweeping the same beam across the corridor sees all of it. That is the entire
argument, and it is a geometric fact about a 5° beam rather than a preference.

**Step size must be ≤ the beam width** or the scan has holes: a 7.5° step with a
5° beam leaves a 2.5° gap, which at 70 cm is a 3 cm blind stripe — wide enough
to swallow a chair leg. **5° steps**, reduced if §11.2 measures the beam
narrower.

### 2.1 Why the servo axis is yaw

The usual reflex is "yaw so the reaction torque misses the balance loop." Check
the number first. Sensor plus bracket ~2e-6 kg·m², stepping 5° in 10 ms:

```
alpha_ang = 4*theta/t^2 = 4*0.0873/(0.010)^2 = 3492 rad/s^2
torque    = J*alpha_ang = 2e-6 * 3492         = 7.0 mN*m
```

Against the robot's restoring torque of `m*g*l*sin(1°)` ≈ 38.5 mN·m per degree
of lean, that is **0.18° of equivalent lean for 10 ms**. Reaction torque is a
non-argument on either axis.

Yaw wins on information instead: **azimuth is the axis nothing on this robot
measures**, and it is the axis the coverage table above is about. Pitch is
already known every 5 ms from the complementary filter — and as
[§4.3](#43-why-there-is-no-pitch-compensation) shows, within the collision-
avoidance window it needs no correction at all.

*(The genuine coupling path from the servo into the balance loop is electrical,
not mechanical — [§6.2](#62-supply-and-decoupling).)*

---

## 3. What "collision avoidance" means here

Precisely one behaviour:

> **The scanner may reduce the pilot's forward velocity setpoint, down to zero.
> Nothing else.**

It does not steer, does not reverse, does not command lean, does not touch PWM,
and has no authority over turn. The pilot keeps the stick, the arm switch, and
reverse at all times.

Braking comes free from the existing cascade. With `targetVel` forced toward
zero while the robot rolls forward, `velError = targetVel - forwardVel` goes
negative, `velocityLeanI` winds negative, the outer loop derives a rearward
lean, and the inner PD chases it. `posSetpoint += targetVel*DT` also stops
advancing, so position hold freezes the target where the robot is. **No new
control mechanism is needed and none should be added** —
[§10](#10-integration-with-the-cascade) is the one rule that keeps it that way.

---

## 4. Geometry: sector, corridor, pitch

### 4.1 The sector is fixed at ±15°

An earlier draft speed-scheduled the sector (wide when slow, narrow at speed,
with a hysteresis pair on `forwardVel`). Collision avoidance does not need it.
The braking window is 45–70 cm, and the corridor half-angle over that window is
8.1°–12.5°. A single **±15° sector at 5° steps — 7 bearings** covers it with
margin at every range in the window, plus close-in coverage below 40 cm.

Fixed sector means: no schedule, no hysteresis pair, no mid-sweep sector change,
no `forwardVel` threshold to chatter on. One constant.

### 4.2 The corridor test, precomputed

A return at bearing `az` and range `x` is only a threat if the robot will
actually hit it: `|x·sin(az)| < 10 cm`. Rearranged, each bearing has a fixed
maximum in-corridor range — and since range is monotone in raw ADC counts across
10–80 cm, **that becomes one integer compare per bearing with no runtime trig**:

| Bearing | In corridor only closer than | Raw-count threshold |
|---|---|---|
| 0° | always | — |
| ±5° | 115 cm (beyond range) | — always in corridor |
| ±10° | 57.6 cm | `N ≥ 402` |
| ±15° | 38.6 cm | `N ≥ 577` |

Without this, the robot brakes for a door frame it would pass cleanly. With it,
seven constants and seven compares.

### 4.3 Why there is no pitch compensation

This is the subsystem that narrowing the scope deletes, and the argument is
worth following because it is the reason the whole design gets simple.

A sensor at height `h` pitched nose-down by `theta` puts the lower edge of its
beam into the floor at `d_floor = h / sin(theta + alpha/2)`. A mapping build
cares about this out to 80 cm; collision avoidance only cares out to 70 cm, and
only *stops* inside 45 cm:

| Lens height `h` | Floor enters the 70 cm taper at | Floor reaches the 45 cm stop at |
|---|---|---|
| 0.30 m | 22.9° nose-down | 39.3° |
| **0.35 m** | **27.5° nose-down** | **48.6°** |
| 0.40 m | 32.3° nose-down | 60.2° |

Reachable commanded lean on this robot is set by `VEL_I_CLAMP` = 20° (the
dominant term), with the `Kpos` term adding up to 9° and `Kvel` up to ~2.7° in
the worst transient. So at **h = 0.35 m the floor stays out of the taper through
normal driving**, and cannot reach the stop threshold below 48.6° — an attitude
the robot only sees on the way down.

And the residual case is **self-extinguishing**, which is the part that makes
compensation unnecessary rather than merely rare:

```
hard acceleration -> big nose-down lean -> floor return enters the taper
-> cap reduces targetVel -> less acceleration demanded
-> velocityLeanI unwinds -> lean reduces -> floor return disappears
```

A floor-induced slow-down removes its own cause. It costs a little speed during
an aggressive launch and nothing else. Compare that against the cost of the
alternative: `MOUNT_HEIGHT_M`, `RANGE_PITCH_OFFSET_DEG`, per-sample pitch
tagging, two trig calls per point, a height-above-floor rejection test, and a
bench procedure to calibrate the optical axis against `BALANCE_SETPOINT`.

> **Requirement: h ≥ 0.35 m, and no downward tilt bias, ever.** In exchange,
> pitch does not appear in the firmware at all.

**Nose-up is a limitation, not a bug to fix.** Braking pitches the nose up and
the beam rides above the target — at 15° up and 45 cm the beam is looking 12 cm
high. It cannot be compensated away (you cannot invent a return you did not
get), and [§5.3](#53-the-approach-simulation) shows the rate-limited cap
recovery already contains it. It is recorded in
[§12](#12-limitations), not engineered around.

Note also that this only bites in transients: per the README, *constant speed
needs zero lean*. At steady drive speed the sensor is level.

---

## 5. Timing and the approach simulation

### 5.1 The 38.3 ms cycle forbids a continuous sweep

The sensor integrates over its whole 38.3 ms measurement window. If the beam
moves during that window, the returned range is a blend over whatever it swept
across, belonging to no particular bearing. Even a leisurely 90°/s continuous
sweep moves the beam **3.4° per measurement** — comparable to the entire beam
width. The scan would be angularly meaningless.

So the sweep is **step-and-dwell**, and the dwell is set by the sensor, not the
servo:

```
dwell = servo travel (5 deg @ 0.12 s/60 deg)  ~10 ms
      + mechanical settle                      ~20 ms
      + one full 38.3 ms measurement cycle    >=38 ms
      ~= 90 ms per bearing, sampled over the last 60 ms
```

### 5.2 Sweep period

7 bearings × 90 ms = 0.63 s, plus a flyback. The scan is **unidirectional**:
a bidirectional sweep approaches each bearing from opposite sides on alternate
passes, so the two passes are offset by the full gear backlash (1–2° on SG90-
class trains) and one narrow object appears at two bearings on alternate sweeps
— an artefact that looks exactly like a real object flickering in and out. Fly
back to the start at full servo speed and discard everything sampled during it.

```
sweep 7 x 0.090 = 0.63 s  +  flyback ~0.16 s  =  0.79 s  (1.27 Hz)
```

At `DRIVE_MAX_VEL` 0.90 rev/s on 140 mm wheels (0.396 m/s) the robot covers
**31 cm between two looks at the same bearing.** That is the number the stop
thresholds have to absorb.

### 5.3 The approach simulation

The thresholds were not guessed. A 1-D approach was simulated: sensor updates a
bearing once per 0.79 s sweep, cap tapers linearly between `RANGE_STOP` and
`RANGE_CLEAR`, robot velocity follows the cap through a 0.30 s first-order lag
(lean build plus the non-minimum-phase reversal) with deceleration limited to
`g·tan(20°)` = 3.57 m/s², nothing visible beyond 80 cm. Worst sweep phase of
eight.

With **`RANGE_STOP` = 0.45 m, `RANGE_CLEAR` = 0.70 m**:

| Scenario | @ 0.90 rev/s | @ 0.55 | @ 0.45 |
|---|---|---|---|
| Nominal approach | 27.5 cm clear | 39.4 | 42.6 |
| Slow lean build (τ = 0.5 s) | 22.6 cm clear | 35.2 | — |
| Obstacle revealed only at 60 cm (rounding a corner) | 30.5 cm clear | 40.0 | 42.6 |
| Both stresses together | 22.6 cm clear | 35.2 | 39.3 |
| Beam rides over target while braking (nose-up blindness) | 30.5 cm clear | 40.0 | — |

Two conclusions, both of which simplify the build:

> **`DRIVE_MAX_VEL` does not need reducing.** Even at the full 0.90 rev/s and
> under both stresses at once, the robot stops with 22 cm to spare. This is
> the taper doing its job: because the cap starts falling at 70 cm rather than
> hard-stopping at 45 cm, speed is already down before the deadline, and every
> latency term is proportional to speed. One fewer knob to tune, and one fewer
> way for the feature to change how the robot drives when nothing is in front
> of it.

> **The rate-limited cap recovery is load-bearing, not decoration.** The
> nose-up blindness row is the case where the robot brakes, pitches up, loses
> the return, and would re-accelerate into the obstacle. Instant-drop /
> slow-recover contains it on its own; the sim shows no benefit from adding a
> pitch interlock on top, so there isn't one.

Both results come from the taper being **continuous**. A binary stop/go gate on
a noisy signal is the bug class this repo has hit three times already (the
friction floor, the integral rolling detector, the `driving` flag) — and the
friction floor was fixed by exactly this move, from a binary gate to a
continuous blend.

---

## 6. Electrical spec

### 6.1 Grounding — read this before anything else

The ADC measures `Vo` against the Teensy's analog ground. Any voltage developed
across a **shared ground return** by motor current appears directly as range
error, at 2 kHz, correlated with motor torque — which is correlated with
everything the robot does.

```
2 A of motor return through 0.05 ohm of shared ground = 100 mV of offset

curve sensitivity (from V = 33.9/(d + 4.74)):
  at 45 cm: 13.7 mV/cm  ->  100 mV = 7.3 cm of error
  at 70 cm:  6.1 mV/cm  ->  100 mV =  16 cm of error
```

16 cm of range error, modulated by motor effort, right across the braking
window.

> **E1: the sensor's ground returns to the logic/ADC star point on its own
> conductor, and never shares a conductor with motor return current.** The
> README's "all logic grounds must share a reference" is necessary but not
> sufficient — this is a *separate-conductor* requirement, not a common-node
> one.

Route `Vo` twisted with its own ground return, away from motor leads and the
servo lead. If it must cross a motor lead, cross at 90°.

### 6.2 Supply and decoupling

The sensor pulses its IR LED once per 38.3 ms cycle and draws a spike doing it.
The datasheet asks for **more than 10 µF** between `Vcc` and `GND`.

> **E2: 100 µF electrolytic ‖ 0.1 µF ceramic, at the sensor connector, not at
> the buck.**
>
> **E3: sensor and servo get separate feeds from the 5 V buck (star from the
> buck output). Do not daisy-chain the sensor off the servo's 5 V.**

E3 as a worst-case bound: a 250 mA servo transient across 0.1 Ω of shared 5 V
wire is a 25 mV supply dip, correlated with every scan step. Two extra wires
buys it away.

Budget: sensor ~30 mA average; servo ~10 mA idle, 100–250 mA slewing, ~700 mA
stalled. **The 5 V buck needs ≥1 A of headroom over its current load.**

### 6.3 The divider — the Teensy is not 5 V tolerant

The output is not bounded by its useful range: it *peaks* near 3.1 V around
4–6 cm, inside the fold-back region ([§9.4](#94-the-fold-back)). The Teensy
4.1's ADC reference and absolute pin maximum are both 3.3 V.

```
        Vo ----[ 6.8k ]----+---- Teensy A0 (pin 14)
                           |
                          --- 0.1uF
                          ---
                           |
        AGND ----[ 10k ]---+
```

* Ratio `10/16.8` = **0.5952**, Thevenin source impedance **4.05 kΩ**.
* Normal peak 3.1 V → 1.85 V at the pin. **Fault case (Vcc shorted to Vo,
  5.0 V) → 2.98 V, still inside the rail.** That headroom is why 0.60 and not a
  "tighter" 0.68.
* RC with the 0.1 µF: 0.41 ms — three orders under the 38.3 ms sensor cycle, so
  it filters noise without smearing the measurement.

Resolution cost, the objection worth checking rather than assuming. At 12 bits
over 3.3 V the braking window spans `N` = 503 (45 cm) to `N` = 335 (70 cm), and
the worst-case gradient is 4.5 counts/cm at 70 cm — against a sensor whose own
noise there is several centimetres. The divider costs nothing that matters.

### 6.4 Servo signal level

The Teensy drives 3.3 V logic. SG90/MG90S-class servos generally latch on it;
none of them specify it. The repo settled the equivalent question for the
BTS7960 by reading the datasheet (`V_IN(H)` max 2.0 V, absolute and not
ratiometric — the planned 74HCT244 was unnecessary); here there is no such
guarantee to read. **Wire it direct, and if the servo jitters or will not hold
position, add a 74HCT125 buffer on 5 V rather than tuning around it.** Jitter
from an unlatched input looks exactly like backlash.

### 6.5 Connector

Datasheet pinout: **pin 1 `Vo`, pin 2 `GND`, pin 3 `Vcc`**. The supplied JST-PH
pigtail is commonly red / black / white or red / black / yellow, and the
colour-to-pin mapping is not something to take on trust.

> **Verify with a meter before first power-up.** Continuity from pin 2 to the
> sensor ground identifies `GND`; power it on the bench with the third wire
> floating to confirm which pin reads ~5 V. Swapping `Vcc` and `Vo` puts 5 V
> into the divider — survivable at the 0.60 ratio specified above, and not
> survivable without it.

---

## 7. Pin budget and the two Teensy traps

Current allocation in `balance_v2.ino`:

| Pins | Function |
|---|---|
| 0, 1 | `Serial1` — CRSF / ELRS RX |
| 2, 3, 4, 5 | encoder A/B, left and right (A on interrupt) |
| 6, 9 | left motor FWD/REV — **FlexPWM2.2 A/B** |
| 18, 19 | I2C to the GY-91 (also A4/A5) |
| 22, 23 | right motor FWD/REV — **FlexPWM4.0/4.1** (also A8/A9) |

Additions:

| Pin | Function | Note |
|---|---|---|
| **14 (A0)** | sensor `Vo` via the 6.8k/10k divider | ADC only |
| **8** | servo signal | FlexPWM1_3_A — a different FlexPWM *module* from both motors |

### Trap 1 — `analogWriteResolution` is global; the servo cannot use hardware PWM

`balance_v2.ino:1498` sets `analogWriteResolution(MOTOR_PWM_BITS)` with
`MOTOR_PWM_BITS = 8`. On Teensy 4 that is **global to every `analogWrite`
channel** — the file's own comment already says so ("analogWrite range is
explicitly 0..255 on every channel").

```
analogWrite for the servo at the current 8-bit setting:
  50 Hz period 20000 us / 256   = 78.1 us per count
  servo band 1000..2000 us      = 12.8 counts for the ENTIRE travel
                                ~ 14 deg per count           -> unusable

raising the global resolution to 12 bits to fix that:
  every motor analogWrite(0..255) now runs against a 4095 full scale
  -> ~6 % duty ceiling, the robot does not stand up
  -> the measured deadband floors (10/10 static, 18/18 moving, RAW 8-bit
     counts at 2000 Hz) all become meaningless
```

> **Do not change `MOTOR_PWM_BITS`. Do not use `analogWrite` for the servo. Use
> the `Servo` library**, which on Teensy 4.x drives the pin from a hardware
> timer interrupt and does not touch `analogWrite` state. Command it with
> `writeMicroseconds()`, never `write(degrees)` — the degree mapping is a
> library convention, and [§11.3](#113-servo_cal--endpoints-and-scale) measures
> the real endpoints.

Avoid `PWMServo` here specifically *because* it uses the hardware PWM path.

### Trap 2 — FlexPWM submodules share a frequency

`analogWriteFrequency(pin, 50)` sets the frequency of the pin's whole FlexPWM
submodule. Land that on a submodule a motor uses and `MOTOR_PWM_HZ` silently
becomes 50 — and `MOTOR_PWM_HZ` is a control parameter here, matched to measured
deadband floors. Motors sit on FlexPWM2_2 and FlexPWM4_0/4_1; pin 8 is
FlexPWM1_3_A, a different module. **Re-check that against the current PJRC pin
table before soldering.**

> **Guard, whichever route you take: after the servo is wired and running,
> re-run `DEADBAND_TEST` and confirm the floors still measure 10/10 and 18/18.
> If they moved, the motor PWM frequency moved.**

### Timing

Nothing added may stretch the 5 ms tick (`CONTROL_PERIOD_US = 5000`).

* `analogReadResolution(12)` and `analogReadAveraging(16)` are unrelated to
  `analogWriteResolution` and safe; 16 averages costs tens of microseconds.
* `Servo::writeMicroseconds()` is a register write.
* **Never** call `attach()`/`detach()`, `delay()`, or any blocking wait from the
  control path. The scan is a state machine on the existing tick
  ([§9.5](#95-the-scan-state-machine)).
* Servo ISR load is 2 pin edges per 20 ms — negligible beside the encoder ISRs
  (~500 edges/s per wheel at full speed).

---

## 8. Mechanical spec

| Req | Statement | Why |
|---|---|---|
| **M1** | Lens height **h ≥ 0.35 m**, optical axis level, **no downward tilt bias** | §4.3 — this is what buys the deletion of pitch compensation |
| **M2** | Sensor rigidly mounted; no compliant standoffs | A wobbling sensor bearing is a wandering corridor |
| **M3** | Mast first bending mode ≥ 30 Hz, checked by tap test with telemetry running | A mast resonance also feeds the gyro and the balance loop |
| **M4** | Assembly on the pitch-axis centreline where possible; `BALANCE_SETPOINT` re-measured after mounting | 20 g at 3 cm off-centre shifts the balance point ~0.15°, and the setpoint is quoted to 0.01° |
| **M5** | Servo travel limited to ±60° mechanically **and** in firmware | Cable twist — §8.2 |
| **M6** | No external obstacle can reach within 10 cm of the lens (wheel envelope plus a standoff or bumper) | §9.4 fold-back — sub-10 cm targets read as *far*, the dangerous direction |

### 8.1 Mass — negligible, and pointing the right way

20 g (SG90 9 g + sensor 3.5 g + bracket) at 0.35 m on a ~1.5 kg robot with its
CoM at ~0.15 m — **substitute your real numbers, these are estimates**:

```
new CoM height = (1.5*0.15 + 0.020*0.35)/1.520 = 0.153 m   (+2.6 mm)
tau = sqrt(l/g):  0.1237 s -> 0.1250 s          (+1.0 %)
```

Raising the CoM *lengthens* the fall time constant, giving the loop more time.
The only real effect is on `BALANCE_SETPOINT` — hence M4.

### 8.2 Servo selection and mounting

Torque is a non-issue: §2.1 puts the peak accelerating torque at 7 mN·m
= 0.07 kg·cm against ~1.8 kg·cm for an SG90. Pick on the two things that
actually cost scan quality:

* **Backlash** — 1–2° on plastic gears, about half that on metal. The
  unidirectional sweep (§5.2) removes the artefact rather than tolerating it, so
  **an SG90 is adequate.**
* **Pulse deadband** — 5° is ~53 µs at 10.5 µs/deg. A servo with a deadband
  approaching that cannot resolve a step, whatever its speed rating.

Servo body fixed to the mast, horn carries the sensor, so the 3-wire tail twists
with every sweep: limit travel to ±60° (≤120° total twist), silicone wire, a
service loop above the rotation axis, strain relief at **both** ends. A cable
that stiffens the sweep changes the settle time, and settle time is a term in
the dwell budget — re-check with `SWEEP_TEST` after any re-route.

---

## 9. Firmware spec

### 9.1 Constants

Behind one master switch, so it compiles out entirely until proven — matching
the file's existing `ENCODER_TEST` / `DEADBAND_TEST` / `EMF_TEST` convention.

```c
// ---- SHARP 2Y0A21 collision avoidance --------------------------------------
#define RANGE_AVOID  0    // master enable. 0 = not compiled in at all.
#define RANGE_TEST   0    // 11.1 : motors off, stream raw counts
#define BEAM_TEST    0    // 11.2 : servo parked, find the beam edges
#define SERVO_CAL    0    // 11.3 : drive raw microseconds from Serial
#define SWEEP_TEST   0    // 11.4 : motors off, dump one sweep per line

#define RANGE_ADC_PIN  14      // A0, behind the 6.8k/10k divider (6.3)
#define SERVO_PIN       8      // FlexPWM1_3_A - not a motor submodule (7)

// Sector (4.1). Fixed. No speed schedule, no hysteresis pair.
const int   SCAN_POINTS      = 7;
const float SCAN_STEP_DEG    = 5.0f;    // <= measured beam width (11.2)
const float SCAN_SECTOR_DEG  = 15.0f;   // +/-

// Servo, in MICROSECONDS. MEASURED with SERVO_CAL (11.3) -- 1000/2000 and the
// Servo library's degree mapping are conventions, not facts about a servo.
const int   SERVO_US_CENTRE  = 1500;
const float SERVO_US_PER_DEG = 10.5f;
const float SERVO_LIMIT_DEG  = 60.0f;   // hard: cable twist (M5). Never exceed.

// Dwell budget (5.1), counted in 5 ms control ticks.
const int SCAN_SETTLE_TICKS = 6;    // 30 ms  servo travel + mechanical settle
const int SCAN_SAMPLE_TICKS = 12;   // 60 ms  covers one 38.3 ms sensor cycle

// EVERYTHING BELOW IS IN RAW 12-BIT ADC COUNTS. There is no distance in the
// control path -- see 9.3. Higher count = nearer.
// MEASURED with RANGE_TEST (11.1): a card at 45 cm and a card at 70 cm.
const int RANGE_N_STOP    =  503;  // seed: count at 45 cm  -> cap reaches 0
const int RANGE_N_CLEAR   =  335;  // seed: count at 70 cm  -> cap unrestricted
const int RANGE_N_BLOCKED = 1773;  // above this: nearer than 10 cm, FOLD-BACK
const int RANGE_N_NONE    =  200;  // below this: nothing in range
const int RANGE_N_SPREAD  =   60;  // sample spread that invalidates a dwell

// Corridor (4.2): minimum count for a bearing to be in the corridor at all.
// 0 = always in corridor. Index matches the bearing order below.
//                                     -15  -10   -5    0   +5  +10  +15
const int CORRIDOR_N_MIN[SCAN_POINTS] = {577, 402,   0,   0,   0, 402, 577};

const float RANGE_CAP_RECOVER = 1.0f;  // rev/s per second. The DROP is instant.
const unsigned long RANGE_POINT_AGE_MS = 2500;  // older than this = discarded
```

### 9.2 ADC configuration

```c
analogReadResolution(12);
analogReadAveraging(16);        // unrelated to analogWriteResolution; safe
pinMode(RANGE_ADC_PIN, INPUT);  // no pullup - it would fight the divider
```

Sample every control tick during the sample window: ~12 samples per dwell, which
is ~7–8 per 38.3 ms sensor cycle. The oversampling buys **noise rejection, not
freshness** — the sensor still produces a new number only every 38.3 ms.

### 9.3 No calibration curve: two points and a reciprocal

A mapping build needs `d` in metres and therefore a fitted curve. Collision
avoidance needs a *taper between two distances*, and that can be had without
ever computing a distance.

The sensor obeys `V ≈ A/(d + C)`, so raw count `N ∝ 1/(d + C)`, so **`u = 1/N` is
affine in distance.** A taper linear in `u` is therefore linear in distance —
with no `A`, no `C`, no curve fit, and no `pow()`:

```c
float u      = 1.0f / (float)nMax;              // nMax = nearest = highest count
float uStop  = 1.0f / (float)RANGE_N_STOP;
float uClear = 1.0f / (float)RANGE_N_CLEAR;
float frac   = constrain((u - uStop) / (uClear - uStop), 0.0f, 1.0f);
```

`frac` is 0 at the stop distance, 1 at the clear distance, and exactly linear in
metres in between. Calibration is **two bench readings** — a card at 45 cm, a
card at 70 cm, write down the two counts — instead of a five-point fit, and it
is calibrated at the two distances that actually matter rather than at the
datasheet's typicals.

Note also that "nearest obstacle" is just "**highest ADC count**", so the
reduction over the sweep is one integer `max`.

If you want centimetres in the telemetry for debugging, the same two points give
a linear fit for free — but keep it out of the control path:

```c
// a = (d2-d1)/(1/N2 - 1/N1) ; b = d1 - a/N1   -> with the seeds: 25075, -4.85
float d_cm = 25075.0f / (float)N - 4.85f;   // TELEMETRY ONLY
```

### 9.4 The fold-back

The output curve is not monotone: it rises from 0.4 V at 80 cm to ~2.3 V at
10 cm, peaks near 3.1 V at 4–6 cm, then **falls again**. A target at ~3 cm
produces roughly 1.0 V, which any distance formula reports as ~29 cm — an
obstacle pressed against the robot read as comfortably clear. This is the one
failure mode of this sensor that fails *toward* a collision.

Two defences, both required:

1. **`N > RANGE_N_BLOCKED` forces `frac = 0` (full stop), and is never converted
   to a distance.** That catches the rising flank of the peak.
2. **Requirement M6, mechanical:** the lens sits far enough behind the robot's
   frontmost surface that no external obstacle can physically reach within
   10 cm. This is what actually closes the hole — defence 1 cannot see past the
   peak.

### 9.5 The scan state machine

Driven by the existing 5 ms tick. No `delay()`, no blocking, no allocation.

```
PARK     -> disarmed / fallen / link down / scanner switch off:
            servo to 0 deg, no sampling, map invalidated, cap released.
            Entered from anywhere, immediately.

SETTLE   -> servo just commanded to the next bearing. Wait SCAN_SETTLE_TICKS.

SAMPLE   -> accumulate ADC samples for SCAN_SAMPLE_TICKS, then take the MEDIAN
            (not the mean: one commutation spike moves a mean and not a median).
            spread = max - min:
              spread > RANGE_N_SPREAD -> the window straddled a sensor update;
                                         mark INVALID, keep the previous value
                                         for this bearing with its age
              else                    -> store {N, millis()} for this bearing
            advance bearing; last one -> FLYBACK, else -> SETTLE.

FLYBACK  -> command the start bearing, discard everything, wait the flyback
            budget, then -> SETTLE.  (unidirectional, 5.2)
```

### 9.6 Reduction and the cap

```c
nMax = max{ N[i] : valid, N[i] >= CORRIDOR_N_MIN[i],
                   millis() - age[i] < RANGE_POINT_AGE_MS }
       (nothing qualifying -> nMax = 0 -> frac = 1, no restriction)

capRaw = DRIVE_MAX_VEL * frac;          // frac from 9.3, or 0 if BLOCKED (9.4)

if (capRaw < rangeVelCap) rangeVelCap = capRaw;                    // instant
else rangeVelCap = min(rangeVelCap + RANGE_CAP_RECOVER*DT,         // slow
                       DRIVE_MAX_VEL);
```

The asymmetry is the same shape as the drive release debounce already in the
file ("demand engages instantly, and it takes N consecutive ticks of no demand
to disengage"): safety engages now, releasing safety has to be sure. §5.3 shows
it is what contains the nose-up blindness case.

**Engagement is not debounced.** A validated point inside the stop threshold
acts on the tick it is produced, mid-sweep. Requiring confirmation across sweeps
would add 0.79 s to every stop and consume the whole margin in §5.3.

### 9.7 Telemetry

One block appended to the existing 100 ms line:

```
| AVOID <state> AZ <deg> N <raw> NMAX <raw> @<deg> CAP <rev/s> AGE <ms>
```

`N` and `NMAX` are raw counts on purpose. Every threshold, every calibration and
every noise investigation in §11 is done in counts, and a converted centimetre
figure hides a rail problem as a plausible-looking distance.

### 9.8 Safety and state

* **Disarm, fall, or link loss → `PARK` immediately**, on the same path that
  cuts the motors. Servo to 0°, map invalidated, `rangeVelCap` released.
* **Enabled by a spare TX16S channel, defaulting to OFF.** CH5/SA is the obvious
  candidate (CH6/SB is arm, CH7/SC is gain select, CH8/S1 is the tuning knob). A
  feature that can refuse the pilot's stick must be switchable off in flight
  without reflashing.
* **Never caps reverse.** There is no rear sensor, and the pilot must always be
  able to back out of whatever the scanner has decided about the space ahead.
* `RANGE_AVOID 0` compiles the whole thing out, including the hook in §10.

---

## 10. Integration with the cascade

> ### The one rule
>
> **The scanner may only reduce forward `targetVel`. It may never command a
> lean, never write `leanCmd` / `velocityLeanI` / `posSetpoint`, and never touch
> PWM.**

Not style. From the README: *lean is an output, not a command* — a lean angle is
an **acceleration** command, and every drive feature that has failed in this
repo failed by bypassing the cascade and commanding lean or PWM directly. A
scanner that "leans back to brake" would be the next entry on that list. §3
shows braking already falls out of the existing structure for free.

### 10.1 The trap: where the cap is applied

`balance_v2.ino:2500` reads:

```c
bool driveDemand = fabs(targetVel) > 0.001f;
```

and `driving` — with its release debounce — gates a full state wipe
(`posSetpoint = positionRev`, `velocityLeanI = 0`) plus the `DRIVE_KP` /
`DRIVE_KD` gain schedule.

So if the cap is applied *upstream* of that line, a scanner stop is
indistinguishable from the pilot letting go of the stick. After
`DRIVE_RELEASE_TICKS` the robot re-anchors `posSetpoint` and **zeroes
`velocityLeanI` in the middle of the braking manoeuvre — dumping precisely the
integral that was producing the brake lean** — and drops back to the
non-driving gains at the same moment.

The fix is placement, not logic. Keep the pilot's demand separate from the
commanded value:

```c
// existing line ~2455, renamed:
float pilotVel = 0.0f;
if (driveIn != 0.0f) { ... pilotVel = DRIVE_SIGN * ... ; }

float targetVel = pilotVel;
#if RANGE_AVOID
if (targetVel > 0.0f && targetVel > rangeVelCap) targetVel = rangeVelCap;
#endif

// ...and the drive state machine tests the PILOT, not the capped command:
bool   driveDemand    = fabs(pilotVel) > 0.001f;    // was fabs(targetVel)
int8_t driveDirection = pilotVel > 0.0f ? +1 : -1;  // was targetVel
```

Everything downstream of the state machine — `posSetpoint` integration,
`velError`, the burst log — keeps using `targetVel`, now the capped value.
`driving` stays true through a scanner stop, so the integral survives, the gain
schedule stays on `DRIVE_KP`/`DRIVE_KD` where it belongs mid-brake, and the
position target is not silently re-anchored.

### 10.2 What must not change

* No change to `DRIVE_MAX_VEL` — §5.3 shows it does not need one.
* No change to `LEAN_LPF`, `VEL_I_CLAMP`, `POS_ERROR_CLAMP`, `Kpos`, `Kvel`,
  `KVEL_I`, or any inner-loop gain. **If a scanner stop needs the tune changed,
  the scanner is wrong, not the tune.**
* Turn (`TURN_AUTHORITY`) is untouched. It is an open-loop effort differential
  with no velocity feedback for a cap to act on, and the scanner has no
  information about what a pivot will sweep into.

---

## 11. Bench procedures and acceptance tests

In order. Each produces a number a later step depends on.

### 11.1 `RANGE_TEST` — the two numbers, and a part check

Motors off, servo parked at 0°, raw counts streamed at 10 Hz.

1. Flat matte white card, perpendicular to the optical axis, on a tape measure.
2. **The calibration is two readings: the count at 45 cm → `RANGE_N_STOP`, and
   the count at 70 cm → `RANGE_N_CLEAR`.** Let each settle for 2 s and take the
   median of the stream, not one line. That is the whole calibration (§9.3).
3. Also record 20 cm and 80 cm as a **part check**: the response must be visibly
   flat past ~80 cm and reach ~2.3 V (≈1700 counts) at 10 cm. Still climbing at
   100 cm means the unit is a `2Y0A02` (20–150 cm); saturated by 30 cm means a
   `2Y0A41` (4–30 cm). Both look identical from the outside.
4. **Fold-back:** bring the card from 10 cm in to 3 cm in 1 cm steps. Note the
   peak count and confirm `RANGE_N_BLOCKED` sits below it, so the entire rising
   flank is inside the BLOCKED band (§9.4).
5. Note the noise band at 70 cm — the peak-to-peak spread of the stream on a
   stationary target. It sets `RANGE_N_SPREAD`.

### 11.2 `BEAM_TEST` — measure the angle

**The test that answers "what is the angle of this thing", with no substitute,
because Sharp does not publish the number (§1.1).**

1. Sensor rigidly clamped, axis horizontal, servo parked at 0°.
2. Flat card at exactly `D` = 50.0 cm, perpendicular. Confirm a stable reading.
3. Remove it. Substitute a narrow vertical target of known width `w` — a 10 mm
   dowel is ideal — at the same `D`, on the axis. Confirm it still reads the
   same. **Nothing else within 1.5 m behind it.**
4. Translate the target laterally in 2 mm steps, recording the count at each.
   Note `x_left` and `x_right`, where the reading stops tracking the target and
   drops to no-return.
5. Full effective beam angle, with the target's own width removed:

   ```
   alpha = 2 * atan( ((x_right - x_left) - w) / (2*D) )
   ```

   Expected span at 50 cm: ~4–5 cm plus `w`, giving alpha ≈ 4.6–5.7°.
6. **Repeat at `D` = 30 cm and `D` = 70 cm.** Constant `alpha` means a cone from
   a point and the §2 tables apply as written; a constant *span* instead means a
   collimated beam and the coverage arithmetic must be redone.

Feed the result into `SCAN_STEP_DEG` (must be ≤ alpha) and the coverage table
in §2.

### 11.3 `SERVO_CAL` — endpoints and scale

Never assume 1000/2000 µs, never use `Servo::write(degrees)`.

1. Sensor mounted, cable dressed as it will be in service (§8.2).
2. Type raw microsecond values over Serial. Find the values at which the horn
   reaches the **mechanical** limits of the mount, back off 100 µs from each,
   record those as the usable endpoints.
3. Command a known travel and measure the actual angle with a protractor against
   the mount. Compute `SERVO_US_PER_DEG` from the measurement.
4. Confirm `SERVO_LIMIT_DEG` = 60° keeps total cable twist ≤ 120°.
5. Watch for jitter with the servo holding still. Jitter is either an unlatched
   3.3 V signal (§6.4) or a supply problem (§6.2) — not something to filter in
   software.

### 11.4 `SWEEP_TEST` — dwell adequacy and backlash

Motors off, robot clamped, one sweep dumped per line.

* **Dwell:** aim at a flat wall square-on. The sweep should be a smooth arc.
  Smearing at object edges, or a bearing whose reading depends on the bearing
  sampled *before* it, means `SCAN_SETTLE_TICKS` or `SCAN_SAMPLE_TICKS` is too
  short (§5.1).
* **Invalid rate:** log how often a dwell fails the `RANGE_N_SPREAD` test.
  Persistently above ~20 % means the sample window is landing on sensor updates
  too often — lengthen `SCAN_SAMPLE_TICKS` and accept the slower sweep.
* **Backlash:** temporarily allow a bidirectional sweep, put a narrow target at
  a known bearing, and measure the bearing difference between directions. Then
  **turn bidirectional back off** and confirm the artefact is gone.
* **Period:** confirm the real sweep period matches the 0.79 s of §5.2. If it
  does not, re-run the §5.3 simulation with the real number before driving.

### 11.5 Acceptance — the four guards

Not optional; each covers a way this feature silently breaks something already
working.

| # | Test | Pass criterion |
|---|---|---|
| **A1** | **Re-run `DEADBAND_TEST`** — loaded, wheels on the floor, robot upright, steadied by hand at the top | Floors still measure 10/10 and 18/18. Anything else means the motor PWM frequency moved — §7, Trap 2 |
| **A2** | Watch `HZ` in the telemetry with the scanner running | Still 200. The tick must not stretch — §7, Timing |
| **A3** | Re-measure the rest pitch, free-standing and disarmed | `BALANCE_SETPOINT` updated for the added mass — M4 |
| **A4** | **Ground-noise test.** Robot propped, wheels free, fixed target at 60 cm, servo parked. Arm and sweep the drive stick through its full range | Reported count moves **< 10 counts** (≈2 cm at 70 cm) across the full PWM range. A count that tracks motor effort means motor return current is sharing the sensor's ground conductor — §6.1, requirement E1 |

A4 is the one that gets skipped and the one that matters. 16 cm of
effort-correlated range error (§6.1) does not look like a wiring fault in the
telemetry — it looks like a world that moves when you drive.

### 11.6 First drive

Scanner switch **off**. Drive normally, confirm nothing changed. Then switch it
on with the robot stationary and an obstacle at 1 m, and walk the obstacle in
while watching `NMAX` and `CAP` — the cap should start falling as the obstacle
crosses 70 cm and reach zero at 45 cm. Only then drive at it, slowly, with a
hand on the arm switch.

---

## 12. Limitations

State these in the commit message and anywhere this feature is described. A
sensor whose limits are undocumented gets trusted by the next person.

* **Thin objects read as clear.** A chair leg is ~3 cm wide; the beam spot at
  60 cm is ~5 cm, so the target only partially fills it. A weak partial return
  frequently reads as no return at all — i.e. **"clear"**. This is the most
  likely real-world miss.
* **Dark and matte surfaces.** Triangulation is far less reflectance-dependent
  than intensity ranging, but a matte black surface at 70 cm can still fail to
  return enough light. Fails toward "clear".
* **Specular surfaces are invisible.** Glass doors, mirrors, a polished floor at
  a grazing angle: the beam leaves and does not come back.
* **Sub-10 cm targets read as far** (§9.4). Mitigated mechanically by M6, not
  eliminated.
* **Nothing above or below the swept plane.** One horizontal slice at 0.35 m.
  Steps, thresholds, floor cables, low shelves, table edges: invisible. A
  downward-looking `2Y0A41` would cover that axis and is out of scope here.
* **Blind while braking.** Nose-up pitch rides the beam over the target (§4.3).
  §5.3 shows the rate-limited cap recovery contains it; it is not eliminated.
* **31 cm of closure between looks** at the same bearing, at full speed (§5.2).
  Absorbed by the thresholds in §5.3, not removed.
* **No authority over turn.** The robot can still pivot into something.

> **The arm switch remains the safety system.** This is a driver assist on a
> platform the pilot can already stop instantly. Nothing here should be
> described to anyone as a guarantee.

---

## 13. Scope note: what this is not

Recorded so the ground is not re-covered.

**Not SLAM, and not close.** The binding constraint is the 80 cm range ceiling —
rooms are 3–5 m, so there is no room geometry within range for scan matching to
work against, whatever the front end. Behind that: ~11 points/s against the
~8,000/s of an entry-level 360° lidar, 7 points per sweep against ~1,450, and a
0.79 s sweep during which the robot moves 31 cm and pitches, so every point in a
"scan" comes from a different pose.

**Not a 3-D scan.** One servo gives a 1-D polar slice. A genuine point cloud
needs a second (tilt) axis and a stationary robot: ±60° × ±20° at 5° is 225
points at ~20 s per frame, 7 cm spacing at 80 cm, nothing beyond 80 cm.

**Not a telemetry/visualisation pipeline.** Streaming the full raw stream to a
PC is trivially cheap (~4.8 kB/s at 200 Hz with pose, over the existing USB CDC
link, whose 115200 baud argument is ignored anyway) and would be worth doing for
a mapping build. It is not needed to stop the robot hitting a wall, and it is
not in this spec.

If any of that becomes the goal later, the sensor changes with it: a
**VL53L5CX** (8×8 multizone ToF, 4 m, 15 Hz, ~$15) for 3-D-ish data without a
servo, or an **RPLIDAR A1** (360°, 12 m, 8 kHz, ~$100) with Hector SLAM — which
is specifically designed to run without odometry, and therefore suits a
balancing robot, whose translation odometry is corrupted by the balance loop
while its differential/yaw odometry stays comparatively clean. Note the servo
conclusion inverts there: a 360° lidar already scans azimuth itself and instead
needs its scan plane held level through ±20° of body pitch, so on that build the
servo belongs on the **pitch** axis.

---

## References

* [Sharp GP2Y0A21YK0F datasheet (Sharp)](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf)
  — range, output curve, timing, marking scheme, bypass capacitor note. Contains
  no beam-angle specification.
* [Datasheet mirror (Pololu)](https://www.pololu.com/file/0j85/gp2y0a21yk0f.pdf)
  · [Pololu product page](https://www.pololu.com/product/136)
  · [SparkFun GP2Y0A21YK](https://www.sparkfun.com/infrared-proximity-sensor-sharp-gp2y0a21yk.html)
* [Makerguides — GP2Y0A21YK0F with Arduino](https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/)
  — linearisation fits and wiring practice.
* [CONTROL_THEORY.md](CONTROL_THEORY.md) — the cascade this must not bypass.
* [README.md](README.md) — pinout, current tune, and the bare-threshold rule.
