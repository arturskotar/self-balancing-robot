# SHARP 2Y0A21 Scanning Rangefinder — Wiring & Configuration Spec

Status: **design spec, not yet built.** Nothing in `balance_v2.ino` implements
this yet. Every number below is either arithmetic you can check, a datasheet
figure, or an explicitly-flagged estimate with the bench procedure that settles
it. Nothing here is a measured result from this robot.

## Table of contents

1. [What the part actually is](#1-what-the-part-actually-is)
2. [The angle — the question this document exists to answer](#2-the-angle)
3. [Why it needs a servo](#3-why-it-needs-a-servo)
4. [Which axis the servo turns, and why it is yaw](#4-which-axis-the-servo-turns)
5. [Electrical spec](#5-electrical-spec)
6. [Pin budget and the two Teensy traps](#6-pin-budget-and-the-two-teensy-traps)
7. [Mechanical spec](#7-mechanical-spec)
8. [Firmware spec](#8-firmware-spec)
9. [Integration with the cascade — the one rule](#9-integration-with-the-cascade)
10. [Bench procedures and acceptance tests](#10-bench-procedures-and-acceptance-tests)
11. [Known limitations](#11-known-limitations)
12. [Open questions](#12-open-questions)

---

## 1. What the part actually is

**It is not a lidar.** No time-of-flight, no laser, no scanning, no phase
measurement. It is an *IR triangulation rangefinder*: an infrared LED throws a
spot, a position-sensitive detector (PSD) sitting ~19.5 mm to one side images
that spot, and the spot's position on the PSD gives the range by similar
triangles. That distinction is not pedantry — it sets everything downstream:
one range per measurement, no bearing information, ~26 Hz update, and a
non-linear, non-monotone output curve. Calling it a lidar in code or comments
will eventually cause someone to assume it returns a scan. It returns a number.

| | |
|---|---|
| **Part** | Sharp `GP2Y0A21YK0F` (body marking `2Y0A21`) |
| **Type** | IR triangulation, analog output |
| **Rated range** | 10–80 cm |
| **Supply** | 4.5–5.5 V, ~30 mA average |
| **Output** | ~2.3 V @ 10 cm → ~0.4 V @ 80 cm, **non-linear, non-monotone below 10 cm** |
| **Update period** | 38.3 ms ± 9.6 ms (~26 Hz nominal, 21 Hz worst case) |
| **Connector** | JST PH 3-pin, pin 1 = `Vo`, pin 2 = `GND`, pin 3 = `Vcc` |

### On "F25"

The datasheet's marking scheme is `2Y0A21` followed by a lot code — the
datasheet's own example prints as `2Y0A21 F 4Z`. The `F` is a fixed marking
character and the trailing characters are the production lot, so **`F25` is a
lot code, not a variant**. There is no `GP2Y0A21…-F25` ordering part number in
Sharp's line-up. Practical consequence: it is an ordinary `GP2Y0A21YK0F`, and
the calibration in §8.4 must be measured *per unit* anyway, because unit-to-unit
spread on these is the dominant error term — bigger than anything the lot code
would tell you.

If your unit turns out **not** to be a 2Y0A21 (a 2Y0A02 is 20–150 cm, a 2Y0A41
is 4–30 cm, and they look identical), the range test in §10.1 will show it
immediately: check where the output curve tops out.

---

## 2. The angle

This is the part everyone gets wrong, so it gets the longest section.

### 2.1 Sharp does not publish one

There is no field-of-view, beam-angle, or spot-size specification in the
`GP2Y0A21YK0F` datasheet. It gives range, output voltage, supply, timing, and
outline dimensions, and it shows an example detection graph — no angular
characteristic. Anyone quoting an exact FOV for this part is quoting a
measurement, not a spec, and usually not their own.

So this spec does two things: derive the bound the optics impose, and give you
the bench procedure (§10.2) that pins it down for *your* unit.

### 2.2 What the geometry bounds it to

The emitted spot is roughly ø5–7 cm at 80 cm — that is the figure consistently
reported from measurement, and it is consistent with the ~7 mm lens aperture on
a 44.5 mm package. Full cone angle from a spot diameter `w` at range `d`:

```
alpha = 2 * atan(w / (2*d))

w = 5 cm @ 80 cm  ->  alpha = 3.6 deg
w = 7 cm @ 80 cm  ->  alpha = 5.0 deg
```

The *effective* beam is narrower than the emitted one, because a target only
produces a reading if it is inside the emitter cone **and** imaged on the PSD.
So:

> **Working assumption: full effective beam angle alpha ≈ 4–5°, i.e. ±2–2.5°
> about the optical axis. Treat 5° as the pessimistic (widest) case for
> coverage arithmetic and 4° as the pessimistic (narrowest) case for gap
> arithmetic, until §10.2 replaces both with a measurement.**

Spot diameter at range, at alpha = 5°:

| Range | 15 cm | 30 cm | 50 cm | 80 cm |
|---|---|---|---|---|
| Spot ø | 1.3 cm | 2.6 cm | 4.4 cm | 7.0 cm |

### 2.3 What that angle means for a robot that has to not hit things

The robot's collision corridor is its own width. Take the half-width as
**w = 10 cm** (track plus a margin — measure yours and substitute). The
half-angle you must actually see at range `d` is `phi = atan(w/d)`:

| Range `d` | 20 cm | 30 cm | 40 cm | 60 cm | 80 cm |
|---|---|---|---|---|---|
| Required half-angle `phi` | 26.6° | 18.4° | 14.0° | 9.5° | 7.1° |
| Fixed sensor covers | ±2.5° | ±2.5° | ±2.5° | ±2.5° | ±2.5° |
| **Corridor fraction seen** | **9 %** | **13 %** | **17 %** | **26 %** | **35 %** |

Read the bottom row twice. A rigidly-mounted 2Y0A21 pointed dead ahead sees
about **an eighth of the corridor at 30 cm** — and the coverage gets *worse* as
the obstacle gets closer, which is exactly backwards from what you want. A
table leg, a door frame, a chair foot, the corner of a box: all of them are
15–20 cm off-axis at the range where you still had time to stop, and all of
them are invisible.

That table is the entire argument for the servo, and it is a geometric fact
about a 5° beam, not a preference.

---

## 3. Why it needs a servo

### 3.1 The coverage argument

From §2.3: one beam sees 9–35 % of the corridor. Sweeping the same beam across
the corridor sees all of it. Because the required half-angle is
range-dependent, so is the sector:

* **±15°** covers the full corridor from ~37 cm outward.
* **±30°** covers it from ~18 cm outward.
* **±60°** is only useful stopped or crawling — it buys situational awareness
  (where is the wall, where is the gap) rather than collision avoidance.

Step size must be **≤ the beam width**, or the scan has holes. At a 7.5° step
with a 5° beam there is a 2.5° gap between spots, which at 80 cm is a 3.5 cm
blind stripe — wide enough to swallow a chair leg. **Use a 5° step and shrink
it if §10.2 measures the beam narrower than 5°.**

### 3.2 The sensor's 38.3 ms cycle forbids a continuous sweep

The sensor integrates over its whole 38.3 ms measurement window. If the beam is
moving during that window, the returned range is a blend over whatever the beam
swept across — an obstacle edge smears into a ramp, and the reported range
belongs to no particular bearing. At a leisurely 90°/s continuous sweep the
beam moves **3.4° per measurement cycle**, comparable to the entire beam width.
The scan would be angularly meaningless.

So the sweep is **step-and-dwell**, not continuous, and the dwell is set by the
sensor, not the servo:

```
dwell = servo travel + mechanical settle + sensor latency
      = ~10 ms (5 deg at 0.12 s/60 deg)
      + ~20 ms (settle)
      + >= 38.3 ms (one full measurement cycle taken entirely after settling)
      ~= 90 ms per point, with the sample taken in the last 40 ms
```

The `>= 38.3 ms` is a floor, not a guarantee: the sensor's cycle free-runs and
is not synchronised to us, so a 38.3 ms wait can still land on a sample that
began before the servo stopped. §8.5 handles that with an agreement test rather
than by waiting 2 full cycles (which would cost 130 ms/point).

### 3.3 What that costs, and why the sector is speed-scheduled

At 90 ms/point, plus a flyback (§7.3 — the scan is unidirectional to defeat
gear backlash):

| Sector | Points @5° | Scan | Flyback | Period | Rate | Distance travelled @ 0.396 m/s |
|---|---|---|---|---|---|---|
| ±60° | 25 | 2.25 s | 0.34 s | 2.59 s | 0.39 Hz | 103 cm |
| ±45° | 19 | 1.71 s | 0.28 s | 1.99 s | 0.50 Hz | 79 cm |
| ±30° | 13 | 1.17 s | 0.24 s | 1.41 s | 0.71 Hz | 56 cm |
| ±15° | 7 | 0.63 s | 0.16 s | 0.79 s | 1.27 Hz | 31 cm |

(0.396 m/s is this robot at `DRIVE_MAX_VEL` 0.90 rev/s on 140 mm wheels:
0.90 × pi × 0.140.)

The last column is the one that matters: it is how far the robot moves between
two looks at the same bearing. Against a 80 cm maximum range, a ±60° sweep at
full speed is worthless — the map is a metre stale before it completes. Only
the ±15° sector keeps up.

And here the timing and the geometry agree, which is the good news:

> **±15° is not a compromise forced by the servo's speed. It is exactly the
> collision corridor at 40–80 cm** (§2.3: `phi` = 14.0° at 40 cm, 7.1° at
> 80 cm). The narrow-at-speed / wide-when-slow schedule is what the geometry
> wanted anyway.

Latency budget for a stop from full speed:

```
scan latency (worst case, +-15 deg)          0.79 s -> 31 cm
lean build (LEAN_LPF 0.95, ~98 ms, plus the
  non-minimum-phase reversal transient)      ~0.30 s -> 12 cm
braking at max lean a = g*tan(20 deg)
  = 3.57 m/s^2, from 0.396 m/s                        ~2 cm
                                             -------------
                                                     ~45 cm
```

Against a 80 cm ceiling that leaves ~35 cm of margin — workable, and tight
enough that **`DRIVE_MAX_VEL` must be re-examined the moment the scanner is
given any authority over the drive command.** Doubling the speed does not
double the stopping distance, it eliminates the margin.

### 3.4 The mass penalty is negligible, and points the right way

Standard objection: don't put mass on top of an inverted pendulum. The
arithmetic says it does not matter here, and what little it does is helpful.

Take the added mass as 20 g (SG90 9 g + sensor 3.5 g + bracket and fasteners)
at 0.30 m, on a robot of ~1.5 kg with its CoM at ~0.15 m. **Substitute your
real numbers — these are estimates.**

```
new CoM height  = (1.5*0.15 + 0.020*0.30) / 1.520 = 0.152 m   (+1.3 mm)
pendulum time constant  tau = sqrt(l/g)
  before: sqrt(0.150/9.81) = 0.1237 s
  after:  sqrt(0.152/9.81) = 0.1245 s   (+0.7 %)
```

Raising the CoM *lengthens* the fall time constant, which gives the loop more
time, not less. +0.7 % is noise.

The one real effect is the **balance setpoint**, because the repo treats it as a
measured quantity to 0.01° (`BALANCE_SETPOINT = -3.22`). A 20 g mass mounted
3 cm forward of the CoM line shifts the horizontal CoM by
`0.020 * 0.03 / 1.52 = 0.39 mm`, i.e. `atan(0.00039/0.152) = 0.15°`. Small, but
larger than the resolution the setpoint is quoted at.

> **Acceptance requirement: after mounting, re-measure the rest pitch
> (`STATIC_LEAN_TEST` path, free-standing and disarmed) and update
> `BALANCE_SETPOINT`.** Mount the assembly on the pitch-axis centreline if you
> can, so the shift is near zero.

### 3.5 What one servo does not buy you

Be clear about the ceiling so nobody over-promises this thing:

* **No vertical coverage.** One azimuth sweep is a 1-D polar slice at one
  height. A step, a threshold, a cable on the floor, a low shelf: invisible.
* **No map.** 7–25 points at 0.4–1.3 Hz with no yaw sensor (the robot has open-
  loop turn only — see `TURN_AUTHORITY`) is not odometry-registerable into a
  world frame. This is a *reactive* corridor check, not SLAM.
* **No guarantee.** §11 lists targets this sensor genuinely cannot see. The arm
  switch remains the safety system.

---

## 4. Which axis the servo turns

### 4.1 Reaction torque does not decide it

The usual reflex is "put the servo on the yaw axis so its reaction torque
doesn't disturb the balance loop." Check the number before believing it.

Sensor plus bracket, ~2e-6 kg·m² about the rotation axis, stepping 5°
(0.0873 rad) in 10 ms with a trapezoidal profile:

```
alpha_ang = 4*theta / t^2 = 4*0.0873 / (0.010)^2 = 3492 rad/s^2
torque    = J * alpha_ang = 2e-6 * 3492            = 7.0e-3 N*m
```

Against the robot's gravitational restoring torque per degree of lean
(`m*g*l*sin(1 deg)` = 1.5 × 9.81 × 0.15 × 0.01745 = 0.0385 N·m/deg), 7 mN·m is
**0.18° of equivalent lean, for 10 ms, twice per scan point.** That is below
`ANGLE_DEADZONE`-scale signals and far below what the complementary filter
and `LEAN_LPF` would pass. Reaction torque is a non-argument on either axis.

(The genuine coupling path from the servo into the balance loop is *electrical*,
not mechanical — see §5.2. That one is real and is why the star wiring is a
requirement.)

### 4.2 Information decides it: yaw

The servo has one degree of freedom. Spend it where the robot is blind.

* **Pitch is already measured.** `pitch` is available every 5 ms from the
  complementary filter. Mechanically stabilising a quantity you already know to
  a fraction of a degree is redundant hardware doing a job one multiply does
  for free (§4.3).
* **Azimuth is not observable at all.** Nothing on this robot measures bearing
  to anything. There is not even a yaw rate in the control path — turn is
  open-loop effort differential (`TURN_AUTHORITY`), with "nothing measuring
  yaw" as the file's own comment.
* **A pitch gimbal does not fix the coverage problem.** It would still see 13 %
  of the corridor at 30 cm (§2.3). A yaw sweep is the only thing that changes
  that number.

> **Decision: the servo axis is vertical. The sensor sweeps in azimuth. Pitch
> is compensated in software.**

### 4.3 The pitch problem, and the software fix

Pitch is not a nuisance to be ignored — on this platform it is the dominant
geometric error, because the chassis genuinely leans. `VEL_I_CLAMP` is 20° and
is the dominant lean term; transients go past it.

A sensor at height `h` with its axis horizontal at zero pitch, pitched nose-down
by `theta`, puts the *lower edge* of its beam (`theta + alpha/2`) into the floor
at:

```
d_floor = h / sin(theta + alpha/2)      (alpha/2 = 2.5 deg)
```

| Nose-down pitch | 10° | 12° | 15° | 20° | 25° | 30° |
|---|---|---|---|---|---|---|
| `d_floor`, sensor at **h = 0.20 m** | 0.92 m | **0.80 m** | 0.67 m | 0.52 m | 0.43 m | 0.37 m |
| `d_floor`, sensor at **h = 0.30 m** | 1.39 m | 1.20 m | 1.00 m | **0.78 m** | 0.65 m | 0.56 m |

Bold marks where the floor first enters the 80 cm window. At 20 cm the floor
becomes a "obstacle at 80 cm" at only **12° of nose-down lean** — which is
routine driving here. At 30 cm it takes 20°. So:

**Requirement M1: mount the sensor as high as practical, h ≥ 0.30 m.**
**Requirement M2: no downward tilt bias, ever.** Tilting down for cliff
detection trades the whole forward range away.

Nose-*up* is the mirror failure and cannot be mounted away: the beam rises
`d*sin(theta)` above nominal, so at 15° up and 60 cm the beam is 15.5 cm high of
where it should be. With h = 0.30 m that means **obstacles shorter than about
15 cm are unreliable at 60 cm**, whatever you do. Say so in the limitations, do
not pretend otherwise.

Software compensation — cheap, exact, and it uses the pitch you already have.
Tag every sample with the `pitch` at the instant of sampling, then:

```c
// theta_dn = downward beam-axis angle, positive = nose down
float theta_dn = -(pitch - BALANCE_SETPOINT) + RANGE_PITCH_OFFSET_DEG;

float z = MOUNT_HEIGHT_M - d_m * sinf(radians(theta_dn));  // target height AGL
float x = d_m * cosf(radians(theta_dn));                   // horizontal range

if (z < FLOOR_REJECT_M) -> classify FLOOR, discard, do not treat as obstacle
else                    -> horizontal range is x, not d
```

`FLOOR_REJECT_M` ≈ 0.04 m. This turns the dominant false-positive into a
labelled, discarded sample instead of a phantom wall that brakes the robot
every time it leans to accelerate.

**`RANGE_PITCH_OFFSET_DEG` exists so that the sensor is aligned to the
*balance* attitude, not the mechanical chassis frame.** `BALANCE_SETPOINT` is
-3.22°, i.e. the IMU's zero is not the attitude the robot stands at. Align the
optical axis horizontal when `pitch == BALANCE_SETPOINT`, and keep the residual
in this one constant so a future setpoint retune is a one-number edit rather
than a re-shim.

---

## 5. Electrical spec

### 5.1 Grounding — read this before anything else

The ADC measures `Vo` against the Teensy's analog ground. Any voltage developed
across a shared ground return by motor current appears **directly** as range
error, with no filtering that can remove it (it is at 2 kHz, correlated with
motor torque, which is correlated with everything the robot does).

```
motor return 2 A through 0.05 ohm of shared ground wire = 100 mV of offset

sensitivity of the curve (from V = 33.9/(d + 4.74), section 8.4):
  dV/dd at 30 cm = 33.9/(34.74)^2 = 28.1 mV/cm  -> 100 mV = 3.6 cm error
  dV/dd at 80 cm = 33.9/(84.74)^2 =  4.7 mV/cm  -> 100 mV =  21 cm error
```

21 cm of range error, modulated by motor effort, at exactly the range where you
were hoping to detect things early.

> **Requirement E1: the sensor's ground returns to the logic/ADC star point on
> its own conductor. It must never share a conductor with motor return
> current.** The README's "all logic grounds must share a reference" is
> necessary but not sufficient here — this is a *separate-conductor*
> requirement, not just a common-node one.

Route `Vo` twisted with its own ground return wire, away from the motor leads
and away from the servo lead. If the servo lead must cross a motor lead, cross
at 90°.

### 5.2 Supply and decoupling

The sensor pulses its IR LED once per 38.3 ms cycle and draws a current spike
doing it. The datasheet asks for a bypass capacitor of **more than 10 µF**
between `Vcc` and `GND`. Do better than the minimum:

> **Requirement E2: 100 µF electrolytic ‖ 0.1 µF ceramic, at the sensor
> connector, not at the buck.**

> **Requirement E3: sensor and servo get separate feeds from the 5 V buck (star
> from the buck output). Do not daisy-chain the sensor off the servo's 5 V.**

Why E3 matters, as a worst-case bound: a 250 mA servo transient across 0.1 Ω of
shared 5 V wire is a 25 mV supply dip. If the sensor output tracked its supply
1:1 (it does not, quite, but bound it), that is 5 cm of range error at 80 cm,
correlated with every scan step. The star wiring costs one extra pair of wires.

Budget: sensor ~30 mA average; servo ~10 mA idle, 100–250 mA slewing, up to
~700 mA stalled. **The 5 V buck needs ≥1 A of headroom over its current load.**

### 5.3 The divider — the Teensy is not 5 V tolerant

The 2Y0A21's output is not bounded by its useful range: it *peaks* near 3.1 V
around 4–6 cm, inside the fold-back region (§8.4). The Teensy 4.1's ADC
reference and absolute pin maximum are both 3.3 V. Roughly 0.2 V of margin on a
loosely-specified peak, with no protection against a mis-wire putting 5 V on
the wire, is not acceptable.

```
        Vo ----[ 6.8k ]----+---- Teensy A0 (pin 14)
                           |
                          ---  0.1uF
                          ---
                           |
              [ 10k ]      |
        AGND ----------+---+
```

* Ratio `10/(6.8+10)` = **0.5952**. Thevenin source impedance **4.05 kΩ**.
* Normal peak 3.1 V → 1.85 V at the pin. **Fault case (Vcc shorted to Vo,
  5.0 V) → 2.98 V — still inside the rail.** That headroom is the reason for
  0.60 rather than a "tighter" 0.68.
* RC with the 0.1 µF: 0.41 ms — three orders under the 38.3 ms sensor cycle, so
  it filters noise without smearing the measurement. (0.47 µF → 1.9 ms is also
  fine if the trace is noisy.)

Resolution cost of the divider — the objection to check, not assume. At 12-bit
over 3.3 V:

```
10 cm: 2.30 V * 0.5952 = 1.369 V -> 1699 counts
80 cm: 0.40 V * 0.5952 = 0.238 V ->  295 counts
span over the whole rated range   -> 1404 counts

worst-case resolution, at 80 cm: 4.7 mV/cm * 0.5952 = 2.8 mV/cm = 3.5 counts/cm
```

3.5 counts per centimetre at the far end, against a sensor whose own noise is
several centimetres there. The divider costs nothing that matters.

### 5.4 Servo signal level

The Teensy drives 3.3 V logic. Most SG90/MG90S-class servos latch reliably on
3.3 V but none of them *specify* it. This is the same question the repo already
settled for the BTS7960 by reading the datasheet (`V_IN(H)` max 2.0 V, absolute
and not ratiometric — the planned 74HCT244 was unnecessary). Here there is no
such guarantee to read, so: **wire it direct, and if the servo jitters or
refuses to hold position, add a 74HCT125 buffer on 5 V rather than tuning
around it.** Jitter from an unlatched signal will look exactly like backlash.

### 5.5 Connector and cable

Datasheet pinout at the sensor: **pin 1 = `Vo`, pin 2 = `GND`, pin 3 = `Vcc`**.
The supplied JST-PH pigtail is commonly red / black / white or red / black /
yellow, but the colour-to-pin mapping is not something to take on trust.

> **Verify with a meter before first power-up.** Continuity from pin 2 to the
> sensor's ground plane identifies `GND`; then power the sensor on the bench
> with the third wire unconnected and confirm which pin reads ~5 V. Getting
> `Vcc` and `Vo` swapped puts 5 V into the divider — survivable with the 0.60
> ratio specified above, and not survivable without it.

---

## 6. Pin budget and the two Teensy traps

Current allocation, from `balance_v2.ino`:

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
| **14 (A0)** | sensor `Vo` via the 6.8k/10k divider | ADC only, no PWM involvement |
| **8** | servo signal | FlexPWM1_3_A — a different FlexPWM *module* from both motors |

### Trap 1 — `analogWriteResolution` is global, and the servo cannot use hardware PWM

`balance_v2.ino:1498` sets `analogWriteResolution(MOTOR_PWM_BITS)` with
`MOTOR_PWM_BITS = 8`. On Teensy 4 that setting is **global to every
`analogWrite` channel**, which the file's own comment already states
("analogWrite range is explicitly 0..255 on every channel").

Consequences, both directions:

```
Using analogWrite for the servo at the current 8-bit setting:
  50 Hz period 20000 us / 256 = 78.1 us per count
  servo band 1000..2000 us    = 12.8 counts for the ENTIRE travel
                              ~ 14 deg per count            -> unusable

Raising the global resolution to 12 bits to fix that:
  every motor analogWrite(0..255) now runs against a 4095 full scale
  -> ~6 % duty ceiling, the robot does not stand up
  -> and the measured deadband floors (10/10 static, 18/18 moving, in RAW
     8-bit counts at 2000 Hz) all become meaningless
```

> **Do not change `MOTOR_PWM_BITS`. Do not use `analogWrite` for the servo. Use
> the `Servo` library**, which on Teensy 4.x drives the pin from a hardware
> timer interrupt and does not touch `analogWrite` state at all. Command it
> with `writeMicroseconds()`, never `write(degrees)` — the degree mapping is a
> library convention, and §10.3 measures the real endpoints.

Avoid `PWMServo` here specifically *because* it uses the hardware PWM path.

### Trap 2 — FlexPWM submodules share a frequency

If anyone later reaches for hardware PWM anyway, `analogWriteFrequency(pin, 50)`
sets the frequency of the pin's whole FlexPWM submodule. Land that on a
submodule a motor is using and `MOTOR_PWM_HZ` silently becomes 50 — and
`MOTOR_PWM_HZ` is a control parameter in this repo, matched to measured deadband
floors. Motors sit on FlexPWM2_2 and FlexPWM4_0/4_1; pin 8 is FlexPWM1_3_A, a
different module. **Re-check that against the current PJRC pin table before
soldering** — the mapping is the kind of thing that is right until it isn't.

> **Guard, independent of which route you take: after the servo is wired and
> the firmware runs, re-run `DEADBAND_TEST` and confirm the floors still
> measure 10/10 and 18/18. If they moved, the motor PWM frequency moved.**

### Timing

Nothing added may stretch the 5 ms tick (`CONTROL_PERIOD_US = 5000`).

* `analogReadResolution(12)` and `analogReadAveraging(16)` are unrelated to
  `analogWriteResolution` and are safe; 16 averages costs tens of microseconds.
* `Servo::writeMicroseconds()` is a register write.
* **Never** call `attach()`/`detach()`, `delay()`, or any blocking wait from the
  control path. The scan is a state machine driven by the existing tick
  counter (§8.5).
* Servo ISR load is 2 pin edges per 20 ms for one channel — negligible beside
  the encoder ISRs (~500 edges/s per wheel at full speed).

---

## 7. Mechanical spec

### 7.1 Requirements

| Req | Statement | Why |
|---|---|---|
| **M1** | Lens height `h` ≥ 0.30 m above the floor | §4.3 — at 0.20 m the floor becomes a phantom 80 cm obstacle at only 12° of nose-down lean |
| **M2** | Optical axis horizontal when `pitch == BALANCE_SETPOINT`; residual captured in `RANGE_PITCH_OFFSET_DEG`. **No downward tilt bias.** | §4.3 |
| **M3** | Sensor and IMU rigidly coupled — same structural member, no compliant standoffs | Software pitch compensation assumes the sensor's pitch *is* the IMU's pitch |
| **M4** | Mast first bending mode ≥ 30 Hz, verified by tap test with telemetry running | 2° of flex = 2.1 cm of vertical error at 60 cm; a mast resonance also feeds the gyro and the balance loop |
| **M5** | Assembly mounted on the pitch-axis centreline where possible; `BALANCE_SETPOINT` re-measured afterwards | §3.4 — 20 g at 3 cm off-centre shifts the balance point ~0.15° |
| **M6** | Servo travel limited to ±60° mechanically **and** in firmware | Cable twist; see below |
| **M7** | Nothing in front of the lens closer than 10 cm can be reached by an external obstacle | §8.4 fold-back — sub-10 cm targets read as *far*, which is the dangerous direction |

### 7.2 Servo selection

Torque is a non-issue: §4.1 puts the peak accelerating torque at **7 mN·m
= 0.07 kg·cm**, against ~1.8 kg·cm for an SG90. Twenty-five times the margin, on
the cheapest servo in the drawer. Nothing about this application is torque
limited.

Pick on the two things that actually cost you scan quality:

* **Backlash** — sets the bearing artefact of §7.3. Plastic gears ~1–2°, metal
  gears roughly half that.
* **Deadband / positioning repeatability** — a servo with a wide pulse deadband
  will not resolve a 5° step reliably (5° ≈ 53 µs at 10.5 µs/deg). Anything with
  a deadband approaching that is unusable here regardless of its speed rating.

An **SG90 is adequate** given the unidirectional scan (§7.3), which removes the
backlash artefact rather than tolerating it. An **MG90S** buys back the flyback
time if §10.6 shows the sweep period is the binding constraint. Speed ratings in
the 0.10–0.12 s/60° class all land inside the `SCAN_SETTLE_TICKS` budget of
§3.2; verify with §10.6 rather than trusting the number on the label.

### 7.3 Backlash, and why the scan is unidirectional

SG90-class plastic gear trains carry roughly 1–2° of backlash. At 80 cm that is
1.4–2.8 cm of azimuth uncertainty — comfortably inside the 5–7 cm beam spot, so
as an absolute error it does not matter.

What does matter: a **bidirectional** sweep approaches each bearing from
opposite sides on alternate passes, so the two passes are offset by the full
backlash. A single narrow object then appears at two different bearings on
alternate sweeps — a systematic artefact that looks exactly like a real second
object appearing and disappearing at ~0.5 Hz.

> **Scan in one direction only. Fly back to the start at full servo speed and
> discard every sample taken during the flyback.** The cost is the flyback
> column in the §3.3 table (0.16–0.34 s). An MG90S (metal gears) roughly halves
> the backlash if you later want that time back.

### 7.4 Servo mounting and the cable

Servo body fixed to the mast, horn carries the sensor. The sensor's 3-wire tail
then twists with every sweep:

* Limit travel to ±60° so total twist never exceeds 120°.
* Use silicone-insulated wire with a service loop above the rotation axis.
* Strain-relieve at **both** ends — at the sensor and at the mast.
* A cable that stiffens the sweep changes the settle time, and the settle time
  is a term in the dwell budget (§3.2). If you re-route the cable, re-check the
  dwell with `SWEEP_TEST`.

### 7.5 Package roll orientation — undecided, see §10.5

The triangulation baseline runs along the 44.5 mm axis of the package. When the
beam straddles a depth discontinuity (a door frame, a box corner), the PSD sees
returns from two ranges at once and reports a blend rather than either. Whether
rolling the package 90° — baseline vertical — reduces that error for a sensor
sweeping horizontally is **not established here**, and it is a plausible enough
effect to be worth twenty minutes of bench time. §10.5 is the A/B test. Until
it is run, mount it in the conventional orientation (baseline horizontal) and
do not claim either way.

---

## 8. Firmware spec

### 8.1 Constants

Everything is behind one master switch so the feature compiles out entirely
until it is proven, matching the file's existing `ENCODER_TEST` /
`DEADBAND_TEST` / `EMF_TEST` convention.

```c
// ---- SHARP 2Y0A21 scanning rangefinder -------------------------------------
#define RANGE_SCANNER   0     // master enable. 0 = not compiled in at all.
#define RANGE_TEST      0     // 8.9 / 10.1 : motors off, stream raw + cm
#define BEAM_TEST       0     // 10.2 : servo parked, find the beam edges
#define SERVO_CAL       0     // 10.3 : drive raw microseconds from Serial
#define SWEEP_TEST      0     // 10.6 : motors off, dump one polar scan per sweep

#define RANGE_ADC_PIN   14        // A0, behind the 6.8k/10k divider (5.3)
#define SERVO_PIN        8        // FlexPWM1_3_A - not a motor submodule (6)

// Geometry (7). MOUNT_HEIGHT_M and RANGE_PITCH_OFFSET_DEG are MEASURED, 10.4.
const float MOUNT_HEIGHT_M         = 0.30f;   // lens height above floor, m
const float RANGE_PITCH_OFFSET_DEG = 0.00f;   // axis vs the BALANCE attitude
const float FLOOR_REJECT_M         = 0.04f;   // target below this AGL = floor

// Servo, in MICROSECONDS. Measured with SERVO_CAL (10.3) -- 1000/2000 and the
// Servo library's degree mapping are both conventions, not facts about a servo.
const int   SERVO_US_CENTRE  = 1500;
const float SERVO_US_PER_DEG = 10.5f;
const float SERVO_LIMIT_DEG  = 60.0f;   // hard: cable twist (M6). Never exceed.

// Sector schedule (3.3). Two velocities, not one: a single threshold on a noisy
// signal is the bug class this repo has hit three times (README).
const float SCAN_STEP_DEG        = 5.0f;   // <= measured beam width (10.2)
const float SCAN_SECTOR_WIDE_DEG = 30.0f;  // +/- deg, slow or stopped
const float SCAN_SECTOR_NARROW_DEG = 15.0f;// +/- deg, at speed = the corridor
const float SCAN_NARROW_VEL_HI   = 0.30f;  // rev/s: above -> narrow
const float SCAN_NARROW_VEL_LO   = 0.15f;  // rev/s: below -> wide

// Dwell budget (3.2), counted in 5 ms control ticks.
const int SCAN_SETTLE_TICKS = 6;    // 30 ms  servo travel + mechanical settle
const int SCAN_SAMPLE_TICKS = 12;   // 60 ms  covers one 38.3 ms sensor cycle
const int SCAN_EXTRA_TICKS  = 8;    // 40 ms  granted ONCE if samples disagree

// Sample validity, in raw 12-bit counts behind the 0.5952 divider (5.3, 8.4).
const int RANGE_ADC_BLOCKED   = 1773;  // above: nearer than 10 cm, FOLD-BACK
const int RANGE_ADC_NO_RETURN =  200;  // below: nothing in range
const int RANGE_ADC_DISAGREE  =   60;  // counts: spread that fails the dwell

// Corridor and response (9).
const float CORRIDOR_HALF_W_M = 0.10f;  // half track + margin. MEASURE YOURS.
const float RANGE_STOP_M      = 0.35f;  // cap reaches zero here
const float RANGE_CLEAR_M     = 0.45f;  // cap is unrestricted beyond here
const float RANGE_CAP_RECOVER = 1.0f;   // rev/s per second; drop is instant
const unsigned long RANGE_POINT_AGE_MS = 3000;  // a bearing older than this is
                                                // discarded, not trusted
```

### 8.2 ADC configuration

```c
analogReadResolution(12);
analogReadAveraging(16);   // ~tens of us; unrelated to analogWriteResolution
pinMode(RANGE_ADC_PIN, INPUT);   // no pullup - it would fight the divider
```

Sample every control tick (200 Hz) during the sample window. That is ~7–8
samples per 38.3 ms sensor cycle, so the oversampling buys **noise rejection,
not freshness** — the sensor still only produces a new number every 38.3 ms and
nothing in software changes that.

### 8.3 Per-dwell reduction

Take the **median** of the samples collected in the window, not the mean: a
single motor-commutation spike on the analog line displaces a mean and does not
displace a median. Then check the spread:

```
if (max - min) > RANGE_ADC_DISAGREE  ->  the window straddled a sensor update
                                          or the target is unstable
    grant SCAN_EXTRA_TICKS once and re-reduce
    still disagreeing -> mark the point INVALID, keep the previous value for
                         this bearing (with its age), move on
```

This is the cheap answer to the free-running 38.3 ms cycle (§3.2): instead of
paying two full cycles (130 ms/point) to *guarantee* a post-settle sample,
detect the straddle when it happens and pay the extra 40 ms only then.

### 8.4 Linearisation, and the fold-back

The output is `V ≈ A/(d + C)` — a hyperbola, not a line. Fitting the datasheet's
two anchor points (2.3 V at 10 cm, 0.4 V at 80 cm):

```
2.3 = A/(10 + C)                    A = 33.90
0.4 = A/(80 + C)        =>          C =  4.74

d_cm = 33.90 / V - 4.74
```

Sanity: `V` = 1.0 → 29.2 cm; `V` = 1.3 → 21.3 cm. Both within a couple of cm of
the published curve, and the residual is smaller than unit-to-unit spread.

Straight from raw counts, with the 0.5952 divider and 12-bit over 3.3 V
(`V = N × 3.3/4095 / 0.5952 = N × 1.3538e-3`):

```c
d_cm = 25040.0f / (float)N - 4.74f;      // one divide, no pow(), no table
```

Check the endpoints: `N` = 1699 → 10.0 cm. `N` = 295 → 80.1 cm. Good.

> **This is a seed, not a calibration.** Unit-to-unit spread on these parts is
> the dominant error term. `RANGE_TEST` (§10.1) produces a five-point table for
> *your* sensor; re-fit `A` and `C` from it and replace the constant `25040`.

**The fold-back is the dangerous part.** The curve is not monotone: output rises
from 0.4 V at 80 cm to ~2.3 V at 10 cm, peaks around 3.1 V at 4–6 cm, and then
**falls again**. So a target at ~3 cm produces roughly 1.0 V, which the formula
above reports as **29 cm** — an obstacle pressed against the robot reported as
comfortably clear. This is the one failure mode of this sensor that fails
*toward* a collision.

Two defences, both required:

1. **`N > RANGE_ADC_BLOCKED` (≈1773 counts, ≈2.4 V) is reported as `BLOCKED`,
   never converted to a distance.** That catches the rising side of the peak.
2. **Requirement M7 (mechanical):** the lens is recessed far enough behind the
   robot's frontmost surface — wheel envelope plus a standoff or bumper — that
   no external obstacle can physically get within 10 cm of it. This is what
   actually closes the hole, because defence 1 cannot see past the peak.

### 8.5 The scan state machine

Driven by the existing 5 ms tick. No `delay()`, no blocking, no allocation.

```
PARK      -> disarmed / fallen / link down: servo commanded to 0 deg, no
             sampling, map invalidated. Entered from anywhere, immediately.

SETTLE    -> servo has just been commanded to the next bearing.
             wait SCAN_SETTLE_TICKS.

SAMPLE    -> accumulate ADC samples for SCAN_SAMPLE_TICKS.
             reduce per 8.3. store {x, y, valid, millis()} for this bearing.
             advance bearing by SCAN_STEP_DEG.
             last bearing in the sector -> FLYBACK, else -> SETTLE.

FLYBACK   -> command the servo to the start bearing, discard everything,
             wait the flyback budget, then -> SETTLE.
             (unidirectional scanning, 7.3)
```

Sector selection is re-evaluated **only at FLYBACK**, never mid-sweep, so the
map is always one consistent sector. The wide/narrow choice is hysteretic
(`SCAN_NARROW_VEL_LO` / `_HI` on `|forwardVel|`), because a single threshold on
`forwardVel` is precisely the bug the README warns about — `VF` is quantised at
0.366 rev/s per encoder count before filtering, so any bare threshold in that
band chatters at loop rate.

Per stored point, using the `pitch` captured at sample time (§4.3):

```c
x = d * cos(theta_dn);              // horizontal range,  m
y = x * sin(azimuth);               // lateral offset,    m
z = MOUNT_HEIGHT_M - d*sin(theta_dn);   // height above floor, m

valid = (N in range) && (z >= FLOOR_REJECT_M) && (x > 0)
```

### 8.6 Reduction to a single number

```c
xNear = min{ x : point valid, |y| < CORRIDOR_HALF_W_M,
                 age < RANGE_POINT_AGE_MS }
        (no qualifying point -> xNear = +inf)
```

Points outside the corridor are map, not threat — a wall 25 cm to the side at
40 cm range is something a ±30° sweep sees constantly and must never brake for.

### 8.7 The cap is a continuous taper, not a threshold

```c
capRaw = DRIVE_MAX_VEL * constrain((xNear - RANGE_STOP_M)
                                   / (RANGE_CLEAR_M - RANGE_STOP_M), 0, 1);

if (capRaw < rangeVelCap) rangeVelCap = capRaw;             // drop instantly
else rangeVelCap = min(rangeVelCap + RANGE_CAP_RECOVER*DT,  // recover slowly
                       DRIVE_MAX_VEL);
```

Two deliberate choices, both borrowed from fixes already in this file:

* **Continuous, not binary.** The friction floor used to be a
  `moving ? dbMoving : dbStatic` gate and square-waved the output by 8 counts at
  loop rate, producing an ~8.7 Hz limit cycle; it was fixed by making it a
  continuous blend. A stop/go range gate is the same mistake with a noisier
  input. A taper has no threshold to chatter on.
* **Asymmetric.** Instant drop, rate-limited recovery — the same shape as the
  drive release debounce (`DRIVE_RELEASE_TICKS`: "demand engages instantly, and
  it takes N consecutive ticks of no demand to disengage"). Safety engages now;
  releasing safety is the thing that has to be sure.

Note what is *not* debounced: engagement. A validated point inside
`RANGE_STOP_M` acts on the tick it is produced, mid-sweep. Requiring
confirmation across sweeps would add 0.8–1.4 s to the stop and blow the latency
budget of §3.3 entirely.

### 8.8 Telemetry

Append one block to the existing 100 ms line, in the established style:

```
| SCAN <state> AZ <deg> N <raw> D <cm> Z <cm> XN <cm> AZN <deg> CAP <rev/s> AGE <ms>
```

`N` (raw counts) is there on purpose: every calibration and every noise
investigation in §10 is done in raw counts, and a converted number hides a
rail problem as a plausible-looking distance.

### 8.9 Safety and state

* **Disarm, fall, or link loss → `PARK` immediately**, on the same path that
  cuts the motors. Servo to 0°, map invalidated, `rangeVelCap` reset to
  `DRIVE_MAX_VEL`. A servo sweeping while the robot is being carried is noise
  on the analog line and a chewed cable.
* **The scanner is enabled by a spare TX16S channel** (a free switch — CH5/SA
  is the obvious candidate; CH6/SB is arm, CH7/SC is gain select, CH8/S1 is the
  tuning knob). It **defaults to disabled**. A feature that can refuse the
  pilot's stick must be switchable off in flight without reflashing.
* **Never caps reverse.** There is no rear sensor, and the pilot must always be
  able to back out of whatever the scanner has decided about the space ahead.
* `RANGE_SCANNER 0` compiles the whole thing out, including the hook in §9.

---

## 9. Integration with the cascade

> ### The one rule
>
> **The scanner may only reduce forward `targetVel`. It may never command a
> lean, never write `leanCmd` / `velocityLeanI` / `posSetpoint`, and never
> touch PWM.**

This is not style. From the README: *lean is an output, not a command* — a lean
angle is an **acceleration** command, and every drive feature that has failed in
this repo failed by bypassing the cascade and commanding lean or PWM directly. A
scanner that "leans back to brake" would be the next entry on that list.

Injecting at the velocity setpoint is enough, because the cascade already brakes
correctly on its own: with `targetVel` forced to 0 while the robot is rolling
forward, `velError = 0 - forwardVel` goes negative, `velocityLeanI` winds
negative, the outer loop derives a rearward lean, and the inner PD chases it.
`posSetpoint += targetVel*DT` also stops advancing, so position hold freezes the
target where the robot is. Braking, position hold and the return to neutral all
come free from the existing structure. No new mechanism is needed and none
should be added.

### 9.1 The trap: where the cap is applied

`balance_v2.ino:2500` reads:

```c
bool driveDemand = fabs(targetVel) > 0.001f;
```

and `driving` (with its release debounce) gates a full state wipe —
`posSetpoint = positionRev` and `velocityLeanI = 0` — plus the `DRIVE_KP` /
`DRIVE_KD` gain schedule.

So if the cap is applied *upstream* of that line, a scanner stop looks
identical to the pilot letting go of the stick. After `DRIVE_RELEASE_TICKS` the
robot re-anchors `posSetpoint` and **zeroes `velocityLeanI` in the middle of the
braking manoeuvre — dumping precisely the integral that was producing the brake
lean**, and dropping back to the non-driving gains at the same moment.

The fix is placement, not logic. Keep the pilot's demand separate from the
commanded value:

```c
// existing line ~2455, renamed:
float pilotVel = 0.0f;
if (driveIn != 0.0f) { ... pilotVel = DRIVE_SIGN * ... ; }

float targetVel = pilotVel;
#if RANGE_SCANNER
if (targetVel > 0.0f && targetVel > rangeVelCap) targetVel = rangeVelCap;
#endif

// ... and the drive state machine tests the PILOT, not the capped command:
bool  driveDemand    = fabs(pilotVel) > 0.001f;      // was fabs(targetVel)
int8_t driveDirection = pilotVel > 0.0f ? +1 : -1;   // was targetVel
```

Everything downstream of the state machine — `posSetpoint` integration,
`velError`, the burst log — keeps using `targetVel`, which is now the capped
value. `driving` stays true through a scanner stop, so the integral survives,
the gain schedule stays on `DRIVE_KP`/`DRIVE_KD` where it belongs mid-brake, and
the position target is not silently re-anchored.

### 9.2 What must not change

* `DRIVE_MAX_VEL` is the ceiling the cap scales; the cap never exceeds it.
* No change to `LEAN_LPF`, `VEL_I_CLAMP`, `POS_ERROR_CLAMP`, `Kpos`, `Kvel`,
  `KVEL_I`, or any inner-loop gain. If a scanner stop needs the tune changed,
  the scanner is wrong, not the tune.
* Turn (`TURN_AUTHORITY`) is untouched. It is an open-loop effort differential
  with no velocity feedback for the cap to act on, and the scanner has no
  information about what a pivot will sweep into.

---

## 10. Bench procedures and acceptance tests

Run them in this order. Each one produces a number that a later step depends on.

### 10.1 `RANGE_TEST` — calibration curve, and confirm the part

Motors off. Servo parked at 0°. Stream raw counts, volts, and the seeded cm at
10 Hz.

1. Flat matte white card, perpendicular to the optical axis, on a tape measure.
2. Record raw counts at **10, 15, 20, 30, 40, 60, 80 cm**, plus **100 cm** to see
   where the curve flattens. Let each reading settle for 2 s and record the
   median of the stream, not one line.
3. Re-fit `V = A/(d + C)` on the 10–80 cm points and replace the `25040`
   constant in §8.4.
4. **Part check:** the response must be visibly flat past ~80 cm and must reach
   ~2.3 V at 10 cm. A curve still climbing at 100 cm means the unit is a
   `2Y0A02` (20–150 cm); a curve saturated by 30 cm means a `2Y0A41` (4–30 cm).
   Both look identical to a 2Y0A21 from the outside.
5. **Also record the fold-back:** bring the card in from 10 cm to 3 cm in 1 cm
   steps. Note the peak count and confirm `RANGE_ADC_BLOCKED` sits below it. The
   whole rising flank must be inside the BLOCKED band.

### 10.2 `BEAM_TEST` — measure the angle

**This is the test that answers "what is the angle of this thing", and there is
no substitute for it, because Sharp does not publish the number (§2.1).**

1. Sensor rigidly clamped, optical axis horizontal, servo parked at 0°.
2. Flat card at exactly `D` = 50.0 cm, perpendicular. Confirm a stable ~50 cm.
3. Remove the card. Substitute a narrow vertical target of known width
   `w` — a 10 mm dowel is ideal — at the same `D` = 50.0 cm, on the axis.
   Confirm it still reads ~50 cm. **Nothing else within 1.5 m behind it.**
4. Translate the target laterally in 2 mm steps across the axis, recording the
   reading at each. Note `x_left` and `x_right`, the positions where the reading
   stops tracking the target and jumps to "no return".
5. Full effective beam angle, with the target's own width removed:

   ```
   alpha = 2 * atan( ((x_right - x_left) - w) / (2*D) )
   ```

   Expected span at 50 cm: roughly 4–5 cm plus `w`, giving alpha ≈ 4.6–5.7°.
6. **Repeat at `D` = 25 cm and `D` = 75 cm.** If `alpha` comes out constant, the
   beam is a cone from a point and the §2.3 tables apply as written. If the
   *span* is constant instead, the beam is collimated and the coverage
   arithmetic must be redone against a fixed spot width.
7. **Repeat vertically** (translate the target up and down). The emitter and PSD
   are separated along the package's long axis, so there is no reason to assume
   the response is circular, and §7.5 depends on knowing whether it is.

Feed the result back into: `SCAN_STEP_DEG` (must be ≤ alpha, §3.1), the
coverage tables in §2.3, and `FLOOR_REJECT_M` via the `alpha/2` term in §4.3.

### 10.3 `SERVO_CAL` — endpoints and scale

Never assume 1000/2000 µs, and never use `Servo::write(degrees)`.

1. Sensor mounted, cable dressed as it will be in service (§7.4).
2. Type raw microsecond values over Serial. Find the two values at which the
   horn reaches the **mechanical** limits of the mount, back off 100 µs from
   each, and record those as the usable endpoints.
3. Command a known travel (e.g. centre ±40°-worth) and measure the actual angle
   with a protractor against the mount. Compute `SERVO_US_PER_DEG` from the
   measurement.
4. Confirm `SERVO_LIMIT_DEG` = 60° keeps total cable twist ≤ 120° (§7.4).
5. Watch for jitter with the servo holding still. Jitter here is either an
   unlatched 3.3 V signal (§5.4) or a supply problem (§5.2) — it is not
   something to filter in software.

### 10.4 Pitch alignment — `RANGE_PITCH_OFFSET_DEG` and `MOUNT_HEIGHT_M`

1. Robot free-standing and **disarmed**, at rest, undisturbed. Read `PITCH` off
   the telemetry — this is the attitude the robot actually stands at.
2. With the robot held at that attitude, shim the sensor mount until the beam
   is horizontal (aim at a mark on a wall at the measured lens height, 2 m away;
   the beam is IR, so use a phone camera without an IR filter to see the spot,
   or sight along the package).
3. Measure the lens height above the floor at that attitude → `MOUNT_HEIGHT_M`.
4. Any residual you could not shim out goes in `RANGE_PITCH_OFFSET_DEG`,
   positive = nose-down. **Do not shim it into `BALANCE_SETPOINT`.**
5. Verify: with the robot tipped by hand to a known pitch, the reported `Z` of a
   fixed target must stay constant. If `Z` moves with pitch, the offset is
   wrong or the mount is compliant (M3/M4).

### 10.5 Package roll orientation A/B (§7.5, currently undecided)

1. Vertical edge target: the corner of a box at 50 cm, with the next surface
   ≥ 1 m behind it.
2. Sweep across the edge in 1° steps and record the reported range at each
   bearing. The blend region — where the reading is neither the near nor the far
   surface — is the error.
3. Roll the package 90° (triangulation baseline vertical), repeat.
4. Adopt whichever orientation gives the narrower blend region and record the
   measurement here. **If the difference is under one step's worth of bearing,
   record that it does not matter and stop worrying about it.**

### 10.6 `SWEEP_TEST` — dwell adequacy and backlash

Motors off, robot clamped. One `az,cm` scan dumped per sweep.

* **Dwell:** point at a flat wall square-on. The scan should be a smooth arc.
  Angular smearing at the edges of objects, or ranges that depend on which
  bearing was sampled *before* them, means `SCAN_SETTLE_TICKS` or
  `SCAN_SAMPLE_TICKS` is too short (§3.2).
* **Disagreement rate:** log how often `SCAN_EXTRA_TICKS` is granted. Persistently
  above ~20 % of points means the sample window is landing on sensor updates too
  often — lengthen `SCAN_SAMPLE_TICKS` and accept the slower sweep.
* **Backlash:** temporarily allow a bidirectional sweep and put a narrow target
  at a known bearing. The bearing difference between the two sweep directions is
  the backlash. Then **turn bidirectional back off** (§7.3) and confirm the
  artefact is gone.
* **Timing:** confirm the real sweep period matches the §3.3 table. If it
  doesn't, the table's latency budget in §3.3 is wrong and `DRIVE_MAX_VEL` needs
  re-examining against the real number.

### 10.7 Integration acceptance — the four guards

Nothing here is optional; each one covers a way this feature can silently break
something already working.

| # | Test | Pass criterion |
|---|---|---|
| **A1** | **Re-run `DEADBAND_TEST`** (loaded, wheels on the floor, robot upright, steadied by hand at the top) | Floors still measure 10/10 and 18/18. Anything else means the motor PWM frequency moved — §6, Trap 2 |
| **A2** | Watch `HZ` in the telemetry with the scanner running | Still 200. The tick must not stretch — §6, Timing |
| **A3** | Re-measure the rest pitch free-standing and disarmed | `BALANCE_SETPOINT` updated for the added mass — §3.4 |
| **A4** | **Ground-noise test.** Robot propped, wheels free, a fixed target at 60 cm, servo parked. Arm and sweep the drive stick through its full range | Reported range moves **< 2 cm** across the full PWM range. Range that tracks motor effort means motor return current is sharing the sensor's ground conductor — §5.1, requirement E1 |

A4 is the one that gets skipped and the one that matters. 21 cm of
effort-correlated range error (§5.1) does not look like a wiring fault in the
telemetry — it looks like a world that moves when you drive.

### 10.8 First drive

Scanner switch **off** (§8.9). Drive normally, confirm nothing changed. Then
switch the scanner on with the robot stationary and an obstacle at 1 m, and walk
the obstacle in while watching `XN` and `CAP`. Only then drive at it, slowly,
with a hand on the arm switch.

---

## 11. Known limitations

State these in the commit message and in any README paragraph. A sensor whose
limits are undocumented gets trusted by the next person.

* **Thin objects read as clear.** A chair leg is ~3 cm wide; at 60 cm the beam
  spot is ~5 cm, so the target only partially fills it. The PSD reports the
  centroid of what comes back, and a weak partial return frequently reads as no
  return at all — i.e. **"clear"**. This is the most likely real-world miss.
* **Dark and matte surfaces.** Triangulation is far less reflectance-dependent
  than intensity ranging, but a matte black surface at 80 cm can still fail to
  return enough light. It fails toward "clear".
* **Specular surfaces are invisible.** Glass doors, mirrors, a polished floor at
  a grazing angle: the beam leaves and does not come back.
* **Sub-10 cm targets read as far** (§8.4 fold-back). Mitigated mechanically by
  M7, not eliminated.
* **No vertical coverage.** One horizontal slice at one height. Steps,
  thresholds, floor cables, low shelves, table edges: invisible. With the sensor
  at 0.30 m and ±15° of pitch, obstacles shorter than ~15 cm are unreliable at
  60 cm even within the swept plane (§4.3).
* **The map goes stale.** 0.4–1.3 Hz per sweep; at full speed the robot covers
  31–103 cm between looks at the same bearing (§3.3).
* **No pose estimate — though the ingredients exist.** Yaw *rate* is available
  twice over and unused: `imu.getGyroZ()` is read every tick, and
  `rotationVel = 0.5*(wheelVelL - wheelVelR)` is already computed at line 2216.
  Neither is integrated into a heading, and nothing estimates translation.
  Note the asymmetry if you ever do: balance corrections are **common-mode**
  wheel motion, so differential (yaw) odometry is comparatively clean while
  `positionRev` is corrupted by every catch and lean the balance loop makes.
  As shipped this is a reactive corridor check, not a map.
* **It cannot see the pivot.** The scanner has no authority over turn (§9.2) and
  no information about what a pivot sweeps into.
* **Direct sunlight degrades it.** Indoor sensor. Not a limitation for this
  robot, but say it anyway.

> **The arm switch remains the safety system.** This sensor is a driver assist
> on a platform that can already be stopped instantly by the pilot. Nothing
> above should be described to anyone as collision *avoidance*.

---

## 12. Open questions

Things this spec does not settle, listed so they get measured rather than
assumed:

1. **The beam angle itself.** §2.2 gives 4–5° as a bounded working assumption
   from optical geometry and reported spot sizes. §10.2 replaces it with a
   measurement. Until then, every coverage and step-size number in §2.3 and
   §3.1 is provisional.
2. **Whether the beam is a cone or collimated** over 25–80 cm (§10.2 step 6).
   Changes the coverage arithmetic if it is the latter.
3. **Whether the response is circular** or elongated along the triangulation
   baseline (§10.2 step 7) — which feeds directly into §7.5.
4. **Package roll orientation** (§7.5 / §10.5). Untested; do not claim either
   way in the meantime.
5. **The real robot mass and CoM height.** §3.4 uses 1.5 kg at 0.15 m as
   estimates. The conclusion (added mass is negligible and slightly helpful) is
   robust to a factor of two, but `BALANCE_SETPOINT` must still be re-measured.
6. **Corridor half-width.** `CORRIDOR_HALF_W_M` = 0.10 m is a placeholder;
   measure the track and add margin.
7. **`DRIVE_MAX_VEL` under scanner authority.** §3.3 shows the latency chain
   consuming ~45 cm of the 80 cm range at 0.90 rev/s. Whether that margin is
   acceptable, or whether the scanner should cap the ceiling itself, is a
   decision to make after §10.6 gives the real sweep period.
8. **Second sensor?** A fixed downward-looking 2Y0A41 (4–30 cm) would cover
   cliffs and low obstacles — the axis this design explicitly does not cover
   (§3.5). Out of scope here; noted so the gap is not mistaken for an oversight.

---

## References

* [Sharp GP2Y0A21YK0F datasheet (Sharp)](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf)
  — range, output curve, timing, marking scheme, bypass capacitor note. Contains
  no beam-angle specification.
* [Sharp GP2Y0A21YK0F datasheet mirror (Pololu)](https://www.pololu.com/file/0j85/gp2y0a21yk0f.pdf)
* [Pololu product page — GP2Y0A21YK0F](https://www.pololu.com/product/136)
* [SparkFun — Infrared Proximity Sensor GP2Y0A21YK](https://www.sparkfun.com/infrared-proximity-sensor-sharp-gp2y0a21yk.html)
* [Makerguides — GP2Y0A21YK0F with Arduino](https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/)
  — linearisation fits and wiring practice.
* [CONTROL_THEORY.md](CONTROL_THEORY.md) — the cascade this must not bypass.
* [README.md](README.md) — pinout, current tune, and the bare-threshold rule.
