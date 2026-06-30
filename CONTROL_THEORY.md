# Control Theory & Robotics Principles — Self-Balancing Robot

An in-depth walkthrough of *every* principle this robot uses: the physics of the
inverted pendulum, how we estimate its state from cheap sensors, and how the
cascaded controller keeps it upright and in place. Formulas are given with plain
explanations so you can follow it without a controls course — but it's rigorous
enough to be the real thing.

Cross-reference the code in [`balance_v2/balance_v2.ino`](balance_v2/balance_v2.ino);
every symbol below maps to a variable there.

---

## Table of contents
1. [The plant: a wheeled inverted pendulum](#1-the-plant-a-wheeled-inverted-pendulum)
2. [Equations of motion](#2-equations-of-motion)
3. [Why it's hard: instability, underactuation, non-minimum-phase](#3-why-its-hard)
4. [Sensing & state estimation](#4-sensing--state-estimation)
5. [The complementary filter](#5-the-complementary-filter)
6. [Odometry: encoders, velocity, position](#6-odometry-encoders-velocity-position)
7. [PID and the inner balance loop](#7-pid-and-the-inner-balance-loop)
8. [The actuator: DC motor, PWM, stiction](#8-the-actuator-dc-motor-pwm-stiction)
9. [Cascade control: the outer loop](#9-cascade-control-the-outer-loop)
10. [Full-state feedback & LQR (the bigger picture)](#10-full-state-feedback--lqr)
11. [Discretization, timing, and filtering](#11-discretization-timing-and-filtering)
12. [Safety & practical engineering](#12-safety--practical-engineering)
13. [Symbol & parameter reference](#13-symbol--parameter-reference)
14. [Further reading](#14-further-reading)

---

## 1. The plant: a wheeled inverted pendulum

In control language, the thing you're controlling is the **plant**. Ours is a
**two-wheeled inverted pendulum** (TWIP): a body whose center of mass (CoM) sits
*above* the wheel axle. Left alone it falls — the upright position is an
**unstable equilibrium**, like balancing a broom on your palm.

The controller's job: measure how the body is tilting and move the wheels so the
contact patch stays under the falling CoM. You don't *stop* it falling; you
continuously *catch* it.

Key physical quantities:

| Symbol | Meaning | In code |
|---|---|---|
| $\theta$ | tilt angle from vertical (pitch) | `pitch` |
| $\dot\theta$ | tilt rate (angular velocity) | `gyroRate` |
| $x$ | wheel/chassis position along the floor | `positionRev` (in wheel-revs) |
| $\dot x$ | chassis forward velocity | `forwardVel` |
| $m, l$ | body mass, axle→CoM distance | physical |
| $g$ | gravity, $9.81\ \mathrm{m/s^2}$ | — |

The **full state** of the plant is the 4-vector $[\,\theta,\ \dot\theta,\ x,\ \dot x\,]$.
Everything the controller does is a function of these four numbers.

---

## 2. Equations of motion

### 2.1 The bare pendulum

Ignore the wheels for a second. A point mass on a massless rod of length $l$,
tilted by $\theta$ from vertical, feels gravity trying to topple it. Newton's law
for rotation about the pivot ($I\ddot\theta = \tau$) gives:

$$ \ddot\theta = \frac{g}{l}\sin\theta \approx \frac{g}{l}\,\theta \quad(\text{small }\theta)$$

That **linearization** $\sin\theta\approx\theta$ (valid to ~1% below ~15°) is what
lets us use linear control. Solving $\ddot\theta = (g/l)\theta$ gives exponentials
$e^{\pm t/\tau}$ with

$$ \tau = \sqrt{\frac{l}{g}} \qquad\text{(the natural fall time constant)} $$

For our robot $l \approx 0.10$–$0.15\ \mathrm{m}$, so $\tau \approx 0.10$–$0.12\ \mathrm{s}$.
**This single number drives the whole design:** the robot falls on a ~0.1 s
timescale, so the controller must sense and react many times faster than that.
That's why the control loop runs at **200 Hz** (every 5 ms) — see §11.

### 2.2 Adding the cart (the wheels)

Now let the pivot move: the wheels apply a horizontal force $F$ (from motor
torque $\tau_m$ over wheel radius $r$, $F = \tau_m/r$) to the base. The body and
base are coupled. The standard linearized cart-pole equations (body mass $m$,
base mass $M$, CoM height $l$) are:

$$ (M+m)\,\ddot x + m l\,\ddot\theta = F $$
$$ m l\,\ddot x + m l^2\,\ddot\theta - m g l\,\theta = 0 $$

The top line is "horizontal force = total mass × accel + reaction from the
swinging body." The bottom line is the torque balance on the body. The
$-mgl\,\theta$ term is gravity feeding the instability.

You don't need to solve these by hand — the takeaways are structural and appear in
§3. (We also never plug in exact $m, l$ values: we tune the controller gains
empirically instead. The equations tell us the *shape* of the problem; tuning
fills in the numbers.)

---

## 3. Why it's hard

Three properties of this plant shape every design decision.

### 3.1 It's open-loop unstable
$\ddot\theta = (g/l)\theta$ has solutions $e^{+t/\tau}$ and $e^{-t/\tau}$. The
$e^{+t/\tau}$ term means any tiny tilt grows **exponentially** — a pole in the
right half of the complex plane. There is no "do nothing" option; without active
control it falls, period. Feedback must move that pole into the left half-plane.

### 3.2 It's underactuated
We have **one** actuator axis (drive the wheels) but **two** things to control
(stay upright *and* hold position). You can't independently command both — driving
the wheels to fix position necessarily disturbs the tilt, and vice-versa. This is
why a naive "PID on angle + PID on position" fights itself, and why the **cascade**
(§9) — which deliberately *uses* tilt to control position — is the right structure.

### 3.3 It's non-minimum-phase
This is the subtle, counter-intuitive one, and it bit this project repeatedly. The
transfer function from wheel force to *position* has a **zero in the right
half-plane**. In plain terms:

> **To move the chassis forward, the wheels must first roll *backward*.**

Drive the wheels backward → the contact point slips back → the body tips *forward*
→ now the body is falling forward and you drive forward to chase it → net result:
the robot moves forward. The initial motion is **opposite** to the goal.

Consequences you can feel:
- A position controller that naively "drives the wheels toward home" pushes the
  body the wrong way and can **run away** (we saw exactly this — dragging the robot
  made it accelerate off in the drag direction).
- The fix isn't a sign tweak on a motor command; it's to command a **lean** and
  let the fast inner loop convert that lean into the correct (counter-intuitive)
  wheel motion. That's the cascade, §9.

---

## 4. Sensing & state estimation

We must recover $[\theta, \dot\theta, x, \dot x]$ from sensors. We have an
**IMU** (MPU-9250: 3-axis accelerometer + 3-axis gyroscope) for the tilt states
and **wheel encoders** for the position states. Neither sensor gives a clean
answer alone.

### 4.1 Accelerometer → absolute tilt (noisy, no drift)
At rest the accelerometer measures the gravity vector. Its direction in the
sensor frame tells you the tilt:

$$ \theta_{\text{accel}} = \operatorname{atan2}(a_x,\ a_z) $$

(`atan2` gives the correct angle in all quadrants.) In code:

```c
float accelPitch() {
  float a = atan2(imu.getAccX(), imu.getAccZ()) * 180.0f / PI;
  return a - PITCH_ZERO_OFFSET;   // trim so a level robot reads ~0
}
```

- ✅ **Absolute** — referenced to gravity, so it never drifts.
- ❌ **Noisy and fooled by motion.** The accelerometer measures gravity *plus* any
  real acceleration. When the robot accelerates/vibrates, that horizontal $a$
  corrupts the angle. So it's truthful *on average* but jittery moment-to-moment.

### 4.2 Gyroscope → tilt rate (clean, but drifts when integrated)
The gyro measures angular **velocity** $\dot\theta$ directly. Integrate it to get
angle:

$$ \theta_{\text{gyro}}(t) = \theta_0 + \int_0^t \dot\theta\,dt $$

- ✅ **Smooth and immune to linear acceleration** — exactly the fast signal we need.
- ❌ **Drifts.** Any constant bias $b$ in the rate integrates into an
  ever-growing angle error $b\cdot t$. We subtract the bulk bias (`GYRO_Y_BIAS`),
  but residual drift means raw gyro integration is untrustworthy over seconds.

```c
float gyroRate = -(imu.getGyroY() - GYRO_Y_BIAS);   // pitch rate is on the Y gyro, negated
```

> **War story — which axis is "pitch"?** It is *not* obvious, and it is not
> stable across library versions. On this MPU9250 build the pitch-axis rate reads
> on **`getGyroY()`, inverted** — not the X axis you'd guess. A library update
> silently moved it; the code kept reading `getGyroX()`, which now returned ≈0. The
> symptom was maddening: the robot balanced *slowly* (on the accelerometer alone)
> but **exploded the instant it was pushed** — because with a zero rate signal there
> was no damping (D term) and no fast term in the filter. Every downstream tuning
> mystery traced back to this. Lesson: when you change IMU or library, **verify the
> axis empirically** (the `IMU_TEST` mode streams all three gyro channels while you
> hand-tilt) before trusting any gain.

The two sensors are **complementary**: the accelerometer is right at *low
frequencies* (long term), the gyro is right at *high frequencies* (short term).
Fuse them.

---

## 5. The complementary filter

The cheapest good fusion. Each tick:

$$ \theta_k = \alpha\,(\theta_{k-1} + \dot\theta_k\,\Delta t) \;+\; (1-\alpha)\,\theta_{\text{accel},k} $$

In code (`α = 0.99`, `Δt = 5 ms`):

```c
pitch = 0.99f * (pitch + gyroRate * DT) + 0.01f * accelPitch();
```

What it does, frequency-wise:
- The $\alpha(\theta_{k-1}+\dot\theta\,\Delta t)$ part **high-pass filters the gyro**
  (trusts fast changes).
- The $(1-\alpha)\theta_{\text{accel}}$ part **low-pass filters the accelerometer**
  (trusts slow truth, averages out the jitter).

They cross over at the time constant

$$ \tau_c = \frac{\alpha\,\Delta t}{1-\alpha} = \frac{0.99 \times 0.005}{0.01} \approx 0.5\ \text{s} $$

So **above ~0.5 s** the estimate follows the accelerometer (no drift), **below
~0.5 s** it follows the gyro (smooth, fast). Since the robot falls on a ~0.1 s
timescale, the *control-relevant* band is gyro-dominated — fast and clean — while
the accelerometer quietly corrects long-term drift. That's why $\alpha$ is so close
to 1.

> A **Kalman filter** does the same fusion *optimally*, modeling the noise
> statistics and gyro bias as states. It's better (it can estimate and subtract the
> drifting bias), but heavier and harder to tune. The complementary filter is a
> fixed-gain Kalman filter — 90% of the benefit for 5% of the effort, perfect for
> an 8-bit AVR.

**Caveat we exploit in §12:** during a fall/handling, the gyro-dominated estimate
*drifts* (you're rotating the robot, the 1% accel pull lags). So for the fall
*safety latch* we re-check against the raw accelerometer, which can't drift.

---

## 6. Odometry: encoders, velocity, position

The wheels carry **magnetic Hall quadrature encoders** (Waveshare DCGM-3865).

### 6.1 Quadrature & resolution
Two Hall channels (A, B) output square waves 90° out of phase. The phase order
tells you direction; counting edges tells you distance. Base resolution is
**13 pulses per motor-shaft revolution**, and the gearbox multiplies by **42:1**:

$$ \text{counts/output-rev} = 13 \times 42 = 546 \quad(\text{counting rising edges of A}) $$

We count rising edges of A only (half-quadrature) and read B inside the interrupt
for direction — the Uno has just two hardware-interrupt pins, enough for one
channel per wheel. Full 4× quadrature would give $13\times4\times42=2184$ counts/rev
but needs four interrupt pins.

```c
void leftEncISR()  { leftTicks  += ENC_LEFT_DIR  * (digitalRead(LEFT_ENC_B)  ? -1 : +1); }
```

### 6.2 Position (integrating counts)
Average the two wheels and convert to revolutions from a "home" reference:

$$ x = \frac{(\text{leftTicks}+\text{rightTicks})/2 \;-\; \text{home}}{546} \quad[\text{rev}] $$

```c
positionRev = 0.5f * ((l + r) - homeTicksSum) / ENC_COUNTS_PER_REV;
```

### 6.3 Velocity (differentiating counts)
Velocity is the derivative of position: counts per tick, scaled. But raw
per-tick differences are **coarse** (at 5 ms, a slow roll is <1 count), so the
signal is quantized and noisy. We low-pass it with an **exponential moving
average (EMA)**:

$$ v_k = \beta\,v_{k-1} + (1-\beta)\,v_{\text{raw},k}, \qquad v_{\text{raw}} = \frac{\Delta\text{counts}}{546\,\Delta t} $$

```c
wheelVelL = VEL_LPF * wheelVelL + (1.0f - VEL_LPF) * vL_raw;   // VEL_LPF = 0.90
forwardVel = 0.5f * (wheelVelL + wheelVelR);
```

An EMA with weight $\beta$ has time constant $\tau = \Delta t\,\beta/(1-\beta)
\approx 45$ ms here — enough to smooth quantization without lagging the control
loop badly. (Same math as the complementary filter, just applied to velocity.)

> **Asymmetry note:** the two wheels count slightly differently, so the robot
> veers as it drives. On the bench this is mostly the **USB tether dragging one
> side** (a test-rig artifact that disappears once it runs untethered on battery),
> not a real friction imbalance. Averaging hides it from the balance loop anyway; a
> future yaw controller would use the *difference* `(rightTicks − leftTicks)` to
> steer straight.

Now we have all four states: $\theta, \dot\theta$ from the IMU, $x, \dot x$ from
the encoders.

---

## 7. PID and the inner balance loop

### 7.1 PID in one paragraph
A **PID controller** drives an error $e$ (desired − measured) to zero with three
terms:

$$ u(t) = \underbrace{K_p\,e}_{\text{Proportional}} + \underbrace{K_i\!\int e\,dt}_{\text{Integral}} + \underbrace{K_d\,\frac{de}{dt}}_{\text{Derivative}} $$

- **P** — push proportional to how wrong you are. More P = stiffer, faster, but
  too much oscillates.
- **I** — accumulate past error to kill steady-state offset. Powerful but adds lag
  and can "wind up."
- **D** — react to the *rate* of error — anticipation/damping. Reduces overshoot,
  but amplifies sensor noise (it differentiates).

### 7.2 Our inner loop is a PD on tilt
The balance error is the tilt away from the target lean:

$$ e = \theta - \theta_{\text{sp}} $$

```c
float u = -(Kp * error + Kd * dRateFilt + Ki * integral);
```

Two elegant things happen here:

**(a) We never differentiate to get D.** Normally the D term needs $de/dt$, a
noisy numerical derivative. But $e = \theta - \theta_{\text{sp}}$, so
$\dot e = \dot\theta$ = the **gyro reading we already have**. The D term is just
`Kd * gyroRate` — measured, not computed. This is a big practical win (the gyro is
much cleaner than a finite-difference derivative).

**(b) The sign.** $u = -(K_p e + \dots)$: a forward lean ($e>0$) produces a
correcting drive to catch it. (The exact motor polarity is a wiring detail; see
the firmware comments.)

`Ki` is kept at **0** — integral action on tilt invites windup, and the outer
loop (§9) handles standing bias far more cleanly.

### 7.3 Filtering the derivative — and why `Kd` is *small* here
D amplifies noise, and these geared motors **couple vibration into the gyro**: at
rest, the motors buzzing at the deadband floor make the rate signal swing ±35°/s
while the body barely moves ±1°. That phantom is fast and large. Two layers tame
it before it reaches the D term:

1. **On-chip low-pass.** The MPU9250 has a hardware digital low-pass (DLPF); we set
   the gyro to **20 Hz** (from the 41 Hz default). The balance dynamics live below
   ~5 Hz, so this costs negligible phase lag but knocks down the motor-vibration
   band at the sensor.
2. **A software low-pass on the rate used by the D term only:**

$$ \dot\theta_{\text{filt},k} = \gamma\,\dot\theta_{\text{filt},k-1} + (1-\gamma)\,\dot\theta_k $$

```c
dRateFilt = D_LPF * dRateFilt + (1.0f - D_LPF) * gyroRate;   // D_LPF = 0.6 (~7 ms)
```

The **angle estimate still uses the raw rate** (we want it fast); only the damping
term is smoothed.

> **Counter-intuitive result: `Kd = 0.3`, not 3–5.** Most balancer tunes carry a
> hefty `Kd`. Here, *raising* `Kd` made things **worse** — a violent limit cycle —
> because the rate still carries a fast lagged component, and on a lag-sensitive
> plant a big derivative gain stops damping and starts *pumping*. We re-tuned D from
> scratch (after fixing the gyro axis, §4.2, which had made the old `Kd≈4.5`
> meaningless) and found the stable value is **low**. The real cure for the
> vibration is mechanical (a foam/rubber IMU mount); with that, `Kd` could rise
> again for crisper recovery. Until then: small `Kd` + the coast band (§7.4).

### 7.4 The coast band: resting the motors when settled
Near perfect balance, tiny corrections just chatter the motors — and because of
the deadband feed-forward (§8.2) each chatter is a full ~11-PWM kick. That's
continuous current, heat, battery drain, and wear, even standing still. So when
the robot is within a band of the target **and** the wheels are stopped, force the
output to exactly zero and let the motors rest:

```c
if (fabs(error) < ANGLE_DEADZONE && fabs(forwardVel) < VEL_DEADZONE) {
  u = 0; integral = 0;          // ANGLE_DEADZONE = 1.0 deg
}
```

Two design choices matter:

- **We gate on `error` and `forwardVel`, *not* on `gyroRate`.** The rate is
  corrupted by motor vibration (§7.3) — gating the coast on it would mean it could
  *never* engage (the phantom ±35°/s always exceeds any sane threshold). The
  filtered pitch and the wheel velocity are the trustworthy "settled" signals.
- **The band is wide (~1°).** Tight bands never let the motors rest.

> **The elegant part — the jitter was self-sustaining.** The buzz fed itself:
> motors twitch → vibration → the gyro reads it as motion → the controller twitches
> the motors. Cutting the motors at balance *breaks the loop* — the vibration stops,
> the gyro goes quiet (`RATE` falls from ±35 to ≈0), and it stays settled with
> `U = 0`. The coast band didn't just *save* the motors; it **dissolved** the
> jitter. The trade is a gentle ~±1° rock (the pendulum drifts in the dead band
> until it nudges the edge) in exchange for silence — an excellent deal here.

---

## 8. The actuator: DC motor, PWM, stiction

The controller's output `u` must become motor voltage. Two real-world effects
dominate.

### 8.1 PWM and the H-bridge
We can't output an analog voltage, so we **pulse-width modulate**: switch the
supply on/off fast, and the *duty cycle* (0–255) sets the average voltage. An
**IBT-2 (BTS7960) H-bridge** lets us drive each motor in either direction (forward
pin vs reverse pin). To first order, **PWM duty ∝ motor voltage ∝ motor speed**
(at constant load).

### 8.2 Stiction and the deadband feed-forward
A geared DC motor doesn't move at all until the PWM overcomes static friction
(**stiction**) in the motor + gearbox. Below that threshold, commanding "drive a
little" does nothing — a **dead band** in the actuator. For a balancer that's
fatal: small corrections vanish and the robot can't catch a gentle lean.

The fix is **feed-forward deadband compensation** — jump straight past the dead
band whenever we want *any* motion:

$$ \text{PWM} = \operatorname{sign}(u)\,\big(\text{MOTOR\_DEADBAND} + |u|\big), \quad\text{clamped to MAX\_PWM} $$

```c
void driveControl(float u) {
  if (fabs(u) > OUT_DEADZONE) {
    int pwm = constrain(MOTOR_DEADBAND + fabs(u), 0, MAX_PWM);
    motorRaw(u < 0 ? -pwm : pwm);
  } else motorRaw(0);
}
```

We **measured** `MOTOR_DEADBAND` directly (the `DEADBAND_TEST` mode ramps PWM and
watches the encoders for first motion: ~11–13 here). Setting it too low → small
corrections stall (can't catch itself); too high → a discontinuous "kick" through
zero that feeds a limit cycle (we saw both failure modes).

> **Battery & voltage:** PWM maps to *voltage*, not torque. As a 4S LiPo sags from
> 16.8 V → 14 V, the same PWM gives less torque and the effective stiction rises —
> the tune drifts. Proper fix: scale the output by measured battery voltage
> (future work). The motors are rated 12 V; we cap `MAX_PWM` to limit heat/current.

### 8.3 Gear ratio & torque
The 42:1 gearbox trades speed for torque ($\tau_{\text{out}} = 42\,\tau_{\text{motor}}$,
ignoring losses) — essential for the wheels to accelerate the body fast enough to
catch a fall, at the cost of top speed.

---

## 9. Cascade control: the outer loop

The inner PD (§7) keeps the robot at *whatever tilt we ask for*. But we also want
it to **stop drifting and hold position** — and §3 told us we can't just push the
wheels toward home (non-minimum-phase → runaway). 

**The trick:** a balancer accelerates by leaning. So to move/brake, don't command a
wheel motion — command a **lean**, and let the fast inner loop produce the
(counter-intuitive) wheel motion that achieves it. This is a **cascade**: a slow
outer loop generates the setpoint for a fast inner loop.

```
   position x, velocity ẋ ──►  OUTER loop  ──►  lean setpoint  ──►  INNER PD  ──►  motor
        (encoders)              (this §)          (θ_sp)            (§7)
```

### 9.1 The control law
The outer loop is a PD controller **on position** (position error and its
derivative, velocity), whose *output is a tilt command*:

$$ \theta_{\text{lean}} = -\big(K_{\text{pos}}\,x + K_{\text{vel}}\,\dot x\big), \qquad \theta_{\text{lean}} \leftarrow \operatorname{clamp}(\theta_{\text{lean}},\ \pm\text{LEAN\_CLAMP}) $$
$$ \theta_{\text{sp}} = \text{BALANCE\_SETPOINT} + \theta_{\text{lean}} $$

```c
leanRaw = -(Kpos * positionRev + Kvel * forwardVel);
leanRaw = constrain(leanRaw, -LEAN_CLAMP, LEAN_CLAMP);
leanCmd = LEAN_LPF * leanCmd + (1.0f - LEAN_LPF) * leanRaw;   // slow! ~500 ms — see §9.3
float error = pitch - (BALANCE_SETPOINT + leanCmd);
```

### 9.2 Why the signs are now intuitive
With the cascade, the sign reasoning is physical, not algebraic:
- **Moving forward** ($\dot x > 0$) and want to brake → command a **backward** lean
  ($\theta_{\text{lean}} < 0$). The robot tilts back, the inner loop drives to hold
  that backward tilt, and it decelerates. That's the **reverse-kick brake** — and
  it's strong, because a lean commands acceleration through the high-authority
  inner loop, not a weak direct nudge.
- **Behind home** ($x < 0$) → command a **forward** lean → it leans forward, falls
  toward home, and the inner loop chases it home.

Both fall straight out of $-(K_{\text{pos}}x + K_{\text{vel}}\dot x)$: lean
*against* the displacement and *against* the velocity. No sign-flipping voodoo —
which is exactly why this replaced the old, fragile direct-to-motor terms.

### 9.3 Timescale separation: why the outer loop must be *slow*
A cascade only works if the inner loop is *fast* and the outer loop is *slow* — so
the outer loop sees the inner loop as an already-settled servo. Here that's not a
nicety; it's the difference between balancing and bolting across the room.

Recall §3.3: to return home the wheels must first roll the **wrong way**. When the
outer loop commands "lean back to brake," the inner loop's *first* action is to
drive the wheels **forward** (to tip the body back). That momentarily *increases*
the forward velocity. Now watch the feedback path:

> faster forward velocity → outer loop commands *more* back-lean → inner loop drives
> the wheels *more* forward → … **runaway.**

The only thing that stops it is **delay**: the outer loop must wait long enough for
the inner loop to finish the wrong-way transient (the body actually tips back and
the velocity reverses) *before* it reacts. That delay is the outer-loop low-pass
`LEAN_LPF`. The wrong-way transient lasts ~100–300 ms (it's set by the pendulum
time constant $\tau$, §2.1), so:

$$ \tau_{\text{outer}} = \Delta t\,\frac{\text{LEAN\_LPF}}{1-\text{LEAN\_LPF}} \;\gg\; \tau_{\text{wrong-way}} $$

With `LEAN_LPF = 0.99`, $\tau_{\text{outer}} \approx 0.5$ s — comfortably slower than
the transient. We learned this the hard way: an earlier `LEAN_LPF ≈ 0.95`
($\tau \approx 0.1$ s) was *not* slower than the transient, and the robot ran away
on every drag exactly as the feedback chain above predicts. **The cascade structure
was never the problem — the loop was just too fast.** (Earlier still, the whole
cascade had looked hopeless, but that was the dead-gyro inner loop of §4.2; a
cascade on a broken inner loop can't work no matter how you tune the outer one.)

### 9.4 The clamp
`LEAN_CLAMP` caps how far the outer loop may tilt the robot (±5°). It's a hard
safety: even a huge position error can't command a tip-over lean. It also keeps the
outer loop's authority below the inner loop's, reinforcing the timescale
separation of §9.3.

### 9.5 Tuning intuition
- `LEAN_LPF` is the **safety/speed knob** — keep it slow (≥0.99) or §9.3 bites.
  Only lower it if recovery is sluggish, and watch for runaway each step.
- `Kvel` is the **brake / recovery damper** — raise it until a shove/drag is
  arrested crisply. Too high → twitchy; too low → it overshoots home with big
  pitch swings (under-damped return).
- `Kpos` is the **return-home spring** — raise it until position converges back to
  0. Too high → slow oscillation about home; too low → parks off-center.

Because position is the integral of velocity, the $K_{\text{pos}}\,x$ term *is*
integral action on velocity — it drives the steady-state drift to zero without a
separate, windup-prone integrator.

---

## 10. Full-state feedback & LQR

The cascade is a hand-structured version of a more general idea. Write the plant
in **state-space** form with state $\mathbf{s} = [\theta, \dot\theta, x, \dot x]^T$:

$$ \dot{\mathbf{s}} = A\,\mathbf{s} + B\,u $$

where $A$ encodes the linearized dynamics (§2) and $B$ how the motor enters.
**Full-state feedback** chooses

$$ u = -K\,\mathbf{s} = -(k_1\theta + k_2\dot\theta + k_3 x + k_4\dot x) $$

— a single weighted sum of all four states. With the right gain vector $K$, the
closed-loop poles ($A - BK$) all move into the left half-plane → stable. Two ways
to pick $K$:

- **Pole placement:** choose desired closed-loop pole locations, solve for $K$.
- **LQR (Linear-Quadratic Regulator):** define a cost
  $J = \int (\mathbf{s}^T Q\,\mathbf{s} + R\,u^2)\,dt$ that penalizes state error and
  control effort, and solve for the $K$ that minimizes it (via the Riccati
  equation). $Q$ and $R$ are knobs: weight $\dot x$ heavily in $Q$ and you get
  aggressive braking; weight $u$ in $R$ and you get gentler control.

Our controller *is* a full-state feedback law — `Kp, Kd` are $k_1, k_2$ and the
outer loop supplies $k_3, k_4$ — but the gains are **hand-tuned** rather than
LQR-optimal, and we route the position/velocity terms through the setpoint
(cascade) instead of summing directly, which makes the non-minimum-phase signs
manageable and the loop easy to reason about on hardware. Computing an LQR $K$
from a measured/identified model is a natural next step for a smoother,
provably-optimal controller.

---

## 11. Discretization, timing, and filtering

### 11.1 Fixed-rate loop
All the calculus above is continuous-time; a microcontroller runs in discrete
steps. We use a **fixed control period** $\Delta t = 5\ \mathrm{ms}$ (200 Hz):

```c
if ((now - lastControlMicros) < CONTROL_PERIOD_US) return;  // wait for the next tick
lastControlMicros += CONTROL_PERIOD_US;
```

A **constant** $\Delta t$ matters: every integral ($\int e\,dt \to \Sigma e\,\Delta t$),
derivative, and filter time constant depends on it. A jittery loop rate makes the
effective gains wander. We also *measure* the achieved rate (`HZ` in telemetry) to
confirm we're keeping up.

### 11.2 How fast is fast enough?
**Nyquist** says you must sample at least 2× the fastest dynamics, but for
*control* of an unstable plant you want **10–20×** margin. The fall time constant
is ~0.1 s (≈10 rad/s); 200 Hz gives ~20× headroom — comfortably fast to catch the
fall and to run the IMU at 400 kHz I²C for fresh samples.

### 11.3 The filters, unified
Three first-order low-pass filters (EMAs) appear, all the same math
$y_k = \lambda y_{k-1} + (1-\lambda)x_k$ with time constant $\tau=\Delta t\,\lambda/(1-\lambda)$:

| Filter | Variable | $\lambda$ | $\tau$ | Purpose |
|---|---|---|---|---|
| Complementary (angle) | `pitch` | 0.99 | ~0.5 s | fuse gyro + accel |
| D-term rate | `dRateFilt` | 0.60 | ~7 ms | de-spike the derivative (§7.3) |
| Wheel velocity | `wheelVel*` | 0.90 | ~45 ms | de-quantize encoder speed |
| Outer-loop lean | `leanCmd` | 0.99 | ~0.5 s | **slow** the cascade so it can't run away (§9.3) |

Every filter trades **noise rejection** (higher $\lambda$) against **lag** (also
higher $\lambda$). Lag is poison in a fast feedback loop, so each $\lambda$ is the
*minimum* smoothing that makes its signal usable — **except `leanCmd`**, where lag
is the *point*: the outer loop is deliberately made slow to enforce the timescale
separation of §9.3.

There's also a **hardware** low-pass that isn't an EMA: the MPU9250's on-chip
gyro/accel DLPF, set to 20/21 Hz (§7.3). It filters motor vibration at the sensor,
before it ever reaches the firmware.

---

## 12. Safety & practical engineering

The textbook stops at §10; a working robot needs the rest.

### 12.1 Anti-windup
If an integrator keeps accumulating while the output is saturated, it "winds up"
and overshoots badly on recovery. We clamp the integral:
```c
integral = constrain(integral, -40.0f, 40.0f);
```

### 12.2 Fall latch with hysteresis
Past ~30° it can't recover, so cut the motors. But a naive per-tick cutoff
**flails**: while the robot lies on its back being handled, the gyro-dominated
`pitch` estimate *drifts* (§5) and periodically dips back under 30°, re-firing the
motors with the wheels free in the air. Two fixes:

- **Latch + hysteresis:** once tripped, stay off until clearly recovered (don't
  re-engage the instant the estimate wobbles across the threshold).
- **Re-arm on the absolute accelerometer**, not the drift-prone fused estimate —
  the accel can't be fooled while it's lying down:

```c
if (!fallen && fabs(pitch) > FALL_CUTOFF_DEG) fallen = true;
if (fallen && fabs(accelPitch() - BALANCE_SETPOINT) < FALL_REARM_DEG) {
  fallen = false; pitch = accelPitch();   // reseed; only standing it up re-arms
  homeTicksSum = ...;                       // re-home so it doesn't bolt to the old spot
}
```

### 12.3 Stall cutoff — a cautionary tale (now disabled)
The original idea: if the controller commands hard drive ($|u|$ large) but the
wheels **aren't turning** for ~1 s, the robot is wedged and the motors are just
stalling — burning current as heat — so cut them. It rested on the assumption that
*"a balancing wheel always moves while $u$ is large."*

**That assumption is false for a cascade.** The cascade's whole job is to **lean and
hold** — which means a *moderate, steady* $u$ with the wheels essentially still.
That's normal balancing, and the stall detector misread it as a wedge. Worse, the
cutoff **deadlocked**: once it cut the motors, $\dot x$ stayed 0, so the robot fell,
so $|u|$ grew, so the trip condition stayed true *forever* — a one-way trip to the
floor. We **disabled it** (`STALL_CUTOFF 0`).

> **Lessons.** (1) A protection mechanism must not have a state where it guarantees
> the failure it's meant to prevent — always give a latch a way to re-arm from a
> *trustworthy* signal (the fall latch in §12.2 does this right, re-arming off the
> absolute accelerometer). (2) An assumption baked into safety logic ("a balancing
> wheel always moves") can quietly become false when the *controller* changes. A
> correct redesign would trip only near PWM **saturation** with the wheels truly
> stuck, and re-arm off the absolute tilt — not latch on a moderate command.

### 12.4 Clamps everywhere
`MAX_PWM` (current/heat), `LEAN_CLAMP` (tip-over), integral clamp (windup),
`OUT_DEADZONE`/`ANGLE_DEADZONE` (chatter). Real controllers are mostly the core
law plus a lot of carefully-reasoned limits.

---

## 13. Symbol & parameter reference

| Symbol | Code | Meaning | Value |
|---|---|---|---|
| $\theta$ | `pitch` | tilt from vertical (deg) | state |
| $\dot\theta$ | `gyroRate` | tilt rate (deg/s) | state |
| $x$ | `positionRev` | chassis position (wheel-rev from home) | state |
| $\dot x$ | `forwardVel` | chassis velocity (rev/s) | state |
| $\theta_{\text{sp}}$ | `BALANCE_SETPOINT`+`leanCmd` | target tilt | — |
| $K_p$ | `Kp` | inner proportional | 2.0 |
| $K_d$ | `Kd` | inner derivative (damping) | 0.3 |
| $K_i$ | `Ki` | inner integral | 0.0 |
| $K_{\text{pos}}$ | `Kpos` | outer: position→lean (deg/rev) | 1.0 |
| $K_{\text{vel}}$ | `Kvel` | outer: velocity→lean (deg per rev/s) | 4.0 |
| — | `LEAN_CLAMP` | max outer-loop lean (deg) | 5.0 |
| — | `LEAN_LPF` | outer-loop low-pass weight (slow! §9.3) | 0.99 |
| — | `ANGLE_DEADZONE` | coast-band half-width (deg) | 1.0 |
| — | `GYRO_Y_BIAS` | resting `getGyroY()` rate (deg/s) | −9.0 |
| — | gyro/accel DLPF | on-chip hardware low-pass (Hz) | 20 / 21 |
| $\alpha$ | (literal `0.99`) | complementary-filter weight | 0.99 |
| $\gamma$ | `D_LPF` | D-term rate filter weight | 0.60 |
| $\beta$ | `VEL_LPF` | velocity filter weight | 0.90 |
| $\Delta t$ | `DT` | control period | 5 ms (200 Hz) |
| — | `MOTOR_DEADBAND` | stiction feed-forward (PWM) | 11 |
| — | `MAX_PWM` | output cap (PWM) | 110 |
| — | `BALANCE_SETPOINT` | calibrated balance angle (deg) | −3.22 |
| — | counts/rev | encoder resolution | 546 |

---

## 14. Further reading

- **Cart-pole / inverted pendulum dynamics** — any controls text (Ogata, *Modern
  Control Engineering*; Åström & Murray, *Feedback Systems* — free online).
- **Complementary & Kalman filtering** — Pieter-Jan Van de Maele's "Reading a
  IMU" notes; the classic "balance filter" write-ups.
- **LQR / state-space** — Brunton & Kutz, *Data-Driven Science & Engineering* (the
  control chapters), or the MIT 6.832 underactuated robotics notes (Tedrake) —
  which cover the non-minimum-phase and underactuation themes directly.
- **Non-minimum-phase systems** — Åström & Murray, ch. on limits of performance.
- **This project's own logs** — the commit history and `MEMORY` notes trace every
  failure mode above happening in real hardware, and the fix.

---

*This document describes the controller in [`balance_v2/balance_v2.ino`](balance_v2/balance_v2.ino).
Gains are the current working tune; see the README for the practical tuning guide.*
