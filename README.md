# Self-Balancing Robot

Self-balancing robot on 2 wheels using a GY-91 IMU (MPU9250) and a **cascaded
controller**. It **stands up by itself, finds level, holds its spot, recovers
from drags, and rests its motors silently when balanced**: an inner pitch PD
loop (with a low-passed D term) wrapped by an outer position/velocity loop that
steers a *lean setpoint* (`Kpos`/`Kvel`), plus a **coast band** that cuts the
motors when settled so they don't buzz, heat, or drain the battery.

## Project Structure

```
self-balancing-robot/
├── balance_v2/
│   └── balance_v2.ino          # Main control firmware (current)
├── arduino-cli.yaml            # Arduino CLI configuration
├── CONTROL_THEORY.md           # In-depth control/robotics theory with the physics
└── README.md                   # This file
```

## Hardware

- **Microcontroller**: Arduino Uno (temporary bench tester; Teensy 4.1 in final build)
- **IMU**: GY-91 (MPU9250 — accel + gyro used; mag/baro unused in V1)
- **Motor Driver**: 2x IBT-2 / BTS7960 H-Bridge (one per motor)
- **Motors**: 2x Waveshare DCGM-3865 12V geared DC, ~240 RPM no-load, integrated Hall encoders (used for velocity damping + position hold)
- **Wheels**: 90–100 mm grippy rubber/TPU
- **Power**: 4S LiPo (~14–16.8 V) → motors; LM2596 buck → 5 V logic

> ⚠️ Motors are rated **12 V** but the 4S pack runs **~14–16.8 V**. Keep PWM
> capped and watch motor temperature/current. See `MAX_PWM` in the firmware.

## Wiring

| Component | Pin (Arduino Uno) |
|-----------|-------------------|
| MPU9250 SDA | A4 |
| MPU9250 SCL | A5 |
| Left Motor Forward  | 5 |
| Left Motor Reverse  | 6 |
| Right Motor Forward | 9 |
| Right Motor Reverse | 10 |
| Left Encoder A (green)  | 2 (INT0) |
| Left Encoder B (yellow) | 4 |
| Right Encoder A (green) | 3 (INT1) |
| Right Encoder B (yellow)| 7 |

All logic grounds must share a common reference with the motor-driver ground.

### Motor / encoder connector (Waveshare DCGM-3865)

Each motor has one 6-pin PH2.0 cable carrying both motor power and the encoder.
On this batch the connector PCB is silkscreened **`M V A B G M`**:

| Pin | Label | Wire   | Function          | Goes to            |
|-----|-------|--------|-------------------|--------------------|
| 1   | M     | white  | motor power       | IBT-2 motor out (M−) |
| 2   | V     | blue   | encoder 3.3/5 V   | 5 V (Uno) / **3.3 V (Teensy)** |
| 3   | A     | green  | Hall A            | interrupt pin (D2/D3) |
| 4   | B     | yellow | Hall B            | digital pin (D4/D7) |
| 5   | G     | black  | encoder ground    | GND (shared)       |
| 6   | M     | white  | motor power       | IBT-2 motor out (M+) |

Both `M` wires are white — M1/M2 are interchangeable; if a wheel runs the wrong
way, swap them. Encoder resolution: 13 PPR base (26-pole magnet ring) × 42:1 gearbox
= **546 counts/rev** at the output shaft (counting rising edges of A; full 4× quadrature
would be 2184 but needs more interrupt pins than the Uno has). The Teensy 4.1 is
**not 5 V-tolerant** — power the encoder from 3.3 V there so A/B are 3.3 V logic.

**Bench-testing the encoders:** set `#define ENCODER_TEST 1` in
`balance_v2/balance_v2.ino`, flash, and open the monitor. The motors stay off so
you can roll each wheel by hand; the loop streams `ENC_TEST LENC … RENC … | rev L … R …`.
One full wheel turn should move the count by ~546 (≈1.00 rev). Set it back to `0`
to restore normal balancing.

## Building & Uploading (WSL)

### Prerequisites (one-time)
```bash
sudo apt update
sudo apt install -y arduino-cli rsync
arduino-cli core install arduino:avr
```

### Compile, upload, monitor
```bash
# from the repo root
arduino-cli compile --fqbn arduino:avr:uno balance_v2
arduino-cli upload  -p /dev/ttyUSB0 --fqbn arduino:avr:uno balance_v2
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

Replace `/dev/ttyUSB0` with your port (`arduino-cli board list`). On WSL you
first need to attach the USB device from Windows (`usbipd`) and grant access to
the port (`sudo chmod a+rw /dev/ttyUSB0`).

## Control Algorithm

**Complementary filter** (sensor fusion):
- 99% gyroscope integration (fast, drifts over time)
- 1% accelerometer correction (slow, noisy, but absolute reference)

**Cascaded control law** — an outer position/velocity loop shapes the *lean
setpoint* for an inner pitch PD loop (a balancer can only accelerate by leaning,
so to brake/return it tilts *against* the motion and lets the fast inner loop
chase that tilt — a strong, sign-stable "reverse-kick"):
```
// OUTER loop: position + velocity  ->  a commanded lean (deg), clamped + SLOW-low-passed
leanRaw     = -(Kpos * positionRev + Kvel * forwardVel)        clamped to ±LEAN_CLAMP
leanCmd     = LEAN_LPF * leanCmd + (1 - LEAN_LPF) * leanRaw    // ~500 ms — MUST be slow (see below)
effSetpoint = BALANCE_SETPOINT + leanCmd

// INNER loop: balance to the leaned setpoint
error = pitch - effSetpoint
u     = -(Kp * error + Kd * rate_filt + Ki * integral)        // D on a low-passed rate
```
`u` is then mapped to PWM with **motor-deadband feed-forward** so small
corrections actually move the geared motors, and clamped to `MAX_PWM`. The D
term runs on a **low-passed gyro rate** (`D_LPF`) so single-sample gyro spikes
don't become motor kicks.

Two non-obvious pieces make this work on a non-minimum-phase plant:
- **`LEAN_LPF` (outer-loop low-pass) must be slow (~500 ms).** To return home the
  wheels must first roll the "wrong way" (non-minimum-phase). If the outer loop
  reacts *faster* than that wrong-way transient (~100–300 ms), it re-commands the
  lean mid-transient → positive feedback → **runaway** (an earlier ~100 ms setting
  did exactly this). Slowing it below the transient kills the runaway.
- **Coast band (motor protection):** when `|error| < ANGLE_DEADZONE` *and* the
  wheels are stopped, output is forced to **0** — the motors go fully off. This
  isn't just power saving: the residual jitter was a self-sustaining loop (motors
  buzz → vibration → the gyro reads it as motion → motors buzz). Cutting the
  motors at balance stops the vibration, the gyro goes quiet, and it stays
  settled. Telemetry shows long runs of `U 0` with `RATE ≈ 0`.

> **IMU axis gotcha:** on this MPU9250 library build the *pitch-axis* gyro rate
> reads on **`getGyroY()` (negated)**, not X — `gyroRate = -(getGyroY() -
> GYRO_Y_BIAS)`. A library update silently moved it, which zeroed the rate signal
> and made the robot "balance slowly but explode when pushed." If you swap IMUs or
> libraries, re-verify with `IMU_TEST` (hand-tilt, watch which gyro axis tracks).

See **[CONTROL_THEORY.md](CONTROL_THEORY.md)** for the full derivation and physics.

**Key parameters** (in `balance_v2/balance_v2.ino`) — current working tune:

| Param | Value | Meaning |
|-------|-------|---------|
| `Kp` | 2.0 | inner proportional gain (pitch) |
| `Kd` | 0.3 | inner derivative damping (on the low-passed rate). **Low** — see note below |
| `Ki` | 0.0 | inner integral (unused; the outer loop handles standing bias) |
| `Kpos` | 1.0 | outer loop: position → lean (deg per rev from home); raise = returns home harder |
| `Kvel` | 4.0 | outer loop: velocity → lean (deg per rev/s); raise = brakes/damps recovery harder |
| `LEAN_CLAMP` | 5.0° | max lean the outer loop may command (tip-over safety cap) |
| `LEAN_LPF` | 0.99 | outer-loop low-pass (~500 ms). **Must stay slow** or the plant runs away |
| `D_LPF` | 0.60 | low-pass weight on the D-term gyro rate (~7 ms) |
| `ANGLE_DEADZONE` | 1.0° | coast band: within this of target (and wheels stopped) → motors off |
| `GYRO_Y_BIAS` | −9.0 | raw `getGyroY()` at rest — **recalibrate per unit** (`IMU_TEST`, hold still) |
| `BALANCE_SETPOINT` | −3.22° | the angle where it truly balances — **calibrate on the wheels** |
| gyro/accel on-chip DLPF | 20 / 21 Hz | MPU9250 hardware low-pass — tames motor vibration at the sensor |
| `MAX_PWM` | 110 | output ceiling (raise carefully; watch heat/current) |
| `MOTOR_DEADBAND` | 11 | lowest PWM that just spins the loaded wheels — **measure with `DEADBAND_TEST`** |
| control loop | 200 Hz | fixed-rate, constant `dt` |

> **Why `Kd` is so low (0.3, not the 3–5 typical of balancers):** these motors
> couple vibration into the gyro, so the *rate* signal carries a fast phantom
> component. A high `Kd` amplifies it into a violent limit cycle. The on-chip DLPF
> + a low `Kd` + the coast band together keep the loop quiet. If you fit a
> vibration-isolating (foam/rubber) IMU mount, you can carry more `Kd` for crisper
> recovery.

## Tuning Guide

Do the two **physical calibrations first** — they matter more than the gains:

1. **`MOTOR_DEADBAND`**: wheels off the ground, ramp PWM until each wheel just
   starts to spin under its own gearbox friction. Put that number in. Easiest way:
   set `#define DEADBAND_TEST 1`, flash, and read the `first-move L@<pwm> R@<pwm>`
   line — it ramps PWM and reports each wheel's stiction threshold via the
   encoders. Set `MOTOR_DEADBAND` to ~the larger of the two, then `DEADBAND_TEST 0`.
   This is the usual cause of "can't catch itself": if the deadband is below true
   stiction, a few-degree lean commands a PWM too small to move the wheels.
2. **`BALANCE_SETPOINT`**: hold the bot on its wheels, find the angle where it
   has no tendency to tip either way, read `PITCH`, set it.

Then tune **P → D → I**, one gain at a time:

1. `Kd ≈ 0.4`, `Ki = 0`. Raise **Kp** until it pushes back and just begins a
   steady oscillation; back off to ~60–70% of that.
2. Raise **Kd** until the oscillation damps out smoothly. Too high → motor
   hiss/chatter (Kd amplifies gyro noise) → reduce, or rely on the D-term
   low-pass (`D_LPF`) which lets you carry more Kd without amplifying spikes.
3. (Optional) a **tiny Ki** (0.05–0.2) removes a standing lean, but here the
   outer loop's `Kpos` (position → lean) handles that, so Ki stays 0.
4. Once the inner PD balances, tune the **outer (cascade) loop**, which steers a
   *lean setpoint* from velocity + position (see the control law above):
   - **`LEAN_LPF` first — and keep it slow.** This is the outer-loop low-pass. If
     it's faster than the plant's non-minimum-phase wrong-way transient
     (~100–300 ms), the velocity term re-commands the lean mid-transient and the
     robot **runs away** when dragged. Start at **0.99** (~500 ms). Only lower it
     (toward 0.97) if recovery feels sluggish — and watch for runaway each notch.
   - **`Kvel`** (velocity → lean) is the *brake / recovery damper*. Raise it until
     a push/drag is arrested crisply without overshoot. Watch `VF` (should return
     to 0) and `LEAN` (the commanded tilt). Too high → twitchy; too low →
     overshoots home and lurches (big pitch swings during the return).
   - **`Kpos`** (position → lean) is the *return-home* spring. Raise it until
     `POS` converges back to 0 after a disturbance. Too high → slow oscillation
     about home; too low → parks off-center.
   - `LEAN_CLAMP` caps the commanded lean so the outer loop can never tip it over.
   The cascade sign is intuitive — *lean against the motion* — so unlike the old
   direct terms you shouldn't need to flip anything.
5. **Coast band (`ANGLE_DEADZONE`)** — tune last, for quiet motors. Widen it
   until the motors go silent at rest (`U 0` in telemetry, no buzz) but it doesn't
   visibly sway. ~**1.0°** here. Too wide → a lazy ±1° rock; too narrow → the
   motors never get to rest and buzz continuously (this is also what removes the
   self-sustaining vibration jitter — see the control-law section).

Read the serial `ERR / RATE / U` columns while tuning: growing amplitude = too
much P or too little D; a fast limit cycle when you *raise* `Kd` = the rate is too
noisy/lagged for that much D (lower it); `U` never reaching 0 at rest = widen the
coast band. If it drives the **wrong way** and accelerates the fall, the gyro or
motor sign is off — verify with `IMU_TEST` before touching gains.

## Troubleshooting

**Robot overshoots / "head-bashes"**:
- Make sure `MOTOR_DEADBAND` is set so small corrections move the wheels.
- Recheck `BALANCE_SETPOINT` on the wheels; a wrong setpoint = constant lean.
- Reduce Kp or raise Kd.

**Robot falls immediately / balances slowly but explodes when pushed**:
- Check motor polarity (forward/reverse pins swapped?) — wrong sign accelerates the fall.
- **Verify the gyro axis** with `IMU_TEST`: hand-tilt the robot and confirm which
  gyro channel tracks the pitch motion. This firmware uses `getGyroY()` negated
  (`GYRO_Y_BIAS`); a library/IMU change can move it. A dead rate signal balances
  on the accelerometer alone (sluggish) but has no damping, so it blows up fast.
- Verify `PITCH_ZERO_OFFSET` / `BALANCE_SETPOINT` trim.

**Robot drifts forward/backward, or runs away when dragged**:
- Raise `Kvel` (recovery brake) and `Kpos` (return-home spring). Watch `VF` / `POS`.
- If it *accelerates away* on a slow drag, `LEAN_LPF` is too fast — raise it back
  toward 0.99 (the outer loop must be slower than the wrong-way transient).

**Motors buzz / never go quiet at rest**:
- Widen `ANGLE_DEADZONE` (coast band) until `U` reaches 0 when settled.
- The residual gyro vibration is mechanical — a foam/rubber IMU mount removes it
  at the source and lets you carry more `Kd`.

> **Note:** the old stall cutoff is **disabled** (`STALL_CUTOFF 0`). It false-tripped
> on the cascade's lean-and-hold (moderate `u`, near-zero `VF` is *normal* here)
> and could deadlock into a guaranteed topple. Re-add only with a saturation-level
> `|u|` threshold and a non-latching re-arm (off the absolute accel angle).

**Arduino not discovered on USB**:
- On WSL, attach the device from Windows via `usbipd`; then `ls /dev/ttyUSB*` should show it.
- `dmesg | tail` for USB device messages.

## Serial Protocol

Output every 100 ms:
```
PITCH <deg> ERR <deg> RATE <deg/s> U <pwm> SET <n> HZ <hz> LENC <t> RENC <t> VL <rev/s> VR <rev/s> VF <rev/s> LEAN <deg> POS <rev>
```
Example (settled — note `U 0`, the motors coasting):
```
PITCH -3.92 ERR -0.76 RATE 0.09 U 0 SET 0 HZ 188 LENC -28 RENC -3 VL 0.00 VR 0.00 VF 0.00 LEAN 0.05 POS -0.03
```
- **PITCH**: estimated angle (deg), positive = forward lean
- **ERR**: `PITCH − effSetpoint` (the *leaned* target, not the bare setpoint)
- **RATE**: pitch-axis angular velocity (deg/s) from `getGyroY()` negated, positive = forward
- **U**: control effort before deadband/PWM mapping (`0` when coasting in the dead band)
- **SET**: active PID sweep set (0 if sweep disabled)
- **HZ**: achieved control-loop rate (expect ~200)
- **LENC / RENC**: left / right encoder ticks since boot
- **VL / VR**: per-wheel speed (rev/s, low-pass filtered)
- **VF**: chassis forward speed (mean of the two wheels) — outer loop brakes this
- **LEAN**: lean offset (deg) the outer loop is commanding (negative = leaning back to brake/return)
- **POS**: average wheel position in revs from home — outer loop drives this to 0

(Startup banner: `BALANCE_V2_READY`.)

## License

MIT
