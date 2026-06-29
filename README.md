# Self-Balancing Robot

Self-balancing robot on 2 wheels using a GY-91 IMU (MPU9250) and full-state
feedback. It **stands up, holds level, holds its spot, and recovers from
nudges**: a PD inner loop on pitch (with a low-passed D term) plus wheel-encoder
**velocity damping** (`Kv`) and **position hold** (`Kx`) so it doesn't drift.

## Project Structure

```
self-balancing-robot/
├── balance_v2/
│   └── balance_v2.ino          # Main control firmware (current)
├── arduino-cli.yaml            # Arduino CLI configuration
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

**Full-state control law** (angle PD + encoder velocity & position):
```
u = -(Kp * error + Kd * rate_filt + Ki * integral)   // pitch PD (D on low-passed rate)
u += Kv * forwardVel                                  // brake wheel velocity (anti-drift)
u += clamp(Kx * positionRev, ±POS_CLAMP)              // pull back toward home
```
`u` is then mapped to PWM with **motor-deadband feed-forward** so small
corrections actually move the geared motors, and clamped to `MAX_PWM`. There is
**no per-cycle slew limiter** — it added phase lag and caused overshoot. The D
term runs on a **low-passed gyro rate** (`D_LPF`) so single-sample gyro spikes
don't become motor kicks.

**Key parameters** (in `balance_v2/balance_v2.ino`) — current working tune:

| Param | Value | Meaning |
|-------|-------|---------|
| `Kp` | 2.0 | proportional gain (pitch) |
| `Kd` | 3.5 | derivative damping (on the low-passed rate) |
| `Ki` | 0.0 | integral (unused — `Kx` handles standing bias) |
| `Kv` | 5.0 | wheel-velocity damping (rev/s units, so runs large) |
| `Kx` | 4.0 | position hold — pulls back toward home (rev-from-home; flip sign if it wanders) |
| `D_LPF` | 0.60 | low-pass weight on the D-term gyro rate |
| `BALANCE_SETPOINT` | −3.22° | the angle where it truly balances — **calibrate on the wheels** |
| `MAX_PWM` | 110 | output ceiling (raise carefully; watch heat/current) |
| `MOTOR_DEADBAND` | 11 | lowest PWM that just spins the loaded wheels — **measure with `DEADBAND_TEST`** |
| control loop | 200 Hz | fixed-rate, constant `dt` |

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
3. (Optional) a **tiny Ki** (0.05–0.2) removes a standing lean, but here `Kx`
   (position hold) handles that, so Ki stays 0.
4. Once PD balances, add **Kv** (wheel-velocity damping) to stop the robot
   creeping across the floor. Watch `VF` — it should hover near 0 when balanced.
   Raise Kv from 0 (try ~5; rev/s is small so the gain runs large). If raising
   Kv makes the creep *worse*, flip the sign of the `u += Kv * forwardVel` term.
   This damps velocity but won't return the robot to a fixed spot — that's `Kx`.
5. Add **Kx** (position hold) so it returns toward where it booted (home) after a
   push. Watch `POS` (revs from home) — `Kx` drives it to 0. Raise from ~4 if it
   returns too slowly; **lower or flip the sign** if it wanders off or slowly
   oscillates across home (position feedback on a balancer is non-minimum-phase,
   so the sign is the first thing to suspect). It's clamped (`POS_CLAMP`) so it
   can't overpower the angle loop.

Read the serial `ERR / RATE / U` columns while tuning: growing amplitude = too
much P or too little D; buzzing = too much D; slow lean = needs I or a wrong
setpoint. If it drives the **wrong way** and accelerates the fall, flip the sign
of `u` (motor wiring/axis) before touching gains.

## Troubleshooting

**Robot overshoots / "head-bashes"**:
- Make sure `MOTOR_DEADBAND` is set so small corrections move the wheels.
- Recheck `BALANCE_SETPOINT` on the wheels; a wrong setpoint = constant lean.
- Reduce Kp or raise Kd.

**Robot falls immediately**:
- Check motor polarity (forward/reverse pins swapped?) — wrong sign accelerates the fall.
- Verify IMU calibration offsets (`GYRO_X_OFFSET`, `PITCH_ZERO_OFFSET`).

**Robot drifts forward/backward**:
- Raise `Kv` (velocity damping) to stop creeping; raise `Kx` (position hold) to
  make it return toward home. Watch `VF` / `POS` in the telemetry.

**Arduino not discovered on USB**:
- On WSL, attach the device from Windows via `usbipd`; then `ls /dev/ttyUSB*` should show it.
- `dmesg | tail` for USB device messages.

## Serial Protocol

Output every 100 ms:
```
PITCH <deg> ERR <deg> RATE <deg/s> U <pwm> SET <n> HZ <hz> LENC <t> RENC <t> VL <rev/s> VR <rev/s> VF <rev/s> POS <rev>
```
Example:
```
PITCH 0.83 ERR 0.07 RATE -3.14 U 12 SET 1 HZ 200 LENC 1432 RENC 1419 VL 0.04 VR 0.05 VF 0.05 POS 2.61
```
- **PITCH**: estimated angle (deg), positive = forward lean
- **ERR**: `PITCH − BALANCE_SETPOINT`
- **RATE**: angular velocity (deg/s), positive = falling forward
- **U**: control effort before deadband/PWM mapping
- **SET**: active PID sweep set (0 if sweep disabled)
- **HZ**: achieved control-loop rate (expect ~200)
- **LENC / RENC**: left / right encoder ticks since boot (both climb when rolling forward)
- **VL / VR**: per-wheel speed (rev/s, low-pass filtered)
- **VF**: chassis forward speed (mean of the two wheels) — the signal `Kv` damps
- **POS**: average wheel position in revs from home (boot) — the signal `Kx` drives to 0

(Startup banner: `BALANCE_V2_READY`.)

## License

MIT
