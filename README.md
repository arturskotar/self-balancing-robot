# Self-Balancing Robot

Self-balancing robot on 2 wheels using a GY-91 IMU (MPU9250) and a PID
controller. This is the **V1 PoC**: IMU-only startup balancing — no wheel
encoders yet (velocity/position feedback comes later).

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
- **Motors**: 2x Waveshare DCGM-3865 12V geared DC, ~240 RPM no-load, integrated encoders (unused for now)
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

All logic grounds must share a common reference with the motor-driver ground.

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

**PID control law** (PD by default — `Ki = 0` until PD balances):
```
u = -(Kp * error + Kd * rate + Ki * integral)
```
`u` is then mapped to PWM with **motor-deadband feed-forward** so small
corrections actually move the geared motors, and clamped to `MAX_PWM`. There is
**no per-cycle slew limiter** — it added phase lag and caused overshoot.

**Key parameters** (in `balance_v2/balance_v2.ino`):

| Param | Default | Meaning |
|-------|---------|---------|
| `Kp` | 1.6 | proportional gain |
| `Kd` | 1.8 | derivative damping |
| `Ki` | 0.0 | integral (enable only after PD works) |
| `BALANCE_SETPOINT` | 0.76° | the angle where it truly balances — **calibrate on the wheels** |
| `MAX_PWM` | 100 | output ceiling (raise carefully; watch heat/current) |
| `MOTOR_DEADBAND` | 28 | lowest PWM that just spins the loaded wheels — **measure this** |
| control loop | 200 Hz | fixed-rate, constant `dt` |

## Tuning Guide

Do the two **physical calibrations first** — they matter more than the gains:

1. **`MOTOR_DEADBAND`**: wheels off the ground, ramp PWM until each wheel just
   starts to spin under its own gearbox friction. Put that number in.
2. **`BALANCE_SETPOINT`**: hold the bot on its wheels, find the angle where it
   has no tendency to tip either way, read `PITCH`, set it.

Then tune **P → D → I**, one gain at a time:

1. `Kd ≈ 0.4`, `Ki = 0`. Raise **Kp** until it pushes back and just begins a
   steady oscillation; back off to ~60–70% of that.
2. Raise **Kd** until the oscillation damps out smoothly. Too high → motor
   hiss/chatter (Kd amplifies gyro noise) → reduce.
3. Only if it slowly leans one way, add a **tiny Ki** (0.05–0.2) to remove the
   standing bias. Without encoders, don't rely on Ki to hold position.

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
- Expected without velocity feedback. Add encoders (V2) for station-keeping.

**Arduino not discovered on USB**:
- On WSL, attach the device from Windows via `usbipd`; then `ls /dev/ttyUSB*` should show it.
- `dmesg | tail` for USB device messages.

## Serial Protocol

Output every 100 ms:
```
PITCH <deg> ERR <deg> RATE <deg/s> U <pwm-units>
```
Example:
```
PITCH 0.83 ERR 0.07 RATE -3.14 U 12
```
- **PITCH**: estimated angle (deg), positive = forward lean
- **ERR**: `PITCH − BALANCE_SETPOINT`
- **RATE**: angular velocity (deg/s), positive = falling forward
- **U**: control effort before deadband/PWM mapping

(Startup banner: `BALANCE_V2_READY`.)

## License

MIT
