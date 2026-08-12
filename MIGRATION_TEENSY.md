# Migration: Arduino Uno → Teensy 4.1

Status: **planning** (V2). The V1 balancer (cascade controller + coast band) is working on
the Uno bench tester. This document is the plan for moving to a Teensy 4.1 as the permanent
brain and growing the feature set: CRSF/ELRS radio, movement planning, LIDAR, telemetry.

> **Naming:** the radio protocol is **CRSF** (Crossfire), which is what an **ELRS**
> (ExpressLRS) receiver outputs over UART. Not "CSRF" (that's a web-security term).

---

## 1. Why move at all — what the Uno is actually costing us

The Uno isn't just "slower." Three hard walls are already limiting V1, and every planned
feature slams into one of them:

| Wall | Uno (ATmega328P @ 16 MHz) | Teensy 4.1 (i.MX RT1062 @ 600 MHz) |
|---|---|---|
| **Float math** | No FPU — every `float` op is software-emulated. The complementary filter + cascade math is a big chunk of the 200 Hz budget. | Hardware single **and** double FPU. Balance math is essentially free → **1 kHz loop is trivial**. |
| **Encoder counting** | Only **2** external-interrupt pins. We count one edge of one channel per motor (x1) and read B for direction → half the resolution the encoder can give, and every pulse is a software ISR. | **Hardware quadrature decoders** (`QuadEncoder`, up to 4 channels). True **x4 decode, zero CPU**. ~4× resolution for free, no ISR load. |
| **Serial ports** | **One** hardware UART, shared with the USB programming/telemetry port. CRSF *or* LIDAR *or* debug — pick one. | **8** hardware UARTs **plus** native USB. CRSF, LIDAR, and a spare all at once; debug/telemetry over USB doesn't steal a UART. |
| RAM / Flash | 2 KB RAM / 32 KB flash. No room for a map, a trajectory buffer, or logging. | 1 MB RAM / 8 MB flash, **SDIO card slot**, optional PSRAM footprint. Room for LIDAR frames, maps, black-box logs. |
| Timers | Few, and `analogWrite` PWM frequencies are coupled to `millis()`/`delay()`. | Many; `IntervalTimer` gives a **dedicated hardware periodic ISR** for the balance loop, decoupled from everything else. PWM frequency freely configurable. |

Bonus: FlexCAN (CAN bus), Ethernet (4.1 + kit), and USB host — all latent for later.

**Bottom line:** the Uno is a fine *balance* controller but has zero headroom for a second
job. Every new feature on the Uno means giving something up. On the Teensy the balance loop
is a rounding error in the CPU budget and the extra I/O exists in hardware.

---

## 2. The one gotcha that will kill hardware if ignored: **3.3 V logic**

The Uno is a 5 V part. **Teensy 4.x pins are 3.3 V and are NOT 5 V-tolerant.** Feeding 5 V
into a Teensy input can damage the pin/chip. Every signal crossing into the Teensy must be
checked:

| Signal | Direction | 5 V today? | Action for Teensy |
|---|---|---|---|
| **Encoder A/B (Hall)** | sensor → MCU (**input**) | **No — RESOLVED** | Datasheet rates the encoder **DC3.3V/DC5.0V** with a **built-in pull-up** ("directly connect to MCU"). **Power it from the Teensy 3V3 rail** → outputs are 3.3 V native. Direct connect, zero parts. ✅ *(The built-in pull-up ties to the encoder's own Vcc, so the "5 V power + external 3.3 V pull-up" trick does NOT work here — match Vcc to the logic rail instead.)* |
| **MPU9250 I²C (SDA/SCL)** | bidirectional | Module usually has onboard regulator + level shifting; verify | Prefer a 3.3 V IMU breakout; if the module is 5 V-native, use an I²C level shifter. |
| **IBT-2 RPWM/LPWM** | MCU → driver (**output**) | Threshold referenced to driver's 5 V Vcc | **Marginal at 3.3 V, not guaranteed by the BTS7960 spec.** Either bench-verify direct-drive across full duty + warm, or **buffer up to 5 V with a 74HCT244** (TTL thresholds, push-pull 5 V out). Only 4 signals (RPWM/LPWM ×2). |
| **IBT-2 R_EN/L_EN** | MCU → driver (enable) | — | Just on/off — **tie straight to 5 V** (always enabled). No shifting. |
| **CRSF (ELRS RX)** | RX → MCU | ELRS RX is **3.3 V** native | Direct connect. ✅ |
| **LIDAR UART** | LIDAR → MCU | Most are 3.3 V; some 5 V | Check per unit; level-shift if 5 V. |

Outputs (MCU → 5 V device) are lower risk — a 3.3 V high *usually* reads as logic-high — but
the IBT-2 PWM inputs are the exception worth buffering (safety-critical), and
**every input into the Teensy from a 5 V source needs a shifter or a re-power**.

### 2.1 Final power architecture (decided)

Two rails, single common ground:

- **5 V rail** → powers the **Teensy** (VIN) **and** the **IBT-2 logic Vcc** + its R_EN/L_EN enables.
- **3V3 rail** → the Teensy's **onboard 3.3 V regulator** powers the **Hall encoders** (few mA each — well within budget). Encoder outputs come back at 3.3 V, safe.
- **Motor high-current** (battery → IBT-2 B+/GND) stays on its own fat wiring, separate from the 5 V logic rail; the IBT-2 only draws logic-level current from 5 V.
- **Single-point common ground:** battery/motor GND, 5 V logic GND, and 3V3/encoder GND all meet at one node.
- **USB vs VIN:** `flash.sh` uploads over USB. **Cut the Teensy 4.1 VUSB↔VIN pad** so the 5 V rail (VIN) and USB 5 V don't back-feed each other — then USB powers only during flashing, the 5 V rail powers the robot.
- **Open item:** IBT-2 PWM lines are 3.3 V from the Teensy into a 5 V-referenced input — buffer with a 74HCT244 or bench-verify direct-drive (see IBT-2 row above).

---

## 3. Hardware / wiring checklist

- [x] ~~Confirm encoder supply/logic voltage.~~ **RESOLVED: rated DC3.3V/DC5.0V, built-in pull-up → power from 3V3, direct connect.**
- [ ] ~~Level shifter for encoders~~ — **not needed** (powered at 3.3 V).
- [ ] Choose 4 encoder pins on **hardware-quad-capable** Teensy pins (see `QuadEncoder` pin map).
- [ ] IBT-2 PWM: either **bench-verify 3.3 V direct-drive** (full duty, warm board) **or** add a **74HCT244** buffer (3.3 → 5 V). Tie **R_EN/L_EN to 5 V**.
- [ ] Wire the two rails: **5 V** → Teensy VIN + IBT-2 Vcc/enables; **3V3** (Teensy onboard reg) → Hall encoders.
- [ ] **Cut the Teensy 4.1 VUSB↔VIN pad** so the 5 V rail and USB (for `flash.sh`) don't back-feed.
- [ ] Motor high-current path (battery → IBT-2 B+/GND) on separate fat wiring from the 5 V logic rail.
- [ ] **Single-point common ground:** battery/motor, 5 V logic, and 3V3/encoder grounds meet at one node.
- [ ] Dedicated UART pins assigned: CRSF (Serial_x), LIDAR (Serial_y), spare (GPS/companion).
- [ ] IMU soft-mount (still an open V1 item — vibration rectification into the gyro). Migration is a good moment to do the mechanical damping.
- [ ] SD card seated (for black-box logging / map storage) if used.

---

## 3.1 Pinout mapping (Uno → Teensy 4.1)

Pins chosen to (a) put both encoders on **hardware-quadrature-capable** pins, (b) keep the
UART pins free for the feature roadmap (CRSF, LIDAR), and (c) avoid the SPI bus + LED.

**Active signals (V1 balancer):**

| Function | Signal | Uno pin | **Teensy 4.1 pin** | Teensy peripheral | Notes |
|---|---|---:|---:|---|---|
| Motor L | RPWM (fwd) | 5 | **6** | FlexPWM2.2 A | → 74HCT244 → IBT-2 |
| Motor L | LPWM (rev) | 6 | **9** | FlexPWM2.2 B | shares submodule w/ 6 (one freq set) |
| Motor R | RPWM (fwd) | 9 | **22** | FlexPWM4.0 A | → 74HCT244 → IBT-2 |
| Motor R | LPWM (rev) | 10 | **23** | FlexPWM4.0 B | shares submodule w/ 22 |
| Motor | R_EN / L_EN | — | **(none)** | — | tie to **5 V** (always on), not an MCU pin |
| Encoder L | Phase A | 2 (INT0) | **2** | QuadEncoder ch1 A | hardware **x4** decode, zero CPU |
| Encoder L | Phase B | 4 | **3** | QuadEncoder ch1 B | |
| Encoder R | Phase A | 3 (INT1) | **4** | QuadEncoder ch2 A | |
| Encoder R | Phase B | 7 | **5** | QuadEncoder ch2 B | pin 5 is in XBAR group {0,5,37} — OK, we use neither 0 nor 37 |
| IMU | SDA | A4 | **18** | Wire (SDA0) | 3.3 V |
| IMU | SCL | A5 | **19** | Wire (SCL0) | 3.3 V |
| Status | LED | 13 | **13** | onboard LED | heartbeat/fault blink |

**Reserved for the feature roadmap (don't reassign):**

| Future use | Bus | Teensy 4.1 pins |
|---|---|---|
| **CRSF / ELRS** RX (+ telemetry back) | Serial1 | RX1 **0**, TX1 **1** |
| **LIDAR** | Serial2 | RX2 **7**, TX2 **8** |
| Spare UART (telemetry / GPS / 2nd LIDAR) | Serial3 | RX3 **15**, TX3 **14** |
| SPI (SD is separate SDIO on 4.1) | SPI0 | MOSI **11**, MISO **12**, SCK **13**\*, CS **10** |

\* SCK shares pin 13 with the onboard LED — pick one if you actually bring up SPI.

**QuadEncoder-capable pins (T4.1):** 0,1,2,3,4,5,7,8,30,31,33,36,37. Only **4 hardware
channels** exist. XBAR-exclusive groups (can't use two from the same group): {0,5,37} and
{1,36}. Our choice (2,3,4,5) touches only pin 5 from any group → safe.

**PWM-capable pins (T4.1):** 0-15, 18, 19, 22-25, 28, 29, 33, 36, 37, 42-47, 51, 54. Set
`analogWriteFrequency(6, 20000)` and `analogWriteFrequency(22, 20000)` (each covers its
submodule partner) to move motor whine out of the audible/vibration band.

**Code changes (the whole pin remap is just `#define`s + two constructors):**
```c
// Motors (each pair -> 74HCT244 buffer -> IBT-2 RPWM/LPWM)
#define LEFT_MOTOR_FORWARD_PIN   6
#define LEFT_MOTOR_REVERSE_PIN   9
#define RIGHT_MOTOR_FORWARD_PIN  22
#define RIGHT_MOTOR_REVERSE_PIN  23
// Encoders: QuadEncoder(channel 1-4, phaseA, phaseB, pullups). Built-in pull-up
// + 3.3 V supply -> pullups = 0. Replaces leftEncISR/rightEncISR entirely.
QuadEncoder encL(1, 2, 3, 0);
QuadEncoder encR(2, 4, 5, 0);
// IMU stays on default Wire (SDA 18 / SCL 19) - no pins to declare.
```

> **counts/rev = 2184 with x4 decode.** Bench-verified on the Uno: **13 PPR × 42:1 = 546**
> rising-A edges per output-shaft rev (x1 counting). `QuadEncoder` does **full x4** quadrature,
> so **546 × 4 = 2184 counts/rev** on the Teensy. Re-derive `positionRev`/`forwardVel` from
> 2184. *(The encoder datasheet's "11 PPR / 22 poles" is wrong for this unit — 13 PPR was
> measured and tested; trust the bench value, not the sheet.)* Re-check the per-side signs
> (`ENC_LEFT_DIR`/`ENC_RIGHT_DIR`) since the right encoder is mirror-mounted.

---

## 4. Firmware migration checklist

### 4.1 Toolchain & flashing — **VERIFIED**
`arduino-cli` **compiles** Teensy fine (`teensy:avr:teensy41`), but it **cannot upload** it
the way it uploads the Uno: Teensy flashes via the **HalfKay HID bootloader** and
**re-enumerates to a different USB device (VID 16C0:0478) when it enters bootloader mode**,
which breaks the `usbipd` attachment mid-flash. So the serial-upload model does not apply.

**Resolved approach (in `flash.sh`, board-aware):** keep the *sync→build-in-WSL* architecture;
**compile in WSL, upload with the Windows Teensy loader** (which handles re-enumeration
natively). `flash.sh` calls the Windows `teensy_post_compile.exe` directly from WSL — no
`usbipd` for Teensy. The Uno path is unchanged (usbipd + serial).

- [x] ~~Pick a flashing method.~~ **Done:** hybrid compile-in-WSL / upload-via-Windows-loader.
- [ ] One-time: `bash flash.sh --board teensy --setup` (adds PJRC index, installs `teensy:avr` into WSL's arduino-cli).
- [ ] Confirm the Windows Teensy tools path in `flash.sh` (`TEENSY_TOOLS_LINUX`, currently `…/teensy-tools/1.60.0`) matches the installed version.
- [ ] First flash of a blank Teensy needs one physical button press; auto-reboot works after. *(Hands-off rule still applies: the user runs `flash.sh`; I don't upload.)*

Usage: `bash flash.sh --teensy --all` (sync + compile + upload). Serial monitor is a Windows
COM port: `bash flash.sh --teensy --monitor --port COM5`.

### 4.2 Direct ports (should "just work")
- [ ] `Wire` (I²C) → Teensy `Wire` works; pick the I²C bus/pins. MPU9250 (hideakitai lib) is portable.
- [ ] `analogWrite` PWM → works, but **set an explicit `analogWriteFrequency`** on the motor pins (e.g. 20 kHz to move motor whine out of the audible/vibration band — a nice side-win for the jitter story).
- [ ] `analogWriteResolution(8)` to keep the existing 0–255 MAX_PWM mapping, or widen it.

### 4.3 Rewrites (platform-specific)
- [ ] **Balance loop timing** → move from the `millis()`-gated super-loop to an **`IntervalTimer` ISR** at a fixed rate (start at the current 200 Hz, then raise toward 1 kHz once floats are free). Deterministic, jitter-free, independent of everything else.
- [ ] **Encoders** → replace the two pin-change ISRs (`leftEncISR`/`rightEncISR`) with the **`QuadEncoder`** hardware decoder (x4, zero CPU). **counts/rev = 2184** (546 x1 × 4, see §3.1); re-derive `positionRev`/`forwardVel`. Re-check `ENC_LEFT_DIR`/`ENC_RIGHT_DIR` signs.
- [ ] **Pin map** → apply the §3.1 mapping (motors 6/9/22/23, encoders 2/3/4/5, IMU on Wire 18/19). The Uno's D2/D3-interrupt constraint is gone — pins chosen by hardware function, keeping UARTs free for CRSF/LIDAR.
- [ ] Re-validate `MOTOR_DEADBAND` / stiction floor at the new PWM frequency (higher PWM freq changes effective torque at low duty).

### 4.4 Re-tune (expect small shifts, not a redo)
- [ ] Gains should carry over conceptually, but **DT changes** (200 Hz → 1 kHz) rescale any term that isn't already normalized by `DT`. Audit `Kd`/`Ki` usage against the new loop period.
- [ ] Higher encoder resolution → cleaner `forwardVel` → the outer cascade loop can likely run tighter (revisit `Kvel`, `LEAN_LPF`).
- [ ] Keep the **coast band** — it's a control-design win, not an Uno workaround.

---

## 5. New-feature architecture (how the pieces fit around the balance loop)

Think in **rate tiers**. The balance loop is sacred and fast; everything else is slower and
feeds it setpoints or reads its state. Nothing else is ever allowed to stall it.

```
  [IntervalTimer ISR ~1 kHz]  ← hard real-time, highest priority
     balance loop: IMU read → complementary filter → cascade PD → motor PWM
        ▲ reads                                   │ writes
        │ velocity/position setpoint              ▼ pose/state (pitch, vel, pos)
  ------│------------------------------------------│------------------------------
  [~50–100 Hz]  movement planner  ────────────────┘   (soft real-time)
     turns high-level goals (go-to, velocity cmd) into cascade setpoints
        ▲
  [event / ~50–150 Hz]  CRSF/ELRS RX parser  → stick/aux channels → planner
  [~5–15 Hz]  LIDAR ingest + obstacle/map   → planner (avoid / stop)
  [~10–50 Hz]  telemetry out (CRSF back-channel / USB / SD)  ← reads state
```

- **CRSF / ELRS:** dedicate one hardware UART @ 420000 baud. Parse CRSF frames → RC channels → feed the **movement planner** (not the balance loop directly). CRSF is bidirectional, so **telemetry can go back to the transmitter over the same link** (battery voltage, link stats, attitude).
- **Movement planning:** a slow layer that emits `targetVelocity` / `targetPosition` into the existing cascade **outer loop**. This is the clean seam the cascade design already gives us — the planner never touches pitch. Start trivial (map sticks → velocity setpoint), grow toward waypoints/trajectories.
- **LIDAR:** heaviest consumer. Ingest on its own UART; reduce to "nearest obstacle / free directions" and let the planner throttle or stop. **Reality check:** full SLAM is heavy for a single M7 core alongside a 1 kHz balance loop — plan for *reactive* obstacle avoidance on the Teensy, and offload mapping/SLAM to a companion computer (Pi/Jetson over UART/CAN/Ethernet) if it grows.
- **Telemetry:** three cheap options, not mutually exclusive — CRSF back-channel to the TX, USB serial to a laptop, and **SD black-box logging** for post-run analysis (huge for tuning).

---

## 6. Should we move to FreeRTOS? — reasoned recommendation

Short version: **not yet, and never for the balance loop.** Here's the reasoning.

### 6.1 The balance loop must be an ISR, not an RTOS task
The hard real-time constraint is deterministic, low-jitter execution of the balance loop. A
timer ISR (`IntervalTimer`) gives exactly that. An RTOS scheduler *adds* latency and jitter
(tick granularity, context switches, priority decisions) — the opposite of what this loop
needs. **Even if you adopt FreeRTOS later, the balance loop stays in the timer ISR** and
communicates with tasks via shared state / queues. The RTOS never schedules balancing.

### 6.2 What an RTOS actually buys you
Preemptive priorities + blocking primitives (queues, semaphores) shine when you have
**multiple independent subsystems that block and could starve each other** — e.g. a LIDAR
read that waits on a full frame while SD logging flushes a buffer while CRSF needs servicing.
The RTOS lets each block on its own without hand-rolling state machines.

### 6.3 Why not reach for it on day one
- The critical loop is already isolated (it's an ISR), so the RTOS isn't protecting the thing that matters most.
- Cost: stack sizing per task, priority-inversion hazards, more failure modes, harder to reason about timing.
- On a **single 600 MHz M7 with 1 MB RAM**, a **cooperative super-loop scheduler** (each subsystem is a non-blocking `service()` called on its own `elapsedMillis` cadence) handles CRSF + planner + telemetry comfortably. Add the balance ISR on top and you're done.

### 6.4 Recommended path (escalate only when a trigger fires)
1. **Now:** balance loop in `IntervalTimer` ISR + **cooperative super-loop** for the soft tasks. Simplest thing that works; fully deterministic where it counts.
2. **When soft tasks start blocking each other** (LIDAR frame waits stall telemetry/CRSF, or SD flush causes hiccups): introduce **TeensyThreads** (lightweight, Teensy-native, cooperative with optional preemption) *or* the **FreeRTOS Teensy port** — moving only the *soft* subsystems into tasks, balance loop still in the ISR.
3. **Adopt full FreeRTOS** if/when you have ≥3 genuinely concurrent blocking subsystems that need real priorities and inter-task queues (LIDAR + planning + logging + comms) and the cooperative model is getting brittle.

**Decision triggers to graduate to an RTOS:** (a) a subsystem *must* block (wait on I/O) without a clean non-blocking API; (b) you're hand-writing complex state machines just to interleave tasks; (c) you need guaranteed priority servicing between soft tasks under load. Until one of those bites, the cooperative model is less code and easier to debug.

---

## 7. Suggested phased rollout

- **Phase 0 — port parity:** Teensy + level shifting; move the *existing* V1 balancer over unchanged in behavior (`IntervalTimer` loop, `QuadEncoder`, re-tune DT). Prove it balances as well as the Uno. No new features.
- **Phase 1 — RC control:** CRSF/ELRS UART → movement planner → cascade setpoints. Drive it around. Add CRSF telemetry back to the TX.
- **Phase 2 — logging:** SD black-box (state + gains) for offline tuning. Raise loop to 1 kHz, re-tune.
- **Phase 3 — perception:** LIDAR ingest → reactive obstacle stop/avoid in the planner. Evaluate whether a companion computer is needed.
- **Phase 4 — scheduler upgrade (only if triggered):** move soft tasks to TeensyThreads/FreeRTOS per §6.4.

---

## 8. Open V1 items to fold into the migration
- IMU **soft-mount** (vibration rectification into the gyro — still unaddressed on the bench).
- **Yaw/heading** control (parked in V1 because the USB tether skewed the left wheel; once untethered on the Teensy, revisit).
- **Battery-voltage compensation** of the PWM output (torque sags as the pack drains) — natural to add alongside telemetry, which already reads pack voltage.
