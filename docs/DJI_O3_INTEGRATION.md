# DJI O3 Air Unit Integration Spec

Status: **specification / not yet built.** This is the design document for adding
the FPV payload the V1 PoC BOM always called for — a **DJI O3 Air Unit** streaming
to DJI FPV goggles — to the Teensy 4.1 balancer described in
[README.md](../README.md).

Scope of this document: **what the O3 needs from the robot.** Power line
requirements, mechanical/mounting requirements, cooling, RF placement, the
firmware work item, and the effect on the existing balance tune. It does not
cover chassis CAD — it produces the constraints the CAD has to satisfy.

> **The three things that will actually bite, in order:**
> 1. **Cooling.** The O3 is designed to be cooled by prop wash. On a ground robot
>    there is none, and it thermally shuts itself down in single-digit minutes. A
>    fan is a requirement, not a "nice to have".
> 2. **It does not run off 5 V.** The O3 wants **7.4–26.4 V**, i.e. the 4S pack
>    directly. The existing LM2596 5 V logic rail cannot power it, and it does not
>    need a BEC either — see [§2](#2-power-line-requirements).
> 3. **Without an arm signal over MSP it sits at 25 mW.** Full transmit power is
>    unlocked by the flight controller telling it "armed". There is no flight
>    controller here, so either the Teensy speaks MSP or you accept low power.

---

## 1. What the payload actually is

The O3 Air Unit is three physically separate parts joined by fixed-length cables.
The cable lengths are layout constraints, so they are listed first.

| Part | Size (mm) | Mass | Attachment |
|---|---|---|---|
| **Transmission module** ("air unit") | 32.5 × 30.5 × 14.5 | ~28 g | 25.5 × 25.5 mm hole pattern |
| **Camera module** | 21.2 × 20 × 19.5 (L×W×H) | (36.4 g incl. air unit + coax) | frame-specific bracket |
| **Camera coaxial cable** | **115 mm** | — | fixed, shielded, **do not modify or extend** |
| **Power/UART cable** ("3-in-1") | **100 mm** | — | 6-pin JST-GH 1.25 mm |
| **Antennas** ×2 | 85 mm long | ~3 g each | **MMCX** |

Plus, for this build: a printed cage/mount (~10 g), a 25–30 mm fan (~8 g), and
harness. **Budget ~65–70 g of payload total.**

**The two cable lengths set the layout before anything else does:**

- The camera can be no further than **115 mm of routed cable** from the air unit.
  A nose camera therefore forces the air unit into the front half of the body, or
  demands a very direct diagonal run. This is the single hardest geometric
  constraint in the whole install.
- The air unit's power/UART pigtail is **100 mm**. The Teensy shelf and the power
  distribution point must both be reachable within that, or you make an extension
  (6-pin JST-GH 1.25 mm, twisted power pair) — extending *this* cable is fine, and
  is the normal fix. Extending the **camera coax is not**.

Video is DJI-proprietary and digital: it goes to DJI goggles and nowhere else.
There is no analog tap, no HDMI out, and no way to get this feed into a PC in
real time. Latency to Goggles 2 is ~30 ms at 1080p/100fps.

---

## 2. Power line requirements

### 2.1 The headline: it goes straight on the pack

| Parameter | Value | Consequence for this robot |
|---|---|---|
| Input voltage | **7.4 – 26.4 V** (2S–6S) | 4S at **14.8 V nominal / 16.8 V full / ~12 V sagged** sits comfortably mid-range |
| Draw, typical | ~8–12 W | ~0.6–0.8 A at 4S |
| Draw, worst case | **~16 W** (high TX power + recording) | **~1.1 A at 4S**; design the branch for **2 A** |
| Operating temperature | −10 to +40 °C ambient | see [§4](#4-cooling--the-hard-requirement) |

Two conclusions, and the second one **corrects the V1 PoC BOM**:

1. **The O3 must not be hung off the LM2596 5 V logic rail.** 5 V is below its
   minimum input; it will not start.
2. **The BOM line "O3 Power — dedicated BEC/regulator for DJI O3" is not needed.**
   A 4S pack is *already inside* the O3's input window across its entire discharge
   curve. Adding a regulator buys nothing and adds a failure point. What the O3
   actually needs from the pack is **a clean, separately-run branch** — see §2.3.
   Replace that BOM line with *"filtered direct-from-pack branch, own fuse"*.

### 2.2 Revised power tree

```
4S LiPo (14.8 V nom / 16.8 V full)
  |
  +-- XT60 -> main fuse (10-15 A) -> master rocker switch -> distribution block
        |
        +-- IBT-2 #1  B+/GND  -> left motor      (fat wire, 470-1000 uF local)
        +-- IBT-2 #2  B+/GND  -> right motor     (fat wire, 470-1000 uF local)
        |
        +-- LM2596 -> 5 V rail -> Teensy VIN, IBT-2 logic Vcc + R_EN/L_EN
        |                          (Teensy onboard 3V3 reg -> Hall encoders)
        |
        +-- O3 BRANCH  ------------------------------------------------ NEW
              inline fuse 2-3 A
              -> (optional LC filter / ferrite)
              -> 470-1000 uF low-ESR electrolytic at the O3 end
              -> O3 pin 1 (VBAT, red) / pin 2 (power GND, black)
```

**Design rules for the O3 branch:**

| Item | Requirement | Why |
|---|---|---|
| Tap point | **At the distribution block**, star-fed from the pack | Not daisy-chained off an IBT-2 B+ terminal — that node carries the motor current and its switching noise |
| Fuse | Own inline **2–3 A** fuse, downstream of the main fuse and master switch | An O3 fault must not open the 10–15 A main fuse; the master switch must still kill the video |
| Wire | 20 AWG is ample for ≤2 A; **twist the +/− pair** | Twisting cuts the loop area that radiates into 5.8 GHz |
| Bulk capacitance | **470–1000 µF low-ESR** across the branch at the O3 end, plus a 100 nF ceramic | Absorbs the O3's inrush at switch-on and the pack sag when the motors break away |
| Filtering | LC filter or ferrite ring on the branch **if video shows noise that tracks motor effort** | Do not fit it speculatively — fit it if the symptom appears, so you know it did something |
| Routing | Keep the O3 branch physically away from, and ideally crossing at right angles to, the motor leads | The IBT-2 outputs are the noisiest thing on the robot |

### 2.3 Grounding

The O3's connector brings out **two separate grounds** — pin 2 power GND and pin 5
signal GND. Keep them separate along their run and land them per the robot's
existing single-point rule:

- **Power GND (pin 2)** → the distribution block ground, alongside its own +.
- **Signal GND (pin 5)** → Teensy GND, alongside the UART pair.
- Both ultimately meet at the **one** common node the migration plan already
  specifies (battery/motor GND, 5 V logic GND, 3V3/encoder GND).

Do not bridge the two grounds at the O3 end as well as at the robot end — that
makes a loop enclosing the motor wiring, which is exactly the antenna you do not
want.

### 2.4 Effect on endurance and on the fuse

At 4S/2000 mAh the pack holds ~29.6 Wh. The O3 at ~12 W draws **~0.8 A
continuously whether or not the robot is moving**, which is comparable to the
robot's own standing-and-balancing draw. Expect the addition of the O3 to roughly
**halve standby endurance**. It does not meaningfully change peak current, so the
existing 10–15 A main fuse sizing stands — but re-check it once the real draw is
measured, since the O3 branch adds ~1 A of steady load beneath the motor peaks.

### 2.5 Startup

The O3 presents an inrush at power-on. If the Teensy or the ELRS receiver ever
misbehaves at switch-on after the O3 is added, that dip is the first suspect: the
fix is the bulk capacitor in §2.2, not a rail change. Powering the O3 branch
through its own small switch (so the robot can be brought up first, video second)
is a cheap way to isolate the question during bring-up.

> When the O3 is connected to a computer over **USB-C** (for activation or
> firmware), it powers itself from USB. No pack, no bench supply.

---

## 3. Signal wiring

### 3.1 The 6-pin JST-GH connector

**Confirmed against the cable in hand** (2026-08-21): it carries exactly this
six-colour set, in this order.

| Pin | Function | Wire colour | Connect to |
|---:|---|---|---|
| 1 | **Power +** (VBAT, 7.4–26.4 V) | **red** | O3 branch + (§2.2) |
| 2 | **Power GND** | **black** | distribution block GND |
| 3 | **UART RX** (into the O3) | **white** | Teensy **TX** (pin 8) |
| 4 | **UART TX** (out of the O3) | **grey** | Teensy **RX** (pin 7) |
| 5 | **Signal GND** | **brown** | Teensy GND |
| 6 | DJI HDL / SBUS | **yellow** | **not used** — insulate individually |

Because the wires are colour-coded and the connector is keyed into the O3, the
mapping is determined by **colour, not by counting pins** — so the classic hazard
of counting from the wrong end of the header (which mirrors the pinout and lands
pack voltage on the SBUS pin) does not apply at the robot end. Land the wires by
colour.

Two checks that are still worth the two minutes before power goes anywhere near it:

- **Continuity from each bare wire to its connector pin.** Confirms nothing is
  transposed inside the moulding — the one thing colour alone cannot tell you.
- **Red and black only, on the bench, first.** Stage 2 of §10 is power-only for a
  reason: the two UART wires cannot be miswired if they are not yet wired.

Insulate **white, grey, brown and yellow separately** during the power-only stage.
Heatshrink them individually, not as a bundle — a bundle that chafes shorts four
signals to each other, including pack-adjacent ones.

The O3's UART is **3.3 V logic**, so it connects directly to the Teensy 4.1 with
no level shifting. This is the same situation as the ELRS receiver.

### 3.2 Port allocation on the Teensy

Currently used: pins 0/1 (`Serial1`, CRSF), 2–5 (encoders), 6/9 and 22/23 (motor
PWM), 18/19 (I²C to the GY-91).

**Allocate `Serial2` (RX2 = pin 7, TX2 = pin 8) to the O3**, at **115200 8N1**.
It is free, it is adjacent to nothing else in use, and it leaves `Serial3`+ for
the LIDAR the migration plan reserves.

| Signal | Teensy pin | O3 pin |
|---|---:|---:|
| MSP out (Teensy TX2) | 8 | 3 (UART RX, white) |
| MSP in (Teensy RX2) | 7 | 4 (UART TX, grey) |
| Signal ground | GND | 5 (brown) |

### 3.3 Do you need the UART at all?

**For video: no.** Power alone (pins 1 and 2) gets a picture in the goggles. The
UART buys two things:

1. **OSD** — telemetry drawn over the video in the goggles, via MSP DisplayPort.
2. **Full transmit power** — see [§5](#5-the-low-power-mode-problem).

**Recommendation: build Stage A power-only first** (two wires), prove the video,
the cooling and the RF, and only then add the UART. See the bring-up plan in §9.

---

## 4. Cooling — the hard requirement

DJI's own installation guidance states the O3 shrank ~40 % in size while power
consumption rose ~40 %, and that **extra ventilation and heat dissipation
measures are necessary** when installing it. On a quad the prop wash does this
for free. This robot has no prop wash and, worse, spends most of its life
stationary — the exact condition DJI calls out as the risk case.

**Community bench figures for an O3 with no airflow:** overheat warning in the
order of a few minutes; automatic shutdown a few minutes after that, and sooner at
high transmit power. Treat "10 minutes" as an optimistic ceiling and "3 minutes"
as the number to design against.

Behaviour when it does overheat: in the standby state it **shuts down
automatically**; in the flight state it warns, ends recording, and keeps video up
briefly before shutting down. For this robot the practical meaning is **the video
link disappears mid-drive with little warning** — which, on a machine that is
driven exclusively by that video, is a control failure, not a nuisance.

### 4.1 Requirements

| Req | Specification |
|---|---|
| **C1** | A **25–30 mm fan** blowing directly onto the air unit's heatsink face. Not optional, not "reserve the mount". |
| **C2** | A defined air path: **side/front inlet → across the air unit → rear/top exhaust.** A fan in a sealed box moves nothing. |
| **C3** | The air unit must **not** be buried in a closed shell, sandwiched against the battery, or wrapped in foam. Leave ≥5 mm clear on the heatsink face. |
| **C4** | Keep the O3 thermally separated from the other two heat sources — the IBT-2 drivers and the battery. Do not put the O3 downstream of the IBT-2s in the same air path. |
| **C5** | Leave the air unit's own temperature protection / auto power reduction **enabled**. Disabling it converts a graceful power reduction into an unannounced hard shutdown. |
| **C6** | Run the **lowest transmit power that gives a usable link** for the environment (25–200 mW indoors). Transmit power is the dominant heat term you control. |
| **C7** | Disable onboard recording for long tuning sessions. Recording adds heat for no benefit while bench-testing. |
| **C8** | Ambient limit is **−10 to +40 °C**. A warm room plus a closed shell eats most of that margin. |

Fan power: a 25–30 mm 5 V fan draws ~0.1 A and can run from the LM2596 5 V rail,
but it is a brushed motor on the same rail as the Teensy — give it a **local
100 µF + 100 nF and a series ferrite**, or feed it from its own small regulator.
Do not discover this as mystery IMU noise later.

### 4.2 Acceptance test

Before any of it goes in a closed shell: power the assembled O3 in its final
mount, with the fan running, at the transmit power you intend to use, and **leave
it for 20 minutes stationary** with the goggles connected. No warning, no
shutdown, no power reduction = pass. Anything else means the air path is wrong,
and no amount of firmware fixes it.

---

## 5. The low-power-mode problem

The O3 transmits at a **25 mW standby power** until it is told the aircraft is
armed, at which point (within ~5 s) it steps up to the region's full power. It
learns the arm state from the flight controller **over MSP**. Turning off "low
power mode" in the goggles menu does *not* substitute for this — without the MSP
arm signal it stays low.

This robot has no flight controller. Three options:

| Option | What it takes | Verdict |
|---|---|---|
| **A. Accept 25 mW** | nothing | **Fine for V1.** Indoor line-of-sight over tens of metres is exactly what 25 mW is for, and it runs much cooler. |
| **B. Teensy speaks MSP** | firmware work, §6 | **The right end state.** Gets full power *and* the OSD in one job. |
| **C. Use the ELRS RX's DisplayPort output** | RX must expose a second UART with a DisplayPort/MSP mode | Known trick on some ELRS receivers, but the **RP1's single UART is already carrying CRSF to the Teensy** — verify before counting on it. |

> A design note for option B: tie the O3's reported "armed" flag to **link-up**,
> not to the robot's CH6 motor-arm switch. The video link must not drop to 25 mW
> exactly when the robot is disarmed and you are walking up to it. Make it a
> compile-time choice if you want the standby thermal benefit back.

---

## 6. Firmware work item: MSP on the Teensy

Only needed for option B above. `Serial2` @ 115200 8N1.

The Teensy must impersonate enough of a Betaflight flight controller for the O3 to
accept it. Two halves:

**Half 1 — answer the O3's polls.** The air unit queries the FC; respond as
Betaflight does. The relevant MSP v1 message IDs:

| ID | Message | Purpose here |
|---:|---|---|
| 1 | `MSP_API_VERSION` | handshake |
| 2 | `MSP_FC_VARIANT` | reply `"BTFL"` |
| 3 | `MSP_FC_VERSION` | handshake |
| 10 | `MSP_NAME` | craft name shown in the OSD |
| 101 / 150 | `MSP_STATUS` / `MSP_STATUS_EX` | **carries the ARM flag (bit 0 of the mode flags) — this is what unlocks full TX power** |
| 105 | `MSP_RC` | stick positions, if you want them on screen |
| 110 | `MSP_ANALOG` | pack voltage / current |
| 130 | `MSP_BATTERY_STATE` | pack state |

**Half 2 — push the OSD.** `MSP_DISPLAYPORT` (182), with the usual sub-commands
(heartbeat / clear / write-string / draw-screen). Write the same fields the serial
telemetry already prints — the ones actually used for tuning: `PITCH`,
`TARGET`, `LEAN VELI`, `VF`, `PWM L/R`, `ARM`, `LINK`, pack voltage.

> Verify the message IDs and struct layouts against the Betaflight source before
> writing code — this table is a starting map, not a protocol reference. Existing
> implementations worth reading: Betaflight's `msp_displayport`, iNav's, and
> ArduPilot's `AP_MSP` (its DisplayPort backend targets exactly this air unit).

Cost estimate: a few hundred lines, no real-time constraints — MSP runs at 10–20 Hz
in a background task and must **never** be allowed to block the 200 Hz balance
loop. Keep it off the control path entirely, feeding from a snapshot struct.

---

## 7. Mounting requirements

### 7.1 Air unit module

| Req | Specification |
|---|---|
| **M1** | Hole pattern **25.5 × 25.5 mm**. Community sources report **M1.6** screws into the module; 20×20 adapter plates for it commonly use M2 through-holes. **Test-fit before committing a printed part** — measure the thread on the unit in hand. |
| **M2** | Bay envelope: module is 32.5 × 30.5 × 14.5 mm; allow **≥45 × 45 × 30 mm** for the module plus airflow plus cable exits. |
| **M3** | Mount on **soft standoffs / grommets**, not rigidly. The IMU vibration problem this repo already has is not one to feed a second rigid mass into. |
| **M4** | Orientation: heatsink face into the airflow (§4), MMCX ports toward the antenna exits, JST-GH exit toward the harness. |
| **M5** | **Serviceable without disassembly**: the USB-C port (activation, firmware), the microSD slot, and the **bind button + status LED** must all be reachable through a hatch. Bind and activate before final assembly regardless. |
| **M6** | Position: **upper rear**, as the PoC layout already specifies — which is also the right answer for CoM (§7.4) and antenna height (§7.3). |

### 7.2 Camera module

| Req | Specification |
|---|---|
| **M7** | 21.2 × 20 × 19.5 mm, front-facing, in a **protected nose mount**. This robot falls over as a normal part of operation; the camera is the most exposed and most expensive-per-gram thing on it. |
| **M8** | **Within 115 mm of routed coax** of the air unit. Route the coax with a generous bend radius, strain-relieved at both ends, and **never modify or extend it** — it is a shielded MIPI-class link. |
| **M9** | Field of view is ~155°, so the wheels and any nose structure **will** be in frame. Set the tilt and the nose profile with the camera live in the goggles before finalising the shell. |
| **M10** | **Tilt matters more here than on a quad**: this chassis pitches ±20–30° in normal driving and the horizon swings with it. Make the camera tilt **adjustable**, and expect to want it pitched up relative to the body's balance attitude, not level with the frame. |
| **M11** | Do not mount the camera rigidly to a panel that rings. Same grommet logic as M3. |

### 7.3 Antennas

| Req | Specification |
|---|---|
| **M12** | Two antennas, **MMCX**, 85 mm long. Provide two exits with strain relief; MMCX is a snap-fit and will unseat in a crash if the cable is loaded. |
| **M13** | Mount them **as high as the chassis allows** and clear of the body. A ground robot's antennas sit near the floor, and ground reflection plus a low mounting height cost far more range than they would on a drone — this is the single biggest RF difference from the intended use case. |
| **M14** | Keep them away from the battery, motors, IBT-2s, and any metal or carbon. Separate the two by at least a few centimetres, ideally non-parallel. |
| **M15** | **Keep the ELRS 2.4 GHz antenna ≥100 mm away** from the O3 antennas, and orthogonal to them. See §8. |
| **M16** | Antennas are crash consumables. Guards or a swept-back position, and keep spares. |

### 7.4 Centre of mass — this payload helps

The README is explicit that **low CoM is the root cause** of the robot's weak
acceleration: available acceleration goes as

```
a = g * L * sin(θ) / (R + L * cos(θ))        L = CoM height above the axle, R = wheel radius
```

and with `L` small, a large lean buys little force, so the outer loop has to wind
`VEL_I_CLAMP` most of the way to breakaway. **Raising the CoM is named as the real
fix.** A ~70 g payload mounted high is therefore not a tax — it is a small
instalment on the fix.

Rough magnitude, using the repo's own numbers (`R` = 70 mm for 140 mm wheels) and
an *assumed* 1.5 kg chassis with `L` = 40 mm:

| | `L` | `a` at θ = 26° |
|---|---:|---:|
| Before | 40 mm | 1.62 m/s² |
| After 70 g at 150 mm above the axle | ~47 mm | 1.80 m/s² (**+11 %**) |

> Both chassis mass and the current `L` are **assumed here, not measured** — the
> repo does not record them. Measure both before treating the +11 % as real. The
> direction of the effect is certain; the size is not.

The design consequence: **mount the O3 stack high and central, and do not
counterweight it low.** Put the air unit at the top of the rear, not in the belly.

### 7.5 What this does to the existing tune

Adding ~70 g high up changes the plant. Expect to redo, in this order:

1. **`BALANCE_SETPOINT`** — it encodes the physical CoM offset. It *will* move.
   Recalibrate on the wheels, as the README instructs.
2. **`FALL_CUTOFF` (63° / 47°)** — these were set at the *measured frame contacts*
   (+61° / −51°). A nose camera pod and a tall rear bay **change where the chassis
   hits the floor.** Re-run `LEAN_SWEEP` after the shell changes and re-derive
   them. Skipping this leaves the cutoffs either useless or firing early.
3. **Inner PD (`Kp` / `Kd`)** — more rotational inertia slows the pitch response.
   Retune on the bench, not on the floor.
4. **Deadbands, `EMF_FF_GAIN`, `MOTOR_PWM_HZ`** — unaffected by payload mass.
   Do **not** re-run `DEADBAND_TEST` for this change; nothing it measures moved.

---

## 8. RF coexistence with the ELRS link

The O3 uses **5.8 GHz** for video and **2.4 GHz** for its uplink. The robot's
control link is **ExpressLRS on 2.4 GHz**. They share a band, on one small
chassis, and the goggles at the operator end also transmit.

| Req | Specification |
|---|---|
| **R1** | Physical separation ≥100 mm between the ELRS antenna and the O3 antennas, non-parallel orientation. Keep the ELRS RX **upper/rear and away from motor power wiring**, as the BOM already says. |
| **R2** | **Measure the interaction, don't assume it.** CRSF gives link statistics back over the same UART: log LQ and RSSI with the O3 powered off, then on, then transmitting at full power, at a fixed distance. A drop is a finding; a shrug is not. |
| **R3** | Keep the ELRS transmitter module on current firmware. DJI-goggles-vs-ELRS 2.4 GHz interference is a known, actively-patched class of problem. |
| **R4** | Set the O3's region correctly and stay inside it (roughly: ≤1200 mW FCC, ≤700 mW / 14 dBm EIRP at 5.8 GHz CE, 25 mW in some regions). Lower power also means less heat and less ELRS desense — for indoor use the low setting is the better engineering answer as well as the legal one. |
| **R5** | **Link loss policy is already correct and must stay that way**: `LINK_TIMEOUT_MS` 500 ms disarms. Note the asymmetry to be aware of while driving — **losing video does not disarm the robot.** It keeps balancing and keeps whatever velocity setpoint it had. Decide deliberately whether that is what you want, and consider whether the video link should feed a separate, slower failsafe that zeroes the drive setpoint while leaving the balance loop up. |

---

## 9. Goggles compatibility and activation

| Goggles | Works with O3? | Notes |
|---|---|---|
| **DJI Goggles 2** | Yes | 1080p/100fps, ~30 ms latency. No DVR on the goggles side with an Air Unit. |
| **DJI Goggles Integra** | Yes | As Goggles 2. |
| **DJI FPV Goggles V2** | Yes, reduced | 810p/120fps. Needs goggles firmware ≥ 01.06.0000 and air unit ≥ 01.01.0100 — check both. |
| **DJI Goggles 3** | Yes, with current firmware | O3 support was added later; an air unit on V01.01.0000 **will not bind** until updated via DJI Assistant 2. |
| **DJI FPV Goggles V1** | **No** | — |

**Before the payload goes in the chassis:**

1. Connect the air unit to a PC over **USB-C** (it self-powers) and **activate** it
   in **DJI Assistant 2 (FPV series)** — a DJI account is required.
2. **Update firmware on both** the air unit and the goggles. Most "won't bind"
   reports are a firmware mismatch.
3. **Bind** — the recessed bind button beside the LED; slow green flash = standby,
   solid green = bound and transmitting.
4. Set region and transmit power.

Doing all four on the bench, before the unit is buried in a shell, is worth the
five minutes.

---

## 10. Bring-up plan

Staged, in the order that lets each stage fail on its own — the same discipline as
the existing test harnesses.

| Stage | Do | Pass criterion |
|---|---|---|
| **0** | Activate, update, bind, set region/power — on the bench, off the robot (§9) | Live video in the goggles, unit not mounted |
| **1** | Continuity-check the 6-pin cable against §3.1 (colours already confirmed) | Every wire traced to its connector pin **before** any power |
| **2** | Power the O3 from the pack branch alone — fuse, cap, twisted pair, **no UART** | Video up, nothing else on the robot disturbed |
| **3** | **Thermal test** in the final mount with the fan (§4.2) | 20 min stationary, no warning, no shutdown |
| **4** | Mount camera, set tilt and nose profile with video live | Wheels and nose acceptable in frame, camera protected |
| **5** | Power the robot and the O3 together; drive with the motors under load | No video noise tracking motor effort; no brownout at switch-on |
| **6** | ELRS coexistence: log CRSF LQ/RSSI with O3 off / on / full power (§8, R2) | No meaningful LQ loss at working range |
| **7** | Re-run `LEAN_SWEEP`, recalibrate `BALANCE_SETPOINT`, re-derive `FALL_CUTOFF`, retune inner PD (§7.5) | Balances as well as it did before the payload |
| **8** | *Optional:* MSP on `Serial2` — OSD and full-power arm flag (§6) | OSD renders; unit steps up from 25 mW |

Stages 2, 3 and 5 are the ones that decide whether this works at all. Stage 7 is
the one that will get skipped and shouldn't be.

---

## 11. Open items

| # | Item | Needed for |
|---|---|---|
| 1 | **Measure chassis mass and current CoM height `L`.** Neither is recorded anywhere in this repo, and the §7.4 numbers are assumptions without them. | CoM sizing, and honestly for the whole `VEL_I_CLAMP` discussion |
| 2 | Confirm the **air unit screw thread** (M1.6 vs M2) on the physical unit | Printed mount |
| ~~3~~ | ~~Confirm the **6-pin cable colour order**~~ — **done 2026-08-21**: red / black / white / grey / brown / yellow = pins 1–6, matching §3.1. Continuity to the connector pins still worth checking. | — |
| 4 | Measure **actual O3 current draw** at the chosen transmit power | Fuse sizing, endurance figures |
| 5 | Decide **Stage A (25 mW, no UART) vs Stage B (MSP)** for V1 | Firmware scope |
| 6 | Decide the **video-loss policy** (§8, R5) | Failsafe design |
| 7 | Check whether the **RP1 exposes a second UART** for the DisplayPort trick (§5, option C) | Possible shortcut past §6 |

---

## 12. Changes this implies elsewhere in the repo

- **V1 PoC BOM** — the "O3 Power: dedicated BEC/regulator" line is wrong; the O3
  runs directly on 4S. Replace with a fused, filtered direct-from-pack branch.
- **V1 PoC BOM** — "optional 25–30 mm fan mount" is **not optional** (§4).
- **README hardware table** — add the O3 payload and the `Serial2` allocation once
  the install is real.
- **MIGRATION_TEENSY.md §3.1** — record `Serial2` (pins 7/8) as claimed by the O3,
  so the LIDAR plan does not collide with it.

---

## 13. Sources

Vendor-published figures (dimensions, mass, voltage window, temperature range,
cable lengths, EIRP limits) come from DJI's O3 Air Unit specifications and
installation guidance. Figures for **current draw, time-to-thermal-shutdown
without airflow, the MSP arm behaviour, and the connector pinout** are
**community-measured and vary between sources** — they are good enough to design
against and not good enough to skip the bench measurements in §11.

- [DJI — O3 Air Unit support and specifications](https://www.dji.com/support/product/o3-air-unit)
- [DJI — O3 Air Unit installation guidelines (heat dissipation)](https://support.dji.com/help/content?customId=en-us03400007215&spaceId=34&re=US&lang=en)
- [DJI — A Beginner's Guide to DJI O3 Air Unit (storage, protection behaviour)](https://support.dji.com/help/content?customId=en-us03400007099&spaceId=34&re=US&lang=en)
- [DroneTrest — O3 Air Unit low power and temperature protection guide](https://www.dronetrest.com/t/dji-o3-air-unit-low-power-and-temperature-protection-guide/10603)
- [Oscar Liang — DJI O3 Air Unit and Goggles 2 review and setup](https://oscarliang.com/dji-o3-air-unit-fpv-goggles-2/)
- [Oscar Liang — DJI Goggles 3 support for the O3 Air Unit](https://oscarliang.com/dji-goggles-3-supports-o3/)
- [UAVMODEL — O3 Air Unit setup: wiring, binding, Betaflight configuration](https://blog.uavmodel.com/dji-o3-air-unit-setup-wiring-binding-and-betaflight-configuration-2026-guide/)
- [UAVMODEL — O3 range optimization: power levels and thermal behaviour](https://blog.uavmodel.com/dji-o3-air-unit-range-optimization-power-levels-antenna-selection-and-penetration-testing-2026/)
- [Unmanned Tech — full RF power on O3/O4 without a flight controller](https://blog.unmanned.tech/expresslrs-arm-dji-o3-o4-no-fc/)
- [ArduPilot — MSP DisplayPort OSD documentation](https://ardupilot.org/copter/docs/common-displayport.html)
- [ArduPilot Discourse — DJI Air Unit O3 and MSP DisplayPort OSD](https://discuss.ardupilot.org/t/dji-air-unit-o3-and-msp-displayport-osd/93549)
- [fpv-wtf/msp-osd — MSP DisplayPort implementation](https://github.com/fpv-wtf/msp-osd)
- [simacFPV — DJI Goggles 3/N3 and ExpressLRS 2.4 GHz interference](https://simacfpv.com/blog/dji-goggles3-n3-expressLRS-2.4-a-troublesome-combination)
- [Oscar Liang — LC filters for FPV video power](https://oscarliang.com/lc-filter-fpv/)
