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
| **Power/UART cable** ("3-in-1") | **100 mm** | — | **6-pin JST-SH, 1.0 mm pitch** |
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
  (6-pin JST-SH 1.0 mm, twisted power pair) — extending *this* cable is fine, and
  is the normal fix. Extending the **camera coax is not**.

Video is DJI-proprietary, digital and encrypted: it goes to DJI goggles, and
everything else comes **out of the goggles**, not off the air. There is no analog
tap and no receiver but a DJI headset — see [§13](#13-getting-the-video-onto-a-pc)
for the routes onto a PC. Latency to Goggles 2 is ~30 ms at 1080p/100fps.

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

### 3.1 The 6-pin JST-SH connector

**The connector standard is 6-pin JST-SH, 1.0 mm pitch** — the same family as the
small "SH1.0" cables used all over FPV flight controllers.

> **Do not size this from the previous generation.** The DJI FPV Air Unit V1 and
> the Caddx/RunCam Vista used the larger **1.25 mm pitch** connector (8-pin on the
> DJI end). The O3 shrank ~40 % and moved to **1.0 mm**. The two are not
> intermateable, and a lot of writing about "the DJI air unit connector" still
> describes the old one. If a part is listed as GH 1.25 mm, it is for the old air
> unit, not this one.

**Sourcing.** 1.0 mm pitch is too fine to hand-crimp reliably — do not try. Buy
**pre-made, pre-crimped 6-pin SH1.0 cables** (Flywoo, Pyrodrone and generic kits
sell them in 100 / 150 / 200 mm, which is also how you get the extension §1 calls
for). Buy at least one spare and **build the robot harness from the spare, keeping
DJI's stock 100 mm cable intact** — it is the reference for wire order and it is
the thing you will want on hand when something is miswired.

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
  transposed inside the moulding — the one thing colour alone cannot tell you. At
  1.0 mm pitch, probe with a **needle/fine-tip probe or at the wire end**; a
  standard multimeter probe will bridge adjacent pins or splay them.
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

**No external board is needed.** The Teensy 4.1 has **eight hardware UARTs**, and
this build uses exactly one of them. Having spare UARTs was one of the stated
reasons for the Uno → Teensy migration in the first place; this is that headroom
being spent.

Pin map, from the Teensy core source (`HardwareSerial1..8.cpp`), checked against
every pin this project already uses:

| Port | RX | TX | Status here |
|---|---:|---:|---|
| `Serial1` | 0 | 1 | **used** — CRSF from the RP1 |
| **`Serial2`** | **7** | **8** | free → **allocated to the O3** |
| `Serial3` | 15 | 14 | free — the migration plan's LIDAR candidate |
| `Serial4` | 16 | 17 | free |
| `Serial5` | 21 | 20 | free |
| `Serial6` | 25 | 24 | free |
| `Serial7` | 28 | 29 | free |
| `Serial8` | 34 | 35 | free |

None of them collide with the encoders (2–5), the motor PWM (6, 9, 22, 23) or
I²C (18/19). Serial telemetry to the PC runs over **native USB** and costs no
UART at all.

> ⚠️ **`Serial2` and `Serial4` overlap in the core's alternate pin tables** —
> `Serial2` can also be placed on 16/17 and `Serial4` on 7/8. Harmless as long as
> you never assign both to the same physical pins. Take `Serial2` on **7/8** and
> leave `Serial4` alone.

**Allocate `Serial2` (RX2 = pin 7, TX2 = pin 8) to the O3**, at **115200 8N1**.

| Signal | Teensy pin | O3 pin |
|---|---:|---:|
| MSP out (Teensy TX2) | 8 | 3 (UART RX, white) |
| MSP in (Teensy RX2) | 7 | 4 (UART TX, grey) |
| Signal ground | GND | 5 (brown) |

### 3.3 Why connect the UART to the Teensy at all?

**For video: you don't.** Power alone (pins 1 and 2) gets a picture in the
goggles — pairing and video are entirely an air-unit-to-goggles affair, with no
flight controller in the path (§9.1). So the two signal wires are a deliberate
choice, and worth arguing rather than assuming.

They buy exactly two things.

**1. The telemetry, in the goggles — the real reason.**

This repo's entire debugging method is reading the 100 ms telemetry stream: which
of the three outer-loop terms saturated, whether `VELI` *stepped* to zero or
*slid*, whether a stalled wheel is a torque null or a lost sign contest. Every
troubleshooting entry in the README ends in "look at the telemetry".

That method works today because the robot is on a box next to the monitor. **The
moment it is driving around under FPV, the serial monitor is somewhere else and
you are wearing goggles.** You get the symptom — it stopped, it curved, it lost
its lean — with none of the state that explains it, and you have to walk over,
tether it, and try to reproduce on the bench what happened in the next room.

MSP DisplayPort puts `PITCH`, `TARGET`, `LEAN VELI`, `VF`, `PWM L/R`, `ARM`,
`LINK` and pack voltage **on the video you are already looking at**. It is not a
cosmetic overlay; it is the difference between keeping the repo's debugging
practice once the robot goes mobile and losing it.

**2. Full transmit power** (§5). The unit sits at 25 mW until an FC reports
"armed" over MSP. Indoors, line-of-sight, 25 mW is fine. But the build this robot
is modelled on is a **scout that goes through doorways and around corners** —
which means walls between the antennas and the goggles, and 5.8 GHz does not like
walls. If the video breaks up two rooms away, this is the fix, and it is the
*only* fix.

**Why both wires, not just one:** MSP is request/response. The air unit **polls**
the Teensy and expects answers — the arm flag arrives as a *reply* to a status
request, not as something you can push unprompted. Teensy TX (white) carries the
OSD frames and the replies; Teensy RX (grey) hears the polls. Wiring only TX gets
you neither reliably.

**Recommendation: land all four wires during the build, use them later.** The
hardware cost is two extra wires in a cable you are already plugging in, and the
cost of *adding* them afterwards is opening a sealed, cooled, antenna-routed
chassis. The firmware (§6) can arrive whenever. Build **Stage A power-only**
first — prove video, cooling and RF with nothing else in the way — but solder for
Stage B while the shell is still open.

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
| **M4** | Orientation: heatsink face into the airflow (§4), MMCX ports toward the antenna exits, JST-SH exit toward the harness. |
| **M5** | **Serviceable without disassembly**: the USB-C port (activation, firmware), the microSD slot, and the **bind button + status LED** must all be reachable through a hatch. Bind and activate before final assembly regardless — noting that binding needs a **VBAT lead**, not just USB (§9.3). |
| **M6** | Position: **upper rear**, as the PoC layout already specifies — which is also the right answer for CoM (§7.4) and antenna height (§7.3). |

### 7.2 Camera module

| Req | Specification |
|---|---|
| **M7** | 21.2 × 20 × 19.5 mm, front-facing, in a **protected nose mount**. This robot falls over as a normal part of operation; the camera is the most exposed and most expensive-per-gram thing on it. |
| **M8** | **Within 115 mm of routed coax** of the air unit. Route the coax with a generous bend radius, strain-relieved at both ends, and **never modify or extend it** — it is a shielded MIPI-class link. |
| **M9** | Field of view is ~155°, so the wheels and any nose structure **will** be in frame. Set the tilt and the nose profile with the camera live in the goggles before finalising the shell. |
| **M10** | **Tilt matters more here than on a quad**: this chassis pitches ±20–30° in normal driving and the horizon swings with it. Make the camera tilt **adjustable**, and expect to want it pitched up relative to the body's balance attitude, not level with the frame. |
| **M11** | Do not mount the camera rigidly to a panel that rings. Same grommet logic as M3. |
| **M11b** | **Reserve a second front aperture and cable path** for the future machine-vision camera (§13.3). Cheap now, structural surgery later. |

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

### 7.6 Head tracking and a panning camera

The Goggles 3 **do** contain a head-tracking IMU — it is a real feature of the
hardware. The catch is who is allowed to read it.

**Head tracking is gated to DJI's own aircraft** (Avata 2, Neo and similar), where
DJI owns both ends of the link. On a custom build with an O3 Air Unit there is **no
documented path for head angles to reach your own controller.** Pin 6 (DJI
HDL/SBUS) carries channel data from a bound **DJI remote controller**, not head
orientation — and this robot has no DJI remote in the first place; control is
ELRS/CRSF from the TX16S.

> **Confidence:** this is not confirmed from a primary DJI source. The evidence is
> circumstantial but consistent: the community threads asking for it end without a
> solution, and the market's answer is **external head trackers sold specifically
> "for DJI O3/O4"** — which would not exist if the goggles' own sensor were
> reachable. Treat it as "no known route", not "proven impossible", and re-check
> before designing around it either way.

**The route that does work** fits this build unusually well: an **external head
tracker feeding the TX16S**, whose angles then travel as ordinary **CRSF channels**
→ RP1 → Teensy → servo. No new radio link, no new protocol, and `crsf.h` already
parses channels — a pan servo would be one more channel alongside CH1/CH2/CH3.

**Two constraints before designing a panning head:**

| | |
|---|---|
| **The coax is not a flex cable.** | Panning the camera alone repeatedly flexes the 115 mm shielded coax (§1), which is not rated for continuous articulation. Either **pan the air unit and camera together as one assembly**, or accept a limited sweep with a generous service loop. Do not build a 180° continuous pan around that cable. |
| **A panning head is a moving mass above the axle.** | Panning the camera alone (~8 g) is negligible. Panning the whole ~70 g stack moves a meaningful fraction of the payload off-centre, which shifts the CoM laterally and feeds a disturbance into a loop that is already working near its authority limits (§7.4, §7.5). If the whole stack pans, treat it as a balance-affecting change and re-check on the bench. |

**Cheapest alternative: turn the robot.** It already pivots in place. That costs
nothing to build, and the trade is that looking around also changes heading — for
a scout easing round a doorway, that may be exactly wrong, which is the argument
for the servo.

---

## 8. RF coexistence with the ELRS link

The O3 uses **5.8 GHz** for video and **2.4 GHz** for its uplink. The robot's
control link is **ExpressLRS on 2.4 GHz**. They share a band, on one small
chassis, and the goggles at the operator end also transmit.

| Req | Specification |
|---|---|
| **R1** | Physical separation ≥100 mm between the ELRS antenna and the O3 antennas, non-parallel orientation. Keep the ELRS RX **upper/rear and away from motor power wiring**, as the BOM already says. |
| **R2** | **Measure the interaction, don't assume it.** CRSF gives link statistics back over the same UART: log LQ and RSSI with the O3 powered off, then on, then transmitting at full power, at a fixed distance. A drop is a finding; a shrug is not. |
| **R3** | **Goggles 3 + ELRS 2.4 GHz is a known, named problem — and it lives at the *pilot's* end, not the robot's.** The Goggles 3 transmit continuously on 2.4 GHz to talk to DJI's RC3 **even when no DJI controller is in use**, and they sit ~30 cm from the TX16S's ELRS antenna on the pilot's head. That is near-field desense of the *transmitter*, which no amount of care on the robot fixes. Mitigations, in order: **flash the ELRS 3.x maintenance-branch firmware to the TX module and the RP1** (this is the specific fix the ELRS project shipped for it); keep the TX16S antenna up and angled away from the goggles; do not rest the radio against the headset. |
| **R3b** | **Fly the range test with the goggles actually on your head**, not on the bench beside the robot. Bench-testing this particular interaction reproduces the wrong geometry and will tell you it is fine. |
| **R4** | Set the O3's region correctly and stay inside it (roughly: ≤1200 mW FCC, ≤700 mW / 14 dBm EIRP at 5.8 GHz CE, 25 mW in some regions). Lower power also means less heat and less ELRS desense — for indoor use the low setting is the better engineering answer as well as the legal one. |
| **R5** | **Link loss policy is already correct and must stay that way**: `LINK_TIMEOUT_MS` 500 ms disarms. Note the asymmetry to be aware of while driving — **losing video does not disarm the robot.** It keeps balancing and keeps whatever velocity setpoint it had. Decide deliberately whether that is what you want, and consider whether the video link should feed a separate, slower failsafe that zeroes the drive setpoint while leaving the balance loop up. |

---

## 9. Goggles compatibility and activation

**This build targets DJI Goggles 3.** The other models are kept in the table for
reference only.

| Goggles | Works with O3? | Notes |
|---|---|---|
| **DJI Goggles 3** | **Yes — the target here.** Firmware-gated, see §9.2 | O3 support arrived in goggles firmware **v01.00.0300** (mid-2024). Requires air unit **≥ V01.02.0000**. |
| **DJI Goggles 2** | Yes | 1080p/100fps, ~30 ms latency. No DVR on the goggles side with an Air Unit. |
| **DJI Goggles Integra** | Yes | As Goggles 2. |
| **DJI FPV Goggles V2** | Yes, reduced | 810p/120fps. Needs goggles firmware ≥ 01.06.0000 and air unit ≥ 01.01.0100. |
| **DJI FPV Goggles V1** | **No** | — |

> ⚠️ **Goggles 3 carries a second consequence beyond firmware — it is the model
> named in the ELRS 2.4 GHz interference reports. See [§8](#8-rf-coexistence-with-the-elrs-link), which is not
> optional reading for this pairing.**

### 9.1 The video link needs power and nothing else

Worth stating plainly, because it is the thing that makes Stage A viable:

**The video link is entirely between the air unit and the goggles.** Pairing,
video, and the goggles' own recording all happen over DJI's own radio link. No
flight controller is involved in the video path at any point. Two wires — pin 1
red and pin 2 black — get a live picture.

What the UART adds is only:

- the **OSD overlay** drawn on top of the video (§6), and
- the **arm flag** that lets the unit leave 25 mW standby (§5).

Neither is needed to *see* anything. An unbound, un-UARTed, power-only O3 is a
complete FPV camera the moment it is paired.

### 9.2 Activation — the gate that catches everyone

A new O3 **must be activated before it will work properly.** Out of the box it
reports that functions are limited, and no amount of correct wiring gets past it.

1. Connect the air unit to a **PC or Mac** with **USB-C**. It **self-powers from
   USB** — no pack, no bench supply, nothing else connected. **A phone cannot do
   this**: the air unit has no phone or over-the-air update path, and it cannot be
   updated through the goggles (§14.1).
2. Open **DJI Assistant 2**, log in with a DJI account, click the O3 Air Unit
   icon, and follow **Start Activation**.

> **Which DJI Assistant 2?** DJI publishes several builds and they are not
> interchangeable. **Use "DJI Assistant 2 (Consumer Drones Series)"** — it is the
> right one for **both** the O3 Air Unit and the Goggles 3, despite both being FPV
> products. The **"DJI FPV Series"** build is for the original DJI FPV drone and
> is only worth trying as a fallback. Links in [§14](#14-software-and-downloads).
>
> "Air unit does not show up in Assistant 2" is usually the wrong build, a
> **charge-only USB-C cable**, or a hub. Use a known-good *data* cable, straight
> into the machine.

3. While you are there, **update the firmware on the air unit** — and separately
   update the **goggles**. The large majority of "it will not bind" reports are a
   firmware mismatch between the two, not a fault.

**For Goggles 3 specifically, these are hard gates, not recommendations:**

| Device | Symptom if too old |
|---|---|
| **Goggles 3** | **O3 is not offered as a device at all** — the bind option simply is not in the menu |
| **O3 Air Unit** | **Will not bind.** The button does nothing useful, the LED blinks, and the goggles never see it — no error, just silence |

> **Do not chase a specific version number.** Sources disagree on the exact
> minimum — Goggles 3 gained O3 support around **v01.00.0300**, and reported air
> unit minimums range from **V01.02.0000 to v01.03.0000**. The two devices speak a
> proprietary protocol that changes between releases, so the only reliable
> instruction is: **update both to the latest available, then power-cycle both.**
> DJI's notes call for the restart and it matters.

A new-old-stock O3 is very likely to ship too old, so **assume the air unit needs
updating before it will ever talk to Goggles 3.**

### 9.3 Pairing (binding) to the goggles

**There is no cable between the goggles and the air unit.** The link is the
wireless 5.8 GHz downlink — that is the entire purpose of the air unit. The only
USB-C cables in this whole process run from a *computer* to one device at a time,
for activation and firmware (§9.2), never between the two.

So "connecting them" means exactly one thing: **binding them once.** After that
they find each other automatically every time both are powered, and the binding
survives power cycles, republishes and remounting. You redo it only if you change
goggles or reset the unit.

> ⚠️ **Binding needs real power — USB-C is not enough.** Activation and firmware
> (§9.2) work off USB because the unit only has to enumerate to the PC. **Binding
> needs the radio up**, and reports are that on USB power the air unit stays
> **dormant** — it never transmits, so the goggles never see it. Give it proper
> **VBAT (7.4–26.4 V) on pins 1 and 2** before you try to bind: the 4S pack, a
> spare 3S/4S, or a bench supply at ~12 V.
>
> *Confidence: secondary sources, not DJI's own manual. If you want to settle it
> in ten seconds, plug USB only and watch the LED — if it never reaches the
> booted state, that is your answer.*

**So the desk work splits in two, and they need different power:**

| Step | Power | Where |
|---|---|---|
| Activate + firmware-update | **USB-C from the PC**, nothing else | §9.2 |
| **Bind** | **VBAT on pins 1/2** — pack or bench supply | below |

Still do both **before the unit is buried in the chassis** — binding through a
service hatch is miserable. It just means having a power lead ready, not only a
USB cable.

> **While it is powered on the bench to bind, it is transmitting with no
> airflow** — the §4 thermal clock is running. Bind promptly, or point a fan at
> it.

> **"There is no button on the air unit, and no bind setting in the goggles."**
> Two symptoms, almost always **one cause: firmware (§9.2)** — do that first, then
> look again.
>
> - **The button exists.** It is a *small recessed* button beside the status LED
>   on the module, easy to miss and easy to mistake for a moulding mark. Look
>   next to the LED and the USB-C port. (DJI's quick-start manual shows it on its
>   Activation/Linking page — worth opening for the photo.)
> - **The goggles' bind option is absent until the goggles support O3.** On
>   firmware predating O3 support, the O3 is not offered as a device at all, so
>   there is nothing under Transmission to find. It appears after the update.
>
> This is exactly why §10 puts activation and updating at **Stage 0**, before any
> wiring: on out-of-the-box firmware, the pairing UI may not exist yet.

**Air unit LED, in order:**

| LED | Means |
|---|---|
| **Green** | powering on / booting |
| **Solid red** | booted, **not linked** — ready to bind |
| **Blinking red** | depends on what you did — see below |
| **Solid green** | **linked.** Video should be up |

**Blinking red means two different things, and the difference is the diagnosis:**

- **After you press the link button** → actively in linking mode, searching. This
  is the good state mid-bind.
- **On its own, from power-up, without pressing anything** → the unit is *not*
  in link mode. On an **unactivated** unit this is what you get: an unactivated
  O3 **will not transmit at all**, cannot set functions or channels, and
  therefore **cannot be found by the goggles no matter how often you press
  bind.** Go to §9.2.

> It is transmitting while it searches. Do not leave it blinking red on the bench
> with no airflow (§4).

### 9.3.1 "The goggles cannot find the air unit"

The single most common bind failure, and the order to work it:

**First, bisect it — the goggles tell you which side is at fault:**

| What the goggles show | Verdict |
|---|---|
| **No O3 / Air Unit option at all** in the device list | **Goggles firmware** predates O3 support. Nothing to press. Update the goggles. |
| **The O3 option exists**, but the air unit never appears under Transmission | **The goggles are fine.** The fault is on the **air unit** — activation or its firmware. Work the table below from the top. |

| # | Check | Why |
|---|---|---|
| 1 | **Air unit activated** (§9.2) | **A new O3 is not activated out of the box.** An unactivated unit **will not transmit at all** — so the goggles cannot find it, however many times you press bind. This is a hard gate, not a degraded mode. |
| 2 | **Both devices on current firmware.** | The other ~90 % case. The O3 and Goggles 3 speak a proprietary protocol that **changes between firmware releases** — a mismatch produces silence, not an error. Reports are that the versions need to be within about one minor revision of each other. Same USB-C session as activation, so there is no reason to separate them. |
| 3 | **Put the goggles into pairing mode *first*, then the air unit.** | Sources differ on the order and the control — **Settings → Transmission → Bind**, or **holding the goggles' power button until they beep**. Both devices must be searching *at the same time*; getting the order backwards is a real cause. |
| 4 | **Press the air unit's link button twice.** | Some units are reported not to enter link mode on the first press. Free to try. |
| 5 | **Power-cycle the goggles fully** and retry. | Also free, and it clears a goggles-side state that firmware updates are known to leave behind. |

> **The tell that it is firmware:** if the goggles offer **no O3 option and no bind
> entry in the menu at all**, their firmware predates O3 support. There is nothing
> to press, and no amount of button-pressing on the air unit will change that.
> Go to §9.2.

**Goggles 3 — the procedure for this build.** Note the menu path differs from
Goggles 2: it is under **Transmission**, not Status.

1. Confirm both firmware versions clear the gates in §9.2, and that both have been
   **power-cycled since updating**.
2. Power the air unit **from VBAT, not USB** (see the warning above) and wait for
   it to finish booting.
3. Press the air unit's **bind button** — the small recessed one beside the LED —
   until the **LED blinks rapidly**.
4. On the goggles: **Settings → Transmission → Bind**.
5. Binding takes **5–10 seconds**. Bound: the LED goes **solid** and the camera
   image appears.

**Goggles 2 / Integra**, for reference: boot the air unit until the LED goes green
→ off → **red**, then swipe right on the touchpad → **Status** → select *DJI O3
Air Unit* → press the **Link button between the lenses** → press the air unit's
bind button. **FPV Goggles V2:** same shape, link button elsewhere on the body.

### 9.4 Then set

- **Region and transmit power** — see §8 R4 and §4 C6. Low power indoors: legal,
  cooler, and kinder to the ELRS link.
- **Leave temperature protection enabled** (§4 C5).

**What "connected" looks like in normal use:** power the robot, put the goggles
on, and the image appears within a few seconds with the air unit's LED solid.
Nothing is pressed. If it does not come up, check in this order: both actually
powered; the air unit not in thermal shutdown from a previous session (§4); the
goggles on the same channel/band; and only then suspect the bind.

---

## 10. Bring-up plan

Staged, in the order that lets each stage fail on its own — the same discipline as
the existing test harnesses.

| Stage | Do | Pass criterion |
|---|---|---|
| **0a** | **Activate and firmware-update** both devices — bench, **USB-C to the PC**, no pack (§9.2; software in §14) | Both on current firmware, both power-cycled |
| **0b** | **Bind**, then set region/power — bench, but now on **VBAT** (pack or ~12 V supply), still off the robot (§9.3) | Live video in the goggles, unit not mounted |
| **1** | Continuity-check the 6-pin cable against §3.1 (colours already confirmed) | Every wire traced to its connector pin **before** any power |
| **2** | Power the O3 from the pack branch alone — fuse, cap, twisted pair, **no UART** | Video up, nothing else on the robot disturbed |
| **3** | **Thermal test** in the final mount with the fan (§4.2) | 20 min stationary, no warning, no shutdown |
| **4** | Mount camera, set tilt and nose profile with video live | Wheels and nose acceptable in frame, camera protected |
| **5** | Power the robot and the O3 together; drive with the motors under load | No video noise tracking motor effort; no brownout at switch-on |
| **6** | ELRS coexistence: log CRSF LQ/RSSI with O3 off / on / full power — **wearing the Goggles 3**, radio in hand (§8 R2, R3, R3b) | No meaningful LQ loss at working range |
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
| 5b | **Check the O3's shipped firmware version against ≥ V01.02.0000** before assuming it will bind to the Goggles 3 (§9.2) | Getting a picture at all |
| 5c | **Confirm the ELRS TX module and RP1 are on 3.x maintenance-branch firmware** before judging any link degradation (§8 R3) | Not chasing a fixed bug |
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

## 13. Getting the video onto a PC

Worth settling early, because the obvious-looking answer is a dead end and the
right answer depends on *why* you want it.

### 13.1 Not with an SDR

**The O3 link is encrypted and proprietary.** OcuSync uses AES-256 with a session
key regenerated at every power-on, over an undocumented LTE-like OFDM physical
layer. There is **no open receiver, no public decoder, and no known break of the
video payload.**

The HackRF is doubly the wrong tool even before the crypto:

| | HackRF One | What O3 needs |
|---|---|---|
| Bandwidth | ~20 MHz max sample rate | 20 MHz *and* 40 MHz channel modes — **40 MHz is physically uncapturable**, and 20 MHz leaves no oversampling headroom |
| ADC | **8-bit** (~48 dB) | OFDM has high peak-to-average ratio and wants 12+ bits |
| 5.8 GHz | top edge of its 1 MHz–6 GHz range, where sensitivity is worst | the whole downlink lives there |

For scale: the academic team that did get somewhere used a **USRP B200mini**
(56 MHz, 12-bit) — and what they recovered was **DroneID, the telemetry beacon DJI
left unencrypted**, not the video. The video payload remains unbroken publicly.

So: capturing RF gets you a spectrogram, not a picture. Treat this as closed.
(Nothing here is about anyone else's link — receiving your own transmission is
the only case in view, and it still does not work.)

### 13.2 What actually works

> **Decided:** the PC feed is for **test capture and review only** — watching runs
> back alongside the telemetry, not feeding anything. **Route: goggles → PC.**
> Computer vision is a separate board with its own camera (§13.3), so nothing
> downstream depends on this path's latency or on DJI keeping it open.

**Live, on a PC:**

| Path | Notes |
|---|---|
| **Goggles 3 → Wi-Fi → DJI Fly app** | Native and free. Goggles 3 share the live feed to the phone app over Wi-Fi; restream or screen-capture from there. Adds latency — fine for watching, not for closing a loop. |
| **Goggles 3 → USB-C → PC** | Third-party software (e.g. Cosmostreamer, which lists Goggles 2/3/Integra/N3) takes the goggles' USB output and gives clean video or RTSP/SRT/NDI/UDP-H.264 restreaming at low latency. Paid, but it is the least-friction route to a real video stream on a desktop. |
| **DJI RC Pro relay → HDMI** | Works, but only if you already own an RC Pro. Goggles 3 have **no HDMI port** of their own. |

**Recorded, after the fact** — often all you need for reviewing test runs:

- **O3 internal storage** (~20 GB, expandable with a microSD in the module) — pull
  over USB-C.
- **Goggles 3 DVR** to their own card.

> Recording adds heat to a unit that is already thermally marginal on this
> chassis (§4 C7). For long tuning sessions, record on the goggles, not the air
> unit.

**Try the free paths before paying for one.** If reviewing runs afterwards is
enough, the **Goggles 3 DVR** costs nothing and needs no software. The USB-C
route earns its keep only for the thing the card cannot do: **video and the
100 ms telemetry stream on one screen, live, at the same time.**

That combination is worth something specific here. Most of the tuning in this
repo was read off the scrolling telemetry with the robot on a box; being able to
see *what the robot saw* against `LEAN VELI` saturating, or a breakaway stalling,
turns two separate observations into one. If you go that way, capture both in one
recording — an OBS scene with the video source and the serial monitor window side
by side — so the frames and the `MS` timestamps stay aligned without any sync
work.

### 13.3 Machine vision is a separate camera — decided

**Decided:** when the deferred compute work lands, computer vision runs on **its
own board with its own camera**. The O3 is not in that path.

This is the right split and worth keeping to. The O3 is a closed, pilot-eyes
downlink — encrypted, latency-optimised for human viewing, and terminating in a
headset. Anything built on extracting frames back out of DJI's goggles inherits
the whole chain's latency, an extra PC in the loop, and a dependency on DJI not
closing the route in a firmware update.

**Two cameras, two jobs:** the O3 for the pilot's view, a plain UVC/CSI camera
straight into the companion board for vision. The consequence for *this* spec is
small but real — **the chassis should reserve a second front aperture and a cable
path** for that camera now, while the shell is still being designed, rather than
have it retrofitted into a nose already full of O3.

---

## 14. Software and downloads

Everything needed to activate, update, bind and configure the pair. Nothing here
is required to *build* the robot — it is the desk work in §9.

### 14.1 Which device is updated from what

They are not the same, and this is the thing that causes an afternoon of
confusion:

| Device | Activation | Firmware |
|---|---|---|
| **O3 Air Unit** | **PC/Mac only** | **PC/Mac only** — USB-C into DJI Assistant 2. There is **no phone path and no over-the-air path.** It cannot be updated through the goggles. |
| **Goggles 3** | — | **Either**: DJI Fly on a phone (Profile → Device Management → Firmware Update, over the OTG cable), **or** DJI Assistant 2 on a PC. |

**So the air unit is a desk job with a computer, full stop.** Plan for it: a
known-good USB-C *data* cable and a PC, before the unit is mounted.

> **Why the download page looks phone-only.** The product page
> (`dji.com/downloads/products/o3-air-unit`) lists **manuals, release notes and
> phone apps**. The desktop installer is not on it — DJI keeps Assistant 2 on its
> own **software** page, linked below. If you only looked at the product page, you
> saw apps and reasonably concluded phone-only. It is not; you were on the wrong
> page for the tool.

### 14.2 Required

| Software | Platform | What it does | Link |
|---|---|---|---|
| **DJI Assistant 2 (Consumer Drones Series)** | **Windows / macOS** | **The one you need.** Activates the O3 Air Unit and firmware-updates both it and the Goggles 3 over USB-C. | https://www.dji.com/downloads/softwares/dji-assistant-2-consumer-drones-series |
| **DJI Fly app** | iOS / Android | Goggles 3 firmware over the OTG cable, and the endpoint for the **Wi-Fi live feed** in §13.2. **Cannot touch the air unit.** | https://www.dji.com/downloads/djiapp/dji-fly |

A **DJI account** is required for activation.

### 14.3 Firmware, manuals and release notes

These pages carry **manuals, release notes and apps** — not the desktop
installer (§14.1).

| Item | Link |
|---|---|
| O3 Air Unit — manuals and release notes | https://www.dji.com/downloads/products/o3-air-unit |
| O3 Air Unit — support / specifications | https://www.dji.com/support/product/o3-air-unit |
| Goggles 3 — downloads | https://www.dji.com/downloads/products/goggles-3 |
| Goggles 3 — support | https://www.dji.com/support/product/goggles-3 |

Release-note PDFs are the place to confirm the §9.2 version gates before
concluding a bind failure is a fault. They live on DJI's CDN under dated paths
that change with each release — reach them from the download pages above rather
than bookmarking a URL.

### 14.4 Fallback

| Software | When | Link |
|---|---|---|
| **DJI Assistant 2 (DJI FPV Series)** | Only if the Consumer Drones build refuses to see the unit. Primarily for the original DJI FPV drone. | https://www.dji.com/downloads/softwares/dji-assistant-2-dji-fpv-series |

### 14.5 Not DJI, but part of this build

| Software | Why it is on this list | Link |
|---|---|---|
| **ExpressLRS Configurator** | Flash the **3.x maintenance branch** to the TX16S module and the RP1 — the specific fix for the Goggles 3 / ELRS 2.4 GHz interference in §8 R3. Do this *before* judging any link degradation. | https://github.com/ExpressLRS/ExpressLRS-Configurator/releases · https://www.expresslrs.org/ |
| **Cosmostreamer** *(optional, paid)* | Goggles 3 → USB-C → PC video for test capture (§13.2). Only needed for live video and telemetry on one screen; the goggles' own DVR is free. | https://cosmostreamer.com/products/djigoggles2/ |
| **OBS Studio** *(optional, free)* | Capture the video source and the serial-monitor window in one scene so frames stay aligned with the `MS` timestamps (§13.2). | https://obsproject.com/ |

> These URLs were collected from search results; DJI's download-center paths are
> stable but the site reorganises occasionally. If one 404s, start from
> https://www.dji.com/downloads and search the product name.

---

## 15. Sources

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
- [Schiller et al., NDSS 2023 — *Drone Security and the Mysterious Case of DJI's DroneID*](https://www.ndss-symposium.org/wp-content/uploads/2023-217-paper.pdf) (OcuSync reverse engineering; DroneID recovered, video payload not)
- [DJI — System Security white paper (OcuSync AES-256, per-session keys)](https://stockrc.com/pdfdoc/DJI%20Security%20White%20Paper.pdf)
- [DJI — Goggles 3 FAQ (Wi-Fi live feed sharing, no HDMI port, RC Pro relay)](https://www.dji.com/goggles-3/faq)
- [Cosmostreamer — video out for DJI Goggles 2/3/Integra/N3](https://cosmostreamer.com/products/djigoggles2/)
- [iFlight — how to activate and link a DJI O3/O4 Air Unit](https://iflightrc.freshdesk.com/support/solutions/articles/48001236409-how-to-activate-link-dji-o3-o4-air-unit-)
