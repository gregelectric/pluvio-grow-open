<p align="center"><img src="docs/img/pluvio_mark.png" width="90" alt="Pluvio"></p>

# Pluvio Grow — M5 Open Build

**Rain, with a brain.** 🌧️🧠

![Pluvio Grow — M5 build, sectioned](docs/img/hero_section.png)

An open, 3D-printable, **13-plant gravimetric** plant waterer you can build from
off-the-shelf **M5Stack** parts. One rotary arm visits each plant in turn and
doses it **by weight** from a single reservoir, each plant on its own schedule.
It runs **fully offline** — local dial UI + a Wi-Fi web page — and, if you want,
your build can flash our **official signed firmware** and join a Pluvio garden in
the app (see the ladder below).

> This is the **maker prototype**: syringes, a hacked stepper, bit-banged
> sensors, lots of moving parts — a genuinely fun build, not a polished product.
> The sleek injection-molded unit is a separate thing. This is the one you make
> yourself.

Brought to you by **[Pluvio Grow](https://pluviogrow.com)**, a product of
**Powell Electronic Inc.** · **Patent pending (US 64/071,683)**.

> **A word on the licence, up front rather than in the footnotes.** This is
> **source-available**, not OSI open source. Everything here is yours to build,
> modify and share for **personal, non-commercial** use. The gravimetric
> per-plant dosing mechanism is patent-pending, so commercial manufacture or
> sale isn't granted by this release — that's the one string attached, and we'd
> rather you knew it now than after you'd bought the parts. Talk to us if you
> want to build these to sell.

---

## Three ways to build it

1. **Hardware** — print the parts, source the M5 modules, assemble. *(this repo)*
2. **Basic firmware** — flash the open `pluvio_m5` Arduino sketch. Offline-only,
   no account, no cloud. *(this repo, [`firmware/`](firmware/))*
3. **Official image + claim** *(coming)* — flash our **signed** M5 release and
   **claim** your self-built unit into your garden in the app, with OTA updates.
   Same cloud as the real product, free. Best-effort / community support.

Rungs 1–2 are here now. Rung 3 is in progress.

---

## Hardware

Built around the **M5AtomS3 / AtomS3R** (ESP32-S3) on the **M5 AtomicMotion**
base. A 13-position carousel, indexed by an **AS5600 magnetic encoder**, rotates
each plant's tube under a dosing spout; a solenoid valve opens while an **HX711
load cell weighs the water actually delivered** — closed-loop, per plant.

### Bill of materials (indicative prototype pricing)

| Part | Qty | Notes |
|---|---:|---|
| **M5AtomS3R** dev kit (ESP32-S3 + 0.85″ round display + Wi-Fi) | 1 | the controller |
| **M5 AtomicMotion** base (STM32 + dual H-bridge + servo ch) | 1 | drives stepper **and** valve |
| **M5 Unit Mini Scales** (U177 — HX711 + load cell) | 1 | gravimetric dosing |
| **AS5600** magnetic rotary encoder + a **diametric magnet** | 1 | absolute arm position |
| **28BYJ-48** stepper motor — **bipolar-modded** (see wiring) | 1 | carousel drive |
| Mini **solenoid valve**, 5 V | 1 | the dosing valve |
| **Servo→DC H-bridge** module (RC-PWM input) | 1 | lets AtomicMotion's servo channel drive the valve |
| **60 mL syringe** bodies | 13 | per-plant carousel vessels |
| Female **luer-lock** adapters | 13 | tube-to-vessel |
| Small-bore **tubing** (~6 ft × 13 ≈ 78 ft) | — | one line per plant; buy in bulk |
| 5 V power supply (PSA05A-class) + USB-C cable | 1 | |
| M3 thread-forming screws (×8 mm ~12, ×20 mm ~9) | ~21 | |
| **Printed parts** (below) | 1 set | carousel disk, housing body + shroud, base plate, tank platform, dosing spout, MCU mount, leveling shim |

*The parts actually consumed in one build run ~**$80–120**; the real-world retail
spend is ~**$200–265** once minimum-order packs, bulk tubing, and the dev modules
are counted (you'll have spares). Neither figure reflects volume cost.*

### Printed parts

**Eight printed parts, included** in [`hardware/stl/`](hardware/stl/), extracted
from the CAD assembly: `carousel_disk`
(13 metering positions), `housing_body`, `housing_shroud`, `base_plate`,
`tank_platform`, `dosing_spout`, `mcu_mount`, and `leveling_shim`. See
**[`docs/img/parts_overview.png`](docs/img/parts_overview.png)** and
[`hardware/stl/README.md`](hardware/stl/README.md) for the size table.
Print in PLA/PETG, 0.2 mm layers, 3–4 walls, 20–30% infill. (Verify fit against
your modules before a full run.)

### Wiring — read this twice

This is where the "fun" is. Two non-obvious steps trip people up:

- **The 28BYJ-48 is modified from unipolar to bipolar.** You open the motor and
  cut the center-tap trace so the two coils come out as independent pairs. Then
  the coils wire **straight to the AtomicMotion H-bridges — no external driver**:

  | Coil | Wires | AtomicMotion |
  |---|---|---|
  | A | orange + yellow | **M1** (channel 0) |
  | B | pink + blue | **M2** (channel 1) |

  Driven full-step, both coils energized.

- **The solenoid valve is driven through a servo→DC H-bridge module** on
  **AtomicMotion Servo 1** (channel 1). The firmware "opens" and "closes" the
  valve by writing servo angles to that module (`setServoAngle`).

The two sensors are on **software bit-banged I²C** (both hardware I²C peripherals
are already used by AtomicMotion and the IMU):

| Device | Addr | AtomicMotion port | SCL / SDA |
|---|---|---|---|
| AS5600 encoder | `0x36` | Port B | GPIO 7 / GPIO 8 |
| U177 Mini Scales (HX711) | `0x26` | Port C | GPIO 5 / GPIO 6 |

(AtomicMotion's own STM32 `0x38` + INA226 `0x40` sit on the internal Wire bus,
GPIO 38/39; the IMU is on M5Unified's internal bus. Both untouched.)

A visual wiring diagram is in the works; until it lands, **the tables above are
the authoritative wiring**.

## Assembly (outline)

1. Print the parts; thread-form the M3 screws.
2. Do the **bipolar mod** on the stepper; wire coils A/B to M1/M2.
3. Mount the **MCU mount** (`mcu_mount`) + AtomS3 and the **U177 scale** (Port C) to the chassis.
4. Press the **diametric magnet into the dosing spout** on the stepper shaft; glue the **AS5600 under the tank platform** (Port B, 5 V, **DIR→GND**) so it reads that magnet.
5. Wire the solenoid through the servo→H-bridge module (Servo 1).
6. Seat the 13 syringe vessels, drop the **leveling shim**, plumb the dosing spout; route tubing to your plants.
7. Flash the firmware (see [`firmware/`](firmware/)) and **calibrate the carousel**.
8. Trim the 1-gal jug for `housing_shroud` clearance and screw it down from the chassis base.

A full step-by-step with photos is in [`docs/`](docs/) (and as a printable PDF).

## Firmware

See **[`firmware/README.md`](firmware/README.md)** — open the Arduino sketch,
install four libraries, set the board, upload. Offline-complete; no cloud.

## License & notices

- **Hardware designs & docs:** [CC BY-NC-SA 4.0](LICENSE) — build, remix, and
  share **non-commercially**, with attribution, under the same license.
- **Firmware (`pluvio_m5`):** released for personal/non-commercial use; see
  [`NOTICE`](NOTICE).
- **Patent pending (US 64/071,683)** on the gravimetric per-plant dosing
  mechanism, held by Powell Electronic Inc. This release permits personal, non-
  commercial building. **Commercial use/sale is not granted here** — contact us.
- **"Pluvio," "Pluvio Grow," and "Rain, with a brain."** are trademarks of Powell
  Electronic Inc. Build your own unit; please don't sell it as a Pluvio.

## ⚠️ Safety

This moves **water around electronics in your home**. Build and run it at your own
risk — there is **no warranty**. Watch for leaks, use a sensible reservoir size,
and don't leave an untested build unattended near anything water can ruin.

**Check your valve polarity on a dry bench before you connect water.** The
solenoid is driven by servo angles through an H-bridge module, and the two
angles are an *operator measurement, not something the firmware can deduce* — a
reversed pair leaves the valve energised open while the firmware believes it
shut, and nothing in the code can tell. Run a full dry cycle with the reservoir
empty and the line disconnected, confirm the valve closes when the UI says
closed, and only then introduce water. This is the failure that empties a
reservoir onto your floor.

---

<p align="center"><img src="docs/img/mabel.png" width="150" alt="Mabel"><br><em>Mabel · Quality Control</em></p>

*Made by one person who got tired of killing plants. If you build one, show us —
tag [@pluviogrow](https://pluviogrow.com). Generation Green. 🌱*
