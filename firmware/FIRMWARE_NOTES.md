# Pluvio "Drip" — Grow-Pod Irrigation Controller

**Rain, with a brain.**

Production firmware for **Drip**, the controller in the **Pluvio Grow
Pod**: a 13-bucket automated gravimetric irrigation carousel built around
the **M5AtomS3 / AtomS3R** (ESP32-S3) on the **M5 AtomicMotion** base. A
single rotary arm visits each plant in turn and doses it **by mass** from
one shared reservoir, on a per-bucket schedule. A WiFi web UI and a
128×128 on-device dial give live monitoring, dose/interval editing,
diagnostics, and OTA updates.

The brand identity — warm **bone**, **burnt-orange**, **water-blue** —
is carried into both the web UI and the LCD.

> We build for the **M5AtomS3R** (the `drip-r` PlatformIO env). The same
> binary also runs on the AtomS3 (`drip` env) — M5Unified auto-detects
> the IMU (BMI270 vs MPU6886) at runtime.

This README is developer-and-operator facing and is reconciled against
[src/main.cpp](src/main.cpp). Anywhere the firmware diverges from the
original design intent, the **code's actual behavior is documented here**
and the divergence is listed under [Open questions](#open-questions)
rather than smoothed over.

---

## Hardware

- **M5AtomS3 / AtomS3R** (ESP32-S3) on the **AtomicMotion** base.
- **Stepper carousel:** a bipolar-modded **28BYJ-48** wired directly to
  the AtomicMotion base's two H-bridges — coil A (orange+yellow) → M1
  (ch 0), coil B (pink+blue) → M2 (ch 1). Full-step, both coils
  energized; coils are released between moves and the gearbox detent
  holds the rotor. No external driver.
- **Valve:** an external servo-to-H-bridge module on **AtomicMotion
  Servo channel 1** switches a **5 V solenoid** valve (servo angle sets
  the H-bridge polarity — the angles are effectively "open"/"closed"
  tags, not physical positions).
- **AS5600 magnetic absolute encoder** (I2C `0x36`): a 12-bit absolute
  angle from one diametric magnet on the arm's rotation axis. **This is
  the single source of truth for arm position.**
- **M5 Unit Mini Scales U177** (I2C `0x26`): a load cell with an on-board
  **HX711 + STM32**; the unit's own MCU owns HX711 timing, tare, and
  calibration. Used to measure dose mass.
- **Software bit-bang I2C bus on Port A** (the HY2.0-4P connector on the
  AtomS3R itself): **GPIO 1 = SCL, GPIO 2 = SDA**, fed through a passive
  **Unit Hub U006** splitter to *both* the AS5600 and the U177. This is a
  hand-rolled bit-bang master (`swi2c*` in [src/main.cpp](src/main.cpp)),
  **not** a hardware I2C peripheral — see "Why bit-bang" below.
- **Status LED:** a single **WS2812** on **Port B / GPIO 8**, dedicated
  to status. (There is no hall sensor — the carousel-magnet ring is
  gone.)
- **IMU** (BMI270 on AtomS3R / MPU6886 on AtomS3) on M5Unified's internal
  bus (`M5.In_I2C` @ GPIO 45/0) for tilt + motion safety.
- **AtomicMotion STM32** (`0x38`) + **INA226** (`0x40`) on the internal
  `Wire` bus (GPIO 38/39). The INA226 is used only for bus-voltage
  diagnostics.

### Why bit-bang I2C on Port A

Both ESP32-S3 hardware I2C peripherals are already committed:
`I2C_NUM_0` = `Wire` @ 38/39 (AtomicMotion), `I2C_NUM_1` = `M5.In_I2C`
@ 45/0 (IMU). Port A's GPIO 1/2 are a third bus with no free peripheral,
and earlier attempts to steal `I2C_NUM_1` after IMU init left Arduino's
`Wire1` state machine wedged on repeated-START reads. Bit-bang on
GPIO 1/2 sidesteps the peripheral and the Arduino layer entirely. The
rate (~30–50 kHz, dominated by `pinMode`/`digitalWrite` overhead) is
fine for ~50 Hz AS5600 reads and few-Hz U177 reads. This is a
**hardware-forced choice, not a preference.**

## Position model

Position is the **absolute AS5600 angle, always** — there is no homing
sweep, no hall sensor, no per-bucket magnet table, and no step count used
as truth. A diagnostic step counter (`g_stepCounter`) is incremented by
`stepN()` for debug readouts only; it is never authoritative.

### Carousel geometry

13 buckets sit in a **14-slot ring** (slot angle = 360/14 ≈ 25.714°). The
14th slot is an **unreachable dead zone at 6 o'clock**, between bucket 13
and bucket 1. Two hard mechanical endstops bound the travel, sitting
**7.4° past** bucket 1's and bucket 13's centres — on the *dead-zone
side* of each end bucket (`BUCKET_ENDSTOP_OFFSET_DEG = 7.4`). A full
home→far sweep is therefore `7.4 + 12 × 25.714 + 7.4 ≈ 323.37°`.

Because the dead zone is unreachable, **a 13→1 move travels the long way
(~308°) by geometric necessity, not a bug.** Direction selection uses an
*arc* error (distance along the reachable arc from the home stop), not a
shortest-path circle error, so the rotor never tries to cross the dead
zone.

### Calibration (`calibrateEndstops`)

Replaces all the old homing / steps-per-bucket / magnet-mapping
machinery:

1. **Motion-integrity probe:** a short CW burst must produce ≥ 3°
   (`CAL_PROBE_MIN_DEG`) of encoder motion, or calibration aborts loud —
   this separates "rotor turning, magnet tracking, coils live" from a
   decoupled magnet / dead coil / mechanical jam.
2. **Phase 1:** drive CCW into the home stop; record the raw AS5600 angle
   there as `g_encoderZeroOffset` (bucket 1's centre is 7.4° further in).
3. **Phase 2:** a short CW burst in free space determines the direction
   sign (`g_bucketAngleStepPositive` — does "motor +1 step" raise or
   lower the encoder reading).
4. Persist both + a validity flag, then park at bucket 1.

`gotoBucket(n)` is a closed-loop servo: read the encoder, step a small
burst toward the target arc position, re-read, repeat until within
`ARRIVAL_TOL_DEG` (2°) or a zero-crossing of the arc error. It is bounded
by `GOTO_MAX_ITER` (600) so a stalled rotor exits cleanly (returns
`false`); a single one-shot direction flip handles a rotor pinned against
a stop. A `gotoBucket` failure is **not** a fault — the caller skips the
bucket (or, for a home/park, logs `HOME FAIL` and re-homes next cycle).
The encoder dropping its magnet or failing I2C for `ENC_MAX_CONSEC_FAILS`
(10) reads in a row **is** a fault (`FAULT_ENCODER_LOST`).

## Dosing

Dosing is **gravimetric**: the load cell is the source of truth for
delivered volume. Weight is computed **host-side** from the U177's raw
ADC (reg `0x00`) and a persisted `(tare, gap)` pair — see `weightFromRaw`
for why the unit's own reg `0x10` weight is bypassed (its internal offset
can't be re-pushed across a cold boot, so it goes stale until re-tared).
The dispense loop closes the valve when grams-delivered hits the target,
with noise rejection (a rolling raw-sample window bounded above and below,
plus a reverse-drift detector).

A **per-bucket safety timeout** (sized at 2× the expected pour time,
from `DOSE_SEC_PER_ML`) only backstops a **non-tracking cell** — e.g. the
reservoir lifted off the cell, the cell unmounted, or a valve stuck shut.
A healthy cell always hits the target first. If the timeout fires having
seen < `DOSE_FLOW_MIN_G` of loss, it's a "no-flow" event; two in a row
fault `FAULT_NO_FLOW`.

After each pour the firmware waits a **post-valve dwell** (1–10 s, default
5) with the rotor still parked, so water already in flight in the supply
tubing lands on the bucket it belongs to instead of being smeared onto
the next one.

## States, faults, and deferrals

```
BOOTING -> READY -> WATERING -> READY        (or -> ERROR on a latched fault)
```

**Faults latch into `ERROR`** and suspend the scheduler until
acknowledged via serial `A`, a ≥ 3 s BtnA hold (in `ERROR`), or the web
**Clear Fault** button. **Three** faults are raised at runtime:

| Fault | Raised when |
|---|---|
| `FAULT_ENCODER_LOST` | AS5600 magnet absent or I2C fails for `ENC_MAX_CONSEC_FAILS` reads in a row. Position is unknown; **blocks motor ops** until cleared. |
| `FAULT_NO_FLOW` | Two consecutive no-flow dispenses, or the cell reading drifts *up* for several samples mid-pour. |
| `FAULT_ROTOR_STALL` | The rotor cannot traverse: a second unreachable bucket in one cycle, or the post-calibration park to bucket 1 fails to move. Clearing it **forces a recalibration on the next cycle** so the rig re-establishes home and re-runs the motion-integrity probe before resuming. |

> `FAULT_RESERVOIR_EMPTY` exists in the enum but is **never raised** — a
> low/empty reservoir now *defers* (below). See
> [Open questions](#open-questions).

**Water-safety conditions DEFER, they do not fault.** A deferral returns
the cycle cleanly to `READY` without stamping the due bucket(s); the
scheduler simply retries next tick. **No acknowledgement is needed** —
fix the condition and the next cycle runs:

| Defer reason | Condition |
|---|---|
| `DEFER_NOT_LEVEL` | Rig tilted past the spill threshold (IMU). |
| `DEFER_MOVING` | Rig jostled above the motion threshold (IMU). |
| `DEFER_RESERVOIR_LOW` | Reservoir below the sum of doses (pre-cycle) or emptied mid-cycle. |

A resolved deferral auto-clears within ~2 s. Motor operations (homing,
calibration, headless arm moves, diagnostic tests) are **never** gated by
tilt or motion. The active reason is shown in the web UI, on the LED, and
in `/api/state`.

### Sealed-unit survival features

The production unit is **sealed and battery-backed** — no reset button,
no serial port, and you can't power-cycle by unplugging. Several features
exist specifically because there is no manual recovery path:

- **Self-healing load cell:** weight math is host-side off a persisted
  `(tare, gap)` pair, so a U177 reset or swap doesn't strand calibration;
  `/api/recheckscale` re-detects the unit; bus failures propagate as
  `NaN` rather than silent zeros.
- **WiFi resilience:** `loop()` re-attempts an STA reconnect (and
  re-registers mDNS) every 30 s if the link drops. The rig runs fully
  without WiFi.
- **Button-only WiFi recovery:** a sealed unit with stale/wrong stored
  credentials never connects and is unreachable over the network, so
  there is no remote way to fix it. **Holding BtnA for ≥ 10 s while idle
  (`READY`)** wipes the stored credentials and reboots; the auto-AP path
  then brings the unit up in the config portal for re-provisioning. This
  needs no cable and no boot-window timing (unlike serial `W` or the 5 s
  hold-at-boot), works with WiFi down, and is gated to `READY` so it can
  never fire mid-dispense. The panel shows a cancellable countdown ~2 s
  into the hold (see [BtnA gestures](#btna-gestures)).
- **Watchdog:** intended as the sole automatic hang recovery — but note
  the firmware does **not** explicitly configure a task/loop watchdog
  (see [Open questions](#open-questions)).

> ⚠️ Diagnostic evidence is fragile on a sealed unit: the diagnostic log
> is a **RAM ring buffer** (lost on any reset), the richest motion trace
> is **serial-only** (`logDebug`, invisible with no serial port), and
> **no fault record or reset reason is persisted**. See
> [docs/ANALYSIS.md](docs/ANALYSIS.md) for a full robustness &
> observability review.

## Status LED

A single WS2812 on the rotary arm mirrors controller state via a strict
priority ladder (first match wins), in a three-color / two-animation
language: **green** = healthy, **blue** = water in motion, **amber** =
auto-clearing warning or user pause, **red** = latched fault. Master
brightness is configurable; `0` disables the LED entirely.

| Priority | State | LED |
|---|---|---|
| 1 | Latched fault (`ERROR`) | Red slow pulse |
| 2 | WiFi config (AP) mode | Cyan blink (1 Hz) |
| 3a | Pressed while position untrusted | Brief red flash (~300 ms) |
| 3b | Headless dose-pulse (after advance/adjust) | Blue pulses (one per 10 mL; one gray pulse for skip) |
| 3c | Headless — parked at a bucket | Steady blue scaled by that bucket's dose (warm gray for skip) |
| 4 | Watering / dispensing | Blue breathe, brightness ceiling rises with dose progress |
| 4b | Demo loop between cycles | Green slow breathe |
| 5 | Deferred: not level | Amber slow breathe |
| 5 | Deferred: reservoir low | Amber slow pulse |
| 6 | Scheduler paused | Amber slow breathe |
| 7 | Reservoir below sum of doses (idle) | Amber slow breathe |
| 8 | Motor op running (home / calibrate / test / rotate / arm hop) | Green slow breathe |
| 9 | Booting | Dim green, solid |
| 10 | Operating without WiFi | Dim neutral gray, solid |
| 11 | Healthy idle | Green slow breathe |

> A `DEFER_MOVING` deferral has no dedicated rung — it's transient and
> clears within a tick or two, falling through to the idle rung.

The on-screen web "arm" mirrors the LED's live colour and the
instantaneous animation envelope, so the screen matches the physical arm.

## On-device UI (128×128 LCD)

A radial dial that mirrors the web UI and the physical carousel,
composited off-screen and pushed in one frame (no tearing):

- A bone **reservoir tank** fills the dial with a gravity-style water
  level that drains as the cycle dispenses (a cyan line marks the
  surface; turns to a red droplet warning when the tank can't cover all
  doses).
- **13 buckets** of 6 concentric rounded dose rings (one ring per 10 mL,
  inner lights first). A skip bucket shows all rings unlit.
- A **yellow/orange selection halo** marks the selected (or arm-parked)
  bucket.
- The **centre disk** shows two status lines (time to next cycle +
  reservoir `mL / mL` normally; live `delivered / target` while watering;
  a red `FAULT` banner in `ERROR`).
- A **water droplet** sits in the 6 o'clock dead zone.

The panel dims after 60 s idle and sleeps after 5 min; any interaction
(button, serial, web request, or an attention-worthy state) wakes it. A
wake tap is swallowed so it doesn't also fire a gesture.

## BtnA gestures

| Gesture | Action |
|---|---|
| Short press | Advance the arm to the next bucket and park there (headless "arm-as-UI") |
| Double press | Cycle the parked bucket's dose (skip → 10 → 20 → … → 60 → skip) |
| Triple press | Trigger a watering cycle now |
| Long press (≥ 1 s) | Tare the scale |
| Extra-long (≥ 3 s) in `ERROR` | Clear the fault |
| Hold ≥ 10 s while idle (`READY`) | **Reset WiFi credentials** and reboot to the config portal (see [Sealed-unit recovery](#sealed-unit-survival-features)) |
| Any press during motion | Abort the current cycle / test |

Hold BtnA for ≥ 5 s **at boot** to enter WiFi config. The setup hotspot
is `Pluvio-Setup-<code>` (password `pluvio12`), captive portal at
`192.168.4.1`.

The **≥ 10 s idle hold** is the *runtime* WiFi-reset gesture, distinct
from the boot-time 5 s one above: it works at any time the unit is sitting
in `READY` (no cable, no reboot-window timing needed). About 2 s into the
hold the panel shows a `Hold to reset WiFi` countdown with a filling
progress bar; **releasing before 10 s cancels** with no change. At 10 s it
briefly shows `Resetting WiFi...`, wipes the stored credentials (the same
path as serial `W`), and reboots — the auto-AP path then comes up in the
config portal. The gesture is gated to `READY`, so it can **never** fire
during watering or active motion, and it works even with WiFi down (it is
for exactly that case).

The headless arm session is **RAM-only**: every boot starts with the arm
at home and no session. A short press while position is untrusted
(uncalibrated or encoder unhealthy) flashes red and does nothing. A
session auto-parks back to bucket 1 after 30 s idle.

## Aborting a cycle

Stop a running cycle (or a diagnostic test) three ways: **BtnA**, serial
`x`, or the web **Cancel** button (`POST /api/abort`). Any of them stops
the current dispense or motion, closes the valve, and parks the carousel
back at bucket 1 so it always ends at a known position.

That final park to bucket 1 is **uninterruptible** — button presses,
serial `x`, and Cancel taps during the park are deliberately dropped so a
held button can't bump the selection, tare the scale, or fire another
action once the park finishes. If the park can't reach bucket 1 (jam),
the controller logs the failure and forgets the rotor position; the next
cycle re-homes from scratch.

## How settings are saved

Bucket doses, intervals, the scale tare, and each bucket's last-watered
time save automatically — no "save" button. To spare the flash, changes
are **coalesced**: a write waits for a ~3 s lull (`PERSIST_DEBOUNCE_MS`)
and commits everything at once, so rapid edits become one write.

Some moments commit **immediately**: a watering cycle finishing, a fault
occurring, a firmware update starting, or a WiFi reset. The only catch:
change a setting and pull power within ~3 s (outside those moments) and
that one change may not have been written. Wait a few seconds (or let a
cycle run) and it's durable.

## Identifying your device on the network

Each unit derives a unique 6-character code from its factory MAC (e.g.
`bccca0`) and uses it everywhere, so multiple Drips coexist with no
setup:

- **Web address:** `http://pluvio-<code>.local/` (e.g.
  `http://pluvio-bccca0.local/`). If `.local` won't resolve, use the IP
  printed on the USB serial console at boot.
- **Network/mDNS name:** `pluvio-<code>`.
- **Setup hotspot:** `Pluvio-Setup-<code>` (password `pluvio12`).

The boot log prints `Device identity: name=pluvio-bccca0
ap=Pluvio-Setup-bccca0` right after the banner. The code is recomputed from
the chip every boot, so it never needs saving and survives a board swap.

## Web UI

Once on WiFi, browse to `http://pluvio-<code>.local/`. The page centres on
a **circular carousel diagram that mirrors the physical device**: a ring
of 13 plant wedges radiating clockwise from the bottom, a bottom-centre
**dead zone** with a home-stop tick, lit blue segments (10 mL each) per
plant, a drop-shaped centre **arm** that rotates to the tended plant and
matches the LED colour (with a flowing halo while watering), a status
centre, and a draining **reservoir** that turns red when short.

Click a wedge (or the ◀ ▶ controls) to change a plant's dose/interval or
skip it. Below the dial: primary actions (Water Now, Pause/Resume,
Clear Fault) and a **Settings drawer** (Reservoir & scale, Carousel &
motion, Tuning, Diagnostics, live log viewer). A Firmware Update link
points to `/update`. The page polls a few times a second while
watering/moving, ~once a second at rest.

## REST API

Action endpoints that would block (water, home, calibrate, tests,
rotate, demo) only **set a flag**; the foreground `loop()` consumes one
per pass and runs the motion off the request thread, so HTTP never blocks
on motion. `handleClient()` runs every loop tick, including during
watering (the dispense loop's `delay()` windows let polls land).

| Endpoint | Method | Notes |
|---|---|---|
| `/api/state` | GET | Full controller state (JSON) |
| `/api/water` | POST | Queue a full cycle (force all non-skip; `READY` only) |
| `/api/water/bucket?n=N` | POST | Water exactly one bucket (1-based; `READY` only) |
| `/api/abort` | POST | Cancel the running cycle / abortable motion |
| `/api/home` | POST | Servo to bucket 1 (re-home) |
| `/api/calibrate` | POST | `?grams=G` → U177 gap calibration (inline); no args → endstop calibration (queued) |
| `/api/tare` | POST | Tare load cell |
| `/api/setresvfull?value=ML` | GET | Set reservoir-full volume (mL), 100..20000 — UI gauge full-scale, display-only |
| `/api/recheckscale` | POST | Re-detect the U177 |
| `/api/clearfault` | POST | Acknowledge fault |
| `/api/select?bucket=N` | GET | Select bucket N (1-based; web highlight only) |
| `/api/dose?bucket=N&delta=±10` | GET | Adjust bucket N's dose (snaps to ladder) |
| `/api/interval/set?bucket=N&hours=H` | GET | Set interval (12/24/48/72/120/168/240/336/504) |
| `/api/skip?bucket=N` | POST | Set bucket N to skip (dose 0) |
| `/api/valve` | POST | Toggle valve, or `?open=0\|1` |
| `/api/setvalveangles?open=O&close=C` | GET | Set valve open/closed servo angles (0..180) |
| `/api/setdwell?value=N` | GET | Post-valve drain dwell, 1..10 s |
| `/api/setledbrightness?value=N` | GET | Status-LED brightness, 0..255 (0 = off) |
| `/api/setstepprofile?fast=&slow=&ramp=` | GET | Step rate (`fast` only; `slow`/`ramp` accepted as no-ops) |
| `/api/setmotorinv` | POST | Toggle motor direction, or `?enable=0\|1` |
| `/api/stepper/rotate?dir=fwd\|rev` | POST | Continuous rotate test |
| `/api/stepper/stop` | POST | Stop a rotate test |
| `/api/test/dispense?wet=0\|1` | POST | Dispense test (dry = valve stays shut) |
| `/api/demo/start?mode=dry\|wet\|random` | POST | Start the continuous demo loop |
| `/api/demo/stop` | POST | Stop the demo |
| `/api/demo/config?pause_s=N` | POST | Set inter-cycle pause (1..10 s) |
| `/api/demo` | GET | Demo status |
| `/api/scheduler` | POST | Toggle scheduler, or `?enable=0\|1` |
| `/api/debug/fastsched` | POST | Toggle debug fast-schedule (intervals as seconds) |
| `/api/imu/threshold?spill=&motion=` | POST | Set spill (deg) / motion (g) thresholds |
| `/api/imu/mount?orientation=` | POST | Set mount (Z_UP/Z_DOWN/Y_UP/Y_DOWN/X_UP/X_DOWN) |
| `/api/resetwifi` | POST | Forget WiFi and reboot to AP mode |
| `/api/log?since=N` | GET | Pull diagnostic log lines with seq > N |
| `/api/menu` | GET | Plain-text serial menu reference |
| `/update` | GET / POST | Firmware upload form (GET) + multipart sink (POST), backed by ESP32 `Update` |

Notes:
- `/api/state` reports `selected` **0-based**; every control endpoint
  takes **1-based** bucket numbers.
- There is **no** stepper autotune endpoint and **no** position-test
  endpoint (both were removed; see [Open questions](#open-questions)).

## Serial menu

Send `?` over USB serial (115200 baud) for the full menu. Highlights:

| Cmd | Action |
|---|---|
| `w` | Water now |
| `h` | Home (servo to bucket 1) |
| `C` | Calibrate endstops (drive into both stops; learn encoder zero + direction) |
| `t` | Tare / capture empty-cell zero for calibration |
| `c <g>` | Calibrate U177 gap value with a known mass (tare first, in-session) |
| `F <mL>` | Set reservoir-full volume (UI gauge full-scale, 100..20000); bare `F` prints |
| `s` | Re-detect U177 scale |
| `d N V` | Set bucket N dose to V mL (0 = skip, else 10..60) |
| `i N H` | Set bucket N interval to H hours |
| `D V` | Dispense test (`0` dry / `1` wet) |
| `V O A` / `V C A` | Set valve open / closed angle (0..180) |
| `Y V` | Post-valve dwell, 1..10 s |
| `B V` | Status-LED brightness, 0..255 |
| `P F V` | Step rate (fast µs); `P S`/`P R` are no-ops; bare `P` prints |
| `R` / `r` | Rotate forward / reverse continuously (`x`/button to stop) |
| `E` | Toggle scheduler (pause / resume) |
| `Z` | Toggle debug fast-schedule (intervals as seconds) |
| `L V` / `M V` / `O N` | Spill threshold (deg) / motion threshold (g) / mount (0..5) |
| `X` | Toggle demo mode (dry; wet/random demos are web-only) |
| `A` | Acknowledge fault |
| `p` | Print state |
| `x` | Abort current motion |
| `W` | Reset WiFi credentials and reboot |

> The serial menu has **no autotune (`Q`) and no position test (`T`)** —
> those commands do not exist in this firmware.

## Persistence

Stored in NVS under namespace `irrig`, surviving reboots:

- U177 gap (calibration) value, plus the raw-tare baseline + validity
  flag captured at tare time
- Reservoir-full volume (UI gauge full-scale; default 2500 mL, NVS key
  `resvFullMl`, band 100–20000 mL) — display-only, does not gate dispensing;
  set via serial `F` or `GET /api/setresvfull?value=`
- Per-bucket dose volumes, intervals, and last-watered timestamps (the
  latter only in NTP/wall-clock mode — uptime values are meaningless
  across reboots)
- Endstop calibration triple: encoder zero offset, direction sign,
  validity flag
- Valve open/closed angles and post-valve dwell
- Step rate (`stepDelayFast`); motor-direction inversion
- Status-LED brightness
- IMU spill/motion thresholds and mount orientation
- Scheduler enabled/paused
- WiFi credentials (managed by WiFiManager)

The bucket count is fixed at 13 and not persisted. The debug
fast-schedule toggle and demo state are RAM-only and reset to off every
boot. On boot, stale pre-AS5600/pre-U177 keys (`hxCounts`, `tareOff`,
`stepsPerBucket`, `magPos`, …) are removed.

Scheduling uses a **time-source cascade**: NTP wall-clock if WiFi+NTP
succeed (reboot-survivable due times), otherwise seconds-since-boot
(every bucket treated as "just watered" at boot to avoid a cycle storm).

## Build / upload

PlatformIO. The default env is `drip` (AtomS3); we target the AtomS3R via
`drip-r`. Both envs build an **identical binary** — M5Unified
auto-detects the IMU at runtime; the split exists only for per-board
upload tagging.

```
pio run -e drip-r            # build for AtomS3R (preferred)
pio run -e drip-r -t upload  # USB upload
pio device monitor           # serial console (115200)
pio run                      # build the default (drip / AtomS3) env
```

After first WiFi provisioning, OTA updates push from the browser at
`http://pluvio-<code>.local/update` — no USB cable needed.

> **Bring-up debug left in `setup()`:** the firmware currently runs an
> I2C bus scan (both buses) and an IMU read dump at boot, tagged
> `[DBG-I2C-SCAN]` / `[DBG-IMU-CHECK]` with "remove before commit"
> comments. Harmless, but noisy on the serial console.

## Troubleshooting

**The selected bucket / dose / tare changed on its own right after I
aborted a cycle.** A button press during the uninterruptible park-back
used to be "remembered" and acted on. It's fixed — presses during the
park are dropped on the spot. If you still see it, update from
`http://pluvio-<code>.local/update`.

**The rig says it deferred and never watered.** A deferral (not level,
moving, reservoir low) is normal and self-clearing — level the rig,
hold it still, or refill the tank and the next cycle runs. No
acknowledgement needed. The reason shows in the web UI and on the LED
(amber).

**Position untrusted / a short press flashes red.** The endstops aren't
calibrated or the encoder is unhealthy. Run **Home** or **Calibrate**
from the web UI; the next scheduled cycle also auto-calibrates.

---

## Open questions

These are places where the code and the stated design intent don't line
up, or where intent is genuinely ambiguous from the code plus comments.
Nothing here is guessed to fill a gap — each is grounded in
[src/main.cpp](src/main.cpp).

1. **Dry reservoir: intent says "fault," code defers.** The design
   intent lists *dry-reservoir* among the latched faults, but the
   firmware *defers* on a low/empty reservoir (`DEFER_RESERVOIR_LOW`) and
   never raises `FAULT_RESERVOIR_EMPTY`. That enum value, its name, and
   its banner label are **vestigial** (`enterFault` is only ever called
   with `FAULT_ENCODER_LOST` and `FAULT_NO_FLOW`). Intended behavior, or
   leftover dead code to remove?

2. **`WIFI_AP_STA` fallback — not implemented as stated.** The intent
   describes an `AP_STA` fallback, but the code runs **STA-only**
   (`WiFi.mode(WIFI_STA)`), and on STA-connect failure it **continues
   without WiFi** rather than bringing up an AP. AP/config mode is a
   separate, mutually-exclusive boot path entered only by holding BtnA
   ≥ 5 s at boot (a blocking WiFiManager portal, then reboot). There is
   no simultaneous AP+STA and no automatic fall-back to AP. Is the
   STA-only-with-manual-portal design the intended one?

3. **The watchdog is assumed but not configured.** The intent treats the
   watchdog as the sole automatic hang recovery on a sealed unit, but the
   firmware contains **no explicit watchdog setup** — no
   `esp_task_wdt_*`, no `enableLoopWDT()`. Any protection would come from
   the ESP32 Arduino core's defaults, which do not subscribe the
   Arduino `loop` task to the task watchdog. Several paths can block the
   loop for a long time (`rotateContinuous` spins until abort;
   `calibrateEndstops`/`gotoBucket` can run for thousands of steps). Is a
   watchdog meant to be configured, and if so for which task/timeout?

4. **No persisted fault record or boot reset-reason.** Given the sealed,
   battery-backed, no-serial constraints, the absence of any persistent
   fault/reset-reason capture means a field unit that reboots itself
   leaves no trace of why. This is the top item in
   [docs/ANALYSIS.md](docs/ANALYSIS.md). Is durable fault telemetry in
   scope for this product?

5. **Diagnostic evidence is RAM-only / serial-only.** The diagnostic log
   ring is RAM (lost on the reset the watchdog itself triggers), and the
   richest motion/calibration trace is `logDebug` (USB-serial only —
   nonexistent on a sealed unit). Intended, or should the field-relevant
   subset move to the persisted/ring path? (Detailed in ANALYSIS.md.)

6. **`getRawADC()` no-arg returns 0 on bus failure, and
   `captureScaleTare()` uses it.** A tare during a transient bit-bang bus
   glitch can silently persist `rawTare = 0` with no validation or log,
   skewing every later weight. The calibration paths already guard with a
   liveness read; the tare path does not. Intended asymmetry, or a gap?

7. **`g_selected` (web highlight) vs the headless arm session are
   independent.** `/api/select` moves only the web card highlight;
   BtnA short-press moves only the physical arm/`g_interactionParkedBucket`.
   They never sync. This is by design per the comments, but is a common
   source of "the screen and the arm disagree" confusion — worth
   confirming it's the intended UX.

8. **Endstop calibration cadence — resolved.** `runWateringCycle` no longer
   recalibrates every cycle. It recalibrates at most once per
   `CALIBRATE_EVERY_N_CYCLES` (= 3) cycles, plus on demand when any of:
   the saved calibration isn't currently trusted (`isSavedCalTrusted()`),
   a one-shot force is pending (`g_forceRecalNextCycle`), the encoder is
   unhealthy/uncalibrated, a prior cycle missed a `gotoBucket`, a
   motor-direction flip, a drive-wedge recovery, or a `FAULT_ROTOR_STALL`
   clear. Between calibrations the absolute AS5600 is trusted and the rotor
   is not driven into the stops — bounding mechanical wear while keeping
   home re-established whenever position trust is in doubt. A non-calibrating
   cycle also skips the home/return parks (it sweeps the due set from
   wherever the arm idles), saving carousel travel.
