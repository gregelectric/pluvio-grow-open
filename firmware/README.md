# Firmware — `pluvio_m5` (Arduino IDE)

The open **maker** firmware for the M5 variant: the free, **offline-complete**
local core — per-plant gravimetric dosing, the on-device dial UI, a local Wi-Fi
web UI, and basic OTA. **No cloud, no fleet, no telemetry.** If you want your
build to join a Pluvio garden in the app, see *Official signed image* in the
[top-level README](../README.md) (coming — "rung 3").

> **Provenance, stated plainly.** This is Pluvio firmware **tag `v1.0.3`** (the
> last release before the cloud/fleet features landed), lifted verbatim and
> renamed from `main.cpp` to `pluvio_m5.ino`. It shipped and ran as a release,
> so it is known-good at that tag. It builds on the **stock** Espressif ESP32
> Arduino core — no custom toolchain.

## What you need

- **Arduino IDE 2.x**
- **ESP32 boards by Espressif** — in *Boards Manager*, add this URL under
  *Preferences → Additional boards manager URLs*, then install "esp32":
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- **Libraries** (Library Manager — install these exact versions, which this tag
  was built against):
  - `M5AtomS3` — 1.0.2
  - `M5Atomic-Motion` — 1.0.1
  - `FastLED` — 3.10.3
  - `WiFiManager` by tzapu — 2.0.17
  (`WiFi`, `WebServer`, `DNSServer`, `Preferences`, `Update`, `ESPmDNS` ship with
  the ESP32 core.)

## Board settings (Tools menu)

| Setting | Value |
|---|---|
| Board | **M5AtomS3** (or *ESP32S3 Dev Module*) |
| USB CDC On Boot | **Enabled** |
| USB Mode | **Hardware CDC and JTAG** |
| Flash Size | **8MB** |
| Partition Scheme | any **dual-OTA** scheme (8MB) — OTA rollback needs two app slots |
| Upload Speed | 921600 |

These match the firmware's build flags (`ARDUINO_USB_CDC_ON_BOOT=1`,
`ARDUINO_USB_MODE=1`) and its 8 MB / dual-OTA partition assumptions.

## Build & flash

1. Open `firmware/pluvio_m5/pluvio_m5.ino` in the Arduino IDE.
2. Select the board + settings above, pick the serial port.
3. **Verify** first (compile). Then **Upload**.
4. Open Serial Monitor at **115200** — you'll see `=== Pluvio Grow Drip booting (dev, build …) ===`.

> `FW_VERSION` self-defaults to `"dev"` (the PlatformIO git-stamp hook isn't
> used here) — no action needed; the banner and mDNS just report `dev`.

## First run

- On first boot with no Wi-Fi saved, WiFiManager opens a **`Pluvio-Setup-…`**
  access point — join it, pick your network, done.
- The device serves its web UI at its mDNS name / IP for live monitoring and
  dose/interval editing; the on-device dial shows the same.
- **Calibrate the carousel** before first watering (drives into both endstops to
  learn the AS5600 zero) — see the dial UI / web UI calibrate action.

## Hardware this firmware expects

It targets the exact reference M5 build — see [Hardware & wiring](../README.md#hardware)
in the top-level README, including the two build quirks that are easy to miss:
the **28BYJ-48 unipolar→bipolar winding mod** and the **RC-servo-PWM → DC
H-bridge module** driving the solenoid valve. If your wiring differs, this image
won't run as-is.
