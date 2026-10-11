// Drip - production firmware for the M5AtomS3 +
// AtomicMotion irrigation rig. Boot reads the AS5600 magnetic
// rotary encoder for absolute arm position (no homing sweep); runtime
// drives a 12-hour watering cycle with per-bucket dose volume control
// and live UI on the 128x128 display. The UI shows a top countdown
// strip, a center row of vertical segmented blue bars (one per bucket,
// lit segments = dose in 10mL steps), and a bottom reservoir strip.
// The selected bucket gets a yellow underline plus its 1-indexed
// number rendered in small text directly below the underline.
//
// Hardware:
//   - M5AtomS3 / AtomS3R on AtomicMotion Base
//   - 28BYJ-48 stepper (bipolar-modded) coils wired directly to the
//     AtomicMotion Base H-bridges:
//       coil A (orange + yellow) -> M1 (ch 0)
//       coil B (pink + blue)     -> M2 (ch 1)
//     Driven full-step, both coils energized. No external driver.
//   - External servo-to-DC-H-bridge module on AtomicMotion Servo 1
//     (ch 1), driving the 5V solenoid valve. setServoAngle controls
//     the module: g_valveOpenAng = open, g_valveCloseAng = closed.
//   - Two SOFTWARE bit-banged I2C buses, one device each (the passive
//     Unit Hub U006 that used to split Port A is removed):
//       * AtomicMotion Port B (GPIO 7 SCL / GPIO 8 SDA): AS5600 magnetic
//         rotary encoder (I2C 0x36): 12-bit absolute angle, one diametric
//         magnet on the arm rotation axis. GPIO 8 was the WS2812 status
//         LED; the LED output is now disabled to free the pin.
//       * AtomicMotion Port C (GPIO 5 SCL / GPIO 6 SDA): M5 Unit Mini
//         Scales U177 (I2C 0x26): load cell with on-board HX711 + STM32;
//         tare/calibration live on the unit's own MCU. The U177 STM32
//         clock-stretches between register-write and data-read, which the
//         bit-bang read path now honors (see swi2cSclReleaseWait).
//     Software bit-bang -- NOT a hardware peripheral -- because both
//     ESP32-S3 I2C peripherals are already spoken for: I2C_NUM_0 by Wire
//     (AtomicMotion @ 38/39) and I2C_NUM_1 by M5.In_I2C (IMU @ 45/0).
//     Earlier attempts to steal I2C_NUM_1 after the IMU init left
//     Arduino's Wire1 state machine wedged on repeated-START reads.
//     Bit-bang sidesteps the conflict entirely; the slow rate (~30-50 kHz
//     given pinMode/digitalWrite overhead) is fine for AS5600 reads at
//     ~50 Hz and U177 reads at a few Hz. See swi2c* helpers below.
//   - AtomicMotion STM32 @ I2C 0x38 + INA226 @ 0x40 on the internal
//     bus (GPIO 38/39, the Wire peripheral). Untouched.
//   - IMU (BMI270 on AtomS3R / MPU6886 on AtomS3) on M5Unified's
//     internal bus M5.In_I2C @ GPIO 45/0 (I2C_NUM_1). Untouched --
//     accessed via AtomS3.Imu.* only.
//
// Carousel topology (encoder-driven):
//   13 buckets arranged in a 14-slot ring (slot angle = 360/14
//   ~= 25.714 deg). The 14th slot is the dead zone at 6 o'clock,
//   between bucket 13 and bucket 1 -- physically unreachable. Two
//   hard mechanical endstops bound the travel, sitting symmetrically
//   7.4 deg PAST bucket 1's and bucket 13's centres (NOT directly over
//   them -- they're on the dead-zone side of each bucket). So a full
//   home->far sweep covers
//     BUCKET_ENDSTOP_OFFSET_DEG       (home stop -> bucket 1 centre)
//     + 12 * BUCKET_SLOT_DEG          (bucket 1 -> bucket 13)
//     + BUCKET_ENDSTOP_OFFSET_DEG     (bucket 13 centre -> far stop)
//   = ~323.37 deg. Calibration drives into both stops and records:
//     g_encoderZeroOffset       - raw AS5600 angle (deg) at the CCW
//                                 (home) endstop. Bucket 1's centre is
//                                 g_encoderZeroOffset + 7.4*sign.
//     g_bucketAngleStepPositive - true if "motor +1 step" increases
//                                 the encoder reading
//   Bucket n's absolute encoder target =
//     (g_encoderZeroOffset + (BUCKET_ENDSTOP_OFFSET_DEG +
//      (n-1)*BUCKET_SLOT_DEG)*sign) mod 360,
//   sign = +1 if g_bucketAngleStepPositive else -1.
//   There are no per-bucket magnets, no hall sensor, no step count as
//   position truth -- the encoder is the source of truth at all times.
//
// Runtime overview:
//   - States: BOOTING -> READY -> WATERING -> READY (or ERROR).
//   - Faults: two consecutive no-flow events, an empty reservoir, or
//     an AS5600 read failure / magnet-missing transition to ERROR
//     with a red banner over the bottom strip. The scheduled cycle
//     is suspended in ERROR. Acknowledge with serial 'A' or a >=3s
//     BtnA hold.
//   - Motion: stepN() runs a hand-rolled ramp (SLOW -> FAST -> SLOW)
//     using delayMicroseconds() between phase writes. Both coils are
//     always driven at +/-127 (full magnitude) for full-step drive.
//     After each move, stepperReleaseCoils() drops the H-bridges to
//     zero and the gearbox detent holds the rotor.
//     gotoBucket() is a closed-loop servo: step a small burst, re-read
//     the encoder, repeat until within ARRIVAL_TOL_DEG of the target.
//     Bounded iteration count detects a stalled rotor (returns false,
//     no fault -- caller skips the bucket).
//     calibrateEndstops() drives into both mechanical stops to learn
//     g_encoderZeroOffset and direction sign; persists both.
//   - BtnA: short-press cycles the selected bucket; double-press
//     cycles its dose; triple-press triggers a watering cycle now;
//     long-press (>=1s) tares the scale; extra-long (>=3s) clears
//     a fault when in ERROR. Any press during WATERING / 'D' / 'h'
//     aborts the running motion. A 10 s hold while idle (READY) wipes
//     stored WiFi credentials and reboots to the config portal -- the
//     runtime, cable-free recovery for a sealed unit with stale creds
//     (see WIFI_RESET_HOLD_MS / runWifiResetHoldGesture).

#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <M5AtomS3.h>
#include <M5AtomicMotion.h>
#include <Preferences.h>
// WiFi / web UI / OTA additions. Sync WebServer is intentional;
// OTA uses the ESP32 core's Update library directly (no third-party
// dep, no advertising banners) -- see initWebServer() for the
// /update GET form + POST streaming handler.
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <Update.h>
#include <esp_ota_ops.h>   // OTA app-rollback: partition state + mark-valid API
#include <ESPmDNS.h>
#include <time.h>
// LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
// Status LED. FASTLED_INTERNAL silences FastLED's compile-time
// version/pragma banner (same "no advertising banners" stance the
// OTA path takes against ElegantOTA). Single WS2812 driven via the
// ESP32-S3 RMT clockless backend.
// #define FASTLED_INTERNAL
// #include <FastLED.h>

// -------- Pin map --------
// AtomicMotion internal I2C bus: GPIO 38/39, the Wire peripheral
// (I2C_NUM_0). Carries the onboard STM32 (0x38) + INA226 (0x40) only.
//
// Two independent software-bit-bang I2C buses (see swi2c*), one device
// each -- the passive U006 hub on Port A is removed:
//   * AtomS3R Port A (the HY2.0 connector on the AtomS3R itself):
//     GPIO 1 (SCL) / GPIO 2 (SDA). LEGACY -- no longer used; the two
//     sensors moved to their own AtomicMotion-base ports below.
//   * AtomicMotion Port B: GPIO 7 (SCL) / GPIO 8 (SDA) -> AS5600
//     encoder (0x36). GPIO 8 was the WS2812 status LED; the LED is
//     disabled (see resolveLedState/updateLed) to free the pin. GPIO 7
//     was the now-removed carousel hall sensor.
//   * AtomicMotion Port C: GPIO 5 (SCL) / GPIO 6 (SDA) -> U177 Mini
//     Scales (0x26).
// External pull-ups are present on both Port B and Port C, so the
// open-drain bit-bang drivers need no internal pulls. NOTE: confirm the
// SDA/SCL assignment against the physical harness -- swapping the pair
// on a port makes every transfer NACK. Port A's prior assignment was
// SCL=lower GPIO / SDA=higher GPIO; Ports B and C follow the same
// convention here.
#define I2C_SDA_PIN     38
#define I2C_SCL_PIN     39
// Encoder bus (AS5600 0x36) on AtomicMotion Port B: SCL=GPIO7, SDA=GPIO8.
// Follows the same lower-GPIO=SCL convention as Port C (scale bus).
#define ENC_SWI2C_SCL_PIN    7
#define ENC_SWI2C_SDA_PIN    8
// Scale bus (U177 0x26) on AtomicMotion Port C.
#define SCALE_SWI2C_SCL_PIN  5
#define SCALE_SWI2C_SDA_PIN  6
// Half-period delay between SDA/SCL transitions. At 3us nominal the
// open-drain pinMode+digitalWrite overhead on the ESP32-S3 dominates;
// measured rate ends up ~30-50 kHz, well under I2C standard mode's
// 100 kHz ceiling. Both AS5600 and the U177 STM32 tolerate any rate
// down to DC.
#define SWI2C_HALF_US    3
// Worst-case time the master will wait for a slave to release (stop
// holding low) SCL before aborting the transaction -- real bit-bang
// clock-stretch handling, not a fixed delay. The U177's STM32
// clock-stretches between the register-pointer write and the first
// data byte while it services the I2C request off its main loop;
// observed stretch is sub-millisecond, with a few-ms tail under HX711
// conversion load. 8 ms sits an order of magnitude above that tail yet
// well below the SMBus 25/35 ms stretch ceiling, and is short enough
// that a wedged or absent slave aborts inside one ~20 ms encoderTick /
// loop iteration without tripping the task watchdog. The AS5600 never
// stretches, so this bound is only ever reached on a faulted scale bus.
#define SWI2C_STRETCH_TIMEOUT_US      8000
// Rate-limit for the clock-stretch-timeout log (per bus): counted
// always, logged at most this often so a stuck bus can't flood serial.
#define SWI2C_STRETCH_LOG_INTERVAL_MS 5000
// LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
// Single WS2812 RGB status LED was on AtomicMotion Port B (silkscreen
// "G33", which the AtomS3 remaps to ESP32-S3 GPIO 8). GPIO 8 is now the
// Port B I2C SDA line for the AS5600 encoder; the LED output is disabled.
// #define STATUS_LED_PIN 8

// -------- WiFi / web UI config --------
// Captive portal AP credentials are baked here (the password is the
// AP's, not a user-WiFi credential). User WiFi credentials are
// stored by WiFiManager in NVS and never appear in source.
#define AP_SSID                  "Pluvio-Setup"
#define AP_PASSWORD              "pluvio12"   // 8+ chars required by ESP32 AP mode
#define WIFI_HOSTNAME            "pluvio"
#define AP_BUTTON_HOLD_MS        5000   // hold BtnA at boot for >=5s to force AP mode
#define WIFI_CONNECT_TIMEOUT_MS  15000  // give up STA connect after this
#define WIFI_CHECK_INTERVAL_MS   30000  // recheck WiFi health this often
#define WEB_UI_TITLE             "Pluvio Drip"

// -------- Firmware identity --------
// FW_VERSION is injected by a PlatformIO build flag
// (-DFW_VERSION=\"x.y.z\" in platformio.ini) and defaults to "dev" for
// ad-hoc builds with no flag. FW_BUILD is the compiler timestamp. Both
// are emitted in the boot banner, in /api/state (fwVersion / fwBuild),
// and as mDNS TXT records, so the LAN fleet manager can inventory units
// and target OTA pushes. Defined as bare adjacent string literals (no
// wrapping parens) so they concatenate inside the boot-banner literal.
#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif
#define FW_BUILD __DATE__ " " __TIME__

// -------- Motion config --------
// Stepper drive via the AtomicMotion Base's two H-bridges. Coils wired
// directly (no external stepper driver). Full-step, both coils
// energized; phase index advances 0->1->2->3->0 for one direction,
// 0->3->2->1->0 for the other. The g_motorInverted flag swaps the
// direction sense at the software level for reversed wiring.
//
// Why a full table of four signed-PWM phases instead of microstepping
// or PWM modulation: the AtomicMotion's H-bridges only accept a
// signed speed per channel; treating them as plain step phases keeps
// the inner loop branch-free and the I2C traffic minimal (two writes
// per step). Coils are released between moves via stepperReleaseCoils
// so the gearbox detent holds the rotor without dissipating power.
#define STEPPER_M1_CH         0      // AtomicMotion M1
#define STEPPER_M2_CH         1      // AtomicMotion M2
#define VALVE_SERVO_CH        1      // Servo channel for valve (external
                                     // servo-to-H-bridge module ->
                                     // 5V solenoid). NOT the M2 motor
                                     // channel anymore; that's now coil B.
#define STEPS_PER_REV_DEFAULT 2038   // 28BYJ-48 theoretical; informational
                                     // only -- position is read from the
                                     // AS5600, not counted.
#define BUCKET_COUNT          13     // FIXED. 13 buckets in a 14-slot
                                     // ring; do not change without re-
                                     // working the physical chassis.

// -------- Bucket geometry --------
// The carousel is a 14-slot ring: 13 buckets + 1 dead zone at 6 o'clock
// (between bucket 13 and bucket 1, physically unreachable). The CCW
// (home) endstop sits BUCKET_ENDSTOP_OFFSET_DEG before bucket 1's
// centre; the CW (far) endstop sits BUCKET_ENDSTOP_OFFSET_DEG after
// bucket 13's centre. (Earlier comments / constants placed the stops
// directly over buckets 1 and 13 -- that was wrong; the stops are on
// the dead-zone side of each end bucket.) Bucket angles are derived
// purely from this geometry plus g_encoderZeroOffset (raw encoder angle
// at the home stop, i.e. BUCKET_ENDSTOP_OFFSET_DEG before bucket 1).
// The radial web UI uses the same 14-slot layout, so keeping these
// constants in sync with the JS in webUiHtml matters -- changing one
// without the other will visually misalign the dial.
#define BUCKET_SLOT_DEG          (360.0f / 14.0f)   // ~25.714 deg per slot
// Angular distance from each end endstop to the nearest bucket's centre.
// Measured mechanically: the CCW stop sits 7.4 deg before bucket 1, the
// CW stop sits 7.4 deg after bucket 13.
#define BUCKET_ENDSTOP_OFFSET_DEG  7.4f
// Full home->far sweep: endstop offset + 12 slot-widths between
// buckets 1 and 13 + endstop offset on the far side = ~323.37 deg.
#define BUCKET_HOME_TO_FAR_DEG   (2.0f * BUCKET_ENDSTOP_OFFSET_DEG \
                                  + 12.0f * BUCKET_SLOT_DEG)

// Stepping rate. delayMicroseconds() between phase writes determines
// step rate. I2C write time is NOT subtracted; real STEP/DIR drivers
// care about pulse separation, not pulse width, so the I2C transit
// time is treated as part of the step period. Every motion path
// (gotoBucket, calibrateEndstops, rotateContinuous) runs at this one
// rate -- there used to be a slow/ramp profile but mid-arc slowdowns
// were tripping the calibration stall detector and the closed-loop
// servo already handles approach without a ramp.
#define STEP_DELAY_FAST_US_DEFAULT      3000   // ~333 steps/sec cruise
#define STEP_DELAY_FAST_US_MIN           100
#define STEP_DELAY_FAST_US_MAX         10000

// Coil settling delay after stepperWake() re-energizes the saved
// phase, giving the rotor time to pull against the detent before
// stepping resumes. Used by both gotoBucket() and calibrateEndstops().
#define COIL_WAKE_SETTLE_MS         50

// -------- Closed-loop motion --------
//
// gotoBucket() drives the rotor toward a bucket's encoder target as a
// servo: pick step direction from the signed angular error (shortest
// path), run a short burst, re-read the encoder, repeat. No magnets,
// no hall sensor, no slip recovery -- the encoder IS truth, the rotor
// either reaches the target or doesn't.
//
// Tuning notes:
//   ARRIVAL_TOL_DEG     - how close (signed-shortest-path) is "there"
//                         on the band-pass arrival check. Tight: the
//                         loop also exits on a zero-crossing of arc_err
//                         (sign flip = crossed target = at-or-past), so
//                         a small tolerance does not cause overshoot
//                         oscillation -- it just means the loop exits
//                         on the cross instead of within the band.
//   GOTO_STEP_BURST     - steps between encoder reads. Bigger = faster
//                         convergence but more overshoot risk; smaller
//                         = more I2C reads. 4 is a sweet spot at the
//                         3ms step period (~12ms per re-read).
//   GOTO_MAX_ITER       - bounded iteration count to detect a stalled
//                         rotor. At 4 steps/burst this is up to
//                         4 * 600 = 2400 steps per gotoBucket call,
//                         comfortably more than the full home->far
//                         travel (~1900 steps at ~5.7 steps/deg) so a
//                         worst-case CW-stop -> bucket 1 traversal of
//                         the long arc fits with margin. Early exit on
//                         arrival keeps typical calls short.
#define ARRIVAL_TOL_DEG          2.0f
#define GOTO_STEP_BURST             4
#define GOTO_MAX_ITER             600
// Max post-cross error the zero-crossing exit will accept as a real
// arrival. A genuine target cross lands a fraction of a burst past the
// target: GOTO_STEP_BURST (4) steps at ~5.7 steps/deg is ~0.7deg, so a
// legitimate single-burst overshoot is at most ARRIVAL_TOL_DEG + ~0.7
// ~= 2.7deg past. A sign flip with a LARGER magnitude is not a target
// cross at all -- it's the rotor crossing the 360->0 dead-zone wrap
// boundary, where arcErrorDeg jumps ~360 and flips sign at large
// magnitude (e.g. the observed park-into-bucket-1 entered near arc
// ~348deg and the first burst flipped the sign at |err|=7.31deg). At
// 2.5x ARRIVAL_TOL_DEG (5.0deg) the band sits comfortably above the
// ~2.7deg worst-case real overshoot yet far below that 7.31deg
// wrap-cross, so it accepts true arrivals and rejects wrap crossings.
#define GOTO_ZEROCROSS_MAX_ERR_DEG  (2.5f * ARRIVAL_TOL_DEG)
// Stop-proximity band for gotoBucket()'s no-progress direction flip.
// The single direction flip only legitimately recovers a rotor pinned
// against a mechanical stop, which can only happen in the end regions
// of the reachable arc -- arc position near 0 (home stop) or near
// BUCKET_HOME_TO_FAR_DEG (far stop). Mid-arc, a no-progress window is a
// stale bit-bang encoder-read cluster, NOT a stall; flipping there
// drives the rotor away from target and burns the iteration budget into
// a terminal HOME FAIL (the 13->1 park travels ~308deg mid-arc the whole
// way -- exactly the observed failure). Sized to BUCKET_ENDSTOP_OFFSET_DEG
// (7.4deg) for two reasons: (1) it is >= the stall window's expected
// travel -- STALL_W=8 * GOTO_STEP_BURST=4 = 32 steps ~= 5.6deg at
// ~5.7 steps/deg -- so a window that fills while the rotor is genuinely
// pinned at a stop fits inside the band; and (2) it equals the
// stop->end-bucket offset, so the band reaches no further inward than
// bucket 1's / bucket 13's centres and never covers an inter-bucket arc
// where a transient could masquerade as a stall.
#define GOTO_FLIP_STOP_PROXIMITY_DEG  BUCKET_ENDSTOP_OFFSET_DEG
// Bounded gotoBucket() move retry. A single MAX_ITER exhaustion -- e.g.
// a stale-read cluster the in-loop mid-arc recovery didn't fully absorb
// -- must not be terminal. Each retry re-reads ground truth and re-runs
// the closed-loop servo from scratch. 1 (2 attempts total) is enough:
// the observed failure was a one-shot transient and a second clean
// attempt completes. On final exhaustion the contract is unchanged
// (return false -> caller's HOME FAIL / fault path).
#define GOTO_MAX_RETRIES            1

// Endstop calibration stall detection. We drive in one direction and
// watch the encoder; if the angle moves less than ENC_STALL_DEG_CAL
// over the last STALL_WINDOW_STEPS, the rotor is against the mechanical
// stop. STALL_WINDOW_STEPS is in steps (sampled at the step rate), not
// milliseconds. The threshold must be larger than encoder noise floor
// (~0.1 deg) but small enough that a real stop registers. A real
// mechanical stop produces ~zero motion forever; a mid-arc slowdown
// (binding, friction blip) still produces small motion. Window must be
// long enough that brief slowdowns don't fully fill it with near-zero
// samples -- 64 steps is ~200ms at the fast rate, which is well past
// any transient. ENC_STALL_DEG_GOTO is the gotoBucket no-progress flip
// threshold; it serves a different purpose (one-shot direction flip)
// and keeps the previous looser value.
#define ENC_STALL_DEG_CAL          0.3f
#define ENC_STALL_DEG_GOTO         0.5f
#define STALL_WINDOW_STEPS           64
#define STALL_SAMPLE_EVERY_STEPS      8
// Maximum cumulative steps in one endstop drive. End-to-end travel is
// ~2000 steps; 4000 leaves comfortable margin for a fully unsupported
// starting position without letting a runaway grind forever.
#define ENDSTOP_MAX_STEPS         4000

// Pre-calibration motion-integrity probe. CAL_PROBE_STEPS in the CW
// direction must produce >= CAL_PROBE_MIN_DEG of encoder motion or we
// abort calibration. The AS5600 jitter floor is ~0.5deg peak-to-peak
// and CAL_PROBE_STEPS at the fast step rate produces ~9deg on a
// healthy 28BYJ-48 bipolar-mod rig, so 3deg cleanly separates "rotor
// turning, magnet tracking, coils energized" from any broken link in
// that chain (decoupled magnet, dead coil drive, mechanical jam).
// Direction is CW so a rotor parked at the home stop still has room
// to move; the FAR stop is the only false-positive case and is rare
// in practice.
#define CAL_PROBE_STEPS              50
#define CAL_PROBE_MIN_DEG          3.0f

// Per-cycle calibration cadence. runWateringCycle() used to drive into
// the home stop (calibrateEndstops) on EVERY cycle. With the AS5600 the
// absolute position is trusted between calibrations, so we recalibrate
// at most once per this many cycles -- the home drive (and the
// end-of-cycle park that pairs with it) is skipped on the cycles in
// between, which is where the carousel-travel saving comes from. A
// position loss is still self-healed within one cycle: a missed
// gotoBucket on a skipped cycle forces a recal next cycle (see
// g_forceRecalNextCycle). Set to 1 to restore the old behavior exactly
// (calibrate every cycle, park every cycle).
#define CALIBRATE_EVERY_N_CYCLES     3

// Poll cadence inside stepN's inner loop for serviceAbort(). I2C
// contention with the stepping rate means we cannot pump web every
// step; once every 32 steps gives ~16ms granularity at the fast rate
// and is fine for the button/abort path.
#define STEP_ABORT_POLL_PERIOD     32

// -------- Open-loop manual jog (bench debug, no encoder) --------
// A blind, bounded motor jog for verifying coils/mechanics when the AS5600
// is dead/absent and closed-loop motion is (correctly) refused. Bounded over
// continuous-spin because there is no feedback to catch a stall against a
// stop. DEFAULT is a few degrees' worth of steps; PER_CMD caps a single
// invocation; PER_SESSION caps cumulative blind travel since boot / last
// successful calibration so a blind motor can't be walked indefinitely into
// a hard stop. At ~5.7 steps/deg (28BYJ bipolar-mod gearbox): 32 steps ~=
// 5.6 deg, 114 ~= 20 deg (< one BUCKET_SLOT_DEG), 1140 ~= 200 deg.
#define JOG_DEFAULT_STEPS            32
#define JOG_MAX_STEPS_PER_CMD       114
#define JOG_MAX_STEPS_PER_SESSION  1140

// -------- AS5600 absolute encoder --------
//
// I2C address 0x36; 12-bit raw angle in registers 0x0E (MSB) and 0x0F
// (LSB), masked to the low 12 bits. Status register 0x0B carries
// magnet-detection bits: MD (bit 5, magnet present), ML (bit 4,
// too weak), MH (bit 3, too strong). The AS5600's own zero-position
// adjustment is left at its POR default (0) so the ANGLE register at
// 0x0E reads the raw mechanical angle; firmware handles the offset.
//
// One diametric magnet sits on the arm rotation axis above the sensor;
// the encoder reading is the live arm angle relative to the AS5600's
// internal zero. g_encoderZeroOffset captures the raw angle observed
// when the arm is parked against the home (CCW) endstop -- i.e. the
// raw angle BUCKET_ENDSTOP_OFFSET_DEG before bucket 1's centre.
// Bucket n's target angle is (g_encoderZeroOffset +
// (BUCKET_ENDSTOP_OFFSET_DEG + (n-1) * BUCKET_SLOT_DEG) * sign) mod 360.
#define AS5600_I2C_ADDR        0x36
#define AS5600_REG_STATUS      0x0B
#define AS5600_REG_ANGLE_H     0x0E
#define AS5600_REG_ANGLE_L     0x0F
#define AS5600_STATUS_MD       0x20   // magnet detected
#define AS5600_STATUS_ML       0x10   // magnet too weak
#define AS5600_STATUS_MH       0x08   // magnet too strong
#define ENC_TICK_PERIOD_MS       20   // ~50 Hz live read
#define ENC_STATUS_PERIOD_MS   1000   // status reread cadence
#define ENC_MAX_CONSEC_FAILS     10   // I2C-fail run before fault

// -------- Watering config --------
#define WATER_INTERVAL_HOURS    12
#define DOSE_MIN_ML             10
#define DOSE_MAX_ML             60
#define DOSE_STEP_ML            10
#define DOSE_DEFAULT_ML         10
#define DEFAULT_INTERVAL_HOURS  24
// Sentinel: a bucket with dose_ml == DOSE_SKIP is visited by the
// rotor (so the edge-counter position model stays consistent) but
// has no valve open, no scale sample, and renders as an empty bar.
// DOSE_MIN_ML stays at 10 -- it's the minimum *non-skip* dose; the
// dose ladder (10..60 in 10mL steps) is unchanged, and validation
// elsewhere accepts DOSE_SKIP as a separate allowed value.
#define DOSE_SKIP                0
#define G_PER_ML                1.0f
#define DOSE_TOLERANCE_G        2.0f
// The load cell is the source of truth for dose volume: the dispense
// loop closes the valve when grams-delivered hits the target. A
// per-bucket safety timeout sized at 2x the expected pour time
// (DOSE_SEC_PER_ML calibrated to your valve's bench flow rate) only
// kicks in if the load cell isn't tracking -- e.g., reservoir off
// the cell, load cell unmounted, or the valve is stuck shut. Bump
// DOSE_SEC_PER_ML if your valve flows slower than ~0.33 mL/s; the
// 2x multiplier already gives generous margin for normal variation.
// Empirical: this rig runs at ~0.4 mL/s, hence the 3.0 s/mL setting.
#define DOSE_SEC_PER_ML          3.0f
#define DOSE_TIMEOUT_MARGIN      2.0f
// If the safety timeout fires having seen less than this much weight
// loss, the load cell never tracked the pour -- flag as "no flow"
// and treat consecutive occurrences as a dry-reservoir abort.
#define DOSE_FLOW_MIN_G          1.0f

// -------- Mid-dispense trustworthiness + credit-and-complete resume --------
// (STEP 2/3 robustness.) What the stall detector exists for: if the cell
// reads LOW mid-pour (interference offloading it), the weight cutoff never
// fires and the valve pours to the full safety timeout. The excess does NOT
// flood/leak -- it overflows into the ADJACENT bucket -- so this is a WASTE /
// reservoir-drain protection (wasted water, mis-dosed neighbor, ~1.6 L drained
// at timeout), NOT a plant-safety guard. Accordingly it is tuned CONSERVATIVE:
// bias hard against false-tripping a slow-but-valid pour (which would wrongly
// skip a real watering); a slightly-late stop only wastes a little water.
//
// Expected-by-now uses the conservative DOSE_SEC_PER_ML rate (~0.33 mL/s),
// which is ~14x slower than this rig's real ~4.6 mL/s. So the trip threshold
// (DISP_STALL_MIN_FRACTION x expected) sits far below ANY real flow and only
// fires on near-zero delivery (no-flow / cell-reads-low). Acceptance bar:
// "never false-trips slow-but-valid" FIRST, "catches the runaway eventually"
// SECOND. The real backstop is the RESERVOIR: stall-trip + the cumulative
// safety-timeout ceiling + one-resume-then-skip bound total valve-open volume
// per bucket to ~dose+margin, so one misbehaving bucket can't empty the tank.
// BENCH-VALIDATED boundary; tune here, do not hardcode at the site.
#define DISP_STALL_GRACE_MS          4000   // ignore flow-stall this long after open (pipe-fill/valve lag)
#define DISP_STALL_MIN_FRACTION      0.25f  // delivered must reach >= k * expected-by-now
#define DISP_STALL_TRIP_MS           3000   // sustained under-fraction for this long => SUSPECT
#define DISP_MOTION_TRIP_MS          1500   // sustained mid-pour motion this long => SUSPECT (debounced
                                            // so normal pour/valve vibration can't false-trip a defer)
// SUSPECT -> auto-recover gate. To resume, the cell (valve CLOSED) must read
// STABLE (spread <= STABLE_G across a SETTLE window) AND CONSISTENT with the
// original baseline minus what we already delivered (|mean - (g0-delivered)|
// <= BASELINE_MAX_DRIFT_G). The consistency half is STEP 2's g0-validity
// guard: a refill / preload shift during the pause moves the mean away from
// (g0 - delivered) and is rejected, so a stale baseline can never drive an
// over-pour -- the bucket is skipped instead. This gate is strictly stronger
// than the under-flow trigger, so a resume never starts on a still-bad signal.
#define DISP_SUSPECT_SETTLE_MS       2000   // cell must hold stable this long
#define DISP_SUSPECT_STABLE_G        2.0f   // max sample spread to call it stable
#define DISP_SUSPECT_MAX_WAIT_MS    15000   // give up waiting for sane -> skip bucket
#define RESUME_BASELINE_MAX_DRIFT_G  5.0f   // |mean - (g0-delivered)| above this => baseline stale => skip
#define DISP_MAX_RESUMES                1   // one resume attempt, then flag+skip (no loop)

// -------- No-flow valve pulse-priming (hydraulic recovery) --------
// Operator-confirmed failure: the valve audibly opens but no water flows
// (line stiction / airlock) until it is stopped and restarted -- then it
// flows. So BEFORE escalating a genuine no-flow to FAULT_NO_FLOW, pulse
// the valve closed/open a few times (~1.5 Hz) to break the stiction and
// prime the line, then keep pouring. This is a retry that sits BEFORE the
// fault, not a separate fault path: if priming establishes flow the
// normal cutoff finishes the dose; if it does not, the existing flow-
// stall / no-flow escalation still latches FAULT_NO_FLOW as before. Pulse
// time counts against the per-bucket cumulative safety timeout, so
// priming can never let a bucket over-pour.
#define NOFLOW_PRIME_MAX_ATTEMPTS       2   // prime bursts per bucket before real fault
#define NOFLOW_PRIME_PULSES             4   // close/open cycles per burst (3-5)
#define NOFLOW_PRIME_CLOSED_MS        330   // valve-closed dwell per pulse (~1.5 Hz)
#define NOFLOW_PRIME_OPEN_MS          330   // valve-open dwell per pulse
#define NOFLOW_CHECK_MS              2000   // no weight drop within this of open => no-flow => prime
#define NOFLOW_MIN_FLOW_G            2.0f   // delivered above this => flow established (stop priming)

// -------- Settle-gated dispense baseline (creep/vibration robustness) --------
// The per-bucket baseline g0 was captured with readScaleAvg() ~25ms after
// gotoBucket() finished moving the arm. On a vibration-sensitive / free-
// hanging cell the arm move leaves the reading CREEPING upward for
// seconds, so g0 was captured low and the pour then read "reverse drift"
// (cell mass rising vs baseline) -> false SUSPECT -> skip -> FAULT_NO_FLOW.
// Instead wait for the cell to actually settle before trusting g0: require
// a stable rolling window (spread <= STABLE_G) whose mean matches the
// previous window (creep flattened), else fall back to an instantaneous
// average after MAX_WAIT so a genuinely noisy cell still dispenses.
// Latency: adds up to ~MAX_WAIT per bucket in the worst case, ~2 windows
// (~1.6s) typically -- an acceptable cost for a trustworthy baseline.
#define DISP_BASELINE_WINDOW_MS       800   // one settle window
#define DISP_BASELINE_STABLE_G       2.0f   // max spread AND max window-to-window mean drift
#define DISP_BASELINE_MAX_WAIT_MS    6000   // give up settling -> instantaneous avg
// Post-disturbance grace for the reverse-drift detector: after each
// valve-open (initial or post-prime pulse) the cell is mechanically
// disturbed and may briefly creep up. Don't evaluate reverse-drift until
// it has had this long to settle, so priming disturbance can't false-trip.
#define DISP_REVERSE_GRACE_MS        1500

// -------- Gesture thresholds (ms) --------
// SHORT     : held < 400 ms; counts toward multi-press tally.
// LONG      : held >= 1000 ms; fires immediately while still held.
//             In non-ERROR states this fires the tare; in ERROR it
//             is suppressed in favor of EXTRA_LONG.
// EXTRA_LONG: held >= 3000 ms; only meaningful in ERROR state,
//             where it clears the fault. The release after this
//             fires nothing (g_longConsumed is set).
// GAP       : quiet window after the last release before the
//             multi-press tally is committed
//             (1->short, 2->double, 3+->triple).
// Held in [400, 1000) ms is a no-op (neither short nor long).
#define GESTURE_SHORT_MAX_MS    400
#define GESTURE_LONG_MS        1000
#define GESTURE_EXTRA_LONG_MS  3000
#define GESTURE_GAP_MS          400
#define TARE_FEEDBACK_MS        500

// Runtime WiFi-credential reset: hold BtnA for >=10 s while idle (READY)
// to wipe stored STA credentials and reboot into the config portal. This
// is the *runtime* counterpart to the boot-time AP_BUTTON_HOLD_MS gesture
// (checkApModeRequest): a sealed unit with stale/wrong stored creds never
// connects and is therefore unreachable over the network, so the only
// prior recovery (serial 'W', or the 5 s hold caught at boot) needed a
// cable or a reboot window. BtnA is on the enclosure, so this lets the
// unit recover with the button alone. A hint overlay with a countdown
// appears WIFI_RESET_HINT_MS into the hold so the gesture is discoverable
// and cancellable; releasing before WIFI_RESET_HOLD_MS aborts with no
// change. Gated to READY (see updateGestures) so it can never fire
// mid-dispense or during motion. Distinct from the [1 s,3 s) thresholds
// above: 10 s is far past them and is consumed continuously while held.
#define WIFI_RESET_HOLD_MS     10000
#define WIFI_RESET_HINT_MS      2000

// -------- UI colors (Pluvio brand palette, RGB565) --------
// Harmonized with the web UI's warm "bone + burnt-orange + muted
// water-blue" identity. Background stays dark for small-display
// contrast; the warmth comes from the bone text and brand accents,
// not the backdrop. RGB565 is 5/6/5 bits, so the warm grays are
// hand-picked to keep R5 > B5 (warm bias) and G6 <= 2*R5 (no green
// cast) -- a naive 8->565 truncation of a warm gray drifts green
// because green carries the spare bit.
//
//   name          web role          sRGB       565       use
//   UI_BLACK      warm near-black   #1a1714    0x18A2    screen fill / text bg
//   UI_WHITE      bone              #f4f1ec    0xF79D    primary text
//   UI_BLUE       water / active    #3d7fb0    0x3BF6    dose segments + resv fill
//   UI_CYAN       water highlight   #6fb0d8    0x6D9B    droplet + demo countdown
//   UI_YELLOW     orange-bright     #df7e09    0xDBE1    selected bucket accent
//   UI_RED        alarm (kept pure) #ff0000    0xF800    fault banner + low level
//   UI_DARKGREY   warm inactive     #5a534a    0x5A89    empty / unlit outlines
//   UI_ORANGE     accent primary    #c46f02    0xC360    brand chrome (available)
//   UI_ORANGE_DK  accent shadow     #af6306    0xAB00    brand chrome (available)
//
// UI_BLUE is the darkened brand water-blue (#3d7fb0, not #4a90c2) so
// the bone reservoir label centered over a full fill stays ~3.5:1
// instead of ~2.8:1; it still reads as brand water on the dark bg.
// If 0x18A2 looks muddy on the panel, use 0x0000 -- text/accents still
// carry the identity.
#define UI_BLACK      0x18A2
#define UI_WHITE      0xF79D
#define UI_GREEN      0x07E0   // unused on LCD; retained for compat
#define UI_DARKGREEN  0x03E0   // unused on LCD; retained for compat
#define UI_YELLOW     0xDBE1   // = brand orange-bright (selection accent)
#define UI_CYAN       0x6D9B   // = lighter water blue (droplet / countdown)
#define UI_RED        0xF800   // KEPT pure red for fault / low-level alarm
#define UI_DARKGREY   0x5A89   // warm mid-gray (empty / inactive)
#define UI_BLUE       0x3BF6   // = brand water blue, darkened for label contrast
#define UI_ORANGE     0xC360   // brand primary accent (available, not yet wired)
#define UI_ORANGE_DK  0xAB00   // brand accent shadow (available, not yet wired)

// -------- Layout (128x128 display) --------
// Bar grid: a single row of vertical segmented bars, one per
// bucket. Bar geometry is computed at render time from
// g_bucketCount so the row scales cleanly to 12, 13, or 14
// buckets without a separate constant set per count. Each bar
// shows the bucket's dose as 1..UI_BAR_SEGMENTS lit blue
// segments mapped to DOSE_MIN_ML..DOSE_MAX_ML in DOSE_STEP_ML
// steps; the selected bucket gets a yellow underline plus its
// 1-indexed number rendered in small text below the underline.
#define UI_BARS_X         2
#define UI_BARS_Y         16
#define UI_BARS_W         124   // 128 - 2px margin each side
#define UI_BARS_H         72    // leaves 8px below for label row
#define UI_BAR_LABEL_Y    90    // y-origin for the selected-bucket
                                // number label, just above the
                                // existing UI_BOTTOM_Y=98 strip
#define UI_BAR_SEGMENTS   6     // 10..60 mL in 10mL steps
#define UI_BAR_SEG_GAP    1     // px gap between segments
#define UI_BAR_GAP_MIN    1     // min px gap between adjacent bars
#define UI_BAR_W_MAX      8     // cap bar width so 12-bucket case
                                // doesn't end up looking chunky
// The lit-segment count is derived directly from doseMl using
// DOSE_MIN_ML / DOSE_STEP_ML, so UI_BAR_SEGMENTS must stay
// consistent with the dose ladder. If DOSE_MIN_ML, DOSE_MAX_ML,
// or DOSE_STEP_ML ever change, bump UI_BAR_SEGMENTS to match.
static_assert(UI_BAR_SEGMENTS
              == ((DOSE_MAX_ML - DOSE_MIN_ML) / DOSE_STEP_ML) + 1,
              "UI_BAR_SEGMENTS must equal the number of discrete "
              "dose levels in [DOSE_MIN_ML, DOSE_MAX_ML]");
#define UI_BOTTOM_Y       98
#define UI_BOTTOM_H       30
#define UI_REDRAW_MIN_MS  250

// -------- Display power management (idle dim / sleep) --------
// Idle the panel down in two stages to save power and avoid burn-in
// when the rig sits untouched (the common case -- it waters on a
// schedule with nobody watching). Times are measured from the most
// recent interaction (button, serial, web request, or an attention-
// worthy state transition); see g_lastInteractionMs / manageDisplayPower.
#define DISPLAY_DIM_AFTER_MS         60000   // ms idle before dimming
#define DISPLAY_SLEEP_AFTER_MS      300000   // ms idle before panel off
#define DISPLAY_BRIGHTNESS_ACTIVE      255   // full brightness
#define DISPLAY_BRIGHTNESS_DIM          40   // dimmed but still legible

// -------- Software bit-bang I2C master (two independent buses) --------
// Why bit-bang: both ESP32-S3 hardware I2C peripherals are already in
// use (I2C_NUM_0 = Wire @ 38/39 for AtomicMotion; I2C_NUM_1 = M5.In_I2C
// @ 45/0 for the IMU). The two remaining sensors get their own pins.
// Earlier attempts to release I2C_NUM_1 after IMU init and re-install
// Arduino's Wire1 left the TwoWire state machine wedged on repeated-START
// reads (single-byte probes acked, multi-byte reads NACKed
// mid-transaction). Hand-rolled bit-bang sidesteps the peripheral and the
// Arduino layer entirely.
//
// Two buses, one device each (the passive U006 hub on Port A is gone):
//   * g_encBus   -- AS5600 encoder (0x36), AtomicMotion Port B (SCL 7 / SDA 8)
//   * g_scaleBus -- U177 scale (0x26), AtomicMotion Port C (SCL 5 / SDA 6)
// The primitives are parameterized on a SwI2cBus context (pin pair +
// per-bus stretch-timeout counters) instead of the old global pins, so
// the same code drives both buses. Timing, addressing and byte order are
// unchanged from the single-bus version -- this is a refactor, not a
// retiming.
//
// Pin handling: OUTPUT_OPEN_DRAIN gives true open-drain; digitalWrite
// HIGH releases the pin (external pull-ups on each port bring it high),
// LOW drives it low. The input buffer stays live in open-drain mode so
// digitalRead during ACK / read / clock-stretch phases works without
// flipping pinMode mid-transaction. SWI2C_HALF_US sets the half-period.
//
// Clock-stretch handling: on every SCL release we poll the SCL line and
// wait until it actually reads high before proceeding (a slave may hold
// SCL low to stretch), bounded by SWI2C_STRETCH_TIMEOUT_US. This is the
// U177 scale fix -- its STM32 stretches between register-write and
// data-read, and a fixed delay (the old behavior) can't cover the
// variable stretch. On timeout we abort cleanly and return the existing
// failure code so the caller's retry/health logic handles it; the bus is
// not left hung. The AS5600 never stretches, so the wait is a no-op cost
// (one digitalRead per clock) on the encoder bus.
// The swi2c bus code logs (one-shot init, rate-limited stretch timeout)
// via LOGI, but the logging layer is defined further down. Forward-
// declare its backing function and the macro here; the later definition
// of LOGI is a token-identical redefinition, which is allowed.
static void logAppend(const char* fmt, ...);
#ifndef LOGI
#define LOGI(...) logAppend(__VA_ARGS__)
#endif

struct SwI2cBus {
    uint8_t     sdaPin;
    uint8_t     sclPin;
    const char* name;             // one-shot init log + stretch log label
    uint32_t    stretchTimeouts;  // lifetime count of stretch-timeout aborts
    uint32_t    lastTimeoutLogMs; // rate-limit gate for the stretch log
};

static SwI2cBus g_encBus   = { ENC_SWI2C_SDA_PIN,   ENC_SWI2C_SCL_PIN,
                               "encoder(AS5600) PortB", 0, 0 };
static SwI2cBus g_scaleBus = { SCALE_SWI2C_SDA_PIN, SCALE_SWI2C_SCL_PIN,
                               "scale(U177) PortC",     0, 0 };

static inline void swi2cSdaHigh(SwI2cBus& b) { digitalWrite(b.sdaPin, HIGH); }
static inline void swi2cSdaLow(SwI2cBus& b)  { digitalWrite(b.sdaPin, LOW);  }
static inline void swi2cSclLow(SwI2cBus& b)  { digitalWrite(b.sclPin, LOW);  }
static inline int  swi2cSdaRead(SwI2cBus& b) { return digitalRead(b.sdaPin); }
static inline void swi2cDelay()              { delayMicroseconds(SWI2C_HALF_US); }

// Release SCL high and wait until the line actually reads high. The
// slave can hold SCL low to clock-stretch; we honor it up to
// SWI2C_STRETCH_TIMEOUT_US, then abort (returns false) so the caller
// stops the transaction. Counts + rate-limited-logs timeouts per bus.
static bool swi2cSclReleaseWait(SwI2cBus& bus) {
    digitalWrite(bus.sclPin, HIGH);          // release; pull-up takes it high
    uint32_t t0 = micros();
    while (digitalRead(bus.sclPin) == LOW) {  // slave stretching?
        if ((uint32_t)(micros() - t0) >= SWI2C_STRETCH_TIMEOUT_US) {
            bus.stretchTimeouts++;
            uint32_t now = millis();
            if (now - bus.lastTimeoutLogMs >= SWI2C_STRETCH_LOG_INTERVAL_MS) {
                bus.lastTimeoutLogMs = now;
                LOGI("swi2c %s: SCL stretch timeout (>%uus); count=%lu\n",
                     bus.name, (unsigned)SWI2C_STRETCH_TIMEOUT_US,
                     (unsigned long)bus.stretchTimeouts);
            }
            return false;
        }
    }
    return true;
}

static void swi2cInit(SwI2cBus& bus) {
    pinMode(bus.sdaPin, OUTPUT_OPEN_DRAIN);
    pinMode(bus.sclPin, OUTPUT_OPEN_DRAIN);
    swi2cSdaHigh(bus);
    digitalWrite(bus.sclPin, HIGH);
    swi2cDelay();
    LOGI("swi2c bus init: %s SDA=GPIO%u SCL=GPIO%u\n",
         bus.name, (unsigned)bus.sdaPin, (unsigned)bus.sclPin);
}

static bool swi2cStart(SwI2cBus& bus) {
    swi2cSdaHigh(bus);
    if (!swi2cSclReleaseWait(bus)) return false;
    swi2cDelay();
    swi2cSdaLow(bus);  swi2cDelay();
    swi2cSclLow(bus);  swi2cDelay();
    return true;
}

// Repeated START: SDA must come back high while SCL is low, then both
// rise, then SDA falls before SCL falls. The slave commonly stretches
// here (after acking the register-pointer write), so the SCL release
// honors the stretch and can abort.
static bool swi2cRepeatedStart(SwI2cBus& bus) {
    swi2cSdaHigh(bus); swi2cDelay();
    if (!swi2cSclReleaseWait(bus)) return false;
    swi2cDelay();
    swi2cSdaLow(bus);  swi2cDelay();
    swi2cSclLow(bus);  swi2cDelay();
    return true;
}

// Best-effort bus release. Uses the bounded stretch wait so a slave that
// is still holding SCL can't hang STOP; the return is ignored because
// STOP's job is to free the bus regardless.
static void swi2cStop(SwI2cBus& bus) {
    swi2cSdaLow(bus);  swi2cDelay();
    swi2cSclReleaseWait(bus);
    swi2cDelay();
    swi2cSdaHigh(bus); swi2cDelay();
}

// MSB-first. Returns true on ACK (slave pulled SDA low). Returns false
// on a clock-stretch timeout too, which the caller treats like a NACK.
static bool swi2cWriteByte(SwI2cBus& bus, uint8_t b) {
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) swi2cSdaHigh(bus); else swi2cSdaLow(bus);
        swi2cDelay();
        if (!swi2cSclReleaseWait(bus)) return false;
        swi2cDelay();
        swi2cSclLow(bus);  swi2cDelay();
        b <<= 1;
    }
    swi2cSdaHigh(bus);              // release SDA so slave can drive ACK
    swi2cDelay();
    if (!swi2cSclReleaseWait(bus)) return false;
    swi2cDelay();
    int ack = swi2cSdaRead(bus);
    swi2cSclLow(bus);  swi2cDelay();
    return ack == 0;
}

// nack=true on the final byte of a multi-byte read so the slave releases
// the bus and the master can STOP. Returns false on a stretch timeout;
// the read byte is written to *out only on success.
static bool swi2cReadByte(SwI2cBus& bus, bool nack, uint8_t* out) {
    uint8_t b = 0;
    swi2cSdaHigh(bus);              // release SDA so slave drives data
    for (int i = 0; i < 8; i++) {
        b <<= 1;
        if (!swi2cSclReleaseWait(bus)) return false;
        swi2cDelay();
        if (swi2cSdaRead(bus)) b |= 1;
        swi2cSclLow(bus);  swi2cDelay();
    }
    if (nack) swi2cSdaHigh(bus); else swi2cSdaLow(bus);
    swi2cDelay();
    if (!swi2cSclReleaseWait(bus)) return false;
    swi2cDelay();
    swi2cSclLow(bus);  swi2cDelay();
    swi2cSdaHigh(bus);
    *out = b;
    return true;
}

// Combined register read: START, addr|W, reg, repeated-START, addr|R,
// read n bytes, STOP. NACK on the final byte. Returns false at the
// first un-acked address/register byte or any clock-stretch timeout
// (slave absent, stuck bus, or over-long stretch).
static bool swi2cReadRegister(SwI2cBus& bus, uint8_t addr, uint8_t reg,
                              uint8_t* buf, size_t n) {
    if (!swi2cStart(bus))                                { swi2cStop(bus); return false; }
    if (!swi2cWriteByte(bus, (uint8_t)(addr << 1) | 0)) { swi2cStop(bus); return false; }
    if (!swi2cWriteByte(bus, reg))                       { swi2cStop(bus); return false; }
    if (!swi2cRepeatedStart(bus))                        { swi2cStop(bus); return false; }
    if (!swi2cWriteByte(bus, (uint8_t)(addr << 1) | 1)) { swi2cStop(bus); return false; }
    for (size_t i = 0; i < n; i++) {
        if (!swi2cReadByte(bus, i == n - 1, &buf[i]))    { swi2cStop(bus); return false; }
    }
    swi2cStop(bus);
    return true;
}

static bool swi2cWriteRegister(SwI2cBus& bus, uint8_t addr, uint8_t reg,
                               const uint8_t* buf, size_t n) {
    if (!swi2cStart(bus))                                { swi2cStop(bus); return false; }
    if (!swi2cWriteByte(bus, (uint8_t)(addr << 1) | 0)) { swi2cStop(bus); return false; }
    if (!swi2cWriteByte(bus, reg))                       { swi2cStop(bus); return false; }
    for (size_t i = 0; i < n; i++) {
        if (!swi2cWriteByte(bus, buf[i])) { swi2cStop(bus); return false; }
    }
    swi2cStop(bus);
    return true;
}

// 7-bit address probe: write the address byte only, look for ACK.
static bool swi2cProbe(SwI2cBus& bus, uint8_t addr) {
    if (!swi2cStart(bus)) { swi2cStop(bus); return false; }
    bool ack = swi2cWriteByte(bus, (uint8_t)(addr << 1) | 0);
    swi2cStop(bus);
    return ack;
}

// -------- U177 Mini Scales driver --------
// Drop-in replacement for the M5Unit-Miniscale UNIT_SCALES class --
// same method names and signatures so all callsites are unchanged --
// but talks over the bit-bang bus instead of TwoWire. The U177's
// register map matches the upstream library; we only implement the
// subset used elsewhere in this file. Register addresses are from
// the M5Stack UNIT_SCALES.h header. The on-board STM32 owns HX711
// timing, calibration and tare; gap value (cal data reg) and offset
// reset are persisted on the unit's own MCU.
class UNIT_SCALES {
public:
    bool begin(uint8_t addr = 0x26) {
        _addr = addr;
        // Two probe attempts: first transaction after a cold POR
        // sometimes NACKs while the Port C pull-ups settle.
        if (swi2cProbe(*_bus, _addr)) return true;
        delay(5);
        return swi2cProbe(*_bus, _addr);
    }
    int32_t getRawADC() {
        uint8_t d[4] = {0};
        if (!swi2cReadRegister(*_bus, _addr, 0x00, d, 4)) return 0;
        return (int32_t)((uint32_t)d[0]
                       | ((uint32_t)d[1] << 8)
                       | ((uint32_t)d[2] << 16)
                       | ((uint32_t)d[3] << 24));
    }
    // Overload that distinguishes I2C failure from a legitimate zero
    // reading: returns false on NACK, leaving `out` untouched. Runtime
    // weight reads use this so a transient bus failure propagates as
    // NAN instead of being silently treated as raw=0 (which the
    // host-side weight formula would mis-interpret as a real reading).
    bool getRawADC(int32_t& out) {
        uint8_t d[4] = {0};
        if (!swi2cReadRegister(*_bus, _addr, 0x00, d, 4)) return false;
        out = (int32_t)((uint32_t)d[0]
                      | ((uint32_t)d[1] << 8)
                      | ((uint32_t)d[2] << 16)
                      | ((uint32_t)d[3] << 24));
        return true;
    }
    float getWeight() {
        uint8_t d[4] = {0};
        if (!swi2cReadRegister(*_bus, _addr, 0x10, d, 4)) return NAN;
        float f; memcpy(&f, d, 4); return f;
    }
    float getGapValue() {
        uint8_t d[4] = {0};
        if (!swi2cReadRegister(*_bus, _addr, 0x40, d, 4)) return NAN;
        float f; memcpy(&f, d, 4); return f;
    }
    void setGapValue(float v) {
        uint8_t d[4];
        memcpy(d, &v, 4);
        swi2cWriteRegister(*_bus, _addr, 0x40, d, 4);
        delay(100);              // matches upstream lib's settle delay
    }
    void setOffset() {
        uint8_t v = 1;
        swi2cWriteRegister(*_bus, _addr, 0x50, &v, 1);
    }
    // Reads reg 0xFE; returns true only if the I2C transaction
    // completed. Side-effect-free and calibration-independent, so it
    // doubles as the "is the STM32 actually serving?" probe used by
    // tryScaleInit(). `out` is left untouched on failure so a caller
    // can't mistake a silent NACK for a real fw=0.
    bool getFirmwareVersion(uint8_t& out) {
        uint8_t v = 0;
        if (!swi2cReadRegister(*_bus, _addr, 0xFE, &v, 1)) return false;
        out = v;
        return true;
    }
private:
    uint8_t   _addr = 0x26;
    SwI2cBus* _bus  = &g_scaleBus;   // U177 lives on Port C
};

M5AtomicMotion motion;
UNIT_SCALES    scale;
Preferences    prefs;
static bool    g_prefsOK = false;
// Persisted calibration scale factor (counts-per-gram equivalent for
// the U177's on-board STM32). The U177 also stores it internally on
// power-up, but we persist it here too so an NVS restore can re-push
// the user's calibration after a firmware-up flash or a U177 reset.
// Set by tryScaleInit() from getGapValue() on first successful begin().
static float   g_gapValue = 0.0f;

// Raw HX711 counts captured at the moment of the most recent tare.
// The U177's reg 0x00 returns the absolute (non-offset-subtracted)
// raw value, so a cal of the form `gap = raw / known` is dominated
// by the tare offset and is also sign-ambiguous when the cell wires
// such that load decreases counts. We instead compute the cal as
// `gap = (raw_now - g_rawTare) / known`: the delta is exactly the
// counts contributed by the known mass, and its sign falls out of
// the wiring (positive for "load adds counts", negative for
// inverted) without a hand-rolled negate anywhere. Persisted so a
// reboot between tare and cal doesn't invalidate the sequence.
static int32_t g_rawTare      = 0;
static bool    g_rawTareValid = false;
// Session-only freshness flag: set true by captureScaleZero() on
// every in-session zero capture, never persisted. The cal handlers gate on
// THIS rather than g_rawTareValid so a tare snapshot from a prior
// session can't be paired with the U177's current internal offset
// (captured by setOffset() this boot). Without the gate, divergent
// baselines produce wrong-sign / wrong-magnitude deltas and weight
// reads negative even though the user thinks they tared. A reboot
// drops this back to false; operator must re-tare before re-calibrating.
static bool    g_rawTareSession = false;
// True when a full calibration (zero + gap) has been committed to NVS
// under key "scaleCalOk". This is the single source of truth for the
// UI: false -> Settings shows the calibrate wizard and status reads
// "needs calibration" (never a silent zero/NAN weight); true -> the
// calibrate control is locked and the panel shows status + live weight
// only. A factory reset clears the flag, the zero and the gap together.
static bool    g_scaleCalibrated = false;

// Calibration ownership lock. Set when the operator captures the empty-
// cell zero (step 1); held until the gap commit (step 2) completes, a
// factory reset, or a watchdog timeout. While held, the background scale
// health-probe / auto-re-detect (scaleHealthTick) is suspended so it can
// neither flip scaleOK nor re-adopt the U177's stale on-device gap out
// from under the two-step wizard -- the race that made step 2 silently
// no-op. The captured zero itself (g_rawTare / g_rawTareSession) is
// cleared ONLY by a fresh capture (start over), a completed calibrate, or
// a factory reset -- never by the timeout, which only resumes the probe.
static bool     g_scaleCalActive  = false;
static uint32_t g_scaleCalStartMs = 0;
// Operator-walked-away watchdog: release scale ownership (resume the
// health probe) this long after a zero capture with no calibrate commit.
// Sized for the real bench flow: capture zero, then prepare/weigh a known
// mass (e.g. 2440 g of water) off-scale and type it in -- easily minutes.
// While ownership is held the health probe is suspended, which is what
// keeps a marginal fresh-unit bus glitch from flipping scaleOK=false and
// silently disabling the Calibrate button mid-wizard. So this must cover
// the whole realistic wizard duration, not just a quick tap-tap.
#define SCALE_CAL_TIMEOUT_MS  1800000UL   // 30 minutes
// Minimum raw-count delta between empty zero and loaded cell for a
// calibrate to be accepted. Below this the mass clearly isn't on the
// cell (or a flaky read mirrored the zero); computing a gap from it
// would yield ~0/garbage. Real cal masses move the 24-bit ADC by 1e5+
// counts, so 1000 is a wide safety floor (< ~10 g at a typical cell).
#define SCALE_CAL_MIN_DELTA   1000L

bool scaleOK = false;

// -------- Stepper state --------
// Diagnostic step counter -- incremented by stepN() with the signed
// step delta. NOT a position-truth source (the AS5600 is). Resets on
// each calibrateEndstops() run and rolls naturally; only used by
// printState / api/state for "how many steps did motion take" debug
// readouts.
static int32_t  g_stepCounter    = 0;
// Diagnostics: count of gotoBucket arrivals that landed OUTSIDE
// ARRIVAL_TOL_DEG -- i.e. accepted via a wide zero-crossing rather than a
// tight tolerance land. The absolute encoder is re-read every hop, so these
// do NOT accumulate into position drift and are deliberately NOT a recal /
// fault trigger; this is pure telemetry the production test and field watch
// can trend to flag a mechanically marginal rig. Exposed in printState and
// /api/state.
static uint32_t g_largeFinalErrCount   = 0;
static float    g_lastLargeFinalErrDeg = 0.0f;
// Cumulative open-loop blind-jog steps since boot or the last successful
// calibration (bounds total blind travel, JOG_MAX_STEPS_PER_SESSION). Reset
// to 0 by calibrateEndstops() on success; 0 at boot.
static uint32_t g_jogSessionSteps      = 0;
// Phase index 0..3 mapping to the four-phase drive table. Persists
// across motion calls so a release-then-resume re-energizes the same
// phase the rotor was last left on.
static uint8_t  g_phaseIndex     = 0;
// True while coils are energized (motion in progress or held).
// Cleared by stepperReleaseCoils(); set by stepperWake() and the
// inner step loop. Diagnostic only -- the coil release is the
// authoritative action.
static bool     g_coilsEnergized = false;

// -------- AS5600 encoder state --------
// g_armPositionDeg is refreshed by encoderTick() at ~50 Hz and also
// re-read inline by gotoBucket between step bursts. Range [0, 360).
// g_encoderHealthy reflects the last AS5600 status read AND tracks
// I2C-failure runs (set false after ENC_MAX_CONSEC_FAILS in a row).
// g_encoderConsecFails is the running fail counter; reset on success.
static volatile float    g_armPositionDeg     = 0.0f;
static volatile bool     g_encoderHealthy     = false;
static volatile uint8_t  g_encoderConsecFails = 0;
static uint8_t           g_encoderStatus      = 0;     // last STATUS read
static uint32_t          g_lastEncoderTickMs  = 0;
static uint32_t          g_lastEncoderStatusMs = 0;
// -------- Endstop calibration (persisted) --------
// g_encoderZeroOffset is the raw AS5600 angle (degrees, 0..360) when
// the arm is parked against the home (CCW) endstop -- i.e. centred
// over bucket 1. g_bucketAngleStepPositive is true if driving the
// stepper in direction +1 increases the encoder reading; false if it
// decreases (depends on the magnet orientation above the AS5600 and
// on g_motorInverted). g_endstopsCalibrated is set true after a
// successful calibrateEndstops() run; false on fresh NVS means
// "must calibrate before any gotoBucket". All three persist under
// NVS keys "encZero", "encDirPos", "encCalOk".
static float    g_encoderZeroOffset       = 0.0f;
static bool     g_bucketAngleStepPositive = true;
static bool     g_endstopsCalibrated      = false;

// -------- Per-cycle calibration cadence (RAM-only) --------
// runWateringCycle() recalibrates at most once per CALIBRATE_EVERY_N_CYCLES
// cycles instead of every cycle. g_cyclesSinceCalibration counts cycles in
// the current calibration epoch (the calibrating cycle is #1 of its epoch);
// when it reaches the threshold the next cycle recalibrates. Initialized to
// the threshold so the FIRST cycle after boot always calibrates (even when a
// trusted calibration was loaded from NVS), which re-establishes a known home
// against the mechanical stop. RAM-only by design: a reboot should re-home
// once. g_forceRecalNextCycle is a one-shot override set when a skipped
// (non-calibrating) cycle misses a gotoBucket target -- a possible silent
// position loss (e.g. motor skip under load) -- so the next cycle recalibrates
// regardless of the counter, bounding the loss to a single cycle.
static int      g_cyclesSinceCalibration  = CALIBRATE_EVERY_N_CYCLES;
static bool     g_forceRecalNextCycle     = false;

// Tunable step rate. Persisted to NVS under "stepDelayFast". The
// legacy "stepDelaySlow" / "stepRampSteps" keys (slow/ramp profile)
// are no longer read or written; their stale values in NVS are
// harmless.
static uint16_t g_stepDelayFastUs = STEP_DELAY_FAST_US_DEFAULT;

// -------- Valve state --------
// Servo angles for valve open / closed. The external module driving
// the solenoid maps servo angle to H-bridge polarity, so these are
// effectively "open" and "closed" tags rather than physical angles.
// Defaults work for the bench module; tune via 'V O' / 'V C' or
// /api/setvalveangles. Persisted to NVS under "valveOpenAng" and
// "valveCloseAng".
static uint8_t g_valveOpenAng  = 80;
static uint8_t g_valveCloseAng = 135;

// Post-valve-close dwell in seconds. After the solenoid closes at
// the end of a bucket's dispense, the firmware waits this long
// before advancing the rotor to the next bucket. This compensates
// for water that's already in flight inside the supply tubing
// after the valve shuts: without the pause, that residual volume
// lands AFTER the rotor has moved off the bucket and ends up
// dumped onto whatever the rotor crossed next, causing visible
// overflow between buckets and skewing the per-bucket dose. The
// load cell continues to track during the wait, so the post-cycle
// reservoir-loss sanity check still reflects the true total. 5
// seconds is conservative for a ~30cm tubing run at the typical
// flow rate; tune via 'V V' or web UI (range 1..10). Persisted to
// NVS under "dwellSec" -- existing NVS values override this default.
static uint8_t g_postValveDwellSec = 5;
#define POST_VALVE_DWELL_MIN_SEC  1
#define POST_VALVE_DWELL_MAX_SEC  10

// Motor direction polarity. When false (default), phase-advance
// 0->1->2->3 drives the rotor in the "forward" direction (toward
// higher bucket numbers). When true, the coil wiring or magnet
// polarity is reversed and the phase advance is flipped in software
// (0->3->2->1) for the same physical motion. Persisted to NVS
// under "motorInv". Range: bool.
static bool g_motorInverted = false;

// -------- WiFi / web UI globals --------
// HTTP is sync (single-threaded against the WebServer object); flags
// below queue work for loop() to dispatch in the READY branch so the
// HTTP handler returns quickly and motion never runs in a TCP context.
static WebServer webServer(80);
static bool      g_wifiUp        = false;
static bool      g_apMode        = false;   // true if running in config AP, not STA

// Per-device network identity, derived once at boot from the ESP32
// factory MAC by initDeviceIdentity(). These make two Pluvio units on
// the same LAN distinguishable instead of all colliding on "pluvio"/
// "Pluvio-Setup". g_deviceSuffix is 6 lowercase hex digits (the
// NIC-specific half of the MAC); g_deviceName is the DHCP/mDNS
// hostname (e.g. "pluvio-bccca0") and g_apSsid the config-portal SSID
// (e.g. "Pluvio-Setup-bccca0"). Not persisted to NVS -- recomputing
// from the chip every boot is free and stays correct if the board is
// swapped. Empty until initDeviceIdentity() runs (early in setup()).
static String    g_deviceSuffix;
static String    g_deviceName;
static String    g_apSsid;
// User-settable friendly hostname, persisted to NVS ("friendlyName").
// Empty = unset, in which case the network identity falls back to the
// immutable g_deviceName. When set (and valid, see isValidFriendlyName)
// it overrides the DHCP/mDNS hostname only -- NEVER the AP SSID, the
// logs, or the immutable "device" mDNS TXT record. Loaded by
// persistApplyGlobals(); read through activeHostname().
static String    g_friendlyName;
static uint32_t  g_lastWifiCheck = 0;
static bool      g_otaInProgress = false;
// Set by the /update upload handler's START step when an OTA must be
// refused before any flash write; read (and cleared) by the /update
// request handler to choose the rejection status. g_otaRejectBusy -> 409
// (motion in progress); g_otaRejectAuth -> 401 (missing/wrong token).
static bool      g_otaRejectBusy = false;
static bool      g_otaRejectAuth = false;
static volatile bool g_webRequestWater = false;
// When true, the next runWateringCycle() bypasses per-bucket due
// checks and dispenses every non-skip bucket. Cleared by
// runWateringCycle on completion. Set by /api/water (Water Now).
static volatile bool g_webForceAll     = false;
// Layer 2 hook: water exactly one bucket on demand (Home Assistant
// / on-off automation). g_webRequestSingleN is 1-indexed; the
// runWateringCycle schedule check only permits this bucket. Both
// flags are cleared at end-of-cycle alongside g_webForceAll.
static volatile bool g_webRequestSingle  = false;
static volatile int  g_webRequestSingleN = 0;
static volatile bool g_webRequestHome  = false;
// Diagnostic-test request flags. Mirror the water/home pattern: HTTP
// handlers set them, the READY branch in loop() consumes one per pass.
// Keeps long-running motion off the WebServer request thread.
static volatile bool g_webRequestDtest      = false;
// Endstop calibration trigger (replaces the old steps-per-bucket /
// magnet-mapping flag).
static volatile bool g_webRequestCalibrate  = false;
// Continuous-rotate test trigger. dir: +1 forward, -1 reverse, 0
// idle. Consumed in the READY/ERROR foreground-dispatch branches.
static volatile bool g_webRequestRotate     = false;
static volatile int  g_webRotateDir         = 0;

// Open-loop manual JOG trigger (debug escape hatch). Unlike rotate, the jog
// is dispatched even when canRunMotorOps() is false (e.g. a dead encoder
// latched FAULT_ENCODER_LOST) -- it is the one motor op allowed without
// position trust. Consumed in BOTH foreground branches. dir: +1/-1.
static volatile bool g_webRequestJog        = false;
static volatile int  g_webJogDir            = 0;
static volatile int  g_webJogSteps          = 0;

// Test-mode flags for runWateringCycle. When g_testModeActive, the
// cycle skips all load-cell logic (reservoir guards, dose targeting,
// no-flow tracking, post-cycle sanity check) and emulates each
// bucket's dose with a time-based valve-open of (dose_ml /
// DOSE_STEP_ML) * g_postValveDwellSec seconds. g_testModeWet
// controls whether the valve actuates: true = water flows,
// false = timing-only emulation. Both default false on every boot
// and are cleared by cycleEndCleanup() so the next runWateringCycle
// call is a real cycle unless the caller re-sets them.
static bool g_testModeActive = false;
static bool g_testModeWet    = false;

// -------- Demo mode --------
// Continuous-loop demonstration that runs the existing test-mode
// dispense path (runDispenseTest -> runWateringCycle with
// g_testModeActive) back to back so an observer can watch the rig
// work. It is a thin driver on top of the test infrastructure, NOT a
// new dispense path: each iteration just sets g_testModeWet, calls
// runDispenseTest(), bumps the cycle counter, and pauses.
//
//   Dry  -> g_testModeWet = false: valve never actuates, doses are
//           emulated via the post-valve dwell timing.
//   Wet  -> g_testModeWet = true:  valve actuates; load-cell guards
//           are skipped (same as the 'D 1' dispense test). Operator
//           running a wet demo owns the reservoir.
//
// IMU level + motion safety stays fully active (inherited from
// runWateringCycle's pre-flight and mid-cycle watchdog) -- dry mode
// only means the pump doesn't run, it does NOT bypass safety. A
// mid-cycle fault or any user abort exits demo mode entirely; demo
// never auto-restarts after a fault.
//
// All demo state is RAM-only and resets on boot: the rig always
// powers up in normal scheduled mode. The pause is runtime-tunable
// (1..10s) but does not persist. The cycle counter resets to 0 on
// every demo entry.
enum class DemoMode { Off, Dry, Wet, Random };
static DemoMode g_demoMode       = DemoMode::Off;
// Armed-but-not-yet-running mode. The trigger (web /api/demo/start, serial
// X/N) sets ONLY this + g_webRequestDemoStart; it must NOT touch g_demoMode.
// runDemoLoop() promotes g_demoPendingMode into g_demoMode on real entry and
// clears both on exit. Consequence: g_demoMode is non-Off -- and so
// demo.active in /api/state is true -- if and ONLY if the blocking demo loop
// is actually executing. A start intent that never runs (e.g. faults first)
// can no longer strand demo.active true (see
// docs/demo-state-investigation.md, Finding 1).
static DemoMode g_demoPendingMode = DemoMode::Off;
static uint32_t g_demoCycleCount = 0;
static uint8_t  g_demoPauseSec   = 2;     // inter-cycle pause, RAM-only
#define DEMO_PAUSE_MIN_SEC  1
#define DEMO_PAUSE_MAX_SEC  10
// Foreground-dispatch flags, mirroring the existing g_webRequest*
// pattern. Start is consumed by loop()'s READY branch (so the
// blocking demo loop runs on the foreground task, not the WebServer
// thread). Stop is consumed by the running demo loop between cycles;
// it is paired with g_abortRequested so it can also break an
// in-flight dispense.
static volatile bool g_webRequestDemoStart = false;
static volatile bool g_webRequestDemoStop  = false;
// Set true by the demo loop during the inter-cycle pause so the LCD
// watering view can show a "Next in Ns" countdown instead of a stale
// fill bar. g_demoPauseRemainSec is the live countdown value.
static volatile bool    g_demoInterCyclePause = false;
static volatile uint8_t g_demoPauseRemainSec  = 0;

// Demo Random bucket selection. Random is now orthogonal to wet/dry,
// mirroring how Dry and Wet modes work: g_demoRandomWet carries the
// caller's deterministic wet/dry choice (web &wet=, serial 'N') for the
// whole session, while the BUCKETS are re-rolled every cycle. Each Random
// cycle runDemoLoop() picks 1..3 distinct non-skip buckets and records
// them in g_demoRandomMask (bit i set => water bucket i), then sets
// g_demoRandomActive so runWateringCycle()'s service-list builder
// restricts the cycle to exactly that subset -- the real cycle path, no
// parallel motion. cycleEndCleanup() clears the active flag + mask each
// cycle; g_demoRandomWet persists across cycles (reset on demo start).
static bool     g_demoRandomActive = false;
static uint16_t g_demoRandomMask   = 0;     // BUCKET_COUNT(13) fits uint16_t
static bool     g_demoRandomWet    = false;

static const char* demoModeName(DemoMode m) {
    switch (m) {
        case DemoMode::Off: return "off";
        case DemoMode::Dry: return "dry";
        case DemoMode::Wet: return "wet";
        case DemoMode::Random: return "random";
    }
    return "?";
}

// -------- Log ring buffer for diagnostic web streaming --------
// Lines pushed by LOG() are mirrored to Serial AND stored here; web
// clients poll /api/log?since=N for incremental fetch. Sized to hold
// a full X-test summary (~150 lines) without wraparound. Foreground
// task only -- do not call from ISR (vsnprintf is not ISR-safe).
#define LOG_BUF_LINES    256
#define LOG_BUF_LINE_MAX 160

struct LogLine {
    uint32_t seq;
    uint32_t ms;
    uint32_t epochSec;   // wall-clock epoch at write time, 0 if no NTP yet
    char     text[LOG_BUF_LINE_MAX];
};
static LogLine  g_logBuf[LOG_BUF_LINES];
static uint32_t g_logHead    = 0;   // next write index
static uint32_t g_logSeqNext = 1;   // monotonic seq assigned on write

// Forward decl for the NTP-acquired flag defined further down the
// file; logAppend needs it to decide between wall-clock and uptime
// prefixes on the USB serial mirror. The corresponding definition
// drops the `static` qualifier so the linkage matches.
bool g_timeIsWallClock = false;

static void logAppend(const char* fmt, ...) {
    char buf[LOG_BUF_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    // USB-serial mirror gets a per-line timestamp prefix so the
    // raw stream is readable without the web UI's renderer. Two
    // formats: HH:MM:SS in NTP mode, right-aligned uptime ms
    // otherwise. The ring-buffer slot stays raw -- /api/log emits
    // both ms and epochSec separately so the JS can render its
    // own prefix.
    char prefix[16];
    uint32_t epochSec = 0;
    if (g_timeIsWallClock) {
        time_t now = time(nullptr);
        struct tm t;
        localtime_r(&now, &t);
        snprintf(prefix, sizeof(prefix), "[%02d:%02d:%02d] ",
                 t.tm_hour, t.tm_min, t.tm_sec);
        epochSec = (uint32_t)now;
    } else {
        snprintf(prefix, sizeof(prefix), "[%8lums] ",
                 (unsigned long)millis());
    }
    Serial.print(prefix);
    Serial.print(buf);
    LogLine& slot = g_logBuf[g_logHead % LOG_BUF_LINES];
    slot.seq      = g_logSeqNext;
    slot.ms       = millis();
    slot.epochSec = epochSec;
    // Truncation is intentional if the formatted line was longer
    // than LOG_BUF_LINE_MAX-1; vsnprintf already null-terminated.
    strncpy(slot.text, buf, sizeof(slot.text) - 1);
    slot.text[sizeof(slot.text) - 1] = '\0';
    g_logHead++;
    g_logSeqNext++;
}

// Debug-only log: writes to USB serial ONLY, never to the ring
// buffer. Use for per-edge / per-tick motion trace that's essential
// when debugging hardware but would otherwise drown the web log.
// Foreground task only; vsnprintf is not ISR-safe.
static void logDebug(const char* fmt, ...) {
    char buf[LOG_BUF_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    Serial.print(buf);
}

#define LOGI(...) logAppend(__VA_ARGS__)
#define LOGD(...) logDebug(__VA_ARGS__)
// Backward-compat alias: existing LOG() callsites mirror to both
// USB and ring buffer (same as LOGI). New code should pick LOGI
// (user-relevant) or LOGD (verbose trace) explicitly.
#define LOG(...)  LOGI(__VA_ARGS__)

// -------- Status LED --------
// Single WS2812 on STATUS_LED_PIN, driven via FastLED. The LED is a
// pure status mirror: updateLed() runs once per loop tick, calls
// resolveLedState() to map the current global state to a color +
// animation pattern, and writes the strip. NOTHING else in the
// firmware touches the LED -- no setLed() calls sprinkled into motion
// / fault / cycle paths. resolveLedState() reads globals; the rest of
// the code is unaware the LED exists.
//
// Brightness is the master scale (0..255). 0 is a hard kill switch
// (LED stays dark regardless of state). Persisted to NVS under
// "ledBri"; default 80 is visible without being harsh in a dark room.
static uint8_t g_ledBrightness = 80;   // 0..255, 0 disables LED entirely
#define LED_BRIGHTNESS_MIN  0
#define LED_BRIGHTNESS_MAX  255

// 24-bit 0xRRGGBB status colors. Chosen independently of the web/brand
// RGB565 UI_* palette: the WS2812 wants 24-bit and FastLED scales per
// channel, and these values are tuned for vivid LED rendering rather
// than to match the on-screen colors. Kept separate from the UI_*
// constants on purpose.
//
// Pure / saturated intent: each status color drives one (or two)
// channels hard and leaves the rest near zero, so the hue reads
// vividly on the WS2812 instead of washing toward white as
// brightness drops. GREEN is a true green with only a hint of blue
// (zero red) so there's no yellow/olive cast; AMBER and ORANGE keep
// red at full with a measured green and no blue, reading as clean
// orange rather than olive; WATERBLUE and CYAN are actual blue /
// cyan with no green-warming. RED is fully pure for alarm
// legibility. The two grays are deliberately muted and neutral --
// they're "absent / inactive" cues (no WiFi, skip-dose) and must
// not compete with the saturated rungs. Keep new tweaks in this
// "one dominant channel" family rather than blending channels.
#define LED_COL_RED        0xFF0000   // latched fault
#define LED_COL_ORANGE     0xFF5500   // brand accent, saturated
#define LED_COL_WATERBLUE  0x0080FF   // headless / interaction feedback
#define LED_COL_CYAN       0x00C0FF   // water highlight / AP mode
#define LED_COL_GREEN      0x00FF20   // healthy / watering, pure green
#define LED_COL_GRAYDIM    0x404040   // no WiFi (neutral, low)
#define LED_COL_GRAYWARM   0x806040   // headless skip-dose
#define LED_COL_WHITE      0xFFFFFF   // (available, not currently used)
#define LED_COL_AMBER      0xFF6000   // warning states
#define LED_COL_GREEN_DIM  0x004008   // muted green for booting

enum LedPattern { LED_OFF, LED_SOLID, LED_BREATHE_SLOW,
                  LED_PULSE_SLOW, LED_PULSE_FAST, LED_BLINK_SLOW,
                  LED_ALTERNATE_TWO,
                  LED_DISPENSE_PROGRESS,   // green breathe whose brightness
                                           // ceiling scales with dose progress
                                           // (g_activeBucketDeliveredMl /
                                           //  g_activeBucketTargetMl)
                  LED_PARKED_AT_BUCKET,    // headless: steady blue scaled by parked dose (gray for skip)
                  LED_DOSE_PULSE_SEQUENCE, // headless: N blue pulses (one gray for skip) after advance/adjust
                  LED_UNKNOWN_POSITION };  // headless: brief red flash when pressed while position untrusted
struct LedState { uint32_t color1; uint32_t color2; LedPattern pattern; };

// LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
// FastLED strip backing store (single pixel) + the millis() gate for
// the ~20 Hz updateLed() tick. RAM-only; no persistence.
// static CRGB    g_leds[1];
static uint32_t g_lastLedUpdateMs = 0;
#define LED_UPDATE_PERIOD_MS  50   // ~20 Hz

// -------- Web-arm LED mirror cache --------
// The status LED rides on the physical rotary arm; the radial web UI
// draws a matching on-screen arm and mirrors the LED's colour and
// brightness so the screen is a live mirror of the device. updateLed()
// (the firmware's sole LED writer) snapshots the resolved LedState and
// the instantaneous pattern envelope (0..255, before the master
// brightness scale) here on every tick; apiStateJson() reads them to
// build the JSON "led" block. Both run on the loop() task, so the plain
// (non-atomic) reads/writes are safe -- no ISR or second core touches
// these.
static LedState g_ledSnapshot = { 0, 0, LED_OFF };
static uint8_t  g_ledEnvelope = 0;

// -------- Display power state --------
// RAM-only idle dim / sleep state for the front panel; every boot
// starts ACTIVE (nothing persists). g_lastInteractionMs is the
// millis() of the most recent interaction -- a BtnA edge, a serial
// byte, a web request, or a state transition into an attention-worthy
// mode (watering, fault, demo, any abortable motion). manageDisplayPower()
// in loop() drives the ACTIVE -> DIM -> ASLEEP progression off it, and
// markInteraction() resets it (waking the panel if needed).
//
// g_displayJustWoke marks the single BtnA edge that woke a sleeping
// panel so updateGestures() can swallow that press rather than acting
// on it (a wake tap shouldn't also cycle the bucket selection).
enum DisplayPowerState { DISPLAY_ACTIVE, DISPLAY_DIM, DISPLAY_ASLEEP };
static DisplayPowerState g_displayState      = DISPLAY_ACTIVE;
static uint32_t          g_lastInteractionMs = 0;
static bool              g_displayJustWoke   = false;
// Defined down in the UI section (it calls renderUI); forward-declared
// here because AbortableMotionGuard below calls it on motion entry.
// Returns true ONLY when it actually woke a sleeping panel.
static bool markInteraction();

// -------- Abort handling --------
// Any button press during a watering cycle, or 'x'/'X' over serial,
// requests an abort. All blocking motion primitives
// (calibrateEndstops, gotoBucket, rotateContinuous) and the dispense
// loop poll this via serviceAbort() and bail early. runWateringCycle
// clears the flag before its recovery park so the rotor still returns
// to bucket 1.
static volatile bool g_abortRequested = false;

// Set true while the foreground task is in a long-running motion
// path that the user should be able to abort with BtnA. Without
// this flag, serviceAbort() only honors the button when
// g_state == WATERING, so a button press during 'D', 'X', 'h', 'r',
// or 'T' is silently ignored. The 'x' / 'X' serial path is always
// honored regardless. Set/cleared via AbortableMotionGuard at the
// entry/exit of each abort-eligible top-level handler.
static volatile bool g_abortableMotion = false;

// When true, serviceAbort() drops any pending abort and ignores new
// abort sources for the duration. Used exclusively by the post-abort
// recovery park to bucket 1: once the user has cancelled the cycle,
// the park is part of the cancel sequence and must not be re-cancelled
// (a second button press during the park would leave the rotor
// stranded mid-travel, which is the exact symptom from the log:
// "Abort: parking at bucket 1" / "gotoBucket(1): from steps=1145 ->
// 0 (delta=-1145)" with NO completion line and the rotor not at
// bucket 1). gotoBucket's own bounded retries still terminate the
// park if the rotor is mechanically stuck.
static volatile bool g_suppressAbort = false;

// -------- Headless interaction (arm-as-UI) --------
// The status LED rides on the rotary arm, so the arm physically
// pointing at a bucket IS the selection feedback. A short press
// advances the arm to the next bucket and parks there; a double press
// adjusts the parked bucket's dose. This state is entirely RAM-only:
// every boot starts with no session (arm at home, LED healthy-idle),
// and the user opens a session by pressing BtnA. It is independent of
// g_selected, which still drives ONLY the web UI's bucket-card
// highlight via /api/select.
//
//   g_interactionParkedBucket  1-indexed bucket the arm is parked at,
//                              0 when no session is active.
//   g_interactionTargetBucket  bucket the queue is advancing toward
//                              (== parked when idle between hops).
//   g_interactionAdvancing     true while a short-press advance is
//                              mid-motion; loop()'s READY branch drives
//                              the actual gotoBucket so the gesture
//                              handler never blocks. A press DURING an
//                              advance bumps the target (queue) instead
//                              of aborting the hop -- see serviceAbort().
//   g_interactionLastPressMs   millis() of the most recent interaction
//                              press; drives the 30s idle auto-park.
static volatile int      g_interactionParkedBucket = 0;
static volatile int      g_interactionTargetBucket = 0;
static volatile uint32_t g_interactionLastPressMs  = 0;
static volatile bool     g_interactionAdvancing    = false;
#define INTERACTION_IDLE_MS  30000   // park back to bucket 1 after this idle

// LED feedback for the interaction model, consumed by resolveLedState/
// updateLed.
//   g_ledDosePulseCount   0 = skip (one warm-gray pulse); 1..6 = that
//                         many blue pulses (= dose / DOSE_STEP_ML).
//   g_ledDoseSequenceMs   millis() the current pulse sequence began;
//                         0 = no sequence.
//   g_ledUnknownPositionFlashMs  millis() of the last "pressed while
//                         position untrusted" red flash; 0 = none.
static volatile uint8_t  g_ledDosePulseCount         = 0;
static volatile uint32_t g_ledDoseSequenceMs         = 0;
static volatile uint32_t g_ledUnknownPositionFlashMs = 0;

// Forward decl: defined alongside the gesture state below.
// serviceAbort() calls this when a BtnA press triggers an abort,
// so the press that caused the abort isn't reinterpreted as a
// long-press tare or stale short-press once loop() resumes calling
// updateGestures(). Mirrors the existing call inside
// runWateringCycle() for symmetry across abort paths.
static void resetGestureAfterCycle();

// RAII pair for g_abortableMotion. Place at the top of any
// long-running motion handler so the flag is cleared on every exit
// path -- including early returns from inner failures. The flag is
// independent of g_state: 'D' / 'X' / 'h' / 'r' / 'T' all run while
// g_state == READY, so we can't piggyback on the WATERING state for
// abort-eligibility there.
struct AbortableMotionGuard {
    // Entry into any long-running motion handler is an interaction:
    // wake the panel (if dimmed/asleep) so the operator can watch
    // progress. Exit bumps the timer so the panel doesn't dim the
    // instant a multi-minute operation finishes.
    AbortableMotionGuard()  { g_abortableMotion = true; markInteraction(); }
    ~AbortableMotionGuard() { g_abortableMotion = false;
                              g_lastInteractionMs = millis(); }
};

// -------- IMU / level / motion --------
// Tilt and vibration safety. The IMU is the onboard MPU6886 (AtomS3)
// or BMI270 (AtomS3R); M5Unified auto-detects and normalizes the
// AtomS3R's X/Y axis swap, so getAccel() returns the same coordinate
// frame on both boards. Mount orientation handles the OTHER axis
// question: which board axis points UP when the rig is installed
// level. Default MOUNT_Y_UP matches this rig's install (the board's
// Y axis points up when level); flip via /api/imu/mount, serial 'O',
// or the web Tuning panel if the build is different. Raw axes are exposed in
// /api/state during bring-up so the mount can be verified visually.
//
// Gravity is the absolute reference: no zero-offset calibration. The
// install is expected to be level; threshold widens the acceptance
// band for users who knowingly install slightly off.
//
// The IMU lives on M5Unified's internal I2C bus (m5::In_I2C), not the
// user-managed Wire bus that talks to AtomicMotion. No contention.
enum MountOrientation {
    MOUNT_Z_UP = 0,
    MOUNT_Z_DOWN,
    MOUNT_Y_UP,
    MOUNT_Y_DOWN,
    MOUNT_X_UP,
    MOUNT_X_DOWN,
};
static const char* mountName(MountOrientation m) {
    switch (m) {
        case MOUNT_Z_UP:   return "Z_UP";
        case MOUNT_Z_DOWN: return "Z_DOWN";
        case MOUNT_Y_UP:   return "Y_UP";
        case MOUNT_Y_DOWN: return "Y_DOWN";
        case MOUNT_X_UP:   return "X_UP";
        case MOUNT_X_DOWN: return "X_DOWN";
    }
    return "?";
}
static bool parseMountName(const String& s, MountOrientation* out) {
    if      (s == "Z_UP")   *out = MOUNT_Z_UP;
    else if (s == "Z_DOWN") *out = MOUNT_Z_DOWN;
    else if (s == "Y_UP")   *out = MOUNT_Y_UP;
    else if (s == "Y_DOWN") *out = MOUNT_Y_DOWN;
    else if (s == "X_UP")   *out = MOUNT_X_UP;
    else if (s == "X_DOWN") *out = MOUNT_X_DOWN;
    else return false;
    return true;
}

#define IMU_SAMPLE_PERIOD_MS    20      // ~50 Hz
#define IMU_GRAVITY_ALPHA       0.05f   // EMA for tilt
#define IMU_MOTION_WINDOW       50      // ~1s @ 50Hz
// Consecutive getAccel() failures tolerated before the motion metric
// is forced to 0. ~200ms at the 20ms sample period: long enough that a
// normal single-sample I2C hiccup is ignored, short enough that a
// wedged IMU can't latch a stale "moving" metric (and thus a stale
// motion deferral) for more than a fraction of a second.
#define IMU_MAX_CONSEC_FAILS    10
#define SPILL_THRESH_DEFAULT    10.0f
#define SPILL_THRESH_MIN        1.0f
#define SPILL_THRESH_MAX        30.0f
#define MOTION_THRESH_DEFAULT   0.15f
#define MOTION_THRESH_MIN       0.02f
#define MOTION_THRESH_MAX       1.0f
// Hysteresis: once the motion alert is logged, require the metric to
// drop below this fraction of the threshold before logging the
// "returned within threshold" line. Stops a marginal noise floor
// from spamming on/off log lines around the threshold edge.
#define MOTION_CLEAR_FRACTION   0.5f

static bool   g_imuPresent     = false;
static int    g_imuType        = 0;      // m5::imu_t, cast to int
static float  g_axRaw = 0, g_ayRaw = 0, g_azRaw = 0;   // bring-up debug
static float  g_axFilt = 0, g_ayFilt = 0, g_azFilt = 0;
static bool   g_imuFiltSeeded  = false;
static float  g_pitchDeg       = 0;
static float  g_rollDeg        = 0;
static float  g_motionMetric   = 0;
static bool   g_motionAlertActive = false;
static float  g_motionRing[IMU_MOTION_WINDOW];
static int    g_motionRingFill = 0;
static int    g_motionRingHead = 0;
static uint32_t g_lastImuSampleMs = 0;
// Consecutive getAccel() failure tracking (see imuTick). Reset on any
// successful read; g_imuStaleWarned is a one-shot so the "IMU wedged"
// notice logs once per failure run, not every tick.
static int    g_imuFailCount   = 0;
static bool   g_imuStaleWarned = false;

static float  g_spillThreshDeg = SPILL_THRESH_DEFAULT;
static float  g_motionThreshG  = MOTION_THRESH_DEFAULT;
static MountOrientation g_mountOrientation = MOUNT_Y_UP;

// -------- System state --------
enum SystemState { BOOTING, READY, WATERING, ERROR };
static SystemState g_state = BOOTING;

// -------- Fault state --------
// When runWateringCycle hits a terminal failure (two consecutive
// no-flow dispenses, dry reservoir, or an AS5600 encoder that drops
// its magnet / fails I2C), it stops the rest of the cycle, parks the
// rotor at bucket 1, transitions to ERROR, and records what happened
// so renderUI() can show a red banner and 'p' / 'A' can report and
// clear the fault. ERROR persists until acknowledged via 'A' (serial)
// or an extra-long button hold (>= 3s); the WATERING scheduler in
// loop() is already gated on g_state == READY, so a fault correctly
// suspends scheduled cycles.
enum FaultCode {
    FAULT_NONE,
    FAULT_NO_FLOW,
    FAULT_RESERVOIR_EMPTY,
    // AS5600 read returned an I2C error or status bits report the
    // magnet is missing / too weak / too strong for ENC_MAX_CONSEC_FAILS
    // reads in a row. Position is unknown; motion is unsafe until the
    // operator acknowledges and (presumably) reseats the magnet.
    FAULT_ENCODER_LOST,
    // The encoder is healthy but the rotor will not traverse: two or
    // more buckets came back unreachable in one cycle, or the post-
    // calibration park could not reach bucket 1. The bench signature is
    // a motor that develops torque one direction only (miswired coil
    // pair / weak or open phase / marginal Vbus). Position is known but
    // motion is unsafe until the operator fixes the drive and acks.
    FAULT_ROTOR_STALL,
};
static FaultCode g_lastFault   = FAULT_NONE;
static int       g_faultBucket = -1;     // 1-indexed; -1 if N/A
static uint32_t  g_faultMs     = 0;      // millis() at fault

static const char* faultName(FaultCode f) {
    switch (f) {
        case FAULT_NONE:               return "NONE";
        case FAULT_NO_FLOW:            return "NO_FLOW";
        case FAULT_RESERVOIR_EMPTY:    return "RESERVOIR_EMPTY";
        case FAULT_ENCODER_LOST:       return "ENCODER_LOST";
        case FAULT_ROTOR_STALL:        return "ROTOR_STALL";
    }
    return "?";
}

// Short label for the on-screen banner. Display is 128px wide and
// the banner overlays the bottom strip, so trim the longer codes
// to stay inside the cell.
static const char* faultLabel(FaultCode f) {
    switch (f) {
        case FAULT_NONE:               return "OK";
        case FAULT_NO_FLOW:            return "NO FLOW";
        case FAULT_RESERVOIR_EMPTY:    return "RESV EMPTY";
        case FAULT_ENCODER_LOST:       return "ENC LOST";
        case FAULT_ROTOR_STALL:        return "STALLED";
    }
    return "FAULT";
}

static bool isMotionFault(FaultCode f) {
    // FAULT_ENCODER_LOST blocks motor ops: without position truth we
    // can't trust gotoBucket. Re-acknowledging (clearFault) re-enables
    // motor ops; the next gotoBucket will re-fault if the encoder is
    // still bad. FAULT_ROTOR_STALL blocks them the same way: the drive
    // can't move the rotor reliably, so every gotoBucket would burn its
    // iteration budget pinned against a stop until the wiring/Vbus fault
    // is fixed.
    return f == FAULT_ENCODER_LOST || f == FAULT_ROTOR_STALL;
}

// -------- Cycle deferral state --------
// Water-safety conditions (tilt, motion, low reservoir) no longer
// latch into ERROR. Instead runWateringCycle() DEFERS: it returns
// cleanly to READY without stamping the due bucket(s), so the
// scheduler retries on its next tick once the condition clears. No
// user acknowledgement is needed -- level the rig or refill the tank
// and the next tick waters. resolveLedState() reflects the current
// reason as an amber warning (amber breathe for tilt, amber pulse for
// low reservoir) and /api/state exposes it for the web UI.
enum DeferReason {
    DEFER_NONE,
    DEFER_NOT_LEVEL,
    DEFER_MOVING,
    DEFER_RESERVOIR_LOW,
};
static DeferReason g_lastDeferReason     = DEFER_NONE;
static uint32_t    g_deferStartedMs      = 0;
static uint32_t    g_lastDeferReminderMs = 0;

#define DEFER_REMINDER_INTERVAL_MS  (5UL * 60UL * 1000UL)

static const char* deferReasonName(DeferReason r) {
    switch (r) {
        case DEFER_NONE:           return "none";
        case DEFER_NOT_LEVEL:      return "not_level";
        case DEFER_MOVING:         return "moving";
        case DEFER_RESERVOIR_LOW:  return "reservoir_low";
    }
    return "?";
}

// Log a deferral with built-in deduplication. The scheduler keeps
// calling runWateringCycle() every tick, so without dedup a persistent
// condition would spam the log. First deferral of a reason (or the
// first after a different reason / a successful cycle) logs once; the
// same reason thereafter is silent until DEFER_REMINDER_INTERVAL_MS
// has elapsed, then emits a single "still deferred" reminder.
static void noteDeferral(DeferReason r, const char* msg) {
    uint32_t now = millis();
    if (g_lastDeferReason != r) {
        // New reason, or first deferral after a successful cycle.
        LOGI("Cycle deferred (%s): %s\n", deferReasonName(r), msg);
        g_lastDeferReason     = r;
        g_deferStartedMs      = now;
        g_lastDeferReminderMs = now;
    } else if (now - g_lastDeferReminderMs >= DEFER_REMINDER_INTERVAL_MS) {
        uint32_t mins = (now - g_deferStartedMs) / 60000UL;
        LOGI("Cycle still deferred (%s) for %u min\n",
             deferReasonName(r), (unsigned)mins);
        g_lastDeferReminderMs = now;
    }
}

// Called from the successful exit of runWateringCycle() so the next
// deferral logs fresh and the LED / UI drop back to a healthy state.
static void clearDeferralState() {
    g_lastDeferReason     = DEFER_NONE;
    g_deferStartedMs      = 0;
    g_lastDeferReminderMs = 0;
}

// True when motor-only operations (home, calibration, all diagnostic
// tests) are safe to run. READY is always OK. ERROR is OK when the
// fault is non-motion-blocking. Motion faults block until the user
// explicitly acknowledges.
static bool canRunMotorOps() {
    if (g_state == READY) return true;
    if (g_state == ERROR && !isMotionFault(g_lastFault)) return true;
    return false;
}

// True when the device is fully idle: READY, no abortable motion in flight,
// and no motion request already queued for loop()'s foreground dispatch. This
// is stricter than canRunMotorOps() (which only asks "is a motor op safe",
// not "is the foreground actually free"). It is the arm-gate for demo start:
// without the abortable-motion / pending-request checks a demo could be armed
// mid-calibrate / mid-rotate / mid-jog and then leak past a fault into a
// stranded demo.active / surprise-demo-on-clear (investigation Finding 1).
static bool deviceIsIdle() {
    return g_state == READY
        && !g_abortableMotion
        && !g_webRequestWater     && !g_webRequestSingle
        && !g_webRequestHome      && !g_webRequestDtest
        && !g_webRequestCalibrate && !g_webRequestRotate
        && !g_webRequestJog       && !g_webRequestDemoStart
        && !g_interactionAdvancing;
}

// Stepper gotoBucket() handles its own one-shot retry internally
// (hall-sanity slip -> re-home -> retry once -> FAULT_STEPS_SLIPPED).
// runWateringCycle no longer wraps the call in a per-bucket retry
// loop; if gotoBucket returns false, the cycle aborts.

// Live watering state, populated by runWateringCycle and exposed
// via /api/state. Cleared back to defaults when cycle ends or
// aborts. Web UI uses these to render the focused-bucket view.
static volatile int      g_activeBucket            = 0;   // 1-indexed; 0 = idle
static volatile float    g_activeBucketDeliveredMl = 0.0f;
static volatile uint16_t g_activeBucketTargetMl    = 0;
static volatile int      g_cycleBucketIdx          = 0;   // 1-indexed of N
static volatile int      g_cycleBucketTotal        = 0;

static void clearActiveBucketState() {
    g_activeBucket            = 0;
    g_activeBucketDeliveredMl = 0.0f;
    g_activeBucketTargetMl    = 0;
    g_cycleBucketIdx          = 0;
    g_cycleBucketTotal        = 0;
}

struct Bucket {
    uint16_t dose_ml;
    uint16_t interval_hours;   // 12..504 hours; how often this bucket waters
};
static Bucket g_buckets[BUCKET_COUNT];
// Kept as a non-const int for compatibility with the existing UI /
// scheduling code that iterates `i < g_bucketCount`. The carousel is
// a fixed 13-magnet ring -- the value never changes at runtime.
static int    g_bucketCount = BUCKET_COUNT;
// Time source. Cascades from "wall clock via NTP" to "uptime
// since boot" depending on what's available. Layer 0 always works;
// Layer 1 (NTP) gives reboot-survivable scheduling.
// g_timeIsWallClock is defined near the logging block above (logAppend
// needs to read it); duplicate definition would be a linker error.
static uint32_t g_bootEpochSec    = 0;       // epoch sec at boot, if NTP

// Debug fast-schedule mode. When true, scheduler interprets
// interval_hours as SECONDS (24h "daily" -> 24-second cadence).
// Used to validate scheduling logic without waiting real-world
// intervals. RAM-only -- defaults to false on every boot, so a
// forgotten toggle can't make the rig over-water. Toggle via
// serial 'Z' or POST /api/debug/fastsched.
static bool g_debugFastSchedule = false;

// User-controllable scheduler pause. When false, anyBucketDue() returns
// false unconditionally and the auto-scheduler in loop() never fires
// cycles. Manual operations (web Water Now, Home, all diagnostic tests,
// and serial 'w' command) bypass this flag and continue to work.
// Persisted to NVS under "schedOn" -- survives reboots so a user who
// paused for vacation doesn't get a surprise cycle after a power blip.
// On transition from disabled to enabled, all non-skip buckets are
// stamped with scheduleTimeNow() so each bucket restarts its interval
// clock fresh rather than firing immediately on whatever accumulated
// during the pause.
static bool g_schedulerEnabled = true;

// Per-bucket last-watered timestamp. Semantics:
//   - If g_timeIsWallClock: epoch seconds (real time)
//   - Else: seconds-since-boot (millis()/1000, capped to uint32_t)
// Persisted to NVS only when in wall-clock mode (uptime values
// are meaningless across reboots).
// At boot in uptime mode (no NTP), every bucket is treated as
// "just watered" relative to boot to prevent a cycle storm after
// reboot. In NTP mode, persisted epoch values are restored.
static uint32_t g_lastWateredAt[BUCKET_COUNT] = {0};

static const char* TIMEZONE_TZ = "PST8PDT,M3.2.0,M11.1.0";
static const char* NTP_SERVER  = "pool.ntp.org";
#define NTP_TIMEOUT_MS  10000
static int    g_selected    = 0;
// Last-known bucket index (1-indexed). Echoed for the UI and
// printState; recomputed by gotoBucket() on success, and by
// bucketFromAngle() against the live encoder angle. -1 on motion
// failure / uninitialised. NOT a position-truth source -- the AS5600
// is. This is just a UI label cache.
static int    g_currentBucket = -1;

static uint32_t g_lastWaterMs    = 0;
static uint32_t g_nextWaterMs    = 0;
static float    g_reservoirMl    = 0;
// Reservoir-full mark: the tank's full volume, used ONLY as the UI gauge's
// full-scale (LCD tank fill + web dial). It does NOT gate dispensing -- the
// reservoir-low logic compares the live weight against the sum of bucket
// doses, not this value -- so changing it is display-only and dispense-safe.
// It is hardware-dependent (the reference bench tank is 2500 mL) and the tank
// can change, so it is configurable + persisted: default RESERVOIR_FULL_ML_
// DEFAULT, overridden by the NVS key "resvFullMl" (band-checked on load),
// settable via serial 'F' and POST /api/setresvfull. (The OLD "resvFull" key
// from the removed "Set Reservoir Full" feature is a different key and is
// still wiped by persistInit's migration.)
#define RESERVOIR_FULL_ML_DEFAULT   2500.0f
#define RESERVOIR_FULL_ML_MIN        100.0f
#define RESERVOIR_FULL_ML_MAX      20000.0f
static float g_reservoirFullMl = RESERVOIR_FULL_ML_DEFAULT;

// -------- Forward decls --------
static bool calibrateEndstops();
static bool gotoBucket(int idx);
static bool encoderReadAngle(float* outDeg);
static bool encoderReadStatus(uint8_t* outStatus);
static bool encoderInit();
static void encoderTick();
static float bucketTargetAngleDeg(int n);
static int   bucketFromAngle(float deg);
static float angleErrorDeg(float target, float current);
static float arcPositionFromHome(float angleDeg);
static float arcErrorDeg(float targetAng, float currentAng);
static void stepperReleaseCoils();
static void stepperWake();
static bool stepN(int32_t signed_steps);
static void setStepperPhase(uint8_t phase);
static void valveOpen();
static void valveClose();
static bool runWateringCycle();
static bool anyBucketDue();
static uint32_t secUntilNextDue();
static uint32_t scheduleTimeNow();
static bool tryAcquireNtpTime();
static void persistApplyLastWatered();
static void stampAllLastWatered();
static void persistSaveSchedEnabled();
static void persistSaveEndstopCal();
static void cycleEndCleanup(bool stampWatered);
static void renderUI(bool force = false);
static void renderWateringFocused();
static void renderDroplet(int cx, int cy, int h, bool low);
static void say(const String& line);
static void clearFault(const char* why);
static void enterFault(FaultCode code, int bucket1Indexed);
// Returns runWateringCycle()'s result: true on a clean cycle, false
// on abort / fault / home-fail. The demo loop uses the return to
// decide whether to keep looping.
static bool runDispenseTest();
static void runDemoLoop();
static bool serviceAbort(bool pumpWeb = true);
// Defined down in the status-LED section; forward-declared here so
// serviceAbort() can tick it. Blocking motion paths (watering, demo,
// calibration, rotate) all poll serviceAbort at high frequency, so
// this is what lets the LED animate mid-op even though loop() (the
// other updateLed caller) is stalled. updateLed's own millis() gate
// throttles the high call rate down to its ~20Hz.
static void updateLed();
// Same rationale as updateLed above: serviceAbort() ticks the OTA
// mark-valid check so a freshly OTA'd image still certifies during a long
// watering/demo cycle (which blocks loop()'s top-level call). Defined down
// by setup(); cheap after the first mark (single bool test).
static void markOtaValidIfHealthy();

// Headless interaction model (definitions live in the gesture
// section). Forward-declared here because serviceAbort() queues an
// advance via interactionAdvance(), and enterFault() / runWateringCycle()
// stomp any active session via interactionEnd() -- all defined later.
static void interactionAdvance();
static void interactionAdjustDose();
static void interactionTick();
static void interactionEnd();

// ---------- helpers ----------
static void say(const String& line) {
    Serial.println(line);
    AtomS3.Display.fillScreen(UI_BLACK);
    AtomS3.Display.setCursor(2, 2);
    AtomS3.Display.setTextSize(1);
    AtomS3.Display.setTextColor(UI_WHITE, UI_BLACK);
    AtomS3.Display.println(line);
}

// INA226 helper kept for diagnostics (printState / UI). NEVER call
// from inside the step loop -- each I2C read adds ~1ms and craters
// the step rate.
static float busVoltage() { return motion.ina226.readBusVoltage(); }

// Pump M5 button + serial input during long-running motion / dispense.
// Sets g_abortRequested on rising-edge button press while WATERING or
// while g_abortableMotion is true (i.e. inside an abort-eligible
// foreground motion handler), or when 'x'/'X' arrives on the serial
// stream. Returns the abort state. Other serial bytes are left in
// the buffer for the main handleSerial.
//
// pumpWeb: when true (default), also calls webServer.handleClient()
// so POST /api/abort can land mid-motion. When false (passed by
// stepN's inner loop), the web pump is skipped because each
// handleClient() can take 1-50ms and that's incompatible with the
// stepper's 500us inter-step rate. With pumpWeb=false the web abort
// path takes effect only between step bursts (still well under a
// second of latency).
//
// On a button-driven abort we also reset the gesture machine: the
// press that caused the abort already had wasPressed consumed here,
// but BtnA may still be physically held by the time the handler
// returns. Without the reset, updateGestures() running on the next
// loop tick would compare a stale g_pressStartMs against the still-
// held button and spuriously fire a long-press tare.
static bool serviceAbort(bool pumpWeb) {
    // Tick the status LED so it animates during blocking motion ops.
    // loop() can't reach updateLed() while a cycle / test runs, but
    // every motion primitive polls serviceAbort frequently; updateLed's
    // internal millis() gate throttles this to its ~20Hz cadence.
    updateLed();
    // Same reason: certify a freshly OTA'd image even if it spends the
    // whole 60 s health window inside a watering/demo cycle. Without this,
    // loop()'s top-level call never runs during the cycle and OTA verify
    // times out. No-op (one bool test) after the first successful mark.
    markOtaValidIfHealthy();
    AtomS3.update();
    if (pumpWeb && g_wifiUp) webServer.handleClient();
    // Suppress mode: the post-abort park is uninterruptible. Drop every
    // input that arrived this iteration -- the button edge that
    // AtomS3.update() just latched, any /api/abort flag, and queued
    // serial 'x' bytes -- and exit without observing it.
    if (g_suppressAbort) {
        g_abortRequested = false;
        // Drain the button edges AtomS3.update() latched above. update()
        // only records the press/release into the M5Unified one-shots; it
        // does NOT consume them. If we leave them set, the very next
        // updateGestures() call after suppress ends reads them as fresh
        // input and fires a spurious gesture (advance, tare, or a phantom
        // headless hop) -- the abort-then-bucket-jumped symptom. Reading
        // the one-shots here clears the latch, mirroring the serial 'x'
        // drain below.
        (void)M5.BtnA.wasPressed();
        (void)M5.BtnA.wasReleased();
        // Drain any serial 'x' bytes so they don't fire as soon as
        // suppress mode ends; whitespace passes through normally so
        // handleSerial still gets a clean stream when motion ends.
        while (Serial.available()) {
            int c = Serial.peek();
            if (c == 'x' || c == 'X') Serial.read();
            else                      break;
        }
        return false;
    }
    if ((g_state == WATERING || g_abortableMotion)
        && M5.BtnA.wasPressed()) {
        if (g_interactionAdvancing) {
            // Headless interaction advance in progress: a press QUEUES
            // one more bucket instead of aborting the hop.
            // interactionAdvance() bumps the target and returns early
            // (we're already advancing); the current hop completes and
            // loop() chains another gotoBucket to the new target.
            interactionAdvance();
        } else {
            g_abortRequested = true;
            Serial.println("ABORT (button press)");
            resetGestureAfterCycle();
        }
    }
    while (Serial.available()) {
        int c = Serial.peek();
        if (c == 'x' || c == 'X') {
            Serial.read();
            g_abortRequested = true;
            Serial.println("ABORT (serial 'x')");
        } else if (c == '\r' || c == '\n' || c == ' ') {
            Serial.read();
        } else {
            break;
        }
    }
    return g_abortRequested;
}

// -------- Time abstraction --------
// scheduleTimeNow() is the single source of "now" for the scheduling
// logic, in seconds. In NTP mode it returns epoch seconds (real time);
// otherwise it returns seconds-since-boot. Use this EVERYWHERE in
// scheduling code (instead of millis()) so wall-clock and uptime modes
// share the same arithmetic. Subtraction (now - g_lastWateredAt[i])
// stays correct under uint32 wrap as long as both operands live in
// the same domain, which they always do.
static uint32_t scheduleTimeNow() {
    if (g_timeIsWallClock) {
        time_t now;
        time(&now);
        return (uint32_t)now;
    }
    return (uint32_t)(millis() / 1000UL);
}

// Convert interval_hours to seconds, honoring debug fast mode.
// ALL scheduling math goes through this so the two modes never
// diverge by accident.
static inline uint32_t intervalToSec(uint16_t hours) {
    return g_debugFastSchedule ? (uint32_t)hours
                               : (uint32_t)hours * 3600UL;
}

// One-shot NTP acquire at boot. Blocks up to NTP_TIMEOUT_MS for the
// SNTP daemon to populate the system clock; on success, flips
// g_timeIsWallClock so scheduleTimeNow() switches to epoch seconds
// and the last-watered flush (persistFlushAll) is allowed to commit.
// On failure the caller stays in uptime mode -- we never retry, since
// a flapping NTP would silently shift bucket due-times mid-day.
static bool tryAcquireNtpTime() {
    if (WiFi.status() != WL_CONNECTED) return false;
    configTzTime(TIMEZONE_TZ, NTP_SERVER);
    uint32_t deadline = millis() + NTP_TIMEOUT_MS;
    struct tm tinfo;
    while (millis() < deadline) {
        if (getLocalTime(&tinfo, 100)) {
            time_t now;
            time(&now);
            if (now > 1700000000) {   // sanity: post-2023
                g_timeIsWallClock = true;
                g_bootEpochSec    = (uint32_t)now;
                LOG("NTP OK: %04d-%02d-%02d %02d:%02d:%02d %s\n",
                    tinfo.tm_year + 1900, tinfo.tm_mon + 1, tinfo.tm_mday,
                    tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec, TIMEZONE_TZ);
                return true;
            }
        }
        delay(200);
    }
    LOG("%s\n", "NTP failed; using uptime-based scheduling");
    return false;
}

// -------- Persistence --------
// User-configured state -- U177 scale gap value, captured reservoir-
// full mark, per-bucket doses, encoder zero offset + direction sign,
// valve angles -- is stored in NVS under the "irrig" namespace so it
// survives reboots. The bucket count is fixed (BUCKET_COUNT = 13) and
// not persisted. Tare offset lives on the U177's own MCU (the M5
// Mini Scales library setOffset() takes no args -- it captures the
// current reading on the device), so NVS does not store it.
static void persistInit() {
    g_prefsOK = prefs.begin("irrig", false);
    if (!g_prefsOK) LOGI("%s\n", "Preferences NVS open failed");
    if (!g_prefsOK) return;
    // One-shot migration: wipe NVS keys from the pre-AS5600 / pre-U177
    // firmware. These keys are unreachable in the current build (no code
    // reads or writes them), but leaving them resident wastes NVS space
    // and confuses any out-of-band inspection tool. prefs.remove() is a
    // no-op when the key is absent, so this is safe to run on every
    // boot -- a flash from a fresh rig hits the no-op path immediately.
    static const char* kStaleKeys[] = {
        "hxCounts",       // HX711 counts-per-gram (replaced by gapValue)
        "tareOff",        // HX711 tare offset (U177 stores tare on-device)
        "stepsPerBucket", // pre-encoder step calibration
        "magPos",         // 13-magnet position blob
        "magPosValid",    // magPos validity flag
        "resvFull",       // reservoir-full mark (now a compile-time const)
    };
    for (size_t i = 0; i < sizeof(kStaleKeys) / sizeof(kStaleKeys[0]); i++) {
        prefs.remove(kStaleKeys[i]);
    }
}
// Capture the empty-cell zero for calibration -- RAM ONLY, no NVS
// write. This is step 1 of the atomic calibrate wizard: it snapshots
// reg 0x00 (raw HX711 counts) with the cell empty into g_rawTare and
// arms g_rawTareSession. The zero is NOT persisted here; it is committed
// to NVS together with the gap in persistCommitScaleCal() at the end of
// calibrate, so the two can never desync. A reboot between this capture
// and the calibrate commit simply reloads the previous persisted pair
// (or stays uncalibrated) -- no half-written state. Called right after
// scale.setOffset() from every calibration zero-capture path.
// Returns false (and stores nothing) if the empty-cell read NACKs, so a
// flaky bus during handling can never seed a silent g_rawTare = 0 that
// later makes the calibrate delta collapse to ~0. On success it also
// takes scale ownership (g_scaleCalActive) for the duration of the
// wizard, suspending the background health probe.
static bool captureScaleZero() {
    // NOT gated on scaleOK: calibration works on raw counts (reg 0x00),
    // which are valid whether or not the converted-weight health probe
    // currently trusts the bus -- and calibration is precisely what
    // recovers an unhealthy scale. The checked getRawADC below is the
    // real gate: a genuine I2C failure returns false (reported to the
    // caller) and never stores a silent 0. Callers restore scaleOK on
    // this proven-live read (see markScaleLiveFromCal at the callsites).
    int32_t z = 0;
    if (!scale.getRawADC(z)) return false;   // I2C failure -> report, don't store 0
    g_rawTare         = z;
    g_rawTareValid    = true;   // RAM validity; NVS commit at calibrate
    g_rawTareSession  = true;
    g_scaleCalActive  = true;   // own the scale until calibrate / reset / timeout
    g_scaleCalStartMs = millis();
    return true;
}
// Atomic calibration commit: persist the zero (tare) and gap together
// in one block so they can never desync across a reboot. Called only
// from the calibrate path, after a fresh in-session zero capture and a
// freshly computed gap. The negative-gap (inverted wiring) case is
// preserved -- gap is written verbatim, never abs()'d.
static void persistCommitScaleCal() {
    if (!g_prefsOK) return;
    prefs.putInt("tareRaw",   g_rawTare);
    prefs.putUChar("tareRawV", 1);
    prefs.putFloat("gapValue", g_gapValue);
    prefs.putUChar("scaleCalOk", 1);
    g_scaleCalibrated = true;
}
// Factory reset of scale calibration: clear the calibrated flag, the
// zero and the gap, in RAM and NVS, returning the scale to the
// uncalibrated state (UI re-exposes the calibrate wizard). Leaves the
// U177's own on-device gap untouched -- it is irrelevant to the
// host-side weight path, which now reads NAN until recalibrated.
static void persistResetScaleCal() {
    g_scaleCalibrated = false;
    g_rawTareValid    = false;
    g_rawTareSession  = false;
    g_scaleCalActive  = false;   // wizard reset -> release scale ownership
    g_rawTare         = 0;
    g_gapValue        = 0.0f;
    if (!g_prefsOK) return;
    prefs.putUChar("scaleCalOk", 0);
    prefs.putUChar("tareRawV", 0);
    prefs.putInt("tareRaw", 0);
    prefs.putFloat("gapValue", 0.0f);
}
// Per-bucket dose / interval / last-watered are the three highest-
// frequency NVS writes on this rig: a user mashing the dose buttons,
// a per-plant schedule edit pass, and the 13 last-watered stamps
// fired back-to-back every cycle all hammer the same handful of keys.
// Writing each inline both wears the flash (the AtomS3's sectors are
// good for ~100k erases; wear-leveling stretches that but doesn't
// make it free) and costs ~5-30ms of blocking latency per write --
// visible on web button clicks and stacked 13-deep inside
// runWateringCycle(). So these go through a debounced dirty-bit layer
// instead of the old persistSave*() inline writes.
//
// Dirty-bit tracking for debounced NVS writes. Each persistMark*()
// call sets a flag and a timestamp; persistFlushDue() commits any
// flag whose dirty-age exceeds PERSIST_DEBOUNCE_MS. persistFlushAll()
// commits everything immediately (used on cycle end / fault entry /
// OTA start / reboot so a power loss right after never strands
// changes in RAM).
//
// Per-bucket arrays (dose, interval, last-watered) use a bitmask per
// array; one bit per bucket, 16 bits is plenty for BUCKET_COUNT=13.
// dirtySinceMs is the millis() of the FIRST mark in the current dirty
// run; flush clears it back to 0.
//
// Why 3 seconds: long enough to coalesce a button-mashing user
// (every double-press fires within ~400ms, and a user setting a
// per-plant schedule clicks every 1-2s), short enough that an
// unexpected power loss usually doesn't lose anything the user
// just did. Don't expose this as user-tunable; it's an
// implementation detail.
#define PERSIST_DEBOUNCE_MS  3000UL

static_assert(BUCKET_COUNT <= 16,
              "PersistDirty bitmasks are uint16_t; widen if "
              "BUCKET_COUNT ever exceeds 16");

struct PersistDirty {
    uint16_t doseMask;
    uint16_t intervalMask;
    uint16_t lastWateredMask;
    uint32_t dirtySinceMs;
};
static PersistDirty g_persistDirty = {};

static void persistMarkDose(int idx) {
    if (idx < 0 || idx >= BUCKET_COUNT) return;
    g_persistDirty.doseMask |= (uint16_t)(1u << idx);
    if (g_persistDirty.dirtySinceMs == 0) g_persistDirty.dirtySinceMs = millis();
}
static void persistMarkInterval(int idx) {
    if (idx < 0 || idx >= BUCKET_COUNT) return;
    g_persistDirty.intervalMask |= (uint16_t)(1u << idx);
    if (g_persistDirty.dirtySinceMs == 0) g_persistDirty.dirtySinceMs = millis();
}
static void persistMarkLastWatered(int idx) {
    if (idx < 0 || idx >= BUCKET_COUNT) return;
    g_persistDirty.lastWateredMask |= (uint16_t)(1u << idx);
    if (g_persistDirty.dirtySinceMs == 0) g_persistDirty.dirtySinceMs = millis();
}

// Force-commit every dirty key right now, ignoring the debounce.
// Called from cycleEndCleanup(), enterFault() (before the state
// flip), the OTA upload-start hook, and the WiFi-reset paths.
// Each branch is a no-op if its mask is clear. Foreground task
// only; do not call from ISR (NVS writes block).
//
// lastWatered note: in uptime mode (no NTP), persisted values
// would be meaningless across reboots, so we skip the commit and
// LEAVE THE MASK SET. If NTP acquires later, the next flush will
// then commit.
static void persistFlushAll() {
    if (!g_prefsOK) { g_persistDirty = {}; return; }
    char key[16];
    if (g_persistDirty.doseMask) {
        for (int i = 0; i < BUCKET_COUNT; i++) {
            if (g_persistDirty.doseMask & (1u << i)) {
                snprintf(key, sizeof(key), "dose%d", i);
                prefs.putUShort(key, g_buckets[i].dose_ml);
            }
        }
        g_persistDirty.doseMask = 0;
    }
    if (g_persistDirty.intervalMask) {
        for (int i = 0; i < BUCKET_COUNT; i++) {
            if (g_persistDirty.intervalMask & (1u << i)) {
                snprintf(key, sizeof(key), "intvl%d", i);
                prefs.putUShort(key, g_buckets[i].interval_hours);
            }
        }
        g_persistDirty.intervalMask = 0;
    }
    if (g_persistDirty.lastWateredMask && g_timeIsWallClock) {
        for (int i = 0; i < BUCKET_COUNT; i++) {
            if (g_persistDirty.lastWateredMask & (1u << i)) {
                snprintf(key, sizeof(key), "lwat%d", i);
                prefs.putUInt(key, g_lastWateredAt[i]);
            }
        }
        g_persistDirty.lastWateredMask = 0;
    }
    // Only reset the timestamp when nothing remains dirty (the
    // wall-clock guard above may have left lastWateredMask set).
    if (g_persistDirty.doseMask == 0
        && g_persistDirty.intervalMask == 0
        && g_persistDirty.lastWateredMask == 0) {
        g_persistDirty.dirtySinceMs = 0;
    }
}

// Commit any dirty keys whose dirty-age exceeds PERSIST_DEBOUNCE_MS.
// Idempotent and cheap when nothing is dirty (single uint32
// comparison). Foreground task only.
static void persistFlushDue() {
    if (g_persistDirty.dirtySinceMs == 0) return;
    if (millis() - g_persistDirty.dirtySinceMs < PERSIST_DEBOUNCE_MS) return;
    persistFlushAll();
}
// Restore persisted last-watered epoch values. Call ONLY after NTP
// has succeeded -- the saved values are epoch seconds and only make
// sense with a real wall clock. Skips entries that look corrupt
// (pre-2023 epoch); those buckets keep whatever default the caller
// initialized them to.
static void persistApplyLastWatered() {
    if (!g_prefsOK) return;
    // Default everyone to "just watered now" first, then overlay any
    // valid persisted values. This prevents zero-init values from
    // making buckets appear billions of seconds overdue on first
    // boot or when NVS is fresh.
    uint32_t nowSec = scheduleTimeNow();
    for (int i = 0; i < g_bucketCount; i++) {
        g_lastWateredAt[i] = nowSec;
    }
    char key[16];
    for (int i = 0; i < g_bucketCount; i++) {
        snprintf(key, sizeof(key), "lwat%d", i);
        uint32_t saved = prefs.getUInt(key, 0);
        // Sanity: must be a real epoch (post-2023) AND not in
        // the future. Future timestamps mean clock skew/corruption;
        // safer to ignore than honor.
        if (saved > 1700000000UL && saved <= nowSec) {
            g_lastWateredAt[i] = saved;
        }
    }
}

// Stamp every non-skip bucket as "watered now," with optional
// skip index. Used on abort paths so the scheduler doesn't
// immediately re-fire after a user cancellation, and by
// /api/water (force all) to reset the schedule after a manual
// cycle. The motion-watchdog mid-cycle deferral uses the skipIdx
// form so the disturbed bucket retries next cycle while every
// other non-skip bucket gets stamped (prevents a cycle storm
// on the next scheduler tick after the deferral).
//
// skipIdx is 0-indexed; pass -1 to stamp every non-skip bucket.
static void stampAllLastWateredExcept(int skipIdx) {
    uint32_t nowSec = scheduleTimeNow();
    for (int i = 0; i < g_bucketCount; i++) {
        if (i == skipIdx) continue;
        if (g_buckets[i].dose_ml == DOSE_SKIP) continue;
        g_lastWateredAt[i] = nowSec;
        persistMarkLastWatered(i);
    }
}
static void stampAllLastWatered() { stampAllLastWateredExcept(-1); }

// Toggle the scheduler pause. On transition from disabled -> enabled,
// stamp every non-skip bucket as just-watered so each restarts its
// interval clock fresh from now. The user explicitly chose to pause;
// when they un-pause, firing every overdue bucket immediately is rarely
// what they want. Mirrors the stampAllLastWatered() semantics already
// used on user-abort cycle ends.
static void setSchedulerEnabled(bool enable, const char* source) {
    bool wasEnabled = g_schedulerEnabled;
    g_schedulerEnabled = enable;
    persistSaveSchedEnabled();
    if (enable && !wasEnabled) {
        stampAllLastWatered();
        LOGI("Scheduler ENABLED (%s); stamped all non-skip buckets\n",
             source ? source : "?");
    } else if (!enable && wasEnabled) {
        LOGI("Scheduler PAUSED (%s); manual ops still work\n",
             source ? source : "?");
    }
}

// Common cycle-end cleanup. Called from every exit path of
// runWateringCycle so flags don't leak across cycles and the
// scheduler doesn't immediately re-fire on next tick.
static void cycleEndCleanup(bool stampWatered) {
    if (stampWatered) stampAllLastWatered();
    g_webForceAll       = false;
    g_webRequestSingle  = false;
    g_webRequestSingleN = 0;
    g_testModeActive    = false;
    g_testModeWet       = false;
    // Random subset is per-cycle: clear so a subsequent non-random cycle
    // (or the scheduler after the demo ends) is never accidentally
    // restricted. g_demoRandomWet is the session-wide caller choice and is
    // deliberately NOT cleared here -- runDemoLoop re-rolls the mask each
    // iteration and resets g_demoRandomWet on the next demo start.
    g_demoRandomActive  = false;
    g_demoRandomMask    = 0;
    clearActiveBucketState();
    // Every exit path of runWateringCycle() runs through here, so this
    // is where a cycle's last-watered stamps (and any dose/interval
    // edits a user made during the cycle) become durable. Commit now
    // rather than wait for the debounce: a power loss right after a
    // cycle must not lose the stamps that prove the cycle happened, or
    // the scheduler would re-fire on the next boot.
    persistFlushAll();
}
static void persistSaveDwell() {
    if (!g_prefsOK) return;
    prefs.putUChar("dwellSec", g_postValveDwellSec);
}
static void persistSaveReservoirFull() {
    if (!g_prefsOK) return;
    prefs.putFloat("resvFullMl", g_reservoirFullMl);
}
static void persistSaveLedBrightness() {
    if (!g_prefsOK) return;
    prefs.putUChar("ledBri", g_ledBrightness);
}
static void persistSaveFriendlyName() {
    if (!g_prefsOK) return;
    // Empty string is a valid stored value -- it's how "unset" persists.
    prefs.putString("friendlyName", g_friendlyName);
}
static void persistSaveLevelConfig() {
    if (!g_prefsOK) return;
    // NVS key stays "lvlThresh" for backward compatibility -- existing
    // rigs persist their (renamed) spill threshold under the old key.
    prefs.putFloat("lvlThresh", g_spillThreshDeg);
    prefs.putFloat("motThresh", g_motionThreshG);
}
static void persistSaveMountOrientation() {
    if (!g_prefsOK) return;
    prefs.putUChar("mountOri", (uint8_t)g_mountOrientation);
}
static void persistSaveSchedEnabled() {
    if (!g_prefsOK) return;
    prefs.putUChar("schedOn", g_schedulerEnabled ? 1 : 0);
}
static void persistSaveMotorInverted() {
    if (!g_prefsOK) return;
    prefs.putUChar("motorInv", g_motorInverted ? 1 : 0);
}
// Endstop calibration triple: encoder zero offset (raw degrees at
// bucket 1's centre), the +/- sign of the encoder reading vs. motor
// step direction, and a validity flag. Keys "encZero" / "encDirPos" /
// "encCalOk".
static void persistSaveEndstopCal() {
    if (!g_prefsOK) return;
    prefs.putFloat("encZero",   g_encoderZeroOffset);
    prefs.putUChar("encDirPos", g_bucketAngleStepPositive ? 1 : 0);
    prefs.putUChar("encCalOk",  g_endstopsCalibrated ? 1 : 0);
}

// Restore the endstop calibration triple. Returns true if a valid
// calibration was loaded. Call after persistInit().
static bool persistApplyEndstopCal() {
    if (!g_prefsOK) return false;
    uint8_t ok = prefs.getUChar("encCalOk", 0);
    if (!ok) {
        g_endstopsCalibrated      = false;
        g_encoderZeroOffset       = 0.0f;
        g_bucketAngleStepPositive = true;
        return false;
    }
    g_encoderZeroOffset       = prefs.getFloat("encZero", 0.0f);
    uint8_t dirPos            = prefs.getUChar("encDirPos", 1);
    g_bucketAngleStepPositive = (dirPos != 0);
    g_endstopsCalibrated      = true;

    // Sanity-check the saved offset against the live encoder. After
    // disassembly, magnet shift, or a geometry-constant change the arm
    // can land somewhere the saved calibration can't explain -- inside
    // the ~27deg dead zone arc between bucket 13 and bucket 1 that the
    // mechanical stops physically prevent during normal motion. If we
    // catch that, drop the calibration so setup() falls through to a
    // fresh sweep. Encoder read failures (sensor not yet present, bus
    // glitch) leave the cal intact -- the regular encoderInit() +
    // FAULT_ENCODER_LOST path handles those.
    float curDeg = 0.0f;
    if (encoderReadAngle(&curDeg)) {
        float sign = g_bucketAngleStepPositive ? +1.0f : -1.0f;
        float rel  = (curDeg - g_encoderZeroOffset) * sign;
        while (rel <    0.0f) rel += 360.0f;
        while (rel >= 360.0f) rel -= 360.0f;
        // Valid arc relative to home (in sweep direction):
        //   [0, BUCKET_HOME_TO_FAR_DEG + 5]  (just past CW stop)
        //   U [360 - 5, 360)                 (just before CCW stop)
        const float spanMax = BUCKET_HOME_TO_FAR_DEG + 5.0f;
        const float wrapTol = 360.0f - 5.0f;
        bool valid = (rel <= spanMax) || (rel >= wrapTol);
        if (!valid) {
            LOGI("Saved calibration stale; re-running "
                 "(cur=%.2fdeg zero=%.2fdeg rel=%.2fdeg "
                 "outside valid arc [-5, +%.2f])\n",
                 (double)curDeg, (double)g_encoderZeroOffset,
                 (double)rel, (double)spanMax);
            g_endstopsCalibrated      = false;
            g_encoderZeroOffset       = 0.0f;
            g_bucketAngleStepPositive = true;
            return false;
        }
    }
    return true;
}
static void persistSaveValveAngles() {
    if (!g_prefsOK) return;
    prefs.putUChar("valveOpenAng",  g_valveOpenAng);
    prefs.putUChar("valveCloseAng", g_valveCloseAng);
}
static void persistSaveStepProfile() {
    if (!g_prefsOK) return;
    prefs.putUShort("stepDelayFast", g_stepDelayFastUs);
}
// Apply persisted values that are independent of bucket count. Call
// BEFORE tryScaleInit so the saved gap value can be pushed to the
// U177 immediately after it comes up. The U177 also remembers the
// gap value internally across power cycles, so this NVS-side copy is
// belt-and-suspenders: if the unit is swapped (or its NVS wiped) the
// firmware can re-push the calibrated value on next boot.
// Validate a user-supplied friendly hostname against DNS label rules so
// it is always safe to hand to WiFi.setHostname()/MDNS.begin(): 1..32
// chars, only lowercase a-z / 0-9 / hyphen, and must start with an
// alphanumeric (a leading/anywhere-else hyphen and uppercase are both
// rejected). The empty string is NOT valid here -- callers treat empty
// as the explicit "clear / revert to g_deviceName" signal before this
// runs. Shared by the load path and the HTTP setter so they can never
// disagree on what is acceptable.
static bool isValidFriendlyName(const String& n) {
    size_t len = n.length();
    if (len < 1 || len > 32) return false;
    for (size_t i = 0; i < len; i++) {
        char c = n[i];
        bool alnum = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
        if (i == 0) {
            if (!alnum) return false;           // must start alphanumeric
        } else if (!alnum && c != '-') {
            return false;                        // hyphen allowed after pos 0
        }
    }
    return true;
}

// The hostname this unit actually advertises: the user's friendly name
// when set, else the immutable MAC-derived g_deviceName. Used for
// WiFi.setHostname() and MDNS.begin(); g_deviceName remains the identity
// in logs, the AP SSID, and the "device" mDNS TXT record.
static const String& activeHostname() {
    return g_friendlyName.length() ? g_friendlyName : g_deviceName;
}

static void persistApplyGlobals() {
    if (!g_prefsOK) return;
    // Friendly hostname (user-set; empty = unset -> fall back to
    // g_deviceName). Re-validate on load: a value corrupted or written
    // by a future/older firmware that doesn't meet the hostname rules is
    // dropped rather than pushed to WiFi.setHostname()/MDNS.begin().
    g_friendlyName = prefs.getString("friendlyName", "");
    if (g_friendlyName.length() && !isValidFriendlyName(g_friendlyName)) {
        LOGI("Ignoring invalid persisted friendlyName '%s'\n",
             g_friendlyName.c_str());
        g_friendlyName = "";
    }
    // Reservoir-full mark (UI gauge full-scale; display-only). Restore the
    // persisted value if present and in band, else keep the compile-time
    // default. Out-of-band (NVS corruption / a future firmware writing
    // nonsense) falls back rather than rendering a broken gauge.
    {
        float storedFull = prefs.getFloat("resvFullMl", g_reservoirFullMl);
        if (storedFull >= RESERVOIR_FULL_ML_MIN
            && storedFull <= RESERVOIR_FULL_ML_MAX) {
            g_reservoirFullMl = storedFull;
        }
    }
    g_gapValue        = prefs.getFloat("gapValue", 0.0f);
    g_rawTare         = prefs.getInt("tareRaw", 0);
    g_rawTareValid    = (prefs.getUChar("tareRawV", 0) != 0);
    // Calibrated flag. Sentinel 0xFF distinguishes "key absent" (a unit
    // that predates the flag) from a real 0/1. Migration: a pre-flag
    // unit is considered calibrated iff it carries BOTH a persisted zero
    // (tareRawV set) AND a non-zero gap -- i.e. it was fully calibrated
    // under the interim firmware; back-fill the flag so it isn't forced
    // to recalibrate. A unit with a gap but NO persisted zero (calibrated
    // under firmware older than the host-side-weight change) correctly
    // stays uncalibrated until a fresh factory calibrate captures a
    // matched zero+gap pair.
    {
        uint8_t calFlag = prefs.getUChar("scaleCalOk", 0xFF);
        if (calFlag == 0xFF) {
            g_scaleCalibrated = g_rawTareValid && (g_gapValue != 0.0f);
            if (g_prefsOK) prefs.putUChar("scaleCalOk",
                                          g_scaleCalibrated ? 1 : 0);
        } else {
            g_scaleCalibrated = (calFlag != 0);
        }
    }
    // Post-valve-close dwell override: 1..10 seconds. An out-of-range
    // persisted value (NVS corruption) falls back to the compile-time
    // default rather than silently honoring something nonsensical --
    // a 0s dwell defeats the purpose, and >10s is long enough that
    // it could mask a stuck-open valve.
    uint8_t storedDwell = prefs.getUChar("dwellSec", g_postValveDwellSec);
    if (storedDwell >= POST_VALVE_DWELL_MIN_SEC
        && storedDwell <= POST_VALVE_DWELL_MAX_SEC) {
        g_postValveDwellSec = storedDwell;
    }
    // Status LED brightness. Whole uint8 range is valid (0 = off),
    // so no band check -- any persisted byte is honored.
    g_ledBrightness = prefs.getUChar("ledBri", g_ledBrightness);
    uint8_t storedInv = prefs.getUChar("motorInv", 0);
    g_motorInverted = (storedInv != 0);
    uint8_t storedSched = prefs.getUChar("schedOn", 1);
    g_schedulerEnabled = (storedSched != 0);

    // Valve angles. Range 0..180 covers all servo positions; out-of-
    // range falls back to the compile-time defaults.
    uint8_t storedOpen  = prefs.getUChar("valveOpenAng",  g_valveOpenAng);
    uint8_t storedClose = prefs.getUChar("valveCloseAng", g_valveCloseAng);
    if (storedOpen  <= 180) g_valveOpenAng  = storedOpen;
    if (storedClose <= 180) g_valveCloseAng = storedClose;

    // Step rate.
    uint16_t storedFast = prefs.getUShort("stepDelayFast", g_stepDelayFastUs);
    if (storedFast >= STEP_DELAY_FAST_US_MIN
        && storedFast <= STEP_DELAY_FAST_US_MAX) {
        g_stepDelayFastUs = storedFast;
    }

    // IMU level / motion thresholds. Out-of-range NVS values fall
    // back to the compile-time defaults rather than silently honoring
    // something nonsensical.
    float storedLvl = prefs.getFloat("lvlThresh", g_spillThreshDeg);
    if (storedLvl >= SPILL_THRESH_MIN && storedLvl <= SPILL_THRESH_MAX) {
        g_spillThreshDeg = storedLvl;
    }
    float storedMot = prefs.getFloat("motThresh", g_motionThreshG);
    if (storedMot >= MOTION_THRESH_MIN && storedMot <= MOTION_THRESH_MAX) {
        g_motionThreshG = storedMot;
    }
    uint8_t storedMnt = prefs.getUChar("mountOri", (uint8_t)g_mountOrientation);
    if (storedMnt <= (uint8_t)MOUNT_X_DOWN) {
        g_mountOrientation = (MountOrientation)storedMnt;
    }

    // Endstop calibration. Restored separately so a stale NVS that
    // never had this key falls through cleanly to first-boot
    // calibration in setup().
    (void)persistApplyEndstopCal();

    // Push the saved gap value to the U177 if both the unit is up
    // AND we have a non-default saved value. A negative gap is
    // legitimate when the load cell is wired so load decreases
    // counts -- the U177's `(raw - offset) / gap` formula returns
    // positive grams under load only if both numerator and gap
    // share sign. Only a zero gap means "never calibrated"; trust
    // the unit's own stored value in that case.
    if (scaleOK && g_gapValue != 0.0f) {
        scale.setGapValue(g_gapValue);
        LOGI("Pushed saved gap value %.3f to U177\n", (double)g_gapValue);
    }

    LOGI("Restored reservoir full = %.1f mL, "
                  "gapValue = %.3f, "
                  "dwell = %us, valveOpen = %u, valveClose = %u, "
                  "stepFast = %uus, "
                  "motorInv = %s, spillThresh = %.1fdeg, "
                  "motionThresh = %.3fg, mount = %s, "
                  "endstopsCal = %s (encZero=%.2fdeg, dirPos=%s)\n",
                  g_reservoirFullMl, (double)g_gapValue,
                  (unsigned)g_postValveDwellSec,
                  (unsigned)g_valveOpenAng,
                  (unsigned)g_valveCloseAng,
                  (unsigned)g_stepDelayFastUs,
                  g_motorInverted ? "yes" : "no",
                  (double)g_spillThreshDeg,
                  (double)g_motionThreshG,
                  mountName(g_mountOrientation),
                  g_endstopsCalibrated ? "yes" : "no",
                  (double)g_encoderZeroOffset,
                  g_bucketAngleStepPositive ? "yes" : "no");
}
// Apply persisted dose volumes by bucket index. Call after
// persistInit so the NVS handle is open. Bucket count is fixed.
static void persistApplyDoses() {
    if (!g_prefsOK) return;
    char key[16];
    for (int i = 0; i < g_bucketCount; i++) {
        snprintf(key, sizeof(key), "dose%d", i);
        uint16_t d = prefs.getUShort(key, DOSE_DEFAULT_ML);
        // Accept the skip sentinel or any value on the dose ladder;
        // anything else is treated as corrupted NVS and the prior
        // (post-discovery default) value is left in place.
        if (d == DOSE_SKIP || (d >= DOSE_MIN_ML && d <= DOSE_MAX_ML)) {
            g_buckets[i].dose_ml = d;
        }
    }
    char ikey[16];
    for (int i = 0; i < g_bucketCount; i++) {
        snprintf(ikey, sizeof(ikey), "intvl%d", i);
        uint16_t h = prefs.getUShort(ikey, DEFAULT_INTERVAL_HOURS);
        // Validate against the ladder. Anything else falls back to
        // default; corrupted NVS doesn't break scheduling.
        bool valid = (h == 12 || h == 24 || h == 48 || h == 72
                      || h == 120 || h == 168 || h == 240
                      || h == 336 || h == 504);
        g_buckets[i].interval_hours = valid ? h : DEFAULT_INTERVAL_HOURS;
    }
}

// (Re)init the M5 Unit Mini Scales (U177) via its I2C library. The
// unit's own STM32 owns HX711 timing + filtering; runtime weight is
// computed host-side from the raw ADC (reg 0x00) and our persisted
// (g_rawTare, g_gapValue) pair -- see weightFromRaw() for the why.
// On a clean begin(), pull the unit's gap value into g_gapValue so
// the JSON state / serial readout shows the live value; if the rig
// has a saved value, persistApplyGlobals() pushes it back to the unit.
static bool tryScaleInit() {
    // The U177's STM32 will ACK its 7-bit address on the bit-bang bus
    // well before its register table is ready -- a "ghost ACK" that
    // makes a probe-only check return OK while every subsequent
    // register read NACKs (gap=NAN, weight=NAN, fw=0). We instead
    // require a real register read to consider the unit live; the fw
    // version register (0xFE) is side-effect-free and independent of
    // calibration state, so it's the safe sentinel.
    //
    // Budget: 20 attempts x 75 ms = 1.5 s. Comfortably covers the
    // STM32's observed cold-boot window (well under 1 s) while staying
    // short enough that setup() doesn't visibly stall before WiFi /
    // OTA come up. 75 ms gives the STM32 several main-loop ticks
    // between probes -- finer than that just thrashes the bit-bang bus
    // for no gain.
    scale.begin(0x26);                  // sets _addr; ack result ignored on purpose
    constexpr uint32_t kAttempts = 20;
    constexpr uint32_t kDelayMs  = 75;
    uint8_t fw = 0;
    for (uint32_t i = 0; i < kAttempts; i++) {
        if (scale.getFirmwareVersion(fw)) {
            // Live. Gap handling depends on whether we already hold a
            // calibrated value:
            //   * g_gapValue != 0 -- we have a calibrated gap (NVS
            //     restore, or a prior session). RE-PUSH it to the unit
            //     rather than adopting the unit's reg-0x40 read, which is
            //     0/NAN on a swapped or NVS-wiped U177. This is what makes
            //     an automatic runtime re-detect safe: it can no longer
            //     overwrite a good calibration with the device's blank
            //     value. Negative (inverted-wiring) gaps are pushed
            //     verbatim, never abs()'d.
            //   * g_gapValue == 0 -- never calibrated (e.g. the boot call,
            //     which runs before persistApplyGlobals loads NVS). Adopt
            //     whatever the unit holds; 0/NAN both mean uncalibrated.
            if (g_gapValue != 0.0f) {
                scale.setGapValue(g_gapValue);
            } else {
                float gv = scale.getGapValue();
                g_gapValue = isnan(gv) ? 0.0f : gv;
            }
            return true;
        }
        delay(kDelayMs);
    }
    LOGI("U177 register read failed after %ums (ghost ACK or absent)\n",
         (unsigned)(kAttempts * kDelayMs));
    return false;
}

// Host-side weight calculation: bypass the U177's reg 0x10
// (which applies the unit's own `(raw - storedOffset) / storedGap`
// formula) and compute weight from the raw ADC and our persisted
// tare baseline + gap directly.
//
// Why: the U177 firmware exposes only a capture-trigger at reg 0x50
// (write 1 byte to snapshot current raw as offset) and no value-write
// for offset. After a power cycle we can re-push the persisted gap
// (reg 0x40 accepts an explicit value) but cannot re-push the matching
// tare baseline -- so the unit's stored offset is whatever it captured
// in the last tare, which may not pair with the gap we just restored.
// Reading reg 0x10 in that mismatched state yields wrong-sign /
// wrong-magnitude weight until the user manually re-tares.
//
// Doing the math host-side instead pairs reg 0x00's raw counts with
// our own NVS-persisted (g_rawTare, g_gapValue), which were committed
// together at tare -> cal time and survive cold boots as a matched
// pair. The unit's reg 0x10 / reg 0x50 state becomes irrelevant.
//
// Returns NAN if scale isn't up, we never tared, gap is zero
// (uncalibrated), or the I2C read fails.
static float weightFromRaw() {
    if (!scaleOK || !g_rawTareValid || g_gapValue == 0.0f) return NAN;
    int32_t raw = 0;
    if (!scale.getRawADC(raw)) return NAN;
    return (float)((double)(raw - g_rawTare) / (double)g_gapValue);
}

// Block briefly for the U177 to return a value. We average `n`
// host-computed readings (weightFromRaw) to smooth the load cell's
// natural jitter, mirroring the behavior the old HX711 path provided.
// n is the same caller-chosen sample count used everywhere else.
static float readScaleAvg(int n) {
    if (!scaleOK || n <= 0) return NAN;
    double sum = 0;
    int    got = 0;
    for (int i = 0; i < n; i++) {
        float g = weightFromRaw();
        if (!isnan(g)) { sum += g; got++; }
        if (i + 1 < n) delay(5);
    }
    if (got == 0) return NAN;
    return (float)(sum / (double)got);
}

// Settle-gated baseline read for the dispense path. The arm move that
// precedes a pour disturbs a vibration-sensitive cell, which then creeps
// for seconds; an instantaneous average (readScaleAvg) captures a wrong
// baseline and the pour mis-reads as "reverse drift". Wait for the cell
// to actually settle: collect ~DISP_BASELINE_WINDOW_MS windows and accept
// the mean only once a window is tight (spread <= STABLE_G) AND its mean
// matches the previous window (creep has flattened, not just low jitter).
// Falls back to an instantaneous average after DISP_BASELINE_MAX_WAIT_MS
// so a genuinely noisy cell still dispenses. Writes the settled grams to
// *out and returns true; returns false on user abort or an unreadable
// cell (sets no output). Pumps the UI/web and abort check while waiting.
static bool readScaleSettled(float* out) {
    if (!scaleOK) return false;
    uint32_t start   = millis();
    float    prevMean = NAN;
    while ((uint32_t)(millis() - start) < DISP_BASELINE_MAX_WAIT_MS) {
        float mn = INFINITY, mx = -INFINITY, sum = 0.0f;
        int   got = 0;
        uint32_t w = millis();
        while ((uint32_t)(millis() - w) < DISP_BASELINE_WINDOW_MS) {
            float g = weightFromRaw();
            if (!isnan(g)) { if (g < mn) mn = g; if (g > mx) mx = g; sum += g; got++; }
            if (serviceAbort()) return false;   // user stop while settling
            renderWateringFocused();
            delay(50);
        }
        if (got >= 3 && (mx - mn) <= DISP_BASELINE_STABLE_G) {
            float mean = sum / (float)got;
            if (!isnan(prevMean) && fabsf(mean - prevMean) <= DISP_BASELINE_STABLE_G) {
                *out = mean;
                return true;
            }
            prevMean = mean;
        } else if (got >= 3) {
            // wide window: reset the consecutive-window tracker
            prevMean = sum / (float)got;
        }
    }
    // Settle budget spent (still jittering/creeping): best-effort average
    // so the cycle still dispenses rather than stalling on a noisy cell.
    float g = readScaleAvg(5);
    if (isnan(g)) return false;
    *out = g;
    LOGI("%s\n", "baseline settle timed out; using instantaneous average");
    return true;
}

// ---------- scale health / auto re-detect ----------
//
// Periodic liveness check so a wedged or unplugged U177 recovers without
// the operator hitting a manual Re-detect button. The probe is a REAL
// data read (getRawADC, reg 0x00) -- NOT getFirmwareVersion -- so it
// fails on exactly the clock-stretch read failure that takes weight
// readings down, rather than passing on an address/ghost ACK.
//
// While online: a real read every SCALE_HEALTH_PERIOD_MS; after
// SCALE_HEALTH_MAX_FAILS consecutive failures the scale is marked
// offline (scaleOK=false) and re-detect is armed. While offline: a
// single fast getRawADC probe every SCALE_REDETECT_PERIOD_MS (bounded
// by the bit-bang stretch timeout, so cheap on a dead bus); only when
// that probe answers do we run the full tryScaleInit() to re-establish
// gap state. This never runs the 1.5s tryScaleInit budget against a
// dead bus.
#define SCALE_HEALTH_PERIOD_MS     3000
#define SCALE_HEALTH_MAX_FAILS        3
#define SCALE_REDETECT_PERIOD_MS   5000
static uint32_t g_lastScaleHealthMs   = 0;
static uint32_t g_lastScaleRedetectMs = 0;
static uint8_t  g_scaleHealthFails    = 0;

// A successful raw read during the calibrate wizard proves the U177 is
// live -- the same criterion the health probe uses. Re-adopt it as
// online (and clear any accumulated probe failures) so a stale
// scaleOK=false from a pre-cal probe glitch can't leave weightG null
// after a good calibrate, or re-disable anything mid-wizard.
static void markScaleLiveFromCal() {
    scaleOK            = true;
    g_scaleHealthFails = 0;
}

static void scaleHealthTick() {
    if (g_state == WATERING) return;   // never perturb an active dispense
    uint32_t now = millis();
    // The two-step calibrate wizard owns the scale exclusively from the
    // empty-cell zero capture until the gap commit. Suspend the probe so
    // it can neither flip scaleOK nor re-adopt the device's stale gap
    // mid-wizard. A watchdog releases ownership (probe resumes) if the
    // operator captured zero and then walked away -- but the captured
    // zero is left intact, so a later Calibrate still pairs with it.
    if (g_scaleCalActive) {
        if (now - g_scaleCalStartMs < SCALE_CAL_TIMEOUT_MS) return;
        g_scaleCalActive = false;
        LOGI("%s\n", "Scale cal: ownership timed out; health probe resumed "
                     "(captured zero retained)");
    }
    if (scaleOK) {
        if (now - g_lastScaleHealthMs < SCALE_HEALTH_PERIOD_MS) return;
        g_lastScaleHealthMs = now;
        int32_t raw = 0;
        if (scale.getRawADC(raw)) { g_scaleHealthFails = 0; return; }
        if (g_scaleHealthFails < 0xFF) g_scaleHealthFails++;
        if (g_scaleHealthFails >= SCALE_HEALTH_MAX_FAILS) {
            LOGI("Scale health: %u consecutive read failures -- marking "
                 "U177 offline; auto-redetect armed\n",
                 (unsigned)g_scaleHealthFails);
            scaleOK = false;
            g_lastScaleRedetectMs = now;
        }
    } else {
        if (now - g_lastScaleRedetectMs < SCALE_REDETECT_PERIOD_MS) return;
        g_lastScaleRedetectMs = now;
        int32_t raw = 0;
        if (!scale.getRawADC(raw)) return;       // one-shot, fast on failure
        if (tryScaleInit()) {
            scaleOK = true;
            g_scaleHealthFails = 0;
            LOGI("%s\n", "Scale health: U177 re-detected; back online");
        }
    }
}

// ---------- IMU ----------
//
// Idempotent init. M5.begin() (called via AtomS3.begin in setup)
// already started the IMU; this just queries the detected type and
// flips g_imuPresent. Failure is non-fatal: level/motion checks
// short-circuit to "ok" when the IMU is absent, so a broken IMU
// doesn't brick the rig.
static bool imuInit() {
    g_imuType = (int)AtomS3.Imu.getType();
    g_imuPresent = AtomS3.Imu.isEnabled();
    if (g_imuPresent) {
        const char* nm = "?";
        switch (g_imuType) {
            case m5::imu_mpu6886: nm = "MPU6886"; break;
            case m5::imu_bmi270:  nm = "BMI270";  break;
            case m5::imu_mpu6050: nm = "MPU6050"; break;
            case m5::imu_mpu9250: nm = "MPU9250"; break;
            case m5::imu_sh200q:  nm = "SH200Q";  break;
            default:              nm = "unknown"; break;
        }
        LOGI("IMU OK: %s (mount=%s)\n", nm, mountName(g_mountOrientation));
    } else {
        LOGI("%s\n", "IMU not present (continuing without level checks)");
    }
    return g_imuPresent;
}

// Map raw board-axis values to (up, tx, ty) for the configured
// mount orientation. 'up' is the gravity-aligned axis when the rig
// is installed level; tx and ty are the two tilt-sensing axes.
// Pitch and roll are computed from those three values in imuTick.
static void axesForMount(float ax, float ay, float az,
                         float* up, float* tx, float* ty) {
    switch (g_mountOrientation) {
        case MOUNT_Z_UP:   *up = az;  *tx = ax;  *ty = ay;  break;
        case MOUNT_Z_DOWN: *up = -az; *tx = ax;  *ty = -ay; break;
        case MOUNT_Y_UP:   *up = ay;  *tx = ax;  *ty = -az; break;
        case MOUNT_Y_DOWN: *up = -ay; *tx = ax;  *ty = az;  break;
        case MOUNT_X_UP:   *up = ax;  *tx = -az; *ty = ay;  break;
        case MOUNT_X_DOWN: *up = -ax; *tx = az;  *ty = ay;  break;
    }
}

// One sample tick. Called from loop() at ~50 Hz via a millis gate.
// Updates the gravity EMA (drives pitch/roll), the rolling |a| ring
// (drives the vibration metric), and publishes g_pitchDeg /
// g_rollDeg / g_motionMetric. Cheap enough (~50us per call) to run
// during WATERING too -- IMU is on a separate I2C bus from the
// AtomicMotion / HX711 path.
static void imuTick() {
    if (!g_imuPresent) return;
    uint32_t now = millis();
    if (now - g_lastImuSampleMs < IMU_SAMPLE_PERIOD_MS) return;
    g_lastImuSampleMs = now;

    float ax = 0, ay = 0, az = 0;
    if (!AtomS3.Imu.getAccel(&ax, &ay, &az)) {
        // A single failed read is normal on a shared I2C bus, so the
        // first few failures just keep the previous metric. But holding
        // the last value indefinitely is the bug: if a read fails right
        // after the metric crossed the threshold, the stale "moving"
        // reading persists across every subsequent failed call and
        // motionOk() keeps returning false against it -- latching a
        // motion deferral that never clears. After IMU_MAX_CONSEC_FAILS
        // failures in a row the IMU is wedged/unplugged, not merely
        // mid-sample, so force the metric to 0 (reads as "still") and
        // warn once until a successful read recovers.
        if (g_imuFailCount < IMU_MAX_CONSEC_FAILS) {
            g_imuFailCount++;
        } else {
            g_motionMetric = 0.0f;
            if (!g_imuStaleWarned) {
                LOGI("IMU getAccel() failed %d consecutive reads; "
                     "forcing motion metric to 0\n", g_imuFailCount);
                g_imuStaleWarned = true;
            }
        }
        return;
    }
    // Successful read: clear the failure streak (and re-arm the warning
    // so a later run logs again). Note the recovery only if we'd warned.
    if (g_imuStaleWarned) {
        LOGI("%s\n", "IMU getAccel() recovered");
    }
    g_imuFailCount   = 0;
    g_imuStaleWarned = false;
    g_axRaw = ax; g_ayRaw = ay; g_azRaw = az;

    // Gravity EMA. First sample seeds the filter so the tilt readout
    // doesn't ramp from zero on the first second after init or after
    // a mount-orientation change.
    if (!g_imuFiltSeeded) {
        g_axFilt = ax; g_ayFilt = ay; g_azFilt = az;
        g_imuFiltSeeded = true;
    } else {
        g_axFilt += IMU_GRAVITY_ALPHA * (ax - g_axFilt);
        g_ayFilt += IMU_GRAVITY_ALPHA * (ay - g_ayFilt);
        g_azFilt += IMU_GRAVITY_ALPHA * (az - g_azFilt);
    }

    float up, tx, ty;
    axesForMount(g_axFilt, g_ayFilt, g_azFilt, &up, &tx, &ty);
    // Standard tilt-from-gravity formula. atan2 handles every
    // quadrant; the asin form is gimbal-locked past 90deg and we
    // don't want a silent wrap.
    g_pitchDeg = atan2f(-tx, sqrtf(ty * ty + up * up)) * 180.0f / (float)M_PI;
    g_rollDeg  = atan2f(ty, up) * 180.0f / (float)M_PI;

    // Vibration: peak-to-peak of |a| over the rolling window.
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    g_motionRing[g_motionRingHead] = mag;
    g_motionRingHead = (g_motionRingHead + 1) % IMU_MOTION_WINDOW;
    if (g_motionRingFill < IMU_MOTION_WINDOW) g_motionRingFill++;
    float mn = g_motionRing[0], mx = g_motionRing[0];
    for (int k = 1; k < g_motionRingFill; k++) {
        float v = g_motionRing[k];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    g_motionMetric = mx - mn;

    // Hysteretic threshold-crossing log. Without hysteresis a noisy
    // floor sitting near the threshold would spam a chatty pair of
    // log lines on every flap.
    bool over = g_motionMetric > g_motionThreshG;
    if (over && !g_motionAlertActive) {
        LOGI("IMU motion metric crossed threshold: %.3fg > %.3fg\n",
             (double)g_motionMetric, (double)g_motionThreshG);
        g_motionAlertActive = true;
    } else if (!over && g_motionAlertActive
               && g_motionMetric < g_motionThreshG * MOTION_CLEAR_FRACTION) {
        LOGI("IMU motion returned within threshold: %.3fg\n",
             (double)g_motionMetric);
        g_motionAlertActive = false;
    }
}

// IMU absence is silently OK -- the feature is a safety enhancement,
// not a hard dependency. motionOk() also returns true until the
// ring buffer has filled at least once so a fresh boot doesn't
// trip on a half-empty window.
//
// spillOk(): rig tilt (pitch or roll) is within the spill threshold,
// i.e. shallow enough that a watering pour won't slosh out of the
// bucket. Gates dispensing only; motor ops ignore it.
static inline bool spillOk() {
    if (!g_imuPresent) return true;
    return fabsf(g_pitchDeg) <= g_spillThreshDeg
        && fabsf(g_rollDeg)  <= g_spillThreshDeg;
}
static inline bool motionOk() {
    if (!g_imuPresent) return true;
    if (g_motionRingFill < IMU_MOTION_WINDOW) return true;
    return g_motionMetric <= g_motionThreshG;
}

// How long the deferred condition must read "resolved" continuously
// before the deferral is cleared. Debounces a momentary dip below the
// motion threshold (or a single noisy scale sample) so the deferral
// state doesn't flap on and off.
#define DEFER_CLEAR_DEBOUNCE_MS  2000UL

// Per-tick re-evaluation of the active deferral. A deferral is meant to
// be transient -- it should clear the instant the underlying condition
// resolves -- but the only existing clear path is clearDeferralState()
// on a clean exit of runWateringCycle(). If the scheduler keeps
// deferring, no bucket is due, or the scheduler is paused, a long-since
// resolved condition stays latched forever, and the LED / web UI keep
// reporting it (the "rig moving (0.00g, threshold 0.15g)" banner the
// user sees: the metric has dropped but g_lastDeferReason was never
// reset). This checks the live condition for the current reason and
// clears the deferral once it has held resolved for
// DEFER_CLEAR_DEBOUNCE_MS. Cheap: an if-ladder plus a debounce, no I/O
// (the reservoir case reads the cached g_reservoirMl, which renderUI
// keeps fresh from the non-blocking scale read). Called from loop()'s
// READY branch before the scheduler check.
static void clearStaleDeferralIfResolved() {
    if (g_lastDeferReason == DEFER_NONE) return;

    bool resolved = false;
    switch (g_lastDeferReason) {
        case DEFER_NOT_LEVEL:
            resolved = spillOk();
            break;
        case DEFER_MOVING:
            resolved = motionOk();
            break;
        case DEFER_RESERVOIR_LOW: {
            // Mirror the pre-cycle guard's sum-of-doses + 5% margin so
            // the deferral clears under exactly the condition that lets
            // the next cycle proceed.
            //
            // g_reservoirMl is otherwise refreshed only inside renderUI(),
            // which early-returns while the display sleeps -- so a refill
            // during display sleep would leave the cached value frozen and
            // this deferral would never clear. Take a direct cell read
            // here, millis-gated to at most once per 2s so the poll stays
            // cheap. Skip when the scale is down; on a NAN read keep the
            // cached value (no worse than the old frozen behavior).
            static uint32_t lastReadMs = 0;
            uint32_t nowMs = millis();
            if (scaleOK && (uint32_t)(nowMs - lastReadMs) >= 2000) {
                lastReadMs = nowMs;
                float g = weightFromRaw();
                if (!isnan(g)) g_reservoirMl = g / G_PER_ML;
            }
            float totalDoseMl = 0;
            for (int i = 0; i < g_bucketCount; i++)
                totalDoseMl += (float)g_buckets[i].dose_ml;
            resolved = g_reservoirMl >= totalDoseMl * 1.05f;
            break;
        }
        default:
            return;
    }

    // Debounce. resolvedSinceMs marks when the current uninterrupted
    // resolved streak began; reset it whenever the check fails, when no
    // streak is in progress, or when the reason changed underneath us
    // (a fresh deferral for a different reason must not inherit a
    // half-elapsed timer).
    static uint32_t   resolvedSinceMs = 0;
    static DeferReason debouncingReason = DEFER_NONE;
    uint32_t now = millis();

    if (!resolved || debouncingReason != g_lastDeferReason) {
        resolvedSinceMs  = resolved ? now : 0;
        debouncingReason = g_lastDeferReason;
        return;
    }
    if (resolvedSinceMs == 0) {
        resolvedSinceMs = now;
        return;
    }
    if (now - resolvedSinceMs >= DEFER_CLEAR_DEBOUNCE_MS) {
        LOGI("Deferral (%s) condition resolved; clearing\n",
             deferReasonName(g_lastDeferReason));
        clearDeferralState();
        resolvedSinceMs  = 0;
        debouncingReason = DEFER_NONE;
    }
}

// ---------- motion primitives ----------
//
// 28BYJ-48 (bipolar-modded) driven directly through the AtomicMotion
// Base's two H-bridges (M1 = coil A, M2 = coil B), full-step, both
// coils at full magnitude. Position is the absolute AS5600 angle;
// stepN() just advances phases (the step count is informational, not
// authoritative). gotoBucket() is a closed-loop servo against the
// encoder reading.
//
// The valve sits on Servo channel 1 (external servo-to-H-bridge
// module driving the 5V solenoid). All valve actuation goes through
// valveOpen() / valveClose() so the angles persist and the wiring
// can be reversed via the servo angle constants without code edits.

// Four-phase full-step drive table. Each entry is the signed PWM for
// (M1, M2). Phases advance 0->1->2->3->0 for one direction; reversing
// the index goes the other way. Both coils always at full magnitude
// (+/-127) -- no microstepping, no PWM modulation.
static const int8_t kPhaseTable[4][2] = {
    { +127, +127 },   // 0
    { -127, +127 },   // 1
    { -127, -127 },   // 2
    { +127, -127 },   // 3
};

// -------- AS5600 driver --------
// Register reads over the bit-bang bus (see swi2c* above). AS5600
// shares Port A with the U177 scale via the U006 hub; the AtomicMotion
// internal devices (0x38 + 0x40) are on a different peripheral and
// don't contend.
// AS5600 datasheet (rev 1.06):
//   * Section 5.2: register map (STATUS=0x0B, ANGLE_H=0x0E, ANGLE_L=0x0F)
//   * Section 5.4: STATUS bits MD=0x20, ML=0x10, MH=0x08
//   * Section 5.5: ANGLE register is 12-bit (low nibble of MSB + LSB);
//                  the AS5600's ZPOS/MPOS are left at POR=0 so ANGLE
//                  reads the raw mechanical angle (Section 7.2.1).
static bool encoderReadAngle(float* outDeg) {
    uint8_t b[2];
    if (!swi2cReadRegister(g_encBus, AS5600_I2C_ADDR, AS5600_REG_ANGLE_H, b, 2)) {
        return false;
    }
    uint16_t raw = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
    *outDeg = (float)raw * 360.0f / 4096.0f;
    return true;
}

static bool encoderReadStatus(uint8_t* outStatus) {
    return swi2cReadRegister(g_encBus, AS5600_I2C_ADDR, AS5600_REG_STATUS, outStatus, 1);
}

static bool encoderInit() {
    uint8_t s = 0;
    bool stRead = encoderReadStatus(&s);
    g_encoderStatus = stRead ? s : 0;
    float deg = 0.0f;
    bool angRead = encoderReadAngle(&deg);
    if (angRead) g_armPositionDeg = deg;
    g_encoderConsecFails = 0;
    // Health = MD only. A real-world diametric magnet shows AGC variation
    // across rotation -- ML/MH flicker briefly even though the ANGLE
    // register tracks correctly. Treat ML/MH as informational and fault
    // only on MD=0 (magnet absent) or sustained I2C failure.
    bool magnetOk = stRead && (s & AS5600_STATUS_MD);
    g_encoderHealthy = stRead && angRead && magnetOk;
    if (!stRead || !angRead) {
        LOGI("%s\n", "AS5600 init: I2C read failed");
        return false;
    }
    LOGI("AS5600 OK: angle=%.2fdeg status=0x%02X (MD=%d ML=%d MH=%d)\n",
         (double)deg, (unsigned)s,
         (s & AS5600_STATUS_MD) ? 1 : 0,
         (s & AS5600_STATUS_ML) ? 1 : 0,
         (s & AS5600_STATUS_MH) ? 1 : 0);
    if (stRead && (s & (AS5600_STATUS_ML | AS5600_STATUS_MH))) {
        LOGI("AS5600 init: ML/MH set in status 0x%02X "
             "(informational; not a fault)\n", (unsigned)s);
    }
    return g_encoderHealthy;
}

// ~50 Hz refresh of g_armPositionDeg, and a slower (1 Hz) status reread.
// Cheap (~200us per call worst case via two I2C reads). Bumps the
// consecutive-fail counter on I2C errors; transitions g_encoderHealthy
// false after ENC_MAX_CONSEC_FAILS and back to true on any clean read
// with the MD bit set.
//
// Health policy: only MD=0 (magnet absent) and sustained I2C failure
// flip g_encoderHealthy false. ML (signal weak) / MH (signal strong)
// flicker harmlessly during normal rotation as the AGC tracks; they
// log on transition (so a real degradation is visible) but do NOT
// fault. Prior-iteration policy treated any of {ML, MH} as a fault,
// which made the rig unusable in bench testing.
static void encoderTick() {
    uint32_t now = millis();
    if (now - g_lastEncoderTickMs < ENC_TICK_PERIOD_MS) return;
    g_lastEncoderTickMs = now;

    float deg;
    if (encoderReadAngle(&deg)) {
        g_armPositionDeg = deg;
        g_encoderConsecFails = 0;
    } else {
        if (g_encoderConsecFails < 0xFF) g_encoderConsecFails++;
        if (g_encoderConsecFails >= ENC_MAX_CONSEC_FAILS) {
            g_encoderHealthy = false;
        }
    }

    if (now - g_lastEncoderStatusMs >= ENC_STATUS_PERIOD_MS) {
        g_lastEncoderStatusMs = now;
        uint8_t s;
        if (encoderReadStatus(&s)) {
            // [DBG-CAL] log MD (magnet detected) transitions only.
            // ML/MH flicker harmlessly under AGC during normal rotation
            // and were too noisy in the bench trace.
            uint8_t prevMd = g_encoderStatus & AS5600_STATUS_MD;
            uint8_t newMd  = s              & AS5600_STATUS_MD;
            if (prevMd != newMd) {
                LOGD("[DBG-CAL] AS5600 MD transition: %s -> %s "
                     "(status=0x%02X)\n",
                     prevMd ? "present" : "absent",
                     newMd  ? "present" : "absent",
                     (unsigned)s);
            }
            g_encoderStatus = s;
            // Loosened health: MD presence only. ML/MH alone are
            // informational. Same I2C-fail-counter gate on restore.
            bool magnetPresent = (s & AS5600_STATUS_MD);
            if (magnetPresent && g_encoderConsecFails == 0) {
                g_encoderHealthy = true;
            } else if (!magnetPresent) {
                if (g_encoderHealthy) {
                    LOGI("AS5600 magnet ABSENT: status=0x%02X\n",
                         (unsigned)s);
                }
                g_encoderHealthy = false;
            }
        }
    }
}

// Signed shortest-path angular error in degrees, wrapped to (-180,+180].
// target - current, treating both as moduluso of 360.
static float angleErrorDeg(float target, float current) {
    float e = target - current;
    while (e >   180.0f) e -= 360.0f;
    while (e <= -180.0f) e += 360.0f;
    return e;
}

// Position along the valid (reachable) arc, measured as non-negative
// distance from the CCW home stop, normalized to [0, 360). The carousel
// has a ~37deg dead zone between the CW and CW stops that the rotor
// physically cannot cross, so the unreachable wedge maps to values in
// (BUCKET_HOME_TO_FAR_DEG, 360) -- a rotor in the dead zone shouldn't
// happen mechanically, but if it did its arc position would be in that
// "above the far stop" range. Home stop = 0, far stop ~= 323.37.
//
// Used by gotoBucket() so the direction selection accounts for the
// dead zone: shortest-path encoder error can point through the unreach-
// able wedge (e.g. far stop -> bucket 1 is "52deg the short way" but
// the rotor must travel the long way, ~307deg). Arc position turns
// every move into a 1-D problem along [0, BUCKET_HOME_TO_FAR_DEG], no
// wrap-around to worry about.
static float arcPositionFromHome(float angleDeg) {
    float sign = g_bucketAngleStepPositive ? +1.0f : -1.0f;
    float rel  = (angleDeg - g_encoderZeroOffset) * sign;
    while (rel <    0.0f) rel += 360.0f;
    while (rel >= 360.0f) rel -= 360.0f;
    return rel;
}

// Signed arc-distance error in degrees: targetArcPos - currentArcPos.
// Sign tells which direction along the valid arc to travel; magnitude
// is the true arc distance, NOT shortest-path. With +arc defined as
// "away from home toward far", motor dir +1 always moves +arc (it
// inverts the encoder delta sign exactly when g_bucketAngleStepPositive
// is false, so the two sign flips cancel). So:
//     dir = (arcErr > 0) ? +1 : -1
// drives the rotor along the arc toward target regardless of which
// calibration sign came out of calibrateEndstops().
static float arcErrorDeg(float targetAng, float currentAng) {
    return arcPositionFromHome(targetAng) - arcPositionFromHome(currentAng);
}

// Absolute encoder angle of bucket n's centre, mod 360.
// The home (CCW) endstop sits BUCKET_ENDSTOP_OFFSET_DEG before bucket 1,
// so bucket 1's centre is g_encoderZeroOffset + BUCKET_ENDSTOP_OFFSET_DEG
// * sign, and bucket n's centre is one slot-width further per index.
static float bucketTargetAngleDeg(int n) {
    float sign = g_bucketAngleStepPositive ? +1.0f : -1.0f;
    float fromZero = BUCKET_ENDSTOP_OFFSET_DEG
                   + (float)(n - 1) * BUCKET_SLOT_DEG;
    float a = g_encoderZeroOffset + fromZero * sign;
    while (a >= 360.0f) a -= 360.0f;
    while (a <  0.0f)   a += 360.0f;
    return a;
}

// Map an encoder angle to the closest bucket (1-indexed). Used by
// printState / api/state to render a coarse "currently at" label.
static int bucketFromAngle(float deg) {
    int best = 1;
    float bestErr = 360.0f;
    for (int n = 1; n <= BUCKET_COUNT; n++) {
        float e = fabsf(angleErrorDeg(bucketTargetAngleDeg(n), deg));
        if (e < bestErr) { bestErr = e; best = n; }
    }
    return best;
}

// Write the coil currents for a given phase index. Two I2C transactions;
// caller controls the timing between phases via delayMicroseconds().
static void setStepperPhase(uint8_t phase) {
    phase &= 0x03;
    g_phaseIndex = phase;
    motion.setMotorSpeed(STEPPER_M1_CH, kPhaseTable[phase][0]);
    motion.setMotorSpeed(STEPPER_M2_CH, kPhaseTable[phase][1]);
    g_coilsEnergized = true;
}

// Drop both coils. The gearbox detent holds the rotor against ambient
// load; this saves the ~150 mA per coil that would otherwise burn
// continuously between moves. Phase index is NOT changed -- the next
// stepperWake() re-energizes the same phase so motion resumes without
// a phantom step.
static void stepperReleaseCoils() {
    motion.setMotorSpeed(STEPPER_M1_CH, 0);
    motion.setMotorSpeed(STEPPER_M2_CH, 0);
    g_coilsEnergized = false;
}

// Re-energize the saved phase and give the coils a brief moment to
// pull the rotor against the detent before stepping resumes.
static void stepperWake() {
    setStepperPhase(g_phaseIndex);
    // Bench-trace the supply rail at every motion start. This is the one
    // safe place for the read: stepperWake() is outside the inner step
    // loop (one INA226 I2C transaction here costs ~1ms, harmless; the
    // same read inside stepN() would crater the step rate). The 28BYJ
    // bipolar mod wants ~5V; a sagging rail (bench saw 4.02V) loses
    // torque, so having Vbus in the trace at each move makes
    // brownout-vs-wiring triage instant when the rotor won't traverse.
    LOGD("[DBG-CAL] stepperWake: Vbus=%.2fV\n", (double)busVoltage());
    delay(COIL_WAKE_SETTLE_MS);
}

// One-shot I2C ACK probe of the AtomicMotion. M5AtomicMotion::setMotorSpeed()
// returns an argument-validation code, NOT the bus ACK, and discards the
// write result -- so a wedged Wire link to the module (setMotorSpeed
// becoming a silent no-op) is otherwise invisible. A bare address probe on
// the same Wire bus returns the endTransmission status (0 == ACK). Cheap
// (one short transaction); call only at decision points, never in the step
// loop.
static bool atomicMotionAck() {
    Wire.beginTransmission(M5_ATOMIC_MOTION_I2C_ADDR);
    return Wire.endTransmission() == 0;
}

// Re-establish the motor-drive I2C link after a confirmed drive wedge
// (steps commanded, encoder healthy, yet zero angle delta). This is NOT a
// power/sequencing action -- Vbus is fine; the failure is the Wire link to
// the AtomicMotion STM32 going unresponsive so setMotorSpeed() silently
// no-ops. Drop the coils, re-run motion.begin() on the same Wire bus (which
// re-inits the peripheral and re-probes the module's ACK), then re-wake.
// Returns true only if the module ACKs again. The carousel encoder/scale
// bit-bang buses live on different pins (GPIO 7/8, 5/6) and are untouched
// by re-begin'ing Wire on GPIO 38/39. For a sealed unit with no reboot
// available, this is the in-place recovery for an otherwise-permanent wedge.
static bool recoverMotorDrive() {
    LOGI("%s\n", "motor-drive recovery: re-initializing AtomicMotion I2C link");
    stepperReleaseCoils();
    bool ok = motion.begin(&Wire, M5_ATOMIC_MOTION_I2C_ADDR,
                           I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    if (!ok) {
        LOGI("%s\n", "motor-drive recovery: AtomicMotion still not ACKing "
                     "after re-init -- giving up this move");
        return false;
    }
    LOGI("%s\n", "motor-drive recovery: AtomicMotion re-init OK; re-waking coils");
    stepperWake();
    return true;
}

// Apply the inversion flag and compute the next phase index for a
// signed direction. dir = +1 advances 0->1->2->3; dir = -1 reverses;
// g_motorInverted swaps the sense.
static inline uint8_t nextPhase(uint8_t cur, int dir) {
    int effective = g_motorInverted ? -dir : dir;
    int next = ((int)cur + effective) & 0x03;
    return (uint8_t)next;
}

// Hand-rolled stepping at the single configured rate. The delay
// between phase writes is the time held in the current phase before
// advancing; I2C write time is NOT subtracted from that delay (same
// as real STEP/DIR drivers, which care about pulse separation not
// pulse width).
//
// Updates g_stepCounter (diagnostic only -- the AS5600 is position
// truth). Polls serviceAbort every STEP_ABORT_POLL_PERIOD steps with
// pumpWeb=false (web pump is too slow for the inner loop; it runs from
// loop() between motion calls).
//
// Returns true on completion, false on abort. On abort the rotor is
// left in the current phase with coils still energized -- the caller
// is responsible for the eventual stepperReleaseCoils().
static bool stepN(int32_t signed_steps) {
    if (signed_steps == 0) return true;

    int dir              = (signed_steps > 0) ? 1 : -1;
    uint32_t totalSteps  = (uint32_t)((signed_steps > 0)
                                       ? signed_steps : -signed_steps);
    uint16_t delayUs     = g_stepDelayFastUs;

    g_coilsEnergized = true;
    for (uint32_t i = 0; i < totalSteps; i++) {
        setStepperPhase(nextPhase(g_phaseIndex, dir));
        g_stepCounter += dir;
        delayMicroseconds(delayUs);

        if ((i & (STEP_ABORT_POLL_PERIOD - 1)) == 0) {
            // pumpWeb=false: handleClient() takes 1-50ms which would
            // crater the step rate. The web abort path still fires
            // between motion calls from the top-level loop() pump.
            if (serviceAbort(/*pumpWeb=*/false)) {
                return false;
            }
        }
    }
    return true;
}

// ---------- closed-loop motion ----------
//
// Drive the rotor to bucket n's encoder target as a position servo:
// compute signed-shortest-path error, take a small step burst in the
// inferred direction, re-read the encoder, repeat. Bounded by
// GOTO_MAX_ITER iterations so a stalled rotor exits cleanly (returns
// false, no fault). The caller (runWateringCycle) treats false as
// "skip this bucket" -- same graceful-degradation policy as the old
// hall-based code, but without any slip-recovery machinery: the
// encoder IS truth, every iteration starts from a fresh ground reading.
//
// Direction sign: g_bucketAngleStepPositive (set during calibrateEndstops)
// captures whether "motor +1 step" increases or decreases the encoder
// reading. If err > 0 we need to INCREASE the encoder angle. We then
// pick the motor direction that achieves that. An adaptive flip kicks
// in once if the first burst increased the error past the hysteresis
// threshold, so a wrong calibration sign (or a transient wedge) doesn't
// burn the whole iteration budget moving away from the target.
//
// Encoder-lost handling: a clean fault path -- if g_encoderHealthy
// goes false at any point (either MD/ML/MH flagged it or
// encoderConsecFails exceeded ENC_MAX_CONSEC_FAILS), enter
// FAULT_ENCODER_LOST and bail. The cycle path treats this as a
// terminal failure; user must ack and (presumably) reseat the magnet.
static bool gotoBucket(int n) {
    if (n < 1 || n > g_bucketCount) {
        LOGI("gotoBucket: bad idx %d\n", n);
        return false;
    }
    // Auto-recovery: position is "lost" if endstops were never calibrated
    // or were invalidated (e.g. user just cleared FAULT_ENCODER_LOST and
    // the saved calibration is stale). If the encoder itself is healthy
    // we can re-establish position by re-calibrating instead of refusing
    // forever. Only fault if calibration itself fails.
    if (!g_endstopsCalibrated) {
        if (!g_encoderHealthy) {
            LOGI("gotoBucket(%d): encoder unhealthy and uncalibrated; "
                 "cannot recover\n", n);
            enterFault(FAULT_ENCODER_LOST, n);
            return false;
        }
        LOGI("gotoBucket(%d): endstops uncalibrated; auto-calibrating\n", n);
        if (!calibrateEndstops()) {
            LOGI("gotoBucket(%d): auto-calibration failed\n", n);
            if (g_state != ERROR) enterFault(FAULT_ENCODER_LOST, n);
            return false;
        }
        // calibrateEndstops() parks at bucket 1 on success. If the caller
        // wanted bucket 1, we're already there; fall through to the
        // closed-loop servo so the final-error log still fires.
    }
    if (!g_encoderHealthy) {
        LOGI("gotoBucket(%d): encoder unhealthy; skipping\n", n);
        enterFault(FAULT_ENCODER_LOST, n);
        return false;
    }

    float target = bucketTargetAngleDeg(n);

    // One-shot motor-drive recovery latch, function-scoped so the I2C-link
    // re-init happens at most ONCE per gotoBucket call (not once per
    // attempt). Set when a mid-arc no-progress window is diagnosed as a
    // drive wedge (steps commanded + encoder healthy + zero delta) rather
    // than a stale encoder read; after recovery is attempted once, further
    // no-progress windows fall back to the stale-read re-read path.
    bool driveRecoveryTried = false;

    // Bounded move retry. A single MAX_ITER exhaustion (e.g. a stale
    // bit-bang encoder-read cluster the in-loop mid-arc recovery didn't
    // fully absorb) must not be terminal: each attempt re-reads ground
    // truth and re-runs the closed-loop servo from scratch. The abort
    // and FAULT_ENCODER_LOST paths below still return false immediately
    // (they exit the function, not just the attempt). Final exhaustion
    // returns false unchanged -> caller's HOME FAIL / fault path.
    for (int attempt = 0; attempt <= GOTO_MAX_RETRIES; attempt++) {
        // Force a fresh encoder read so the initial error reflects ground
        // truth even if the 50Hz tick hasn't fired recently.
        float startDeg = 0.0f;
        if (encoderReadAngle(&startDeg)) {
            g_armPositionDeg = startDeg;
            g_encoderConsecFails = 0;
        } else {
            if (g_encoderConsecFails < 0xFF) g_encoderConsecFails++;
            if (g_encoderConsecFails >= ENC_MAX_CONSEC_FAILS) {
                g_encoderHealthy = false;
                enterFault(FAULT_ENCODER_LOST, n);
                return false;
            }
        }

        // Arc-aware error: signed distance along the valid arc, NOT
        // shortest-path on the circle. Picks the direction that goes the
        // physically reachable way around the dead zone (e.g. far stop ->
        // bucket 1 must travel ~307deg the long way, not 52deg through the
        // unreachable wedge). Motor dir +1 always moves +arc regardless of
        // the calibration sign -- see arcErrorDeg comment.
        float err = arcErrorDeg(target, g_armPositionDeg);
        int dir = (err > 0) ? +1 : -1;
        bool triedFlip       = false;   // one direction flip per attempt
        bool staleReadLogged = false;   // one mid-arc recovery log per attempt

        // [DBG-CAL] gotoBucket entry: target, current, initial arc err, dir.
        LOGI("gotoBucket(%d): attempt %d/%d target=%.2fdeg current=%.2fdeg "
             "arc_err=%+.2f dir=%+d (curArc=%.2f tgtArc=%.2f)\n",
             n, attempt + 1, GOTO_MAX_RETRIES + 1,
             (double)target, (double)g_armPositionDeg,
             (double)err, dir,
             (double)arcPositionFromHome(g_armPositionDeg),
             (double)arcPositionFromHome(target));

        // Rolling encoder-angle window for no-progress detection. A
        // full window of <ENC_STALL_DEG_GOTO travel means either the
        // rotor is pinned against a mechanical stop (only plausible near
        // the arc ends -- flip direction once to recover) or a stale
        // bit-bang read cluster mid-arc (recover the read, keep going).
        constexpr int STALL_W = 8;
        float angWindow[STALL_W] = {0};
        int   windowFill = 0;
        int   windowHead = 0;
        // Step counter snapshot at the moment the current window opened, so
        // a no-progress window can be checked against how many steps were
        // actually COMMANDED across it. A full window (windowFill==STALL_W)
        // means STALL_W bursts of GOTO_STEP_BURST were issued -- if those
        // steps moved the rotor the encoder angle would have changed, so
        // zero delta despite a commanded step count is a drive wedge, not a
        // stale read. g_stepCounter is the diagnostic step tally (signed).
        int32_t windowStartStepCount = g_stepCounter;
        // Reset both the angle window and its step-count baseline together,
        // so the "steps commanded since window opened" measure stays honest
        // after every flip / recovery that clears the window.
        auto resetWindow = [&]() {
            windowFill = 0;
            windowHead = 0;
            windowStartStepCount = g_stepCounter;
        };

        // Zero-crossing arrival: if arc_err sign flips between iterations,
        // the rotor just crossed the target. Declare arrived even if the
        // absolute error is outside the ARRIVAL_TOL_DEG band so a tight
        // tolerance doesn't cause overshoot oscillation when the mechanics
        // can't land precisely on it. prev_arc_err starts at the entry
        // arc-err so a single overshooting burst counts as a cross.
        float prev_arc_err = err;
        // One-shot guard so a rejected (wide) zero-cross logs a single
        // bench-trace line, not one per iteration while converging.
        bool  zeroCrossWideLogged = false;

        stepperWake();
        int iter;
        for (iter = 0; iter < GOTO_MAX_ITER; iter++) {
            err = arcErrorDeg(target, g_armPositionDeg);
            float absErr = fabsf(err);
            if (absErr <= ARRIVAL_TOL_DEG) break;
            // Zero-crossing: sign flipped vs previous iteration (and we've
            // actually moved away from the entry point) means we just
            // crossed something. A cross that lands NEAR the target (within
            // GOTO_ZEROCROSS_MAX_ERR_DEG) is a real overshoot -- bail out
            // instead of dithering. A cross that lands far out is the rotor
            // crossing the 360->0 dead-zone wrap boundary (arcErrorDeg flips
            // sign at large magnitude), NOT an arrival: keep iterating. The
            // step direction is already correct for the new sign post-cross,
            // so the servo converges from there.
            if (iter > 0 && (signbit(prev_arc_err) != signbit(err))) {
                if (absErr <= GOTO_ZEROCROSS_MAX_ERR_DEG) {
                    LOGI("gotoBucket(%d): zero-crossing arrival iter=%d "
                         "prev_err=%+.2f err=%+.2f (|err|=%.2f tol=%.2f)\n",
                         n, iter, (double)prev_arc_err, (double)err,
                         (double)absErr, (double)ARRIVAL_TOL_DEG);
                    break;
                }
                if (!zeroCrossWideLogged) {
                    LOGI("gotoBucket(%d): zero-cross rejected (wide) iter=%d "
                         "prev_err=%+.2f err=%+.2f (|err|=%.2f > %.2f); "
                         "wrap boundary, continuing\n",
                         n, iter, (double)prev_arc_err, (double)err,
                         (double)absErr, (double)GOTO_ZEROCROSS_MAX_ERR_DEG);
                    zeroCrossWideLogged = true;
                }
            }
            prev_arc_err = err;

            if (!stepN((int32_t)dir * GOTO_STEP_BURST)) {
                // Abort: leave coils up; caller's serviceAbort path will
                // handle release. Don't set g_currentBucket since we
                // didn't arrive.
                return false;
            }

            // Re-read encoder. Failure increments the consec-fail counter
            // and may trip FAULT_ENCODER_LOST.
            float fresh;
            if (encoderReadAngle(&fresh)) {
                g_armPositionDeg = fresh;
                g_encoderConsecFails = 0;
            } else {
                if (g_encoderConsecFails < 0xFF) g_encoderConsecFails++;
                if (g_encoderConsecFails >= ENC_MAX_CONSEC_FAILS) {
                    g_encoderHealthy = false;
                    stepperReleaseCoils();
                    enterFault(FAULT_ENCODER_LOST, n);
                    return false;
                }
            }

            // Update the no-progress window with the freshly-read angle.
            // Span is computed as shortest-path delta from window[0] so the
            // detector tolerates 0/360 wrap-around. Reset the window after
            // any flip / recovery so post-event motion isn't immediately
            // misclassified.
            angWindow[windowHead] = g_armPositionDeg;
            windowHead = (windowHead + 1) % STALL_W;
            if (windowFill < STALL_W) windowFill++;
            bool noProgressStall = false;
            if (windowFill >= STALL_W && !triedFlip) {
                float mn = 0.0f, mx = 0.0f;
                for (int k = 0; k < STALL_W; k++) {
                    float d = angleErrorDeg(angWindow[k], angWindow[0]);
                    if (k == 0) { mn = d; mx = d; }
                    else { if (d < mn) mn = d; if (d > mx) mx = d; }
                }
                if ((mx - mn) < ENC_STALL_DEG_GOTO) noProgressStall = true;
            }

            // A no-progress window is only a real stall when the rotor is
            // near a mechanical stop (arc position within
            // GOTO_FLIP_STOP_PROXIMITY_DEG of 0 = home stop, or of
            // BUCKET_HOME_TO_FAR_DEG = far stop) -- there, flip direction
            // once to recover a wrong-way-into-the-stop pin. Mid-arc, a
            // no-progress window is a stale bit-bang read cluster, not a
            // stall; flipping would drive away from target and burn the
            // budget into a terminal HOME FAIL. So mid-arc we instead
            // force a fresh read, reset the window, and continue in the
            // same direction -- the read recovers and the move completes.
            if (noProgressStall && !triedFlip) {
                float curArc = arcPositionFromHome(g_armPositionDeg);
                bool nearStop =
                    (curArc <= GOTO_FLIP_STOP_PROXIMITY_DEG)
                 || (curArc >= BUCKET_HOME_TO_FAR_DEG
                                - GOTO_FLIP_STOP_PROXIMITY_DEG);
                if (nearStop) {
                    // [DBG-CAL] flip with old->new dir (legit stop stall).
                    LOGI("gotoBucket(%d): flipping dir (no-progress stall "
                         "near stop, curArc=%.2f) dir %+d->%+d\n",
                         n, (double)curArc, dir, -dir);
                    dir = -dir;
                    triedFlip = true;
                    resetWindow();
                    // Resync prev_arc_err to the fresh reading so the
                    // zero-crossing detector doesn't fire spuriously off
                    // the pre-flip error sign.
                    prev_arc_err = arcErrorDeg(target, g_armPositionDeg);
                } else {
                    // Mid-arc no-progress: NOT near a stop, so flipping
                    // would drive away from target. Two distinct causes,
                    // handled differently:
                    //   (1) stale bit-bang encoder read (a cluster of bad
                    //       reads) -- the rotor really IS moving; re-read
                    //       and continue. Historical behavior, common case.
                    //   (2) drive wedge -- the encoder reads cleanly but the
                    //       angle hasn't changed despite STALL_W*GOTO_STEP_
                    //       BURST steps just COMMANDED. setMotorSpeed() has
                    //       silently no-op'd because the AtomicMotion I2C
                    //       link is unresponsive (the post-abort symptom).
                    //       Re-reading can NEVER fix that -- the move loops
                    //       to MAX_ITER "rotor stuck?". Re-init the drive
                    //       path ONCE and retry instead.
                    float reread;
                    bool  readOk = encoderReadAngle(&reread);
                    if (readOk) {
                        g_armPositionDeg = reread;
                        g_encoderConsecFails = 0;
                    }
                    int32_t stepsCommanded = g_stepCounter - windowStartStepCount;
                    int32_t stepsMag = stepsCommanded < 0
                                     ? -stepsCommanded : stepsCommanded;
                    // A full window is STALL_W bursts of GOTO_STEP_BURST;
                    // require at least half that to call it "steps issued".
                    bool stepsIssued = stepsMag >= (STALL_W * GOTO_STEP_BURST) / 2;
                    if (readOk && stepsIssued && !driveRecoveryTried) {
                        // Drive wedge: encoder healthy, steps commanded,
                        // zero motion. Distinct from a stale read.
                        driveRecoveryTried = true;
                        bool ack = atomicMotionAck();
                        LOGI("gotoBucket(%d): mid-arc no-progress (curArc=%.2f) "
                             "with %ld steps commanded and a healthy encoder "
                             "read -- not a stale read but a MOTOR-DRIVE WEDGE "
                             "(AtomicMotion ACK=%s); attempting one-shot drive "
                             "re-init\n",
                             n, (double)curArc, (long)stepsCommanded,
                             ack ? "yes" : "NO");
                        if (recoverMotorDrive()) {
                            // Re-init OK: reset window/baseline and retry the
                            // move in the same direction. If the rotor now
                            // turns, the servo converges from here.
                            resetWindow();
                            prev_arc_err = arcErrorDeg(target, g_armPositionDeg);
                            // A drive wedge is a notable electrical event.
                            // Position trust is intact (we re-read the absolute
                            // encoder every iter), but force a recal on the
                            // NEXT cycle so the bidirectional motion-integrity
                            // probe re-runs -- early detection of a recurring /
                            // marginal drive before it strands a cycle. One-
                            // shot; consumed by runWateringCycle.
                            g_forceRecalNextCycle = true;
                        } else {
                            // The re-init itself failed (module still not
                            // ACKing). ONLY here do we give up -- a
                            // recoverable wedge never reaches this path.
                            // recoverMotorDrive() already dropped the coils.
                            return false;
                        }
                    } else {
                        // Stale read (read failed, steps unconfirmed, or the
                        // one-shot drive recovery is already spent):
                        // historical behavior -- reset window, keep direction.
                        resetWindow();
                        prev_arc_err = arcErrorDeg(target, g_armPositionDeg);
                        if (!staleReadLogged) {
                            LOGI("gotoBucket(%d): mid-arc no-progress "
                                 "(curArc=%.2f, not near a stop); treating as "
                                 "stale read -- re-reading, continuing dir %+d\n",
                                 n, (double)curArc, dir);
                            staleReadLogged = true;
                        }
                    }
                }
            }

            if (serviceAbort()) {
                stepperReleaseCoils();
                return false;
            }
        }

        stepperReleaseCoils();
        if (iter < GOTO_MAX_ITER) {
            g_currentBucket = n;
            // Telemetry: an arrival outside the tight tolerance band landed
            // via a wide zero-crossing. Not a fault and not a recal trigger
            // (the absolute encoder is re-read every hop, so this doesn't
            // accumulate) -- just count + log so the production test and
            // field watch can trend a mechanically marginal rig.
            if (fabsf(err) > ARRIVAL_TOL_DEG) {
                g_largeFinalErrCount++;
                g_lastLargeFinalErrDeg = err;
                LOGI("gotoBucket(%d): LARGE final err=%.2fdeg (> tol %.2f; "
                     "wide zero-cross land) count=%lu -- telemetry, not a "
                     "fault\n",
                     n, (double)err, (double)ARRIVAL_TOL_DEG,
                     (unsigned long)g_largeFinalErrCount);
            }
            // [DBG-CAL] gotoBucket exit (success): attempt, iter, final err.
            LOGI("gotoBucket(%d): arrived in %d iter (attempt %d/%d), "
                 "final err=%.2fdeg\n",
                 n, iter, attempt + 1, GOTO_MAX_RETRIES + 1, (double)err);
            return true;
        }

        // This attempt exhausted its iteration budget.
        if (attempt < GOTO_MAX_RETRIES) {
            LOGI("gotoBucket(%d): MAX_ITER (%d) on attempt %d/%d; "
                 "err=%.2fdeg (target=%.2f current=%.2f); retrying with "
                 "fresh read\n",
                 n, GOTO_MAX_ITER, attempt + 1, GOTO_MAX_RETRIES + 1,
                 (double)err, (double)target, (double)g_armPositionDeg);
        } else {
            // [DBG-CAL] gotoBucket exit (fail): final attempt, final err.
            LOGI("gotoBucket(%d): MAX_ITER (%d) reached after %d attempts; "
                 "err=%.2fdeg (target=%.2f current=%.2f); rotor stuck?\n",
                 n, GOTO_MAX_ITER, GOTO_MAX_RETRIES + 1, (double)err,
                 (double)target, (double)g_armPositionDeg);
        }
    }
    return false;
}

// ---------- endstop calibration ----------
//
// Replaces all the old homing / steps-per-bucket / magnet-mapping
// machinery. Drives the rotor against the CCW mechanical stop, records
// the AS5600 angle there as g_encoderZeroOffset (= the home stop, which
// sits BUCKET_ENDSTOP_OFFSET_DEG before bucket 1's centre), then does a
// short CW burst to determine the direction sign
// (g_bucketAngleStepPositive).
//
// Stall detection (phase 1 only): every STALL_SAMPLE_EVERY_STEPS steps,
// log the current encoder angle in a rolling window of size
// STALL_WINDOW_STEPS; if the angle hasn't moved more than
// ENC_STALL_DEG_CAL across the full window, the rotor is against the
// stop. Phase 2 doesn't drive to the CW stop -- a few degrees of motion
// is enough to fix the sign, and the CW stop sits near 268deg where the
// rotor catches on hardware we don't want to grind against.
//
// Direction convention: "drive -1" is "CCW toward home stop". If the
// rig wiring sends the rotor the wrong way (no CCW stop hit), the user
// flips g_motorInverted from the web UI and re-runs calibration.
//
// On success: g_encoderZeroOffset and g_bucketAngleStepPositive are
// set + persisted, g_endstopsCalibrated is true, rotor is parked at
// bucket 1, returns true. On failure: returns false; previous saved
// calibration (if any) is left untouched.
//
// Returns true / false; reads + sets globals; persists via
// persistSaveEndstopCal().
//
// Helper: cumulative-aware delta between encoder reads. The CW sweep
// crosses ~308 deg so the start/end raw angles may straddle the 0/360
// boundary; we walk the encoder via small samples and accumulate the
// signed deltas of each sample (each delta itself shortest-path mod
// 360), giving the true unwrapped displacement.
static bool calibrateEndstops() {
    LOGI("%s\n", "== Calibrating endstops ==");

    if (!g_encoderHealthy) {
        LOGI("%s\n", "calibrateEndstops: encoder not healthy; aborting");
        return false;
    }
    AbortableMotionGuard motionGuard;
    g_abortRequested = false;
    g_stepCounter = 0;

    // The 28BYJ-48 with the bipolar mod has plenty of torque at the
    // single fast step rate (~3000us = ~333 steps/sec) -- no need for
    // a separate slow approach for calibration. The stall-detection
    // window absorbs any encoder noise at this rate.

    // Pre-flight motion-integrity probe (BIDIRECTIONAL). A non-tracking
    // encoder (decoupled magnet) or a non-stepping motor (dead coil
    // drive) produce the same zero-travel signature as a real mechanical
    // stall, so phase 1 alone cannot tell them apart and will declare a
    // phantom home position. A miswired coil pair / weak-or-open phase /
    // marginal Vbus is sneakier still: the rotor develops torque in ONE
    // direction while it stalls and jiggles in the other (bench: FORWARD
    // 1088 steps -> 0.09deg travel, REVERSE fine). The OLD single +burst
    // probe passed that broken rig because the one good direction cleared
    // the 3deg floor. So drive BOTH ways: +CAL_PROBE_STEPS, read the
    // delta, then -CAL_PROBE_STEPS back, read the return delta. BOTH
    // magnitudes must clear CAL_PROBE_MIN_DEG. The -steps return burst
    // also restores the rotor's pre-probe position, so phase 1's home
    // drive below is unaffected. steps/deg is logged per direction (a
    // healthy 28BYJ bipolar-mod gearbox runs ~5.7 steps/deg at full
    // step) -- trace only; the 3deg floor stays the sole pass/fail gate.
    LOGI("%s\n", "calibrateEndstops: motion-integrity probe (bidirectional)");
    stepperWake();
    float probeA = 0.0f, probeB = 0.0f, probeC = 0.0f;
    if (!encoderReadAngle(&probeA)) {
        stepperReleaseCoils();
        LOGI("%s\n", "calibrateEndstops: probe encoder read failed (pre)");
        return false;
    }
    if (!stepN(+CAL_PROBE_STEPS)) {
        stepperReleaseCoils();
        if (g_abortRequested) {
            LOGI("%s\n", "calibrateEndstops: aborted in probe phase");
        }
        return false;
    }
    if (!encoderReadAngle(&probeB)) {
        stepperReleaseCoils();
        LOGI("%s\n", "calibrateEndstops: probe encoder read failed (mid)");
        return false;
    }
    if (!stepN(-CAL_PROBE_STEPS)) {
        stepperReleaseCoils();
        if (g_abortRequested) {
            LOGI("%s\n", "calibrateEndstops: aborted in probe phase");
        }
        return false;
    }
    if (!encoderReadAngle(&probeC)) {
        stepperReleaseCoils();
        LOGI("%s\n", "calibrateEndstops: probe encoder read failed (post)");
        return false;
    }
    float probeFwdDelta = angleErrorDeg(probeB, probeA);   // +steps direction
    float probeRevDelta = angleErrorDeg(probeC, probeB);   // -steps direction
    float fwdStepsPerDeg = (fabsf(probeFwdDelta) > 0.001f)
                         ? (float)CAL_PROBE_STEPS / fabsf(probeFwdDelta) : 0.0f;
    float revStepsPerDeg = (fabsf(probeRevDelta) > 0.001f)
                         ? (float)CAL_PROBE_STEPS / fabsf(probeRevDelta) : 0.0f;
    LOGD("[DBG-CAL] calEndstops probe steps=%ld A=%.2f B=%.2f C=%.2f "
         "fwdDelta=%+.2f revDelta=%+.2f\n",
         (long)CAL_PROBE_STEPS, (double)probeA, (double)probeB,
         (double)probeC, (double)probeFwdDelta, (double)probeRevDelta);
    LOGI("calibrateEndstops: probe fwd=%+.2fdeg (%.1f steps/deg) "
         "rev=%+.2fdeg (%.1f steps/deg)\n",
         (double)probeFwdDelta, (double)fwdStepsPerDeg,
         (double)probeRevDelta, (double)revStepsPerDeg);
    if (fabsf(probeFwdDelta) < CAL_PROBE_MIN_DEG
        || fabsf(probeRevDelta) < CAL_PROBE_MIN_DEG) {
        stepperReleaseCoils();
        const char* failDir =
            (fabsf(probeFwdDelta) < CAL_PROBE_MIN_DEG
             && fabsf(probeRevDelta) < CAL_PROBE_MIN_DEG) ? "BOTH directions"
          : (fabsf(probeFwdDelta) < CAL_PROBE_MIN_DEG)    ? "FORWARD (+steps)"
          :                                                 "REVERSE (-steps)";
        LOGI("calibrateEndstops: motion probe FAILED in %s "
             "(fwd=%+.2fdeg rev=%+.2fdeg, expected |delta| >= %.2f each); "
             "asymmetric/no torque -- check coil pairing "
             "M1=orange+yellow, M2=pink+blue, and Vbus; calibration "
             "aborted\n",
             failDir, (double)probeFwdDelta, (double)probeRevDelta,
             (double)CAL_PROBE_MIN_DEG);
        return false;
    }

    // Phase 1: drive CCW (-1) until the encoder stops moving.
    LOGI("%s\n", "calibrateEndstops: phase 1 (CCW drive to home stop)");
    stepperWake();

    auto driveToStop = [&](int dir, float* outFinalAngleDeg,
                           float* outTravelDeg) -> bool {
        float startAngle;
        if (!encoderReadAngle(&startAngle)) {
            LOGI("%s\n", "calibrateEndstops: encoder read failed at start");
            return false;
        }
        // [DBG-CAL] phase start.
        LOGD("[DBG-CAL] calEndstops phase dir=%+d start=%.2fdeg\n",
             dir, (double)startAngle);
        float lastSampleAngle = startAngle;
        float cumulativeTravel = 0.0f;
        // Rolling window of cumulativeTravel snapshots. Stall = window
        // span less than ENC_STALL_DEG_CAL (essentially zero travel
        // across the whole window). A real stop is zero motion forever;
        // mid-arc slowdowns produce small but non-zero motion, so the
        // tight threshold + long window combination only trips on
        // actual mechanical contact.
        const int W = STALL_WINDOW_STEPS / STALL_SAMPLE_EVERY_STEPS;
        float window[STALL_WINDOW_STEPS / STALL_SAMPLE_EVERY_STEPS] = {0};
        int   windowFill = 0;
        int   windowHead = 0;

        // STATUS poll cadence. encoderTick() doesn't run while
        // calibrateEndstops is blocking the foreground task, so the
        // sweep checks the AS5600 status register itself every
        // STATUS_CHECK_EVERY samples. Without this, a magnet that
        // pops off mid-sweep would still ACK on I2C and return
        // garbage angles -- stall detection would either trip on
        // noise (false success) or never trip (timeout to "no stall"
        // fall-through). 8 samples at the slow rate is ~half a second.
        const int STATUS_CHECK_EVERY = 8;
        int       statusCounter      = 0;

        int32_t stepsDone = 0;
        while (stepsDone < ENDSTOP_MAX_STEPS) {
            if (!stepN((int32_t)dir * STALL_SAMPLE_EVERY_STEPS)) {
                return false;   // abort
            }
            stepsDone += STALL_SAMPLE_EVERY_STEPS;

            float ang;
            if (!encoderReadAngle(&ang)) {
                if (g_encoderConsecFails < 0xFF) g_encoderConsecFails++;
                if (g_encoderConsecFails >= ENC_MAX_CONSEC_FAILS) {
                    g_encoderHealthy = false;
                    LOGI("%s\n", "calibrateEndstops: encoder I2C-fail "
                                 "streak exceeded; aborting sweep");
                    return false;
                }
                continue;
            }
            g_armPositionDeg = ang;
            g_encoderConsecFails = 0;

            // Periodic magnet-presence check via STATUS register. Only
            // MD=0 (magnet absent) aborts the sweep -- ML/MH alone
            // flicker harmlessly during normal AGC tracking and the
            // ANGLE register stays valid through them. If we abort here
            // we flip g_encoderHealthy false so the caller (or boot-time
            // loud-fault path) escalates correctly.
            if (++statusCounter >= STATUS_CHECK_EVERY) {
                statusCounter = 0;
                uint8_t st;
                if (encoderReadStatus(&st)) {
                    g_encoderStatus = st;
                    if (!(st & AS5600_STATUS_MD)) {
                        g_encoderHealthy = false;
                        LOGI("calibrateEndstops: magnet ABSENT during sweep "
                             "(status=0x%02X); aborting\n",
                             (unsigned)st);
                        return false;
                    }
                }
                // Status I2C failure here is treated like any other
                // read failure by the angle-read path on the next
                // iteration; no need to double-count.
            }

            // Accumulate signed shortest-path delta into cumulative
            // travel. This unwraps 0/360 boundary crossings cleanly.
            float d = angleErrorDeg(ang, lastSampleAngle);
            cumulativeTravel += d;
            lastSampleAngle = ang;

            window[windowHead] = cumulativeTravel;
            windowHead = (windowHead + 1) % W;
            if (windowFill < W) windowFill++;

            if (windowFill >= W) {
                float mn = window[0], mx = window[0];
                for (int k = 1; k < W; k++) {
                    if (window[k] < mn) mn = window[k];
                    if (window[k] > mx) mx = window[k];
                }
                float windowSpan = mx - mn;
                // [DBG-CAL] one line per window evaluation so a sweep
                // that mis-triggers (or misses a real stop) shows the
                // cumTravel curve. ~100 lines per phase at fast rate.
                LOGD("[DBG-CAL] cal window span=%.2f cumTravel=%+.2f\n",
                     (double)windowSpan, (double)cumulativeTravel);
                if (windowSpan < ENC_STALL_DEG_CAL) {
                    // Stalled.
                    *outFinalAngleDeg = ang;
                    *outTravelDeg     = cumulativeTravel;
                    LOGI("calibrateEndstops: stall detected after %ld steps, "
                         "final angle=%.2fdeg travel=%.2fdeg\n",
                         (long)stepsDone, (double)ang, (double)cumulativeTravel);
                    // [DBG-CAL] stall detail.
                    LOGD("[DBG-CAL] calEndstops STALL dir=%+d steps=%ld "
                         "ang=%.2f cumTravel=%+.2f windowSpan=%.2f\n",
                         dir, (long)stepsDone, (double)ang,
                         (double)cumulativeTravel, (double)windowSpan);
                    return true;
                }
            }
            if (serviceAbort()) return false;
        }
        LOGI("calibrateEndstops: no stall after %ld steps; rotor not "
             "reaching endstop?\n", (long)stepsDone);
        return false;
    };

    float homeAngle = 0.0f, homeTravel = 0.0f;
    if (!driveToStop(-1, &homeAngle, &homeTravel)) {
        stepperReleaseCoils();
        if (g_abortRequested) {
            LOGI("%s\n", "calibrateEndstops: aborted in CCW phase");
            return false;
        }
        LOGI("calibrateEndstops: CCW drive failed (encoderHealthy=%s)\n",
             g_encoderHealthy ? "yes" : "no");
        return false;
    }

    // We're at the home stop. Record the raw home-stop angle (bucket 1's
    // centre is BUCKET_ENDSTOP_OFFSET_DEG further in along the sweep).
    g_encoderZeroOffset = homeAngle;
    LOGI("calibrateEndstops: home stop at AS5600 angle %.2fdeg "
         "(bucket 1 centre = +%.1fdeg from here)\n",
         (double)homeAngle, (double)BUCKET_ENDSTOP_OFFSET_DEG);

    // Phase 2: short CW burst to determine the direction sign. The burst
    // is run in free space rather than against the home stop, because
    // static stiction at the stop can hold the rotor for several steps
    // and the resulting tiny encoder delta (< 1 deg) reads as a non-
    // result. Each attempt backs off CW first to escape the stiction
    // band, then takes the sign sample. A burst that still falls short
    // retries up to PHASE2_MAX_RETRIES times -- each retry's back-off
    // pushes further into free space -- so a transient stiction miss
    // self-heals instead of aborting the whole calibration.
    LOGI("%s\n", "calibrateEndstops: phase 2 (CW burst to determine sign)");
    const int32_t PHASE2_BURST_STEPS   = 50;
    const int32_t PHASE2_BACKOFF_STEPS = 35;     // ~6 deg off home stop
    const int     PHASE2_MAX_RETRIES   = 3;
    const float   PHASE2_MIN_DELTA_DEG = 1.0f;
    float burstDelta = 0.0f;
    bool  burstOk    = false;
    for (int attempt = 1; attempt <= PHASE2_MAX_RETRIES; attempt++) {
        if (!stepN(+PHASE2_BACKOFF_STEPS)) {
            stepperReleaseCoils();
            if (g_abortRequested) {
                LOGI("%s\n", "calibrateEndstops: aborted in CW phase");
            }
            return false;
        }
        float preBurstAngle = 0.0f, postBurstAngle = 0.0f;
        if (!encoderReadAngle(&preBurstAngle)) {
            stepperReleaseCoils();
            LOGI("%s\n", "calibrateEndstops: phase 2 encoder read failed (pre)");
            return false;
        }
        if (!stepN(+PHASE2_BURST_STEPS)) {
            stepperReleaseCoils();
            if (g_abortRequested) {
                LOGI("%s\n", "calibrateEndstops: aborted in CW phase");
            }
            return false;
        }
        if (!encoderReadAngle(&postBurstAngle)) {
            stepperReleaseCoils();
            LOGI("%s\n", "calibrateEndstops: phase 2 encoder read failed (post)");
            return false;
        }
        burstDelta = angleErrorDeg(postBurstAngle, preBurstAngle);
        if (fabsf(burstDelta) >= PHASE2_MIN_DELTA_DEG) {
            burstOk = true;
            break;
        }
        LOGI("calibrateEndstops: phase 2 burst too small (%+.2fdeg), "
             "retry %d/%d\n",
             (double)burstDelta, attempt, PHASE2_MAX_RETRIES);
    }
    if (!burstOk) {
        stepperReleaseCoils();
        LOGI("calibrateEndstops: phase 2 burst still too small after "
             "%d attempts (last delta %+.2fdeg); aborting\n",
             PHASE2_MAX_RETRIES, (double)burstDelta);
        return false;
    }
    g_bucketAngleStepPositive = (burstDelta > 0);
    LOGI("calibrateEndstops: phase 2 burst delta=%+.2fdeg, dir+1 %s encoder\n",
         (double)burstDelta,
         g_bucketAngleStepPositive ? "increases" : "decreases");

    g_endstopsCalibrated = true;
    persistSaveEndstopCal();
    // Position re-established against the home stop: clear the blind-jog
    // session budget so the debug escape hatch is available again.
    g_jogSessionSteps = 0;
    // [DBG-CAL] final summary so the bench operator can grep one line
    // for the canonical zero / direction sign just established.
    LOGD("[DBG-CAL] calEndstops DONE zeroOffset=%.2f burstDelta=%+.2f "
         "sign=%+d\n",
         (double)g_encoderZeroOffset, (double)burstDelta,
         g_bucketAngleStepPositive ? +1 : -1);

    // Park at bucket 1 (we're a few degrees CW of the home stop after
    // the phase 2 burst). gotoBucket now has everything it needs.
    LOGI("%s\n", "calibrateEndstops: parking at bucket 1");
    if (!gotoBucket(1)) {
        LOGI("%s\n",
             "calibrateEndstops: post-calibration park failed (kept "
             "calibration values; user can retry)");
        // A park that fails without a user abort means the rotor cannot
        // traverse the short arc from the home stop to bucket 1 -- the
        // drive moved enough to home and read the sign, then couldn't
        // execute the servo. Latch FAULT_ROTOR_STALL (unless gotoBucket
        // already latched FAULT_ENCODER_LOST, i.e. g_state==ERROR, or the
        // user aborted). enterFault is idempotent -- first fault wins.
        if (!g_abortRequested && g_state != ERROR) {
            LOGI("%s\n",
                 "calibrateEndstops: rotor will not traverse to bucket 1 "
                 "-- FAULT_ROTOR_STALL. If the motor moves one direction "
                 "only, suspect coil wiring -- do NOT toggle motorInv, it "
                 "cannot fix a torque fault and will invalidate "
                 "calibration.");
            enterFault(FAULT_ROTOR_STALL, 1);
        }
        return false;
    }
    LOGI("%s\n", "calibrateEndstops: complete");
    return true;
}

// Convenience wrappers preserving the old public names that other
// code (runWateringCycle, web/serial handlers) still calls. The
// encoder makes a separate "home" operation cheap: gotoBucket(1)
// is a short servo when we already know where the arm is.
static bool homeStepper() {
    if (!g_endstopsCalibrated) {
        LOGI("%s\n", "homeStepper: endstops not calibrated; "
             "running calibration first");
        if (!calibrateEndstops()) return false;
        // calibrateEndstops parks at bucket 1 on success.
        return true;
    }
    return gotoBucket(1);
}

// ---------- valve ----------
//
// External servo-to-H-bridge module drives the 5V solenoid. The
// servo-angle command latches the H-bridge polarity in the module;
// no need to hold or pulse, and no need to poll for completion (the
// physical valve settles ~100ms after the command). Fire-and-forget.
//
// g_valveIsOpen tracks the last command for UI feedback and the
// /api/valve toggle. It's a command echo, not a sensor reading --
// nothing physically reports the valve's actual state.
static volatile bool g_valveIsOpen = false;

static void valveOpen() {
    motion.setServoAngle(VALVE_SERVO_CH, g_valveOpenAng);
    g_valveIsOpen = true;
}

static void valveClose() {
    motion.setServoAngle(VALVE_SERVO_CH, g_valveCloseAng);
    g_valveIsOpen = false;
}

// ---------- fault helpers ----------
// Centralized transitions in/out of ERROR. enterFault() is the only
// path that should set g_state = ERROR from runtime (setup() failures
// still set ERROR directly without a fault code -- those are pre-
// READY hardware-init problems and the device hasn't booted to a
// usable UI yet). clearFault() is the inverse: ERROR -> READY plus
// a UI redraw. Both are idempotent so the serial 'A' and the
// extra-long gesture path can call freely.
static void enterFault(FaultCode code, int bucket1Indexed) {
    // Idempotency: enterFault can be re-entered from within motion
    // primitives (homeStepper / gotoBucket call enterFault directly
    // for FAULT_HOME_NOT_FOUND / FAULT_STEPS_SLIPPED / FAULT_CALIBRATION_FAILED,
    // and those may be called inside another fault-triggered recovery).
    // Bail if we're already in ERROR with a code recorded -- the first
    // fault wins.
    if (g_state == ERROR && g_lastFault != FAULT_NONE) return;

    // Flush any debounced writes before anything else. A fault must
    // not strand recent edits in RAM, and doing the commit first means
    // even an unexpected failure later in this fault-entry path (valve
    // close, coil release) can't lose data the user just changed.
    persistFlushAll();

    g_lastFault   = code;
    g_faultBucket = bucket1Indexed;
    g_faultMs     = millis();
    // Safety invariant: valve closed FIRST, then coils released, then
    // state flip. Defense in depth -- the cycle paths close the valve
    // too, but a fault from the calibration / home paths might not
    // have a watering cycle wrapper to do it.
    valveClose();
    stepperReleaseCoils();
    interactionEnd();   // a fault stomps any active headless session
    // A fault also stomps any ARMED-but-not-yet-running demo. Without this,
    // a demo start that leaked in during an abortable motor op (which then
    // faulted) would survive the fault and silently launch when the user
    // later cleared it -- the surprise-demo-on-clear half of Finding 1. If a
    // demo is actually RUNNING these are already consumed (pending Off,
    // request false), so this is a no-op there; the running loop's own exit
    // block does the cleanup.
    g_webRequestDemoStart = false;
    g_demoPendingMode     = DemoMode::Off;
    g_state = ERROR;
    // Wake the panel so the red fault banner is actually visible if
    // we faulted after the display had dimmed or slept.
    markInteraction();
    LOGI("FAULT %s%s%d (entering ERROR)\n",
                  faultName(code),
                  (bucket1Indexed > 0) ? " bucket " : "",
                  (bucket1Indexed > 0) ? bucket1Indexed : 0);
    renderUI(true);
}

static void clearFault(const char* why) {
    // Only act when there's a runtime fault to clear. Setup-time
    // ERROR states (motion init failed, no buckets discovered)
    // leave g_lastFault at FAULT_NONE: those failures aren't
    // recoverable from runtime, and forcibly flipping to READY
    // would let the scheduler queue cycles against broken hardware.
    if (g_lastFault == FAULT_NONE) {
        Serial.println("clearFault: nothing to clear");
        return;
    }
    FaultCode wasFault = g_lastFault;
    LOGI("fault cleared (%s; was %s%s%d)\n",
                  why ? why : "?",
                  faultName(g_lastFault),
                  (g_faultBucket > 0) ? " bucket " : "",
                  (g_faultBucket > 0) ? g_faultBucket : 0);
    g_lastFault   = FAULT_NONE;
    g_faultBucket = -1;
    g_faultMs     = 0;
    g_state       = READY;
    renderUI(true);
    // If the just-cleared fault was the encoder dropping its magnet, the
    // saved endstop calibration may still be valid (the magnet was just
    // missing, not re-oriented) -- but if calibration WAS lost (fresh NVS,
    // or a prior motor-direction flip invalidated it), the natural user
    // action after reseating the magnet is to recalibrate. Queue that
    // automatically rather than make the user click Calibrate as a
    // separate step. Foreground-dispatched so this stays out of any HTTP
    // request thread; calibrateEndstops() itself bails safely if the
    // encoder is still unhealthy.
    if (wasFault == FAULT_ENCODER_LOST && !g_endstopsCalibrated) {
        LOGI("%s\n", "clearFault: endstops uncalibrated -- "
                     "queuing automatic calibration");
        g_webRequestCalibrate = true;
    }
    // A rotor-stall fault means the rig just failed to traverse. The saved
    // calibration is likely still numerically valid, so don't invalidate it,
    // but force a recal on the next cycle so the rotor re-establishes home
    // and re-runs the motion-integrity probe before resuming normal moves --
    // robustness over resuming blind on a rig that just couldn't move. One-
    // shot; consumed by runWateringCycle.
    if (wasFault == FAULT_ROTOR_STALL) {
        LOGI("%s\n", "clearFault: was rotor-stall -- forcing recal next cycle");
        g_forceRecalNextCycle = true;
    }
}

// ---------- watering ----------

// True when the persisted endstop calibration can be trusted right now
// WITHOUT re-running calibrateEndstops() (i.e. without driving into the
// home stop): the encoder is healthy, a calibration is loaded, no fault is
// active, and the current arc position falls within the valid reachable arc.
// This is the same arc-validity test the recalibration-failed fallback (and
// persistApplyEndstopCal() at boot) uses. When it holds, the absolute AS5600
// reading is enough to servo accurately and the per-cycle home drive can be
// skipped. Reads the encoder (one I2C transaction); call lazily / last in the
// recalibration-decision so the every-cycle path (N=1) doesn't pay for it.
static bool isSavedCalTrusted() {
    if (g_state == ERROR)                          return false;
    if (!g_endstopsCalibrated || !g_encoderHealthy) return false;
    float curDeg = 0.0f;
    if (!encoderReadAngle(&curDeg))                 return false;
    float sign = g_bucketAngleStepPositive ? +1.0f : -1.0f;
    float rel  = (curDeg - g_encoderZeroOffset) * sign;
    while (rel <    0.0f) rel += 360.0f;
    while (rel >= 360.0f) rel -= 360.0f;
    const float spanMax = BUCKET_HOME_TO_FAR_DEG + 5.0f;
    const float wrapTol = 360.0f - 5.0f;
    return (rel <= spanMax) || (rel >= wrapTol);
}

// Outcome of one closed-loop pour attempt (STEP 2/3). DONE = weight cutoff
// hit the target. SUSPECT = the measurement became untrustworthy mid-pour
// (flow stall / reverse drift) and the valve was shut early -- recoverable.
// ABORT = user stop. TIMEOUT = hit the (cumulative) safety ceiling without a
// cutoff -- the existing no-flow path handles it.
enum DispenseResult { DISP_DONE, DISP_SUSPECT, DISP_ABORT, DISP_TIMEOUT };

// Surfaced for /api/state + printState so a SUSPECT pause / skip is visible
// (the "device faults and pauses" signal) without latching ERROR -- it auto-
// recovers. Persistent badness still latches via consecutiveTimeouts -> NO_FLOW.
static volatile bool g_suspectActive    = false;   // true while paused awaiting a sane cell
static uint32_t      g_suspectSkipCount = 0;        // buckets skipped after suspect + resume
static int           g_lastSuspectBucket = 0;       // 1-indexed; 0 = none

// STEP 3 auto-recover gate + STEP 2 g0-validity guard, in one. With the valve
// CLOSED, decide the cell is trustworthy again before resuming a SUSPECT-paused
// dose. Requires a STABLE window (spread <= DISP_SUSPECT_STABLE_G) AND a mean
// CONSISTENT with the original baseline minus what we already delivered
// (|mean - (g0 - delivered)| <= RESUME_BASELINE_MAX_DRIFT_G). The consistency
// half is the g0-staleness check: a refill / preload shift during the pause
// moves the mean off (g0 - delivered) and is rejected -> caller skips the
// bucket rather than risk an over-pour on a stale baseline. Strictly stronger
// than the under-flow trigger, so a resume never begins on a still-bad signal.
// Bounded by DISP_SUSPECT_MAX_WAIT_MS. Returns false on timeout, unreadable
// cell, baseline drift, or a user abort during the pause (caller re-checks
// g_abortRequested to distinguish stop-vs-skip).
static bool waitCellSane(float g0, float deliveredSoFar) {
    const float expected = g0 - deliveredSoFar;
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < DISP_SUSPECT_MAX_WAIT_MS) {
        float mn = INFINITY, mx = -INFINITY, sum = 0.0f;
        int   got = 0;
        uint32_t w = millis();
        while ((uint32_t)(millis() - w) < DISP_SUSPECT_SETTLE_MS) {
            float g = weightFromRaw();
            if (!isnan(g)) {
                if (g < mn) mn = g;
                if (g > mx) mx = g;
                sum += g; got++;
            }
            if (serviceAbort()) return false;   // user stop during the pause
            renderWateringFocused();
            delay(50);
        }
        if (got >= 3 && (mx - mn) <= DISP_SUSPECT_STABLE_G) {
            float mean  = sum / (float)got;
            float drift = fabsf(mean - expected);
            if (drift <= RESUME_BASELINE_MAX_DRIFT_G) return true;
            LOGI("waitCellSane: cell stable but baseline drifted "
                 "(mean=%.1fg expected=%.1fg drift=%.1fg > %.1fg) -- "
                 "refill/shift during pause; will not resume\n",
                 (double)mean, (double)expected, (double)drift,
                 (double)RESUME_BASELINE_MAX_DRIFT_G);
            return false;
        }
        // not stable yet -- re-window until the max-wait budget is spent
    }
    LOGI("%s\n", "waitCellSane: cell did not stabilize within max wait");
    return false;
}

static bool runWateringCycle() {
    g_state = WATERING;
    g_abortRequested = false;   // fresh start; clear any stale request
    interactionEnd();           // a cycle stomps any active headless session
    // Watering already aborts on BtnA via the WATERING state check in
    // serviceAbort(); setting the abortable-motion flag here is
    // harmless and keeps every long-running handler symmetric.
    AbortableMotionGuard motionGuard;
    AtomS3.Display.fillScreen(UI_BLACK);
    AtomS3.Display.setTextSize(2);
    AtomS3.Display.setCursor(2, 2);
    AtomS3.Display.setTextColor(UI_WHITE, UI_BLACK);
    AtomS3.Display.println(g_testModeActive ? "Test Run" : "Watering");
    LOGI("%s\n", "== Watering cycle ==");
    if (g_testModeActive) {
        LOGI("== DISPENSE TEST mode: %s (dwell=%us per dose unit) ==\n",
             g_testModeWet ? "WET" : "DRY",
             (unsigned)g_postValveDwellSec);
    }

    // Defense in depth: explicit valve close before any motion. The
    // dispense loop will close it again on its own exit paths, but a
    // failure between here and the first valveOpen() should never
    // leave the valve actuated.
    valveClose();

    // Water-safety pre-flight. The motor work below is UNCONDITIONAL
    // -- the rotor doesn't care about tilt -- but DISPENSING is gated.
    // When the IMU is present we check two auto-clearing conditions
    // and DEFER (never fault) if either fails: return cleanly to READY
    // without stamping any bucket, so the scheduler retries on its
    // next tick once the rig is level / still again and the due
    // bucket isn't skipped. A dry test (g_testModeActive && no valve)
    // skips both gates -- there's nothing to spill and the operator
    // explicitly asked for a motion-only run, so tilt / motion are
    // irrelevant.
    const bool dryTest = g_testModeActive && !g_testModeWet;
    if (!dryTest && g_imuPresent && !spillOk()) {
        noteDeferral(DEFER_NOT_LEVEL, "rig tilted past spill threshold");
        cycleEndCleanup(/*stampWatered=*/false);
        g_state = READY;
        renderUI(true);
        return false;
    }
    if (!dryTest && g_imuPresent && !motionOk()) {
        noteDeferral(DEFER_MOVING, "rig moving above motion threshold");
        cycleEndCleanup(/*stampWatered=*/false);
        g_state = READY;
        renderUI(true);
        return false;
    }

    // Cheap reservoir pre-check BEFORE any motion. With an empty tank the
    // scheduler retries this cycle every tick; without this gate each
    // attempt would run calibrateEndstops() (a full drive into the
    // mechanical stop) and gotoBucket(1) before the post-home guard below
    // defers -- grinding the motor continuously until the tank is
    // refilled. A quick 3-sample read is enough to refuse an obviously
    // empty tank and skip the motion. Same g_testModeActive bypass as the
    // authoritative post-home guard, which stays in place (it re-reads
    // after the home settle and remains the real check). If the cell is
    // unreadable (NAN) we fall through -- the post-home guard and the
    // per-bucket safety timeouts still cover that case.
    if (!g_testModeActive) {
        float preG = readScaleAvg(3);
        if (!isnan(preG)) {
            float preTotalDoseMl = 0;
            for (int i = 0; i < g_bucketCount; i++) {
                if (g_buckets[i].dose_ml == DOSE_SKIP) continue;
                preTotalDoseMl += (float)g_buckets[i].dose_ml;
            }
            g_reservoirMl = preG / G_PER_ML;
            if (g_reservoirMl < preTotalDoseMl * 1.05f) {
                noteDeferral(DEFER_RESERVOIR_LOW,
                             "reservoir below total dose (pre-motion)");
                cycleEndCleanup(/*stampWatered=*/false);
                g_state = READY;
                renderUI(true);
                return false;
            }
        }
    }

    // Per-cycle calibration cadence. Historically this drove into the home
    // stop (calibrateEndstops) on EVERY cycle. The AS5600 is absolute, so a
    // trusted saved calibration servos accurately without the home drive;
    // recalibrate this cycle only if ANY of:
    //   - the cadence counter reached CALIBRATE_EVERY_N_CYCLES;
    //   - a prior skipped cycle requested a forced recal after a missed
    //     gotoBucket (possible silent position loss);
    //   - the saved calibration is not currently trusted (no cal loaded, the
    //     arc-validity check fails, or the encoder is unhealthy) -- first-boot
    //     and drift-recovery behavior.
    // The || is ordered so the every-cycle config (N=1) short-circuits on the
    // counter and never pays for isSavedCalTrusted()'s encoder read, keeping
    // N=1 byte-identical to the old unconditional path.
    //
    // Pre-flight (IMU) has already gated tilt / motion, so a calibrating
    // cycle only drives on a stable rig.
    bool calibratedThisCycle = false;
    bool mustCalibrate = (g_cyclesSinceCalibration >= CALIBRATE_EVERY_N_CYCLES)
                       || g_forceRecalNextCycle
                       || !isSavedCalTrusted();
    if (mustCalibrate) {
        // If recalibration fails, fall back to the existing saved
        // calibration when one is trusted -- same arc-validity check
        // persistApplyEndstopCal() uses at boot. For a plant-keeper,
        // dropping a scheduled cycle on a marginal mechanical event is
        // worse than running on a still-valid prior calibration; only
        // abort the cycle if there's no trusted calibration to fall back on.
        if (!calibrateEndstops()) {
            // calibrateEndstops latching a fault (g_state == ERROR, e.g. the
            // post-calibration park failed -> FAULT_ROTOR_STALL) must NOT
            // fall through to saved cal: isSavedCalTrusted() returns false
            // while g_state == ERROR, so a rotor that can't traverse surfaces
            // the fault instead of pressing on.
            if (isSavedCalTrusted()) {
                LOGI("Cycle: recalibration failed; continuing on existing "
                     "saved calibration (encZero=%.2fdeg dirPos=%s)\n",
                     (double)g_encoderZeroOffset,
                     g_bucketAngleStepPositive ? "yes" : "no");
            } else {
                say("CALIBRATE FAIL");
                cycleEndCleanup(/*stampWatered=*/false);
                if (g_state != ERROR) g_state = READY;
                renderUI(true);
                return false;
            }
        }
        // Calibration was attempted this cycle (succeeded, or fell back to a
        // trusted saved cal). Either way the rotor was driven toward home, so
        // the cycle-start and end-of-cycle parks below stay -- matching the
        // old path. Open a fresh cadence epoch and clear the one-shot force.
        calibratedThisCycle      = true;
        g_cyclesSinceCalibration = 1;
        g_forceRecalNextCycle    = false;
    } else {
        // Skipped calibration: trust the absolute encoder, do NOT drive to
        // the home stop. The rotor stays where the previous cycle left it
        // (its last due bucket); the service sweep below servos directly from
        // there. Count this cycle toward the cadence.
        g_cyclesSinceCalibration++;
        LOGI("Cycle: skipping calibration (trusted saved cal; %d/%d cycles "
             "since last)\n",
             g_cyclesSinceCalibration, CALIBRATE_EVERY_N_CYCLES);
    }

    // Cycle-start park -- ONLY on a calibrating cycle. calibrateEndstops()
    // already parked at bucket 1, so this gotoBucket(1) is a no-op confirming
    // arrival within tolerance (and catches an abort raised during the home
    // drive). On a non-calibrating cycle we deliberately skip it: re-homing
    // here would undo the travel saving by dragging the arm back to bucket 1
    // before sweeping out again. With the AS5600 the arm's position is known
    // wherever it idles, so the service sweep can start mid-arc.
    if (calibratedThisCycle) {
        bool homeOk = gotoBucket(1);
        if (!homeOk) {
            if (g_abortRequested) {
                say("ABORTED");
                g_abortRequested = false;
                resetGestureAfterCycle();
                g_state = READY;
                cycleEndCleanup(/*stampWatered=*/true);
                renderUI(true);
                return false;
            }
            // gotoBucket(1) failed. It may have already faulted
            // FAULT_ENCODER_LOST, or simply returned false because the
            // rotor wouldn't reach. Snap to READY when no fault was
            // entered so the UI doesn't stick on the watering banner.
            say("HOME FAIL");
            cycleEndCleanup(/*stampWatered=*/false);
            if (g_state != ERROR) g_state = READY;
            renderUI(true);
            return false;
        }
    }

    // Pre-cycle baseline + reservoir guard. Sample fresh after the
    // home settle so the reading reflects the actual reservoir mass
    // before any pours, then compare to the sum of all bucket doses.
    // The 1.05x margin is small on purpose -- enough to refuse
    // cycles that obviously can't complete (e.g. user removed the
    // tank between cycles), without false-faulting on cell noise.
    float totalDoseMl = 0;
    for (int i = 0; i < g_bucketCount; i++) {
        totalDoseMl += (float)g_buckets[i].dose_ml;
    }
    float g0_at_cycle_start = NAN;
    if (!g_testModeActive) {
        g0_at_cycle_start = readScaleAvg(5);
        if (!isnan(g0_at_cycle_start)) {
            g_reservoirMl = g0_at_cycle_start / G_PER_ML;
            if (g_reservoirMl < totalDoseMl * 1.05f) {
                // Reservoir below the sum of all doses (1.05x margin):
                // DEFER rather than fault. The next cycle re-reads the
                // cell; if the tank's been refilled, dispensing
                // proceeds normally with no acknowledgement needed.
                noteDeferral(DEFER_RESERVOIR_LOW, "reservoir below total dose");
                cycleEndCleanup(/*stampWatered=*/false);
                g_state = READY;
                renderUI(true);
                return false;
            }
        } else {
            // Scale unreadable: skip the pre-cycle guard rather than
            // fault, since the per-bucket safety timeout still bounds
            // worst-case over-pour. The dispense loop already handles
            // missing-scale per-bucket.
            LOGI("%s\n", "Pre-cycle: scale unreadable; skipping guard");
        }
    }

    bool      aborted             = false;
    // Sum of doses for the buckets ACTUALLY serviced this cycle, tallied as
    // each one finishes pouring. The post-cycle reservoir-loss sanity check
    // compares the measured weight drop against THIS -- not totalDoseMl (the
    // sum of EVERY configured bucket). Comparing one bucket's pour against
    // the whole carousel's dose is what produced the false "expected 380g
    // -- check tubing/cell" warning on a single-bucket water (~6x high).
    float     servicedDoseMl      = 0.0f;
    int       dispensedCount      = 0;
    // Set when the due-check drops an otherwise-eligible bucket, i.e. the
    // cycle serviced only the due subset. Marks the cycle "partial" so the
    // post-cycle loss check is skipped (same as it's skipped on deferred):
    // a partial cycle's loss legitimately falls short of the full dose sum.
    bool      sawNotDue           = false;
    // Set when a water-safety condition (empty reservoir mid-cycle,
    // rig moved between buckets) defers the rest of the cycle. Takes
    // the same clean exit as a normal completion -- park, return to
    // READY, no fault -- but leaves the deferral state set so the LED
    // / UI reflect it and the scheduler retries next tick. Distinct
    // from pendingFault (which still latches into ERROR).
    bool      deferred            = false;
    int       consecutiveTimeouts = 0;
    // Count buckets that gotoBucket reported unreachable this cycle --
    // counted among the buckets we ATTEMPT, i.e. the due service list
    // (we no longer servo to skip/not-due buckets, so they can't fault
    // the cycle). One is tolerated as a transient (a single stale-read
    // cluster, a brief mechanical catch) and stays a skip; two or more
    // means the rotor genuinely can't traverse, so the cycle latches
    // FAULT_ROTOR_STALL rather than silently skipping its way to a
    // "success".
    int       unreachableCount    = 0;
    FaultCode pendingFault        = FAULT_NONE;
    int       pendingFaultBucket  = -1;

    // ---- Service list: decide which buckets to visit BEFORE moving. ----
    // The AS5600 is absolute and gotoBucket(n) servos directly to any
    // target (g_currentBucket is set to that absolute target on arrival,
    // never dead-reckoned from sequential visits), so the rotor no longer
    // has to step through every slot "to keep the position model
    // consistent." We build the set of buckets to dispense up front and
    // hop straight to each, collapsing the old full out-sweep across all
    // 13 slots into a single monotonic pass over just the due ones.
    //
    // The three predicates below mirror the OLD in-loop gates EXACTLY, in
    // the same precedence, so no plant is newly watered or newly skipped:
    //   (1) single-bucket mode  -> only the named bucket is a candidate
    //   (2) schedule/due        -> bypassed by test mode, force-all, or the
    //                              named bucket in single-bucket mode;
    //                              otherwise the interval-elapsed check
    //   (3) skip-dose sentinel  -> dose == DOSE_SKIP is never dispensed
    int serviceList[BUCKET_COUNT];
    int serviceCount = 0;
    for (int i = 0; i < g_bucketCount; i++) {
        // (1) single-bucket mode dispenses only the named bucket.
        if (g_webRequestSingle && g_webRequestSingleN != i + 1) continue;
        // (1b) demo Random restricts the cycle to a pre-rolled subset of
        // 1..3 buckets (chosen by runDemoLoop). Same "narrow the candidate
        // set" role as single-bucket mode; due-checking is bypassed below
        // via g_testModeActive, so the chosen buckets always water.
        if (g_demoRandomActive && !(g_demoRandomMask & (uint16_t)(1u << i))) continue;
        // (2) schedule/due. The bypass condition is identical to the old
        // in-loop guard: test mode, force-all, or the named bucket in
        // single-bucket mode all skip the interval check (always "due").
        bool dueBypass = g_testModeActive || g_webForceAll
                       || (g_webRequestSingle && g_webRequestSingleN == i + 1);
        if (!dueBypass) {
            uint32_t now         = scheduleTimeNow();
            uint32_t intervalSec = intervalToSec(g_buckets[i].interval_hours);
            uint32_t elapsed     = now - g_lastWateredAt[i];
            if (elapsed < intervalSec) {
                uint32_t hLeft = (intervalSec - elapsed) / 3600UL;
                LOG("Bucket %d: not due (%luh remaining, interval=%uh)\n",
                              i + 1,
                              (unsigned long)hLeft,
                              (unsigned)g_buckets[i].interval_hours);
                sawNotDue = true;   // cycle services only the due subset
                continue;
            }
        }
        // (3) skip-dose sentinel: never dispensed. The rotor used to park
        // here (and, in test mode, pause 500ms) purely so the position
        // model stayed consistent; with a direct servo we simply don't
        // visit it.
        if (g_buckets[i].dose_ml == DOSE_SKIP) {
            LOGI("Bucket %d: skipped (dose=0)\n",
                          i + 1);
            continue;
        }
        serviceList[serviceCount++] = i;
    }

    // Monotonic, no-backtrack order. Sort the service list by arc position
    // from the home stop (arcPositionFromHome of each bucket's target
    // angle). bucketTargetAngleDeg(n) is BUCKET_ENDSTOP_OFFSET_DEG +
    // (n-1)*BUCKET_SLOT_DEG from home, so arc position already rises with
    // bucket index; this insertion sort just makes the "one direction
    // along the arc" invariant explicit (and robust to any future
    // non-monotonic layout). On a calibrating cycle the arm is parked at
    // bucket 1 (home, lowest arc) by the cycle-start gotoBucket(1) above, so
    // ascending arc == sweeping away from home with no reversal. On a
    // non-calibrating cycle the arm starts at the previous cycle's last due
    // bucket (mid-arc): the sweep still visits due buckets in ascending-arc
    // order -- gotoBucket() is an absolute servo so dispensing is correct
    // from any start position, and only the first hop may step back to the
    // lowest due bucket. That initial step-back is never longer than first
    // returning to home would be, so total travel stays <= the old home-first
    // sweep. An empty list dispenses nothing and falls straight through to
    // the end-of-cycle handling below -- no per-bucket motion at all.
    for (int a = 1; a < serviceCount; a++) {
        int   key    = serviceList[a];
        float keyArc = arcPositionFromHome(bucketTargetAngleDeg(key + 1));
        int   b      = a - 1;
        while (b >= 0 &&
               arcPositionFromHome(bucketTargetAngleDeg(serviceList[b] + 1)) > keyArc) {
            serviceList[b + 1] = serviceList[b];
            b--;
        }
        serviceList[b + 1] = key;
    }

    for (int s = 0; s < serviceCount; s++) {
        int i = serviceList[s];
        g_activeBucket            = i + 1;
        g_cycleBucketIdx          = i + 1;
        g_cycleBucketTotal        = g_bucketCount;
        g_activeBucketDeliveredMl = 0.0f;
        g_activeBucketTargetMl    = g_buckets[i].dose_ml;

        renderWateringFocused();

        // gotoBucket return false has three meanings:
        //   (a) user abort -- whole cycle aborts
        //   (b) encoder fault (FAULT_ENCODER_LOST) -- g_state == ERROR,
        //       whole cycle aborts
        //   (c) MAX_ITER reached / endstops not calibrated -- skip
        //       this bucket and continue
        if (!gotoBucket(i + 1)) {
            if (g_abortRequested) { aborted = true; break; }
            if (g_state == ERROR) {
                LOGI("Bucket %d gotoBucket FAIL (system fault)\n",
                     i + 1);
                cycleEndCleanup(/*stampWatered=*/false);
                return false;
            }
            // Self-heal: a missed target on a cycle that SKIPPED calibration
            // may be a silent position loss (saved-cal drift, or a motor skip
            // under load -- a known failure mode on this rig). Force a recal
            // on the next cycle so position is re-established against the home
            // stop, bounding the loss to one cycle instead of waiting out the
            // cadence counter. On a calibrating cycle position was just
            // re-established, so a miss there is mechanical, not drift.
            if (!calibratedThisCycle) g_forceRecalNextCycle = true;
            unreachableCount++;
            if (unreachableCount >= 2) {
                LOGI("Bucket %d: second unreachable bucket this cycle "
                     "(rotor cannot traverse) -- FAULT_ROTOR_STALL. If "
                     "the motor moves one direction only, suspect coil "
                     "wiring -- do NOT toggle motorInv, it cannot fix a "
                     "torque fault and will invalidate calibration.\n",
                     i + 1);
                pendingFault       = FAULT_ROTOR_STALL;
                pendingFaultBucket = i + 1;
                break;
            }
            LOGI("Bucket %d: unreachable, skipping\n",
                 i + 1);
            continue;
        }

        // Single-bucket mode, schedule/due, and skip-dose were all decided
        // when the service list was built above, so by here this bucket is
        // a confirmed dispense target -- the rotor only ever servos to
        // buckets it will actually water.

        float g0 = NAN;
        if (!g_testModeActive) {
            // Settle-gated baseline: the gotoBucket() above just moved the
            // arm, disturbing the cell. Wait for it to settle (valve still
            // closed) before trusting g0, so the pour isn't measured against
            // a still-creeping baseline (the reverse-drift false-trip).
            float g0settled;
            if (!readScaleSettled(&g0settled)) {
                if (g_abortRequested) { aborted = true; break; }
                LOGI("%s\n", "Scale unavailable; skipping dose");
                continue;
            }
            g0 = g0settled;
            // Per-bucket reservoir guard. g0 is fresh on this bucket so
            // it's the most accurate reservoir reading we have. If we'd
            // partially fill and trip the safety timeout, fault here
            // instead so the operator sees which bucket ran out. The
            // 1.1x margin absorbs cell noise and the small mass shift
            // between buckets without false-faulting on the last few mL.
            g_reservoirMl = g0 / G_PER_ML;
            if (g_reservoirMl < g_buckets[i].dose_ml * 1.1f) {
                // Reservoir ran dry partway through the cycle: DEFER
                // rather than fault. Buckets already served this round
                // keep their fresh timestamps; this bucket and the
                // ones after it stay unstamped so the next cycle
                // retries them once the tank is refilled.
                LOGI("Bucket %d reservoir guard: %.1fmL < %umL * 1.1\n",
                     i + 1, g_reservoirMl, g_buckets[i].dose_ml);
                noteDeferral(DEFER_RESERVOIR_LOW,
                             "reservoir emptied mid-cycle");
                deferred = true;
                break;
            }
        }

        // Dispense: open valve, run main loop until weight hit or
        // safety timeout, close valve. Stuck or no-flow valves are
        // caught by the safety timeout plus the consecutive-timeouts
        // counter below (two no-flow events in a row -> FAULT_NO_FLOW).
        // The safety timeout is sized at 2x the expected pour
        // duration (calibrated valve flow rate) so a healthy load
        // cell always hits the target first; the timeout only fires
        // if the cell stops tracking and keeps the cycle from
        // over-pouring forever.
        //
        // In test mode (g_testModeActive): replace the closed-loop
        // scale-driven dispense with a fixed-duration valve-open of
        // (dose_ml/DOSE_STEP_ML) * g_postValveDwellSec seconds, and
        // optionally suppress the valve actuation entirely
        // (g_testModeWet=false). Sets weightHit=true so the no-flow
        // tracking downstream treats every test bucket as success.
        uint32_t openMs;
        uint32_t realPourMs = 0;   // total valve-open across resume attempts (real mode)
        float    delivered = 0;
        bool     weightHit = false;

        if (g_testModeActive) {
            // Place the cast on the operand, not the result, so the
            // intent is clear even if the dose ladder ever picks up
            // values that don't divide cleanly by DOSE_STEP_ML.
            uint32_t doseUnits =
                (uint32_t)g_buckets[i].dose_ml / DOSE_STEP_ML;
            uint32_t valveOpenMs =
                doseUnits * (uint32_t)g_postValveDwellSec * 1000UL;
            LOGI("Bucket %d: TEST %s, valveOpenMs=%lu "
                 "(dose=%umL = %lu units)\n",
                 i + 1, g_testModeWet ? "WET" : "DRY",
                 (unsigned long)valveOpenMs,
                 (unsigned)g_buckets[i].dose_ml,
                 (unsigned long)doseUnits);
            if (g_testModeWet) {
                valveOpen();
            }
            openMs = millis();
            while ((millis() - openMs) < valveOpenMs) {
                uint32_t elapsed = millis() - openMs;
                float frac = (valveOpenMs > 0)
                              ? (float)elapsed / (float)valveOpenMs
                              : 1.0f;
                if (frac > 1.0f) frac = 1.0f;
                g_activeBucketDeliveredMl =
                    (float)g_buckets[i].dose_ml * frac;
                renderWateringFocused();
                if (serviceAbort()) break;
                delay(50);
            }
            if (g_testModeWet) {
                valveClose();
            }
            delivered = (float)g_buckets[i].dose_ml;
            g_activeBucketDeliveredMl = delivered;
            weightHit = true;
        } else {
            // STEP 2/3: closed-loop pour wrapped in a credit-and-complete
            // resume loop with a mid-pour trustworthiness detector. g0 and
            // target stay FIXED across a resume, so the cutoff measures TOTAL
            // delivered -- a resume pours only the remainder, never a second
            // full dose. safetyTimeout is the CUMULATIVE ceiling across the
            // original pour + the one resume, bounding total valve-open volume
            // per bucket to ~dose+margin so a misbehaving bucket can't empty
            // the tank into its neighbour.
            const float    target = g_buckets[i].dose_ml * G_PER_ML;
            uint32_t safetyTimeout = (uint32_t)((float)g_buckets[i].dose_ml
                                                * DOSE_SEC_PER_ML
                                                * DOSE_TIMEOUT_MARGIN
                                                * 1000.0f);
            uint32_t       cumOpenMs     = 0;          // total valve-open across attempts
            float          worstRatio    = INFINITY;   // min delivered/expected seen
            int            resumeCount   = 0;
            bool           suspectSkip   = false;
            bool           suspectMotion = false;
            // No-flow pulse-priming state (per bucket, spans resumes). Once
            // flow is seen we never prime again; primeAttempts bounds how
            // many prime bursts we try before letting the flow-stall detector
            // escalate to a real FAULT_NO_FLOW.
            int            primeAttempts = 0;
            bool           flowSeen      = false;
            DispenseResult dr            = DISP_DONE;
            for (;;) {   // resume loop: original pour + up to DISP_MAX_RESUMES
            valveOpen();
            openMs = millis();

            // Dual-path filtering for noisy HX711 readings:
            //   - filtered (EMA, alpha=0.3): drives the UI globals so
            //     the live mL counter and reservoir bar animate
            //     smoothly. EMA reaches ~90% of a step within ~7
            //     samples (~700ms at HX711's 10Hz).
            //   - rawWindow[N]: rolling buffer of raw deltas. Cutoff
            //     fires only when ALL N most recent raw samples sit in
            //     a PLAUSIBLE band: above (target - tolerance) AND
            //     below CUTOFF_SAMPLE_MAX_G. The upper bound is the
            //     key fix for the prior "delivered=239mL on a 10mL
            //     target" symptom -- a single huge downward glitch on
            //     the cell (gRaw spikes very low, rawDelivered jumps
            //     to a huge positive) would otherwise satisfy "above
            //     threshold" for several consecutive samples and
            //     declare success with wildly inflated delivered.
            //
            // A separate streak counter watches for the reverse case:
            // cell reading drifts UP (rawDelivered goes negative).
            // Real dispense never does this; persistent upward drift
            // (EMI, broken cell, reservoir lifted, etc.) would
            // otherwise prevent the cutoff from ever firing and the
            // dispense would burn the full safety timeout. After
            // REVERSE_FAULT_SAMPLES consecutive reversed samples we
            // fault NO_FLOW so the cycle stops cleanly.
            const float SCALE_FILTER_ALPHA   = 0.3f;
            const int   SCALE_CUTOFF_SAMPLES = 5;
            // Cap how high a single raw sample can be and still count
            // toward the cutoff. 3x target gives comfortable headroom
            // for real overshoot at the moment of cutoff while
            // rejecting noise spikes that are physically impossible
            // (rig flows ~0.4mL/s; 30g in one ~100ms sample = 300mL/s).
            const float CUTOFF_SAMPLE_MAX_G  =
                (float)g_buckets[i].dose_ml * 3.0f * G_PER_ML;
            const int   REVERSE_FAULT_SAMPLES = 5;
            const float REVERSE_THRESHOLD_G   = 2.0f * DOSE_TOLERANCE_G;
            float       gFiltered = NAN;        // EMA state
            float       rawWindow[SCALE_CUTOFF_SAMPLES];
            int         rawWindowFill = 0;       // grows to SCALE_CUTOFF_SAMPLES, then stays
            int         rawWindowHead = 0;       // ring index
            int         reverseStreak = 0;
            uint32_t    stallSinceMs   = 0;   // when delivered first fell below the stall floor
            uint32_t    motionSinceMs  = 0;   // when the rig first read "moving" mid-pour
            // Reference for the reverse-drift grace and the no-flow check
            // window. Set at each valve-open and reset after a prime burst,
            // so a freshly-disturbed cell gets DISP_REVERSE_GRACE_MS to
            // settle and the no-flow window restarts after each prime.
            uint32_t    settleRefMs    = openMs;
            for (int k = 0; k < SCALE_CUTOFF_SAMPLES; k++) rawWindow[k] = 0.0f;

            for (;;) {
                float gRaw = weightFromRaw();
                float rawDelivered = NAN;
                if (!isnan(gRaw)) {
                    // EMA update for the UI path. First sample seeds
                    // the filter so the bar doesn't ramp from zero.
                    if (isnan(gFiltered)) gFiltered = gRaw;
                    else gFiltered += SCALE_FILTER_ALPHA * (gRaw - gFiltered);

                    // Rolling raw-delta window for the cutoff path.
                    rawDelivered = g0 - gRaw;
                    rawWindow[rawWindowHead] = rawDelivered;
                    rawWindowHead = (rawWindowHead + 1) % SCALE_CUTOFF_SAMPLES;
                    if (rawWindowFill < SCALE_CUTOFF_SAMPLES) rawWindowFill++;

                    // Reverse-flow tracking: cell mass rose vs baseline.
                    if (rawDelivered < -REVERSE_THRESHOLD_G) reverseStreak++;
                    else                                     reverseStreak = 0;
                }

                // UI path: filtered values drive the live displays so
                // the operator sees a stable, monotonically-rising
                // delivered count and a smoothly-decreasing reservoir.
                float gShown = isnan(gFiltered) ? gRaw : gFiltered;
                delivered = g0 - gShown;
                g_activeBucketDeliveredMl = delivered;
                if (!isnan(gShown)) g_reservoirMl = gShown / G_PER_ML;
                renderWateringFocused();   // rate-limited internally

                // Cutoff path: require all N samples in the plausible
                // band [target - tol, CUTOFF_SAMPLE_MAX_G]. The upper
                // bound is essential -- without it a 200g+ noise spike
                // trivially satisfies the "above threshold" check.
                if (rawWindowFill >= SCALE_CUTOFF_SAMPLES) {
                    bool allOver = true;
                    for (int k = 0; k < SCALE_CUTOFF_SAMPLES; k++) {
                        float v = rawWindow[k];
                        if (v < target - DOSE_TOLERANCE_G
                            || v > CUTOFF_SAMPLE_MAX_G) {
                            allOver = false;
                            break;
                        }
                    }
                    if (allOver) {
                        weightHit = true;
                        dr        = DISP_DONE;
                        // Dump the window that satisfied the cutoff
                        // so a glitch-driven false positive shows up
                        // in the log instead of just "delivered=X.YmL".
                        String wstr;
                        for (int k = 0; k < SCALE_CUTOFF_SAMPLES; k++) {
                            if (k) wstr += ',';
                            wstr += String(rawWindow[k], 1);
                        }
                        LOGI("Bucket %d cutoff: target=%.1fg cap=%.1fg "
                             "samples=[%s]g\n",
                             i + 1, (double)target,
                             (double)CUTOFF_SAMPLE_MAX_G, wstr.c_str());
                        break;
                    }
                }

                // Flow establishment + no-flow pulse-priming (hydraulic
                // recovery). Operator-confirmed: the valve opens but no
                // water flows (line stiction / airlock) until it's stopped
                // and restarted. So once we've had NOFLOW_CHECK_MS of open
                // valve with no measurable weight drop, pulse the valve a
                // few times to break the stiction and prime the line, then
                // keep pouring against the SAME g0 (nothing left during the
                // stuck period, so the baseline is still valid). This is a
                // retry BEFORE the fault: while priming is still available
                // the flow-stall / reverse detectors below are held off, so
                // priming owns the decision; once flow is seen or the prime
                // budget is spent, the normal detectors resume and a genuine
                // dry line still escalates to FAULT_NO_FLOW as before.
                if (!flowSeen && delivered > NOFLOW_MIN_FLOW_G) {
                    flowSeen = true;   // water is leaving -- priming not needed
                }
                if (!flowSeen && primeAttempts < NOFLOW_PRIME_MAX_ATTEMPTS
                    && (uint32_t)(millis() - settleRefMs) >= NOFLOW_CHECK_MS) {
                    primeAttempts++;
                    LOGI("Bucket %d: no flow %dms after valve open "
                         "(delivered=%.1f/%.0f); pulsing valve to prime "
                         "(attempt %d/%d)\n",
                         i + 1, (int)NOFLOW_CHECK_MS, (double)delivered,
                         (double)target, primeAttempts,
                         (int)NOFLOW_PRIME_MAX_ATTEMPTS);
                    bool primeAborted = false;
                    for (int p = 0; p < NOFLOW_PRIME_PULSES && !primeAborted; p++) {
                        valveClose();
                        uint32_t t = millis();
                        while ((uint32_t)(millis() - t) < NOFLOW_PRIME_CLOSED_MS) {
                            if (serviceAbort()) { primeAborted = true; break; }
                            renderWateringFocused();
                            delay(20);
                        }
                        if (primeAborted) break;
                        valveOpen();
                        t = millis();
                        while ((uint32_t)(millis() - t) < NOFLOW_PRIME_OPEN_MS) {
                            if (serviceAbort()) { primeAborted = true; break; }
                            renderWateringFocused();
                            delay(20);
                        }
                    }
                    if (primeAborted) { dr = DISP_ABORT; break; }
                    // Valve left OPEN. Restart the no-flow window + reverse
                    // grace and drop stale samples so the cutoff/reverse
                    // detectors don't act on pre-pulse data. Pulse time is
                    // already counted in cumElapsed (openMs is unchanged), so
                    // priming still counts against the safety timeout.
                    settleRefMs   = millis();
                    reverseStreak = 0;
                    rawWindowFill = 0;
                    stallSinceMs  = 0;
                    motionSinceMs = 0;
                    continue;
                }
                // primingResolved: flow seen, or the prime budget is spent.
                // Until then the reverse/stall escalations below are held off.
                bool primingResolved =
                    flowSeen || primeAttempts >= NOFLOW_PRIME_MAX_ATTEMPTS;

                // Reverse drift: cell mass rose for too many samples in a
                // row (EMI, reservoir lifted, preload shift, or unsettled
                // creep after the arm move / a prime pulse). The cutoff would
                // never fire. Reclassify as a recoverable SUSPECT (STEP 3) --
                // shut the valve early, then auto-recover once if the signal
                // goes sane, else flag+skip. Gated on primingResolved and a
                // post-disturbance grace so transient settling creep can't
                // false-trip it.
                if (primingResolved
                    && (uint32_t)(millis() - settleRefMs) >= DISP_REVERSE_GRACE_MS
                    && reverseStreak >= REVERSE_FAULT_SAMPLES) {
                    LOGI("Bucket %d: cell reading ROSE for %d consecutive "
                         "samples (last rawDelta=%+.1fg, delivered=%+.1fg); "
                         "measurement SUSPECT (reverse drift)\n",
                         i + 1, reverseStreak,
                         (double)(isnan(rawDelivered) ? 0.0f : rawDelivered),
                         (double)delivered);
                    dr = DISP_SUSPECT; suspectMotion = false;
                    break;
                }

                // Mid-pour motion: keep the IMU ring live during the
                // otherwise-blocking pour so motionOk() is current, then
                // treat a moving rig as a recoverable suspect -- the load
                // cell can't be trusted while the rig shakes. imuTick()
                // self-throttles, so calling it every iteration is cheap.
                imuTick();
                if (g_imuPresent && !motionOk()) {
                    // Debounce: only SUSPECT on SUSTAINED motion, so normal
                    // pour/valve vibration (a brief over-threshold blip) can't
                    // wrongly defer a healthy bucket.
                    if (motionSinceMs == 0) {
                        motionSinceMs = millis();
                    } else if ((uint32_t)(millis() - motionSinceMs)
                               >= DISP_MOTION_TRIP_MS) {
                        LOGI("Bucket %d: rig MOVING mid-pour (metric=%.3fg > "
                             "%.3fg for >=%dms); measurement SUSPECT (motion)\n",
                             i + 1, (double)g_motionMetric,
                             (double)g_motionThreshG, (int)DISP_MOTION_TRIP_MS);
                        dr = DISP_SUSPECT; suspectMotion = true;
                        break;
                    }
                } else {
                    motionSinceMs = 0;
                }

                // Flow-stall (the no-flow / cell-reads-low backstop): after a
                // grace, 'delivered' must reach at least a conservative
                // fraction of expected-by-now (using the deliberately slow
                // DOSE_SEC_PER_ML rate, ~14x below real flow, so this only
                // fires on near-zero delivery and never on a slow-but-valid
                // pour). Cumulative open time so a resume's expectation
                // continues from the original.
                uint32_t cumElapsed = cumOpenMs + (uint32_t)(millis() - openMs);
                if (primingResolved && cumElapsed > DISP_STALL_GRACE_MS) {
                    float expMl = (float)(cumElapsed - DISP_STALL_GRACE_MS)
                                  / 1000.0f / DOSE_SEC_PER_ML;
                    if (expMl > 1.0f) {
                        float ratio = delivered / expMl;
                        if (ratio < worstRatio) worstRatio = ratio;
                        if (ratio < DISP_STALL_MIN_FRACTION) {
                            if (stallSinceMs == 0) {
                                stallSinceMs = millis();
                            } else if ((uint32_t)(millis() - stallSinceMs)
                                       >= DISP_STALL_TRIP_MS) {
                                LOGI("Bucket %d: flow STALL -- delivered=%.1f "
                                     "expected>=%.1f ratio=%.2f < %.2f for "
                                     ">=%dms; measurement SUSPECT (no-flow/"
                                     "low-read)\n",
                                     i + 1, (double)delivered, (double)expMl,
                                     (double)ratio,
                                     (double)DISP_STALL_MIN_FRACTION,
                                     (int)DISP_STALL_TRIP_MS);
                                dr = DISP_SUSPECT; suspectMotion = false;
                                break;
                            }
                        } else {
                            stallSinceMs = 0;
                        }
                    }
                }

                if (cumElapsed >= safetyTimeout) { dr = DISP_TIMEOUT; break; }
                if (serviceAbort()) { dr = DISP_ABORT; break; }
                delay(20);
            }   // end sample loop
            valveClose();
            cumOpenMs += (uint32_t)(millis() - openMs);
            if (dr != DISP_SUSPECT) break;   // DONE / ABORT / TIMEOUT -> leave resume loop

            // SUSPECT: valve already shut. Resume at most DISP_MAX_RESUMES.
            if (resumeCount >= DISP_MAX_RESUMES) { suspectSkip = true; break; }
            g_suspectActive = true;
            renderWateringFocused();
            LOGI("Bucket %d: SUSPECT pause (%s) -- valve shut, delivered=%.1f/"
                 "%.0f, awaiting trustworthy cell\n",
                 i + 1, suspectMotion ? "motion" : "measurement",
                 (double)delivered, (double)target);
            if (!waitCellSane(g0, delivered)) {
                // user stop during the pause -> abort; else not recoverable -> skip
                if (g_abortRequested) dr = DISP_ABORT;
                else                  suspectSkip = true;
                break;
            }
            if (cumOpenMs >= safetyTimeout) { suspectSkip = true; break; }
            resumeCount++;
            g_suspectActive = false;
            LOGI("Bucket %d: cell trustworthy again; RESUMING (credited=%.1f, "
                 "target=%.0f, budget left=%lums)\n",
                 i + 1, (double)delivered, (double)target,
                 (unsigned long)(safetyTimeout - cumOpenMs));
            }   // end resume loop
            g_suspectActive = false;
            realPourMs = cumOpenMs;
            // Bench-validation telemetry: how close the pour came to the
            // stall floor (worstRatio; -1 = pour finished before grace).
            LOGI("Bucket %d: flow-margin worstRatio=%.2f (SUSPECT below %.2f)\n",
                 i + 1, (double)(isinf(worstRatio) ? -1.0f : worstRatio),
                 (double)DISP_STALL_MIN_FRACTION);

            // Disposition of an unrecovered SUSPECT. Motion -> DEFER
            // (transient/environmental; no fault escalation, retried next
            // cycle). Stall/reverse -> FLAG+SKIP this bucket and feed the
            // no-flow escalation so a persistently untrustworthy bucket
            // eventually latches FAULT_NO_FLOW.
            if (suspectSkip) {
                if (suspectMotion) {
                    LOGI("Bucket %d: rig did not settle; DEFER (unstamped, "
                         "retry next cycle)\n", i + 1);
                    noteDeferral(DEFER_MOVING, "rig moving during dispense");
                    deferred = true;
                    stampAllLastWateredExcept(i);
                    break;   // exit bucket loop on a defer
                }
                LOGI("Bucket %d: measurement untrustworthy after %d resume; "
                     "FLAG+SKIP (unstamped)\n", i + 1, resumeCount);
                g_lastSuspectBucket = i + 1;
                g_suspectSkipCount++;
                consecutiveTimeouts++;   // escalate like a no-flow event
                if (consecutiveTimeouts >= 2) {
                    LOGI("Two untrustworthy/no-flow dispenses; FAULT_NO_FLOW "
                         "bucket %d\n", i + 1);
                    pendingFault       = FAULT_NO_FLOW;
                    pendingFaultBucket = i + 1;
                    break;
                }
                continue;   // skip dwell/stamp/tally; next bucket
            }
        }
        const uint32_t pourMs = g_testModeActive ? (uint32_t)(millis() - openMs)
                                                 : realPourMs;

        // Post-valve-close dwell. The solenoid shuts instantly but
        // the supply tubing still holds water under gravity (and
        // residual line pressure) that keeps draining for a few
        // seconds afterward. If we advance the rotor immediately,
        // that tail volume lands on whichever bucket the rotor is
        // crossing next, causing visible overflow between buckets
        // and stealing dose from the bucket we just served. Wait
        // here -- with the rotor still parked under the current
        // bucket -- so the tail finishes draining where it
        // belongs. Keep servicing the abort path and the live UI:
        // the load cell continues to track during the wait, which
        // is exactly what we want for the post-cycle reservoir-
        // loss sanity check below (gFinal reads the steady-state
        // mass after all in-flight water has landed). ~100ms tick
        // granularity matches the cadence used elsewhere in this
        // file for in-loop UI updates.
        //
        // Only dwell when water actually flowed: real dispense or a
        // wet test. In dry test mode the valve never opens, so there
        // is no water in the supply tubing to finish draining and
        // nothing to wait for -- skipping the dwell makes a dry test
        // cycle g_postValveDwellSec shorter per bucket than a wet one.
        if (!g_testModeActive || g_testModeWet) {
            uint32_t dwellMs    = (uint32_t)g_postValveDwellSec * 1000UL;
            uint32_t dwellStart = millis();
            while ((millis() - dwellStart) < dwellMs) {
                if (serviceAbort()) break;
                if (!g_testModeActive && scaleOK) {
                    float gNow = weightFromRaw();
                    if (!isnan(gNow)) {
                        g_reservoirMl = gNow / G_PER_ML;
                        float deliveredNow = g0 - gNow;
                        if (deliveredNow > delivered) {
                            delivered = deliveredNow;
                            g_activeBucketDeliveredMl = delivered;
                        }
                    }
                }
                renderWateringFocused();
                delay(100);
            }
        }

        if (g_abortRequested) {
            LOGI("Bucket %d: aborted mid-dispense "
                          "(delivered=%.1fmL t=%lums)\n",
                          i + 1, delivered, (unsigned long)pourMs);
            aborted = true;
            break;
        }

        // Mid-cycle motion watchdog. Evaluated here so the dwell just
        // gave the IMU ring a few seconds of quiet data, and BEFORE we
        // credit this bucket's g_lastWateredAt. A dry test skips it --
        // motion is irrelevant to a no-water run.
        if (!dryTest && g_imuPresent && !motionOk()) {
            // Rig moved during this dispense: the load-cell reading is
            // suspect, so DEFER rather than fault. Leave this bucket's
            // timestamp unstamped (the next cycle retries it); every
            // other non-skip bucket is stamped to avoid a cycle storm.
            LOGI("Bucket %d: motion mid-cycle: metric=%.3fg thresh=%.3fg\n",
                 i + 1, (double)g_motionMetric, (double)g_motionThreshG);
            noteDeferral(DEFER_MOVING, "rig moved between buckets");
            deferred = true;
            stampAllLastWateredExcept(i);   // i is 0-indexed
            break;
        }

        float gAfter = g_testModeActive ? NAN : readScaleAvg(3);
        if (!isnan(gAfter)) g_reservoirMl = gAfter / G_PER_ML;

        LOGI("Bucket %d: target=%umL delivered=%.1fmL "
                      "t=%lums%s reservoir=%.0fmL\n",
                      i + 1, g_buckets[i].dose_ml, delivered,
                      (unsigned long)pourMs,
                      weightHit ? "" : " [safety-timeout]",
                      g_reservoirMl);
        // Tally the dose we committed to pouring for this bucket. Reached
        // only after a completed dispense -- the abort and motion-defer
        // breaks above exit before here, so a partially-poured bucket is
        // never counted. This is the basis for the post-cycle loss check.
        servicedDoseMl += (float)g_buckets[i].dose_ml;
        dispensedCount++;
        // Test runs (especially dry) must not move the schedule
        // forward -- doing so would suppress the next real cycle.
        if (!g_testModeActive) {
            g_lastWateredAt[i] = scheduleTimeNow();
            persistMarkLastWatered(i);
        }

        // The load cell never tracked any flow AND we hit the safety
        // timeout: either reservoir is dry, valve failed, or load
        // cell isn't seeing the reservoir. Two in a row faults so we
        // don't keep tripping the safety on every bucket.
        if (!weightHit && delivered < DOSE_FLOW_MIN_G) {
            consecutiveTimeouts++;
            if (consecutiveTimeouts >= 2) {
                LOGI("Two no-flow dispenses; "
                              "FAULT_NO_FLOW bucket %d\n",
                              i + 1);
                pendingFault       = FAULT_NO_FLOW;
                pendingFaultBucket = i + 1;
                break;
            }
        } else {
            consecutiveTimeouts = 0;
        }
        // If the dispense loop itself set a pending fault (reverse-
        // flow detector tripped mid-pour), stop the cycle now. The
        // outer pendingFault handler will enterFault and bail.
        if (pendingFault != FAULT_NONE) break;
    }

    // Belt-and-suspenders: never leave the valve open on any exit path.
    valveClose();

    const bool userAborted = g_abortRequested;
    if (userAborted) {
        say("ABORTED");
        // Clear the request so the park-to-bucket-1 below can run.
        g_abortRequested = false;
        stampAllLastWatered();
        // resetGestureAfterCycle() is deliberately deferred until AFTER
        // the park (see below). The button is most likely still being
        // mashed during the park -- those edges are dropped on the floor
        // by serviceAbort's suppress branch as they arrive. Resetting the
        // gesture machine here would only clear edges from before suppress
        // activated; a stale g_pressStartMs captured just before suppress,
        // plus any edge that lands mid-park, would survive to fire a
        // spurious gesture once suppress ends. Reset once, at the very
        // end, when the rotor is back at a known position.
        // Park at bucket 1 so the user always ends up in a known
        // resting position rather than wherever the cycle was
        // interrupted. The park itself is UNINTERRUPTIBLE -- without
        // suppress mode, a still-held button or a second /api/abort
        // tap would re-set g_abortRequested, stepN would return false,
        // and the rotor would stop mid-travel. gotoBucket's own
        // bounded iteration count still terminates the park if the
        // rotor is genuinely stuck.
        if (g_endstopsCalibrated) {
            LOGI("%s\n", "Abort: parking at bucket 1 (uninterruptible)");
            g_suppressAbort  = true;
            g_abortRequested = false;
            bool parkOk = gotoBucket(1);
            g_suppressAbort  = false;
            g_abortRequested = false;
            if (!parkOk) {
                LOGI("%s\n", "Abort: park to bucket 1 did not complete "
                             "(rotor may be mid-travel)");
            }
        }
        // Now that the (uninterruptible) park is done and the rotor is at
        // a known position, drain the gesture machine. This clears any
        // button edges that arrived during suppress mode -- serviceAbort
        // already dropped the abort itself, but this also wipes a stale
        // g_pressStartMs and the multi-press tally -- so the next
        // updateGestures() tick starts clean instead of acting on a
        // still-held button from the abort.
        resetGestureAfterCycle();
    }

    // Fault path: enterFault() shuts the valve, homes best-effort,
    // and transitions to ERROR with banner. User-abort takes
    // precedence over a pending fault -- explicit user action
    // always lands back in READY rather than ERROR, since the user
    // already knows what they did.
    if (!userAborted && pendingFault != FAULT_NONE) {
        cycleEndCleanup(/*stampWatered=*/false);
        enterFault(pendingFault, pendingFaultBucket);
        return false;
    }

    // End-of-cycle park at bucket 1 -- ONLY on a calibrating cycle. If the
    // user already aborted, the abort-path above already parked us; skip this.
    //
    // On a calibrating cycle we park (servo back to bucket 1 from wherever the
    // cycle ended -- fast with the encoder, no full-revolution sweep), keeping
    // that path identical to the old every-cycle behavior. On a NON-calibrating
    // cycle we deliberately leave the arm at its last due bucket: parking home
    // here would re-incur the return trip we set out to eliminate, and the next
    // cycle's service sweep servos directly from this idle position (or
    // recalibrates and re-homes first). This skipped return is the actual
    // carousel-travel saving.
    if (!userAborted && calibratedThisCycle) {
        bool parkOk = gotoBucket(1);
        if (!parkOk) {
            say("HOME FAIL");
            cycleEndCleanup(/*stampWatered=*/false);
            // Same state-leak fix as the cycle-start exit: if no
            // fault was entered (e.g. gotoBucket(1) returned false
            // for "unreachable"), snap to READY so the UI clears
            // the watering banner.
            if (g_state != ERROR) g_state = READY;
            renderUI(true);
            return false;
        }
    }

    // Post-cycle reservoir sanity. If the cell shows much less weight loss
    // than the doses we actually poured, something is off (tubing routing
    // not over the bucket, reservoir not on the cell, etc.). Per-bucket
    // guards already catch hard failures, so this is strictly informational.
    //
    // Two conditions make the check meaningful, and both were previously
    // wrong:
    //   - "expected" must be the sum of doses ACTUALLY serviced this cycle
    //     (servicedDoseMl), NOT totalDoseMl (every configured bucket). A
    //     single-bucket water poured ~60mL but "expected" the whole
    //     carousel's ~380mL and false-warned ~6x high.
    //   - the check only holds for a FULL service. On a partial cycle
    //     (single-bucket water, demo random subset, only-the-due subset, or
    //     a user abort) the loss legitimately falls short of any dose sum,
    //     so skip it -- same rationale as skipping on a mid-cycle defer.
    //
    // Skip too if we never got a baseline (scale unreadable at cycle start)
    // or nothing was dispensed.
    bool partialService = g_webRequestSingle || g_demoRandomActive
                       || sawNotDue || userAborted;
    if (!g_testModeActive && !deferred && !partialService
        && !isnan(g0_at_cycle_start) && dispensedCount > 0) {
        float gFinal = readScaleAvg(3);
        if (!isnan(gFinal)) {
            float lossG     = g0_at_cycle_start - gFinal;
            float expectedG = servicedDoseMl * G_PER_ML;
            if (lossG < 0.5f * expectedG) {
                LOGI("WARN: reservoir loss %.1fg vs "
                              "expected %.1fg -- check tubing/cell\n",
                              lossG, expectedG);
            }
        }
    }

    g_lastWaterMs = millis();
    g_nextWaterMs = millis()
                    + (uint32_t)WATER_INTERVAL_HOURS * 3600000UL;
    cycleEndCleanup(/*stampWatered=*/false);
    g_state = READY;
    // A cycle that actually ran to here (clean completion or user
    // abort) clears any prior deferral so the LED / UI return to a
    // healthy state and the next deferral logs fresh. A deferred
    // cycle keeps its reason set: the scheduler will retry next tick
    // and the LED keeps showing the deferral until the condition
    // clears on a subsequent successful cycle.
    if (!deferred) clearDeferralState();
    renderUI(true);
    LOGI("Cycle done%s\n",
         userAborted ? " (user-aborted)"
                     : deferred ? " (deferred)"
                                : aborted ? " (aborted)" : "");
    return !aborted && !deferred;
}

// ---------- UI ----------
//
// Radial dial UI: a faithful miniature of the web UI. A bone reservoir
// "tank" disk with a gravity water fill fills the panel; on top sit 13
// buckets of 6 concentric rounded-cap dose pills (one pill per 10 mL,
// inner lights first), a yellow selection halo just OUTSIDE the band
// stack, a small dark centre disk carrying a single state glyph
// (droplet / dot / pause-bars / warning-triangle / X), and a water
// droplet in the 6-o'clock dead-zone. Each frame is composed off-screen
// into a 128x128 RGB565 sprite and pushed in one operation, so the
// panel never shows a partially-drawn frame.

// ---- radial geometry (all draw math derives from these) ----
static const int   LCD_W       = 128;
static const int   LCD_H       = 128;
static const int   LCD_CX      = 64;
static const int   LCD_CY      = 64;
static const int   LCD_R       = 63;     // dial outer radius (tank rim)
static const int   BAND_R_OUT  = 58;     // outermost bucket ring (halo sits outside)
static const int   BAND_R_IN   = 28;     // innermost bucket ring (~45% of LCD_R, matches web 90/200)
static const int   CENTRE_R    = 18;     // centre disk
static const float SLOT_DEG    = 360.0f / 14.0f;   // 13 buckets + 1 dead-zone
// Bucket width = slot - fixed gap, same recipe as the web dial. At
// BAND_R_IN=28 the inner ring midline is ~30.5px; cap_r=2 so adjacent
// rounded caps need >=4px of arc between centres. 8deg gives ~4.26px
// at the inner ring (~0.26px clearance), matching the web's tuned look.
static const float BUCKET_GAP_DEG = 8.0f;
static const float BUCKET_DEG     = SLOT_DEG - BUCKET_GAP_DEG;   // ~17.7deg
static const float HALF_DEG       = BUCKET_DEG / 2.0f;
static const float FIRST_DEG   = 180.0f + SLOT_DEG;  // bucket 1 centre

// Ring stack: 6 concentric rings spanning BAND_R_IN..BAND_R_OUT, one
// per 10 mL dose step (matches the web dial's 6 bands; no collapse).
static const int   RING_COUNT      = 6;
static const float RING_STEP       = (BAND_R_OUT - BAND_R_IN) / 6.0f;  // 6px
static const float RING_GAP        = 2.0f;   // radial seam between rings

// Tank / dim tones lifted from the web dial. NOT part of the shared
// UI_* palette -- derived here so the LCD matches the web reservoir
// colours without touching the palette defines.
//   LCD_BONE    web #f0e9d8  dial fill (dry tank)
//   LCD_WATER   web #9bc4dd  water fill
//   LCD_DIMBLUE ~50% UI_BLUE     non-active lit ring (watering view)
//   LCD_DIMGREY ~50% UI_DARKGREY non-active unlit ring (watering view)
static const uint16_t LCD_BONE    = 0xF75B;
static const uint16_t LCD_WATER   = 0x9E3B;
static const uint16_t LCD_DIMBLUE = 0x19EB;
static const uint16_t LCD_DIMGREY = 0x2944;

// ---- off-screen frame buffer ----
// Allocated once, lazily on first render. 32KB RGB565 (128*128*2).
// Fits in AtomS3R's PSRAM, also fits in AtomS3's internal SRAM if
// ever back-ported -- the existing UI heap usage leaves plenty of
// room. (PSRAM is not enabled in this build, so the M5Canvas PSRAM
// preference falls back to DMA-capable internal SRAM automatically.)
// NB: parent must be &M5.Display, not &AtomS3.Display. AtomS3.Display is a
// reference member (M5GFX& Display = M5.Display) bound only when AtomS3's
// constructor runs in another TU; this static initializes before that, so
// &AtomS3.Display reads a still-zeroed reference slot and yields a null
// _parent -> LoadProhibited in pushSprite. &M5.Display is a real member of
// the M5 global = a fixed link-time address, immune to init order.
static M5Canvas g_uiCanvas(&M5.Display);
static bool     g_uiCanvasReady = false;

static void ensureUICanvas() {
    static bool tried = false;
    if (g_uiCanvasReady || tried) return;
    tried = true;   // one allocation attempt; on failure fall back to direct
    g_uiCanvas.setColorDepth(16);
    if (g_uiCanvas.createSprite(LCD_W, LCD_H) != nullptr) {
        g_uiCanvasReady = true;
    } else {
        LOG("uiCanvas alloc failed; drawing direct to display\n");
    }
}

// Drawing target: the off-screen sprite when allocated, otherwise the
// live display. The same drawing code works either way; only the
// final present differs (pushSprite vs. the draws landing directly).
static LovyanGFX* uiTarget() {
    return g_uiCanvasReady ? (LovyanGFX*)&g_uiCanvas
                           : (LovyanGFX*)&AtomS3.Display;
}

// ---- radial helpers ----

// (r, deg-clockwise-from-12) -> canvas (x,y). deg 0 = 12 o'clock,
// 90 = 3 o'clock, 180 = 6 o'clock.
static inline void polarToXY(float r, float deg, int* x, int* y) {
    float t = deg * (float)M_PI / 180.0f;
    *x = LCD_CX + (int)(r * sinf(t) + 0.5f);
    *y = LCD_CY - (int)(r * cosf(t) + 0.5f);
}

// Fill an annular sector r_in..r_out over aStart..aEnd (deg) via
// LovyanGFX's native fillArc -- one draw call per band, clean edges
// without the triangle-fan facets. polarToXY uses 0deg = 12 o'clock,
// LGFX fillArc uses 0deg = 3 o'clock; both rotate clockwise, so the
// only conversion is a -90deg shift on each endpoint.
static void drawAnnularSector(LovyanGFX* g, float r_in, float r_out,
                              float aStart, float aEnd, uint16_t colour) {
    int ri = (int)(r_in  + 0.5f);
    int ro = (int)(r_out + 0.5f);
    g->fillArc(LCD_CX, LCD_CY, ro, ri,
               aStart - 90.0f, aEnd - 90.0f, colour);
}

// Thick arc centred on r_mid (used for the selection halo).
static void drawArc(LovyanGFX* g, float r_mid, float aStart, float aEnd,
                    int strokeWidth, uint16_t colour) {
    drawAnnularSector(g, r_mid - strokeWidth / 2.0f,
                         r_mid + strokeWidth / 2.0f,
                         aStart, aEnd, colour);
}

// Rounded-cap ring segment ("pill"): the annular band r_in..r_out over
// [aStart,aEnd] with a semicircular cap at each angular end, so adjacent
// buckets read as separated pills like the web UI's round-linecap arcs.
// The cap radius equals half the band thickness (the cap centre rides
// the band midline), so caps never spill outside the band's radial span.
static void drawRoundedBand(LovyanGFX* g, float r_in, float r_out,
                            float aStart, float aEnd, uint16_t colour) {
    drawAnnularSector(g, r_in, r_out, aStart, aEnd, colour);
    float r_mid = (r_in + r_out) * 0.5f;
    int   cap_r = (int)((r_out - r_in) * 0.5f + 0.5f);
    if (cap_r < 1) cap_r = 1;
    int xs, ys, xe, ye;
    polarToXY(r_mid, aStart, &xs, &ys);
    polarToXY(r_mid, aEnd,   &xe, &ye);
    g->fillCircle(xs, ys, cap_r, colour);
    g->fillCircle(xe, ye, cap_r, colour);
}

// Angular span [aStart,aEnd] of bucket i0 (0-based). Buckets fill
// slots 1..13; slot 0 (180deg, 6 o'clock) is the droplet dead-zone.
static inline void bucketAngles(int i0, float* aStart, float* aEnd) {
    float c = FIRST_DEG + i0 * SLOT_DEG;
    *aStart = c - HALF_DEG;
    *aEnd   = c + HALF_DEG;
}

// One bucket's 6 concentric dose pills (inner lights first). The lit
// count is one pill per 10 mL step (dose 10..60 -> 1..6), no collapse.
// DOSE_SKIP lights none: 6 dim pills read as a present bucket set to
// "no water this cycle", whereas an empty wedge would read as missing.
static void drawBucketRings(LovyanGFX* g, uint16_t doseMl,
                            float aStart, float aEnd,
                            uint16_t litCol, uint16_t unlitCol) {
    int lit;
    if (doseMl == DOSE_SKIP) {
        lit = 0;
    } else {
        lit = (int)doseMl / DOSE_STEP_ML;     // one pill per 10 mL
        if (lit < 1)          lit = 1;
        if (lit > RING_COUNT) lit = RING_COUNT;
    }
    for (int j = 0; j < RING_COUNT; j++) {
        float r_in  = BAND_R_IN + j * RING_STEP;
        float r_out = r_in + RING_STEP - RING_GAP;
        drawRoundedBand(g, r_in, r_out, aStart, aEnd,
                        (j < lit) ? litCol : unlitCol);
    }
}

// Bone dial + gravity water fill, clipped to the dial circle. frac is
// the reservoir fill fraction [0,1]; the water surface sits at
// y = CY + R - frac*2R and a cyan line marks where water meets air.
static void drawTank(LovyanGFX* g, float frac) {
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    g->fillCircle(LCD_CX, LCD_CY, LCD_R, LCD_BONE);
    int yWater = (int)(LCD_CY + LCD_R - frac * 2.0f * LCD_R + 0.5f);
    int yTop   = (yWater < LCD_CY - LCD_R) ? (LCD_CY - LCD_R) : yWater;
    for (int y = yTop; y <= LCD_CY + LCD_R; y++) {
        int dy = y - LCD_CY;
        int hw = (int)(sqrtf((float)(LCD_R * LCD_R - dy * dy)) + 0.5f);
        g->drawFastHLine(LCD_CX - hw, y, 2 * hw + 1, LCD_WATER);
    }
    if (yWater > LCD_CY - LCD_R && yWater < LCD_CY + LCD_R) {
        int dy = yWater - LCD_CY;
        int hw = (int)(sqrtf((float)(LCD_R * LCD_R - dy * dy)) + 0.5f);
        g->drawFastHLine(LCD_CX - hw, yWater, 2 * hw + 1, UI_CYAN);
    }
}

// Centre disk + two horizontally-centred status lines (size-1 font,
// 6px/char). Text uses a transparent background so it reads over the
// disk fill; lines slightly wider than the disk spill onto the bone
// gap between the disk and the inner ring, not onto the rings.
static void drawCentre(LovyanGFX* g, uint16_t diskFill,
                       const char* top, uint16_t topCol,
                       const char* bot, uint16_t botCol) {
    g->fillCircle(LCD_CX, LCD_CY, CENTRE_R, diskFill);
    g->drawCircle(LCD_CX, LCD_CY, CENTRE_R, UI_WHITE);
    g->setTextSize(1);
    if (top && *top) {
        int w = (int)strlen(top) * 6;
        g->setTextColor(topCol);   // transparent bg
        g->setCursor(LCD_CX - w / 2, LCD_CY - 9);
        g->print(top);
    }
    if (bot && *bot) {
        int w = (int)strlen(bot) * 6;
        g->setTextColor(botCol);   // transparent bg
        g->setCursor(LCD_CX - w / 2, LCD_CY + 1);
        g->print(bot);
    }
}

// Teardrop centred on (cx, cy), total height h. A round bulb at the
// bottom tapering to a point at the top. Drawn into the active UI
// target so it composites with the rest of the radial frame; cyan
// normally, red when the reservoir is low.
static void renderDroplet(int cx, int cy, int h, bool low) {
    LovyanGFX* g = uiTarget();
    int r = h * 4 / 10;            // bulb radius ~0.4 * height
    if (r < 2) r = 2;
    int topY  = cy - h / 2;        // tip
    int bulbY = topY + h - r;      // bulb centre
    // Low reservoir shows as muted warm gray (not red): "low" is a
    // soft advisory cue, while red is reserved for hard faults.
    uint16_t color = low ? UI_DARKGREY : UI_CYAN;
    g->fillCircle(cx, bulbY, r, color);
    g->fillTriangle(cx, topY, cx - r, bulbY, cx + r, bulbY, color);
}

// Small centre disk carrying a single glyph that encodes system state.
// The reservoir level is shown by the full-dial tank (drawTank), not
// here, so the disk is just a dark (or red, on fault) body, a 1px rim,
// and the glyph. All geometry is keyed to CENTRE_R.
//
//   fault          -> white X
//   BOOTING        -> filled white dot (warming up)
//   WATERING       -> filled blue droplet (pouring)
//   deferred (any) -> yellow warning triangle + exclamation
//   paused         -> two yellow vertical bars
//   healthy idle   -> cyan droplet outline
static void drawCentreIcon(LovyanGFX* g, bool fault, SystemState state,
                           bool deferred, bool paused) {
    const int cx = LCD_CX, cy = LCD_CY, r = CENTRE_R;

    // Disk body: red on fault, dark otherwise; 1px rim.
    g->fillCircle(cx, cy, r, fault ? UI_RED : UI_BLACK);
    g->drawCircle(cx, cy, r, fault ? UI_WHITE : UI_DARKGREY);

    // State glyph centred on the disk. Priority mirrors the LED state
    // machine: fault, then activity, then advisory.
    if (fault) {
        // X mark (each diagonal doubled 1px across for ~2px stroke).
        for (int o = 0; o <= 1; o++) {
            g->drawLine(cx - 7 + o, cy - 7, cx + 7 + o, cy + 7, UI_WHITE);
            g->drawLine(cx + 7 - o, cy - 7, cx - 7 - o, cy + 7, UI_WHITE);
        }
    } else if (state == BOOTING) {
        g->fillCircle(cx, cy, 5, UI_WHITE);
    } else if (state == WATERING) {
        // Filled droplet: round bulb tapering to a tip.
        const int dh = 14, dr = 5;
        int topY  = cy - dh / 2;
        int bulbY = topY + dh - dr;
        g->fillCircle(cx, bulbY, dr, UI_BLUE);
        g->fillTriangle(cx, topY, cx - dr, bulbY, cx + dr, bulbY, UI_BLUE);
    } else if (deferred) {
        // Warning triangle outline + exclamation (stem + dot).
        const int apexY = cy - 9, baseY = cy + 7, halfB = 9;
        g->drawLine(cx, apexY, cx - halfB, baseY, UI_YELLOW);
        g->drawLine(cx, apexY, cx + halfB, baseY, UI_YELLOW);
        g->drawLine(cx - halfB, baseY, cx + halfB, baseY, UI_YELLOW);
        g->fillRect(cx - 1, cy - 3, 2, 5, UI_YELLOW);
        g->fillRect(cx - 1, cy + 4, 2, 2, UI_YELLOW);
    } else if (paused) {
        // Two vertical bars.
        g->fillRect(cx - 5, cy - 6, 3, 12, UI_YELLOW);
        g->fillRect(cx + 2, cy - 6, 3, 12, UI_YELLOW);
    } else {
        // Healthy idle: droplet outline.
        const int dh = 14, dr = 5;
        int topY  = cy - dh / 2;
        int bulbY = topY + dh - dr;
        g->drawCircle(cx, bulbY, dr, UI_CYAN);
        g->drawLine(cx, topY, cx - dr, bulbY, UI_CYAN);
        g->drawLine(cx, topY, cx + dr, bulbY, UI_CYAN);
    }
}

// Radial arm pointer that mirrors the physical rotor position. Maps the
// live AS5600 reading into the LCD's dial coordinate frame using the
// same transform the web UI's applyArmAngle() does, then draws a thick
// blue radial line from just outside the centre disk out to the band
// stack. Drawn LAST so it sweeps over bands + halo + centre glyph,
// matching the web UI's layering. The arm radius is bounded inside
// BAND_R_OUT so it never reaches the droplet in the 6-o'clock dead
// zone (and the rotor's mechanical stops prevent the arm from pointing
// there anyway during normal operation).
//
// Trust gating: if endstops aren't calibrated or the encoder isn't
// healthy, the live angle is meaningless -- park the arm at bucket 1
// in UI_DARKGREY so the dial still has a pointer but the colour
// signals "don't believe this position."
static void drawArm(LovyanGFX* g) {
    float dialDeg;
    uint16_t armColor;
    if (g_endstopsCalibrated && g_encoderHealthy) {
        float sign   = g_bucketAngleStepPositive ? +1.0f : -1.0f;
        float arcPos = (g_armPositionDeg - g_encoderZeroOffset) * sign;
        while (arcPos <    0.0f) arcPos += 360.0f;
        while (arcPos >= 360.0f) arcPos -= 360.0f;
        dialDeg = (FIRST_DEG - BUCKET_ENDSTOP_OFFSET_DEG) + arcPos;
        while (dialDeg >= 360.0f) dialDeg -= 360.0f;
        while (dialDeg <    0.0f) dialDeg += 360.0f;
        armColor = UI_BLUE;
    } else {
        dialDeg  = FIRST_DEG;
        armColor = UI_DARKGREY;
    }

    int x0, y0, x1, y1;
    polarToXY(CENTRE_R + 2, dialDeg, &x0, &y0);
    polarToXY(BAND_R_OUT - 1, dialDeg, &x1, &y1);

    // 4-px thick arm rendered as two fillTriangles. polarToXY's radial
    // direction is (sin t, -cos t); the perpendicular (rotated 90deg)
    // is (cos t, sin t). Offset the four corner points by +/- halfW
    // along that perpendicular, with rounding away from zero so the
    // two sides stay symmetric across the full rotation.
    float t  = dialDeg * (float)M_PI / 180.0f;
    float px = cosf(t), py = sinf(t);
    const float halfW = 2.0f;
    int dxh = (int)(px * halfW + (px >= 0 ? 0.5f : -0.5f));
    int dyh = (int)(py * halfW + (py >= 0 ? 0.5f : -0.5f));
    int x0a = x0 + dxh, y0a = y0 + dyh;
    int x0b = x0 - dxh, y0b = y0 - dyh;
    int x1a = x1 + dxh, y1a = y1 + dyh;
    int x1b = x1 - dxh, y1b = y1 - dyh;
    g->fillTriangle(x0a, y0a, x0b, y0b, x1b, y1b, armColor);
    g->fillTriangle(x0a, y0a, x1b, y1b, x1a, y1a, armColor);
}

static void renderUI(bool force) {
    static uint32_t lastRender = 0;
    uint32_t now = millis();
    if (!force && (now - lastRender) < UI_REDRAW_MIN_MS) return;
    lastRender = now;

    // Panel is powered down -- skip the full-screen SPI repaint
    // entirely. (DIM still renders so a glance reads current state;
    // the wake path flips g_displayState back to ACTIVE before its
    // own renderUI(true), so a wake is never blocked here.)
    if (g_displayState == DISPLAY_ASLEEP) return;

    if (scaleOK) {
        float g = weightFromRaw();
        if (!isnan(g)) g_reservoirMl = g / G_PER_ML;
    }

    uint32_t t0 = millis();
    ensureUICanvas();
    LovyanGFX* gfx = uiTarget();
    bool direct = !g_uiCanvasReady;   // no sprite -> manage the bus ourselves
    if (direct) gfx->startWrite();

    // 1. Clear the frame.
    gfx->fillScreen(UI_BLACK);

    // 2. Bone reservoir tank with a gravity water fill.
    float frac = (g_reservoirFullMl > 0)
                  ? (g_reservoirMl / g_reservoirFullMl) : 0.0f;
    drawTank(gfx, frac);

    // 3. Per-bucket dose pills.
    for (int i = 0; i < g_bucketCount; i++) {
        float a0, a1;
        bucketAngles(i, &a0, &a1);
        drawBucketRings(gfx, g_buckets[i].dose_ml, a0, a1,
                        UI_BLUE, UI_DARKGREY);
    }

    // 4. Selection halo just OUTSIDE the ring stack (matches the web
    // UI's orange selection ring). While a headless interaction session
    // is active, the halo tracks the bucket the arm is parked at (so the
    // LCD mirrors the physical arm); otherwise it follows the web-UI
    // selection (g_selected).
    int haloIdx = (g_interactionParkedBucket > 0)
                  ? g_interactionParkedBucket - 1
                  : g_selected;
    if (haloIdx >= 0 && haloIdx < g_bucketCount) {
        float a0, a1;
        bucketAngles(haloIdx, &a0, &a1);
        drawRoundedBand(gfx, BAND_R_OUT + 1.0f, BAND_R_OUT + 5.0f,
                        a0 - 1.0f, a1 + 1.0f, UI_YELLOW);
    }

    // 5. Centre disk: state glyph. ERROR turns the disk red with a white
    // X; the bands + tank still render normally so the operator sees
    // which bucket faulted in context.
    bool  fault    = (g_state == ERROR && g_lastFault != FAULT_NONE);
    bool  deferred = (g_lastDeferReason != DEFER_NONE);
    bool  paused   = (!g_schedulerEnabled && !deferred && g_state == READY);
    drawCentreIcon(gfx, fault, g_state, deferred, paused);

    // 6. Droplet in the 6-o'clock dead-zone. Red when the reservoir
    // can't cover the sum of all bucket doses.
    float totalDoseMl = 0;
    for (int i = 0; i < g_bucketCount; i++) totalDoseMl += g_buckets[i].dose_ml;
    renderDroplet(LCD_CX, LCD_CY + 50, 14, g_reservoirMl < totalDoseMl);

    // 7. Rotor arm pointer, drawn last so it sweeps over bands + halo +
    // centre glyph -- mirrors the web dial's layering. The arm is
    // radius-bounded inside BAND_R_OUT, well clear of the droplet.
    drawArm(gfx);

    // 8. Present the frame.
    if (direct) gfx->endWrite();
    else        g_uiCanvas.pushSprite(0, 0);

    uint32_t dt = millis() - t0;
    if (dt > 150) LOG("renderUI slow: %lums\n", (unsigned long)dt);
}

// LCD render for the active bucket during WATERING. Same radial dial
// as the home view: the active bucket's rings stay full-colour with a
// yellow halo while the rest dim back, and the centre disk shows the
// reservoir level (draining in real time as the cycle dispenses) under
// a blue droplet glyph. Called from runWateringCycle's dispense loop
// (rate-limited internally by UI_REDRAW_MIN_MS).
static void renderWateringFocused() {
    static uint32_t lastRender = 0;
    uint32_t now = millis();
    if ((now - lastRender) < UI_REDRAW_MIN_MS) return;
    lastRender = now;

    uint32_t t0 = millis();
    ensureUICanvas();
    LovyanGFX* gfx = uiTarget();
    bool direct = !g_uiCanvasReady;
    if (direct) gfx->startWrite();

    gfx->fillScreen(UI_BLACK);

    // Bone tank with live water level (drains as the cycle proceeds).
    float frac = (g_reservoirFullMl > 0)
                  ? (g_reservoirMl / g_reservoirFullMl) : 0.0f;
    drawTank(gfx, frac);

    // Bucket pills: the active bucket full-colour, the rest dimmed so
    // the eye lands on the one being watered. g_cycleBucketIdx is
    // 1-indexed; convert to 0-based for the band loop.
    int active0 = (int)g_cycleBucketIdx - 1;
    for (int i = 0; i < g_bucketCount; i++) {
        float a0, a1;
        bucketAngles(i, &a0, &a1);
        if (i == active0) {
            drawBucketRings(gfx, g_buckets[i].dose_ml, a0, a1,
                            UI_BLUE, UI_DARKGREY);
        } else {
            drawBucketRings(gfx, g_buckets[i].dose_ml, a0, a1,
                            LCD_DIMBLUE, LCD_DIMGREY);
        }
    }
    if (active0 >= 0 && active0 < g_bucketCount) {
        float a0, a1;
        bucketAngles(active0, &a0, &a1);
        drawRoundedBand(gfx, BAND_R_OUT + 1.0f, BAND_R_OUT + 5.0f,
                        a0 - 1.0f, a1 + 1.0f, UI_YELLOW);
    }

    // Centre disk: WATERING droplet glyph (the reservoir level is shown
    // by the draining tank behind it).
    drawCentreIcon(gfx, /*fault=*/false, WATERING,
                   /*deferred=*/false, /*paused=*/false);

    // Droplet in the 6-o'clock dead-zone; red when the reservoir can't
    // cover the active bucket's dose.
    renderDroplet(LCD_CX, LCD_CY + 50, 14,
                  g_reservoirMl < g_activeBucketTargetMl);

    // Rotor arm pointer drawn last so it sweeps over the dimmed bands
    // and the active-bucket halo -- the operator sees the arm visibly
    // pointing at the bucket whose dose bar is animating.
    drawArm(gfx);

    if (direct) gfx->endWrite();
    else        g_uiCanvas.pushSprite(0, 0);

    uint32_t dt = millis() - t0;
    if (dt > 150) LOG("renderWatering slow: %lums\n", (unsigned long)dt);
}

// ---------- display power management ----------
//
// markInteraction() is the single "user touched something" signal: it
// bumps the idle timer and, if the panel had dimmed or slept, restores
// it to full brightness. It returns true ONLY when it actually woke a
// sleeping panel, so the BtnA path can swallow the press that did the
// waking (see updateGestures / g_displayJustWoke). Called from the
// button, serial, and web paths, from enterFault, and from
// AbortableMotionGuard's constructor.
static bool markInteraction() {
    g_lastInteractionMs = millis();
    if (g_displayState == DISPLAY_ACTIVE) return false;
    bool wasAsleep = (g_displayState == DISPLAY_ASLEEP);
    if (wasAsleep) AtomS3.Display.wakeup();
    AtomS3.Display.setBrightness(DISPLAY_BRIGHTNESS_ACTIVE);
    g_displayState = DISPLAY_ACTIVE;
    renderUI(true);
    return wasAsleep;
}

// manageDisplayPower() runs once per loop tick. A set of "force-active"
// states (watering, a posted fault, demo, any abortable motion) hold
// the panel on and keep the idle timer fresh, so it never dims while
// the operator most wants to see it -- and so it doesn't dim the moment
// such a state exits. Otherwise the time since the last interaction
// drives ACTIVE -> DIM -> ASLEEP. Brightness/sleep/wakeup calls fire
// only on transitions (guarded by the g_displayState check), never
// every tick.
static void manageDisplayPower() {
    bool forceActive = (g_state == WATERING)
                    || (g_state == ERROR && g_lastFault != FAULT_NONE)
                    || (g_demoMode != DemoMode::Off)
                    || g_abortableMotion;

    if (forceActive) {
        if (g_displayState != DISPLAY_ACTIVE) {
            if (g_displayState == DISPLAY_ASLEEP) AtomS3.Display.wakeup();
            AtomS3.Display.setBrightness(DISPLAY_BRIGHTNESS_ACTIVE);
            g_displayState = DISPLAY_ACTIVE;
            renderUI(true);
        }
        g_lastInteractionMs = millis();
        return;
    }

    uint32_t elapsed = millis() - g_lastInteractionMs;
    if (elapsed >= DISPLAY_SLEEP_AFTER_MS) {
        if (g_displayState != DISPLAY_ASLEEP) {
            AtomS3.Display.sleep();
            g_displayState = DISPLAY_ASLEEP;
        }
    } else if (elapsed >= DISPLAY_DIM_AFTER_MS) {
        if (g_displayState == DISPLAY_ACTIVE) {
            AtomS3.Display.setBrightness(DISPLAY_BRIGHTNESS_DIM);
            g_displayState = DISPLAY_DIM;
        }
    } else if (g_displayState != DISPLAY_ACTIVE) {
        // Recent interaction but the panel isn't active (e.g. the timer
        // was bumped by a path that didn't itself wake it). Restore.
        if (g_displayState == DISPLAY_ASLEEP) AtomS3.Display.wakeup();
        AtomS3.Display.setBrightness(DISPLAY_BRIGHTNESS_ACTIVE);
        g_displayState = DISPLAY_ACTIVE;
        renderUI(true);
    }
}

// ---------- status LED ----------
//
// resolveLedState() maps the current global state to a color +
// animation pattern via a strict priority ladder (first match wins).
// updateLed() runs once per loop tick (~20 Hz, millis()-gated),
// computes the instantaneous envelope from the pattern, scales the
// color, and writes the single WS2812. Nothing else in the firmware
// references the LED -- this reads globals and is the sole writer.
//
// Note: the per-pattern periods below encode each rung's intended
// frequency (e.g. PULSE_SLOW=2000ms=0.5Hz, ALTERNATE_TWO=1000ms=1Hz).
// The scheduler-paused rung shares LED_BREATHE_SLOW's 3000ms period
// with the demo rung; the two are told apart by color (amber warning
// vs healthy green) rather than rate, since LedState carries no period.
//
// updateLed() is only called from loop(), so during a blocking motion
// op (watering, demo, diagnostics) the LED holds its last frame --
// by design; we deliberately don't pump it from the motion paths.
// Total wall-clock duration of the current dose-pulse sequence, shared
// by resolveLedState (to know when the sequence is over and the parked
// indicator should take back over) and the LED_DOSE_PULSE_SEQUENCE
// renderer. A skip dose is a single slow LED_DOSE_SKIP_MS gray pulse;
// otherwise each of N blue pulses is LED_DOSE_PULSE_MS with an
// LED_DOSE_GAP_MS gap between them.
#define LED_DOSE_PULSE_MS   400
#define LED_DOSE_GAP_MS     150
#define LED_DOSE_SKIP_MS    600
// LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
// Only the CRGB render path (ledApply / the real updateLed body) is gated
// out under #if 0 below, because GPIO 8 is now the Port B I2C SDA line and
// FastLED can't own it. The state-resolution + envelope math
// (ledDoseSequenceDurationMs / resolveLedState / ledPatternEnvelope) is
// CRGB-free and stays live: the lightweight updateLed() below still runs
// each loop() tick and refreshes g_ledSnapshot / g_ledEnvelope, so the web
// UI's on-screen arm keeps mirroring the logical LED color and
// breathes/pulses in lockstep with the (now-dark) physical SK6812. The
// state ladder and LED logic are unchanged; only the physical output is
// disabled. Restoring the LED on a future hardware rev means re-enabling
// the #if 0 render body and dropping the lightweight updateLed() below.
static uint32_t ledDoseSequenceDurationMs() {
    if (g_ledDosePulseCount == 0) return LED_DOSE_SKIP_MS;
    return (uint32_t)g_ledDosePulseCount * LED_DOSE_PULSE_MS
         + (uint32_t)(g_ledDosePulseCount - 1) * LED_DOSE_GAP_MS;
}

static LedState resolveLedState() {
    // User-facing priority ladder, first match wins. The LED answers
    // "is the rig healthy / asking for attention?", not "what
    // diagnostic state am I in." It speaks one unified language --
    // three colors and two animation classes:
    //
    //   GREEN  = healthy / good (idle, demo rest, mapping, deliberate
    //            user-initiated motor ops)
    //   BLUE   = water in motion -- active dispense, plus the headless
    //            interaction feedback (parked / dose-pulse)
    //   AMBER  = warning -- the rig is fine and will sort itself out
    //            (auto-clearing deferrals, low reservoir) or is paused
    //            by the user; no acknowledgement needed
    //   RED    = fault -- needs the user to acknowledge / intervene
    //
    //   BREATHE = calm / steady-state
    //   PULSE   = needs attention
    //
    // So a deferral or a paused scheduler reads amber (a warning the
    // rig clears on its own), while only a latched fault reads red.
    // Green is reserved for resting-healthy and healthy activity, so an
    // active dispense reads blue rather than green. The remaining
    // specialty cues (the headless interaction sequences, the AP-mode
    // cyan blink) are interaction feedback rather than status and stand
    // outside this language.

    // 0: master kill switch -- brightness 0 means dark, full stop.
    if (g_ledBrightness == 0) return { 0, 0, LED_OFF };

    // 1: hard fault (latched) -- red, slow pulse.
    if (g_state == ERROR)
        return { LED_COL_RED, 0, LED_PULSE_SLOW };

    // 2: WiFi config / AP mode -- cyan blink (1 Hz). Nothing waters in
    //    AP mode, so this takes priority over the cycle / status rungs.
    if (g_apMode)
        return { LED_COL_CYAN, 0, LED_BLINK_SLOW };

    // 3: headless interaction overrides everything below -- an active
    //    session owns the LED until the next press or the idle auto-park.
    //  3a: pressed while position untrusted -- brief red flash (~300ms).
    if (g_ledUnknownPositionFlashMs != 0
        && (millis() - g_ledUnknownPositionFlashMs) < 300UL)
        return { LED_COL_RED, 0, LED_UNKNOWN_POSITION };
    //  3b: dose pulse sequence after an advance arrival or a dose adjust.
    if (g_ledDoseSequenceMs != 0
        && (millis() - g_ledDoseSequenceMs) < ledDoseSequenceDurationMs())
        return { LED_COL_WATERBLUE, 0, LED_DOSE_PULSE_SEQUENCE };
    //  3c: parked at a bucket -- steady blue scaled by the parked dose.
    if (g_interactionParkedBucket != 0)
        return { LED_COL_WATERBLUE, 0, LED_PARKED_AT_BUCKET };

    // 4: watering in progress -- blue breathe whose brightness ceiling
    //    rises with dose progress. Water is blue in the brand language;
    //    green is reserved for resting-healthy and would give no visible
    //    cue that dispensing is happening. Covers demo dispense too, since
    //    a demo pour runs with g_state == WATERING.
    if (g_state == WATERING)
        return { LED_COL_WATERBLUE, 0, LED_DISPENSE_PROGRESS };

    // 4b: demo loop between cycles (state back to READY, g_demoMode set)
    //     -- resting green breathe. A clean demo loop is healthy
    //     activity, not a warning. The pour itself is rung 4.
    if (g_demoMode != DemoMode::Off)
        return { LED_COL_GREEN, 0, LED_BREATHE_SLOW };

    // 5: deferral states -- amber warnings, not faults. A deferral
    //    auto-clears with no user acknowledgement (level the rig,
    //    refill the tank), so it speaks the warning color rather than
    //    the fault color. NOT_LEVEL breathes ("I'll water once you
    //    level me"); RESERVOIR_LOW pulses for more attention. A motion
    //    deferral (DEFER_MOVING) falls through: it's transient and
    //    clears on its own within a tick or two.
    if (g_lastDeferReason == DEFER_NOT_LEVEL)
        return { LED_COL_AMBER, 0, LED_BREATHE_SLOW };
    if (g_lastDeferReason == DEFER_RESERVOIR_LOW)
        return { LED_COL_AMBER, 0, LED_PULSE_SLOW };

    // 6: scheduler paused by the user -- amber slow breathe. A paused
    //    scheduler is a warning state (the rig is fine but won't water
    //    on its own), so it shares the warning color. Gated on READY so
    //    a paused-but-still-booting rig shows the boot rung.
    if (!g_schedulerEnabled && g_state == READY)
        return { LED_COL_AMBER, 0, LED_BREATHE_SLOW };

    // 7: reservoir low (still has water, but below the sum of all
    //    doses) -- amber slow breathe. A warning, not a stop. Gated on
    //    READY: g_reservoirMl is 0 until the first scale read, so an
    //    unguarded check would flash amber over the whole boot window.
    if (g_state == READY) {
        float totalDoseMl = 0;
        for (int i = 0; i < g_bucketCount; i++)
            totalDoseMl += g_buckets[i].dose_ml;
        if (g_reservoirMl < totalDoseMl)
            return { LED_COL_AMBER, 0, LED_BREATHE_SLOW };
    }

    // 8: abortable foreground motor op (home / calibrate / tests /
    //    rotate / autotune / interaction advance hop) -- green breathe.
    //    A deliberate, user-initiated motor op is healthy activity, not
    //    "needs attention," so it breathes in the health color rather
    //    than pulsing. Ranks below the attention rungs above on purpose:
    //    a motor op signals "moving" only when nothing more important is
    //    being shown.
    if (g_abortableMotion)
        return { LED_COL_GREEN, 0, LED_BREATHE_SLOW };

    // 9: still booting -- dim green, solid.
    if (g_state == BOOTING)
        return { LED_COL_GREEN_DIM, 0, LED_SOLID };

    // 10: healthy but headless (no WiFi, operating) -- dim neutral gray solid.
    if (!g_wifiUp && g_state == READY)
        return { LED_COL_GRAYDIM, 0, LED_SOLID };

    // 11: healthy idle default -- green slow breathe (same breathe rate
    //     as watering, but green: idle is resting-healthy, not pouring).
    return { LED_COL_GREEN, 0, LED_BREATHE_SLOW };
}

// Instantaneous 0..255 pattern envelope for the current LedState, computed
// straight from the pattern's timebase -- the same e*255 scalar each render
// case in the #if 0 updateLed() below feeds into ledApply(), but without
// building a CRGB or touching FastLED. This is the web arm's brightness
// mirror (g_ledEnvelope): it lets the on-screen arm breathe/pulse in
// lockstep with the (now-disabled) physical LED, independent of the master
// g_ledBrightness. Off frames (LED_OFF, blink/gap phases) collapse to 0.
// Keep in sync with the render switch in updateLed() below.
static uint8_t ledPatternEnvelope(const LedState& s, uint32_t now) {
    switch (s.pattern) {
        case LED_OFF:
            return 0;
        case LED_SOLID:
            return 255;
        case LED_BREATHE_SLOW: {
            // Full sinusoid (continuous breathing) over 3000 ms.
            float frac = (float)(now % 3000UL) / 3000.0f;
            float e = sinf(2.0f * (float)M_PI * frac) * 0.5f + 0.5f;
            return (uint8_t)(e * 255.0f + 0.5f);
        }
        case LED_PULSE_SLOW: {
            // Half-sine hump (0->1->0) over 2000 ms.
            float frac = (float)(now % 2000UL) / 2000.0f;
            float e = sinf((float)M_PI * frac);
            if (e < 0.0f) e = 0.0f;
            return (uint8_t)(e * 255.0f + 0.5f);
        }
        case LED_PULSE_FAST: {
            // Half-sine hump over 500 ms.
            float frac = (float)(now % 500UL) / 500.0f;
            float e = sinf((float)M_PI * frac);
            if (e < 0.0f) e = 0.0f;
            return (uint8_t)(e * 255.0f + 0.5f);
        }
        case LED_BLINK_SLOW:
            // 50% duty, 1000 ms period.
            return ((now % 1000UL) < 500UL) ? 255 : 0;
        case LED_ALTERNATE_TWO:
            // color1 first half, color2 second half, 1000 ms period; both
            // halves render at full scale, so the envelope is always full.
            return 255;
        case LED_DISPENSE_PROGRESS: {
            // Blue breathe whose brightness ceiling scales with dose
            // progress: 30% ceiling at 0% poured, rising to 100% at full.
            float progress = 0.0f;
            if (g_activeBucketTargetMl > 0) {
                progress = g_activeBucketDeliveredMl
                           / (float)g_activeBucketTargetMl;
                if (progress < 0.0f) progress = 0.0f;
                if (progress > 1.0f) progress = 1.0f;
            }
            float ceil   = 0.3f + 0.7f * progress;
            float breath = sinf((float)(now % 3000UL) / 3000.0f
                                * 2.0f * (float)M_PI) * 0.5f + 0.5f;
            return (uint8_t)(ceil * breath * 255.0f + 0.5f);
        }
        case LED_PARKED_AT_BUCKET: {
            // Steady indicator scaled by the parked bucket's dose; a skip
            // bucket shows a steady dim warm gray. Mirrors the env values
            // the render case feeds ledApply().
            int idx = g_interactionParkedBucket - 1;
            if (idx < 0 || idx >= g_bucketCount)
                return 38;   // safety; shouldn't happen
            uint16_t dose = g_buckets[idx].dose_ml;
            if (dose == DOSE_SKIP)
                return 76;   // ~30%, steady
            uint16_t env = 64 + (uint16_t)((uint32_t)(dose - DOSE_MIN_ML)
                           * 191UL / (DOSE_MAX_ML - DOSE_MIN_ML));
            return (uint8_t)env;
        }
        case LED_DOSE_PULSE_SEQUENCE: {
            // Count the dose in pulses; render the instantaneous envelope.
            uint32_t elapsed = now - g_ledDoseSequenceMs;
            if (g_ledDosePulseCount == 0) {
                // Skip: one slow warm-gray hump over LED_DOSE_SKIP_MS.
                float frac = (float)elapsed / (float)LED_DOSE_SKIP_MS;
                if (frac > 1.0f) frac = 1.0f;
                float e = sinf((float)M_PI * frac);
                if (e < 0.0f) e = 0.0f;
                return (uint8_t)(e * 255.0f + 0.5f);
            }
            const uint32_t slot = LED_DOSE_PULSE_MS + LED_DOSE_GAP_MS;
            uint32_t idx    = elapsed / slot;    // which pulse
            uint32_t within = elapsed % slot;    // phase within slot
            if (idx >= g_ledDosePulseCount
                || within >= (uint32_t)LED_DOSE_PULSE_MS)
                return 0;   // inter-pulse gap or past the end
            float frac = (float)within / (float)LED_DOSE_PULSE_MS;
            float e = sinf((float)M_PI * frac);   // half-sine hump
            if (e < 0.0f) e = 0.0f;
            return (uint8_t)(e * 255.0f + 0.5f);
        }
        case LED_UNKNOWN_POSITION:
            // Solid red for the ~300ms window resolveLedState gates on.
            return 255;
    }
    return 0;
}

#if 0  // LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
// Scale a 0xRRGGBB color by a pattern envelope (0..255) and then by
// the master g_ledBrightness, per the spec's "(channel * scale) >> 8"
// rule, applied uniformly to every channel.
static CRGB ledApply(uint32_t rgb, uint16_t env) {
    uint8_t r = (uint8_t)((rgb >> 16) & 0xFF);
    uint8_t g = (uint8_t)((rgb >> 8)  & 0xFF);
    uint8_t b = (uint8_t)( rgb        & 0xFF);
    r = (uint8_t)(((uint32_t)r * env) >> 8);
    g = (uint8_t)(((uint32_t)g * env) >> 8);
    b = (uint8_t)(((uint32_t)b * env) >> 8);
    r = (uint8_t)(((uint32_t)r * g_ledBrightness) >> 8);
    g = (uint8_t)(((uint32_t)g * g_ledBrightness) >> 8);
    b = (uint8_t)(((uint32_t)b * g_ledBrightness) >> 8);
    return CRGB(r, g, b);
}

static void updateLed() {
    uint32_t now = millis();
    if (now - g_lastLedUpdateMs < LED_UPDATE_PERIOD_MS) return;
    g_lastLedUpdateMs = now;

    LedState s = resolveLedState();
    CRGB out   = CRGB::Black;
    g_ledSnapshot = s;   // mirror for the web arm (see g_ledSnapshot decl)

    switch (s.pattern) {
        case LED_OFF:
            out = CRGB::Black;
            break;
        case LED_SOLID:
            out = ledApply(s.color1, 255);
            break;
        case LED_BREATHE_SLOW: {
            // Full sinusoid (continuous breathing) over 3000 ms.
            float frac = (float)(now % 3000UL) / 3000.0f;
            float e = sinf(2.0f * (float)M_PI * frac) * 0.5f + 0.5f;
            out = ledApply(s.color1, (uint16_t)(e * 255.0f + 0.5f));
            break;
        }
        case LED_PULSE_SLOW: {
            // Half-sine hump (0->1->0) over 2000 ms.
            float frac = (float)(now % 2000UL) / 2000.0f;
            float e = sinf((float)M_PI * frac);
            if (e < 0.0f) e = 0.0f;
            out = ledApply(s.color1, (uint16_t)(e * 255.0f + 0.5f));
            break;
        }
        case LED_PULSE_FAST: {
            // Half-sine hump over 500 ms.
            float frac = (float)(now % 500UL) / 500.0f;
            float e = sinf((float)M_PI * frac);
            if (e < 0.0f) e = 0.0f;
            out = ledApply(s.color1, (uint16_t)(e * 255.0f + 0.5f));
            break;
        }
        case LED_BLINK_SLOW: {
            // 50% duty, 1000 ms period.
            out = ((now % 1000UL) < 500UL) ? ledApply(s.color1, 255)
                                           : CRGB::Black;
            break;
        }
        case LED_ALTERNATE_TWO: {
            // color1 first half, color2 second half, 1000 ms period.
            out = ((now % 1000UL) < 500UL) ? ledApply(s.color1, 255)
                                           : ledApply(s.color2, 255);
            break;
        }
        case LED_DISPENSE_PROGRESS: {
            // Blue breathe whose brightness ceiling scales with dose
            // progress: a soft 30% ceiling at 0% poured, rising to 100%
            // at full dose. Same gentle 3000 ms breathe as the healthy-
            // idle rung (just blue instead of green), so watering reads
            // as "alive and working" rather than the old diagnostic
            // orange->cyan color sweep.
            float progress = 0.0f;
            if (g_activeBucketTargetMl > 0) {
                progress = g_activeBucketDeliveredMl
                           / (float)g_activeBucketTargetMl;
                if (progress < 0.0f) progress = 0.0f;
                if (progress > 1.0f) progress = 1.0f;
            }
            // 30% brightness ceiling at 0% progress, 100% at full dose.
            float ceil   = 0.3f + 0.7f * progress;
            float breath = sinf((float)(now % 3000UL) / 3000.0f
                                * 2.0f * (float)M_PI) * 0.5f + 0.5f;
            out = ledApply(s.color1,
                           (uint16_t)(ceil * breath * 255.0f + 0.5f));
            break;
        }
        case LED_PARKED_AT_BUCKET: {
            // Steady indicator: brightness scales linearly with the
            // parked bucket's dose (10mL -> ~25%, 60mL -> 100%); a skip
            // bucket shows a steady dim warm gray so "no water here"
            // reads differently from a low dose.
            int idx = g_interactionParkedBucket - 1;
            if (idx < 0 || idx >= g_bucketCount) {
                out = ledApply(s.color1, 38);   // safety; shouldn't happen
                break;
            }
            uint16_t dose = g_buckets[idx].dose_ml;
            if (dose == DOSE_SKIP) {
                out = ledApply(LED_COL_GRAYWARM, 76);   // ~30%, steady
            } else {
                uint16_t env = 64 + (uint16_t)((uint32_t)(dose - DOSE_MIN_ML)
                               * 191UL / (DOSE_MAX_ML - DOSE_MIN_ML));
                out = ledApply(LED_COL_WATERBLUE, env);
            }
            break;
        }
        case LED_DOSE_PULSE_SEQUENCE: {
            // Count the dose in pulses. resolveLedState only selects this
            // pattern while the sequence is live (elapsed < duration), so
            // here we just render the instantaneous envelope.
            uint32_t elapsed = now - g_ledDoseSequenceMs;
            if (g_ledDosePulseCount == 0) {
                // Skip: one slow warm-gray hump over LED_DOSE_SKIP_MS,
                // visibly distinct from a single 10mL blue pulse.
                float frac = (float)elapsed / (float)LED_DOSE_SKIP_MS;
                if (frac > 1.0f) frac = 1.0f;
                float e = sinf((float)M_PI * frac);
                if (e < 0.0f) e = 0.0f;
                out = ledApply(LED_COL_GRAYWARM,
                               (uint16_t)(e * 255.0f + 0.5f));
            } else {
                const uint32_t slot = LED_DOSE_PULSE_MS + LED_DOSE_GAP_MS;
                uint32_t idx    = elapsed / slot;    // which pulse
                uint32_t within = elapsed % slot;    // phase within slot
                if (idx >= g_ledDosePulseCount
                    || within >= (uint32_t)LED_DOSE_PULSE_MS) {
                    out = CRGB::Black;   // inter-pulse gap or past the end
                } else {
                    float frac = (float)within / (float)LED_DOSE_PULSE_MS;
                    float e = sinf((float)M_PI * frac);   // half-sine hump
                    if (e < 0.0f) e = 0.0f;
                    out = ledApply(LED_COL_WATERBLUE,
                                   (uint16_t)(e * 255.0f + 0.5f));
                }
            }
            break;
        }
        case LED_UNKNOWN_POSITION:
            // Solid red for the ~300ms window resolveLedState gates on.
            out = ledApply(LED_COL_RED, 255);
            break;
    }

    // Cache the instantaneous pattern brightness for the web arm. Recover
    // it from the rendered pixel: maxOut == maxC1 * env * g_ledBrightness
    // (both >>8), so env == maxOut * 65536 / (maxC1 * g_ledBrightness).
    // Dividing out maxC1 and the master brightness yields the pure 0..255
    // pattern envelope, so the on-screen arm breathes/pulses identically
    // regardless of the physical LED's brightness setting. Off frames
    // (out == black: LED_OFF, blink/gap phases) collapse to envelope 0.
    {
        uint8_t c1r = (s.color1 >> 16) & 0xFF;
        uint8_t c1g = (s.color1 >> 8)  & 0xFF;
        uint8_t c1b =  s.color1        & 0xFF;
        uint8_t maxC1 = c1r > c1g ? (c1r > c1b ? c1r : c1b)
                                  : (c1g > c1b ? c1g : c1b);
        uint8_t maxOut = out.r > out.g ? (out.r > out.b ? out.r : out.b)
                                       : (out.g > out.b ? out.g : out.b);
        uint16_t env = 0;
        if (maxC1 > 0 && g_ledBrightness > 0) {
            uint32_t e = (uint32_t)maxOut * 65536UL
                         / ((uint32_t)maxC1 * (uint32_t)g_ledBrightness);
            env = (e > 255) ? 255 : (uint16_t)e;
        }
        g_ledEnvelope = (uint8_t)env;
    }

    g_leds[0] = out;
    FastLED.show();
}
#endif  // LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.

// LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
// Lightweight stand-in for the real updateLed() (CRGB render body gated out
// above). Same ~20 Hz throttle and the same resolveLedState() it always
// drove, but it only refreshes the web-arm mirror (g_ledSnapshot /
// g_ledEnvelope) -- it never touches g_leds[0] or calls FastLED.show(), so
// GPIO 8 stays free for the Port B encoder bus. The on-screen arm therefore
// keeps its live color and breathe/pulse animation even though the physical
// SK6812 is dark. Call sites in setup(), loop() and serviceAbort() are
// unchanged. Restore the #if 0 body (and delete this) on the next rev.
static void updateLed() {
    uint32_t now = millis();
    if (now - g_lastLedUpdateMs < LED_UPDATE_PERIOD_MS) return;
    g_lastLedUpdateMs = now;

    LedState s   = resolveLedState();
    g_ledSnapshot = s;                          // web arm color mirror
    g_ledEnvelope = ledPatternEnvelope(s, now); // web arm brightness mirror
}

// 24-bit RGB to "#rrggbb" hex string. Static buffer; result valid until
// the next call. Used only by apiStateJson(), which appends each result
// into the JSON string before the next call, so the single-buffer
// pattern is safe here.
static const char* ledColorHex(uint32_t rgb) {
    static char buf[8];
    snprintf(buf, sizeof(buf), "#%02x%02x%02x",
             (unsigned)((rgb >> 16) & 0xFF),
             (unsigned)((rgb >> 8)  & 0xFF),
             (unsigned)( rgb        & 0xFF));
    return buf;
}

// Enum name (LED_ prefix stripped) for the JSON led.pattern field; lets
// the web UI choose a matching arm animation.
static const char* ledPatternName(LedPattern p) {
    switch (p) {
        case LED_OFF:                 return "OFF";
        case LED_SOLID:               return "SOLID";
        case LED_BREATHE_SLOW:        return "BREATHE_SLOW";
        case LED_PULSE_SLOW:          return "PULSE_SLOW";
        case LED_PULSE_FAST:          return "PULSE_FAST";
        case LED_BLINK_SLOW:          return "BLINK_SLOW";
        case LED_ALTERNATE_TWO:       return "ALTERNATE_TWO";
        case LED_DISPENSE_PROGRESS:   return "DISPENSE_PROGRESS";
        case LED_PARKED_AT_BUCKET:    return "PARKED_AT_BUCKET";
        case LED_DOSE_PULSE_SEQUENCE: return "DOSE_PULSE_SEQUENCE";
        case LED_UNKNOWN_POSITION:    return "UNKNOWN_POSITION";
    }
    return "OFF";
}

// ---------- gesture detection ----------
//
// All button reads go through M5.BtnA, not AtomS3.BtnA. M5.BtnA is
// declared as a reference inside M5Unified (`Button_Class &BtnA =
// _buttons[0];`) and AtomS3.BtnA caches that reference at static
// init time. If the linker happens to schedule AtomS3's global
// constructor before M5's, AtomS3.BtnA snapshots a still-zero
// reference value and stays null forever -- the next call into
// .isPressed() dereferences null and panics with LoadProhibited at
// EXCVADDR=0x18. M5.BtnA is set by M5Unified's constructor; by the
// time setup() body runs, all static init has completed and the
// reference is valid. So we go directly through M5 and skip the
// fragile AtomS3 indirection.
static int      g_pressCount    = 0;
static uint32_t g_pressStartMs  = 0;
static uint32_t g_lastReleaseMs = 0;
static bool     g_longConsumed  = false;

// Drop any in-flight button edges and suppress the long-press path so
// the press that *triggered* an abort isn't re-interpreted as a
// gesture (phantom long-press tare, stale triple-press) once loop()
// resumes calling updateGestures().
static void resetGestureAfterCycle() {
    g_pressCount     = 0;
    g_longConsumed   = true;
    g_displayJustWoke = false;
    g_pressStartMs   = millis();
    g_lastReleaseMs  = millis();
    AtomS3.update();
    (void)M5.BtnA.wasPressed();
    (void)M5.BtnA.wasReleased();
}

// ---------- headless interaction (arm-as-UI) ----------
//
// Definitions for the forward-declared interaction functions. The
// gesture handlers (handleShortPress / handleDoublePress) call into
// these; loop()'s READY branch drives the blocking gotoBucket so the
// gesture handler itself never blocks (the same foreground-dispatch
// pattern used by the g_webRequest* flags).

// Arm the brief red "position untrusted" LED flash.
static void trigger_led_unknown_position_flash() {
    g_ledUnknownPositionFlashMs = millis();
}

// Start a dose-pulse LED sequence reflecting doseMl: skip -> one gray
// pulse (count 0); otherwise (dose / DOSE_STEP_ML) blue pulses (1..6).
static void trigger_led_dose_pulse(uint16_t doseMl) {
    if (doseMl == DOSE_SKIP) {
        g_ledDosePulseCount = 0;
    } else {
        g_ledDosePulseCount =
            (uint8_t)((doseMl - DOSE_MIN_ML) / DOSE_STEP_ML) + 1;   // 1..6
    }
    g_ledDoseSequenceMs = millis();
}

// Clear all interaction-session state. Called when a session ends:
// after the 30s idle auto-park, and (defensively) whenever the rig
// leaves READY -- runWateringCycle() and enterFault() stomp the
// session so the arm/LED model never lingers across a cycle or fault.
static void interactionEnd() {
    g_interactionParkedBucket = 0;
    g_interactionTargetBucket = 0;
    g_interactionLastPressMs  = 0;
    g_interactionAdvancing    = false;
    g_ledDosePulseCount       = 0;
    g_ledDoseSequenceMs       = 0;
}

// Short-press action: advance the arm one bucket. If a session is
// already advancing, this just bumps the target (the queue) and
// loop() chains the next gotoBucket when the current hop finishes.
// When position isn't trusted, flash red and do nothing -- the user
// must home first (web UI, or the next scheduled cycle auto-homes).
static void interactionAdvance() {
    if (g_state != READY) return;            // defense in depth
    if (!g_endstopsCalibrated || !g_encoderHealthy) {
        trigger_led_unknown_position_flash();
        return;
    }
    g_interactionLastPressMs = millis();

    // Advance from the in-flight target if we're mid-queue, else from
    // the parked bucket, else from bucket 1 (implicit start of session).
    int from = (g_interactionTargetBucket > 0) ? g_interactionTargetBucket
             : (g_interactionParkedBucket > 0) ? g_interactionParkedBucket
             : 1;
    int next = from + 1;
    if (next > g_bucketCount) next = 1;
    g_interactionTargetBucket = next;

    // Already moving: target bumped, loop() will chain. Otherwise flag
    // the session as advancing; loop()'s READY branch kicks the hop.
    if (!g_interactionAdvancing) g_interactionAdvancing = true;
}

// Double-press action: step the parked bucket's dose along the ladder
// (skip -> 10 -> 20 -> ... -> 60 -> skip) and play the dose pulse for
// the NEW value. With no active session, bucket 1 is the implicit
// target (matches the old boot-default g_selected == 0 behavior).
// Per the design, a dose change applies to g_interactionParkedBucket
// (the bucket the arm last arrived at), never the in-flight target.
static void interactionAdjustDose() {
    if (g_state != READY) return;
    int bucket = (g_interactionParkedBucket > 0) ? g_interactionParkedBucket : 1;
    int idx = bucket - 1;
    if (idx < 0 || idx >= g_bucketCount) return;

    uint16_t cur = g_buckets[idx].dose_ml;
    uint16_t d;
    if (cur == DOSE_SKIP)        d = DOSE_MIN_ML;
    else if (cur >= DOSE_MAX_ML) d = DOSE_SKIP;
    else                          d = cur + DOSE_STEP_ML;

    g_buckets[idx].dose_ml = d;
    persistMarkDose(idx);
    g_interactionLastPressMs = millis();
    trigger_led_dose_pulse(d);
    LOGI("interaction: bucket %d dose -> %u mL%s\n",
         bucket, d, d == DOSE_SKIP ? " (skip)" : "");
    renderUI(true);   // keep the LCD in sync if present
}

// Per-tick interaction housekeeping, called from loop()'s READY
// branch. Detects arrival of a queued advance (parks + fires the dose
// pulse) and drives the 30s idle auto-park back to bucket 1.
static void interactionTick() {
    if (g_state != READY)            return;
    if (g_demoMode != DemoMode::Off) return;   // demos pause the timer

    // Arrival of an advance hop: the motion completed (!g_abortableMotion)
    // and the rotor is at the current target. If the target was bumped
    // mid-hop (queue), this won't match yet -- loop() chains another
    // gotoBucket and we park only on the final one.
    if (g_interactionAdvancing
        && !g_abortableMotion
        && g_currentBucket == g_interactionTargetBucket) {
        g_interactionParkedBucket = g_interactionTargetBucket;
        g_interactionAdvancing    = false;
        int idx = g_interactionParkedBucket - 1;
        if (idx >= 0 && idx < g_bucketCount) {
            trigger_led_dose_pulse(g_buckets[idx].dose_ml);
        }
        // Redraw so the LCD selection halo jumps to the bucket the arm
        // just parked at (renderUI reads g_interactionParkedBucket).
        renderUI(true);
    }

    // 30s idle auto-park: return to bucket 1 and end the session. (A
    // bucket-1 park is already "home", so a session parked there never
    // auto-parks.) The park runs under AbortableMotionGuard; a press
    // during it aborts the park via serviceAbort's normal path and the
    // session ends -- the operator's next press starts a fresh session.
    if (g_interactionParkedBucket != 0
        && g_interactionParkedBucket != 1
        && !g_interactionAdvancing
        && (millis() - g_interactionLastPressMs) > INTERACTION_IDLE_MS) {
        LOGI("%s\n", "interaction: 30s idle, parking at bucket 1");
        AbortableMotionGuard motionGuard;
        g_abortRequested = false;
        (void)gotoBucket(1);
        g_abortRequested = false;
        stepperReleaseCoils();   // park may have aborted with coils held
        interactionEnd();
    }
}

static void handleShortPress() {
    // Headless model: advance the arm to the next bucket and park there
    // (LED + arm position are the UI). The web UI's g_selected highlight
    // is independent and is intentionally left untouched here.
    interactionAdvance();
}

static void handleDoublePress() {
    // Headless model: adjust the dose at the bucket the arm is physically
    // parked at (g_interactionParkedBucket), not g_selected.
    interactionAdjustDose();
}

static void handleTriplePress() {
    Serial.println("Triple press: triggering cycle");
    runWateringCycle();
}

static void handleLongPress() {
    // Standalone tare is gone -- scale zero/calibration is a factory
    // step in the web Settings panel (empty cell -> known mass ->
    // Calibrate, committed atomically). The on-device long-press no
    // longer tares the scale.
    Serial.println("Long press (no action; calibrate via web Settings)");
}

// Wipe stored STA WiFi credentials from NVS and reboot. Shared by the
// serial 'W' command and the runtime 10 s BtnA hold gesture so both take
// exactly the same code path. WiFi.disconnect(true, true) erases the
// NVS-stored creds; after the reboot the auto-AP path (no creds stored ->
// auto-enter config portal, see setup()) brings the unit up in the
// captive portal for re-provisioning. Flush pending edits first so a
// reset never costs the user dose / interval / tare changes made in the
// last few seconds. Works with WiFi down/unconnected -- it touches only
// NVS and the radio, never any link state. Never returns.
static void wipeWifiCredsAndReboot(const char* src) {
    LOGI("Erasing WiFi credentials and rebooting (%s)...\n", src);
    persistFlushAll();
    WiFi.disconnect(true, true);
    delay(200);
    ESP.restart();
}

// Runtime WiFi-reset hold. Entered from updateGestures() once BtnA has
// been held for >=WIFI_RESET_HINT_MS while g_state == READY. Takes over
// the foreground for the rest of the hold, drawing a cancellable
// countdown + progress bar straight to the panel. Releasing before
// WIFI_RESET_HOLD_MS returns without touching anything (the caller's
// gesture machine is resynced here so the long release can't fire a
// phantom gesture); reaching the threshold wipes creds and reboots.
//
// Blocking is acceptable and deliberate: the rig is idle (READY, no
// watering, no motion -- the caller's gate guarantees it), and this
// gesture exists precisely for a unit that can't reach the network, so
// stalling the (unreachable) web pump for the few seconds of the hold
// costs nothing. The panel is already awake (the press that started the
// hold marked an interaction).
static void runWifiResetHoldGesture() {
    M5.Display.fillScreen(UI_BLACK);
    M5.Display.setTextColor(UI_WHITE, UI_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(6, 6);
    M5.Display.println("Hold to");
    M5.Display.println("reset");
    M5.Display.println("WiFi");

    // Progress bar tracks the visible window [hint, fire]; border drawn
    // once, the orange fill only ever grows so no clear/flicker per tick.
    const int barX = 8, barW = 112, barY = 104, barH = 16;
    M5.Display.drawRect(barX, barY, barW, barH, UI_WHITE);
    int shownLeft = -1;

    while (M5.BtnA.isPressed()) {
        AtomS3.update();
        uint32_t held = millis() - g_pressStartMs;
        if (held >= WIFI_RESET_HOLD_MS) break;   // threshold reached -> fire

        float frac = (float)(held - WIFI_RESET_HINT_MS)
                     / (float)(WIFI_RESET_HOLD_MS - WIFI_RESET_HINT_MS);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        M5.Display.fillRect(barX + 1, barY + 1,
                            (int)(frac * (barW - 2)), barH - 2, UI_ORANGE);

        int left = (int)((WIFI_RESET_HOLD_MS - held + 999) / 1000);
        if (left != shownLeft) {
            M5.Display.fillRect(0, 64, 128, 34, UI_BLACK);
            M5.Display.setTextSize(4);
            M5.Display.setCursor(52, 64);
            M5.Display.printf("%d", left);
            shownLeft = left;
        }
        delay(20);
    }

    if (!M5.BtnA.isPressed()) {
        // Released before the threshold: cancel cleanly. Resync the
        // gesture machine (the press is long past the multi-press window)
        // and force the normal UI back.
        LOGI("%s\n", "WiFi-reset hold cancelled (released before 10s)");
        resetGestureAfterCycle();
        renderUI(true);
        return;
    }

    // Held the full duration: confirm briefly, then wipe + reboot.
    M5.Display.fillScreen(UI_BLACK);
    M5.Display.setTextColor(UI_WHITE, UI_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(6, 40);
    M5.Display.println("Resetting");
    M5.Display.println("WiFi...");
    delay(800);
    wipeWifiCredsAndReboot("button 10s hold");
}

static void updateGestures() {
    if (M5.BtnA.wasPressed()) {
        g_pressStartMs = millis();
        g_longConsumed = false;
        // If this press woke a sleeping panel, swallow it: the timing
        // above is still recorded (so a deliberate hold-after-wake is
        // tracked), but the gesture is suppressed so the wake tap
        // doesn't also cycle the selection / tare / clear a fault.
        // The release below consumes the flag; the next press is normal.
        if (markInteraction()) g_displayJustWoke = true;
    }
    // Long-press dispatch:
    //   - In ERROR: a hold past EXTRA_LONG (3s) clears the fault.
    //     The 1s tare path is suppressed -- a tare while displaying
    //     a fault is rarely the intended action, and the long hold
    //     would otherwise fire two gestures in succession.
    //   - Otherwise: existing 1s tare path.
    // Both are suppressed while g_displayJustWoke is set so a hold that
    // merely woke the panel can't fire a tare / fault-clear.
    if (M5.BtnA.isPressed() && !g_longConsumed && !g_displayJustWoke) {
        uint32_t held = millis() - g_pressStartMs;
        if (g_state == ERROR && held >= GESTURE_EXTRA_LONG_MS) {
            clearFault("button>=3s");
            g_longConsumed = true;
            g_pressCount   = 0;
        } else if (g_state != ERROR && held >= GESTURE_LONG_MS) {
            handleLongPress();
            g_longConsumed = true;
            g_pressCount   = 0;
        }
    }
    // Runtime WiFi-reset hold (READY only). Once BtnA has been held past
    // the hint threshold, hand off to the blocking countdown handler,
    // which fires the wipe at WIFI_RESET_HOLD_MS or cancels on release.
    // READY-gates it so it can never fire during WATERING or active
    // motion (those states don't reach updateGestures, but the guard is
    // explicit and also keeps the 10 s hold clear of the ERROR >=3 s
    // fault-clear above). Allowed even when this press woke the panel
    // (g_displayJustWoke): a 10 s hold is unambiguous, and the wake left
    // the panel available to show the countdown (manageDisplayPower has
    // powered it back on well within the 2 s hint window). The 1 s
    // long-press no-op may or may not have fired already; harmless either
    // way since it only tares (a no-op now).
    if (g_state == READY && M5.BtnA.isPressed()
        && (millis() - g_pressStartMs) >= WIFI_RESET_HINT_MS) {
        runWifiResetHoldGesture();
        return;
    }
    if (M5.BtnA.wasReleased()) {
        markInteraction();
        if (g_displayJustWoke) {
            // Release of the press that woke the panel: consume the
            // flag and don't count it toward a multi-press tally.
            g_displayJustWoke = false;
            g_pressCount      = 0;
        } else if (!g_longConsumed) {
            uint32_t held = millis() - g_pressStartMs;
            if (held < GESTURE_SHORT_MAX_MS) {
                g_pressCount++;
                g_lastReleaseMs = millis();
            } else {
                // Held in [400ms, 1000ms): ambiguous, drop.
                // Also covers ERROR releases between 1s and 3s --
                // intentional, so a near-miss on the 3s threshold
                // doesn't tare the scale by accident.
                g_pressCount = 0;
            }
        }
    }
    if (g_pressCount > 0
        && !M5.BtnA.isPressed()
        && (millis() - g_lastReleaseMs) > GESTURE_GAP_MS) {
        int n = g_pressCount;
        g_pressCount = 0;
        if      (n == 1) handleShortPress();
        else if (n == 2) handleDoublePress();
        else             handleTriplePress();
    }
}

// ---------- serial ----------

static void printMenu() {
    Serial.println("---- menu ----");
    Serial.println("?       print menu");
    Serial.println("s       (re)detect U177 mini-scale");
    Serial.println("t       capture scale zero (calibration step 1; empty cell)");
    Serial.println("c G     calibrate U177 gap value with known mass G grams on cell");
    Serial.println("w       trigger watering cycle now");
    Serial.println("x       abort current cycle / motion (any button press also aborts)");
    Serial.println("h       home now (servo to bucket 1)");
    Serial.println("C       calibrate endstops (drive into both stops to learn encoder zero + direction)");
    Serial.println("d N V   set bucket N (1-indexed) dose to V mL "
                   "(V=0 to skip, or 10..60)");
    Serial.println("i N H   set bucket N interval to H hours "
                   "(12|24|48|72|120|168|240|336|504)");
    Serial.println("D V     dispense test (V=0 dry, V=1 wet); "
                   "same as watering but no load cell");
    Serial.println("V O A   set valve OPEN angle to A (0..180), persists");
    Serial.println("V C A   set valve CLOSED angle to A (0..180), persists");
    Serial.println("Y V     set post-valve dwell to V seconds (1..10), persists");
    Serial.println("F ML    set reservoir-full volume (UI gauge full-scale, "
                   "100..20000 mL), persists; bare F prints");
    Serial.println("B V     set status-LED brightness to V (0..255, 0=off), persists");
    Serial.println("P F|S|R V   stepper profile: P F <us> fast, "
                   "P S <us> slow, P R <n> ramp; bare P prints");
    Serial.println("R       rotate continuously forward (motor test); "
                   "'x' or button to stop");
    Serial.println("r       rotate continuously reverse (motor test); "
                   "'x' or button to stop");
    Serial.println("J D [N] OPEN-LOOP blind jog: step motor D=+/- by N steps "
                   "(default 32, max 114); works with NO encoder; voids "
                   "position trust -> recal required before normal motion");
    Serial.println("A       acknowledge / clear fault (ERROR -> READY)");
    Serial.println("p       print state");
    Serial.println("X       toggle DEMO mode (dry): loop dispense test continuously; "
                   "x/button/web to stop (wet demo via web)");
    Serial.println("N       toggle DEMO Random (dry): loop, watering 1..3 random "
                   "buckets/cycle; cadence test; x/button/web to stop");
    Serial.println("W       reset WiFi credentials and reboot to AP mode");
    Serial.println("E       toggle watering scheduler (PAUSE / RESUME)");
    Serial.println("Z       toggle DEBUG fast-schedule mode (intervals as seconds, for testing)");
    Serial.println("L V     set spill threshold (max tilt for watering) to V degrees (1..30), persists");
    Serial.println("M V     set IMU motion threshold to V g (0.02..1.0), persists");
    Serial.println("O N     set mount orientation: 0=Z_UP 1=Z_DOWN 2=Y_UP 3=Y_DOWN 4=X_UP 5=X_DOWN, persists");
}

// Non-destructive dispense motion test: cycles through every bucket
// without opening the valve, dwelling 500ms on each so the operator
// can visually confirm the rotor parked on the magnet (hall LED
// lit). Factored out of the 'D' serial case so /api/test/dispense
// can drive the same path.
// Dispense test: runs a full watering cycle in test mode. Caller is
// responsible for setting g_testModeWet before calling. The cycle
// itself handles homing, per-bucket gotoBucket + retry, abort, fault
// recovery, post-cycle home, and summary logging -- everything the
// old hand-rolled D-test did, but in the same code path as a real
// cycle. cycleEndCleanup() inside runWateringCycle clears the test
// flags on every exit path.
// Returns runWateringCycle()'s result so callers (the demo loop) can
// tell a clean cycle from an abort/fault. Existing statement-callers
// (serial 'D', the loop() dispatch) simply ignore the return.
static bool runDispenseTest() {
    if (g_state == WATERING) {
        LOG("%s\n", "not while watering");
        return false;
    }
    g_testModeActive = true;
    return runWateringCycle();
}

// ---------- demo mode ----------
//
// Blocking foreground loop dispatched from loop()'s READY branch (see
// g_webRequestDemoStart). g_demoMode is set to Dry or Wet by the
// trigger (serial 'X' or POST /api/demo/start) before this runs.
//
// Each iteration sets g_testModeWet for the mode, calls the existing
// runDispenseTest() (NOT a parallel dispense loop), bumps the cycle
// counter on a clean completion, then waits g_demoPauseSec while
// polling serviceAbort() so a button / serial 'x' / web stop can land
// during the gap. The whole loop exits -- demo OFF, scheduler back on
// -- on ANY of: user abort, a fault (runDispenseTest returns false),
// or POST /api/demo/stop. It never auto-restarts after a fault.
//
// The AbortableMotionGuard makes serviceAbort() honor BtnA during the
// inter-cycle pause too (g_state is READY between cycles, so without
// the guard the button path would be ignored there).
static void runDemoLoop() {
    // g_demoMode is OWNED by this function. The trigger only armed
    // g_demoPendingMode + g_webRequestDemoStart; g_demoMode stays Off until
    // we actually enter, so demo.active is never stranded true by a start
    // that never ran (Finding 1).
    if (g_demoPendingMode == DemoMode::Off) return;   // defensive; nothing armed

    // Honor a stop that landed in the arm->entry window BEFORE running any
    // cycle -- do NOT swallow it. Critical for a wet demo: no water may flow
    // against a stop the user already issued (Finding 2). The old code blindly
    // cleared g_webRequestDemoStop here, losing exactly this stop.
    if (g_webRequestDemoStop) {
        LOGI("%s\n", "demo: stop arrived before start; not entering");
        g_demoPendingMode    = DemoMode::Off;
        g_webRequestDemoStop = false;
        return;
    }
    if (g_state != READY) {
        LOGI("%s\n", "demo: not READY; ignoring start");
        g_demoPendingMode = DemoMode::Off;
        return;
    }

    // Real entry: take ownership of g_demoMode now (consumes the pending mode).
    g_demoMode           = g_demoPendingMode;
    g_demoPendingMode    = DemoMode::Off;
    g_demoCycleCount     = 0;
    // Do NOT clear g_webRequestDemoStop here -- it was already checked above,
    // and clearing it would re-introduce the swallowed-stop bug. Only reset
    // the low-level abort latch so a stale abort can't kill cycle 1.
    g_abortRequested     = false;
    AbortableMotionGuard motionGuard;

    // Pause the scheduler so an interval that comes due mid-demo can't race
    // the demo cycles -- but remember the PRE-demo state first so the exit is
    // pause-transparent: a demo run on a manually-paused rig must leave it
    // paused, not silently resume auto-watering (Finding 3).
    bool schedWasEnabled = g_schedulerEnabled;
    setSchedulerEnabled(false, "demo");
    LOGI("== DEMO MODE start: %s (pause=%us) ==\n",
         g_demoMode == DemoMode::Random ? "RANDOM" :
         g_demoMode == DemoMode::Wet    ? "WET"    : "DRY",
         (unsigned)g_demoPauseSec);

    while (g_demoMode != DemoMode::Off && !g_webRequestDemoStop) {
        // Wet/dry is caller-selected and deterministic for the whole
        // session (orthogonal to randomness), matching Dry/Wet modes:
        // Dry/Wet encode it in the mode; Random carries it in
        // g_demoRandomWet. cycleEndCleanup clears g_testModeWet on every
        // exit, so re-arm it each iteration.
        bool wetThisCycle = (g_demoMode == DemoMode::Random)
                              ? g_demoRandomWet
                              : (g_demoMode == DemoMode::Wet);
        g_testModeWet = wetThisCycle;

        if (g_demoMode == DemoMode::Random) {
            // Re-roll the bucket subset each cycle: 1..3 distinct non-skip
            // buckets, watered via the real cycle path (calibration
            // cadence, arc sort, mid-arc sweep, dispense, park-or-not) --
            // the cycle just sees a narrowed candidate set, no parallel
            // motion. esp_random() is fine; nothing to seed.
            int nonSkip[BUCKET_COUNT];
            int nonSkipCount = 0;
            for (int i = 0; i < g_bucketCount; i++)
                if (g_buckets[i].dose_ml != DOSE_SKIP)
                    nonSkip[nonSkipCount++] = i;
            if (nonSkipCount == 0) {
                LOGI("%s\n", "DEMO Random: all buckets skipped (no dose set); "
                             "stopping");
                break;
            }
            int maxWant = nonSkipCount < 3 ? nonSkipCount : 3;
            int want    = 1 + (int)(esp_random() % (uint32_t)maxWant);
            // Partial Fisher-Yates: shuffle the first `want` slots to draw
            // distinct indices without replacement.
            for (int k = 0; k < want; k++) {
                int j = k + (int)(esp_random() % (uint32_t)(nonSkipCount - k));
                int tmp = nonSkip[k]; nonSkip[k] = nonSkip[j]; nonSkip[j] = tmp;
            }
            // Sort the chosen indices ascending for a stable, readable log
            // (runWateringCycle re-sorts by arc internally regardless).
            for (int a = 1; a < want; a++) {
                int key = nonSkip[a], b = a - 1;
                while (b >= 0 && nonSkip[b] > key) { nonSkip[b + 1] = nonSkip[b]; b--; }
                nonSkip[b + 1] = key;
            }
            g_demoRandomMask = 0;
            for (int k = 0; k < want; k++)
                g_demoRandomMask |= (uint16_t)(1u << nonSkip[k]);
            g_demoRandomActive = true;

            // Predict the calibration decision for the cadence-test log,
            // using the exact predicate runWateringCycle applies (they run
            // back-to-back on the foreground, so the prediction matches).
            bool willCal = (g_cyclesSinceCalibration >= CALIBRATE_EVERY_N_CYCLES)
                         || g_forceRecalNextCycle
                         || !isSavedCalTrusted();

            // Cadence-test summary: chosen buckets (1-indexed), dry/wet,
            // calibrated-or-skipped, starting arm position. Per-bucket
            // gotoBucket arc_err/dir/final-err lines follow from the cycle.
            String blist;
            for (int k = 0; k < want; k++) {
                if (k) blist += ",";
                blist += (nonSkip[k] + 1);
            }
            String startPos = (g_currentBucket >= 1)
                                ? ("bucket" + String(g_currentBucket))
                                : String("unknown");
            LOGI("DEMO Random cycle %lu: buckets [%s] %s %s start@%s\n",
                 (unsigned long)(g_demoCycleCount + 1), blist.c_str(),
                 wetThisCycle ? "WET" : "DRY",
                 willCal ? "calibrated" : "skipped-cal",
                 startPos.c_str());
        } else {
            LOGI("DEMO: cycle starting (%s)\n", wetThisCycle ? "WET" : "DRY");
        }

        bool ok = runDispenseTest();

        // ok==false means the cycle aborted, faulted, or failed to
        // home. In every such case we leave demo mode -- no retry,
        // no auto-restart after a fault.
        if (!ok || g_abortRequested || g_state == ERROR) {
            LOGI("DEMO: stopping (%s)\n",
                 g_abortRequested ? "user abort"
                 : (g_state == ERROR) ? "fault"
                 : "cycle did not complete");
            break;
        }

        g_demoCycleCount++;
        LOGI("DEMO: cycle #%lu complete\n",
             (unsigned long)g_demoCycleCount);

        // Inter-cycle pause. Render the demo countdown on the LCD and
        // poll serviceAbort() so a stop/abort lands promptly.
        g_demoInterCyclePause = true;
        uint32_t pauseMs    = (uint32_t)g_demoPauseSec * 1000UL;
        uint32_t pauseStart = millis();
        while ((millis() - pauseStart) < pauseMs) {
            uint32_t elapsed = millis() - pauseStart;
            uint32_t remMs   = (pauseMs > elapsed) ? (pauseMs - elapsed) : 0;
            g_demoPauseRemainSec = (uint8_t)((remMs + 999UL) / 1000UL);
            renderWateringFocused();
            if (serviceAbort() || g_webRequestDemoStop) break;
            delay(100);
        }
        g_demoInterCyclePause = false;
        if (g_abortRequested || g_webRequestDemoStop) break;
    }

    // Exit: always leave demo OFF and the scheduler back on. Defensively
    // clear the Random subset too -- cycleEndCleanup clears it per cycle,
    // but if any runWateringCycle early-return ever skipped that, a stale
    // mask must not survive to restrict the resumed scheduler.
    g_demoMode            = DemoMode::Off;
    g_demoPendingMode     = DemoMode::Off;
    g_demoInterCyclePause = false;
    g_webRequestDemoStop  = false;
    g_testModeWet         = false;
    g_demoRandomActive    = false;
    g_demoRandomMask      = 0;
    g_abortRequested      = false;
    // Restore the scheduler to its PRE-demo state, not unconditionally on: a
    // demo run while the user had manually paused must stay paused (Finding 3).
    setSchedulerEnabled(schedWasEnabled, "demo");
    // The abort source was likely BtnA; resync the gesture machine so
    // the still-held press doesn't fire a stale gesture next tick.
    resetGestureAfterCycle();
    if (g_state != ERROR) g_state = READY;
    LOGI("== DEMO MODE exit (%lu cycles) ==\n",
         (unsigned long)g_demoCycleCount);
    renderUI(true);
}

// Continuous-rotate motor test. dir = +1 spins forward, -1 reverse.
// Energizes coils, advances phases at g_stepDelayFastUs, polls
// serviceAbort() every STEP_ABORT_POLL_PERIOD steps. Exits on any
// abort path (BtnA, serial 'x'/'X', or POST /api/stepper/stop, which
// just sets g_abortRequested). pumpWeb stays default-true so the
// web Stop button can land while we're spinning.
static void rotateContinuous(int dir) {
    if (g_state == WATERING) {
        LOG("%s\n", "rotateContinuous: not while watering");
        return;
    }
    if (dir == 0) return;
    AbortableMotionGuard motionGuard;
    g_abortRequested = false;

    LOGI("== Continuous rotate %s ==\n",
         dir > 0 ? "FORWARD" : "REVERSE");

    stepperWake();
    uint32_t stepsTaken = 0;
    for (;;) {
        setStepperPhase(nextPhase(g_phaseIndex, dir));
        g_stepCounter += dir;
        stepsTaken++;
        delayMicroseconds(g_stepDelayFastUs);
        if ((stepsTaken & (STEP_ABORT_POLL_PERIOD - 1)) == 0) {
            if (serviceAbort()) break;
        }
    }
    stepperReleaseCoils();
    g_currentBucket = bucketFromAngle(g_armPositionDeg);
    g_abortRequested = false;
    // Open-loop blind spin desyncs the saved calibration from the true rotor
    // angle, exactly like jogOpenLoop(). Void position trust so normal
    // closed-loop motion won't resume until a recalibration re-establishes
    // position -- the bridge that keeps this debug tool from enabling blind
    // auto-watering. (With a dead encoder, recal can't succeed, so the unit
    // correctly stays in refuse-normal-motion until the sensor is fixed.)
    bool wasCal = g_endstopsCalibrated;
    if (wasCal) {
        g_endstopsCalibrated = false;
        persistSaveEndstopCal();
    }
    LOGI("rotateContinuous done: %lu steps taken; arm angle=%.2fdeg; "
         "position trust VOID%s -- recalibrate before normal motion\n",
         (unsigned long)stepsTaken, (double)g_armPositionDeg,
         wasCal ? " (was calibrated, now invalidated)" : "");
}

// OPEN-LOOP MANUAL JOG (debug escape hatch). Steps the motor a BOUNDED,
// fixed amount with NO encoder feedback, so the motor/coils/mechanics can be
// exercised when the AS5600 is dead/absent and closed-loop motion is refused.
// This is the ONLY motor op that bypasses the encoder/position-trust gate --
// it is explicit and manual; the scheduler, auto-watering, and gotoBucket are
// untouched and still refuse to move without a healthy encoder. A blind move
// desyncs any saved calibration from the true rotor position, so it VOIDS
// position trust (g_endstopsCalibrated=false) and forces a recalibration
// before normal motion. If the encoder is dead, recal can't succeed either,
// so the unit correctly stays in no-trust / refuse-normal-motion until the
// sensor is fixed -- the jog never re-enables watering. Refused only while
// WATERING (don't fight an active dispense cycle).
static void jogOpenLoop(int dir, int steps) {
    if (g_state == WATERING) {
        LOGI("%s\n", "jog: refused (watering in progress)");
        return;
    }
    if (dir == 0) { LOGI("%s\n", "jog: bad direction"); return; }
    int d = (dir > 0) ? +1 : -1;
    if (steps <= 0) steps = JOG_DEFAULT_STEPS;
    if (steps > JOG_MAX_STEPS_PER_CMD) {
        LOGI("jog: clamping %d -> %u steps (per-command cap)\n",
             steps, (unsigned)JOG_MAX_STEPS_PER_CMD);
        steps = JOG_MAX_STEPS_PER_CMD;
    }
    if (g_jogSessionSteps + (uint32_t)steps > JOG_MAX_STEPS_PER_SESSION) {
        LOGI("jog: REFUSED -- session cap reached (%lu/%u blind steps); "
             "recalibrate or reboot to reset before further blind jogging\n",
             (unsigned long)g_jogSessionSteps,
             (unsigned)JOG_MAX_STEPS_PER_SESSION);
        return;
    }

    LOGI("== OPEN-LOOP BLIND JOG %s %d steps ==\n",
         d > 0 ? "FWD" : "REV", steps);
    LOGI("%s\n", "jog: BLIND MOVE -- no position feedback, encoder NOT "
                 "consulted; position trust will be VOIDED after this move");

    AbortableMotionGuard motionGuard;   // abortable via BtnA / 'x' / web stop
    g_abortRequested = false;
    stepperWake();
    int taken = 0;
    for (int s = 0; s < steps; s++) {
        setStepperPhase(nextPhase(g_phaseIndex, d));
        g_stepCounter += d;
        taken++;
        delayMicroseconds(g_stepDelayFastUs);
        if ((s & (STEP_ABORT_POLL_PERIOD - 1)) == 0) {
            if (serviceAbort()) break;
        }
    }
    stepperReleaseCoils();
    g_abortRequested = false;
    g_jogSessionSteps += (uint32_t)taken;

    // Void position trust: a blind move desyncs the saved calibration from
    // the true rotor angle. Force a recal before any normal closed-loop move.
    bool wasCal = g_endstopsCalibrated;
    if (wasCal) {
        g_endstopsCalibrated = false;
        persistSaveEndstopCal();
    }
    LOGI("jog done: %d steps (session %lu/%u); position trust VOID%s -- "
         "recalibrate before normal motion\n",
         taken, (unsigned long)g_jogSessionSteps,
         (unsigned)JOG_MAX_STEPS_PER_SESSION,
         wasCal ? " (was calibrated, now invalidated)" : "");
}

static void printState() {
    const char* sname = "?";
    switch (g_state) {
        case BOOTING:  sname = "BOOTING";  break;
        case READY:    sname = "READY";    break;
        case WATERING: sname = "WATERING"; break;
        case ERROR:    sname = "ERROR";    break;
    }
    Serial.printf("State: %s  Buckets: %d  Reservoir: %.1f / %.1f mL\n",
                  sname, g_bucketCount,
                  g_reservoirMl, g_reservoirFullMl);
    if (g_lastFault != FAULT_NONE) {
        uint32_t ago = millis() - g_faultMs;
        Serial.printf("FAULT: %s  bucket=%d  ms_ago=%lu\n",
                      faultName(g_lastFault),
                      g_faultBucket,
                      (unsigned long)ago);
    }
    Serial.printf("Stepper: stepCounter=%ld  phaseIndex=%u  "
                  "coilsEnergized=%s  motorInv=%s\n",
                  (long)g_stepCounter,
                  (unsigned)g_phaseIndex,
                  g_coilsEnergized ? "yes" : "no",
                  g_motorInverted ? "yes" : "no");
    Serial.printf("Profile: fast=%uus (slow/ramp removed; single rate)\n",
                  (unsigned)g_stepDelayFastUs);
    Serial.printf("Motion telemetry: largeFinalErr count=%lu last=%.2fdeg "
                  "(tol=%.1fdeg; not a fault)\n",
                  (unsigned long)g_largeFinalErrCount,
                  (double)g_lastLargeFinalErrDeg,
                  (double)ARRIVAL_TOL_DEG);
    Serial.printf("Dispense telemetry: suspectActive=%s skipCount=%lu "
                  "lastSuspectBucket=%d\n",
                  g_suspectActive ? "yes" : "no",
                  (unsigned long)g_suspectSkipCount,
                  g_lastSuspectBucket);
    Serial.printf("Open-loop jog: sessionSteps=%lu/%u (blind; voids position "
                  "trust until recal)\n",
                  (unsigned long)g_jogSessionSteps,
                  (unsigned)JOG_MAX_STEPS_PER_SESSION);
    Serial.printf("Encoder: armAngle=%.2fdeg  healthy=%s  status=0x%02X  "
                  "endstopsCal=%s  zeroOffset=%.2fdeg  dirPos=%s  "
                  "currentBucket=%d\n",
                  (double)g_armPositionDeg,
                  g_encoderHealthy ? "yes" : "no",
                  (unsigned)g_encoderStatus,
                  g_endstopsCalibrated ? "yes" : "no",
                  (double)g_encoderZeroOffset,
                  g_bucketAngleStepPositive ? "yes" : "no",
                  g_currentBucket);
    Serial.printf("Valve: openAng=%u closeAng=%u  dwell=%us\n",
                  (unsigned)g_valveOpenAng,
                  (unsigned)g_valveCloseAng,
                  (unsigned)g_postValveDwellSec);
    int32_t rem = (int32_t)(g_nextWaterMs - millis());
    Serial.printf("Next cycle in %ld ms\n", (long)rem);
    Serial.printf("Time mode: %s\n",
                  g_timeIsWallClock ? "WALL-CLOCK (NTP)" : "UPTIME");
    Serial.printf("Fast schedule: %s\n",
                  g_debugFastSchedule ? "ENABLED (seconds)" : "disabled");
    Serial.printf("Scheduler: %s\n",
                  g_schedulerEnabled ? "ENABLED" : "PAUSED");
    for (int i = 0; i < g_bucketCount; i++) {
        uint32_t now = scheduleTimeNow();
        uint32_t intervalSec = intervalToSec(g_buckets[i].interval_hours);
        bool due = (g_buckets[i].dose_ml != DOSE_SKIP)
                   && ((now - g_lastWateredAt[i]) >= intervalSec);
        Serial.printf("  bucket %d: dose=%umL interval=%uh%s\n",
                      i + 1,
                      g_buckets[i].dose_ml,
                      (unsigned)g_buckets[i].interval_hours,
                      due ? " [DUE]" : "");
    }
}

static void handleSerial() {
    if (!Serial.available()) return;
    markInteraction();   // any inbound serial byte is an interaction
    int c = Serial.read();
    switch (c) {
        case '?': printMenu(); break;
        case 's':
            Serial.println("Re-detecting U177 scale...");
            scaleOK = tryScaleInit();
            if (scaleOK) {
                uint8_t fw = 0;
                bool ok = scale.getFirmwareVersion(fw);
                Serial.printf("U177 OK. gapValue=%.3f fw=%s\n",
                              (double)g_gapValue,
                              ok ? String((unsigned)fw).c_str() : "?");
            } else {
                Serial.println("U177 still not detected");
            }
            break;
        case 't':
            // Calibration step 1: capture the empty-cell zero (RAM only;
            // committed with the gap by 'c'). Standalone tare is gone.
            // Not gated on scaleOK -- cal runs on raw counts; the checked
            // read in captureScaleZero is the real gate.
            scale.setOffset();
            if (captureScaleZero()) {
                markScaleLiveFromCal();   // proven-live raw read
                Serial.printf("Zero captured for calibration (rawTare=%ld); "
                              "add known mass then send 'c <grams>'\n",
                              (long)g_rawTare);
            } else {
                Serial.println("zero capture failed: scale not responding");
            }
            break;
        case 'w':
            if (g_state == READY) runWateringCycle();
            else                  Serial.println("not ready");
            break;
        case 'h': {
            AbortableMotionGuard motionGuard;
            g_abortRequested = false;
            (void)homeStepper();
            g_abortRequested = false;
            break;
        }
        case 'C': {
            // Endstop calibration: drive into both mechanical stops to
            // learn the encoder zero offset + direction sign. Lower-
            // case 'c' is the U177 scale calibration; uppercase 'C' is
            // the encoder / endstops.
            AbortableMotionGuard motionGuard;
            g_abortRequested = false;
            (void)calibrateEndstops();
            g_abortRequested = false;
            renderUI(true);
            break;
        }
        case 'c': {
            // U177 calibration. Sequence on bench:
            //   1. tare empty reservoir on the cell ('t')
            //   2. add a known mass (e.g. 500 g of water) to the
            //      reservoir on the cell
            //   3. send 'c <known g>' (e.g. 'c 500')
            // Computes a new gap value from the raw ADC and persists
            // it. The U177's STM32 also persists internally, so this
            // NVS copy is belt-and-suspenders if the unit gets swapped.
            // Not gated on scaleOK: the checked getRawADC below is the
            // real liveness gate, and cal on raw counts recovers an
            // unhealthy scale.
            float known = Serial.parseFloat();
            if (!(known > 0.0f)) {
                Serial.println("usage: c <known mass in grams>");
                break;
            }
            // Tared-delta cal: reg 0x00 is absolute raw, so dividing
            // raw by known would be offset-dominated and sign-ambiguous.
            // (raw_now - g_rawTare) is the counts contributed purely
            // by the known mass and inherits the cell's wiring sign.
            if (!g_rawTareSession) {
                Serial.println("calibrate aborted: capture zero first IN THIS SESSION ('t')");
                break;
            }
            // Checked read: distinguishes an I2C failure (NACK) from a
            // real value, so a flaky bus can never silently compute a gap
            // from getRawADC()'s 0-on-failure return. This single read
            // doubles as the bus-liveness check.
            int32_t raw = 0;
            if (!scale.getRawADC(raw)) {
                g_scaleCalActive = false;
                Serial.println("calibrate aborted: scale not responding");
                break;
            }
            markScaleLiveFromCal();   // proven-live raw read
            int32_t delta = raw - g_rawTare;
            int32_t adelta = delta < 0 ? -delta : delta;
            if (adelta < SCALE_CAL_MIN_DELTA) {
                g_scaleCalActive = false;
                Serial.printf("calibrate aborted: no weight change "
                              "(delta=%ld) -- is the mass on the cell?\n",
                              (long)delta);
                break;
            }
            // NVS-independent: compute purely from the live delta and the
            // entered grams, then overwrite device gap + g_gapValue + NVS.
            // Whatever NVS or the U177 held before is irrelevant here.
            float newGap  = (float)delta / known;
            scale.setGapValue(newGap);
            g_gapValue = newGap;
            // Atomic commit: zero + gap persisted together (never desync).
            persistCommitScaleCal();
            g_scaleCalActive = false;   // committed -> release scale ownership
            Serial.printf("Calibrated: rawADC=%ld rawTare=%ld delta=%ld "
                          "known=%.2fg -> gapValue=%.3f\n",
                          (long)raw, (long)g_rawTare, (long)delta,
                          (double)known, (double)newGap);
            break;
        }
        case 'd': {
            int n = Serial.parseInt();
            int v = Serial.parseInt();
            // Accept DOSE_SKIP (0, the skip sentinel) or any value
            // on the dose ladder [DOSE_MIN_ML, DOSE_MAX_ML].
            bool valid = (v == DOSE_SKIP)
                         || (v >= DOSE_MIN_ML && v <= DOSE_MAX_ML);
            if (n < 1 || n > g_bucketCount) {
                Serial.printf("bad bucket %d\n", n);
            } else if (!valid) {
                Serial.printf("dose out of range: %d\n", v);
            } else {
                g_buckets[n - 1].dose_ml = (uint16_t)v;
                persistMarkDose(n - 1);
                Serial.printf("bucket %d dose = %d mL%s\n", n, v,
                              v == DOSE_SKIP ? " (skip)" : "");
                renderUI(true);
            }
            break;
        }
        case 'i': {
            int n = Serial.parseInt();
            int h = Serial.parseInt();
            if (n < 1 || n > g_bucketCount) {
                Serial.printf("bad bucket %d\n", n);
                break;
            }
            bool valid = (h == 12 || h == 24 || h == 48 || h == 72
                          || h == 120 || h == 168 || h == 240
                          || h == 336 || h == 504);
            if (!valid) {
                Serial.println("invalid interval");
                break;
            }
            g_buckets[n - 1].interval_hours = (uint16_t)h;
            persistMarkInterval(n - 1);
            Serial.printf("bucket %d interval = %dh\n", n, h);
            break;
        }
        case 'V': {
            // 'V O <angle>' / 'V C <angle>' set the valve servo angles
            // for the open and closed positions. Whitespace-skip a few
            // bytes then read the sub-command char.
            uint32_t waitDeadline = millis() + 100;
            while (Serial.available() == 0 && millis() < waitDeadline) {
                delay(2);
            }
            int sub = -1;
            while (Serial.available()) {
                int peek = Serial.peek();
                if (peek == ' ' || peek == '\t') { Serial.read(); continue; }
                sub = Serial.read();
                break;
            }
            if (sub != 'O' && sub != 'C') {
                Serial.println("usage: V O <angle 0..180> | V C <angle 0..180>");
                break;
            }
            int v = Serial.parseInt();
            if (v < 0 || v > 180) {
                Serial.println("angle out of range 0..180");
                break;
            }
            if (sub == 'O') g_valveOpenAng  = (uint8_t)v;
            else            g_valveCloseAng = (uint8_t)v;
            persistSaveValveAngles();
            Serial.printf("Valve %s angle set to %d (saved)\n",
                          sub == 'O' ? "OPEN" : "CLOSED", v);
            break;
        }
        case 'P': {
            // Step rate live-tune. Only 'P F <us>' is meaningful; the
            // old 'P S' / 'P R' subcommands (slow rate, ramp) are
            // accepted as no-ops for backward compat with bench
            // scripts. Persists immediately. Bare 'P' prints current.
            uint32_t waitDeadline = millis() + 100;
            while (Serial.available() == 0 && millis() < waitDeadline) {
                delay(2);
            }
            if (Serial.available() == 0) {
                Serial.printf("Profile: fast=%uus (single rate)\n",
                              (unsigned)g_stepDelayFastUs);
                Serial.println("usage: P F <us>");
                break;
            }
            int sub = -1;
            while (Serial.available()) {
                int peek = Serial.peek();
                if (peek == ' ' || peek == '\t') { Serial.read(); continue; }
                sub = Serial.read();
                break;
            }
            int v = Serial.parseInt();
            if (sub == 'F') {
                if (v < STEP_DELAY_FAST_US_MIN || v > STEP_DELAY_FAST_US_MAX) {
                    Serial.printf("fast out of range %d..%d\n",
                                  STEP_DELAY_FAST_US_MIN,
                                  STEP_DELAY_FAST_US_MAX);
                    break;
                }
                g_stepDelayFastUs = (uint16_t)v;
            } else if (sub == 'S' || sub == 'R') {
                Serial.println("slow/ramp removed; single rate (no-op)");
                break;
            } else {
                Serial.println("usage: P F <us>");
                break;
            }
            persistSaveStepProfile();
            Serial.printf("Profile saved: fast=%uus\n",
                          (unsigned)g_stepDelayFastUs);
            break;
        }
        case 'Y': {
            // Post-valve dwell (renamed from 'V' to free up 'V' for
            // valve angles).
            int v = Serial.parseInt();
            if (v < POST_VALVE_DWELL_MIN_SEC
                || v > POST_VALVE_DWELL_MAX_SEC) {
                Serial.printf("usage: Y <sec %d..%d>\n",
                              POST_VALVE_DWELL_MIN_SEC,
                              POST_VALVE_DWELL_MAX_SEC);
                break;
            }
            g_postValveDwellSec = (uint8_t)v;
            persistSaveDwell();
            Serial.printf("Post-valve dwell set to %u s (saved)\n",
                          (unsigned)g_postValveDwellSec);
            break;
        }
        case 'F': {
            // Reservoir-full volume (mL) -- the UI gauge full-scale.
            // Bare 'F' prints the current value; 'F <ml>' sets + persists.
            // Display-only: does NOT affect dispense/reservoir-low logic.
            // Same wait-then-peek-digit pattern as 'B' so a bare 'F'
            // reports instead of parsing 0.
            uint32_t waitDeadline = millis() + 100;
            while (Serial.available() == 0 && millis() < waitDeadline) {
                delay(2);
            }
            bool haveDigit = false;
            while (Serial.available()) {
                int peek = Serial.peek();
                if (peek == ' ' || peek == '\t') { Serial.read(); continue; }
                if ((peek >= '0' && peek <= '9') || peek == '.') haveDigit = true;
                break;
            }
            if (!haveDigit) {
                Serial.printf("Reservoir full = %.0f mL (%.0f..%.0f)\n",
                              (double)g_reservoirFullMl,
                              (double)RESERVOIR_FULL_ML_MIN,
                              (double)RESERVOIR_FULL_ML_MAX);
                Serial.println("usage: F <mL>");
                break;
            }
            float v = Serial.parseFloat();
            if (v < RESERVOIR_FULL_ML_MIN || v > RESERVOIR_FULL_ML_MAX) {
                Serial.printf("usage: F <mL %.0f..%.0f>\n",
                              (double)RESERVOIR_FULL_ML_MIN,
                              (double)RESERVOIR_FULL_ML_MAX);
                break;
            }
            g_reservoirFullMl = v;
            persistSaveReservoirFull();
            Serial.printf("Reservoir full set to %.0f mL (saved)\n",
                          (double)g_reservoirFullMl);
            renderUI(true);
            break;
        }
        case 'B': {
            // Status LED brightness 0..255 (0 = LED off). Bare 'B'
            // prints the current value rather than setting it -- since
            // 0 is a valid brightness, parseInt()'s 0-on-empty would
            // otherwise make a bare 'B' silently kill the LED. Same
            // wait-then-peek pattern as 'P'.
            uint32_t waitDeadline = millis() + 100;
            while (Serial.available() == 0 && millis() < waitDeadline) {
                delay(2);
            }
            bool haveDigit = false;
            while (Serial.available()) {
                int peek = Serial.peek();
                if (peek == ' ' || peek == '\t') { Serial.read(); continue; }
                if (peek >= '0' && peek <= '9') haveDigit = true;
                break;
            }
            if (!haveDigit) {
                Serial.printf("LED brightness = %u (0..255; 0=off)\n",
                              (unsigned)g_ledBrightness);
                Serial.println("usage: B <0..255>");
                break;
            }
            int v = Serial.parseInt();
            if (v < LED_BRIGHTNESS_MIN || v > LED_BRIGHTNESS_MAX) {
                Serial.printf("usage: B <%d..%d>\n",
                              LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX);
                break;
            }
            g_ledBrightness = (uint8_t)v;
            persistSaveLedBrightness();
            Serial.printf("LED brightness set to %u (saved)\n",
                          (unsigned)g_ledBrightness);
            break;
        }
        case 'D': {
            // Dispense test. Drives runWateringCycle in test mode --
            // skips all load-cell logic and either actuates the valve
            // (D 1) or just emulates the timing (D 0, bare D).
            // parseInt() returns 0 on no input, which makes bare D
            // the safer dry-by-default behavior. Out-of-range values
            // also fall through to dry with a usage hint.
            int v = Serial.parseInt();
            g_testModeWet = (v == 1);
            if (v != 0 && v != 1) {
                Serial.println("usage: D 0 (dry) or D 1 (wet); "
                               "defaulting to dry");
            }
            Serial.printf("Dispense test: %s\n",
                          g_testModeWet ? "WET" : "DRY");
            runDispenseTest();
            break;
        }
        case 'p': printState(); break;
        // R/r are OPEN-LOOP blind motor tests: like the jog, they run
        // regardless of ERROR/encoder health (the "sensors absent, move
        // blind, operator-supervised" tools) and void position trust.
        // rotateContinuous() still refuses while WATERING. Normal closed-
        // loop motion (gotoBucket/water/cal) stays gated -- not touched here.
        case 'R':
            rotateContinuous(+1);
            break;
        case 'r':
            rotateContinuous(-1);
            break;
        case 'J': {
            // OPEN-LOOP BLIND JOG (debug). Bounded, no encoder feedback --
            // usable when the AS5600 is dead/absent to verify coils/
            // mechanics. NOT gated on canRunMotorOps()/encoder health (that
            // is the whole point); only refused while WATERING. Voids
            // position trust -> recal required before normal motion.
            // Usage: J <dir +/-> [steps]   (steps default JOG_DEFAULT_STEPS,
            // capped at JOG_MAX_STEPS_PER_CMD). e.g. 'J 1' fwd, 'J -1' rev,
            // 'J 1 64' fwd 64 steps.
            int dir   = Serial.parseInt();
            int steps = Serial.parseInt();
            if (dir == 0) {
                Serial.printf("usage: J <dir +/-> [steps<=%u] (default %u); "
                              "OPEN-LOOP blind, voids position trust\n",
                              (unsigned)JOG_MAX_STEPS_PER_CMD,
                              (unsigned)JOG_DEFAULT_STEPS);
                break;
            }
            jogOpenLoop(dir, steps);
            renderUI(true);
            break;
        }
        case 'A':
            // Acknowledge / clear fault. Idempotent: harmless if no
            // fault is set. The matching gesture is an extra-long
            // (>=3s) BtnA hold while in ERROR state.
            clearFault("serial 'A'");
            break;
        case 'x':
            // No active cycle to abort; serviceAbort() consumes 'x'
            // during cycles, so reaching here means the system is idle.
            Serial.println("nothing to abort (idle)");
            break;
        case 'X':
            // Demo mode (dry) toggle. handleSerial only runs while the
            // foreground is idle -- the blocking demo loop owns the
            // foreground while running, and serviceAbort() consumes
            // 'x'/'X' as the abort there -- so reaching this case means
            // demo is OFF and we should start a DRY demo. (Wet demo is
            // web-only, per design.) The defensive stop branch covers
            // the impossible-but-cheap "already running" case.
            if (g_demoMode != DemoMode::Off || g_webRequestDemoStart) {
                g_webRequestDemoStop = true;
                g_abortRequested     = true;
                Serial.println("Demo: stop requested via serial 'X'");
            } else if (!deviceIsIdle()) {
                Serial.println("demo: not idle (need READY, no motion in flight)");
            } else {
                g_demoPendingMode     = DemoMode::Dry;
                g_webRequestDemoStart = true;
                Serial.println("Demo (DRY) requested via serial 'X' "
                               "(x/button/web to stop; wet demo via web)");
            }
            break;
        case 'N':
            // Demo Random (DRY) -- the primary motion cadence-test entry.
            // Loops dry cycles, each watering 1..3 random non-skip buckets,
            // with no reservoir/water dependency so it runs indefinitely.
            // Mirrors 'X' (dry all-bucket demo) but with random bucket
            // selection; same toggle/stop semantics. Wet-random is web-only
            // (POST /api/demo/start?mode=random&wet=1).
            if (g_demoMode != DemoMode::Off || g_webRequestDemoStart) {
                g_webRequestDemoStop = true;
                g_abortRequested     = true;
                Serial.println("Demo: stop requested via serial 'N'");
            } else if (!deviceIsIdle()) {
                Serial.println("demo: not idle (need READY, no motion in flight)");
            } else {
                g_demoPendingMode     = DemoMode::Random;
                g_demoRandomWet       = false;   // dry
                g_webRequestDemoStart = true;
                Serial.println("Demo Random (DRY) requested via serial 'N' "
                               "(1..3 random buckets/cycle; x/button/web to stop)");
            }
            break;
        case 'W':
            // Forget saved STA credentials and reboot. After the reboot
            // the auto-AP path re-enters the config portal automatically
            // (no creds stored). Same code path as the runtime 10 s BtnA
            // hold (wipeWifiCredsAndReboot). Never returns.
            wipeWifiCredsAndReboot("serial 'W'");
            break;
        case 'E': {
            // Toggle the scheduler. Bare 'E' toggles. (Explicit values via web API.)
            setSchedulerEnabled(!g_schedulerEnabled, "serial");
            break;
        }
        case 'Z': {
            g_debugFastSchedule = !g_debugFastSchedule;
            uint32_t nowSec = scheduleTimeNow();
            for (int i = 0; i < g_bucketCount; i++) {
                g_lastWateredAt[i] = nowSec;
                persistMarkLastWatered(i);
            }
            Serial.printf("DEBUG fast-schedule %s\n",
                          g_debugFastSchedule
                          ? "ENABLED (intervals = seconds)"
                          : "DISABLED (normal hours)");
            break;
        }
        case 'L': {
            // Spill threshold in degrees -- max tilt for watering.
            // parseFloat returns 0.0 on no input which is correctly
            // out-of-range, so a bare 'L' falls through to the usage
            // hint rather than zeroing the threshold.
            float v = Serial.parseFloat();
            if (v < SPILL_THRESH_MIN || v > SPILL_THRESH_MAX) {
                Serial.printf("usage: L <deg %.0f..%.0f>\n",
                              (double)SPILL_THRESH_MIN,
                              (double)SPILL_THRESH_MAX);
                break;
            }
            g_spillThreshDeg = v;
            persistSaveLevelConfig();
            Serial.printf("Spill threshold = %.1f deg (saved)\n",
                          (double)g_spillThreshDeg);
            break;
        }
        case 'M': {
            float v = Serial.parseFloat();
            if (v < MOTION_THRESH_MIN || v > MOTION_THRESH_MAX) {
                Serial.printf("usage: M <g %.2f..%.2f>\n",
                              (double)MOTION_THRESH_MIN,
                              (double)MOTION_THRESH_MAX);
                break;
            }
            g_motionThreshG = v;
            persistSaveLevelConfig();
            Serial.printf("Motion threshold = %.3f g (saved)\n",
                          (double)g_motionThreshG);
            break;
        }
        case 'O': {
            int v = Serial.parseInt();
            if (v < 0 || v > 5) {
                Serial.println("usage: O <0..5>  "
                               "(0=Z_UP 1=Z_DOWN 2=Y_UP 3=Y_DOWN 4=X_UP 5=X_DOWN)");
                break;
            }
            g_mountOrientation = (MountOrientation)v;
            g_imuFiltSeeded    = false;
            persistSaveMountOrientation();
            Serial.printf("Mount orientation = %s (saved)\n",
                          mountName(g_mountOrientation));
            break;
        }
        case '\r': case '\n': case ' ': break;
        default:
            Serial.printf("unknown '%c' (try '?')\n", (char)c);
            break;
    }
}

// ============================================================
// WiFi / Web UI / OTA
// ============================================================
// All HTTP handling is sync-mode WebServer. Action endpoints that
// would block the request thread for >100 ms (water now, home now)
// queue work via g_webRequestWater / g_webRequestHome flags; the
// READY branch in loop() consumes the flag and runs the motion call
// from the foreground task. Short calls (tare, set-reservoir, dose,
// clear-fault) execute inline. webServer.handleClient() is gated
// off during WATERING so motion timing can't be perturbed by an
// inbound HTTP request.

// Web UI HTML, served from PROGMEM at GET /. Plain HTML+JS, no
// external assets. Polls /api/state every 2 s and posts to
// /api/* for actions. Buttons are disabled while in flight to
// prevent double-fires.
static const char webUiHtml[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pluvio</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/@tabler/icons-webfont@3.19.0/dist/tabler-icons.min.css">
<style>
:root{
  --bg:#f4f1ec;--ink:#1a1a1a;--soft:#6b6864;--rule:#d8d3cb;
  --orange:#c46f02;--obright:#df7e09;--odeep:#af6306;
  --water:#4a90c2;--wpale:#cfe1ec;--alarm:#a04040;--bone:#e8e2d4;
}
*{box-sizing:border-box}
body{font-family:'JetBrains Mono',ui-monospace,Menlo,Consolas,monospace;margin:0;
  background:var(--bg);color:var(--ink);font-size:14px;line-height:1.5;-webkit-font-smoothing:antialiased}
.wrap{max-width:560px;margin:0 auto;padding:14px 16px 40px}
b,strong{font-weight:500}
a{color:var(--orange);text-decoration:none}
a:hover{color:var(--obright)}

/* ---- header ---- */
.hdr{display:flex;align-items:center;gap:10px;flex-wrap:wrap;
  padding-bottom:12px;border-bottom:1px solid var(--rule);margin-bottom:14px}
.brand{display:flex;align-items:center;gap:8px}
.mark{color:var(--orange);display:block}
.brandword{font-weight:500;letter-spacing:.02em;font-size:15px}
.brandword span{color:var(--orange)}
.badge{font-size:10.5px;letter-spacing:.09em;padding:3px 10px;border-radius:11px;
  text-transform:uppercase;font-weight:500}
.badge.READY{background:var(--wpale);color:#2c6890}
.badge.WATERING{background:var(--water);color:#fff}
.badge.ERROR{background:var(--alarm);color:#fff}
.badge.BOOTING{background:var(--bone);color:var(--soft)}
.gear{margin-left:auto;background:none;border:1px solid var(--rule);border-radius:50%;
  width:36px;height:36px;display:flex;align-items:center;justify-content:center;
  color:var(--ink);cursor:pointer;font-size:18px;line-height:1}
.gear:hover{border-color:var(--orange);color:var(--orange)}

/* ---- eyebrow ---- */
.eyebrow{display:flex;align-items:center;gap:8px;font-size:10.5px;letter-spacing:.11em;
  text-transform:uppercase;color:var(--orange);margin:18px 0 8px}
.eyebrow::before{content:"";width:20px;height:1px;background:var(--orange);display:inline-block}

/* ---- banners ---- */
.banner{padding:8px 12px;border-radius:6px;margin:8px 0;font-size:12px;letter-spacing:.04em;
  display:none}
.banner.on{display:block}
#motionBanner{display:none;align-items:center;justify-content:space-between;gap:12px;
  background:var(--alarm);color:#fff;padding:10px 14px;border-radius:8px;margin:8px 0}
#motionBanner.on{display:flex}
#schedBanner{background:#f3e6cf;color:var(--odeep);border:1px solid #e3cda0}
#fastBanner{background:var(--orange);color:#fff}
#levelBanner{background:#f0dcdc;color:var(--alarm);border:1px solid #e3c0c0;font-weight:500}

/* ---- radial diagram ---- */
.diagram{max-width:480px;margin:4px auto 0;width:100%;display:flex;
  flex-direction:column;align-items:center;gap:4px}
.droplet-glyph{display:block;width:28px;height:38px;flex:none}
#radial{width:100%;height:auto;display:block;aspect-ratio:1}
.seg{fill:none;stroke:#d4cdb8;stroke-width:10;stroke-linecap:round;opacity:.7}
.seg.lit{stroke:#3d7fb0;opacity:1}
.skipwedge{fill:none;stroke:#cfc7b6;stroke-linecap:round;stroke-dasharray:2 6;opacity:.7}
.deadzone{fill:none;stroke:#d8d3cb;stroke-linecap:round;stroke-dasharray:3 6;opacity:.7}
.hometick{fill:#b8b2a4}
#selHalo{fill:none;stroke:var(--obright);stroke-width:5;stroke-linecap:round;
  transition:stroke .25s linear}
#cShadow{fill:var(--bone)}
#cDisk{fill:var(--bg);stroke:var(--rule);stroke-width:1}
.wedge{cursor:pointer}

/* ---- arm ---- */
#armRot{transform-box:view-box;transform-origin:200px 200px;transition:transform 250ms linear}
#arm .armcol{transition:fill .25s linear,opacity .25s linear}
#arm.breathe{animation:armBreathe 3.4s ease-in-out infinite}
#arm.pulse{animation:armPulse 1.4s ease-in-out infinite}
@keyframes armBreathe{0%,100%{opacity:1}50%{opacity:.7}}
@keyframes armPulse{0%,100%{opacity:1}50%{opacity:.5}}
#armStem.dashed{fill:none!important;stroke:var(--alarm);stroke-width:2.5;stroke-dasharray:4 4;stroke-linejoin:round;stroke-linecap:round}
#armHalo{fill:var(--wpale);opacity:.6;display:none;transform-box:fill-box;transform-origin:center}
#arm.valveopen #armHalo{display:block;animation:haloPulse 1.6s ease-in-out infinite}
@keyframes haloPulse{0%{transform:scale(1);opacity:.55}50%{transform:scale(1.4);opacity:.12}100%{transform:scale(1);opacity:.55}}

/* ---- selection card ---- */
.selcard{display:flex;align-items:center;gap:10px;border:1px solid var(--rule);border-radius:10px;
  padding:10px 14px;margin:14px auto 0;max-width:480px;background:#faf8f4}
.selcard .plbl{font-size:10.5px;letter-spacing:.1em;text-transform:uppercase;color:var(--soft)}
.selcard .pnum{font-size:15px;font-weight:500}
.selcard select{margin-left:auto;min-width:150px;text-align:center}
.selhint{font-size:10px;letter-spacing:.06em;text-transform:uppercase;color:var(--soft);
  max-width:480px;margin:6px auto 0;text-align:center}
select,input[type=number],input[type=text]{font-family:'JetBrains Mono',monospace;font-size:12px;
  background:#fff;color:var(--ink);border:1px solid var(--rule);border-radius:6px;padding:5px 7px}
input[type=range]{accent-color:var(--orange);flex:1;min-width:150px}

/* ---- action row ---- */
.actions{display:flex;flex-wrap:wrap;gap:10px 24px;margin:24px 0 6px}
.act{background:none;border:0;border-bottom:1px solid var(--ink);color:var(--ink);
  font-family:'JetBrains Mono',monospace;font-size:11px;letter-spacing:.08em;text-transform:uppercase;
  padding:5px 1px;cursor:pointer}
.act:hover{border-bottom-color:var(--orange);color:var(--orange)}
.act:disabled{opacity:.32;cursor:default;border-bottom-color:var(--rule);color:var(--soft)}
.act.go{border-bottom-color:var(--obright);color:var(--odeep)}
.act.go:hover{border-bottom-color:var(--orange)}
.act.dn:hover{border-bottom-color:var(--alarm);color:var(--alarm)}

/* ---- fault ---- */
#fault{display:none;background:#f0dcdc;color:var(--alarm);border:1px solid #e3c0c0;
  padding:10px 14px;border-radius:8px;margin:14px 0;font-weight:500}
#fault.on{display:block}

/* ---- details panels ---- */
details.panel{border:1px solid var(--rule);border-radius:8px;margin:10px 0;background:#faf8f4}
details.panel>summary{cursor:pointer;padding:11px 14px;font-size:11px;letter-spacing:.09em;
  text-transform:uppercase;color:var(--soft);list-style:none}
details.panel>summary::-webkit-details-marker{display:none}
details.panel>summary::before{content:"\25B8";color:var(--orange);margin-right:8px;font-size:9px}
details.panel[open]>summary::before{content:"\25BE"}
details.panel[open]>summary{color:var(--ink);border-bottom:1px solid var(--rule)}
.pbody{padding:12px 14px}
.warn{color:var(--odeep);font-size:11.5px;margin:4px 0 10px}
.row{display:flex;align-items:center;gap:10px;margin:8px 0;flex-wrap:wrap}
.row label{min-width:120px;font-size:12px;color:var(--soft)}
.hint{font-size:11px;color:var(--soft);margin:-2px 0 10px 130px}
.btnrow{display:flex;flex-wrap:wrap;gap:10px 20px}
hr{border:0;border-top:1px solid var(--rule);margin:14px 0}

/* ---- IMU bubble ---- */
#bubbleSvg{background:#fff;border-radius:50%;border:1.5px solid var(--rule)}
.imuline{font-size:12px;line-height:1.7}
.imuline .sub{color:var(--soft);font-size:11px}

/* ---- log ---- */
.logctl{display:flex;gap:10px;align-items:center;margin:22px 0 6px;font-size:11px;
  letter-spacing:.08em;text-transform:uppercase;color:var(--soft)}
.logctl button{background:none;border:1px solid var(--rule);border-radius:6px;padding:4px 10px;
  font-family:'JetBrains Mono',monospace;font-size:10px;letter-spacing:.06em;text-transform:uppercase;
  color:var(--ink);cursor:pointer}
.logctl button:hover{border-color:var(--orange);color:var(--orange)}
.logbox{background:#1a1a1a;color:#d7cfa8;font-family:ui-monospace,Consolas,Menlo,monospace;
  font-size:11.5px;height:280px;overflow-y:auto;padding:10px 12px;border-radius:8px;
  border:1px solid var(--rule);white-space:pre-wrap;word-break:break-all}

/* ---- footer ---- */
.foot{display:flex;align-items:center;gap:12px;margin-top:30px;padding-top:16px;
  border-top:1px solid var(--rule);color:var(--soft);font-size:11px;letter-spacing:.04em}
.foot .mark{color:var(--orange)}
.foot .tag{font-style:normal}
.foot .footdev{margin-left:auto;font-size:10.5px}

/* ---- settings drawer ---- */
.drawer-backdrop{position:fixed;inset:0;background:rgba(26,26,26,0.4);opacity:0;
  pointer-events:none;transition:opacity 240ms;z-index:40}
.drawer-backdrop.open{opacity:1;pointer-events:auto}
.drawer{position:fixed;top:0;right:0;height:100vh;width:420px;max-width:100vw;
  background:#faf8f4;border-left:1px solid var(--rule);box-shadow:-8px 0 24px rgba(26,26,26,0.12);
  transform:translateX(100%);transition:transform 240ms ease-out;z-index:50;
  display:flex;flex-direction:column}
.drawer.open{transform:translateX(0)}
.drawer-hdr{display:flex;align-items:center;justify-content:space-between;
  padding:16px 18px 12px;border-bottom:1px solid var(--rule)}
.drawer-hdr h2{margin:0;font-size:15px;font-weight:500}
.drawer-hdr button{background:none;border:0;color:var(--ink);cursor:pointer;font-size:20px;line-height:1}
.drawer-hdr button:hover{color:var(--orange)}
.drawer-body{overflow-y:auto;padding:10px 16px 28px;flex:1}
.drawer .panel{margin:8px 0}
details.panel>summary .ti{margin-right:6px;color:var(--orange)}
.sysblock{margin-top:18px;padding-top:14px;border-top:1px solid var(--rule)}
.syslbl{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:var(--soft);
  font-weight:500;margin-bottom:8px}
.sysrow{display:flex;font-size:11.5px;margin:5px 0}
.sysrow .k{min-width:110px;color:var(--soft)}
.sysrow .v{color:var(--ink)}

@media (max-width:520px){
  .drawer{width:100vw}
  .wrap{padding:12px 12px 32px}
  .actions{gap:10px 18px}
}
/* Non-blocking toast: replaces alert()/confirm(), which iOS Safari
   suppresses (breaking dialog-gated flows like calibrate). */
#toast{position:fixed;left:50%;bottom:24px;transform:translateX(-50%) translateY(20px);
  max-width:92vw;padding:12px 18px;border-radius:10px;font-size:14px;font-weight:600;
  color:#fff;background:#2f8f4f;box-shadow:0 6px 20px rgba(0,0,0,.35);z-index:9999;
  opacity:0;pointer-events:none;transition:opacity .2s,transform .2s;text-align:center}
#toast.on{opacity:1;transform:translateX(-50%) translateY(0)}
#toast.err{background:#a04040}
</style></head><body>
<div id="toast" role="status" aria-live="polite"></div>
<div class="wrap">

<header class="hdr">
  <div class="brand">
    <svg class="mark" width="27" height="24" viewBox="-354 -232 708 631" aria-hidden="true"><path fill-rule="evenodd" fill="currentColor" d="M -345 -135.6773 A 63.5085 63.5085 0 0 1 -290 -230.9401 L 290 -230.9401 A 63.5085 63.5085 0 0 1 345 -135.6773 L 55 366.6174 A 63.5085 63.5085 0 0 1 -55 366.6174 Z M 0 -152.559 L 86.4739 -23.1368 A 104 104 0 1 1 -86.4739 -23.1368 Z"/></svg>
    <span class="brandword">Pluvio <span>Drip</span></span>
    <span id="st" class="badge BOOTING">BOOTING</span>
  </div>
  <button id="gearBtn" class="gear" title="Settings" aria-label="Settings"><i class="ti ti-settings" aria-hidden="true"></i></button>
</header>

<div id="motionBanner">
  <div>
    <strong id="motionLabel">Motion running</strong>
    <div id="motionSub" style="font-size:11px;opacity:.9;margin-top:2px">Tap Stop to abort</div>
  </div>
</div>
<div id="schedBanner" class="banner">Scheduler paused &mdash; automatic cycles are off. Manual controls still work.</div>
<div id="fastBanner" class="banner">Fast-schedule mode: intervals are seconds, not hours.</div>
<div id="levelBanner" class="banner"></div>

<div class="eyebrow">Carousel</div>
<div class="diagram">
<svg id="radial" viewBox="0 0 400 400" role="img" aria-label="Carousel of 13 plants with a rotating arm">
  <!-- reservoir tank: full-dial bone with a wavy water surface, clipped
       to the dial circle. Bucket bands render on top (see buildTankPaths). -->
  <defs>
    <clipPath id="dialClip"><circle cx="200" cy="200" r="200"/></clipPath>
    <clipPath id="waterClip"><path id="waterClipPath" d=""/></clipPath>
  </defs>
  <g id="tankBg" clip-path="url(#dialClip)">
    <circle cx="200" cy="200" r="200" fill="#f0e9d8"/>
    <g clip-path="url(#waterClip)">
      <circle id="tankWater" cx="200" cy="200" r="200" fill="#9bc4dd"/>
    </g>
    <path id="tankSurface" fill="none" stroke="#6fb0d8" stroke-width="2" opacity="0.85"/>
  </g>
  <!-- bucket wedges (built by JS; layer centred at 200,200 so each
       wedge child is a bare rotate()) -->
  <g id="wedgeLayer" transform="translate(200,200)"></g>
  <!-- dead zone + home tick (built by JS) -->
  <g id="deadLayer" transform="translate(200,200)"></g>
  <!-- selection halo -->
  <path id="selHalo" d=""></path>
  <!-- centre disk shadow + face -->
  <circle id="cShadow" cx="200" cy="200" r="73"></circle>
  <circle id="cDisk" cx="200" cy="200" r="70"></circle>
  <!-- centre arc text: top (brand orange) + bottom (muted). Paths are
       the brief's r=55 arcs, offset to the (200,200) dial centre. The
       arm is drawn last and sweeps over this text. -->
  <defs>
    <path id="topArcCentre" d="M 148.3 181.2 A 55 55 0 0 1 251.7 181.2" fill="none"></path>
    <path id="botArcCentre" d="M 148.3 218.8 A 55 55 0 0 0 251.7 218.8" fill="none"></path>
  </defs>
  <text font-family="'JetBrains Mono',monospace" font-size="10" font-weight="500" letter-spacing="0.12em" fill="#c46f02" style="text-transform:uppercase"><textPath id="topArcText" href="#topArcCentre" startOffset="50%" text-anchor="middle"></textPath></text>
  <text font-family="'JetBrains Mono',monospace" font-size="10" font-weight="400" letter-spacing="0.1em" fill="#6b6864"><textPath id="botArcText" href="#botArcCentre" startOffset="50%" text-anchor="middle"></textPath></text>
  <!-- reservoir droplet: sits in the home dead-zone at 6 o'clock, inside
       the dial. Drawn after the centre disk + arc text and before the arm
       so the arm sweeps over it (consistent with other dial elements). -->
  <g transform="translate(200,330)">
    <path id="droplet" fill="#4a90c2" stroke="#3d7fb0" stroke-width="0.8"
          d="M 0 -16 C 10 -4, 14 5, 14 11 C 14 19, 7 24, 0 24 C -7 24, -14 19, -14 11 C -14 5, -10 -4, 0 -16 Z"/>
    <ellipse cx="-5" cy="7" rx="3.5" ry="6" fill="#fff" opacity="0.35"/>
  </g>
  <!-- arm (drawn last; sweeps over disk + arc text). Geometry copied
       verbatim from web_ui_assets/arm.svg (locked v2). -->
  <g id="armRot">
    <g id="arm">
      <circle id="armHalo" cx="200" cy="130" r="14" fill="#cfe1ec" opacity="0.6" style="display:none"/>
      <circle id="armPlatter" class="armcol" cx="200" cy="200" r="45" fill="#4a90c2"/>
      <rect id="armStem" class="armcol" x="192" y="120" width="16" height="80" rx="8" ry="8" fill="#4a90c2"/>
      <rect id="armDuct" x="197" y="128" width="6" height="50" rx="3" fill="#ffffff"/>
      <path id="armRing" d="M 200 200 m -22 0 a 22 22 0 1 0 44 0 a 22 22 0 1 0 -44 0 Z M 200 200 m -14 0 a 14 14 0 1 0 28 0 a 14 14 0 1 0 -28 0 Z" fill="#ffffff" fill-rule="evenodd"/>
      <circle id="armDropAnim" cx="200" cy="200" r="3" fill="#6fb0d8" opacity="0" style="display:none">
        <animate attributeName="cy" from="200" to="131" dur="1.4s" repeatCount="indefinite"/>
        <animate attributeName="opacity" values="0;1;1;0" keyTimes="0;0.15;0.85;1" dur="1.4s" repeatCount="indefinite"/>
      </circle>
    </g>
  </g>
</svg>
</div>

<div class="selcard">
  <span class="plbl">Plant</span>
  <span class="pnum" id="selPlant">1</span>
  <select id="selInterval">
    <option value="12">every 12h</option>
    <option value="24">daily</option>
    <option value="48">every 2d</option>
    <option value="72">every 3d</option>
    <option value="120">every 5d</option>
    <option value="168">weekly</option>
    <option value="240">every 10d</option>
    <option value="336">every 2w</option>
    <option value="504">every 3w</option>
  </select>
</div>
<div class="selhint">Tap dial to adjust dose</div>

<div id="fault"></div>

<div class="actions">
  <button id="bWater" class="act go"><i class="ti ti-droplet" aria-hidden="true"></i> Water Now</button>
  <button id="bWaterSel" class="act go"><i class="ti ti-droplet-filled" aria-hidden="true"></i> Water Selected</button>
  <button id="bStop" class="act dn"><i class="ti ti-player-stop" aria-hidden="true"></i> Stop</button>
  <button id="bSched" class="act"><i class="ti ti-player-pause" aria-hidden="true"></i> Pause Schedule</button>
  <button id="bClear" class="act dn" style="display:none"><i class="ti ti-alert-triangle-off" aria-hidden="true"></i> Clear Fault</button>
</div>

<div id="drawerBackdrop" class="drawer-backdrop"></div>
<aside id="drawer" class="drawer" aria-label="Settings">
  <header class="drawer-hdr">
    <h2>Settings</h2>
    <button id="drawerClose" aria-label="Close"><i class="ti ti-x" aria-hidden="true"></i></button>
  </header>
  <div class="drawer-body">

  <!-- SECTION 1: Reservoir & scale -->
  <details class="panel" open>
    <summary><i class="ti ti-flask"></i>Reservoir &amp; scale</summary>
    <div class="pbody">
      <div class="row">
        <label>Scale</label>
        <span id="scaleStat" style="min-width:40px;font-size:12px">--</span>
      </div>
      <div class="row">
        <label>Live weight</label>
        <span id="liveWeight" style="font-size:12px">--</span>
      </div>
      <div class="row">
        <label>Live raw</label>
        <span id="rawCounts" style="font-size:12px;color:var(--soft)" title="Raw load-cell counts (uncalibrated). Watch these change as you add mass to confirm the cell works.">--</span>
      </div>
      <!-- Calibration wizard: shown only when the scale is uncalibrated
           (factory step). Locked away once calibrated. -->
      <div id="calWizard">
        <div class="btnrow">
          <button id="bZero" class="act" title="Step 1: cell EMPTY, capture the zero"><i class="ti ti-scale"></i> 1. Capture Zero (empty)</button>
        </div>
        <div class="row">
          <label>2. Calibrate</label>
          <input id="calGrams" type="number" min="1" step="0.1" placeholder="known g" style="width:90px">
          <button id="bCal" class="act" title="Step 2: known mass on the cell, enter its grams, click"><i class="ti ti-adjustments-horizontal"></i> Calibrate</button>
          <span id="calStat" style="font-size:12px;color:var(--soft)"></span>
        </div>
        <div class="hint">Factory step: (1) cell empty, Capture Zero. (2) add a known mass, enter grams, Calibrate. Zero and gap are saved together. Gap value: <span id="cpg">--</span>.</div>
      </div>
      <!-- Factory reset: re-exposes the wizard. -->
      <div class="row" id="calResetRow" style="display:none">
        <label>Calibration</label>
        <span style="font-size:12px;color:var(--soft)">Gap <span id="cpg2">--</span></span>
        <button id="bScaleReset" class="act dn" title="Clear calibration (zero + gap) and recalibrate"><i class="ti ti-rotate-2"></i> Reset Calibration</button>
      </div>
    </div>
  </details>

  <!-- SECTION 2: Carousel & motion -->
  <details class="panel">
    <summary><i class="ti ti-rotate"></i>Carousel &amp; motion</summary>
    <div class="pbody">
      <div class="warn">Motor ops move the rig and take time. Do not run during active watering.</div>
      <div class="btnrow">
        <button id="bHome" class="act"><i class="ti ti-home"></i> Home</button>
        <button id="bRedisc" class="act" title="Drive into both mechanical endstops to learn the AS5600 zero offset and direction sign. ~10s."><i class="ti ti-target"></i> Calibrate Endstops</button>
      </div>
      <div class="row">
        <label>Jog stepper</label>
        <button id="bRotFwd" class="act" title="Press and hold to jog forward; release to stop"><i class="ti ti-rotate-clockwise"></i> Jog Fwd</button>
        <button id="bRotRev" class="act" title="Press and hold to jog reverse; release to stop"><i class="ti ti-rotate"></i> Jog Rev</button>
      </div>
    </div>
  </details>

  <!-- SECTION 3: Tuning -->
  <details class="panel">
    <summary><i class="ti ti-adjustments-horizontal"></i>Tuning</summary>
    <div class="pbody">
      <div class="warn">Changes persist across reboots.</div>
      <div class="row">
        <label>Valve open angle</label>
        <input id="voSlider" type="range" min="0" max="180" step="1">
        <input id="voInput" type="number" min="0" max="180" style="width:64px">
      </div>
      <div class="row">
        <label>Valve closed angle</label>
        <input id="vcSlider" type="range" min="0" max="180" step="1">
        <input id="vcInput" type="number" min="0" max="180" style="width:64px">
      </div>
      <div class="hint">Servo channel 1 drives the external H-bridge that switches the valve. Defaults open=180, closed=0; flip if your wiring is reversed.</div>
      <div class="row">
        <label>Valve dwell (s)</label>
        <input id="dwellSlider" type="range" min="1" max="10" step="1">
        <input id="dwellInput" type="number" min="1" max="10" step="1" style="width:64px">
      </div>
      <div class="hint">Wait time after the valve closes for water in the line to finish draining. Increase if you see overflow between buckets.</div>
      <div class="row">
        <label>Status LED brightness</label>
        <input id="ledSlider" type="range" min="0" max="255" step="1">
        <input id="ledInput" type="number" min="0" max="255" step="1" style="width:64px">
      </div>
      <div class="hint">Master brightness of the front status LED (0&ndash;255). 0 turns the LED off entirely. Colour and pattern follow device state automatically.</div>
      <div class="row">
        <label>Step delay (us)</label>
        <input id="sfSlider" type="range" min="100" max="10000" step="50">
        <input id="sfInput" type="number" min="100" max="10000" step="50" style="width:80px">
      </div>
      <div class="hint">Time between step pulses. Lower = faster cruise but less torque. Single rate; default 3000us (~333 steps/sec).</div>
      <div class="row">
        <label>Motor direction</label>
        <span id="motorInvStat" style="min-width:60px;font-size:12px">--</span>
        <button id="bMotorInv" class="act"><i class="ti ti-arrows-left-right"></i> Flip Direction</button>
      </div>
      <div class="hint">Use if homing/discovery moves the carousel the wrong way after a motor swap. Auto-detect at discovery handles the common case; this is the manual override.</div>
      <hr>
      <div class="row">
        <label>Reset WiFi</label>
        <button id="bResetWifi" class="act dn"><i class="ti ti-wifi-off"></i> Reset WiFi &amp; Reboot</button>
      </div>
      <div class="hint">Forgets saved WiFi and reboots into AP setup mode. You will lose web access until reconfigured.</div>
      <div class="row">
        <label>Debug fast schedule</label>
        <button id="bFastSched" class="act"><i class="ti ti-clock-bolt"></i> Toggle</button>
        <span id="fastStat" style="font-size:12px;color:var(--soft)"></span>
      </div>
      <div class="hint">Test mode: intervals interpreted as SECONDS instead of hours. "Daily" fires every 24 seconds. Resets to OFF on every reboot.</div>
      <hr id="imuTuneHr">
      <div id="installPanel" style="display:none">
        <div style="display:flex;align-items:center;gap:18px;flex-wrap:wrap">
          <svg id="bubbleSvg" width="100" height="100" viewBox="0 0 100 100">
            <circle cx="50" cy="50" r="45" fill="none" stroke="#d8d3cb" stroke-width="1"/>
            <circle cx="50" cy="50" r="30" fill="none" stroke="#d8d3cb" stroke-width="1"/>
            <line x1="10" y1="50" x2="90" y2="50" stroke="#d8d3cb" stroke-width="1"/>
            <line x1="50" y1="10" x2="50" y2="90" stroke="#d8d3cb" stroke-width="1"/>
            <circle id="bubble" cx="50" cy="50" r="8" fill="#4a90c2" opacity=".85"/>
          </svg>
          <div class="imuline">
            <div>Left/Right <b id="imuPitch">--</b>&deg;</div>
            <div>Forward/Back <b id="imuRoll">--</b>&deg;</div>
            <div>Motion <b id="imuMotion">--</b> g</div>
            <div class="sub">IMU <span id="imuType">--</span>, mount <span id="imuMountLbl">--</span></div>
            <div class="sub">raw (<span id="imuAxRaw">--</span>, <span id="imuAyRaw">--</span>, <span id="imuAzRaw">--</span>) g</div>
          </div>
        </div>
        <div class="hint" style="margin-left:0;margin-top:10px">
          Gravity is the reference &mdash; no zero calibration. Install the rig level. The bubble shows current tilt; it turns red past the threshold. Hold the rig in the intended level pose and the "up" axis should read ~+1.0 g.
        </div>
      </div>
      <div id="imuTuneRows">
        <div class="row">
          <label>Spill threshold (max tilt)</label>
          <input id="ltSlider" type="range" min="1" max="30" step="0.5">
          <input id="ltInput" type="number" min="1" max="30" step="0.5" style="width:64px"> &deg;
        </div>
        <div class="row">
          <label>Motion threshold</label>
          <input id="mtSlider" type="range" min="0.02" max="1.0" step="0.01">
          <input id="mtInput" type="number" min="0.02" max="1.0" step="0.01" style="width:64px"> g
        </div>
        <div class="hint">Cycles <b>defer</b> (retry next tick) rather than run when the rig is tilted past the spill threshold, or while the vibration metric exceeds the motion threshold. Default 10&deg; / 0.15 g. Motor operations are unaffected &mdash; only dispensing is gated.</div>
        <div class="row">
          <label>Mount orientation</label>
          <select id="moSelect">
            <option value="Z_UP">Z_UP (face up)</option>
            <option value="Z_DOWN">Z_DOWN (face down)</option>
            <option value="Y_UP">Y_UP</option>
            <option value="Y_DOWN">Y_DOWN</option>
            <option value="X_UP">X_UP</option>
            <option value="X_DOWN">X_DOWN</option>
          </select>
        </div>
        <div class="hint">Which board axis points UP when the rig is installed level. Verify with the raw axes above.</div>
      </div>
    </div>
  </details>

  <!-- SECTION 4: Diagnostics -->
  <details class="panel">
    <summary><i class="ti ti-test-pipe"></i>Diagnostics</summary>
    <div class="pbody">
      <div class="warn">Tests run motors and may take 30&ndash;60s. Do not run during active watering.</div>
      <div class="btnrow">
        <button id="bDtestDry" class="act" title="Cycles all buckets, emulates dose timing, valve stays CLOSED. Safe for empty rig."><i class="ti ti-test-pipe"></i> Dispense Test (Dry)</button>
        <button id="bDtestWet" class="act dn" title="Cycles all buckets, emulates dose timing, valve ACTUATES. Water will flow."><i class="ti ti-test-pipe"></i> Dispense Test (Wet)</button>
        <button id="bValve" class="act" title="Toggle the valve open/closed for bench testing. Forbidden during a watering cycle.">Valve: --</button>
      </div>
      <hr>
      <div style="font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:var(--soft);margin-bottom:4px">Demo Mode <span id="demoStat" style="text-transform:none;letter-spacing:0"></span></div>
      <div class="warn">Loops the dispense test continuously to show how the device works. The scheduler auto-pauses while the demo runs and resumes on exit. Any abort or fault stops the demo.</div>
      <div class="btnrow">
        <button id="bDemoDry" class="act" title="Loop the dry dispense test continuously (valve stays CLOSED)."><i class="ti ti-player-track-next"></i> Demo Dry</button>
        <button id="bDemoWet" class="act dn" title="Loop the wet dispense test continuously. Water WILL flow on every non-skip bucket."><i class="ti ti-player-track-next"></i> Demo Wet</button>
        <button id="bDemoRand" class="act" title="Loop continuously (dry), watering 1-3 randomly chosen buckets each cycle. Motion cadence test - no water."><i class="ti ti-player-track-next"></i> Demo Random</button>
        <button id="bDemoStop" class="act dn" title="Stop the demo and resume normal scheduled mode."><i class="ti ti-player-stop"></i> Stop Demo</button>
      </div>
      <div class="row">
        <label>Pause between cycles (s)</label>
        <input id="demoPauseInput" type="number" min="1" max="10" step="1" style="width:64px">
        <button id="bDemoPause" class="act">Set Pause</button>
      </div>
    </div>
  </details>

  <!-- SECTION 5: Live log -->
  <details class="panel" id="logSection">
    <summary><i class="ti ti-terminal-2"></i>Live log</summary>
    <div class="pbody">
      <div class="logctl">
        <button id="bPause">Pause</button>
        <button id="bClearLog">Clear</button>
        <span id="logStat"></span>
      </div>
      <pre id="log" class="logbox"></pre>
    </div>
  </details>

  <!-- System info -->
  <div class="sysblock">
    <div class="syslbl">System</div>
    <div class="sysrow"><span class="k">Device</span><span class="v" id="sysDev">&mdash;</span></div>
    <div class="sysrow"><span class="k">Name</span><span class="v"><input id="nameI" type="text" maxlength="32" placeholder="friendly-name" style="width:120px"> <button id="nameBtn" class="act">Set</button></span></div>
    <div class="sysrow"><span class="k">IP / WiFi</span><span class="v"><span id="sysIp">&mdash;</span> &middot; <b id="rssi">--</b> dBm</span></div>
    <div class="sysrow"><span class="k">Uptime</span><span class="v" id="up">--</span></div>
    <div class="sysrow"><span class="k">Arm angle</span><span class="v"><b id="armAng">--</b>&deg; <span id="armCal" style="color:var(--soft);font-size:10.5px"></span></span></div>
    <div class="sysrow"><span class="k">Encoder</span><span class="v" id="encStat">--</span></div>
    <div class="sysrow"><span class="k">Firmware</span><span class="v">v<span id="sysFw">&mdash;</span> &middot; <a href="/update">Update</a></span></div>
  </div>

  </div>
</aside>

<footer class="foot">
  <svg class="mark" width="34" height="30" viewBox="-354 -232 708 631" aria-hidden="true"><path fill-rule="evenodd" fill="currentColor" d="M -345 -135.6773 A 63.5085 63.5085 0 0 1 -290 -230.9401 L 290 -230.9401 A 63.5085 63.5085 0 0 1 345 -135.6773 L 55 366.6174 A 63.5085 63.5085 0 0 1 -55 366.6174 Z M 0 -152.559 L 86.4739 -23.1368 A 104 104 0 1 1 -86.4739 -23.1368 Z"/></svg>
  <span class="tag">Rain, with a brain.</span>
  <span class="footdev"><a href="/update">Firmware Update</a> &middot; <span id="footDev"></span></span>
</footer>

</div>
<script>
var $=function(i){return document.getElementById(i)};
var inflight={};
var BUCKETS=13, SLOT=360/14, FIRST=180+SLOT, GAP=6.5, HALF_DEG=(SLOT-GAP)/2, FIL=2, CX=200, CY=200;
var BAND_RADII=[90,104,118,132,146,160], BAND_THICK=10;
var TANK_AMP=5, TANK_CYCLES=1.5, TANK_SAMPLES=60;
var g_sel=0, g_curDose=0;
var FAULT_FRIENDLY={NO_FLOW:'NO FLOW',RESERVOIR_EMPTY:'EMPTY',
  ENCODER_LOST:'ENCODER LOST',ROTOR_STALL:'STALLED'};

// Arm rotation is unwrapped so the CSS transition always animates the
// shorter path of a single real tick. CSS rotate() has no rotation
// history, so feeding it the raw firmware angle makes a 13->home->1
// sweep (e.g. 514deg then 206deg) animate the wrong way. We track the
// last applied angle as an unbounded number and map each new reading to
// the multiple-of-360 nearest it, keeping consecutive deltas <=180deg.
var armAngleApplied=null;   // running unwrapped angle (may exceed 360 / go negative)
var armWasTrusted=true;     // last value of s.armPositionTrusted
function applyArmAngle(s){
  var raw=s.armAngleDeg;
  if(typeof raw!=='number'||raw<0||s.endstopsCalibrated===false){
    // Uncalibrated or unknown -- leave the arm where it is.
    return;
  }
  // Translate encoder-space angle into dial-space (SVG 0deg = 12 o'clock,
  // sweeping clockwise). The CCW endstop sits BUCKET_ENDSTOP_OFFSET_DEG
  // before bucket 1's dial centre; arc position grows clockwise from there.
  var zero=s.encoderZeroOffsetDeg;
  var sign=s.bucketAngleStepPositive?1:-1;
  var firstBucketDial=s.firstBucketCentreDeg;
  var endstopOffset=7.4;
  var arcPos=(raw-zero)*sign;
  while(arcPos<0)arcPos+=360;
  while(arcPos>=360)arcPos-=360;
  var deg=(firstBucketDial-endstopOffset)+arcPos;
  while(deg>=360)deg-=360;
  var snap=false;
  // Snap (no animation) when leaving the untrusted state -- the firmware
  // just re-identified rotor position; the physical arm did not move.
  if(s.armPositionTrusted===true&&armWasTrusted===false){snap=true;}
  armWasTrusted=(s.armPositionTrusted!==false);

  var target;
  if(armAngleApplied===null||snap){
    target=deg;   // first application or snap: no unwrap
  } else {
    // Pick the rotation of deg (deg, deg+/-360, ...) closest to the last
    // applied angle. Bounded loop guards against an out-of-range reading.
    target=deg;
    var guard=0;
    while(target-armAngleApplied> 180&&guard++<4)target-=360;
    while(target-armAngleApplied< -180&&guard++<4)target+=360;
  }

  var rot=document.getElementById('armRot');
  if(snap){
    // Kill the transition for one frame so the snap is instantaneous,
    // then restore. The reflow read forces the browser to apply the
    // no-transition transform before re-enabling, otherwise both writes
    // coalesce and it animates anyway.
    rot.style.transition='none';
    rot.style.transform='rotate('+target+'deg)';
    void rot.offsetHeight;       // force reflow
    rot.style.transition='';     // back to stylesheet default (250ms linear)
  } else {
    rot.style.transform='rotate('+target+'deg)';
  }
  armAngleApplied=target;
}

function fmtT(ms){var s=ms/1000|0,h=s/3600|0,m=s%3600/60|0;return h+'h '+(m<10?'0':'')+m+'m'}
function fmtNext(sec){
  if(sec<=0)return 'now';
  if(window._fastMode)return sec+'s';
  var h=sec/3600|0,m=sec%3600/60|0;
  return h?h+'h '+(m<10?'0':'')+m+'m':m+'m '+(sec%60)+'s';
}
function jget(u){return fetch(u).then(function(r){return r.json()})}
function jpost(u){return fetch(u,{method:'POST'}).then(function(r){return r.json()})}

/* Non-blocking toast. iOS Safari suppresses alert()/confirm() (and kills
   them entirely in a home-screen web app), which silently broke every
   dialog-gated flow -- most visibly calibrate. Use this everywhere
   instead. */
var _toastT=0;
function toast(msg,err){
  var t=$('toast');if(!t)return;
  t.textContent=msg;t.className='on'+(err?' err':'');
  clearTimeout(_toastT);
  _toastT=setTimeout(function(){t.className=err?'err':'';},err?5000:3000);
}
/* fetch with a hard timeout so a stalled request (single-threaded ESP32
   server being polled concurrently by Safari) can never hang a button
   forever. Rejects on timeout -> btnAct's finally always re-enables. */
function fetchT(u,opts,ms){
  return new Promise(function(res,rej){
    var done=false;
    var to=setTimeout(function(){if(!done){done=true;rej(new Error('timeout'));}},ms||12000);
    fetch(u,opts).then(function(r){if(!done){done=true;clearTimeout(to);res(r);}},
                       function(e){if(!done){done=true;clearTimeout(to);rej(e);}});
  });
}
function btnAct(b,p){
  if(inflight[b.id])return;
  inflight[b.id]=true;b.disabled=true;
  var ot=b.innerHTML;b.textContent='...';
  /* Safety net: even if p()'s promise never settles, force the button
     back to usable so it can never freeze on '...' the way it did on
     iOS. 15s > fetchT's 12s timeout, so a real network error wins. */
  var watchdog=setTimeout(function(){
    if(inflight[b.id]){inflight[b.id]=false;b.innerHTML=ot;b.disabled=false;toast('No response — try again',true);}
  },15000);
  Promise.resolve().then(p).then(function(r){
    if(r&&r.ok===false)toast('Failed: '+(r.reason||'unknown'),true);
  }).catch(function(e){toast('Error: '+e,true);})
  .then(function(){
    clearTimeout(watchdog);
    inflight[b.id]=false;
    b.innerHTML=ot;
    b.disabled=false;
    setTimeout(refresh,200);
  });
}
function bind(id,fn){var b=$(id);if(b)b.onclick=function(){btnAct(b,fn)}}
bind('bWater',function(){return jpost('/api/water')});
/* Water Selected: dispense ONLY the web-selected bucket (g_selected) via
   the existing /api/water/bucket Layer-2 hook. Separate from "Water Now"
   (force-all). The selected bucket may differ from where the arm is
   parked, so the success toast names the exact plant. Preconditions
   mirror the endpoint (~main.cpp:8170) so the button never fails
   silently: not-READY, no/invalid selection, and skip-dose are caught
   here with an explanatory alert before the POST. */
(function(){
  var b=$('bWaterSel'); if(!b)return;
  b.onclick=function(){
    var s=window._lastState||{};
    var sel=s.selected||0, n=sel+1;
    var bk=(s.buckets&&s.buckets[sel])||null;
    if(s.state!=='READY'){alert('Not ready — wait for the current cycle to finish or clear any fault.');return;}
    if(!bk){alert('No plant selected.');return;}
    if(bk.doseMl===0){alert('Plant '+n+' is set to skip (no dose). Set a dose first.');return;}
    btnAct(b,function(){
      return jpost('/api/water/bucket?n='+n).then(function(r){
        if(r&&r.ok)alert('Watering plant '+n);
        return r;   // btnAct still alerts on ok===false (reason)
      });
    });
  };
})();
bind('bAbort',function(){return jpost('/api/abort')});
bind('bHome',function(){return jpost('/api/home')});
bind('bZero',function(){
  return jpost('/api/scale/zero').then(function(r){
    /* Non-blocking confirmation (no alert -- iOS suppresses it). The
       calStat line + toast tell the operator the zero landed. */
    if(r&&r.ok)toast('Zero captured — add the known mass, enter grams, Calibrate');
    return r;
  });
});
/* Reset is destructive, so guard it -- but with a two-tap arm, NOT
   confirm() (which iOS suppresses -> silent no-op). First tap arms;
   second tap within 4s actually resets. */
var _resetArmed=false,_resetT=0;
bind('bScaleReset',function(){
  if(!_resetArmed){
    _resetArmed=true;
    toast('Tap Reset again within 4s to confirm',true);
    clearTimeout(_resetT);
    _resetT=setTimeout(function(){_resetArmed=false;},4000);
    return Promise.resolve({ok:true});
  }
  _resetArmed=false;clearTimeout(_resetT);
  return jpost('/api/scale/reset').then(function(r){
    if(r&&r.ok)toast('Calibration reset — capture zero to start over');
    return r;
  });
});
bind('bClear',function(){return jpost('/api/clearfault')});
bind('bRedisc',function(){return jpost('/api/calibrate')});
bind('bSched',function(){return jpost('/api/scheduler')});
bind('bDtestDry',function(){return jpost('/api/test/dispense')});
bind('bDtestWet',function(){return jpost('/api/test/dispense?wet=1')});
bind('bValve',function(){return jpost('/api/valve')});
/* Single Stop: aborts foreground motion AND ends a running demo. */
(function(){
  var b=$('bStop'); if(!b)return;
  b.onclick=function(){
    fetch('/api/abort',{method:'POST'});
    var s=window._lastState;
    if(s&&s.demo&&s.demo.active){fetch('/api/demo/stop',{method:'POST'});}
    setTimeout(refresh,200);
  };
})();
bind('bDemoDry',function(){return fetch('/api/demo/start?mode=dry',{method:'POST'}).then(function(r){return r.json()})});
bind('bDemoWet',function(){return fetch('/api/demo/start?mode=wet',{method:'POST'}).then(function(r){return r.json()})});
bind('bDemoRand',function(){return fetch('/api/demo/start?mode=random',{method:'POST'}).then(function(r){return r.json()})});
bind('bDemoStop',function(){return jpost('/api/demo/stop')});
bind('bDemoPause',function(){
  var v=parseInt($('demoPauseInput').value,10);
  if(isNaN(v)||v<1||v>10){alert('Pause must be 1..10 seconds');return Promise.resolve({ok:false});}
  return fetch('/api/demo/config?pause_s='+v,{method:'POST'}).then(function(r){return r.json()});
});
/* Jog stepper: press-and-hold to spin, release to stop. The release POST
   reaches the rotateContinuous() loop via g_abortRequested in the
   foreground-dispatch path; firmware rotate is interruptible. */
(function(){
  function startJog(dir){fetch('/api/stepper/rotate?dir='+dir,{method:'POST'});}
  function stopJog(){fetch('/api/stepper/stop',{method:'POST'});}
  function bindJog(id,dir){
    var b=$(id); if(!b)return;
    var active=false;
    var down=function(e){if(active||b.disabled)return;active=true;e.preventDefault();startJog(dir);};
    var up=function(e){if(!active)return;active=false;if(e&&e.preventDefault)e.preventDefault();stopJog();};
    b.addEventListener('mousedown',down);
    b.addEventListener('mouseup',up);
    b.addEventListener('mouseleave',up);
    b.addEventListener('touchstart',down,{passive:false});
    b.addEventListener('touchend',up);
    b.addEventListener('touchcancel',up);
    // Belt-and-suspenders: if the pointer is released anywhere off the
    // button, still stop the rotor.
    window.addEventListener('blur',up);
  }
  bindJog('bRotFwd','fwd');
  bindJog('bRotRev','rev');
})();

/* ---- friendly device name ---- */
(function(){
  var ni=$('nameI'),nb=$('nameBtn');
  if(!ni||!nb)return;
  // Same rule the firmware enforces: a-z 0-9 hyphen, 1..32 chars,
  // start alphanumeric. Empty is allowed and clears the name.
  function valid(v){return v===''||/^[a-z0-9][a-z0-9-]{0,31}$/.test(v);}
  nb.addEventListener('click',function(){
    var v=ni.value.trim();
    if(!valid(v)){alert('Name must be a-z, 0-9, hyphen; 1-32 chars; start with a letter or digit.');return;}
    nb.disabled=true;
    fetch('/api/setname?name='+encodeURIComponent(v),{method:'POST'})
      .then(function(r){return r.json()})
      .then(function(r){
        nb.disabled=false;
        // Re-arm the one-shot identity block so the next /api/state poll
        // repaints the Device row with the new name.
        if(r&&r.ok){window._idSet=false;if(r.note)alert(r.note);}
        else alert('Set failed: '+((r&&r.reason)||'error'));
      })
      .catch(function(){nb.disabled=false;alert('Set failed: network error');});
  });
})();

/* ---- tuning slider mirroring + touch-grace ---- */
var voS=$('voSlider'),voI=$('voInput');
var vcS=$('vcSlider'),vcI=$('vcInput');
var dwellS=$('dwellSlider'),dwellI=$('dwellInput');
var ledS=$('ledSlider'),ledI=$('ledInput');
var sfS=$('sfSlider'),sfI=$('sfInput');
var voTouchedAt=0,vcTouchedAt=0,dwellTouchedAt=0,ledTouchedAt=0;
var sfTouchedAt=0;
var TOUCH_GRACE_MS=4000;
function stampVo(){voTouchedAt=Date.now();}
function stampVc(){vcTouchedAt=Date.now();}
function stampDwell(){dwellTouchedAt=Date.now();}
function stampLed(){ledTouchedAt=Date.now();}
function stampSf(){sfTouchedAt=Date.now();}
/* Commit on `change` only (slider release / number-input blur / Enter).
   `input` mirrors the paired field but never fires a request -- the
   ESP32's sync WebServer can't keep up with drag-time POST floods. */
function commitValve(){
  var o=parseInt(voI.value,10), c=parseInt(vcI.value,10);
  if(isNaN(o)||isNaN(c)||o<0||o>180||c<0||c>180)return;
  fetch('/api/setvalveangles?open='+o+'&close='+c).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok){voTouchedAt=0;vcTouchedAt=0;}
  });
}
function commitDwell(){
  var v=parseInt(dwellI.value,10);
  if(isNaN(v)||v<1||v>10)return;
  fetch('/api/setdwell?value='+v).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok)dwellTouchedAt=0;
  });
}
function commitLed(){
  var v=parseInt(ledI.value,10);
  if(isNaN(v)||v<0||v>255)return;
  fetch('/api/setledbrightness?value='+v).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok)ledTouchedAt=0;
  });
}
function commitSf(){
  var f=parseInt(sfI.value,10);
  if(isNaN(f)||f<100||f>10000)return;
  fetch('/api/setstepprofile?fast='+f).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok)sfTouchedAt=0;
  });
}
voS.addEventListener('input',function(){voI.value=voS.value;stampVo();});
voS.addEventListener('change',function(){stampVo();commitValve();});
voI.addEventListener('input',function(){voS.value=voI.value;stampVo();});
voI.addEventListener('change',function(){stampVo();commitValve();});
voI.addEventListener('focus',stampVo);
vcS.addEventListener('input',function(){vcI.value=vcS.value;stampVc();});
vcS.addEventListener('change',function(){stampVc();commitValve();});
vcI.addEventListener('input',function(){vcS.value=vcI.value;stampVc();});
vcI.addEventListener('change',function(){stampVc();commitValve();});
vcI.addEventListener('focus',stampVc);
dwellS.addEventListener('input',function(){dwellI.value=dwellS.value;stampDwell();});
dwellS.addEventListener('change',function(){stampDwell();commitDwell();});
dwellI.addEventListener('input',function(){dwellS.value=dwellI.value;stampDwell();});
dwellI.addEventListener('change',function(){stampDwell();commitDwell();});
dwellI.addEventListener('focus',stampDwell);
ledS.addEventListener('input',function(){ledI.value=ledS.value;stampLed();});
ledS.addEventListener('change',function(){stampLed();commitLed();});
ledI.addEventListener('input',function(){ledS.value=ledI.value;stampLed();});
ledI.addEventListener('change',function(){stampLed();commitLed();});
ledI.addEventListener('focus',stampLed);
sfS.addEventListener('input',function(){sfI.value=sfS.value;stampSf();});
sfS.addEventListener('change',function(){stampSf();commitSf();});
sfI.addEventListener('input',function(){sfS.value=sfI.value;stampSf();});
sfI.addEventListener('change',function(){stampSf();commitSf();});
sfI.addEventListener('focus',stampSf);

var ltS=$('ltSlider'),ltI=$('ltInput');
var mtS=$('mtSlider'),mtI=$('mtInput');
var moSel=$('moSelect');
var ltTouchedAt=0,mtTouchedAt=0,moTouchedAt=0;
function stampLt(){ltTouchedAt=Date.now();}
function stampMt(){mtTouchedAt=Date.now();}
function stampMo(){moTouchedAt=Date.now();}
function commitImuThresh(){
  var lv=parseFloat(ltI.value), mv=parseFloat(mtI.value);
  if(isNaN(lv)||lv<1||lv>30)return;
  if(isNaN(mv)||mv<0.02||mv>1.0)return;
  fetch('/api/imu/threshold?spill='+lv+'&motion='+mv,{method:'POST'}).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok){ltTouchedAt=0;mtTouchedAt=0;}
  });
}
function commitMount(){
  fetch('/api/imu/mount?orientation='+encodeURIComponent(moSel.value),{method:'POST'}).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok)moTouchedAt=0;
  });
}
ltS.addEventListener('input',function(){ltI.value=ltS.value;stampLt();});
ltS.addEventListener('change',function(){stampLt();commitImuThresh();});
ltI.addEventListener('input',function(){ltS.value=ltI.value;stampLt();});
ltI.addEventListener('change',function(){stampLt();commitImuThresh();});
ltI.addEventListener('focus',stampLt);
mtS.addEventListener('input',function(){mtI.value=mtS.value;stampMt();});
mtS.addEventListener('change',function(){stampMt();commitImuThresh();});
mtI.addEventListener('input',function(){mtS.value=mtI.value;stampMt();});
mtI.addEventListener('change',function(){stampMt();commitImuThresh();});
mtI.addEventListener('focus',stampMt);
moSel.addEventListener('change',function(){stampMo();commitMount();});
moSel.addEventListener('focus',stampMo);
bind('bMotorInv',function(){return jpost('/api/setmotorinv')});
bind('bCal',function(){
  var g=parseFloat($('calGrams').value);
  /* No confirm()/alert() -- iOS suppresses them, which silently blocked
     the POST (its return value gated the fetch). fetchT() gives the
     request a hard timeout so the button can't freeze on '...'. */
  if(!(g>0)){toast('Enter the known mass in grams (> 0)',true);return Promise.resolve({ok:false});}
  return fetchT('/api/calibrate?grams='+g,{method:'POST'},12000).then(function(r){return r.json()}).then(function(r){
    if(r&&r.ok)toast('Calibrated — gap '+(+r.gapValue).toFixed(3)+', scale OK');
    return r; /* btnAct surfaces any ok:false reason as a toast */
  });
});
bind('bResetWifi',function(){
  if(!confirm('This will FORGET your home WiFi and the device will reboot into AP setup mode. You will lose web access until you reconfigure. Continue?'))return Promise.resolve({ok:false});
  if(!confirm('Are you sure? Last chance.'))return Promise.resolve({ok:false});
  return jpost('/api/resetwifi');
});
bind('bFastSched',function(){return jpost('/api/debug/fastsched')});

/* ---- bucket selection / dose (dial-driven) ---- */
function selectBucket(n){jget('/api/select?bucket='+n).then(refresh)}
function adjDose(n,d){jget('/api/dose?bucket='+n+'&delta='+d).then(refresh)}
$('selInterval').onchange=function(){
  jget('/api/interval/set?bucket='+(g_sel+1)+'&hours='+this.value).then(refresh);
};

/* ---- radial geometry ---- */
function P(r,deg){var t=deg*Math.PI/180;return [CX+r*Math.sin(t), CY-r*Math.cos(t)];}
function f2(n){return Math.round(n*100)/100;}
function xy(p){return f2(p[0])+' '+f2(p[1]);}
// One concentric band as a stroked arc, centred at angle 0 (pointing
// up, -y); the parent <g transform="rotate(angle)"> (inside a
// translate(200,200) layer) places it. midR is the band midline; the
// stroke-width (CSS or inline) sets thickness and stroke-linecap:round
// rounds the ends. Spans +/-halfDeg.
function bandPath(midR,halfDeg){
  var h=halfDeg*Math.PI/180, s=Math.sin(h), c=Math.cos(h);
  var x0=(-midR*s).toFixed(2), y0=(-midR*c).toFixed(2);
  var x1=(midR*s).toFixed(2), y1=(-midR*c).toFixed(2);
  return 'M '+x0+' '+y0+' A '+midR+' '+midR+' 0 0 1 '+x1+' '+y1;
}
// Wavy reservoir surface for the dial tank. Returns the open surface
// trace (surf) and the closed clip region below it down to the canvas
// bottom (clip). fillFrac 0..1 sets the mean water line; phase (radians)
// drifts the ripple in time.
function buildTankPaths(fillFrac,phase,tiltDeg,meanShiftDeg){
  var f=Math.max(0,Math.min(1,fillFrac));
  var yMean=400-f*400;   // yEmpty(400) - f*(yEmpty-yFull=400)
  yMean+=(meanShiftDeg||0)*3;  // fwd/back tilt: +deg -> water falls toward viewer -> mean line moves DOWN (larger y); 3 units/deg, clamped at call site
  var t=(tiltDeg||0)*Math.PI/180, tanT=Math.tan(t), cx=200;  // tilt mean line around dial centre
  var surf='';
  for(var i=0;i<=TANK_SAMPLES;i++){
    var x=(i/TANK_SAMPLES)*400;
    var wp=(i/TANK_SAMPLES)*TANK_CYCLES*2*Math.PI+phase;
    var y=yMean+(x-cx)*tanT+TANK_AMP*Math.sin(wp);   // mean + tilt offset + ripple
    surf+=(i===0?'M ':'L ')+x.toFixed(2)+' '+y.toFixed(2)+' ';
  }
  return {surf:surf, clip:surf+'L 400 400 L 0 400 Z'};
}
function arcPath(r,a0,a1){return 'M'+xy(P(r,a0))+'A'+f2(r)+' '+f2(r)+' 0 0 1 '+xy(P(r,a1));}

var NS='http://www.w3.org/2000/svg', built=false;
function buildDiagram(){
  if(built)return; built=true;
  var wl=$('wedgeLayer');
  for(var k=1;k<=BUCKETS;k++){
    var ang=FIRST+(k-1)*SLOT;
    var g=document.createElementNS(NS,'g');
    g.setAttribute('class','wedge'); g.setAttribute('data-b',k);
    g.setAttribute('transform','rotate('+ang.toFixed(4)+')');
    // Click a wedge that isn't selected -> select it; click the
    // already-selected wedge -> cycle its dose (+10, wrapping through
    // skip via /api/dose's clamp/collapse rules).
    (function(bk){g.addEventListener('click',function(){
      if(bk!==g_sel+1) selectBucket(bk);
      else adjDose(bk,10);
    });})(k);
    for(var i=0;i<6;i++){
      var p=document.createElementNS(NS,'path');
      p.setAttribute('class','seg');
      p.setAttribute('d',bandPath(BAND_RADII[i],HALF_DEG));
      g.appendChild(p);
    }
    // Skip wedge: one stroked arc spanning the whole band stack
    // (midline r=125, width=80 = BAND_RADII span + BAND_THICK).
    var sk=document.createElementNS(NS,'path');
    sk.setAttribute('class','skipwedge');
    sk.setAttribute('d',bandPath(125,HALF_DEG));
    sk.setAttribute('stroke-width','80');
    sk.style.display='none';
    g.appendChild(sk);
    wl.appendChild(g);
  }
  var dz=$('deadLayer');
  var dg=document.createElementNS(NS,'g');
  dg.setAttribute('transform','rotate(180)');
  // Home tick: small home reference mark above the band stack. (The
  // dashed dead-zone band is gone -- the reservoir droplet now occupies
  // the 6 o'clock gap.)
  var tick=document.createElementNS(NS,'rect');
  tick.setAttribute('x',-7);tick.setAttribute('y',-172);
  tick.setAttribute('width',14);tick.setAttribute('height',6);
  tick.setAttribute('rx',3);tick.setAttribute('class','hometick');
  dg.appendChild(tick);
  dz.appendChild(dg);
}

function T(id,t){var e=$(id);if(e)e.textContent=(t==null?'':t);}
function updateCentre(s){
  var top=$('topArcText'), bot=$('botArcText');
  if(!top||!bot)return;
  var st=s.state, fa=s.fault||{}, df=s.defer||{};
  var topStr='', botStr='';
  if(st==='ERROR'){
    var fl=FAULT_FRIENDLY[fa.code]||'FAULT';
    topStr='FAULT · '+fl; botStr='HOLD BUTTON 3S';
  } else if(st==='WATERING'){
    topStr='WATERING · PLANT '+(s.activeBucket||'?');
    var del=(typeof s.activeDeliveredMl==='number')?s.activeDeliveredMl:0;
    botStr=del.toFixed(1)+' / '+(s.activeTargetMl||0)+' ML';
  } else if(st==='BOOTING'){
    topStr='BOOTING'; botStr='';
  } else if(df.reason && df.reason!=='none'){
    var rz={not_level:'NOT LEVEL',moving:'MOVING',reservoir_low:'RESERVOIR LOW'}[df.reason]||df.reason.toUpperCase();
    topStr='WAITING · '+rz; botStr='';
  } else if(s.schedulerEnabled===false){
    topStr='PAUSED'; botStr=Math.round(s.reservoirMl)+' / '+Math.round(s.reservoirFullMl)+' ML';
  } else {
    var nxt=(s.nextCycleSec<0)?'—':fmtNext(s.nextCycleSec);
    topStr='NEXT IN '+nxt.toUpperCase(); botStr=Math.round(s.reservoirMl)+' / '+Math.round(s.reservoirFullMl)+' ML';
  }
  top.textContent=topStr; bot.textContent=botStr;
}

function updateArm(s){
  var led=s.led||{};
  var col=led.color1||'#4a90c2';
  applyArmAngle(s);
  $('armPlatter').setAttribute('fill',col);
  $('armStem').setAttribute('fill',col);
  var arm=$('arm'), stem=$('armStem');
  arm.classList.remove('breathe','pulse','valveopen');
  var valveOpen=!!s.valveIsOpen;
  var watering=(s.state==='WATERING')||(s.demo&&s.demo.active&&valveOpen);
  if(s.state==='ERROR'){
    arm.classList.add('pulse'); arm.style.opacity=''; stem.classList.add('dashed');
  } else {
    if(s.armPositionTrusted===false) stem.classList.add('dashed');
    else stem.classList.remove('dashed');
    if(watering){
      var env=(led.envelope!==undefined?led.envelope:255)/255;
      arm.style.opacity=Math.max(0.4,env).toFixed(3);
    } else if(s.schedulerEnabled===false && s.state==='READY' && !(s.demo&&s.demo.active)){
      arm.style.opacity='0.55';
    } else if(s.state==='BOOTING'){
      arm.style.opacity='0.5';
    } else {
      arm.classList.add('breathe'); arm.style.opacity='';
    }
  }
  if(s.armPositionTrusted===false) arm.style.opacity='0.5';
  if(valveOpen) arm.classList.add('valveopen');
  var dropAnim=$('armDropAnim');
  if(dropAnim){
    var isDispensing=(s.state==='WATERING')&&(s.valveIsOpen===true);
    dropAnim.style.display=isDispensing?'inline':'none';
  }
}

function updateDiagram(s){
  buildDiagram();
  var wl=$('wedgeLayer');
  for(var k=1;k<=BUCKETS;k++){
    var b=s.buckets[k-1]; if(!b)continue;
    var g=wl.children[k-1]; if(!g)continue;
    var segs=g.getElementsByClassName('seg');
    var sk=g.getElementsByClassName('skipwedge')[0];
    var dose=b.doseMl;
    // Skip-dose reads as 6 greyed bars (no lit class on any), matching
    // the LCD renderer. The dashed skipwedge stays hidden in both states.
    if(sk)sk.style.display='none';
    if(dose===0){
      for(var i=0;i<segs.length;i++){
        segs[i].style.display='';
        segs[i].setAttribute('class','seg');
      }
    } else {
      var lit=Math.floor((dose-10)/10)+1;
      for(var i=0;i<segs.length;i++){
        segs[i].style.display='';
        segs[i].setAttribute('class','seg'+(i<lit?' lit':''));
      }
    }
  }
  // Halo priority mirrors the LCD renderer: during a cycle the active
  // bucket wins, otherwise a parked-by-arm bucket overrides g_selected.
  var selB;
  if(s.state==='WATERING'&&s.activeBucket>0)     selB=s.activeBucket;
  else if(s.interactionParkedBucket>0)           selB=s.interactionParkedBucket;
  else                                           selB=(s.selected||0)+1;
  var sel=selB-1;
  var c=FIRST+sel*SLOT, a0=c-(SLOT-GAP)/2, a1=c+(SLOT-GAP)/2;
  var halo=$('selHalo');
  halo.setAttribute('d',arcPath(171,a0,a1));
  var fa=s.fault||{};
  halo.setAttribute('stroke',(s.state==='ERROR'&&fa.bucket===selB)?'#a04040':'#df7e09');
  // Reservoir tank: bone above, wavy water below. fillFrac sets the
  // water line; phase drifts the ripple (~0.5 rad/sec) across renders.
  var frac=s.reservoirFullMl>0?Math.max(0,Math.min(1,s.reservoirMl/s.reservoirFullMl)):0;
  var tankPhase=(Date.now()*0.0005)%(2*Math.PI);
  // Axis mapping for MOUNT_Y_UP: board "pitch" = user-perceived left/right
  // tilt (tilts the surface), board "roll" = forward/back (shifts the mean
  // line). Swap so the JS uses user-facing names. Clamp both to +/-30 deg
  // so an extreme/spurious reading can't push the surface off-canvas.
  var lrDeg=(s.imu&&typeof s.imu.pitchDeg==='number')?-s.imu.pitchDeg:0;
  var fbDeg=(s.imu&&typeof s.imu.rollDeg==='number')?-s.imu.rollDeg:0;
  if(lrDeg>30)lrDeg=30; if(lrDeg<-30)lrDeg=-30;
  if(fbDeg>30)fbDeg=30; if(fbDeg<-30)fbDeg=-30;
  var tp=buildTankPaths(frac,tankPhase,lrDeg,fbDeg);
  var wcp=$('waterClipPath'); if(wcp)wcp.setAttribute('d',tp.clip);
  var tsf=$('tankSurface');   if(tsf)tsf.setAttribute('d',tp.surf);
  // Low-reservoir indicator: the droplet turns alarm-red when the tank
  // can't cover the sum of all non-skip doses. The water stays blue.
  var totalDose=0; for(var i=0;i<s.buckets.length;i++){var d=s.buckets[i].doseMl;if(d>0)totalDose+=d;}
  var drop=$('droplet');
  if(drop){
    var low=(s.reservoirMl<totalDose);
    drop.setAttribute('fill',low?'#5a534a':'#4a90c2');
    drop.setAttribute('stroke',low?'#48433b':'#3d7fb0');
  }
}

function updateStrip(s){
  var sel=s.selected||0, b=s.buckets[sel]||{};
  g_sel=sel; g_curDose=b.doseMl||0;
  T('selPlant',sel+1);
  var iv=$('selInterval');
  if(iv && document.activeElement!==iv && b.intervalH) iv.value=b.intervalH;
  if(iv) iv.disabled=(s.state!=='READY');
}

function applyState(s){
  if(s.deviceName && !window._idSet){
    document.title='Pluvio — '+s.deviceName;
    var fv=s.fwVersion||'?';
    var dn=$('devName'); if(dn)dn.textContent=s.deviceName;
    var fd=$('footDev'); if(fd)fd.textContent=s.deviceName+' · v'+fv;
    // Device row: friendly name with the immutable name in parens when
    // set, else just the immutable name. Header/title above keep using
    // deviceName until a reload picks up a rename -- no live retitling.
    var fn=s.friendlyName||'';
    var sd=$('sysDev'); if(sd)sd.textContent=fn?(fn+' ('+s.deviceName+')'):s.deviceName;
    var ni=$('nameI'); if(ni)ni.value=fn;
    var sf=$('sysFw'); if(sf)sf.textContent=fv;
    window._idSet=true;
  }
  window._fastMode=!!s.fastSchedule;
  $('fastBanner').classList.toggle('on', !!s.fastSchedule);
  $('schedBanner').classList.toggle('on', !s.schedulerEnabled);
  $('bSched').innerHTML=s.schedulerEnabled?'<i class="ti ti-player-pause" aria-hidden="true"></i> Pause Schedule':'<i class="ti ti-player-play" aria-hidden="true"></i> Resume Schedule';
  var fs=$('fastStat');
  if(fs){fs.textContent=s.fastSchedule?'ON (seconds)':'OFF (normal)';fs.style.color=s.fastSchedule?'#af6306':'#6b6864';}
  var st=$('st');st.className='badge '+s.state;st.textContent=s.state;

  var mb=$('motionBanner');
  if(mb){
    var motionRunning=(s.state==='WATERING'||s.abortableMotion);
    mb.classList.toggle('on', motionRunning);
    var lbl=$('motionLabel');
    if(lbl){
      if(s.demo&&s.demo.active){lbl.textContent='Demo running ('+(s.demo.mode||'').toUpperCase()+') — cycle '+s.demo.cycleCount;}
      else if(s.state==='WATERING'){lbl.textContent='Watering cycle running';}
      else if(s.abortableMotion){lbl.textContent='Motor operation running';}
    }
  }
  $('rssi').textContent=s.wifiRssi||'--';
  $('up').textContent=fmtT(s.uptimeMs);
  var armAng=$('armAng');
  if(armAng&&typeof s.armAngleDeg==='number')armAng.textContent=s.armAngleDeg.toFixed(1);
  var armCal=$('armCal');
  if(armCal){
    if(s.endstopsCalibrated===false){armCal.textContent='(uncalibrated)';armCal.style.color='#af6306';}
    else if(s.armPositionTrusted===false){armCal.textContent='(untrusted)';armCal.style.color='#af6306';}
    else{armCal.textContent='';}
  }
  var encStat=$('encStat');
  if(encStat){
    if(s.encoderHealthy===true){encStat.textContent='OK (status 0x'+(s.encoderStatus||0).toString(16).padStart(2,'0')+')';encStat.style.color='#2f8f4f';}
    else{encStat.textContent='NOT HEALTHY (status 0x'+(s.encoderStatus||0).toString(16).padStart(2,'0')+')';encStat.style.color='#a04040';}
  }
  var nowMs=Date.now();
  if((nowMs-voTouchedAt)>TOUCH_GRACE_MS&&s.valveOpenAng!==undefined){voS.value=s.valveOpenAng;voI.value=s.valveOpenAng;}
  if((nowMs-vcTouchedAt)>TOUCH_GRACE_MS&&s.valveCloseAng!==undefined){vcS.value=s.valveCloseAng;vcI.value=s.valveCloseAng;}
  if((nowMs-dwellTouchedAt)>TOUCH_GRACE_MS&&s.dwellSec!==undefined){dwellS.value=s.dwellSec;dwellI.value=s.dwellSec;}
  if((nowMs-ledTouchedAt)>TOUCH_GRACE_MS&&s.ledBrightness!==undefined){ledS.value=s.ledBrightness;ledI.value=s.ledBrightness;}
  if((nowMs-sfTouchedAt)>TOUCH_GRACE_MS&&s.stepDelayFastUs!==undefined){sfS.value=s.stepDelayFastUs;sfI.value=s.stepDelayFastUs;}
  var miStat=$('motorInvStat');
  if(miStat&&s.motorInverted!==undefined){miStat.textContent=s.motorInverted?'INVERTED':'normal';miStat.style.color=s.motorInverted?'#af6306':'#2f8f4f';}

  var ipanel=$('installPanel'),iTuneHr=$('imuTuneHr'),iTuneRows=$('imuTuneRows'),lvlBan=$('levelBanner');
  if(s.imu&&s.imu.present){
    if(ipanel)ipanel.style.display='';
    if(iTuneHr)iTuneHr.style.display='';
    if(iTuneRows)iTuneRows.style.display='';
    $('imuType').textContent=s.imu.type||'?';
    $('imuMountLbl').textContent=s.imu.mountOrientation||'?';
    $('imuPitch').textContent=(+s.imu.pitchDeg).toFixed(1);
    $('imuRoll').textContent=(+s.imu.rollDeg).toFixed(1);
    $('imuMotion').textContent=(+s.imu.motionMetric).toFixed(3);
    if(s.imu.axRaw!==undefined){
      $('imuAxRaw').textContent=(+s.imu.axRaw).toFixed(2);
      $('imuAyRaw').textContent=(+s.imu.ayRaw).toFixed(2);
      $('imuAzRaw').textContent=(+s.imu.azRaw).toFixed(2);
    }
    var th=Math.max(1,+s.imu.spillThreshDeg||10);
    var bx=Math.max(-1,Math.min(1,(+s.imu.pitchDeg)/th))*30;
    var by=Math.max(-1,Math.min(1,(+s.imu.rollDeg)/th))*30;
    var bub=$('bubble');
    if(bub){bub.setAttribute('cx',50+bx);bub.setAttribute('cy',50+by);bub.setAttribute('fill',s.imu.level?'#4a90c2':'#a04040');}
    if(lvlBan){
      var dr=(s.defer&&s.defer.reason)||'none';
      if(dr!=='none'){
        lvlBan.classList.add('on');
        if(dr==='not_level'){lvlBan.textContent='Cycle deferred: not level (pitch '+(+s.imu.pitchDeg).toFixed(1)+'°, roll '+(+s.imu.rollDeg).toFixed(1)+'°, threshold '+th+'°)';}
        else if(dr==='moving'){lvlBan.textContent='Cycle deferred: rig moving ('+(+s.imu.motionMetric).toFixed(2)+'g, threshold '+(+s.imu.motionThreshG).toFixed(2)+'g)';}
        else if(dr==='reservoir_low'){var need=0;if(s.buckets){for(var bi=0;bi<s.buckets.length;bi++)need+=(+s.buckets[bi].doseMl||0);}lvlBan.textContent='Cycle deferred: reservoir low ('+(+s.reservoirMl).toFixed(0)+'mL of '+need.toFixed(0)+'mL needed)';}
        else{lvlBan.textContent='Cycle deferred: '+dr;}
      } else {lvlBan.classList.remove('on');}
    }
    var nowMsImu=Date.now();
    if((nowMsImu-ltTouchedAt)>TOUCH_GRACE_MS&&s.imu.spillThreshDeg!==undefined){ltS.value=s.imu.spillThreshDeg;ltI.value=s.imu.spillThreshDeg;}
    if((nowMsImu-mtTouchedAt)>TOUCH_GRACE_MS&&s.imu.motionThreshG!==undefined){mtS.value=s.imu.motionThreshG;mtI.value=s.imu.motionThreshG;}
    if((nowMsImu-moTouchedAt)>TOUCH_GRACE_MS&&s.imu.mountOrientation!==undefined){moSel.value=s.imu.mountOrientation;}
  } else {
    if(ipanel)ipanel.style.display='none';
    if(iTuneHr)iTuneHr.style.display='none';
    if(iTuneRows)iTuneRows.style.display='none';
    if(lvlBan)lvlBan.classList.remove('on');
  }
  var cpg=$('cpg');if(cpg&&s.gapValue!==undefined)cpg.textContent=(+s.gapValue).toFixed(3);
  var cpg2=$('cpg2');if(cpg2&&s.gapValue!==undefined)cpg2.textContent=(+s.gapValue).toFixed(3);
  var calOk=(s.scaleCalibrated===true);
  var ss2=$('scaleStat');
  if(ss2){
    if(!s.scaleOK){ss2.textContent='NOT DETECTED';ss2.style.color='#a04040';}
    else if(!calOk){ss2.textContent='NEEDS CALIBRATION';ss2.style.color='#af6306';}
    else{ss2.textContent='OK';ss2.style.color='#2f8f4f';}
  }
  var lw=$('liveWeight');
  if(lw){
    if(!s.scaleOK){lw.textContent='--';lw.style.color='var(--soft)';}
    else if(!calOk){lw.textContent='needs calibration';lw.style.color='#af6306';}
    else if(s.weightG===null||s.weightG===undefined){lw.textContent='--';lw.style.color='var(--soft)';}
    else{lw.textContent=(+s.weightG).toFixed(1)+' g';lw.style.color='var(--ink)';}
  }
  /* Live raw counts + delta-from-zero: real-time proof the cell responds,
     independent of calibration. Watch counts move as mass is added. */
  var rc=$('rawCounts');
  if(rc){
    if(s.scaleRaw===null||s.scaleRaw===undefined){rc.textContent='no read';rc.style.color='#a04040';}
    else{
      var d=(s.rawTare!==undefined)?(s.scaleRaw-s.rawTare):null;
      rc.textContent=s.scaleRaw+(s.zeroArmed&&d!==null?('  (Δ '+(d>=0?'+':'')+d+' from zero)'):'');
      rc.style.color='var(--ink)';
    }
  }
  // Show the calibrate wizard only when uncalibrated; lock it (and show
  // the reset row) once calibrated.
  var wiz=$('calWizard'),rst=$('calResetRow');
  if(wiz)wiz.style.display=calOk?'none':'';
  if(rst)rst.style.display=calOk?'':'none';
  var watering=(s.state==='WATERING');
  /* Cal buttons are NOT gated on scaleOK: calibration runs on raw ADC
     counts (valid regardless of the converted-weight health probe) and
     is precisely what re-establishes an unhealthy scale. Only WATERING
     disables them (the cycle owns the cell). A genuine raw-read failure
     is surfaced by the endpoint as a visible "scale not responding". */
  if($('bZero'))$('bZero').disabled=watering;
  if($('bCal'))$('bCal').disabled=watering;
  if($('bScaleReset'))$('bScaleReset').disabled=watering;
  /* Wizard progress: make Step 1's (otherwise invisible) zero capture
     visible, show Step 2 is unblocked even after the ownership watchdog
     lapses (captured zero is retained), and never leave a disabled
     button unexplained. */
  var cst=$('calStat');
  if(cst){
    if(watering){cst.textContent='busy (watering) — cal paused';cst.style.color='var(--soft)';}
    else if(s.zeroArmed){cst.textContent='zero captured — add mass & Calibrate'+(s.scaleOK?'':' (will re-detect scale)');cst.style.color='#2f8f4f';}
    else{cst.textContent='capture zero first (Step 1)'+(s.scaleOK?'':' — scale health probe offline; cal will re-establish it');cst.style.color=s.scaleOK?'var(--soft)':'#af6306';}
  }

  var f=$('fault'),fc=s.fault&&s.fault.code;
  if(s.state==='ERROR'&&fc&&fc!=='NONE'){
    f.classList.add('on');
    f.textContent='Fault: '+(FAULT_FRIENDLY[fc]||fc)+(s.fault.bucket>0?' (plant '+s.fault.bucket+')':'')+'  ·  '+Math.round(s.fault.msAgo/1000)+'s ago';
    $('bClear').style.display='';
  } else {f.classList.remove('on');$('bClear').style.display='none';}

  updateDiagram(s);
  updateCentre(s);
  updateArm(s);
  updateStrip(s);

  var notReady=(s.state!=='READY');
  var motionFault=(s.state==='ERROR'&&s.fault&&s.fault.code==='ENCODER_LOST');
  var motorOpsBlocked=(s.state==='WATERING'||motionFault||s.abortableMotion);
  var demoActive=!!(s.demo&&s.demo.active);
  $('bWater').disabled=notReady;
  /* Water Selected: enabled only when READY with a non-skip bucket
     selected. Label names the plant so the user sees exactly what will
     be watered (it may differ from the parked bucket). Don't rewrite the
     label mid-action -- btnAct owns the button's contents while inflight. */
  var bws=$('bWaterSel');
  if(bws){
    var selIdx=s.selected||0;
    var selBk=(s.buckets&&s.buckets[selIdx])||null;
    var selSkip=!!selBk&&selBk.doseMl===0;
    bws.disabled=notReady||!selBk||selSkip;
    if(!inflight[bws.id])
      bws.innerHTML='<i class="ti ti-droplet-filled" aria-hidden="true"></i> Water Plant '+(selIdx+1)+(selSkip?' (skip)':'');
  }
  if($('bStop'))$('bStop').disabled=!(s.state==='WATERING'||s.abortableMotion||demoActive);
  $('bHome').disabled=motorOpsBlocked;
  $('bRedisc').disabled=motorOpsBlocked;
  $('bDtestDry').disabled=motorOpsBlocked;
  $('bDtestWet').disabled=motorOpsBlocked;
  if($('bRotFwd'))$('bRotFwd').disabled=motorOpsBlocked;
  if($('bRotRev'))$('bRotRev').disabled=motorOpsBlocked;
  if($('bDemoDry'))$('bDemoDry').disabled=motorOpsBlocked;
  if($('bDemoWet'))$('bDemoWet').disabled=motorOpsBlocked;
  if($('bDemoRand'))$('bDemoRand').disabled=motorOpsBlocked;
  if($('bDemoStop'))$('bDemoStop').disabled=!demoActive;
  var dStat=$('demoStat');
  if(dStat){dStat.textContent=demoActive?(' — running '+(s.demo.mode||'').toUpperCase()+', cycle '+s.demo.cycleCount):'';dStat.style.color=demoActive?'#af6306':'#6b6864';}
  var dpi=$('demoPauseInput');
  if(dpi&&document.activeElement!==dpi&&s.demo&&s.demo.pauseSec!==undefined)dpi.value=s.demo.pauseSec;
  var bv=$('bValve');
  if(bv){
    bv.disabled=(s.state==='WATERING');
    if(s.valveIsOpen!==undefined){
      bv.textContent='Valve: '+(s.valveIsOpen?'Open (tap to close)':'Closed (tap to open)');
      bv.classList.toggle('dn',!!s.valveIsOpen);
    }
  }
  $('bMotorInv').disabled=(s.state==='WATERING');
  /* bZero / bCal / bScaleReset disabled state handled above. */
  /* Cache for bStop's demo-aware handler (avoids an extra fetch). */
  window._lastState=s;
}

function refresh(){jget('/api/state').then(function(s){if(s)applyState(s);}).catch(function(){})}

function poll(){
  jget('/api/state').then(function(s){
    if(s)applyState(s);
    var fast=!!(s&&(s.state==='WATERING'||s.abortableMotion===true||(s.demo&&s.demo.active===true)));
    setTimeout(poll, fast?250:1500);
  }).catch(function(){setTimeout(poll,1500);});
}

/* ---- live log ---- */
var lastSeen=0,paused=false,logRetry=500,userScrolled=false;
$('bPause').onclick=function(){paused=!paused;$('bPause').textContent=paused?'Resume':'Pause'};
$('bClearLog').onclick=function(){$('log').textContent='';userScrolled=false};
var lg=$('log');
lg.addEventListener('scroll',function(){userScrolled=(lg.scrollTop+lg.clientHeight<lg.scrollHeight-10);});
function pad2(n){return n<10?'0'+n:''+n}
function logPrefix(ln){
  if(ln.epoch&&ln.epoch>0){var d=new Date(ln.epoch*1000);return '['+pad2(d.getHours())+':'+pad2(d.getMinutes())+':'+pad2(d.getSeconds())+'] ';}
  var s=String(ln.ms||0);while(s.length<8)s=' '+s;return '['+s+'ms] ';
}
function pollLog(){
  var ld=document.getElementById('logSection');
  if(ld&&!ld.open){setTimeout(pollLog,1000);return;}
  if(paused){$('logStat').textContent='paused';setTimeout(pollLog,500);return}
  jget('/api/log?since='+lastSeen).then(function(r){
    logRetry=500;$('logStat').textContent='';
    if(r&&r.lines&&r.lines.length){
      var append='';
      for(var i=0;i<r.lines.length;i++){var ln=r.lines[i];append+=logPrefix(ln)+ln.text;}
      lg.textContent+=append;
      if(lg.textContent.length>65536)lg.textContent=lg.textContent.slice(-49152);
      lastSeen=r.nextSince-1;
      if(!userScrolled)lg.scrollTop=lg.scrollHeight;
    }else if(r){lastSeen=r.nextSince-1}
    setTimeout(pollLog,500);
  }).catch(function(){$('logStat').textContent='retry';logRetry=Math.min(logRetry*2,5000);setTimeout(pollLog,logRetry);});
}

/* ---- settings drawer open/close ---- */
(function(){
  var dr=$('drawer'), bd=$('drawerBackdrop');
  function openDrawer(){if(dr)dr.classList.add('open');if(bd)bd.classList.add('open');}
  function closeDrawer(){if(dr)dr.classList.remove('open');if(bd)bd.classList.remove('open');}
  var g=$('gearBtn'); if(g)g.onclick=openDrawer;
  var x=$('drawerClose'); if(x)x.onclick=closeDrawer;
  if(bd)bd.onclick=closeDrawer;
  document.addEventListener('keydown',function(e){if(e.key==='Escape')closeDrawer();});
  var ip=$('sysIp'); if(ip)ip.textContent=location.hostname||'—';
})();

buildDiagram();
poll();
pollLog();
</script></body></html>)HTML";

// OTA upload page. Tiny HTML+JS form posts the firmware as
// multipart/form-data to POST /update; XHR upload progress drives
// the bar. Matches the main UI's dark theme.
static const char otaUploadHtml[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pluvio — Firmware Update</title>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;margin:0;padding:16px;background:#1a1a1a;color:#e0e0e0;font-size:14px;max-width:520px}
h1{font-size:1.2em;margin:0 0 12px 0}
.warn{background:#a85;color:#000;padding:8px 10px;border-radius:4px;margin:8px 0;font-size:0.9em}
input[type=file]{display:block;margin:12px 0;color:#e0e0e0}
button{background:#345;color:#eee;border:0;border-radius:4px;padding:10px 16px;font-size:0.95em;cursor:pointer}
button:hover{background:#456}
button:disabled{opacity:0.4;cursor:default}
.bar{background:#333;height:20px;border-radius:3px;margin:12px 0;overflow:hidden;position:relative}
.fill{background:#1E90FF;height:100%;width:0;transition:width 0.2s}
.lbl{position:absolute;width:100%;text-align:center;line-height:20px;font-size:0.8em;color:#fff;text-shadow:0 0 2px #000}
#stat{margin-top:8px;font-family:Consolas,Menlo,monospace;font-size:0.9em}
.run{color:#9ab;font-size:0.9em;margin:0 0 10px 0}
.ok{color:#3a8a3a}.err{color:#a04040}
a{color:#8af;text-decoration:none}
</style></head><body>
<h1>Pluvio — Firmware Update</h1>
<div id="run" class="run">Running: --</div>
<div class="warn">Do not start an update during a watering cycle. Motor and valve are forced off when the upload begins.</div>
<form id="f">
<input id="file" type="file" name="firmware" accept=".bin" required>
<button id="go" type="submit">Upload</button>
</form>
<div class="bar"><div id="fill" class="fill"></div><div id="lbl" class="lbl"></div></div>
<div id="stat"></div>
<p><a href="/">&larr; back</a></p>
<script>
document.getElementById('f').onsubmit=function(e){
  e.preventDefault();
  var fi=document.getElementById('file');
  if(!fi.files.length)return;
  var fd=new FormData();fd.append('firmware',fi.files[0]);
  var x=new XMLHttpRequest();
  x.upload.onprogress=function(ev){
    if(ev.lengthComputable){
      var p=(ev.loaded/ev.total*100).toFixed(1);
      document.getElementById('fill').style.width=p+'%';
      document.getElementById('lbl').textContent=p+'% ('+(ev.loaded/1024|0)+' / '+(ev.total/1024|0)+' KB)';
    }
  };
  x.onload=function(){
    var s=document.getElementById('stat');
    if(x.status===200){s.className='ok';s.textContent=x.responseText+' (device rebooting...)'}
    else{s.className='err';s.textContent='HTTP '+x.status+': '+x.responseText}
  };
  x.onerror=function(){
    // Device may reboot before responding which can look like a
    // network error. If the upload fully completed, treat as success.
    var s=document.getElementById('stat');
    s.className='ok';
    s.textContent='Upload finished, device rebooting (connection dropped, normal)';
  };
  x.open('POST','/update');x.send(fd);
  document.getElementById('go').disabled=true;
};
// One-shot: show the running firmware so the operator can confirm what
// they're replacing. No polling -- this page is otherwise static.
fetch('/api/state').then(function(r){return r.json()}).then(function(s){
  document.getElementById('run').textContent=
    'Running: v'+(s.fwVersion||'?')+' ('+(s.fwBuild||'?')+')';
}).catch(function(){
  document.getElementById('run').textContent='Running: --';
});
</script></body></html>)HTML";

// Plain-text duplicate of printMenu(). Kept manually in sync; the
// canonical definition is printMenu(), but exposing this via
// /api/menu is a debugging aid for users who only have web access.
static const char webMenuTextP[] PROGMEM =
    "---- menu ----\n"
    "?       print menu\n"
    "s       (re)detect U177 mini-scale\n"
    "t       capture scale zero (calibration step 1; empty cell)\n"
    "c G     calibrate U177 gap value with known mass G grams on cell\n"
    "w       trigger watering cycle now\n"
    "x       abort current cycle / motion (any button press also aborts)\n"
    "h       home now (servo to bucket 1)\n"
    "C       calibrate endstops (drive into both stops to learn encoder zero + direction)\n"
    "d N V   set bucket N (1-indexed) dose to V mL (V=0 to skip, or 10..60)\n"
    "i N H   set bucket N interval to H hours (12|24|48|72|120|168|240|336|504)\n"
    "D V     dispense test (V=0 dry, V=1 wet); same as watering but no load cell\n"
    "V O A   set valve OPEN angle to A (0..180), persists\n"
    "V C A   set valve CLOSED angle to A (0..180), persists\n"
    "Y V     set post-valve dwell to V seconds (1..10), persists\n"
    "F ML    set reservoir-full volume (UI gauge full-scale, 100..20000 mL), persists; bare F prints\n"
    "B V     set status-LED brightness to V (0..255, 0=off), persists\n"
    "P F|S|R V   stepper profile: P F <us> fast, P S <us> slow, P R <n> ramp\n"
    "R       rotate continuously forward (motor test); 'x' or button to stop\n"
    "r       rotate continuously reverse (motor test); 'x' or button to stop\n"
    "J D [N] OPEN-LOOP blind jog: step motor D=+/- by N steps (default 32, max 114); "
    "works with NO encoder; voids position trust -> recal required before normal motion\n"
    "A       acknowledge / clear fault (ERROR -> READY)\n"
    "p       print state\n"
    "X       toggle DEMO mode (dry); x/button/web to stop (wet demo via web)\n"
    "N       toggle DEMO Random (dry): 1..3 random buckets/cycle; cadence test\n"
    "W       reset WiFi credentials and reboot to AP mode\n"
    "E       toggle watering scheduler (PAUSE / RESUME)\n"
    "Z       toggle DEBUG fast-schedule mode (intervals as seconds, for testing)\n"
    "L V     set spill threshold (max tilt for watering) to V degrees (1..30), persists\n"
    "M V     set IMU motion threshold to V g (0.02..1.0), persists\n"
    "O N     set mount orientation: 0=Z_UP 1=Z_DOWN 2=Y_UP 3=Y_DOWN 4=X_UP 5=X_DOWN, persists\n";

static const char* webStateName() {
    switch (g_state) {
        case BOOTING:  return "BOOTING";
        case READY:    return "READY";
        case WATERING: return "WATERING";
        case ERROR:    return "ERROR";
    }
    return "?";
}

// OTA rollback helpers defined alongside setup()/loop() further down;
// forward-declared here so /api/state can report the running slot + state.
static const char* otaRunningLabel();
static const char* otaRunningStateName();

// JSON for /api/state. Manual concatenation rather than ArduinoJson
// to avoid pulling in another library; payload stays small (<2 KB
// even at 14 buckets). Field names match what the web UI script reads.
static String apiStateJson() {
    String j;
    j.reserve(512);
    j += "{\"state\":\"";  j += webStateName();        j += "\",";
    j += "\"deviceName\":\""; j += g_deviceName;       j += "\",";
    // friendlyName is charset-restricted (a-z0-9-), so no JSON escaping
    // needed; empty string when unset.
    j += "\"friendlyName\":\""; j += g_friendlyName;   j += "\",";
    j += "\"fwVersion\":\""; j += FW_VERSION;          j += "\",";
    j += "\"fwBuild\":\"";   j += FW_BUILD;            j += "\",";
    // OTA rollback status: lets the fleet manager confirm post-update that
    // the device booted the new slot ("otaPartition" flips app0<->app1)
    // and that it marked itself valid ("otaState":"valid"). A lingering
    // "pending-verify" past ~60 s, or an unchanged partition, flags a
    // failed/reverted push.
    j += "\"otaPartition\":\""; j += otaRunningLabel();     j += "\",";
    j += "\"otaState\":\"";     j += otaRunningStateName(); j += "\",";
    j += "\"bucketCount\":"; j += g_bucketCount;       j += ",";
    j += "\"selected\":";   j += g_selected;           j += ",";
    j += "\"reservoirMl\":"; j += String(g_reservoirMl, 1);     j += ",";
    j += "\"reservoirFullMl\":"; j += String(g_reservoirFullMl, 1); j += ",";
    j += "\"stepCounter\":";    j += (long)g_stepCounter;            j += ",";
    j += "\"largeFinalErrCount\":"; j += (unsigned long)g_largeFinalErrCount; j += ",";
    j += "\"lastLargeFinalErrDeg\":"; j += String(g_lastLargeFinalErrDeg, 2); j += ",";
    j += "\"suspectActive\":"; j += (g_suspectActive ? "true" : "false"); j += ",";
    j += "\"suspectSkipCount\":"; j += (unsigned long)g_suspectSkipCount; j += ",";
    j += "\"lastSuspectBucket\":"; j += g_lastSuspectBucket; j += ",";
    j += "\"jogSessionSteps\":"; j += (unsigned long)g_jogSessionSteps; j += ",";
    j += "\"phaseIndex\":";     j += (unsigned)g_phaseIndex;         j += ",";
    j += "\"coilsEnergized\":"; j += (g_coilsEnergized ? "true" : "false"); j += ",";
    // Radial web-UI geometry + live arm position. armAngleDeg is the
    // live AS5600 reading (deg clockwise from the encoder's internal
    // zero); the UI uses it as the source of truth for arm position
    // alongside encoderHealthy / endstopsCalibrated. The dead-zone /
    // slot fields describe the 14-slot layout (BUCKET_COUNT buckets +
    // 1 dead zone at 6 o'clock) so the UI doesn't hard-code geometry.
    j += "\"armAngleDeg\":"; j += String(g_armPositionDeg, 2); j += ",";
    j += "\"encoderHealthy\":"; j += (g_encoderHealthy ? "true" : "false"); j += ",";
    j += "\"encoderStatus\":";  j += (unsigned)g_encoderStatus;         j += ",";
    j += "\"endstopsCalibrated\":"; j += (g_endstopsCalibrated ? "true" : "false"); j += ",";
    j += "\"encoderZeroOffsetDeg\":"; j += String(g_encoderZeroOffset, 2); j += ",";
    j += "\"bucketAngleStepPositive\":"; j += (g_bucketAngleStepPositive ? "true" : "false"); j += ",";
    j += "\"armPositionTrusted\":"; j += (g_endstopsCalibrated && g_encoderHealthy ? "true" : "false"); j += ",";
    j += "\"deadZoneCentreDeg\":180,";
    j += "\"deadZoneWidthDeg\":";     j += String(BUCKET_SLOT_DEG, 2);            j += ",";
    j += "\"firstBucketCentreDeg\":"; j += String(180.0f + BUCKET_SLOT_DEG, 2);   j += ",";
    j += "\"bucketSlotDeg\":";        j += String(BUCKET_SLOT_DEG, 2);            j += ",";
    // LED mirror: lets the on-screen arm match the physical arm-mounted
    // LED's colour and brightness (cached by updateLed(); see g_ledSnapshot).
    j += "\"led\":{";
    j += "\"color1\":\"";  j += ledColorHex(g_ledSnapshot.color1);     j += "\",";
    j += "\"color2\":\"";  j += ledColorHex(g_ledSnapshot.color2);     j += "\",";
    j += "\"pattern\":\""; j += ledPatternName(g_ledSnapshot.pattern); j += "\",";
    j += "\"envelope\":";  j += (unsigned)g_ledEnvelope;
    j += "},";
    j += "\"valveOpenAng\":";   j += (unsigned)g_valveOpenAng;       j += ",";
    j += "\"valveCloseAng\":";  j += (unsigned)g_valveCloseAng;      j += ",";
    j += "\"valveIsOpen\":";    j += (g_valveIsOpen ? "true" : "false"); j += ",";
    j += "\"stepDelayFastUs\":"; j += (unsigned)g_stepDelayFastUs;   j += ",";
    j += "\"dwellSec\":";       j += (unsigned)g_postValveDwellSec;  j += ",";
    j += "\"ledBrightness\":";  j += (unsigned)g_ledBrightness;      j += ",";
    j += "\"motorInverted\":";  j += (g_motorInverted ? "true" : "false"); j += ",";
    j += "\"gapValue\":"; j += String(g_gapValue, 3); j += ",";
    j += "\"scaleOK\":";       j += (scaleOK ? "true" : "false"); j += ",";
    j += "\"scaleCalibrated\":"; j += (g_scaleCalibrated ? "true" : "false"); j += ",";
    // Cal wizard progress: true once Step 1 (empty-cell zero) has been
    // captured this session, so the UI can confirm Step 1 landed and
    // prompt for Step 2 (add mass + Calibrate). Survives the ownership
    // watchdog timeout -- the captured zero is retained, so a later
    // Calibrate still pairs with it. Cleared by reboot, factory reset,
    // or a completed calibrate.
    j += "\"zeroArmed\":"; j += (g_rawTareSession ? "true" : "false"); j += ",";
    {
        // Live raw ADC counts (reg 0x00), independent of calibration.
        // Lets the operator confirm the cell is responding in real time
        // during the wizard -- watch counts move as mass is added, even
        // before any gap is set. null on a bus read failure. rawTare is
        // the captured Step-1 zero (0 until captured) so the UI can show
        // the live delta that Step 2 will divide by the known grams.
        int32_t rawNow = 0;
        if (scale.getRawADC(rawNow)) { j += "\"scaleRaw\":"; j += (long)rawNow; j += ","; }
        else                         { j += "\"scaleRaw\":null,"; }
        j += "\"rawTare\":"; j += (long)g_rawTare; j += ",";
    }
    {
        // Live cell weight in grams, or null when unreadable / not yet
        // calibrated (weightFromRaw returns NAN). The UI shows this as
        // the scale's live readout once calibrated.
        float wg = weightFromRaw();
        if (isnan(wg)) j += "\"weightG\":null,";
        else { j += "\"weightG\":"; j += String(wg, 1); j += ","; }
    }
    j += "\"fastSchedule\":"; j += (g_debugFastSchedule ? "true" : "false"); j += ",";
    j += "\"schedulerEnabled\":"; j += (g_schedulerEnabled ? "true" : "false"); j += ",";
    j += "\"timeMode\":\""; j += (g_timeIsWallClock ? "ntp" : "uptime"); j += "\",";
    if (g_timeIsWallClock) {
        time_t now;
        time(&now);
        struct tm t;
        localtime_r(&now, &t);
        char tbuf[32];
        snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
        j += "\"localTime\":\""; j += tbuf; j += "\",";
    }
    {
        uint32_t secUntil = secUntilNextDue();
        long emit = (secUntil == 0xFFFFFFFFUL) ? -1 : (long)secUntil;
        j += "\"nextCycleSec\":"; j += emit; j += ",";
    }
    int rssi = (WiFi.status() == WL_CONNECTED) ? (int)WiFi.RSSI() : 0;
    j += "\"wifiRssi\":";   j += rssi;                          j += ",";
    j += "\"uptimeMs\":";   j += (unsigned long)millis();       j += ",";
    j += "\"abortableMotion\":";   j += (g_abortableMotion ? "true" : "false"); j += ",";
    j += "\"testMode\":";          j += (g_testModeActive ? "true" : "false");  j += ",";
    // Demo block: lets the web UI show "DEMO RUNNING" indicators and
    // sync the pause input. camelCase here to match the rest of
    // /api/state (the standalone GET /api/demo uses snake_case).
    j += "\"demo\":{";
    j += "\"active\":";     j += (g_demoMode != DemoMode::Off ? "true" : "false"); j += ",";
    j += "\"mode\":\"";     j += demoModeName(g_demoMode);            j += "\",";
    j += "\"cycleCount\":"; j += (unsigned long)g_demoCycleCount;     j += ",";
    j += "\"pauseSec\":";   j += (unsigned)g_demoPauseSec;
    j += "},";
    j += "\"activeBucket\":";      j += g_activeBucket;                          j += ",";
    j += "\"interactionParkedBucket\":"; j += g_interactionParkedBucket;          j += ",";
    j += "\"activeDeliveredMl\":"; j += String(g_activeBucketDeliveredMl, 1);    j += ",";
    j += "\"activeTargetMl\":";    j += g_activeBucketTargetMl;                  j += ",";
    j += "\"cycleBucketIdx\":";    j += g_cycleBucketIdx;                        j += ",";
    j += "\"cycleBucketTotal\":";  j += g_cycleBucketTotal;                      j += ",";
    j += "\"fault\":{\"code\":\""; j += faultName(g_lastFault); j += "\",";
    j += "\"bucket\":";     j += g_faultBucket;
    j += ",\"msAgo\":";     j += (g_faultMs > 0)
                                    ? (unsigned long)(millis() - g_faultMs)
                                    : 0UL;
    j += "},";
    // Deferral status -- a separate signal from latched faults. A cycle
    // defers (rather than faults) on tilt, motion, or low reservoir;
    // the web UI surfaces this so the user knows why the rig hasn't
    // watered recently even though nothing is in ERROR.
    j += "\"defer\":{";
    j += "\"reason\":\""; j += deferReasonName(g_lastDeferReason); j += "\",";
    if (g_lastDeferReason != DEFER_NONE) {
        j += "\"ageMs\":"; j += (unsigned long)(millis() - g_deferStartedMs);
    } else {
        j += "\"ageMs\":0";
    }
    j += "},";
    // IMU block. Raw axes are exposed during bring-up so the user
    // can verify mount orientation visually -- they hold the rig in
    // the intended "level" pose and read which board axis sits at
    // ~+1.0 g. Once mount is confirmed, the raw fields are harmless
    // to keep (only a few extra bytes per poll).
    if (g_imuPresent) {
        const char* tn = "?";
        switch (g_imuType) {
            case m5::imu_mpu6886: tn = "MPU6886"; break;
            case m5::imu_bmi270:  tn = "BMI270";  break;
            case m5::imu_mpu6050: tn = "MPU6050"; break;
            case m5::imu_mpu9250: tn = "MPU9250"; break;
            case m5::imu_sh200q:  tn = "SH200Q";  break;
            default:              tn = "unknown"; break;
        }
        j += "\"imu\":{\"present\":true,";
        j += "\"type\":\"";            j += tn; j += "\",";
        j += "\"pitchDeg\":";          j += String(g_pitchDeg, 1);     j += ",";
        j += "\"rollDeg\":";           j += String(g_rollDeg, 1);      j += ",";
        j += "\"motionMetric\":";      j += String(g_motionMetric, 3); j += ",";
        j += "\"axRaw\":";             j += String(g_axRaw, 3);        j += ",";
        j += "\"ayRaw\":";             j += String(g_ayRaw, 3);        j += ",";
        j += "\"azRaw\":";             j += String(g_azRaw, 3);        j += ",";
        j += "\"spillThreshDeg\":";    j += String(g_spillThreshDeg, 1); j += ",";
        j += "\"motionThreshG\":";     j += String(g_motionThreshG, 3);  j += ",";
        j += "\"mountOrientation\":\""; j += mountName(g_mountOrientation); j += "\",";
        j += "\"level\":";  j += (spillOk()  ? "true"  : "false"); j += ",";
        j += "\"moving\":"; j += (motionOk() ? "false" : "true");
        j += "},";
    } else {
        j += "\"imu\":{\"present\":false},";
    }
    j += "\"buckets\":[";
    for (int i = 0; i < g_bucketCount; i++) {
        if (i) j += ",";
        j += "{\"idx\":";      j += (i + 1);
        j += ",\"doseMl\":";   j += g_buckets[i].dose_ml;
        j += ",\"intervalH\":"; j += g_buckets[i].interval_hours;
        {
            uint32_t now = scheduleTimeNow();
            uint32_t intervalSec = intervalToSec(g_buckets[i].interval_hours);
            uint32_t elapsed = now - g_lastWateredAt[i];
            bool due = (g_buckets[i].dose_ml != DOSE_SKIP) && (elapsed >= intervalSec);
            long secLeft = (intervalSec > elapsed) ? (long)(intervalSec - elapsed) : 0L;
            j += ",\"due\":"; j += (due ? "true" : "false");
            j += ",\"secUntilDue\":"; j += secLeft;
        }
        j += "}";
    }
    j += "]}";
    return j;
}

// AP-mode entry prompt at boot. Shown BEFORE motion init so the user
// can force config mode even on a rig that would otherwise brick on
// homing failure. The button must be held for the entire window;
// releasing at any point exits to normal STA boot.
static bool checkApModeRequest() {
    AtomS3.Display.fillScreen(UI_BLACK);
    AtomS3.Display.setTextSize(2);
    AtomS3.Display.setTextColor(UI_WHITE, UI_BLACK);
    AtomS3.Display.setCursor(2, 4);
    AtomS3.Display.println("Hold BtnA");
    AtomS3.Display.println("for AP");
    AtomS3.Display.println("mode 5s");
    LOGI("%s\n", "AP-mode prompt: hold BtnA for 5s to enter config mode");

    uint32_t startMs = millis();
    uint32_t shownSec = 999;
    while (millis() - startMs < AP_BUTTON_HOLD_MS) {
        AtomS3.update();
        if (!M5.BtnA.isPressed()) {
            AtomS3.Display.fillScreen(UI_BLACK);
            return false;
        }
        uint32_t left = (AP_BUTTON_HOLD_MS - (millis() - startMs)) / 1000 + 1;
        if (left != shownSec) {
            AtomS3.Display.fillScreen(UI_BLACK);
            AtomS3.Display.setCursor(2, 4);
            AtomS3.Display.setTextSize(2);
            AtomS3.Display.println("AP MODE");
            AtomS3.Display.printf("in %lus...", (unsigned long)left);
            shownSec = left;
        }
        delay(20);
    }
    AtomS3.Display.fillScreen(UI_BLACK);
    LOGI("%s\n", "AP mode requested (button held >=5s)");
    return true;
}

// Derive this unit's network identity from the ESP32 factory MAC so
// two Pluvio units on the same network don't clash on hostname, mDNS
// name, or AP SSID -- no user setup required. The suffix is the last
// three bytes of the 48-bit factory MAC (the device-specific half; the
// first three are M5Stack's shared OUI) rendered as 6 lowercase hex
// digits. Lowercase keeps the hostname/mDNS name DNS-friendly (names are
// case-insensitive, lowercase is conventional) and -- crucially -- the
// exact same suffix string is reused for the AP SSID, so a user can
// match the portal name to the device. Call this early in setup(),
// before any WiFi/AP work, so the identity is logged up front and is
// ready when runConfigPortalBlocking()/connectWifiSTA() read it.
//
// BRANDING MIGRATION: WIFI_HOSTNAME/AP_SSID moved from "drip"/"Drip-Setup"
// to "pluvio"/"Pluvio-Setup". Units provisioned under the old
// "drip-xxxxxx" hostname will change their mDNS and DHCP names on this
// update -- any bookmark or automation pointing at "drip-xxxxxx.local"
// breaks and must be repointed at "pluvio-xxxxxx.local". User-set
// friendly names in NVS ("friendlyName") are untouched and keep working.
// Acceptable pre-launch; the immutable "device" mDNS TXT record (also
// MAC-derived) likewise changes prefix, but the fleet manager keys on
// its value as a whole, not the "drip"/"pluvio" prefix.
static void initDeviceIdentity() {
    uint64_t mac = ESP.getEfuseMac();   // 48-bit factory MAC, low 6 bytes
    char suf[7];
    snprintf(suf, sizeof(suf), "%02x%02x%02x",
             (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
    g_deviceSuffix = suf;
    g_deviceName   = String(WIFI_HOSTNAME) + "-" + g_deviceSuffix;  // e.g. pluvio-bccca0
    g_apSsid       = String(AP_SSID) + "-" + g_deviceSuffix;        // e.g. Pluvio-Setup-bccca0
    LOGI("Device identity: name=%s ap=%s\n",
         g_deviceName.c_str(), g_apSsid.c_str());
}

// Set by WiFiManager's save callback the instant credentials are
// written to NVS -- which happens BEFORE the portal's own in-place
// connect attempt. We watch this flag (not the connect result) so a
// sealed, battery-only unit reboots itself even when that in-place
// connect hangs in waitForConnectResult. volatile: written from the
// WiFiManager callback, read in the portal service loop.
static volatile bool g_wifiCredsSaved = false;

// Run the WiFiManager captive portal until the user saves credentials,
// then reboot. Never returns. The config-portal timeout stays disabled
// because the rig may sit in config mode for a while while the user
// finds their phone.
//
// SEALED-UNIT SELF-RECOVERY: this firmware ships in a battery-powered
// enclosure with no reset button and no way to power-cycle, so the
// portal MUST get itself out of config mode after a save without any
// physical intervention. The blocking portal's in-place connect can
// hang indefinitely in waitForConnectResult, so we instead run the
// portal non-blocking and reboot the moment the save callback fires
// (g_wifiCredsSaved) -- the credentials are already in NVS at that
// point, and the reboot re-runs connectWifiSTA(), the reliable connect
// path. Bounded connect timeouts are a belt-and-suspenders backstop in
// case startConfigPortal() ever returns on its own.
static void runConfigPortalBlocking() {
    g_apMode = true;
    AtomS3.Display.fillScreen(UI_BLACK);
    AtomS3.Display.setTextSize(1);
    AtomS3.Display.setTextColor(UI_WHITE, UI_BLACK);
    AtomS3.Display.setCursor(2, 2);
    AtomS3.Display.println("Config Mode");
    AtomS3.Display.println();
    // Per-device SSID (g_apSsid, e.g. "Pluvio-Setup-bccca0", 19 chars).
    // At the size-1 font (~6px/char) the SSID alone is ~114px and still
    // fits the 128px panel -- but "SSID: " + the SSID on one line would
    // overflow and wrap, splitting the distinguishing suffix across two
    // rows. So the label and value get their own lines; this adds one
    // row and the prompt still ends well above the 128px bottom edge.
    AtomS3.Display.println("SSID:");
    AtomS3.Display.println(g_apSsid);
    AtomS3.Display.print("PW:   ");
    AtomS3.Display.println(AP_PASSWORD);
    AtomS3.Display.println();
    AtomS3.Display.println("192.168.4.1");

    LOGI("Starting config portal: SSID=%s PW=%s @ 192.168.4.1\n",
                  g_apSsid.c_str(), AP_PASSWORD);

    g_wifiCredsSaved = false;
    WiFiManager wm;
    // Fires the instant WiFiManager commits credentials to NVS, BEFORE
    // its own connect attempt -- so we get the signal even if that
    // connect then hangs.
    wm.setSaveConfigCallback([]() {
        g_wifiCredsSaved = true;
        LOGI("%s\n", "WiFi creds saved (callback); will reboot into STA mode");
    });
    // Non-blocking: startConfigPortal() returns immediately and we drive
    // the DNS/web server ourselves via wm.process() below, so a hung
    // in-place connect can't trap us.
    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(0);   // never auto-exit the portal on idle
    // Bound the in-place connect so it can't sit forever in
    // waitForConnectResult. A non-zero save timeout is what dodges the
    // root-cause bug: with the default (0) WiFiManager falls through to
    // ESP's WiFi.waitForConnectResult(), which can block indefinitely;
    // a non-zero value switches it to a millis()-bounded poll instead.
    wm.setConnectTimeout(10);
    wm.setSaveConnectTimeout(10);
    // CRITICAL for sealed units: fire the save callback even when the
    // portal's in-place connect FAILS. In non-blocking mode with the
    // default (false), a save whose in-place connect fails leaves the
    // portal "remaining open" and never calls the callback -- so we'd
    // spin here forever. The in-place connect frequently fails while the
    // radio juggles AP+STA, yet the credentials are already committed to
    // NVS, so the post-reboot connectWifiSTA() succeeds. Breaking after
    // config guarantees the callback fires on save regardless of the
    // in-place connect result, which is the signal we reboot on.
    wm.setBreakAfterConfig(true);
    wm.startConfigPortal(g_apSsid.c_str(), AP_PASSWORD);

    // Low-duty portal for the battery-backed sealed unit. The portal is
    // timeout-less (always provisionable), so it can stay up a long time
    // -- and because we never return to loop() from here,
    // manageDisplayPower() never runs to idle the panel. So we dim the
    // LCD ourselves after a readable window (kept ON, not slept, so the
    // SSID stays legible if the user comes back) and drop WiFi TX power.
    // The radio dominates AP draw and must stay up to remain joinable, so
    // 8.5 dBm -- still ample for a phone beside the unit -- is as low as
    // we go without hurting discoverability.
    AtomS3.Display.setBrightness(DISPLAY_BRIGHTNESS_ACTIVE);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    uint32_t portalStartMs = millis();
    bool portalDimmed = false;

    // Service the portal until the user saves. The save callback fires
    // the moment NVS is written -- we reboot on that, NOT on a successful
    // in-place connect, so a sealed unit recovers itself. (startConfigPortal
    // returning true is a secondary exit in case a future lib version
    // ever completes the portal on its own.)
    while (!g_wifiCredsSaved) {
        if (wm.process()) break;   // portal completed its own connect
        if (!portalDimmed
            && (millis() - portalStartMs) >= DISPLAY_DIM_AFTER_MS) {
            AtomS3.Display.setBrightness(DISPLAY_BRIGHTNESS_DIM);
            portalDimmed = true;
        }
        delay(10);
    }

    // After save, clean reboot into STA mode. ESP.restart() never returns.
    LOGI("%s\n", "Config portal saved; rebooting into STA mode");
    delay(200);
    ESP.restart();
}

// Advertise both mDNS services with identity TXT records. Called from
// every site that runs MDNS.begin() (boot connect + late STA reconnect)
// so the two stay in sync. _http._tcp keeps the web UI discoverable;
// _pluvio-grow._tcp lets the LAN fleet manager browse one dedicated
// service type instead of filtering every HTTP device. Both carry the
// same TXT records the fleet manager inventories: "version", the
// immutable "device" identity (MAC-derived, never the friendly name),
// "name" (the user's friendly hostname, empty string when unset), and
// "product" -- the product identifier ("pluvio-grow-drip") so the
// manager can filter one service type across future product types. The
// manager tracks units by "device"; humans browse by hostname.
static void registerMdnsServices() {
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "version", FW_VERSION);
    MDNS.addServiceTxt("http", "tcp", "device",  g_deviceName.c_str());
    MDNS.addServiceTxt("http", "tcp", "name",    g_friendlyName.c_str());
    MDNS.addServiceTxt("http", "tcp", "product", "pluvio-grow-drip");
    MDNS.addService("pluvio-grow", "tcp", 80);
    MDNS.addServiceTxt("pluvio-grow", "tcp", "version", FW_VERSION);
    MDNS.addServiceTxt("pluvio-grow", "tcp", "device",  g_deviceName.c_str());
    MDNS.addServiceTxt("pluvio-grow", "tcp", "name",    g_friendlyName.c_str());
    MDNS.addServiceTxt("pluvio-grow", "tcp", "product", "pluvio-grow-drip");
}

// Connect using credentials WiFiManager has stored in NVS. Returns
// true on success. On failure, caller decides whether to enter AP
// mode (we don't, by design -- the rig must run without WiFi).
static bool connectWifiSTA() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(activeHostname().c_str());   // friendly name if set, else per-device
    WiFi.begin();   // no args: uses last saved credentials in NVS
    LOGI("WiFi: connecting (saved SSID=%s)\n", WiFi.SSID().c_str());

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED
           && (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(200);
    }
    if (WiFi.status() != WL_CONNECTED) {
        LOGI("%s\n", "WiFi: STA connect FAILED (continuing without WiFi)");
        return false;
    }
    IPAddress ip = WiFi.localIP();
    LOGI("WiFi: connected. IP=%s RSSI=%d\n",
                  ip.toString().c_str(), (int)WiFi.RSSI());
    if (MDNS.begin(activeHostname().c_str())) {   // friendly name if set, else per-device
        registerMdnsServices();
        LOGI("mDNS responder started: http://%s.local/\n",
                      activeHostname().c_str());
    } else {
        LOGI("%s\n", "mDNS responder failed to start");
    }
    return true;
}

// Tiny JSON-response helper for the API endpoints below.
static void sendJson(int code, const String& body) {
    webServer.send(code, "application/json", body);
    // Mark a web request as a display interaction -- but NOT the
    // read-only polling endpoints. The dashboard polls /api/state
    // (~1.5s) and /api/log (~0.5s) continuously while a tab is open;
    // if those counted as interactions the idle timer would reset on
    // every poll and the panel would never dim or sleep. /api/demo is
    // a status read too. Everything else through here is a genuine user
    // action -- including the mutating GETs (/api/dose, /api/select,
    // /api/interval/set, /api/setvalveangles, /api/setdwell,
    // /api/setstepprofile) -- so a simple "POST only" rule would wrongly
    // drop those. Exclude by URI instead. Done AFTER send() so the rare
    // wake-path repaint never delays the HTTP response. The non-JSON
    // page handlers (/, /update) call markInteraction() directly.
    String uri = webServer.uri();
    if (uri != "/api/state" && uri != "/api/log" && uri != "/api/demo") {
        markInteraction();
    }
}

// Shared-token auth for mutating endpoints. With -DAPI_TOKEN="..." defined
// at build time, a request is authorized only if its "X-Api-Token" header
// matches. Absent (the default), auth is compiled out entirely and every
// request is authorized -- existing units keep their current behavior.
static inline bool apiTokenOk() {
#ifdef API_TOKEN
    return webServer.header("X-Api-Token") == API_TOKEN;
#else
    return true;
#endif
}

// Call first in every guarded (mutating) handler: `if (!checkAuth()) return;`.
// Returns true to proceed; on mismatch it sends the 401 itself and returns
// false. Only POST /api/* and POST /update are guarded -- GET endpoints stay
// open because the web UI polls them unauthenticated. NOTE: on a token build
// the local web UI's mutating buttons will 401 (the page sends no token);
// that's acceptable for now -- token units are fleet-managed.
static bool checkAuth() {
    if (!apiTokenOk()) {
        sendJson(401, "{\"ok\":false,\"reason\":\"unauthorized\"}");
        return false;
    }
    return true;
}

static void initWebServer() {
    webServer.on("/", HTTP_GET, []() {
        markInteraction();
        webServer.send_P(200, "text/html", webUiHtml);
    });

    webServer.on("/api/state", HTTP_GET, []() {
        sendJson(200, apiStateJson());
    });

    webServer.on("/api/water", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state != READY) {
            sendJson(200, "{\"ok\":false,\"reason\":\"not READY\"}");
            return;
        }
        g_webForceAll = true;
        g_webRequestWater = true;
        sendJson(200, "{\"ok\":true}");
    });

    // Layer 2 hook: water exactly one bucket on demand. Designed for
    // Home Assistant style automations -- a single non-idempotent
    // POST that bypasses the schedule for one bucket, leaving every
    // other bucket's last-watered timestamp untouched. Bucket index
    // is 1-based to match the rest of the API.
    webServer.on("/api/water/bucket", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state != READY) {
            sendJson(200, "{\"ok\":false,\"reason\":\"not READY\"}");
            return;
        }
        if (!webServer.hasArg("n")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing n\"}");
            return;
        }
        int n = webServer.arg("n").toInt();
        if (n < 1 || n > g_bucketCount) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bad bucket\"}");
            return;
        }
        if (g_buckets[n - 1].dose_ml == DOSE_SKIP) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bucket is skip\"}");
            return;
        }
        g_webRequestSingleN = n;
        g_webRequestSingle  = true;
        LOG("Water single bucket %d requested (web)\n", n);
        sendJson(200, "{\"ok\":true,\"bucket\":" + String(n) + "}");
    });

    webServer.on("/api/abort", HTTP_POST, []() {
        if (!checkAuth()) return;
        // Cancel any foreground motion: a watering cycle, or any
        // long-running diagnostic test that took an AbortableMotionGuard.
        // serviceAbort() inside the motion primitives consumes the
        // request on the next tick.
        if (g_state != WATERING && !g_abortableMotion) {
            sendJson(200, "{\"ok\":false,\"reason\":\"no abortable motion\"}");
            return;
        }
        g_abortRequested = true;
        LOG("%s\n", "ABORT (web)");
        sendJson(200, "{\"ok\":true}");
    });

    webServer.on("/api/home", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (!canRunMotorOps()) {
            sendJson(200, "{\"ok\":false,\"reason\":\"not READY\"}");
            return;
        }
        g_webRequestHome = true;
        sendJson(200, "{\"ok\":true}");
    });

    // Calibration step 1: capture the empty-cell zero (RAM only; the
    // gap commit in /api/calibrate persists zero+gap atomically). This
    // replaces the old standalone /api/tare action.
    webServer.on("/api/scale/zero", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        // Not gated on scaleOK: cal runs on raw counts and is what
        // recovers an unhealthy scale. captureScaleZero()'s checked raw
        // read is the real gate -- a genuine bus failure returns a
        // visible "scale not responding".
        scale.setOffset();
        if (!captureScaleZero()) {
            sendJson(200, "{\"ok\":false,\"reason\":\"scale not responding\"}");
            return;
        }
        markScaleLiveFromCal();   // proven-live raw read
        sendJson(200, "{\"ok\":true}");
    });

    // Factory reset of scale calibration: clears the calibrated flag,
    // zero and gap, returning the scale to uncalibrated (re-exposes the
    // calibrate wizard). Forbidden during WATERING.
    webServer.on("/api/scale/reset", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        persistResetScaleCal();
        LOG("%s\n", "Scale calibration factory-reset via web");
        sendJson(200, "{\"ok\":true,\"scaleCalibrated\":false}");
    });

    webServer.on("/api/clearfault", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state != ERROR) {
            sendJson(200, "{\"ok\":false,\"reason\":\"not ERROR\"}");
            return;
        }
        clearFault("web");
        sendJson(200, "{\"ok\":true}");
    });

    webServer.on("/api/dose", HTTP_GET, []() {
        if (!webServer.hasArg("bucket") || !webServer.hasArg("delta")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing args\"}");
            return;
        }
        int n     = webServer.arg("bucket").toInt();
        int delta = webServer.arg("delta").toInt();
        if (n < 1 || n > g_bucketCount) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bad bucket\"}");
            return;
        }
        int idx     = n - 1;
        int newDose = (int)g_buckets[idx].dose_ml + delta;
        // Snap to dose ladder: 0 (skip), 10..60 in 10-step intervals.
        // Anything below DOSE_MIN_ML collapses to DOSE_SKIP; above
        // DOSE_MAX_ML wraps back to DOSE_SKIP. Matches the BtnA
        // double-press wraparound (skip -> 10 -> ... -> 60 -> skip)
        // so web/button paths agree on the ladder.
        if (newDose < (int)DOSE_MIN_ML) newDose = DOSE_SKIP;
        if (newDose > (int)DOSE_MAX_ML) newDose = DOSE_SKIP;
        g_buckets[idx].dose_ml = (uint16_t)newDose;
        persistMarkDose(idx);
        String r = "{\"ok\":true,\"bucket\":";
        r += n;
        r += ",\"doseMl\":";
        r += newDose;
        r += "}";
        sendJson(200, r);
    });

    webServer.on("/api/interval/set", HTTP_GET, []() {
        if (!webServer.hasArg("bucket") || !webServer.hasArg("hours")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing args\"}");
            return;
        }
        int n = webServer.arg("bucket").toInt();
        int h = webServer.arg("hours").toInt();
        if (n < 1 || n > g_bucketCount) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bad bucket\"}");
            return;
        }
        bool valid = (h == 12 || h == 24 || h == 48 || h == 72
                      || h == 120 || h == 168 || h == 240
                      || h == 336 || h == 504);
        if (!valid) {
            sendJson(200, "{\"ok\":false,\"reason\":\"invalid interval\"}");
            return;
        }
        g_buckets[n - 1].interval_hours = (uint16_t)h;
        persistMarkInterval(n - 1);
        renderUI(true);
        String r = "{\"ok\":true,\"bucket\":";
        r += n;
        r += ",\"intervalH\":";
        r += h;
        r += "}";
        sendJson(200, r);
    });

    webServer.on("/api/skip", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (!webServer.hasArg("bucket")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing bucket\"}");
            return;
        }
        int n = webServer.arg("bucket").toInt();
        if (n < 1 || n > g_bucketCount) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bad bucket\"}");
            return;
        }
        g_buckets[n - 1].dose_ml = DOSE_SKIP;
        persistMarkDose(n - 1);
        sendJson(200, "{\"ok\":true,\"doseMl\":0}");
    });

    webServer.on("/api/menu", HTTP_GET, []() {
        // Read-only menu dump -- not a user action, don't mark interaction.
        webServer.send_P(200, "text/plain", webMenuTextP);
    });

    // Diagnostic test triggers. All are gated on g_state == READY so a
    // running cycle never gets perturbed and so the foreground-task
    // dispatch in loop() can take the AbortableMotionGuard cleanly.
    // Body is just a flag set; loop() runs the actual test.
    webServer.on("/api/test/dispense", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (!canRunMotorOps()) {
            sendJson(409, "{\"ok\":false,\"reason\":\"not READY\"}");
            return;
        }
        // Wet test (valve actuates) requires explicit ?wet=1; absent
        // or any other value defaults to dry, matching the safer
        // bare-'D' serial default.
        g_testModeWet     = (webServer.arg("wet") == "1");
        g_webRequestDtest = true;
        sendJson(200, "{\"ok\":true,\"msg\":\"queued\"}");
    });
    // Stepper motor test: spin continuously until stopped. dir=fwd or
    // dir=rev. Foreground-dispatched via g_webRequestRotate so the
    // request handler doesn't block on the spin loop. OPEN-LOOP blind tool:
    // NOT gated on canRunMotorOps()/encoder health (it's a "sensors absent,
    // move blind" debug op) -- only refused while WATERING (in
    // rotateContinuous). Voids position trust. Normal closed-loop motion is
    // unaffected and stays gated.
    webServer.on("/api/stepper/rotate", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(409, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        if (!webServer.hasArg("dir")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing dir\"}");
            return;
        }
        String d = webServer.arg("dir");
        int dir = 0;
        if      (d == "fwd") dir = +1;
        else if (d == "rev") dir = -1;
        else {
            sendJson(200, "{\"ok\":false,\"reason\":\"dir must be fwd or rev\"}");
            return;
        }
        g_webRotateDir     = dir;
        g_webRequestRotate = true;
        sendJson(200, "{\"ok\":true,\"msg\":\"queued\"}");
    });

    // Stop a running rotateContinuous. Always safe: setting the
    // abort flag is harmless when nothing is spinning, and the next
    // serviceAbort() poll inside rotateContinuous picks it up.
    webServer.on("/api/stepper/stop", HTTP_POST, []() {
        if (!checkAuth()) return;
        g_abortRequested = true;
        LOG("%s\n", "STOP (web /api/stepper/stop)");
        sendJson(200, "{\"ok\":true}");
    });

    // POST /api/jog?dir=fwd|rev[&steps=N]  -- OPEN-LOOP BLIND JOG (debug).
    // The one motor op NOT gated on encoder health / canRunMotorOps(): it
    // moves a bounded, fixed number of steps with no position feedback so
    // the motor/coils/mechanics can be exercised when the AS5600 is dead.
    // It VOIDS position trust (forces a recal before normal motion) and is
    // refused only while WATERING. Foreground-dispatched (runs in loop(),
    // not the request thread) so it executes even in ERROR/FAULT_ENCODER_LOST.
    webServer.on("/api/jog", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(409, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        if (!webServer.hasArg("dir")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing dir (fwd|rev)\"}");
            return;
        }
        String ds = webServer.arg("dir");
        int dir = 0;
        if      (ds == "fwd" || ds == "1"  || ds == "+") dir = +1;
        else if (ds == "rev" || ds == "-1" || ds == "-") dir = -1;
        else {
            sendJson(200, "{\"ok\":false,\"reason\":\"dir must be fwd or rev\"}");
            return;
        }
        int steps = webServer.hasArg("steps")
                  ? webServer.arg("steps").toInt() : JOG_DEFAULT_STEPS;
        if (steps <= 0) steps = JOG_DEFAULT_STEPS;
        g_webJogDir     = dir;
        g_webJogSteps   = steps;
        g_webRequestJog = true;
        LOG("Open-loop jog requested (web): %s %d steps\n",
            dir > 0 ? "fwd" : "rev", steps);
        sendJson(200, "{\"ok\":true,\"msg\":\"queued (open-loop blind; voids "
                      "position trust)\"}");
    });

    // ---- Demo mode ----
    // POST /api/demo/start?mode=dry|wet|random  -- start the continuous demo
    // loop. Foreground-dispatched (the blocking loop runs from loop()'s
    // READY branch, not this request thread). Requires READY.
    webServer.on("/api/demo/start", HTTP_POST, []() {
        if (!checkAuth()) return;
        // Arm-gate: must be fully idle, not merely READY. A bare g_state check
        // let a demo be armed mid-calibrate/rotate/jog (g_abortableMotion, but
        // still READY) and leak past a fault (Finding 1). deviceIsIdle() also
        // rejects when another motion request is already queued.
        if (!deviceIsIdle()) {
            sendJson(409, "{\"ok\":false,\"reason\":\"not idle\"}");
            return;
        }
        if (g_demoMode != DemoMode::Off || g_webRequestDemoStart) {
            sendJson(200, "{\"ok\":false,\"reason\":\"demo already running\"}");
            return;
        }
        String m = webServer.arg("mode");
        DemoMode want;
        if      (m == "dry")    want = DemoMode::Dry;
        else if (m == "wet")    want = DemoMode::Wet;
        else if (m == "random") want = DemoMode::Random;
        else {
            sendJson(200, "{\"ok\":false,\"reason\":\"mode must be dry, wet, or random\"}");
            return;
        }
        // Random's wet/dry is caller-selectable and orthogonal to the
        // random bucket pick. Optional &wet=0|1; default DRY (0) -- the
        // safe default and the motion cadence-test mode. mode=random&wet=1
        // is wet-random. (Dry/Wet modes carry wet/dry in the mode itself.)
        if (want == DemoMode::Random)
            g_demoRandomWet = (webServer.arg("wet") == "1");
        // Arm only: set the PENDING mode + request flag. runDemoLoop() promotes
        // it to g_demoMode on real entry (never here) -- see g_demoPendingMode.
        g_demoPendingMode     = want;
        g_webRequestDemoStart = true;
        bool demoWet = (want == DemoMode::Wet)
                       || (want == DemoMode::Random && g_demoRandomWet);
        LOG("Demo start requested (web): %s (%s)\n",
            demoModeName(want), demoWet ? "wet" : "dry");
        String r = "{\"ok\":true,\"mode\":\"";
        r += demoModeName(want);
        r += "\",\"wet\":";
        r += (demoWet ? "true" : "false");
        r += "}";
        sendJson(200, r);
    });

    // POST /api/demo/stop  -- leave demo mode. Sets the stop flag AND
    // g_abortRequested so it breaks an in-flight dispense too (the
    // running demo loop, blocked inside runWateringCycle, only sees the
    // abort path mid-cycle; the stop flag is the between-cycle signal).
    webServer.on("/api/demo/stop", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_demoMode == DemoMode::Off && !g_webRequestDemoStart) {
            sendJson(200, "{\"ok\":false,\"reason\":\"not in demo\"}");
            return;
        }
        g_webRequestDemoStop = true;
        g_abortRequested     = true;
        LOG("%s\n", "Demo stop requested (web)");
        sendJson(200, "{\"ok\":true}");
    });

    // POST /api/demo/config?pause_s=N  -- set the inter-cycle pause
    // (1..10s, RAM-only). Safe to call while a demo is running; the
    // new value takes effect on the next pause.
    webServer.on("/api/demo/config", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (!webServer.hasArg("pause_s")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing pause_s\"}");
            return;
        }
        int v = webServer.arg("pause_s").toInt();
        if (v < DEMO_PAUSE_MIN_SEC || v > DEMO_PAUSE_MAX_SEC) {
            sendJson(200, "{\"ok\":false,\"reason\":\"pause_s out of range 1..10\"}");
            return;
        }
        g_demoPauseSec = (uint8_t)v;
        LOG("Demo pause set to %us (web)\n", (unsigned)g_demoPauseSec);
        String r = "{\"ok\":true,\"pause_s\":";
        r += (unsigned)g_demoPauseSec;
        r += "}";
        sendJson(200, r);
    });

    // GET /api/demo  -- {active, mode, cycle_count, pause_s}
    webServer.on("/api/demo", HTTP_GET, []() {
        String r = "{\"active\":";
        r += (g_demoMode != DemoMode::Off ? "true" : "false");
        r += ",\"mode\":\"";
        r += demoModeName(g_demoMode);
        r += "\",\"cycle_count\":";
        r += (unsigned long)g_demoCycleCount;
        r += ",\"pause_s\":";
        r += (unsigned)g_demoPauseSec;
        r += "}";
        sendJson(200, r);
    });

    // POST /api/calibrate
    //   - with ?grams=N -- U177 scale calibration (runs inline).
    //   - without args  -- endstop calibration (queued; loop() runs
    //                      it in the foreground).
    // Single endpoint, two modes -- the JS clients dispatch by
    // whether they're calibrating the scale (passes grams) or the
    // endstops (no args).
    webServer.on("/api/calibrate", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (webServer.hasArg("grams")) {
            if (g_state == WATERING) {
                sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
                return;
            }
            // Not gated on scaleOK: the checked getRawADC below is the
            // real liveness gate, and cal on raw counts is what recovers
            // an unhealthy scale. A genuine bus failure returns a visible
            // "scale not responding".
            float known = webServer.arg("grams").toFloat();
            if (!(known > 0.0f)) {
                sendJson(200, "{\"ok\":false,\"reason\":\"grams must be > 0\"}");
                return;
            }
            // See serial 'c' for the tared-delta rationale.
            if (!g_rawTareSession) {
                sendJson(200, "{\"ok\":false,\"reason\":\"capture zero first (cell empty)\"}");
                return;
            }
            // Checked read: distinguishes an I2C failure from a real value,
            // so a flaky bus can never silently compute a gap from
            // getRawADC()'s 0-on-failure return. Doubles as the liveness
            // check the old getFirmwareVersion probe provided.
            int32_t raw = 0;
            if (!scale.getRawADC(raw)) {
                g_scaleCalActive = false;
                sendJson(200, "{\"ok\":false,\"reason\":\"scale not responding\"}");
                return;
            }
            markScaleLiveFromCal();   // proven-live raw read
            int32_t delta  = raw - g_rawTare;
            int32_t adelta = delta < 0 ? -delta : delta;
            if (adelta < SCALE_CAL_MIN_DELTA) {
                g_scaleCalActive = false;
                String e = "{\"ok\":false,\"reason\":\"no weight change (delta=";
                e += String((long)delta);
                e += ") -- is the mass on the cell?\"}";
                sendJson(200, e);
                return;
            }
            // NVS-independent: compute purely from the live delta and the
            // entered grams, then overwrite device gap + g_gapValue + NVS.
            // Whatever NVS or the U177 held before is irrelevant here.
            float newGap  = (float)delta / known;
            scale.setGapValue(newGap);
            g_gapValue = newGap;
            // Atomic commit: zero + gap persisted together (never desync).
            persistCommitScaleCal();
            g_scaleCalActive = false;   // committed -> release scale ownership
            LOG("Calibrated via web: rawADC=%ld rawTare=%ld delta=%ld "
                "known=%.2fg -> gapValue=%.3f\n",
                (long)raw, (long)g_rawTare, (long)delta,
                (double)known, (double)newGap);
            String r = "{\"ok\":true,\"gapValue\":";
            r += String(newGap, 3);
            r += "}";
            sendJson(200, r);
            return;
        }
        // Endstop calibration mode.
        if (!canRunMotorOps()) {
            sendJson(409, "{\"ok\":false,\"reason\":\"not READY\"}");
            return;
        }
        g_webRequestCalibrate = true;   // foreground-dispatch flag
        sendJson(200, "{\"ok\":true,\"msg\":\"queued\"}");
    });

    // GET /api/setvalveangles?open=N&close=M  -- update servo angles
    // for valve open / closed. Persists. Both args required.
    webServer.on("/api/setvalveangles", HTTP_GET, []() {
        if (!webServer.hasArg("open") || !webServer.hasArg("close")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"need open= and close= (0..180)\"}");
            return;
        }
        int o = webServer.arg("open").toInt();
        int c = webServer.arg("close").toInt();
        if (o < 0 || o > 180 || c < 0 || c > 180) {
            sendJson(200, "{\"ok\":false,\"reason\":\"out of range 0..180\"}");
            return;
        }
        g_valveOpenAng  = (uint8_t)o;
        g_valveCloseAng = (uint8_t)c;
        persistSaveValveAngles();
        LOG("Valve angles set via web: open=%u close=%u (saved)\n",
            (unsigned)g_valveOpenAng, (unsigned)g_valveCloseAng);
        String r = "{\"ok\":true,\"valveOpenAng\":";
        r += (unsigned)g_valveOpenAng;
        r += ",\"valveCloseAng\":";
        r += (unsigned)g_valveCloseAng;
        r += "}";
        sendJson(200, r);
    });

    // POST /api/valve  or  POST /api/valve?open=0|1
    // No body: toggle. With ?open=N: explicit. Forbidden during
    // WATERING -- the cycle owns the valve there. Returns the new
    // state so the UI button can re-label without an extra poll.
    webServer.on("/api/valve", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        bool wantOpen = !g_valveIsOpen;
        if (webServer.hasArg("open")) {
            wantOpen = (webServer.arg("open") == "1"
                       || webServer.arg("open") == "true");
        }
        if (wantOpen) valveOpen();
        else          valveClose();
        LOG("Valve %s via web\n", wantOpen ? "OPEN" : "CLOSED");
        String r = "{\"ok\":true,\"valveIsOpen\":";
        r += (wantOpen ? "true" : "false");
        r += "}";
        sendJson(200, r);
    });

    // GET /api/setdwell?value=N  -- sets post-valve-close dwell in seconds, persists
    webServer.on("/api/setdwell", HTTP_GET, []() {
        if (!webServer.hasArg("value")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing value\"}");
            return;
        }
        int v = webServer.arg("value").toInt();
        if (v < POST_VALVE_DWELL_MIN_SEC
            || v > POST_VALVE_DWELL_MAX_SEC) {
            sendJson(200,
                "{\"ok\":false,\"reason\":\"out of range 1..10\"}");
            return;
        }
        g_postValveDwellSec = (uint8_t)v;
        persistSaveDwell();
        LOG("Post-valve dwell set to %u s via web (saved)\n",
            (unsigned)g_postValveDwellSec);
        String r = "{\"ok\":true,\"dwellSec\":";
        r += (unsigned)g_postValveDwellSec;
        r += "}";
        sendJson(200, r);
    });

    // GET /api/setresvfull?value=NNN  -- reservoir-full volume (mL), the UI
    // gauge full-scale. Persists. Same shape as /api/setdwell. Display-only:
    // does NOT affect dispense or reservoir-low logic, so it's safe in any
    // state and the LCD/web gauges pick it up on their next redraw.
    webServer.on("/api/setresvfull", HTTP_GET, []() {
        if (!webServer.hasArg("value")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing value\"}");
            return;
        }
        float v = webServer.arg("value").toFloat();
        if (v < RESERVOIR_FULL_ML_MIN || v > RESERVOIR_FULL_ML_MAX) {
            sendJson(200,
                "{\"ok\":false,\"reason\":\"out of range 100..20000\"}");
            return;
        }
        g_reservoirFullMl = v;
        persistSaveReservoirFull();
        LOG("Reservoir full set to %.0f mL via web (saved)\n",
            (double)g_reservoirFullMl);
        String r = "{\"ok\":true,\"reservoirFullMl\":";
        r += String(g_reservoirFullMl, 1);
        r += "}";
        sendJson(200, r);
    });

    // GET /api/setledbrightness?value=N  -- status-LED master
    // brightness 0..255 (0 = LED off), persists. Same shape as
    // /api/setdwell. Safe in any state; updateLed() reads the value
    // live on its next tick.
    webServer.on("/api/setledbrightness", HTTP_GET, []() {
        if (!webServer.hasArg("value")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing value\"}");
            return;
        }
        int v = webServer.arg("value").toInt();
        if (v < LED_BRIGHTNESS_MIN || v > LED_BRIGHTNESS_MAX) {
            sendJson(200,
                "{\"ok\":false,\"reason\":\"out of range 0..255\"}");
            return;
        }
        g_ledBrightness = (uint8_t)v;
        persistSaveLedBrightness();
        LOG("Status-LED brightness set to %u via web (saved)\n",
            (unsigned)g_ledBrightness);
        String r = "{\"ok\":true,\"ledBrightness\":";
        r += (unsigned)g_ledBrightness;
        r += "}";
        sendJson(200, r);
    });

    // POST /api/setname?name=...  -- set or clear the friendly hostname.
    // Empty name= clears it (revert to g_deviceName). A non-empty value
    // is validated against the DNS hostname rules and rejected with
    // {ok:false} on failure. On success it persists, sets the hostname
    // for the next DHCP renew, and re-registers mDNS live so
    // <name>.local resolves without a reboot -- but the DHCP-advertised
    // hostname won't change until renewal, hence the note. Token-guarded
    // like the other mutating POSTs. Never touches g_deviceName, the AP
    // SSID, or the "device" TXT record.
    webServer.on("/api/setname", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (!webServer.hasArg("name")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing name\"}");
            return;
        }
        String name = webServer.arg("name");
        if (name.length() && !isValidFriendlyName(name)) {
            sendJson(200,
                "{\"ok\":false,\"reason\":\"invalid (a-z 0-9 hyphen, 1..32, start alnum)\"}");
            return;
        }
        g_friendlyName = name;            // empty string clears it
        persistSaveFriendlyName();
        // Apply live: setHostname() lands on the next DHCP renew; the
        // mDNS name changes now by tearing the responder down and
        // re-registering under activeHostname().
        WiFi.setHostname(activeHostname().c_str());
        MDNS.end();
        if (MDNS.begin(activeHostname().c_str())) {
            registerMdnsServices();
        }
        LOGI("Friendly name %s via web (saved): host now %s\n",
             g_friendlyName.length() ? "set" : "cleared",
             activeHostname().c_str());
        String r = "{\"ok\":true,\"friendlyName\":\"";
        r += g_friendlyName;
        r += "\",\"note\":\"hostname change fully applies after reboot\"}";
        sendJson(200, r);
    });

    // GET /api/setstepprofile?fast=N[&slow=N&ramp=N]
    // 'fast' is required and range-checked; persists. 'slow' / 'ramp'
    // args are accepted but ignored (single-rate motor now) so the
    // existing web UI POST body still validates server-side.
    webServer.on("/api/setstepprofile", HTTP_GET, []() {
        if (!webServer.hasArg("fast")) {
            sendJson(200,
                "{\"ok\":false,\"reason\":\"need fast=\"}");
            return;
        }
        int f = webServer.arg("fast").toInt();
        if (f < STEP_DELAY_FAST_US_MIN || f > STEP_DELAY_FAST_US_MAX) {
            sendJson(200, "{\"ok\":false,\"reason\":\"fast out of range\"}");
            return;
        }
        g_stepDelayFastUs = (uint16_t)f;
        persistSaveStepProfile();
        LOG("Stepper rate set via web: fast=%uus (saved)\n",
            (unsigned)g_stepDelayFastUs);
        String body = "{\"ok\":true,\"stepDelayFastUs\":";
        body += (unsigned)g_stepDelayFastUs;
        body += "}";
        sendJson(200, body);
    });

    // POST /api/imu/threshold?spill=<deg>&motion=<g>
    // Both args optional; missing ones are left unchanged. Range-
    // checked against SPILL_THRESH_MIN..MAX / MOTION_THRESH_MIN..MAX.
    // Persists only if at least one value changed. Safe during
    // WATERING: thresholds only gate cycle starts and the between-
    // bucket watchdog, never the in-flight dispense.
    webServer.on("/api/imu/threshold", HTTP_POST, []() {
        if (!checkAuth()) return;
        bool changed = false;
        if (webServer.hasArg("spill")) {
            float v = webServer.arg("spill").toFloat();
            if (v < SPILL_THRESH_MIN || v > SPILL_THRESH_MAX) {
                sendJson(200, "{\"ok\":false,\"reason\":\"spill out of range\"}");
                return;
            }
            g_spillThreshDeg = v;
            changed = true;
        }
        if (webServer.hasArg("motion")) {
            float v = webServer.arg("motion").toFloat();
            if (v < MOTION_THRESH_MIN || v > MOTION_THRESH_MAX) {
                sendJson(200, "{\"ok\":false,\"reason\":\"motion out of range\"}");
                return;
            }
            g_motionThreshG = v;
            changed = true;
        }
        if (changed) {
            persistSaveLevelConfig();
            LOG("IMU thresholds set via web: spill=%.1fdeg motion=%.3fg\n",
                (double)g_spillThreshDeg, (double)g_motionThreshG);
        }
        String r = "{\"ok\":true,\"spillThreshDeg\":";
        r += String(g_spillThreshDeg, 1);
        r += ",\"motionThreshG\":";
        r += String(g_motionThreshG, 3);
        r += "}";
        sendJson(200, r);
    });

    // POST /api/imu/mount?orientation=<enum>
    // Accepts Z_UP, Z_DOWN, Y_UP, Y_DOWN, X_UP, X_DOWN. Clears the
    // gravity-EMA seed so the next tick re-bootstraps from a fresh
    // sample under the new axis convention instead of leaking the
    // old-axis filter state. Safe during WATERING since the new
    // value won't be consulted until the next cycle's pre-flight.
    webServer.on("/api/imu/mount", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (!webServer.hasArg("orientation")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing orientation\"}");
            return;
        }
        MountOrientation m;
        if (!parseMountName(webServer.arg("orientation"), &m)) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bad orientation\"}");
            return;
        }
        g_mountOrientation = m;
        g_imuFiltSeeded    = false;
        persistSaveMountOrientation();
        LOG("Mount orientation set via web: %s\n",
            mountName(g_mountOrientation));
        String r = "{\"ok\":true,\"mountOrientation\":\"";
        r += mountName(g_mountOrientation);
        r += "\"}";
        sendJson(200, r);
    });

    // POST /api/setmotorinv  or  POST /api/setmotorinv?enable=0|1
    // No body: toggle. With ?enable=N: explicit. Persists. Forbidden
    // during WATERING since flipping direction mid-cycle would send
    // the rotor the wrong way.
    //
    // Flipping g_motorInverted also INVALIDATES g_endstopsCalibrated.
    // The two are coupled: the saved g_bucketAngleStepPositive sign and
    // the saved g_encoderZeroOffset were measured under the previous
    // direction sense, so leaving them in place would map every bucket
    // onto the wrong physical position. Forcing a re-calibration closes
    // that trap. No-op when the value didn't actually change.
    webServer.on("/api/setmotorinv", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        bool wantInv = !g_motorInverted;
        if (webServer.hasArg("enable")) {
            wantInv = (webServer.arg("enable") == "1"
                       || webServer.arg("enable") == "true");
        }
        if (wantInv != g_motorInverted) {
            g_motorInverted = wantInv;
            persistSaveMotorInverted();
            if (g_endstopsCalibrated) {
                LOG("%s\n", "Motor direction changed -- invalidating "
                            "endstop calibration; re-run 'C' / "
                            "POST /api/calibrate");
                g_endstopsCalibrated = false;
                persistSaveEndstopCal();
            }
        }
        LOG("Motor direction set to motorInv=%s via web (saved)\n",
            g_motorInverted ? "YES" : "NO");
        String r = "{\"ok\":true,\"motorInverted\":";
        r += (g_motorInverted ? "true" : "false");
        r += ",\"endstopsCalibrated\":";
        r += (g_endstopsCalibrated ? "true" : "false");
        r += "}";
        sendJson(200, r);
    });

    // POST /api/scheduler  or  POST /api/scheduler?enable=0|1
    // No body: toggle. With ?enable=N: explicit. Returns the new state.
    // Allowed in any state including WATERING (a user might want to disable
    // the scheduler mid-cycle to prevent the NEXT cycle from auto-firing
    // -- the current cycle is unaffected and will complete normally).
    webServer.on("/api/scheduler", HTTP_POST, []() {
        if (!checkAuth()) return;
        bool wantOn = !g_schedulerEnabled;
        if (webServer.hasArg("enable")) {
            wantOn = (webServer.arg("enable") == "1"
                      || webServer.arg("enable") == "true");
        }
        setSchedulerEnabled(wantOn, "web");
        String r = "{\"ok\":true,\"schedulerEnabled\":";
        r += (g_schedulerEnabled ? "true" : "false");
        r += "}";
        sendJson(200, r);
    });

    webServer.on("/api/debug/fastsched", HTTP_POST, []() {
        if (!checkAuth()) return;
        bool wantOn = !g_debugFastSchedule;
        if (webServer.hasArg("enable")) {
            wantOn = (webServer.arg("enable") == "1"
                      || webServer.arg("enable") == "true");
        }
        g_debugFastSchedule = wantOn;
        // Reset last-watered timestamps so the schedule restarts
        // cleanly under the new regime instead of immediately
        // firing on every bucket.
        uint32_t nowSec = scheduleTimeNow();
        for (int i = 0; i < g_bucketCount; i++) {
            g_lastWateredAt[i] = nowSec;
            persistMarkLastWatered(i);
        }
        LOG("DEBUG fast-schedule %s\n",
            g_debugFastSchedule ? "ENABLED (intervals = seconds)"
                                 : "DISABLED (normal hours)");
        String r = "{\"ok\":true,\"fastSchedule\":";
        r += (g_debugFastSchedule ? "true" : "false");
        r += "}";
        sendJson(200, r);
    });

    webServer.on("/api/resetwifi", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        LOG("%s\n", "WiFi credentials erased via web; rebooting in 2s");
        sendJson(200, "{\"ok\":true,\"msg\":\"rebooting in 2s\"}");
        // Commit pending edits before the reboot (same rationale as
        // the serial 'W' path). Done after sendJson so the HTTP
        // response is already queued; the 2s delay below still covers
        // the NVS write plus the client flush.
        persistFlushAll();
        // Delay slightly so the JSON response actually flushes to
        // the client before we yank WiFi out from under it.
        delay(2000);
        WiFi.disconnect(true, true);
        delay(200);
        ESP.restart();
    });

    // POST /api/recheckscale  -- re-detect U177 (mirrors serial 's')
    webServer.on("/api/recheckscale", HTTP_POST, []() {
        if (!checkAuth()) return;
        if (g_state == WATERING) {
            sendJson(200, "{\"ok\":false,\"reason\":\"watering\"}");
            return;
        }
        scaleOK = tryScaleInit();
        if (scaleOK) {
            uint8_t fw = 0;
            bool ok = scale.getFirmwareVersion(fw);
            LOG("U177 OK (recheck via web). gapValue=%.3f fw=%s\n",
                (double)g_gapValue,
                ok ? String((unsigned)fw).c_str() : "?");
        } else {
            LOG("%s\n", "U177 still not detected (recheck via web)");
        }
        String r = "{\"ok\":true,\"scaleOK\":";
        r += (scaleOK ? "true" : "false");
        r += "}";
        sendJson(200, r);
    });

    // Bucket selection. Mirrors the BtnA short-press semantics: just
    // moves g_selected without triggering motion. GET so the bucket
    // card click handler can be a tiny fetch() without a body.
    webServer.on("/api/select", HTTP_GET, []() {
        if (!webServer.hasArg("bucket")) {
            sendJson(200, "{\"ok\":false,\"reason\":\"missing bucket\"}");
            return;
        }
        int n = webServer.arg("bucket").toInt();
        if (n < 1 || n > g_bucketCount) {
            sendJson(200, "{\"ok\":false,\"reason\":\"bad bucket\"}");
            return;
        }
        g_selected = n - 1;
        renderUI(true);
        String r = "{\"ok\":true,\"selected\":";
        r += g_selected;
        r += "}";
        sendJson(200, r);
    });

    // Log ring-buffer pull. Returns up to 100 lines with seq > since,
    // plus nextSince and head so the client can chain the next batch.
    // Clamps `since` upward to (g_logSeqNext - LOG_BUF_LINES) so a
    // slow client that fell behind still gets the oldest available
    // lines instead of an empty array.
    webServer.on("/api/log", HTTP_GET, []() {
        uint32_t since = 0;
        if (webServer.hasArg("since")) {
            since = (uint32_t)strtoul(webServer.arg("since").c_str(), nullptr, 10);
        }
        // Oldest live seq in the ring. g_logSeqNext starts at 1 and
        // increments on every LOG; once it exceeds LOG_BUF_LINES the
        // oldest live entry is at seq = g_logSeqNext - LOG_BUF_LINES.
        uint32_t oldest = (g_logSeqNext > LOG_BUF_LINES)
                          ? (g_logSeqNext - LOG_BUF_LINES) : 1;
        if (since + 1 < oldest) since = oldest - 1;

        String body;
        body.reserve(2048);
        body += "{\"lines\":[";
        const int kMax = 100;
        int emitted = 0;
        // Walk seqs (since+1 .. g_logSeqNext-1) up to kMax. The seq
        // -> ring-index math: each slot holds the line that was
        // written when its g_logHead was (slot.seq - 1), so we just
        // search by seq value rather than back-walking from g_logHead.
        for (uint32_t s = since + 1;
             s < g_logSeqNext && emitted < kMax;
             s++) {
            // Find the slot. seq - 1 was the head value at write
            // time, so slot index = (seq - 1) % LOG_BUF_LINES.
            const LogLine& ln = g_logBuf[(s - 1) % LOG_BUF_LINES];
            if (ln.seq != s) continue;   // overwritten; skip
            if (emitted) body += ",";
            body += "{\"seq\":";
            body += (unsigned long)ln.seq;
            body += ",\"ms\":";
            body += (unsigned long)ln.ms;
            body += ",\"epoch\":";
            body += (unsigned long)ln.epochSec;
            body += ",\"text\":\"";
            // JSON-escape: " \ \n \r \t and control chars.
            for (const char* p = ln.text; *p; p++) {
                char c = *p;
                if (c == '"' || c == '\\') { body += '\\'; body += c; }
                else if (c == '\n') body += "\\n";
                else if (c == '\r') body += "\\r";
                else if (c == '\t') body += "\\t";
                else if ((unsigned char)c < 0x20) {
                    char esc[8];
                    snprintf(esc, sizeof(esc), "\\u%04x", c);
                    body += esc;
                } else {
                    body += c;
                }
            }
            body += "\"}";
            emitted++;
        }
        body += "],\"nextSince\":";
        body += (unsigned long)g_logSeqNext;
        body += ",\"head\":";
        body += (unsigned long)g_logHead;
        body += "}";
        sendJson(200, body);
    });

    // OTA upload form. Tiny HTML page with a file input and a JS
    // upload handler showing live progress; posts multipart/form-data
    // to POST /update which streams straight into the ESP32 Update
    // library. No third-party dep, no advertising.
    webServer.on("/update", HTTP_GET, []() {
        markInteraction();
        webServer.send_P(200, "text/html", otaUploadHtml);
    });

    // OTA upload sink. WebServer::on() with two handlers: the second
    // (upload) handler is invoked multiple times during the multipart
    // body (START, WRITE, END); the first (request) handler runs
    // after the body is fully consumed and sends the final response.
    webServer.on("/update", HTTP_POST,
        []() {
            // Rejected before any flash write (see the upload handler's
            // START step): report why and do NOT reboot. Clear the flag so
            // the next upload starts clean. Auth is checked here via the
            // reject flag rather than checkAuth() because the upload body
            // has already streamed by the time this request handler runs --
            // gating at START is what actually prevents an unauthorized
            // image from being written to the inactive partition.
            if (g_otaRejectAuth) {
                g_otaRejectAuth = false;
                webServer.sendHeader("Connection", "close");
                webServer.send(401, "application/json",
                               "{\"ok\":false,\"reason\":\"unauthorized\"}");
                return;
            }
            if (g_otaRejectBusy) {
                g_otaRejectBusy = false;
                webServer.sendHeader("Connection", "close");
                webServer.send(409, "text/plain",
                               "busy: motion in progress");
                return;
            }
            // Final response. Send before reboot so the browser's XHR
            // gets the 200/500 instead of a connection-reset.
            webServer.sendHeader("Connection", "close");
            bool ok = !Update.hasError();
            webServer.send(ok ? 200 : 500, "text/plain",
                           ok ? "OK -- rebooting" : "FAIL");
            delay(200);
            ESP.restart();
        },
        []() {
            HTTPUpload& upload = webServer.upload();
            if (upload.status == UPLOAD_FILE_START) {
                // Gate the flash before it begins. Refuse if motion is in
                // progress (the cycle owns the motor/valve) or, on a token
                // build, if the X-Api-Token is missing/wrong. Set a reject
                // flag the request handler turns into a 409/401 and skip
                // Update.begin() entirely, so nothing is ever written to
                // the inactive OTA partition for a rejected push.
                if (g_state == WATERING || g_abortableMotion) {
                    g_otaRejectBusy = true;
                    Serial.printf("OTA refused (busy): %s\n",
                                  upload.filename.c_str());
                    return;
                }
                if (!apiTokenOk()) {
                    g_otaRejectAuth = true;
                    Serial.printf("OTA refused (unauthorized): %s\n",
                                  upload.filename.c_str());
                    return;
                }
                g_otaInProgress = true;
                // Commit any debounced writes before the flash begins.
                // A failed flash reboots the device; pending dose /
                // interval / tare edits from the last few seconds must
                // survive that, not vanish with the RAM-only dirty bits.
                persistFlushAll();
                // Belt-and-suspenders: shut motor + valve so an
                // in-flight flash can't leave them powered.
                stepperReleaseCoils();
                valveClose();
                Serial.printf("OTA start: %s\n",
                              upload.filename.c_str());
                // UPDATE_SIZE_UNKNOWN: the partition manager picks
                // the right slot; size is validated at end().
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_WRITE) {
                if (g_otaRejectBusy || g_otaRejectAuth) return;
                if (Update.write(upload.buf, upload.currentSize)
                    != upload.currentSize) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_END) {
                if (g_otaRejectBusy || g_otaRejectAuth) return;
                if (Update.end(true)) {
                    Serial.printf("OTA done: %u bytes flashed\n",
                                  upload.totalSize);
                } else {
                    Update.printError(Serial);
                    // Failure: clear the in-progress flag so the
                    // device can keep running. Success leaves the
                    // flag set since we're about to reboot anyway.
                    g_otaInProgress = false;
                }
            } else if (upload.status == UPLOAD_FILE_ABORTED) {
                Update.end();
                g_otaInProgress = false;
                Serial.println("OTA aborted");
            }
        });

    webServer.onNotFound([]() {
        webServer.send(404, "text/plain", "not found");
    });

#ifdef API_TOKEN
    // ESP32 WebServer only retains request headers explicitly registered
    // here; without this, checkAuth()/apiTokenOk() would never see the
    // custom X-Api-Token header. Only needed on token builds.
    static const char* kAuthHeaders[] = { "X-Api-Token" };
    webServer.collectHeaders(kAuthHeaders, 1);
#endif
    webServer.begin();
    LOGI("HTTP server listening at http://%s/  (OTA at /update)\n",
                  WiFi.localIP().toString().c_str());
}

// ---------- setup / loop ----------

// -------- OTA app rollback (anti-brick auto-revert) --------
// Background: the precompiled arduino-esp32 bootloader has
// CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y (see the platformio.ini note),
// so an OTA-pushed image boots in ESP_OTA_IMG_PENDING_VERIFY and the
// bootloader reverts to the previous app slot on the next reset until the
// running firmware cancels the rollback. A USB/esptool-flashed slot has
// no PENDING_VERIFY otadata, so none of this affects cable flashing.
//
// The catch: the Arduino core's initArduino() (which runs BEFORE setup())
// would auto-mark the image valid immediately when CONFIG_APP_ROLLBACK_ENABLE
// is set, which defeats the safety net. The core gates that auto-mark on a
// weak verifyRollbackLater() hook -- overriding it to return true makes the
// core DEFER, handing the mark-valid decision to our health gate below.
// extern "C": the core declares this with C linkage (esp32-hal-misc.c), so
// the override must match the unmangled symbol to bind over the weak default.
extern "C" bool verifyRollbackLater() { return true; }

// Human-readable name for an esp_ota_img_states_t (boot banner + /api/state).
// "pending-verify" == on probation: the bootloader will revert on the next
// reset until markOtaValidIfHealthy() cancels the rollback.
static const char* otaStateName(esp_ota_img_states_t s) {
    switch (s) {
        case ESP_OTA_IMG_NEW:            return "new";
        case ESP_OTA_IMG_PENDING_VERIFY: return "pending-verify";
        case ESP_OTA_IMG_VALID:          return "valid";
        case ESP_OTA_IMG_INVALID:        return "invalid";
        case ESP_OTA_IMG_ABORTED:        return "aborted";
        case ESP_OTA_IMG_UNDEFINED:      return "undefined";
        default:                         return "unknown";
    }
}

// Label of the app slot we are running from (e.g. "app0"/"app1"), or "?".
static const char* otaRunningLabel() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    return p ? p->label : "?";
}

// ota_state of the running slot, defaulting to "undefined" when the query
// fails (e.g. a freshly USB-flashed slot with no otadata entry).
static const char* otaRunningStateName() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (p) esp_ota_get_state_partition(p, &st);
    return otaStateName(st);
}

// Cancel a pending rollback once the firmware has proven healthy, so the
// bootloader stops reverting on reset. Called from loop() AND from
// serviceAbort(), so it still runs during a blocking watering/demo cycle
// -- those never return to loop()'s top, so the loop() call alone would
// miss any cycle that spans the OTA verify window. A no-op after the first
// successful mark, and a no-op entirely on images that aren't
// PENDING_VERIFY (USB flashes, already-validated images).
//
// Health definition: the image has run for >= 60 s. That single check is
// sufficient because this function is only ever reached from loop() or
// serviceAbort(), both of which run only AFTER setup() returns -- so being
// called at all already proves setup() finished. The one non-certifying
// condition, an image that wedges before setup() completes, is covered:
// loop()/serviceAbort() never run, the mark never happens, and the
// bootloader reverts on the next reset.
//
// g_state is deliberately NOT consulted. WATERING is "busy," not
// "unhealthy" -- a device mid-cycle is positive proof the firmware works,
// so the old g_state==READY||ERROR requirement only made OTA validation
// time out whenever a watering cycle spanned the verify window (the fleet
// then reported reverted/unconfirmed). ERROR is a HARDWARE fault, with the
// firmware itself fine, so it certifies too.
static void markOtaValidIfHealthy() {
    static bool s_done = false;
    if (s_done) return;

    // 60 s uptime is the whole gate: it implies setup() finished (see
    // header) and gives the image time to prove it boots cleanly.
    if (millis() < 60000UL) return;

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (running && esp_ota_get_state_partition(running, &st) == ESP_OK
        && st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            LOGI("OTA: image on %s marked VALID (rollback cancelled)\n",
                 running->label);
        } else {
            LOGI("%s\n", "OTA: mark-valid FAILED (will roll back on reset)");
        }
    }
    // Latch even when not PENDING_VERIFY: once healthy there is nothing
    // more to do, so stop querying every loop pass.
    s_done = true;
}

void setup() {
    auto cfg = M5.config();
    AtomS3.begin(cfg);
    AtomS3.Display.setRotation(0);   // 0° — LCD mounted upside-down (flipped 180° from prior orientation)
    AtomS3.Display.setTextSize(1);
    Serial.begin(115200);
    delay(200);

    // Earliest possible "I am alive" line on USB. Predates the ring
    // buffer / web log, so use Serial directly. Build timestamp lets
    // the user confirm which firmware is running.
    Serial.println();
    Serial.println("=== Pluvio Grow Drip booting (" FW_VERSION ", build " FW_BUILD ") ===");
    Serial.flush();

    // Derive the per-device identity before any WiFi/AP work so the
    // hostname/mDNS name/AP SSID are all set when used below, and the
    // identity line lands in the log right after the boot banner.
    initDeviceIdentity();

    g_state = BOOTING;
    say(String("Booting ") + FW_VERSION);

    // OTA rollback visibility: report which app slot is running and its
    // verify state. A line reading "pending-verify" means this is a fresh
    // OTA image on probation -- if it never reaches markOtaValidIfHealthy()
    // the bootloader auto-reverts on the next reset, and that revert is
    // visible here (the reverted slot boots as "valid") in serial and the
    // web log ring buffer.
    LOGI("OTA: running partition %s, state=%s\n",
         otaRunningLabel(), otaRunningStateName());

    // LED disabled: GPIO8 reused for Port B I2C -- restore on next hardware rev.
    // Status LED up front so it can show the BOOTING indicator while
    // homing / first-boot mapping run. loop() (and thus updateLed())
    // doesn't tick until setup() returns, so paint one frame here;
    // g_ledBrightness is still its compile default until
    // persistApplyGlobals() runs below, which is fine for boot.
    // FastLED.addLeds<WS2812, STATUS_LED_PIN, GRB>(g_leds, 1);
    updateLed();   // no-op stub (LED disabled)

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    // Two software-bit-banged buses, one device each (see swi2c* above):
    // the AS5600 encoder on Port B and the U177 scale on Port C. The
    // passive U006 hub on Port A is removed. M5.In_I2C stays owned by
    // M5Unified on I2C_NUM_1 @ GPIO 45/0 for the IMU; we don't touch it.
    // Port B pins GPIO7/8 may still be bound to a prior peripheral (GPIO 8
    // was the old WS2812/FastLED pin and stays attached to RMT), which blocks
    // the bit-bang driver from driving them. Release both back to plain GPIO
    // before configuring them as the encoder bus SDA/SCL.
    gpio_reset_pin(GPIO_NUM_7);
    gpio_reset_pin(GPIO_NUM_8);
    swi2cInit(g_encBus);
    swi2cInit(g_scaleBus);

    // AP-mode entry runs BEFORE motion init so the user can force
    // config mode even on a rig where motion hardware later fails;
    // checkApModeRequest() owns the display during its 5s window.
    if (checkApModeRequest()) {
        runConfigPortalBlocking();   // never returns
    }

    if (!motion.begin(&Wire, M5_ATOMIC_MOTION_I2C_ADDR,
                       I2C_SDA_PIN, I2C_SCL_PIN, 400000)) {
        say("MOTION FAIL");
        LOGI("%s\n", "AtomicMotion not found at 0x38");
        g_state = ERROR;
        return;
    }
    LOGI("Motion OK. Vbus=%.2fV\n", busVoltage());

    // Coils off and valve closed at the earliest opportunity so a
    // reboot mid-cycle can't leave power on the H-bridges or hold
    // the solenoid open.
    stepperReleaseCoils();

    persistInit();
    // The U177 stores its own gap value internally, so tryScaleInit()
    // can come up without the saved value pre-pushed. After
    // persistApplyGlobals() (below) reads the NVS copy, it pushes any
    // non-zero saved value back to the unit -- belt-and-suspenders
    // for a unit swap or a wiped on-device NVS.
    scaleOK = tryScaleInit();
    if (scaleOK) {
        uint8_t fw = 0;
        bool ok = scale.getFirmwareVersion(fw);
        LOGI("U177 OK. gapValue=%.3f fw=%s\n",
             (double)g_gapValue,
             ok ? String((unsigned)fw).c_str() : "?");
    } else {
        LOGI("%s\n", "U177 not detected; running without scale.");
    }
    persistApplyGlobals();

    // IMU init runs AFTER persistApplyGlobals so the restored mount
    // orientation and thresholds are in effect when the first sample
    // lands. Failure is non-fatal -- imuInit() logs and clears
    // g_imuPresent; spillOk()/motionOk() then short-circuit to true.
    imuInit();

    // Bring up WiFi + HTTP server BEFORE homing. Motion init may fail
    // (no buckets, home fault, etc.) -- we want OTA reachable in that
    // case so a bad firmware can be replaced over the air without
    // hauling the cable back out.
    // Sealed-unit auto-provision (Option A). If NO STA credentials are
    // stored, there is no network to join and -- on a sealed, button-less
    // unit with no power-cycle -- no way for a user to force the portal.
    // So enter it automatically. We gate on credential *presence*, not on
    // a failed connect: a routine router SSID/password change leaves the
    // stored creds intact, so such a unit stays on the connectWifiSTA()
    // "keep running, retry every 30s" path below and keeps watering -- it
    // never lands here. This no-creds path is only reached by a factory-
    // fresh unit, an NVS reset/corruption, or a deliberate erase (serial
    // 'W' / web /api/resetwifi) -- all cases where there is nothing to
    // water-schedule against yet or an operator is actively re-provisioning.
    // WiFi.mode(STA) first so the WiFi driver is up for the persistent-
    // config read inside getWiFiIsSaved(); connectWifiSTA() re-asserts the
    // mode harmlessly. runConfigPortalBlocking() never returns (it
    // self-reboots into STA the moment creds are saved).
    WiFi.mode(WIFI_STA);
    {
        WiFiManager wmProbe;
        if (!wmProbe.getWiFiIsSaved()) {
            LOGI("%s\n", "No WiFi credentials stored; auto-entering config portal");
            runConfigPortalBlocking();   // never returns
        }
    }

    g_wifiUp = connectWifiSTA();
    if (g_wifiUp) {
        initWebServer();
    } else {
        Serial.println("Web UI / OTA disabled (no WiFi).");
    }

    // Defense in depth: valve closed before any motion.
    valveClose();

    persistApplyDoses();

    say("Encoder...");
    bool encoderOk = encoderInit();
    if (!encoderOk) {
        LOGI("%s\n", "Encoder init failed (AS5600 unreachable or "
                     "magnet absent); will latch FAULT_ENCODER_LOST "
                     "after READY");
    }

    // First-boot endstop calibration: run only if the encoder is
    // healthy AND no saved calibration. Skipped silently when the
    // encoder is unhealthy -- the latched FAULT_ENCODER_LOST below is
    // the loud signal in that case.
    if (encoderOk && !g_endstopsCalibrated) {
        say("Calibrating...");
        LOGI("%s\n", "No saved endstop calibration; running initial drive");
        (void)calibrateEndstops();
        // Failure here is non-fatal -- the rig still comes up READY so
        // the user can fix mechanical issues and retry from the web UI.
    }

    g_state         = READY;
    g_selected      = 0;
    g_lastWaterMs   = 0;
    g_nextWaterMs   = millis()
                    + (uint32_t)WATER_INTERVAL_HOURS * 3600000UL;

    // Time-source cascade. Layer 0 (uptime) always works; Layer 1
    // (NTP) gives reboot-survivable scheduling.
    if (g_wifiUp) {
        if (tryAcquireNtpTime()) {
            // NTP succeeded: restore persisted epoch values.
            // Buckets without a saved value stay at 0 (epoch 1970)
            // which guarantees they're due immediately on first run.
            persistApplyLastWatered();
            LOG("%s\n", "Schedule mode: WALL-CLOCK (NTP)");
        } else {
            // NTP failed; uptime mode. Treat all buckets as
            // "just watered" so we don't cycle-storm at boot.
            uint32_t nowSec = scheduleTimeNow();
            for (int i = 0; i < g_bucketCount; i++) {
                g_lastWateredAt[i] = nowSec;
            }
            LOG("%s\n", "Schedule mode: UPTIME (no NTP)");
        }
    } else {
        uint32_t nowSec = scheduleTimeNow();
        for (int i = 0; i < g_bucketCount; i++) {
            g_lastWateredAt[i] = nowSec;
        }
        LOG("%s\n", "Schedule mode: UPTIME (no WiFi)");
    }

    printMenu();
    // Start the idle timer from boot-complete. Homing + first-boot
    // calibration can run for a minute or more; without this the very
    // first manageDisplayPower() tick would see a stale timer and dim
    // (or sleep) the panel immediately.
    g_lastInteractionMs = millis();
    g_displayState      = DISPLAY_ACTIVE;
    renderUI(true);

    // Loud-fault policy: if the encoder is unhealthy at boot (AS5600
    // unreachable, magnet missing/weak), latch FAULT_ENCODER_LOST so
    // the rig comes up in ERROR (red banner) instead of silently READY
    // with every gotoBucket failing. Done AFTER g_state=READY so
    // enterFault's READY->ERROR transition fires cleanly; done AFTER
    // renderUI so the initial dial paint goes through, then enterFault
    // immediately redraws over it with the fault banner.
    if (!encoderOk || !g_encoderHealthy) {
        enterFault(FAULT_ENCODER_LOST, -1);
    }
}

// Returns true if any bucket is due (its interval has elapsed
// since last watered). Used by loop() to decide when to launch
// a cycle. Buckets with dose == DOSE_SKIP are excluded from
// due-checking; their interval_hours setting is moot until the
// user re-enables them. Operates in scheduleTimeNow()'s domain
// (epoch seconds when NTP, seconds-since-boot otherwise).
static bool anyBucketDue() {
    uint32_t now = scheduleTimeNow();
    for (int i = 0; i < g_bucketCount; i++) {
        if (g_buckets[i].dose_ml == DOSE_SKIP) continue;
        uint32_t intervalSec = intervalToSec(g_buckets[i].interval_hours);
        if ((now - g_lastWateredAt[i]) >= intervalSec) return true;
    }
    return false;
}

// Returns seconds until the earliest due time across all non-skipped
// buckets, or 0 if any is already due, or 0xFFFFFFFFUL if no
// bucket is currently scheduled (all skipped).
static uint32_t secUntilNextDue() {
    uint32_t now = scheduleTimeNow();
    uint32_t earliest = 0xFFFFFFFFUL;
    for (int i = 0; i < g_bucketCount; i++) {
        if (g_buckets[i].dose_ml == DOSE_SKIP) continue;
        uint32_t intervalSec = intervalToSec(g_buckets[i].interval_hours);
        uint32_t elapsed = now - g_lastWateredAt[i];
        if (elapsed >= intervalSec) return 0;
        uint32_t remaining = intervalSec - elapsed;
        if (remaining < earliest) earliest = remaining;
    }
    return earliest;
}

void loop() {
    AtomS3.update();
    handleSerial();
    // ~50 Hz IMU sample (cheap when absent; gated on millis() inside).
    // Run in every state so the live UI level indicator updates while
    // ERROR or BOOTING, and so the motion ring is warm by the time
    // the next cycle's pre-flight check runs.
    imuTick();
    // ~50 Hz AS5600 angle refresh + 1 Hz status reread. Same millis()
    // gate pattern as imuTick. Runs in every state so /api/state and
    // the renderers always see fresh g_armPositionDeg / g_encoderHealthy.
    encoderTick();
    // Periodic scale liveness check + automatic re-detect (millis-gated
    // inside; skips during WATERING). Recovers a wedged/unplugged U177
    // without a manual Re-detect.
    scaleHealthTick();
    // ~20 Hz status LED refresh (millis()-gated inside). Reads the
    // global state via resolveLedState(); the rest of the firmware is
    // unaware the LED exists. Runs in every state, before the ERROR
    // early-return below, so the fault color animates too.
    updateLed();

    // HTTP + OTA: serviced every tick, including during WATERING, so
    // the live watering view in the web UI can poll /api/state and the
    // operator can hit /api/abort. The dispense loop's delay(20)
    // between scale reads gives natural windows for handleClient() to
    // fire; inter-bucket motion (gotoBucket, coil release) still runs
    // uninterrupted because handleClient() only services pending
    // requests when called -- it doesn't preempt anything. If the
    // config portal is active we never reach here -- WiFiManager runs
    // its own DNS+webserver loop blocking inside runConfigPortalBlocking().
    if (g_wifiUp) {
        webServer.handleClient();
    }

    // WiFi health check. A flapping router must not push the rig into
    // AP mode permanently -- only the boot-time button hold does that.
    // We just attempt a reconnect and let WiFi.status() confirm on the
    // next interval.
    if (!g_apMode
        && (millis() - g_lastWifiCheck) >= WIFI_CHECK_INTERVAL_MS) {
        g_lastWifiCheck = millis();
        bool nowConnected = (WiFi.status() == WL_CONNECTED);
        if (g_wifiUp && !nowConnected) {
            Serial.println("WiFi: link lost; attempting reconnect");
            g_wifiUp = false;
            WiFi.reconnect();
        } else if (!g_wifiUp && nowConnected) {
            Serial.printf("WiFi: link restored. IP=%s\n",
                          WiFi.localIP().toString().c_str());
            g_wifiUp = true;
            // Re-register mDNS: connectWifiSTA() only runs MDNS.begin() at
            // boot, so a late STA reconnect through this health block would
            // otherwise leave g_deviceName.local unresolvable until reboot.
            // ESPmDNS.begin() calls mdns_init(), which errors if mDNS is
            // already running, so tear down first with MDNS.end()
            // (mdns_free()) before re-registering.
            MDNS.end();
            if (MDNS.begin(activeHostname().c_str())) {
                registerMdnsServices();
                LOGI("mDNS responder restarted: http://%s.local/\n",
                              activeHostname().c_str());
            } else {
                LOGI("%s\n", "mDNS responder failed to restart");
            }
        }
    }

    // Idle dim / sleep for the panel. Runs in every state (before the
    // ERROR early-return below) so a posted fault keeps the banner lit.
    // Blocking operations (watering, demo, diagnostics) don't tick this
    // mid-run, but AbortableMotionGuard wakes the panel on entry and
    // refreshes the timer on exit, so they stay lit throughout.
    manageDisplayPower();

    // OTA anti-brick: cancel the bootloader rollback once this image has
    // proven healthy. Placed BEFORE the ERROR early-return so an
    // ERROR-with-hardware-fault still counts as healthy firmware (see
    // markOtaValidIfHealthy) and a faulted-but-booting unit doesn't get
    // reverted on its next reset. No-op after the first mark.
    markOtaValidIfHealthy();

    if (g_state == ERROR) {
        // Still service gestures so the extra-long (>=3s) BtnA hold
        // can clear the fault. The scheduled-cycle trigger lives
        // inside the READY branch below, so a fault correctly
        // suspends scheduled cycles until acknowledged. Don't redraw
        // the UI on every tick -- enterFault / clearFault force the
        // redraw at transition time.
        updateGestures();
        // OPEN-LOOP blind motor tools (jog + rotate) are the motor ops
        // allowed despite a motion-blocking fault (the debug escape hatch):
        // a dead encoder latches FAULT_ENCODER_LOST -> canRunMotorOps() is
        // false, but these must still run so the operator can verify the
        // motor during bring-up. Dispatched OUTSIDE the canRunMotorOps() gate
        // below, on purpose; both void position trust, so they can't enable
        // blind auto-watering. Normal closed-loop ops stay behind the gate.
        if (g_webRequestJog) {
            g_webRequestJog = false;
            int jd = g_webJogDir, js = g_webJogSteps;
            g_webJogDir = 0; g_webJogSteps = 0;
            jogOpenLoop(jd, js);
            renderUI(true);
        } else if (g_webRequestRotate) {
            g_webRequestRotate = false;
            int dir = g_webRotateDir;
            g_webRotateDir = 0;
            rotateContinuous(dir);
            renderUI(true);
        }
        // Motor-only ops still run in ERROR if the fault is non-motion.
        // canRunMotorOps() returns true for water-path faults so the
        // user can debug from the web UI without clearing the fault first.
        if (canRunMotorOps()) {
            // Consume the same motor-only flags loop() consumes in READY.
            // Cycle-triggering flags (g_webRequestWater, g_webRequestSingle)
            // are NOT consumed here -- those need full READY.
            if (g_webRequestHome) {
                g_webRequestHome = false;
                AbortableMotionGuard motionGuard;
                g_abortRequested = false;
                (void)homeStepper();
                g_abortRequested = false;
            } else if (g_webRequestDtest) {
                g_webRequestDtest = false;
                runDispenseTest();
            } else if (g_webRequestCalibrate) {
                g_webRequestCalibrate = false;
                AbortableMotionGuard motionGuard;
                g_abortRequested = false;
                (void)calibrateEndstops();
                g_abortRequested = false;
                renderUI(true);
            }
        }
        delay(50);
        return;
    }

    if (g_state == READY) {
        updateGestures();
        // Auto-clear a deferral whose condition has resolved, even if no
        // cycle has completed since. Runs before the scheduler check so
        // the LED / web UI drop the stale banner within ~2s of the rig
        // going still (or being leveled / refilled), rather than waiting
        // for the next clean cycle to call clearDeferralState().
        clearStaleDeferralIfResolved();
        // Commit any debounced NVS writes whose quiet period has
        // elapsed. Cheap when nothing is dirty (one uint32 compare).
        // READY-only on purpose: a faulted rig may have hardware
        // issues and we don't want NVS chatter masking debug, and the
        // user will acknowledge the fault soon (clearFault -> READY)
        // which lets the next tick flush.
        persistFlushDue();
        // Web-requested actions run from the foreground task here so
        // they never execute in the WebServer's request thread (those
        // blocking motion calls would hold the HTTP socket open for
        // minutes). Consume one flag per pass; if both are set, water
        // wins -- the web UI disables the button for the inactive
        // state, so reaching here with both set is operator intent.
        if (g_webRequestWater) {
            g_webRequestWater = false;
            g_webRequestHome  = false;
            runWateringCycle();
        } else if (g_webRequestHome) {
            g_webRequestHome = false;
            AbortableMotionGuard motionGuard;
            g_abortRequested = false;
            (void)homeStepper();
            g_abortRequested = false;
        } else if (g_webRequestDtest) {
            g_webRequestDtest = false;
            runDispenseTest();
        } else if (g_webRequestCalibrate) {
            g_webRequestCalibrate = false;
            AbortableMotionGuard motionGuard;
            g_abortRequested = false;
            (void)calibrateEndstops();
            g_abortRequested = false;
            renderUI(true);
        } else if (g_webRequestRotate) {
            g_webRequestRotate = false;
            int dir = g_webRotateDir;
            g_webRotateDir = 0;
            rotateContinuous(dir);
        } else if (g_webRequestJog) {
            g_webRequestJog = false;
            int jd = g_webJogDir, js = g_webJogSteps;
            g_webJogDir = 0; g_webJogSteps = 0;
            jogOpenLoop(jd, js);
            renderUI(true);
        } else if (g_webRequestDemoStart) {
            // Demo mode. g_demoMode (Dry/Wet) was set by the trigger.
            // runDemoLoop blocks here (foreground) until abort/fault/
            // stop, then restores READY + scheduler on its own.
            g_webRequestDemoStart = false;
            runDemoLoop();
        } else if (g_webRequestSingle) {
            // Layer 2 hook: water one bucket only. Bucket index already
            // validated by the /api/water/bucket handler. The schedule
            // check inside runWateringCycle gates everything else off.
            runWateringCycle();
        } else if (g_schedulerEnabled && g_lastDeferReason == DEFER_NONE
                   && anyBucketDue()) {
            // Only the auto-scheduler is gated on the deferral being
            // clear -- an active deferral means the last attempt's
            // condition (empty tank, tilt, motion) is still true, so
            // re-running would just grind into the same defer-exit.
            // clearStaleDeferralIfResolved() above clears the deferral
            // within ~2s of the condition resolving, so refill -> clear
            // -> next tick waters. Manual paths (g_webRequestWater,
            // g_webRequestSingle, serial 'w', dispense tests) are NOT
            // gated: the operator explicitly asked, and the in-cycle
            // guards re-defer if the condition is still real.
            runWateringCycle();
        }

        // Drive any pending headless interaction advance. Same
        // foreground-dispatch pattern as the g_webRequest* block above:
        // the blocking gotoBucket runs here, not in the gesture handler.
        // A press during the hop queues (bumps g_interactionTargetBucket
        // via serviceAbort) rather than aborting, so on completion the
        // target may have advanced -- the next loop pass chains another
        // gotoBucket. interactionTick() below detects final arrival,
        // parks, and fires the dose pulse.
        if (g_interactionAdvancing && !g_abortableMotion) {
            int target = g_interactionTargetBucket;
            AbortableMotionGuard motionGuard;
            g_abortRequested = false;
            bool ok = gotoBucket(target);
            g_abortRequested = false;
            if (!ok) {
                // Unreachable (skip / MISSING / retries exhausted) or an
                // explicit serial/web abort. Release any held coils and
                // drop the session; the LED returns to idle.
                stepperReleaseCoils();
                interactionEnd();
            }
        }
        interactionTick();
        renderUI(false);
    }

    delay(10);
}
