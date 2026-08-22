# Architecture

This document describes the **current, implemented** prototype architecture.
It intentionally uses two separate ESP32-S3 boards with a PC in between —
this is not a placeholder, it's the current design.

## Hardware blocks

```
3D-printed robotic hand
        |
    e-Flesh patch  (1x, mounted on the hand)
        |
  1 x MLX90393  (single magnetometer, I2C)
        |
  Sensor ESP32-S3  (Seeed XIAO ESP32-S3, board #1)
        |  USB Serial ("S,<ts>,<Bx>,<By>,<Bz>", ~100 Hz)
        v
       PC
        |  tactile processing + haptic encoding (pc/haptic_algorithm.py)
        |  USB Serial ("M,<m0>..<m5>" / "S")
        v
  Haptic ESP32-S3  (Seeed XIAO ESP32-S3, board #2)
        |
   6x motor PWM outputs -> driver stage -> 6x coin ERM vibration motors
        |
       user
```

## Responsibilities

### Sensor ESP32-S3 (`firmware/sensor/eFlesh_sensor_v1/`)
Does exactly three things:
1. Initializes and reads the single MLX90393 (config values taken from the
   upstream eFlesh reference sketch — see `docs/hardware.md`).
2. Timestamps each reading with `millis()`.
3. Streams `S,<ts>,<Bx>,<By>,<Bz>` over USB serial at ~100 Hz.

It does **not** run any filtering, thresholding, or motor logic, and does
not talk to the Haptic ESP32. It tracks consecutive I2C read failures and
emits an `F,<message>` line (never mixed into the `S,` stream) if the
sensor needs recovery.

### PC (`pc/haptic_engine.py` + `pc/haptic_algorithm.py`)
This is the **tactile-processing and haptic-encoding layer** for the
current prototype. `haptic_engine.py` owns serial I/O (two independent
`pyserial` connections, each with a background reader thread), the
terminal dashboard, CSV logging, and CLI modes. `haptic_algorithm.py`
owns the actual signal-processing/decision pipeline: baseline
subtraction, filtering, feature extraction (M/N/S/dM/HF energy), the
tactile state machine, intensity normalization, and the haptic encoder
(PWM curve, pulse timing, motor recruitment). See
`docs/haptic-algorithm.md` for the full math.

**Why the PC is the processing layer right now:** the tactile pipeline is
still being iterated on rapidly (thresholds, curves, recruitment tables
all need on-bench tuning against the real eFlesh patch). Doing this in
Python on the PC means every parameter is a config value that can be
changed and re-run in seconds, with a live dashboard and CSV logging for
free. None of this is fundamental — the pipeline is written as a single
`TactilePipeline`/`HapticEncoder` module specifically so it can be ported
onto an MCU later once the encoding is validated (see `docs/haptic-algorithm.md`,
"V1 vs future").

### Haptic ESP32-S3 (`firmware/haptic-controller/haptic_controller_v1/`)
Receives `M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>` / `S` / `PING` / `STATUS` over
USB serial and drives 6 PWM outputs through the bench-verified CPN2222A
driver stage (see `docs/hardware.md`). It does **not** read the MLX90393
and has no knowledge of the tactile algorithm — it just executes motor
commands.

It owns a **communication watchdog**: if no valid `M,`/`S` command
arrives within `COMM_TIMEOUT_MS` (500 ms in the current firmware), it
zeroes all motors immediately. This is the primary failsafe against a PC
crash, Python crash, or USB disconnect during a demo. **Every transport
below relies on this same watchdog** — it's implemented once, in
firmware, independent of how the command arrived.

## Transport layer (PC -> Haptic ESP32)

The PC side of this link goes through a transport abstraction
(`pc/transports/`, see `docs/transport-options.md` for the full
comparison) so `pc/haptic_algorithm.py` never needs to know how a
`MotorCommand` reaches the Haptic ESP32. Four transports are defined;
only USB is implemented in firmware today.

**USB mode (current, working):**
```
PC -> USB -> Haptic ESP32 -> Motors
```

**Wi-Fi mode (PC client implemented, NOT TESTED, no matching firmware yet):**
```
PC -> Wi-Fi (UDP) -> Haptic ESP32 -> Motors
```

**Bluetooth/BLE mode (PC client implemented, NOT TESTED, no matching firmware yet):**
```
PC -> BLE (GATT write) -> Haptic ESP32 -> Motors
```

**ESP-NOW mode (architectural stub only, not implemented):**
```
PC -> USB -> Sensor ESP32 (acting as a gateway) -> ESP-NOW -> Haptic ESP32 -> Motors
```
This is the only mode where the Sensor ESP32 does double duty — the PC
cannot speak ESP-NOW directly (see `docs/transport-options.md`). This
does **not** change what the Sensor ESP32's current firmware does; it
would require new gateway functionality that has not been built.

All four modes produce the exact same wire-level command as today
(`M,<m0>..<m5>` / `S`) — nothing about the haptic algorithm, motor
mapping, or PWM values changes based on transport.

## Communication boundaries

- Sensor ESP32 <-> PC: one-directional data stream (`S,...`), fault
  reporting only (`F,...`). No commands flow PC -> Sensor ESP32.
- PC <-> Haptic ESP32: one-directional command stream (`M,...` / `S`),
  with `PING`/`STATUS` for diagnostics, over whichever transport is
  selected (USB today; see "Transport layer" above). No sensor data
  flows through this link regardless of transport.
- The two ESP32 boards never talk to each other directly today. The only
  planned exception is the not-yet-built ESP-NOW gateway mode, where the
  Sensor ESP32 would relay PC commands to the Haptic ESP32 — see
  `docs/transport-options.md`. All coordination happens on the PC today.

See `docs/serial-protocol.md` for exact wire formats.

## Failsafe behavior (summary — full detail in serial-protocol.md)

- Sensor ESP32: consecutive I2C failure counter -> attempts sensor
  re-init, reports faults, never sends stale/fabricated readings.
- Haptic ESP32: 500 ms command watchdog -> all motors off. Also responds
  to an explicit `S` stop command immediately.
- PC: on any keyboard interrupt / shutdown, sends `S` to the Haptic ESP32
  before exiting.

## Known timing limitation (PC side, measured)

`MAIN_LOOP_SLEEP_S = 0.002` in `pc/haptic_engine.py` is commented as a
"~500Hz poll". On Windows with Python 3.10 that is not what happens:
`time.sleep()` and `time.monotonic()` both have ~15.6 ms granularity
there, so the main loop actually runs at **~65 Hz**, measured on the
development machine:

```
sleep(0.002) -> 65.2 iterations/s   actual per-sleep 15.33 ms
time.monotonic     resolution = 15.625 ms
time.perf_counter  resolution =  0.0001 ms
```

Consequences, in order of how much they matter:

1. **Pulse timing is quantized to ~15.6 ms.** The fastest row of
   `pulse_table` is 120 ms ON / 40 ms OFF, so at high intensity the OFF
   phase lands on 15.6 ms boundaries -- a meaningful error on the
   shortest phases, and a plausible cause of adjacent intensity levels
   feeling less distinct than the table implies. Worth knowing before
   spending bench time retuning `gamma` or the tables to fix something
   that is actually a clock problem.
2. **`HAPTIC_SEND_HZ = 50` is not achieved.** The send tick can only be
   checked every ~15.6 ms, so commands go out at ~32 Hz with jitter.
   Still far inside the 500 ms firmware watchdog, so this is a fidelity
   issue and not a safety one.
3. The GUI stream tops out around 21 Hz instead of 30 for the same
   reason -- the engine cannot call `publish()` any more often than its
   own loop runs. Harmless for a dashboard.

Two fixes, neither applied here:

- **Run Python 3.11 or newer.** 3.11 switched `time.sleep()` to a
  high-resolution waitable timer on Windows. No code change at all.
- Raise the system timer resolution for the process:
  `ctypes.windll.winmm.timeBeginPeriod(1)` at startup, paired with
  `timeEndPeriod(1)` on exit. Five lines, Windows-only, needs guarding.

`pc/telemetry.py` already uses `perf_counter` rather than `monotonic`
internally for exactly this reason; the engine's own timing still uses
`monotonic`.

## What this repository does NOT implement yet

- Multiple MLX90393 sensors / multi-patch eFlesh (current prototype uses
  exactly one of each).
- WiFi, Bluetooth, or cloud connectivity in the current eFlesh pipeline.
- Any embedded (on-MCU) tactile processing — it all runs on the PC today.
- EEG/sEMG intent sensing.
- Calibrated force estimation (see `docs/haptic-algorithm.md` for why raw
  magnitude is not force).

A separate, earlier prototype (`gui/legacy-alpaca-fsr-glove/`) explored a
WiFi-based FSR glove + embedded GUI for a different hand design (ALPACA).
It is kept in this repo for reference but is **not** part of the current
eFlesh/MLX90393 pipeline described above — see
`gui/legacy-alpaca-fsr-glove/README.md`.
