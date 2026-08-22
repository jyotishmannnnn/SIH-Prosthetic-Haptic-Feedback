# SIH Prosthetic Haptic Feedback

A real-time tactile-sensing-to-vibrotactile-feedback prototype for a
prosthetic/bionic hand: an eFlesh tactile patch with a single MLX90393
magnetometer feeds a two-ESP32 pipeline (sensor node -> PC processing ->
haptic node) that drives six coin vibration motors, so a prosthetic user
can feel a proxy for physical contact with the hand.

## Overview

A prosthetic user has no sensory feedback from what the hand is touching
— grip force, contact, and release all have to be inferred visually or
not at all. This project's approach: an artificial-skin tactile patch
(eFlesh) senses deformation on the hand, a PC processes that signal in
real time, and the result is encoded as vibration on the user's skin —
amplitude, pulse rhythm, and number of active motors all carry
information, not just a single on/off buzz.

```
Artificial skin (e-Flesh) -> tactile sensing -> real-time processing -> vibrotactile feedback
```

## Current prototype

This repository validates the **tactile-sensing-to-feedback pathway
only**. It does not include hand actuation/control, and it does not
include EEG or sEMG intent sensing — those are future integration steps
(see "Future Work"). What's actually implemented and working:

- One eFlesh patch + one MLX90393, mounted on a 3D-printed hand
- A sensor ESP32-S3 streaming raw magnetometer data over USB serial
- A PC-side pipeline (Python) that turns that into a normalized tactile
  intensity and a six-motor haptic pattern
- A haptic ESP32-S3 that drives six coin vibration motors from PC
  commands, with a communication watchdog failsafe

## System Architecture

```
Robotic Hand
     |
   e-Flesh
     |
  MLX90393
     |
Sensor ESP32-S3
     |
  USB Serial
     |
PC Tactile Processing
     |
 Haptic Encoder
     |
  USB Serial
     |
Haptic ESP32-S3
     |
 6 Coin Motors
     |
    User
```

Full detail, including why the PC is currently the processing layer and
the exact responsibilities of each board, in `docs/architecture.md`.

## Hardware

- Seeed Studio XIAO ESP32-S3 x2 (sensor node, haptic node)
- 1x MLX90393 3-axis magnetometer
- 1x e-Flesh tactile patch (single patch, single sensor in this prototype)
- 6x coin ERM vibration motors
- Motor driver stage (transistor/MOSFET per motor — see `docs/hardware.md`)
- 3D-printed robotic/bionic hand
- PC for real-time processing

Full pin tables and I2C/driver wiring: `docs/hardware.md`.

## Software

| Layer | Location | Role |
|---|---|---|
| Sensor firmware | `firmware/sensor/eFlesh_sensor_v1/` | Reads MLX90393, streams `S,ts,Bx,By,Bz` over serial. Nothing else. |
| Haptic firmware | `firmware/haptic-controller/haptic_controller_v1/` | Receives `M,...`/`S` commands, drives 6 PWM motor outputs, 500ms comm watchdog. Nothing else. |
| PC processing | `pc/haptic_algorithm.py` | The actual tactile-processing/haptic-encoding algorithm + calibration logic. |
| PC runner | `pc/haptic_engine.py` | Serial I/O, terminal dashboard, CLI modes, CSV logging. Drives `haptic_algorithm.py`. |
| GUI | `gui/` | **WIP** for this pipeline — see `gui/README.md`. A separate, unrelated legacy GUI (different project) lives in `gui/legacy-alpaca-fsr-glove/`. |
| Legacy firmware | `firmware/legacy/eFlesh_haptic_v1_single_board/` | Superseded single-board (no PC) version of this pipeline. |

## Signal Processing

```
Bx, By, Bz  ->  baseline subtraction  ->  per-axis deadband  ->  EMA filter
            ->  feature extraction (M, N, S, dM, HF energy)
            ->  contact/state detection  ->  normalized tactile intensity (I)
            ->  haptic encoding  ->  6 motor commands
```

`M = sqrt(dx^2+dy^2+dz^2)` is a **relative** deformation signal — not
Newtons, not a calibrated force. Full equations and the state machine:
`docs/haptic-algorithm.md`.

## Haptic Encoding

Tactile intensity `I in [0,1]` is converted into three independent
perceptual dimensions, not a single PWM value:

- **Amplitude**: gamma-corrected PWM curve
- **Temporal pattern**: pulse ON/OFF timing, from slow pulse (light) to
  continuous (maximum)
- **Motor recruitment**: number of active motors grows with intensity
  (1 motor at very light contact, all 6 at strong/maximum)

Full encoding tables and equations: `docs/haptic-algorithm.md`.

## Serial Protocol

Sensor -> PC:
```
S,<timestamp>,<Bx>,<By>,<Bz>
```

PC -> Haptic ESP32:
```
M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>
```

Stop:
```
S
```

Exact formats, ranges, error handling, and the 500ms watchdog behavior:
`docs/serial-protocol.md`.

## Calibration

This system requires calibration against the actual physical eFlesh
installation — baseline, noise floor, contact threshold, and maximum
calibrated deformation are all device-specific and must be measured, not
assumed. No physical force values are assigned without a real force
reference (that's future work, see below). Procedure: `docs/calibration.md`.

## Demo

Intended SIH demo flow: press/deform the eFlesh -> sensor signal changes
-> dashboard responds -> tactile intensity increases -> motor
count/pattern changes -> user feels the vibrotactile feedback change.
Full script: `docs/demo.md`.

## Current Limitations

- Only one eFlesh patch / one MLX90393 in the current prototype (not the
  multi-sensor eFlesh architecture).
- Tactile magnitude (`M`) is relative and uncalibrated — not Newtons.
- No EEG/sEMG in the current prototype.
- Force calibration is not finalized — `contact_level`/`max_level` define
  an intensity scale, not a force scale.
- Slip/texture classification is future work; the `SLIDING_OR_SHEAR`
  state is a logged diagnostic label, not a validated detector.
- Current haptic mapping is V1 (rule-based, hand-tuned thresholds).
- No dedicated GUI for this pipeline yet (terminal dashboard only) — see
  `gui/README.md`.
- All processing currently runs on a PC, not embedded on either ESP32.

## Future Work

- Sentrix/FSR glove integration for ground-truth force
- Synchronized force dataset (magnetometer + glove)
- Data-driven/calibrated force estimation
- Slip detection (validated, not just logged)
- Richer spatial/directional feedback (hook already exists in
  `HapticEncoder`, disabled by default)
- Multi-sensor eFlesh (multiple MLX90393 units)
- EEG/sEMG intent interface (efferent/control half of the loop)
- Fully embedded processing (move the PC pipeline onto an MCU)

None of the above is implemented in this repository yet.

## Repository layout

```
SIH-Prosthetic-Haptic-Feedback/
├── README.md
├── firmware/
│   ├── sensor/eFlesh_sensor_v1/
│   ├── haptic-controller/haptic_controller_v1/
│   └── legacy/eFlesh_haptic_v1_single_board/     (superseded single-board version)
├── pc/
│   ├── haptic_engine.py
│   ├── haptic_algorithm.py
│   ├── requirements.txt
│   └── README.md
├── gui/
│   ├── README.md                                  (WIP status for current pipeline)
│   └── legacy-alpaca-fsr-glove/                   (separate, earlier project)
├── docs/
│   ├── architecture.md
│   ├── haptic-algorithm.md
│   ├── hardware.md
│   ├── serial-protocol.md
│   ├── calibration.md
│   └── demo.md
├── data/               (placeholder for future synchronized datasets, empty)
├── examples/
├── LICENSE-NOTE.md      (no license chosen yet — see this file)
└── .gitignore
```
