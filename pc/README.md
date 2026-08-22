# PC layer

Two files:

- **`haptic_algorithm.py`** — the actual tactile-processing and
  haptic-encoding algorithm (baseline subtraction, filtering, feature
  extraction, state machine, intensity normalization, haptic encoder).
  Also contains calibration logic (`HapticConfig`, `Calibrator`,
  `run_guided_calibration`) and CSV logging schema/helpers. Fully
  documented in `../docs/haptic-algorithm.md`.
- **`haptic_engine.py`** — the runner: serial I/O to both ESP32 boards
  (background reader threads, non-blocking), the terminal dashboard, CLI
  mode selection, and the main loop. Imports and drives
  `haptic_algorithm.py`; contains no processing math of its own.

Note: there is no separate `calibration.py` — calibration lives inside
`haptic_algorithm.py` (`Calibrator` class for the automatic per-run
baseline, `run_guided_calibration()` for the interactive
contact/max-level wizard), invoked from `haptic_engine.py` via
`--calibrate`. Splitting it into its own file would just re-export the
same functions with no behavior change.

## Install

```
pip install -r requirements.txt
```

## Run

```
# Full pipeline (both ESP32 boards connected)
python haptic_engine.py --sensor-port COM7 --haptic-port COM8

# First-time / re-calibration on this eFlesh patch
python haptic_engine.py --sensor-port COM7 --haptic-port COM8 --calibrate

# Haptic ESP32 + motors only, no eFlesh/sensor needed
python haptic_engine.py --simulate --haptic-port COM8

# Sensor + dashboard only, motors never commanded
python haptic_engine.py --sensor-only --sensor-port COM7
```

See `../docs/serial-protocol.md` for the wire format, `../docs/calibration.md`
for tuning, and `../docs/demo.md` for the demo script.

## Logging

CSV logs (schema in `haptic_algorithm.LOG_COLUMNS`) are written to
`demo_logs/<timestamp>.csv` by default; pass `--no-log` to disable.
`demo_logs/` is git-ignored — it's per-run output, not source.
