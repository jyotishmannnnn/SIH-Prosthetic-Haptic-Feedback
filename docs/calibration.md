# Calibration

Every threshold in this system is empirical and device-specific — tied
to your particular eFlesh patch, its mounting, and the MLX90393's exact
position relative to the magnet. Re-run calibration whenever any of
those change (remount, reprint, new patch, moved sensor).

Calibration values live in `HapticConfig` (`pc/haptic_algorithm.py`) and
persist to `calibration.json` (loaded automatically by
`pc/haptic_engine.py`, git-ignored since it's per-device output, not
source).

## 1. Baseline calibration

Every run (and on the `b` keyboard command in `haptic_engine.py`), the PC
collects `BASELINE_SAMPLES` (100) samples with the eFlesh **untouched**
and takes the **median** per axis:
```
baseline_bx = median(Bx samples)
baseline_by = median(By samples)
baseline_bz = median(Bz samples)
```
Median, not mean, so a single touch/vibration spike during the
calibration window doesn't skew the baseline. This runs automatically at
startup in both `full` and `sensor-only` modes.

## 2. Noise-floor measurement

Run `python haptic_engine.py --sensor-only --sensor-port COMx`. With the
eFlesh untouched, watch the `M` (magnitude) readout on the dashboard —
its steady-state fluctuation is your noise floor.

## 3. Deadband

Set `deadband` (per-axis, in `HapticConfig`) a bit above the observed
per-axis noise. The guided wizard (`--calibrate`, below) sets this
automatically as `noise_floor * 1.2` from the resting-sample step.

## 4. Contact threshold (`contact_level`)

Lightly touch the eFlesh and note where `M` settles. `contact_level`
should sit just above the noise floor and at/below the low end of that
light-touch range, so light contact reliably registers without false
triggers at rest.

## 5. Maximum calibrated deformation (`max_level`)

Apply the strongest deformation you consider safe for the patch and note
`M`. This becomes `max_level` — the point at which normalized intensity
`I` reaches 1.0. This is **not** a physical force limit, just the top of
your chosen intensity scale.

## 6. Haptic PWM calibration (`pwm_min`, `pwm_max`, `gamma`)

- `pwm_min`: raise until the weakest commanded level reliably spins the
  motors (coin ERM motors have a dead zone below a certain duty cycle).
- `pwm_max`: usually 255; lower it if maximum output feels
  uncomfortable/too strong on skin.
- `gamma`: run `--simulate` while touching the actual motors against
  skin. If low-intensity levels feel indistinguishable from each other,
  increase `gamma` (compresses/spreads out the low end more). If most of
  the range feels "the same until suddenly strong," decrease it.

## 7. Pulse timing calibration (`pulse_table`, `recruitment_table`)

Edit the tables directly in `HapticConfig` and re-run `--simulate` to
preview before touching hardware, then confirm on skin. If two adjacent
intensity levels feel the same, widen the gap between their `on_ms`/
`off_ms` pair, or move a recruitment threshold so an extra motor kicks in
sooner.

## Guided calibration wizard

`pc/haptic_algorithm.py` includes `run_guided_calibration()`, invoked via:
```
python haptic_engine.py --sensor-port COM7 --haptic-port COM8 --calibrate
```
Walks through: untouched -> light touch -> medium -> strong -> maximum
safe deformation, derives `baseline_b*`, `deadband`, `contact_level`, and
`max_level` from the measured samples, and writes them to
`calibration.json` (`--calibration-file` to change the path). It does
**not** currently derive `gamma`, `pulse_table`, or `recruitment_table`
automatically — those are tuned by feel per step 6/7 above.

## What calibration does NOT do

It does not convert anything to Newtons. There is no force sensor in
this pipeline (that's the planned Sentrix/FSR glove integration — see
`docs/haptic-algorithm.md`, "V1 vs future"). `contact_level`/`max_level`
define an arbitrary-but-consistent intensity scale for this specific
patch, nothing more.
