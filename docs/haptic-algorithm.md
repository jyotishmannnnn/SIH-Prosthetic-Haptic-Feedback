# Haptic encoding algorithm (V1)

Implemented in `pc/haptic_algorithm.py`. This document is the math
reference for that module — if the two disagree, the code is correct and
this file needs updating.

**Standing disclaimer, applies to every equation below:** all magnetic
delta/magnitude values are RELATIVE, uncalibrated signals derived from
raw MLX90393 field-delta units. None of them are Newtons, none of them
are calibrated physical force, and none of them are the original eFlesh
research project's published results (that project's ~0.5mm localization
RMSE, ~0.27N/0.12N force error, ~95% slip detection accuracy belong to a
different, much more instrumented system and are not reproduced here).

## Pipeline

```
Bx, By, Bz (raw, ~100 Hz)
     |  baseline subtraction
     v
dx, dy, dz
     |  per-axis deadband -> EMA filter
     v
filtered dx, dy, dz
     |  feature extraction
     v
M, N, S, dM, HF_energy
     |  state machine (hysteresis)
     v
NO_CONTACT / CONTACT / SLIDING_OR_SHEAR (diagnostic) / RELEASE
     |  normalize
     v
I in [0, 1]
     |  haptic encoder
     v
PWM(I), pulse timing(I), motor_count(I) -> 6 motor values
```

## Equations

```
dx = Bx - baseline_bx
dy = By - baseline_by
dz = Bz - baseline_bz
```
`baseline_b*` is the median of ~100 samples collected while the eFlesh
patch is untouched (see `docs/calibration.md`).

Per-axis deadband (applied before filtering, so noise never enters the
EMA state):
```
if |dx| < DEADBAND: dx = 0   (same for dy, dz)
```

Per-axis EMA filter:
```
filtered_d* = alpha * d* + (1 - alpha) * filtered_d*_prev
```
`alpha` defaults to 0.3 (`HapticConfig.alpha`). Higher = more responsive,
less smoothing; lower = smoother, more latency.

Feature extraction (Stage 2), computed from the filtered deltas:
```
M  = sqrt(dx^2 + dy^2 + dz^2)     -- total relative deformation
N  = |dz|                          -- rough normal-deformation proxy
S  = sqrt(dx^2 + dy^2)             -- rough shear/lateral-deformation proxy
dM(t) = M(t) - M(t-1)              -- rate of change
HF_energy = variance(dM over the last `hf_window` samples)   -- lightweight
                                       high-frequency activity proxy
```
`M` is the primary intensity signal. `N` and `S` are **not** calibrated
normal/shear force — they're rough proxies kept as separate features for
future use (see "V1 vs future" below). `HF_energy` exists to support
future slip detection; it is not validated as a slip detector in V1.

## State machine

States: `NO_CONTACT`, `CONTACT`, `SLIDING_OR_SHEAR`, `RELEASE`.

```
NO_CONTACT -> CONTACT   when M > CONTACT_LEVEL
CONTACT -> NO_CONTACT   when M < CONTACT_LEVEL * release_hysteresis_ratio
```
The hysteresis gap (`release_hysteresis_ratio`, default 0.6) prevents
rapid ON/OFF chatter right at the boundary. `RELEASE` is a one-tick
transient state reported exactly on the falling transition (used to
trigger the optional release pulse — see below).

`SLIDING_OR_SHEAR` is a **diagnostic label only**:
```
if (S / max(N, eps)) > shear_ratio_threshold AND HF_energy > hf_energy_threshold:
    state = SLIDING_OR_SHEAR   (instead of CONTACT)
```
This label is logged but currently does **not** change the haptic output
— it has not been experimentally validated as a reliable slip/shear
detector on the actual eFlesh patch. Treat it as instrumentation for
future analysis, not a working feature.

## Normalized tactile intensity

```
I = clamp((M - CONTACT_LEVEL) / (MAX_LEVEL - CONTACT_LEVEL), 0, 1)
```
`CONTACT_LEVEL` and `MAX_LEVEL` are **calibration parameters**, not fixed
constants — see `docs/calibration.md` for how to obtain them
experimentally on your eFlesh patch. `I = 0` means no contact / just at
the contact threshold; `I = 1` means the deformation you calibrated as
"maximum safe deformation" during setup.

## Haptic encoder

Three independent perceptual dimensions are derived from `I`:

**1. Amplitude (gamma-corrected PWM):**
```
PWM(I) = PWM_MIN + (PWM_MAX - PWM_MIN) * I^GAMMA
```
`GAMMA` (default 1.6) compensates for nonlinear perception of vibration
amplitude and the coin ERM motors' dead-zone/startup behavior. `GAMMA >
1` compresses the low end (finer distinction between light touches);
`GAMMA < 1` expands it. Tune by feel — see `docs/calibration.md`.

**2. Temporal pattern (pulse ON/OFF timing):**
`on_ms(I)`/`off_ms(I)` are linearly interpolated from a configurable
table (`HapticConfig.pulse_table`), e.g.:

| I | on_ms | off_ms | feel |
|---|---|---|---|
| 0.0 | 60 | 440 | (below contact, motors off) |
| 0.1 | 60 | 440 | slow pulse |
| 0.3 | 70 | 300 | |
| 0.5 | 80 | 200 | moderate pulse |
| 0.7 | 100 | 100 | |
| 0.9 | 120 | 40 | fast pulse |
| 1.0 | 0 | 0 | continuous |

**3. Motor recruitment (number of active motors):**
Step function over `HapticConfig.recruitment_table`:

| I >= | active motors |
|---|---|
| 0.0 | 0 |
| 0.05 | 1 |
| 0.25 | 2 |
| 0.5 | 4 |
| 0.75 | 6 |

By default the first N motor indices (in order M0..M5) are activated.
`HapticConfig.spatial_mode_enabled` (off by default) is a wired-but-
disabled hook that lets `dx`/`dy` bias which motors light up first, for
a future directional-feedback mode — see "V1 vs future".

**4. Contact-onset / release transient pulses (optional, on by default):**
On the `NO_CONTACT -> CONTACT` transition, a brief (default 40 ms)
higher-PWM pulse fires on M0+M1, overriding the normal encoding for that
short window, then normal intensity encoding resumes. Same idea, shorter
and lower-PWM, on release. This is meant to make contact
onset/release feel distinct without obscuring the continuous intensity
signal — durations are kept short for that reason.

## Assumptions and calibration-dependent parameters

Everything in `HapticConfig` is a placeholder until measured against the
actual eFlesh patch (`docs/calibration.md`):
`alpha`, `deadband`, `hf_window`, `release_hysteresis_ratio`,
`shear_ratio_threshold`, `hf_energy_threshold`, `contact_level`,
`max_level`, `pwm_min`, `pwm_max`, `gamma`, `pulse_table`,
`recruitment_table`, `onset_pulse_*`, `release_pulse_*`.

## V1 vs future

**V1 (this repo, implemented):**
```
Bx/By/Bz -> relative deformation (M/N/S/dM/HF) -> normalized intensity I
         -> rule-based haptic encoder -> 6 motors
```

**V2 (not implemented):**
```
Bx/By/Bz + Sentrix/FSR glove ground-truth force
         -> calibrated force model -> better haptic encoding
```

**Future (not implemented):**
```
Bx/By/Bz -> contact -> normal force -> shear -> slip -> texture
         -> spatial feedback -> six-motor encoding
```

**Eventually (not implemented):**
```
EEG/sEMG intent + robotic hand + e-Flesh + closed-loop vibrotactile feedback
```

The module is structured so a future calibrated/ML estimator can replace
`TactilePipeline.process()` alone — its output contract (`TactileSample`:
dx, dy, dz, M, N, S, dM, hf_energy, state, intensity, contact_onset,
contact_release) is what `HapticEncoder` consumes, and nothing about
serial I/O, firmware, or the CSV log schema needs to change when that
swap happens.
