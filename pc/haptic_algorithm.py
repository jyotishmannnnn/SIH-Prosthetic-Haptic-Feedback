#!/usr/bin/env python3
"""
haptic_algorithm.py — V1 tactile-to-haptic encoding algorithm.

Sits between the two existing serial endpoints. Does NOT touch firmware
or the serial protocol:

    Sensor ESP32 --"S,ts,Bx,By,Bz"--> [THIS MODULE runs on the PC] --"M,m0..m5"--> Haptic ESP32

Everything here is deterministic, rule-based, and designed to be swapped
for a calibrated ML estimator later (V2+) without touching serial I/O,
firmware, or the CSV log schema (see "V1 VS FUTURE" at bottom).

IMPORTANT — stated once, applies everywhere in this file:
  - M, N, S below are RELATIVE magnetic/deformation signals, NOT Newtons.
  - Slip/shear detection is a diagnostic label only, NOT validated in V1.
  - No numbers in this file are the original eFlesh project's published
    research results; those are unrelated to this rule-based prototype.

============================================================
ALGORITHM — BLOCK DIAGRAM
============================================================

  Bx,By,Bz (raw)
        |
        v
  [Baseline subtraction]  dx = Bx - Bx0, etc.   <-- Calibrator
        |
        v
  [Deadband + EMA filter]  (Stage 1)
        |
        v
  [Feature extraction]  M, N, S, dM, HF_energy   (Stage 2)
        |
        v
  [Tactile state machine]  NO_CONTACT / CONTACT / SLIDING_OR_SHEAR / RELEASE  (Stage 3)
        |
        v
  [Normalized intensity]  I = clamp((M-CONTACT_LEVEL)/(MAX_LEVEL-CONTACT_LEVEL), 0, 1)  (Stage 4)
        |
        v
  [Haptic encoder]                                          (Stage 5-10)
    - gamma-corrected PWM(I)
    - motor recruitment count(I)
    - pulse ON/OFF timing(I)
    - contact-onset / release transient pulses
    - (hook only) directional bias from dx,dy,dz — OFF by default
        |
        v
  [Command builder]  "M,m0,m1,m2,m3,m4,m5"                   (Stage 11)
        |
        v
  Haptic ESP32 (unchanged firmware)

============================================================
MATH SUMMARY
============================================================

  dx = Bx - baseline_bx ; dy = By - baseline_by ; dz = Bz - baseline_bz
  filtered_d* = alpha * d* + (1-alpha) * filtered_d*_prev      (per-axis EMA, alpha configurable)
  M = sqrt(dx^2 + dy^2 + dz^2)                     -- total relative deformation
  N = |dz|                                          -- rough normal-deformation proxy
  S = sqrt(dx^2 + dy^2)                             -- rough shear-deformation proxy
  dM(t) = M(t) - M(t-1)                             -- rate of change
  HF_energy = variance(dM over last K samples)      -- lightweight high-freq activity proxy
  I = clamp((M - CONTACT_LEVEL) / (MAX_LEVEL - CONTACT_LEVEL), 0, 1)
  PWM(I) = PWM_MIN + (PWM_MAX - PWM_MIN) * I^GAMMA
  on_ms(I), off_ms(I)  -- linearly interpolated from a configurable table
  motor_count(I)       -- step function from a configurable table
"""

import bisect
import json
import math
import sys
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from enum import Enum

NUM_MOTORS = 6


# ======================================================================
# CONFIG — every tunable lives here. See "CALIBRATION METHOD" below for
# how to obtain each value experimentally.
# ======================================================================

@dataclass
class HapticConfig:
    # ---- baseline (filled in by Calibrator, or loaded from calibration.json) ----
    baseline_bx: float = 0.0
    baseline_by: float = 0.0
    baseline_bz: float = 0.0

    # ---- Stage 1: signal processing ----
    alpha: float = 0.3          # EMA smoothing factor, higher = more responsive/less smooth
    deadband: float = 3.0       # per-axis-delta noise floor (raw MLX90393 units)

    # ---- Stage 2: high-frequency feature window ----
    hf_window: int = 8          # number of dM samples used for HF_energy variance

    # ---- Stage 3: state machine hysteresis ----
    # OFF threshold expressed as a fraction of CONTACT_LEVEL so it scales
    # automatically when you recalibrate CONTACT_LEVEL.
    release_hysteresis_ratio: float = 0.6

    # diagnostic-only shear/slip labeling (Stage 3) — NOT used to change
    # haptic output in V1. Purely for logging/future validation.
    shear_ratio_threshold: float = 1.5   # S/max(N, eps) above this -> shear-dominant
    hf_energy_threshold: float = 5.0     # HF_energy above this -> "active" motion

    # ---- Stage 4: normalized intensity calibration ----
    contact_level: float = 8.0    # M value at which contact is first felt/declared (I=0 here)
    max_level: float = 80.0       # M value considered "maximum safe deformation" (I=1 here)

    # ---- Stage 5/9: perceptual PWM mapping ----
    pwm_min: int = 90             # PWM at I -> 0+ (coin ERM dead-zone floor, tune per motor batch)
    pwm_max: int = 255
    gamma: float = 1.6            # >1 compresses low end (fine distinctions at low I),
                                   # <1 expands low end. Tune experimentally (Stage 9).

    # ---- Stage 8: temporal pattern table ----
    # (I_at, on_ms, off_ms). Linearly interpolated between rows. Last row's
    # off_ms=0 means "continuous" at and above that I.
    pulse_table: list = field(default_factory=lambda: [
        (0.0, 60, 440),
        (0.1, 60, 440),
        (0.3, 70, 300),
        (0.5, 80, 200),
        (0.7, 100, 100),
        (0.9, 120, 40),
        (1.0, 0, 0),      # continuous
    ])

    # ---- Stage 6: motor recruitment table ----
    # (I_at, motor_count). Step function: count active = highest row whose
    # I_at <= current I.
    recruitment_table: list = field(default_factory=lambda: [
        (0.0, 0),
        (0.05, 1),   # very light: M0 only
        (0.25, 2),   # light: M0+M1
        (0.5, 4),    # medium: M0..M3
        (0.75, 6),   # strong: all six
    ])

    # ---- Stage 7: spatial layout (for future directional mode, OFF by default) ----
    # M0 M1
    # M2 M3
    # M4 M5
    spatial_mode_enabled: bool = False

    # ---- Stage 10: contact-onset / release transient pulses ----
    onset_pulse_enabled: bool = True
    onset_pulse_pwm: int = 180
    onset_pulse_ms: int = 40
    release_pulse_enabled: bool = True
    release_pulse_pwm: int = 140
    release_pulse_ms: int = 30

    def save(self, path):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        cfg = cls()
        for k, v in data.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        return cfg


# ======================================================================
# STAGE 3: TACTILE STATE
# ======================================================================

class TactileState(Enum):
    NO_CONTACT = "NO_CONTACT"
    CONTACT = "CONTACT"
    SLIDING_OR_SHEAR = "SLIDING_OR_SHEAR"   # diagnostic label only, see class docstring
    RELEASE = "RELEASE"                     # one-tick transient on contact loss


# ======================================================================
# UTILITIES
# ======================================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def median(values):
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return (s[mid - 1] + s[mid]) / 2.0 if n % 2 == 0 else s[mid]


def interp_table(table, x):
    """Linear interpolation over a sorted [(x_at, *values), ...] table.
    Returns a tuple of interpolated values. Clamps outside the table range."""
    xs = [row[0] for row in table]
    if x <= xs[0]:
        return table[0][1:]
    if x >= xs[-1]:
        return table[-1][1:]
    i = bisect.bisect_right(xs, x) - 1
    x0, x1 = xs[i], xs[i + 1]
    t = (x - x0) / (x1 - x0) if x1 != x0 else 0.0
    row0, row1 = table[i][1:], table[i + 1][1:]
    return tuple(v0 + (v1 - v0) * t for v0, v1 in zip(row0, row1))


def step_table(table, x):
    """Step function: value from the highest row whose x_at <= x."""
    result = table[0][1]
    for x_at, value in table:
        if x >= x_at:
            result = value
        else:
            break
    return result


# ======================================================================
# CALIBRATOR — Stage: baseline + interactive calibration (Stage 14)
# ======================================================================

class Calibrator:
    """Collects a median baseline from N untouched samples. Also drives
    the guided interactive calibration procedure that determines
    CONTACT_LEVEL / MAX_LEVEL experimentally (see module docstring)."""

    def __init__(self, config: HapticConfig, num_samples=100):
        self.config = config
        self.num_samples = num_samples
        self._buf = ([], [], [])
        self.done = False

    def begin(self):
        self._buf = ([], [], [])
        self.done = False

    def feed(self, bx, by, bz):
        """Returns True once baseline is set."""
        self._buf[0].append(bx)
        self._buf[1].append(by)
        self._buf[2].append(bz)
        if len(self._buf[0]) >= self.num_samples:
            self.config.baseline_bx = median(self._buf[0])
            self.config.baseline_by = median(self._buf[1])
            self.config.baseline_bz = median(self._buf[2])
            self.done = True
        return self.done


def run_guided_calibration(sample_source, config: HapticConfig, out_path="calibration.json"):
    """Interactive terminal-driven calibration (Stage 14).

    sample_source: a callable returning the next (bx, by, bz) tuple,
    blocking briefly if needed (e.g. wraps the sensor queue).
    """
    def collect(n, prompt):
        input(prompt + " Press Enter, then keep still while sampling...")
        samples = []
        for _ in range(n):
            samples.append(sample_source())
        return samples

    print("=== CALIBRATION: eFlesh untouched ===")
    resting = collect(100, "1) Leave eFlesh untouched.")
    config.baseline_bx = median([s[0] for s in resting])
    config.baseline_by = median([s[1] for s in resting])
    config.baseline_bz = median([s[2] for s in resting])

    def mag(s):
        dx = s[0] - config.baseline_bx
        dy = s[1] - config.baseline_by
        dz = s[2] - config.baseline_bz
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    print("=== CALIBRATION: light touch ===")
    light = collect(50, "2) Apply a LIGHT touch/press and hold.")
    light_mags = [mag(s) for s in light]

    print("=== CALIBRATION: medium press ===")
    collect(30, "3) Apply MEDIUM deformation and hold.")  # felt, not stored numerically in V1

    print("=== CALIBRATION: strong press ===")
    collect(30, "4) Apply STRONG deformation and hold.")

    print("=== CALIBRATION: maximum safe deformation ===")
    maxp = collect(50, "5) Apply MAXIMUM SAFE deformation and hold (do not damage the patch).")
    max_mags = [mag(s) for s in maxp]

    resting_mags = [mag(s) for s in resting]
    noise_floor = max(resting_mags) if resting_mags else 0.0

    # CONTACT_LEVEL: just above resting noise, anchored to the low end of
    # the observed light-touch range so light contact reliably registers.
    config.contact_level = max(noise_floor * 1.5, min(light_mags) * 0.8)
    config.deadband = noise_floor * 1.2
    config.max_level = median(max_mags)

    if config.max_level <= config.contact_level:
        print("[WARN] max_level <= contact_level; check your presses. Using fallback ratio.")
        config.max_level = config.contact_level * 5

    config.save(out_path)
    print(f"Saved calibration to {out_path}")
    print(f"  contact_level = {config.contact_level:.2f}")
    print(f"  max_level     = {config.max_level:.2f}")
    print(f"  deadband      = {config.deadband:.2f}")
    return config


# ======================================================================
# STAGE 1+2+3+4: TACTILE PIPELINE (per-sample processing)
# ======================================================================

class TactilePipeline:
    """Owns filtering, feature extraction, state machine, and intensity
    normalization. This is the piece to replace with a calibrated ML
    model in V2+ — its output contract (TactileSample) stays the same."""

    def __init__(self, config: HapticConfig):
        self.cfg = config
        self.filtered_dx = 0.0
        self.filtered_dy = 0.0
        self.filtered_dz = 0.0
        self.prev_M = 0.0
        self._dM_hist = deque(maxlen=config.hf_window)
        self.state = TactileState.NO_CONTACT
        self._contact_latched = False

    def process(self, bx, by, bz):
        cfg = self.cfg

        # --- baseline subtraction ---
        dx = bx - cfg.baseline_bx
        dy = by - cfg.baseline_by
        dz = bz - cfg.baseline_bz

        # --- deadband (per-axis, before filtering to avoid smoothing noise in) ---
        if abs(dx) < cfg.deadband: dx = 0.0
        if abs(dy) < cfg.deadband: dy = 0.0
        if abs(dz) < cfg.deadband: dz = 0.0

        # --- EMA filter (per-axis; keeps direction info for future spatial mode) ---
        a = cfg.alpha
        self.filtered_dx = a * dx + (1 - a) * self.filtered_dx
        self.filtered_dy = a * dy + (1 - a) * self.filtered_dy
        self.filtered_dz = a * dz + (1 - a) * self.filtered_dz
        fdx, fdy, fdz = self.filtered_dx, self.filtered_dy, self.filtered_dz

        # --- Stage 2: features ---
        M = math.sqrt(fdx * fdx + fdy * fdy + fdz * fdz)
        N = abs(fdz)
        S = math.sqrt(fdx * fdx + fdy * fdy)
        dM = M - self.prev_M
        self.prev_M = M
        self._dM_hist.append(dM)
        hf_energy = _variance(self._dM_hist)

        # --- Stage 3: state machine ---
        on_th = cfg.contact_level
        off_th = cfg.contact_level * cfg.release_hysteresis_ratio

        prev_contact = self._contact_latched
        if not self._contact_latched and M > on_th:
            self._contact_latched = True
        elif self._contact_latched and M < off_th:
            self._contact_latched = False

        if self._contact_latched:
            eps = 1e-6
            if (S / max(N, eps)) > cfg.shear_ratio_threshold and hf_energy > cfg.hf_energy_threshold:
                # Diagnostic label only — NOT experimentally validated in V1.
                # Haptic encoding below does NOT branch on this yet.
                state = TactileState.SLIDING_OR_SHEAR
            else:
                state = TactileState.CONTACT
            contact_onset = not prev_contact
            contact_release = False
        else:
            state = TactileState.NO_CONTACT
            contact_onset = False
            contact_release = prev_contact  # was in contact last tick, now isn't

        if contact_release:
            state = TactileState.RELEASE  # transient, reported once

        self.state = state

        # --- Stage 4: normalized intensity ---
        if self._contact_latched:
            I = clamp((M - cfg.contact_level) / max(cfg.max_level - cfg.contact_level, 1e-6), 0.0, 1.0)
        else:
            I = 0.0

        return TactileSample(
            dx=fdx, dy=fdy, dz=fdz,
            M=M, N=N, S=S, dM=dM, hf_energy=hf_energy,
            state=state, intensity=I,
            contact_onset=contact_onset, contact_release=contact_release,
        )


def _variance(values):
    n = len(values)
    if n < 2:
        return 0.0
    m = sum(values) / n
    return sum((v - m) ** 2 for v in values) / n


@dataclass
class TactileSample:
    dx: float
    dy: float
    dz: float
    M: float
    N: float
    S: float
    dM: float
    hf_energy: float
    state: TactileState
    intensity: float
    contact_onset: bool
    contact_release: bool


# ======================================================================
# STAGE 5-10: HAPTIC ENCODER
# ======================================================================

class HapticEncoder:
    """Converts a TactileSample.intensity (0..1) + event flags into six
    motor PWM values, using amplitude, pulse timing, and motor recruitment
    as three separate perceptual dimensions. Non-blocking: call
    update(sample) once per tick, it uses wall-clock time internally."""

    def __init__(self, config: HapticConfig):
        self.cfg = config
        self._phase_start = time.monotonic()
        self._pulse_on = True
        self._transient_until = 0.0
        self._transient_motors = None

    def _base_pwm(self, I):
        cfg = self.cfg
        if I <= 0:
            return 0
        return int(round(cfg.pwm_min + (cfg.pwm_max - cfg.pwm_min) * (I ** cfg.gamma)))

    def _motor_count(self, I):
        return step_table(self.cfg.recruitment_table, I)

    def _pulse_timing(self, I):
        on_ms, off_ms = interp_table(self.cfg.pulse_table, I)
        return on_ms, off_ms

    def _recruitment_order(self, sample: TactileSample):
        """Stage 7 hook: returns the activation order of motor indices.
        Default (spatial_mode_enabled=False): fixed 0..5 order. Enabling
        spatial mode lets dominant dx/dy bias which motors light up first —
        wired but OFF by default per V1 scope."""
        if not self.cfg.spatial_mode_enabled:
            return [0, 1, 2, 3, 4, 5]

        # M0 M1 / M2 M3 / M4 M5 layout. Bias order toward the side the
        # dominant lateral axis points to; falls back to default order
        # once all motors are recruited anyway.
        if abs(sample.dx) >= abs(sample.dy):
            order = [0, 2, 4, 1, 3, 5] if sample.dx >= 0 else [1, 3, 5, 0, 2, 4]
        else:
            order = [0, 1, 2, 3, 4, 5] if sample.dy >= 0 else [4, 5, 2, 3, 0, 1]
        return order

    def update(self, sample: TactileSample):
        cfg = self.cfg
        now = time.monotonic()
        I = sample.intensity

        # --- Stage 10: contact-onset / release transient overlay ---
        if sample.contact_onset and cfg.onset_pulse_enabled:
            self._transient_until = now + cfg.onset_pulse_ms / 1000.0
            self._transient_motors = [cfg.onset_pulse_pwm, cfg.onset_pulse_pwm, 0, 0, 0, 0]
        elif sample.contact_release and cfg.release_pulse_enabled:
            self._transient_until = now + cfg.release_pulse_ms / 1000.0
            self._transient_motors = [cfg.release_pulse_pwm, cfg.release_pulse_pwm, 0, 0, 0, 0]

        if now < self._transient_until:
            return list(self._transient_motors), (0, 0)  # brief overlay wins, skips normal encoding

        if I <= 0.0:
            self._pulse_on = True
            self._phase_start = now
            return [0] * NUM_MOTORS, (0, 0)

        pwm = self._base_pwm(I)
        on_ms, off_ms = self._pulse_timing(I)
        active_count = self._motor_count(I)

        if off_ms <= 0:
            drive = pwm  # continuous (I at/near 1.0)
        else:
            elapsed_ms = (now - self._phase_start) * 1000.0
            phase_len = on_ms if self._pulse_on else off_ms
            if elapsed_ms >= phase_len:
                self._pulse_on = not self._pulse_on
                self._phase_start = now
            drive = pwm if self._pulse_on else 0

        order = self._recruitment_order(sample)
        motors = [0] * NUM_MOTORS
        for rank, motor_idx in enumerate(order):
            if rank < active_count:
                motors[motor_idx] = drive

        return motors, (on_ms, off_ms)


def build_motor_command(motors):
    """Stage 11: PC-side output, GPIO-agnostic — Haptic ESP32 owns pins."""
    return "M," + ",".join(str(int(v)) for v in motors)


# ======================================================================
# LOGGING (Stage 12)
# ======================================================================

LOG_COLUMNS = [
    "timestamp", "Bx", "By", "Bz", "dx", "dy", "dz",
    "M", "N", "S", "dM", "HF_energy",
    "contact", "intensity", "haptic_level",
    "motor0", "motor1", "motor2", "motor3", "motor4", "motor5",
]


def log_row(ts, bx, by, bz, sample: TactileSample, motors):
    return [
        ts, bx, by, bz, sample.dx, sample.dy, sample.dz,
        sample.M, sample.N, sample.S, sample.dM, sample.hf_energy,
        int(sample.state != TactileState.NO_CONTACT), sample.intensity, sample.state.value,
        *motors,
    ]


# ======================================================================
# LIVE TERMINAL VISUALIZATION (Stage 13)
# ======================================================================

def render_bar(value, max_value, width=16):
    filled = int(round(clamp(value / max_value, 0, 1) * width)) if max_value > 0 else 0
    return "#" * filled + "." * (width - filled)


def render_view(bx, by, bz, sample: TactileSample, motors, on_off_ms):
    lines = []
    lines.append(f"Bx={bx:8.2f}  By={by:8.2f}  Bz={bz:8.2f}")
    lines.append(f"M(raw-ish)={sample.M:7.2f}  N={sample.N:7.2f}  S={sample.S:7.2f}  dM={sample.dM:7.2f}  HF={sample.hf_energy:6.2f}")
    lines.append("")
    lines.append(f"TACTILE INTENSITY")
    lines.append(f"{render_bar(sample.intensity, 1.0)} {sample.intensity:.2f}")
    lines.append("")
    lines.append(f"STATE: {sample.state.value}")
    lines.append("")
    lines.append("MOTORS:")
    for i, m in enumerate(motors):
        lines.append(f"M{i} {render_bar(m, 255, width=10)} {m}")
    lines.append("")
    on_ms, off_ms = on_off_ms
    if off_ms <= 0 and any(motors):
        lines.append("Pattern: continuous")
    else:
        lines.append(f"Pattern: {on_ms:.0f} ms ON / {off_ms:.0f} ms OFF")
    return "\n".join(lines)


# ======================================================================
# SIMULATION MODE — feed synthetic tactile values, inspect motor output
# ======================================================================

def simulate(magnitudes, config: HapticConfig = None, hold_seconds=2.0, tick_hz=50, verbose=True):
    """Feeds a list of synthetic M values directly into the state machine
    /encoder (bypassing Bx/By/Bz/baseline), so PC->haptic logic can be
    validated without any sensor or Haptic ESP32 attached."""
    cfg = config or HapticConfig()
    pipeline = TactilePipeline(cfg)
    encoder = HapticEncoder(cfg)
    pipeline._contact_latched = False  # start clean

    results = []
    for target_M in magnitudes:
        t_end = time.monotonic() + hold_seconds
        while time.monotonic() < t_end:
            # Directly synthesize a TactileSample around the target M,
            # reusing the same state-machine/intensity math as real data.
            on_th = cfg.contact_level
            off_th = cfg.contact_level * cfg.release_hysteresis_ratio
            prev_contact = pipeline._contact_latched
            if not pipeline._contact_latched and target_M > on_th:
                pipeline._contact_latched = True
            elif pipeline._contact_latched and target_M < off_th:
                pipeline._contact_latched = False

            contact_onset = pipeline._contact_latched and not prev_contact
            contact_release = (not pipeline._contact_latched) and prev_contact
            state = (TactileState.RELEASE if contact_release else
                     TactileState.CONTACT if pipeline._contact_latched else
                     TactileState.NO_CONTACT)

            I = clamp((target_M - cfg.contact_level) / max(cfg.max_level - cfg.contact_level, 1e-6), 0.0, 1.0) \
                if pipeline._contact_latched else 0.0

            sample = TactileSample(
                dx=0, dy=0, dz=target_M, M=target_M, N=target_M, S=0, dM=0, hf_energy=0,
                state=state, intensity=I,
                contact_onset=contact_onset, contact_release=contact_release,
            )
            motors, on_off = encoder.update(sample)
            results.append((target_M, I, list(motors)))

            if verbose:
                sys.stdout.write("\033[H\033[J")
                print(f"SIMULATE  input M={target_M}")
                print(render_view(0, 0, target_M, sample, motors, on_off))
                sys.stdout.flush()

            time.sleep(1.0 / tick_hz)
    return results


# ======================================================================
# EXAMPLE INPUT/OUTPUT (doctest-style, run this file directly to see it)
# ======================================================================

def _example():
    cfg = HapticConfig()  # defaults: contact_level=8, max_level=80
    pipeline = TactilePipeline(cfg)
    encoder = HapticEncoder(cfg)

    print("Example: synthetic Bx,By,Bz sequence (baseline=0,0,0) -> command\n")
    examples = [
        (0, 0, 0),          # resting
        (2, 1, 3),          # still resting, under deadband
        (5, 3, 10),         # light contact starting
        (10, 5, 20),        # light
        (15, 8, 40),        # medium
        (20, 10, 65),       # strong
        (25, 12, 90),       # maximum
    ]
    for bx, by, bz in examples:
        sample = pipeline.process(bx, by, bz)
        motors, on_off = encoder.update(sample)
        cmd = build_motor_command(motors)
        # NOTE: `cmd` is a single instantaneous snapshot — since the pulse
        # generator toggles ON/OFF over time, it may legitimately show all
        # zeros here if this instant lands in an OFF phase (or in a brief
        # contact-onset transient). target_pwm/count/pattern below show the
        # steady-state encoding independent of pulse phase.
        target_pwm = encoder._base_pwm(sample.intensity)
        active_count = encoder._motor_count(sample.intensity)
        print(f"Bx={bx:4} By={by:4} Bz={bz:4}  -> M={sample.M:6.2f} I={sample.intensity:.2f} "
              f"state={sample.state.value:8s} target_pwm={target_pwm:3d} active_motors={active_count} "
              f"pattern={on_off[0]:.0f}/{on_off[1]:.0f}ms  instant->{cmd}")
        time.sleep(0.05)  # let any contact-onset transient pulse (40ms) clear before next line


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="haptic_algorithm.py standalone demo")
    parser.add_argument("--simulate", action="store_true", help="run built-in synthetic magnitude sweep")
    parser.add_argument("--example", action="store_true", help="print a short worked example")
    args = parser.parse_args()

    if args.simulate:
        simulate([0, 5, 10, 25, 50, 80, 100], hold_seconds=2.0)
    elif args.example:
        _example()
    else:
        _example()
