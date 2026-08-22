#!/usr/bin/env python3
"""
haptic_engine.py — PC-side runner. All tactile processing / haptic
encoding math lives in haptic_algorithm.py (V1 algorithm); this file is
purely serial I/O, mode selection, dashboard, and CLI glue.

Architecture (DO NOT collapse):

    eFlesh -> MLX90393 -> Sensor ESP32-S3 --USB serial--> THIS SCRIPT (PC)
        -> haptic_algorithm.TactilePipeline -> haptic_algorithm.HapticEncoder
        --USB serial--> Haptic ESP32-S3 -> 6 motor drivers -> coin motors

M/N/S/intensity computed in haptic_algorithm.py are RELATIVE magnetic/
deformation signals, NOT Newtons, and are NOT the original eFlesh
project's published research results.

Usage:
    python haptic_engine.py --sensor-port COM7 --haptic-port COM8
    python haptic_engine.py --sensor-port COM7 --haptic-port COM8 --calibrate
    python haptic_engine.py --simulate --haptic-port COM8
    python haptic_engine.py --sensor-only --sensor-port COM7

    # Add the browser dashboard to full or sensor-only mode. The terminal
    # dashboard keeps running either way -- it is the fallback if the
    # browser dies mid-demo.
    python haptic_engine.py --sensor-port COM7 --haptic-port COM8 --gui

    # Alternate haptic transports (see transports/, docs/transport-options.md).
    # USB is the only one tested against real hardware.
    python haptic_engine.py --sensor-port COM7 --transport wifi
    python haptic_engine.py --sensor-port COM7 --transport ble

Required package: pyserial (pip install pyserial). Wi-Fi transport needs
no extra package (stdlib socket); BLE transport needs 'bleak' (pip
install bleak), only if you actually select --transport ble.
"""

import argparse
import csv
import os
import queue
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial. Install with: pip install pyserial")
    sys.exit(1)

from haptic_algorithm import (
    HapticConfig, TactilePipeline, HapticEncoder, Calibrator,
    run_guided_calibration, log_row, LOG_COLUMNS,
    NUM_MOTORS, TactileSample, TactileState,
    simulate as simulate_magnitudes,
)
from transports import get_transport, MotorCommand, get_default_transport_name
import telemetry

BAUD_RATE = 115200
SERIAL_READ_TIMEOUT_S = 0.01

BASELINE_SAMPLES = 100           # startup/'b' auto baseline (median), independent of
                                  # contact_level/max_level which come from calibration.json
MAIN_LOOP_SLEEP_S = 0.002        # ~500Hz poll, cheap, non-blocking-ish
HAPTIC_SEND_HZ = 50              # rate we push M,... commands to haptic ESP32
DASHBOARD_HZ = 8

LOG_DIR = "demo_logs"
DEFAULT_CALIBRATION_FILE = "calibration.json"

DEMO_FEED_HZ = 100        # rate scripted demo input is fed to the pipeline,
                          # matching the real sensor's ~100 Hz stream
HAPTIC_PING_S = 2.0       # PING interval, so the GUI's latency readout is real
EVENT_STICKY_S = 0.2      # how long a one-tick onset/release stays visible to
                          # a 30 Hz GUI (the event itself is not extended)


def idle_sample():
    """A zeroed TactileSample, for the window before the first real one."""
    return TactileSample(0, 0, 0, 0, 0, 0, 0, 0,
                         TactileState.NO_CONTACT, 0.0, False, False)


# ======================================================================
# SERIAL I/O — background reader threads (never block the main loop)
# ======================================================================

class SensorReader(threading.Thread):
    """Reads 'S,<ts>,<bx>,<by>,<bz>' lines from the Sensor ESP32 in the
    background and pushes parsed tuples into a queue. Non-'S,' lines
    (faults, debug) are kept in last_other_line."""

    def __init__(self, port, baud=BAUD_RATE):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=SERIAL_READ_TIMEOUT_S)
        self.out_queue = queue.Queue()
        self.running = True
        self.sample_count = 0
        self.last_rate_check = time.monotonic()
        self.rate_hz = 0.0
        self.connected = True
        self.last_other_line = ""

    def run(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                self.connected = False
                time.sleep(0.1)
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line.startswith("S,"):
                parts = line.split(",")
                if len(parts) != 5:
                    continue  # malformed packet, drop silently
                try:
                    ts = int(parts[1])
                    bx = float(parts[2])
                    by = float(parts[3])
                    bz = float(parts[4])
                except ValueError:
                    continue  # corrupted packet, drop
                self.out_queue.put((ts, bx, by, bz))
                self.sample_count += 1
            else:
                self.last_other_line = line  # F,... or #DBG ...

            now = time.monotonic()
            if now - self.last_rate_check >= 1.0:
                self.rate_hz = self.sample_count / (now - self.last_rate_check)
                self.sample_count = 0
                self.last_rate_check = now

    def read_one_blocking(self, timeout=2.0):
        """Used by the guided calibration wizard — blocks until one sample
        arrives (or raises TimeoutError)."""
        try:
            ts, bx, by, bz = self.out_queue.get(timeout=timeout)
            return (bx, by, bz)
        except queue.Empty:
            raise TimeoutError("No sensor data received — check wiring/port.")

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


def _drain_queue(q):
    """Empty a queue into a list without blocking."""
    out = []
    while True:
        try:
            out.append(q.get_nowait())
        except queue.Empty:
            return out


class KeyboardReader(threading.Thread):
    """Line-buffered keyboard input (type a command then Enter). Kept
    simple and cross-platform instead of raw single-keypress capture."""

    def __init__(self):
        super().__init__(daemon=True)
        self.out_queue = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                break
            cmd = line.strip().lower()
            if cmd:
                self.out_queue.put(cmd)

    def stop(self):
        self.running = False


# ======================================================================
# CSV LOGGER — schema comes from haptic_algorithm.LOG_COLUMNS (Stage 12)
# ======================================================================

class DemoLogger:
    """The one and only CSV writer. Schema is haptic_algorithm.LOG_COLUMNS.

    start()/stop() exist so the GUI's log buttons drive THIS writer at
    runtime rather than a second logger being added alongside it. Each
    start() opens a new timestamped file; stop() closes it cleanly.
    """

    FLUSH_EVERY = 200   # ~2 s at 100 Hz, so a crash loses very little

    def __init__(self, enabled, log_dir=LOG_DIR):
        self.log_dir = log_dir
        self.enabled = False
        self.file = None
        self.writer = None
        self.path = None
        self.rows = 0
        if enabled:
            self.start()

    def start(self):
        """Open a fresh CSV. No-op if already logging, so a double click
        in the GUI cannot orphan a half-written file."""
        if self.enabled:
            return self.path
        os.makedirs(self.log_dir, exist_ok=True)
        fname = time.strftime("%Y%m%d_%H%M%S") + ".csv"
        path = os.path.join(self.log_dir, fname)
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(LOG_COLUMNS)
        self.path = path
        self.rows = 0
        self.enabled = True
        return path

    def log(self, row):
        if not self.enabled:
            return
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % self.FLUSH_EVERY == 0:
            try:
                self.file.flush()
            except Exception:
                pass

    def stop(self):
        if not self.enabled:
            return None
        closed = self.path
        self.enabled = False
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        self.file = None
        self.writer = None
        return closed

    def close(self):
        self.stop()


# ======================================================================
# CALIBRATION HELPERS
# ======================================================================

def load_or_default_config(path):
    if os.path.exists(path):
        print(f"Loaded calibration from {path}")
        return HapticConfig.load(path)
    print(f"No calibration file at {path} — using built-in defaults "
          f"(placeholders, run --calibrate to tune for your eFlesh patch).")
    return HapticConfig()


class MaxCapture:
    """Calibration step 2: capture the peak M of the hardest press the demo
    will use, and make that I = 1.0.

    This step is not optional. contact_level alone only says where contact
    starts; without max_level the normalized intensity I has no defined top
    and every downstream mapping (PWM curve, pulse table, recruitment) is
    reading against an arbitrary scale.

    Writes into the same calibration.json HapticConfig already uses -- no
    second config file, and never in newtons.
    """

    DURATION_S = 4.0

    def __init__(self, cfg: HapticConfig, path):
        self.cfg = cfg
        self.path = path
        self.t0 = time.monotonic()
        self.peak = 0.0
        self.done = False

    def feed(self, m):
        if self.done:
            return
        if m > self.peak:
            self.peak = m
        if time.monotonic() - self.t0 >= self.DURATION_S:
            self.done = True

    def progress(self):
        return min(1.0, (time.monotonic() - self.t0) / self.DURATION_S)

    def status(self):
        return {"progress": self.progress(), "peak": self.peak}

    def commit(self):
        """Returns (ok, message). Refuses to save a peak that would make
        the intensity scale nonsense."""
        if self.peak <= self.cfg.contact_level:
            return False, (f"peak M {self.peak:.2f} is at or below contact_level "
                           f"{self.cfg.contact_level:.2f} - press harder; nothing saved")
        self.cfg.max_level = self.peak
        self.cfg.save(self.path)
        return True, f"max_level = {self.peak:.2f} saved to {self.path}"


class DemoScript:
    """Scripted walk through the intensity bands, for a hands-off demo.

    What this synthesizes is SENSOR INPUT -- Bx/By/Bz -- and nothing else.
    Each step's target is converted back into a synthetic Bz offset and
    pushed through the real TactilePipeline, so the deadband, EMA filter,
    feature extraction, state machine, intensity normalization and haptic
    encoder running in demo mode are exactly the ones running live. No
    pipeline stage is reimplemented here and no motor value is invented.

    Live mode never constructs this class. While it is running, both the
    terminal dashboard and the GUI are badged DEMO MODE.
    """

    # (label, target intensity, seconds). The targets sit inside each band
    # of the default recruitment_table (1/2/4/6 motors at .05/.25/.5/.75),
    # so the walk visibly recruits motors one group at a time.
    STEPS = [
        ("NO CONTACT", 0.00, 3.0),
        ("LIGHT",      0.15, 4.0),
        ("MEDIUM",     0.35, 4.0),
        ("STRONG",     0.60, 4.0),
        ("MAXIMUM",    0.95, 4.0),
        ("RELEASE",    0.00, 3.0),
    ]

    def __init__(self, cfg: HapticConfig):
        self.cfg = cfg
        self.idx = 0
        self.t0 = time.monotonic()
        self.done = False

    def _target_m(self, intensity):
        """Pick a synthetic input magnitude that will land on the given
        intensity. This chooses an INPUT; TactilePipeline still computes
        the actual I from it, through the normal Stage 1-4 path."""
        if intensity <= 0:
            return 0.0
        span = max(self.cfg.max_level - self.cfg.contact_level, 1e-6)
        return self.cfg.contact_level + intensity * span

    def sample(self):
        """Next (bx, by, bz) for pipeline.process(), or None when finished."""
        if self.done:
            return None
        _, intensity, dur = self.STEPS[self.idx]
        if time.monotonic() - self.t0 >= dur:
            self.idx += 1
            self.t0 = time.monotonic()
            if self.idx >= len(self.STEPS):
                self.done = True
                return None
            _, intensity, dur = self.STEPS[self.idx]
        cfg = self.cfg
        # Z axis only: dx = dy = 0, so the pipeline sees a clean
        # normal-ish deformation and S stays at zero.
        return (cfg.baseline_bx, cfg.baseline_by,
                cfg.baseline_bz + self._target_m(intensity))

    def status(self):
        label, intensity, dur = self.STEPS[min(self.idx, len(self.STEPS) - 1)]
        return {
            "active": not self.done,
            "step": label,
            "step_index": self.idx + 1,
            "step_count": len(self.STEPS),
            "remaining_s": max(0.0, dur - (time.monotonic() - self.t0)),
            "target_i": intensity,
        }


def start_hub(args):
    """Bring up the GUI server, or return None if --gui was not passed."""
    if not getattr(args, "gui", False):
        return None
    hub = telemetry.TelemetryHub(host=args.gui_host, port=args.gui_port)
    url = hub.start()
    print(f"GUI dashboard: {url}")
    print(f"Data-path check (raw JSON): {url}raw")
    if args.gui_host not in ("127.0.0.1", "localhost"):
        print(f"[WARN] GUI bound to {args.gui_host}, reachable from the network. "
              f"This endpoint can start and stop motors.")
    time.sleep(1.5)   # leave the URL readable before the dashboard repaints
    return hub


def auto_calibrate_baseline(sensor: SensorReader, cfg: HapticConfig):
    """Quick median-of-N baseline (Stage: baseline). Does NOT touch
    contact_level/max_level — those come from calibration.json / the
    guided wizard, since they don't drift run-to-run the way baseline does."""
    calibrator = Calibrator(cfg, num_samples=BASELINE_SAMPLES)
    calibrator.begin()
    while not calibrator.done:
        while not sensor.out_queue.empty():
            ts, bx, by, bz = sensor.out_queue.get()
            if calibrator.feed(bx, by, bz):
                break
        time.sleep(MAIN_LOOP_SLEEP_S)


# ======================================================================
# DASHBOARD (Stage 13)
# ======================================================================

def render_dashboard(sensor_rate, bx, by, bz, sample, motors, on_off,
                      sensor_ok, haptic_ok, mode_label, gui_url=None):
    def bar(v, vmax, width=16):
        filled = int(round(max(0.0, min(1.0, v / vmax)) * width)) if vmax > 0 else 0
        return "#" * filled + "." * (width - filled)

    lines = []
    lines.append("E-FLESH HAPTIC ENGINE  [{}]".format(mode_label))
    lines.append("-" * 40)
    lines.append(f"Sensor rate: {sensor_rate:.1f} Hz")
    lines.append("")
    lines.append(f"Bx: {bx:8.2f}   By: {by:8.2f}   Bz: {bz:8.2f}")
    lines.append(f"dx: {sample.dx:8.2f}   dy: {sample.dy:8.2f}   dz: {sample.dz:8.2f}")
    lines.append("")
    lines.append(f"M: {sample.M:7.2f}   N: {sample.N:7.2f}   S: {sample.S:7.2f}   "
                 f"dM: {sample.dM:7.2f}   HF: {sample.hf_energy:6.2f}")
    lines.append("")
    lines.append(f"TACTILE INTENSITY")
    lines.append(f"{bar(sample.intensity, 1.0)} {sample.intensity:.2f}")
    lines.append("")
    lines.append(f"STATE: {sample.state.value}")
    lines.append("")
    lines.append("MOTORS:")
    for i, m in enumerate(motors):
        lines.append(f"M{i} {bar(m, 255, width=10)} {m}")
    on_ms, off_ms = on_off
    if off_ms <= 0 and any(motors):
        lines.append("Pattern: continuous")
    else:
        lines.append(f"Pattern: {on_ms:.0f} ms ON / {off_ms:.0f} ms OFF")
    lines.append("")
    lines.append(f"Sensor connection: {'OK' if sensor_ok else 'LOST'}")
    lines.append(f"Haptic connection: {'OK' if haptic_ok else 'LOST'}")
    lines.append("")
    lines.append("Keys: [b]=recalibrate baseline  [s]=stop motors  [q]=quit")
    if gui_url:
        lines.append(f"GUI:  {gui_url}   (raw: {gui_url}raw)")

    sys.stdout.write("\033[H\033[J")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


# ======================================================================
# MODES
# ======================================================================

def run_full_mode(args):
    sensor = SensorReader(args.sensor_port)
    haptic = get_transport(args.transport, port=args.haptic_port)
    if not haptic.connect():
        print(f"[WARN] Could not connect haptic transport '{args.transport}' — "
              f"continuing, dashboard will show 'Haptic connection: LOST'.")
    logger = DemoLogger(enabled=not args.no_log)
    keys = KeyboardReader()
    keys.start()

    # max_level is only meaningful if it came from a real press. Treat a
    # pre-existing calibration.json as calibrated, and anything else as
    # built-in defaults, so the GUI can say so plainly instead of
    # implying I=1.0 means something it does not.
    max_calibrated = os.path.exists(args.calibration_file)
    cfg = load_or_default_config(args.calibration_file)
    hub = start_hub(args)
    gui_url = hub.url if hub is not None else None

    if args.calibrate:
        print("Running guided calibration wizard...")
        run_guided_calibration(lambda: sensor.read_one_blocking(),
                                cfg, out_path=args.calibration_file)

    pipeline = TactilePipeline(cfg)
    encoder = HapticEncoder(cfg)

    print("Calibrating baseline. DO NOT TOUCH the eFlesh...")
    calibrating = True
    calibrator = Calibrator(cfg, num_samples=BASELINE_SAMPLES)
    calibrator.begin()

    last_bxyz = (0.0, 0.0, 0.0)
    last_haptic_send = 0.0
    last_dashboard = 0.0
    last_sample = None
    last_on_off = (0, 0)
    last_motors = [0] * NUM_MOTORS   # the last command ACTUALLY SENT to the haptic ESP32
    last_ping = 0.0
    onset_at = -99.0
    release_at = -99.0
    seq = 0
    demo = None
    demo_next = 0.0
    max_capture = None
    notice = None            # (text, monotonic_ts), surfaced in the GUI
    running = True

    try:
        while running:
            # ---- commands: keystrokes and GUI buttons take the SAME path ----
            pending = [(c, {}) for c in _drain_queue(keys.out_queue)]
            if hub is not None:
                pending.extend(hub.drain_commands())

            for cmd, _cargs in pending:
                if cmd in ("b", "calibrate_baseline"):
                    print("\nRecalibrating baseline. DO NOT TOUCH the eFlesh...")
                    haptic.stop()
                    last_motors, last_on_off = [0] * NUM_MOTORS, (0, 0)
                    max_capture = None
                    calibrator.begin()
                    calibrating = True
                elif cmd in ("s", "stop_motors"):
                    haptic.stop()
                    last_motors, last_on_off = [0] * NUM_MOTORS, (0, 0)
                    demo = None
                    notice = ("motors stopped", time.monotonic())
                elif cmd == "calibrate_max":
                    if calibrating:
                        notice = ("finish baseline calibration first",
                                  time.monotonic())
                    else:
                        max_capture = MaxCapture(cfg, args.calibration_file)
                        print("\nMAX CALIBRATION: press as hard as the demo "
                              "will go, and hold...")
                elif cmd == "start_demo":
                    demo = DemoScript(cfg)
                    demo_next = 0.0
                elif cmd == "stop_demo":
                    demo = None
                    haptic.stop()
                    last_motors, last_on_off = [0] * NUM_MOTORS, (0, 0)
                elif cmd == "start_log":
                    path = logger.start()
                    notice = (f"logging to {path}", time.monotonic())
                elif cmd == "stop_log":
                    closed = logger.stop()
                    notice = ((f"log closed: {closed}" if closed else "not logging"),
                              time.monotonic())
                elif cmd == "q":
                    running = False

            now = time.monotonic()

            # ---- sensor input: real samples, or scripted input in demo mode ----
            # encoder.update() is deliberately NOT called in here. It is
            # stateful (wall-clock pulse phase + one-shot transient window),
            # so it is called exactly ONCE per send tick below. That single
            # result is authoritative for the motor command, the CSV row, the
            # terminal dashboard and the GUI snapshot alike -- calling
            # update() here as well would produce a value that never reaches
            # the motors, and the screen would stop matching the hardware.
            if demo is not None:
                # Real sensor data is discarded for the duration, so live and
                # scripted input can never mix. DEMO MODE is badged on screen
                # the whole time.
                _drain_queue(sensor.out_queue)
                if now >= demo_next:
                    demo_next = now + 1.0 / DEMO_FEED_HZ
                    syn = demo.sample()
                    if syn is None:
                        demo = None
                    else:
                        bx, by, bz = syn
                        last_bxyz = syn
                        last_sample = pipeline.process(bx, by, bz)
                        if logger.enabled:
                            logger.log(log_row(time.time(), bx, by, bz,
                                               last_sample, last_motors))
            else:
                while not sensor.out_queue.empty():
                    ts, bx, by, bz = sensor.out_queue.get()
                    last_bxyz = (bx, by, bz)
                    if calibrating:
                        if calibrator.feed(bx, by, bz):
                            calibrating = False
                            print(f"Baseline set: ({cfg.baseline_bx:.2f}, "
                                  f"{cfg.baseline_by:.2f}, {cfg.baseline_bz:.2f})")
                        continue
                    last_sample = pipeline.process(bx, by, bz)
                    if max_capture is not None:
                        max_capture.feed(last_sample.M)
                    if logger.enabled:
                        # Motor columns are the last command actually sent (at
                        # most 1/HAPTIC_SEND_HZ old), never a phantom value.
                        logger.log(log_row(time.time(), bx, by, bz,
                                           last_sample, last_motors))

            # ---- calibration step 2 completion ----
            if max_capture is not None and max_capture.done:
                ok, msg = max_capture.commit()
                if ok:
                    max_calibrated = True
                print("\n" + ("MAX CALIBRATION: " + msg))
                notice = (msg, time.monotonic())
                max_capture = None

            # ---- drain all pending sensor samples ----
            # encoder.update() is deliberately NOT called here. It is stateful
            # (wall-clock pulse phase + one-shot transient window), so it is
            # called exactly ONCE per send tick below. That single result is
            # authoritative for the motor command, the CSV row, the terminal
            # dashboard and the GUI snapshot alike -- calling update() here as
            # well would produce a value that never reaches the motors, and the
            # screen would stop matching the hardware.
            # ---- push motor command to haptic ESP32 at fixed rate ----
            if now - last_haptic_send >= 1.0 / HAPTIC_SEND_HZ:
                last_haptic_send = now
                if not calibrating and last_sample is not None:
                    last_motors, last_on_off = encoder.update(last_sample)
                    # The one-shot event flags have now been consumed by that
                    # single update() call. Clearing them stops the next tick
                    # from re-arming the onset/release transient off the same
                    # stale sample (which stretched a 40ms pulse indefinitely).
                    last_sample.contact_onset = False
                    last_sample.contact_release = False
                    if last_sample.contact_onset:
                        onset_at = now
                    if last_sample.contact_release:
                        release_at = now
                    haptic.send_motor_command(MotorCommand.from_list(last_motors))
                else:
                    last_motors, last_on_off = [0] * NUM_MOTORS, (0, 0)
                    haptic.stop()

            # ---- liveness probe, so the GUI's latency readout is measured
            # rather than guessed. PING does not reset the firmware watchdog;
            # the 50 Hz M, stream above is what keeps it fed.
            if now - last_ping >= HAPTIC_PING_S:
                last_ping = now
                haptic.ping()

            # ---- dashboard ----
            if now - last_dashboard >= 1.0 / DASHBOARD_HZ:
                last_dashboard = now
                bx, by, bz = last_bxyz
                mode_label = "CALIBRATING..." if calibrating else "RUNNING"
                if last_sample is None:
                    last_sample = idle_sample()
                if demo is not None:
                    mode_label = "DEMO MODE"
                render_dashboard(sensor.rate_hz, bx, by, bz, last_sample, last_motors,
                                  last_on_off, sensor.connected, haptic.is_connected(),
                                  mode_label, gui_url=gui_url)

            # ---- GUI snapshot (decimated inside publish(), never blocks) ----
            if hub is not None:
                seq += 1
                hub.publish(telemetry.build_snapshot(
                    seq=seq, cfg=cfg,
                    sample=last_sample if last_sample is not None else idle_sample(),
                    motors=last_motors, on_off=last_on_off,
                    bx=last_bxyz[0], by=last_bxyz[1], bz=last_bxyz[2],
                    mode=("DEMO" if demo is not None else "LIVE"),
                    calibrating=calibrating,
                    sensor_ok=sensor.connected, sensor_rate_hz=sensor.rate_hz,
                    haptic_ok=haptic.is_connected(),
                    haptic_latency_ms=haptic.get_latency_ms(),
                    transport_name=args.transport,
                    baseline_progress=(
                        min(1.0, len(calibrator._buf[0]) / float(BASELINE_SAMPLES))
                        if calibrating else 1.0),
                    max_calibrated=max_calibrated,
                    max_capture=(max_capture.status() if max_capture else None),
                    log_active=logger.enabled, log_path=logger.path,
                    log_rows=logger.rows,
                    demo=(demo.status() if demo is not None else {"active": False}),
                    stream_hz=hub.stream_hz,
                    events={"onset": (now - onset_at) < EVENT_STICKY_S,
                            "release": (now - release_at) < EVENT_STICKY_S},
                    notice=(notice[0] if notice and (now - notice[1]) < 5.0 else None),
                ))

            time.sleep(MAIN_LOOP_SLEEP_S)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down: stopping all motors...")
        haptic.stop()
        time.sleep(0.1)
        sensor.stop()
        haptic.disconnect()
        keys.stop()
        logger.close()
        if hub is not None:
            hub.stop()


def run_sensor_only_mode(args):
    sensor = SensorReader(args.sensor_port)
    max_calibrated = os.path.exists(args.calibration_file)
    cfg = load_or_default_config(args.calibration_file)
    logger = DemoLogger(enabled=False)   # GUI-startable; off unless asked
    hub = start_hub(args)
    gui_url = hub.url if hub is not None else None

    if args.calibrate:
        print("Running guided calibration wizard...")
        run_guided_calibration(lambda: sensor.read_one_blocking(),
                                cfg, out_path=args.calibration_file)

    pipeline = TactilePipeline(cfg)

    print("Calibrating baseline. DO NOT TOUCH the eFlesh...")
    calibrating = True
    calibrator = Calibrator(cfg, num_samples=BASELINE_SAMPLES)
    calibrator.begin()

    last_dashboard = 0.0
    last_sample = None
    last_bxyz = (0.0, 0.0, 0.0)
    max_capture = None
    notice = None
    seq = 0
    running = True
    print("Sensor-only mode: motors will NOT be commanded. Ctrl+C to quit.")
    try:
        while running:
            # ---- GUI commands (no motor and no demo commands exist here:
            # this mode never opens a haptic transport at all) ----
            if hub is not None:
                for cmd, _cargs in hub.drain_commands():
                    if cmd == "calibrate_baseline":
                        calibrator.begin()
                        calibrating = True
                        max_capture = None
                    elif cmd == "calibrate_max":
                        if calibrating:
                            notice = ("finish baseline calibration first",
                                      time.monotonic())
                        else:
                            max_capture = MaxCapture(cfg, args.calibration_file)
                    elif cmd == "start_log":
                        notice = (f"logging to {logger.start()}", time.monotonic())
                    elif cmd == "stop_log":
                        closed = logger.stop()
                        notice = ((f"log closed: {closed}" if closed else "not logging"),
                                  time.monotonic())
                    elif cmd in ("stop_motors", "start_demo", "stop_demo"):
                        notice = ("sensor-only mode: motors are never commanded",
                                  time.monotonic())

            while not sensor.out_queue.empty():
                ts, bx, by, bz = sensor.out_queue.get()
                last_bxyz = (bx, by, bz)
                if calibrating:
                    if calibrator.feed(bx, by, bz):
                        calibrating = False
                    continue
                last_sample = pipeline.process(bx, by, bz)
                if max_capture is not None:
                    max_capture.feed(last_sample.M)
                if logger.enabled:
                    logger.log(log_row(time.time(), bx, by, bz, last_sample,
                                       [0] * NUM_MOTORS))

            now = time.monotonic()

            if max_capture is not None and max_capture.done:
                ok, msg = max_capture.commit()
                if ok:
                    max_calibrated = True
                notice = (msg, time.monotonic())
                max_capture = None

            if now - last_dashboard >= 1.0 / DASHBOARD_HZ:
                last_dashboard = now
                mode_label = "CALIBRATING..." if calibrating else "SENSOR-ONLY"
                if last_sample is None:
                    last_sample = idle_sample()
                bx, by, bz = last_bxyz
                render_dashboard(sensor.rate_hz, bx, by, bz, last_sample,
                                  [0] * NUM_MOTORS, (0, 0), sensor.connected, False,
                                  mode_label, gui_url=gui_url)

            if hub is not None:
                seq += 1
                hub.publish(telemetry.build_snapshot(
                    seq=seq, cfg=cfg,
                    sample=last_sample if last_sample is not None else idle_sample(),
                    motors=[0] * NUM_MOTORS, on_off=(0, 0),
                    bx=last_bxyz[0], by=last_bxyz[1], bz=last_bxyz[2],
                    mode="SENSOR-ONLY", calibrating=calibrating,
                    sensor_ok=sensor.connected, sensor_rate_hz=sensor.rate_hz,
                    haptic_ok=False, haptic_latency_ms=None,
                    transport_name="none",
                    baseline_progress=(
                        min(1.0, len(calibrator._buf[0]) / float(BASELINE_SAMPLES))
                        if calibrating else 1.0),
                    max_calibrated=max_calibrated,
                    max_capture=(max_capture.status() if max_capture else None),
                    log_active=logger.enabled, log_path=logger.path,
                    log_rows=logger.rows,
                    demo={"active": False},
                    stream_hz=hub.stream_hz,
                    notice=(notice[0] if notice and (now - notice[1]) < 5.0 else None),
                ))

            time.sleep(MAIN_LOOP_SLEEP_S)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop()
        logger.close()
        if hub is not None:
            hub.stop()


def run_simulate_mode(args):
    haptic = get_transport(args.transport, port=args.haptic_port)
    if not haptic.connect():
        print(f"[WARN] Could not connect haptic transport '{args.transport}' — "
              f"the on-screen preview will still run, but no real motors will move.")
    cfg = load_or_default_config(args.calibration_file)

    synthetic_sequence = [0, 5, 10, 25, 50, 80, 100]
    print("SIMULATION MODE: no eFlesh required. Cycling synthetic magnitudes:")
    print(synthetic_sequence)

    try:
        simulate_magnitudes(synthetic_sequence, config=cfg, hold_seconds=3.0, verbose=True)
        # simulate() above only computes; drive the real haptic ESP32 too:
    except KeyboardInterrupt:
        pass

    # Re-run the same sweep, this time actually sending to the Haptic ESP32
    # (kept as a second pass so the on-screen preview above and the real
    # motor drive are not interleaved/confusing).
    encoder = HapticEncoder(cfg)
    last_haptic_send = 0.0
    try:
        for mag in synthetic_sequence:
            step_start = time.monotonic()
            while time.monotonic() - step_start < 2.0:
                # Latch on the ON threshold, matching TactilePipeline.process()
                # and docs/haptic-algorithm.md. This previously compared against
                # off_th (the release threshold), so simulate mode declared
                # contact at 0.6x contact_level -- inconsistent with live mode.
                contact = mag > cfg.contact_level
                I = max(0.0, min(1.0, (mag - cfg.contact_level) /
                                  max(cfg.max_level - cfg.contact_level, 1e-6))) if contact else 0.0
                sample = TactileSample(0, 0, mag, mag, mag, 0, 0, 0,
                                        TactileState.CONTACT if contact else TactileState.NO_CONTACT,
                                        I, False, False)
                now = time.monotonic()
                if now - last_haptic_send >= 1.0 / HAPTIC_SEND_HZ:
                    last_haptic_send = now
                    motors, _ = encoder.update(sample)
                    haptic.send_motor_command(MotorCommand.from_list(motors))
                time.sleep(MAIN_LOOP_SLEEP_S)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nSimulation done. Stopping all motors...")
        haptic.stop()
        time.sleep(0.1)
        haptic.disconnect()


# ======================================================================
# CLI
# ======================================================================

def main():
    parser = argparse.ArgumentParser(description="eFlesh PC haptic processing engine")
    parser.add_argument("--sensor-port", help="Serial port for Sensor ESP32 (e.g. COM7 or /dev/ttyACM0)")
    parser.add_argument("--haptic-port", help="Serial port for Haptic ESP32 (e.g. COM8 or /dev/ttyACM1)")
    parser.add_argument("--simulate", action="store_true", help="Drive haptic ESP32 with synthetic magnitudes, no sensor needed")
    parser.add_argument("--sensor-only", action="store_true", help="Read/display sensor only, never command motors")
    parser.add_argument("--no-log", action="store_true", help="Disable CSV logging to demo_logs/")
    parser.add_argument("--calibration-file", default=DEFAULT_CALIBRATION_FILE,
                         help="Path to calibration.json (default: calibration.json)")
    parser.add_argument("--calibrate", action="store_true",
                         help="Run the guided calibration wizard before starting (requires --sensor-port)")
    parser.add_argument("--gui", action="store_true",
                         help="Serve the browser dashboard (full and sensor-only modes). "
                              "The terminal dashboard keeps running regardless - it is the "
                              "fallback if the browser dies mid-demo.")
    parser.add_argument("--gui-port", type=int, default=telemetry.DEFAULT_PORT,
                         help=f"Port for the GUI server (default {telemetry.DEFAULT_PORT})")
    parser.add_argument("--gui-host", default=telemetry.DEFAULT_HOST,
                         help=f"Bind address for the GUI server (default "
                              f"{telemetry.DEFAULT_HOST}). Only change this if you "
                              f"deliberately want the dashboard - which can start and "
                              f"stop motors - reachable from other machines.")
    parser.add_argument("--transport", default=get_default_transport_name(),
                         choices=["usb", "wifi", "ble", "bluetooth", "espnow"],
                         help="Haptic ESP32 transport (default: usb, or $TRANSPORT env var). "
                              "wifi/ble/espnow read host/device config from pc/.env or "
                              "pc/local_config.json — see .env.example. USB is the only "
                              "transport tested against real hardware; see docs/transport-options.md.")
    args = parser.parse_args()

    if args.simulate:
        if args.transport == "usb" and not args.haptic_port:
            parser.error("--simulate with --transport usb requires --haptic-port")
        run_simulate_mode(args)
    elif args.sensor_only:
        if not args.sensor_port:
            parser.error("--sensor-only requires --sensor-port")
        run_sensor_only_mode(args)
    else:
        if not args.sensor_port:
            parser.error("full mode requires --sensor-port (or use --simulate / --sensor-only)")
        if args.transport == "usb" and not args.haptic_port:
            parser.error("full mode with --transport usb requires --haptic-port")
        run_full_mode(args)


if __name__ == "__main__":
    main()
