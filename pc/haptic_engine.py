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

BAUD_RATE = 115200
SERIAL_READ_TIMEOUT_S = 0.01

BASELINE_SAMPLES = 100           # startup/'b' auto baseline (median), independent of
                                  # contact_level/max_level which come from calibration.json
MAIN_LOOP_SLEEP_S = 0.002        # ~500Hz poll, cheap, non-blocking-ish
HAPTIC_SEND_HZ = 50              # rate we push M,... commands to haptic ESP32
DASHBOARD_HZ = 8

LOG_DIR = "demo_logs"
DEFAULT_CALIBRATION_FILE = "calibration.json"


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
    def __init__(self, enabled, log_dir=LOG_DIR):
        self.enabled = enabled
        self.file = None
        self.writer = None
        if not enabled:
            return
        os.makedirs(log_dir, exist_ok=True)
        fname = time.strftime("%Y%m%d_%H%M%S") + ".csv"
        path = os.path.join(log_dir, fname)
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(LOG_COLUMNS)
        self.path = path

    def log(self, row):
        if not self.enabled:
            return
        self.writer.writerow(row)

    def close(self):
        if self.enabled and self.file:
            self.file.close()


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
                      sensor_ok, haptic_ok, mode_label):
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

    cfg = load_or_default_config(args.calibration_file)

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
    running = True

    try:
        while running:
            # ---- keyboard commands ----
            while not keys.out_queue.empty():
                cmd = keys.out_queue.get()
                if cmd == "b":
                    print("\nRecalibrating baseline. DO NOT TOUCH the eFlesh...")
                    haptic.stop()
                    calibrator.begin()
                    calibrating = True
                elif cmd == "s":
                    haptic.stop()
                elif cmd == "q":
                    running = False

            # ---- drain all pending sensor samples ----
            # encoder.update() is deliberately NOT called here. It is stateful
            # (wall-clock pulse phase + one-shot transient window), so it is
            # called exactly ONCE per send tick below. That single result is
            # authoritative for the motor command, the CSV row, the terminal
            # dashboard and the GUI snapshot alike -- calling update() here as
            # well would produce a value that never reaches the motors, and the
            # screen would stop matching the hardware.
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
                if logger.enabled:
                    # Motor columns are the last command actually sent (at most
                    # 1/HAPTIC_SEND_HZ old), never a recomputed phantom value.
                    logger.log(log_row(time.time(), bx, by, bz, last_sample, last_motors))

            now = time.monotonic()

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
                    haptic.send_motor_command(MotorCommand.from_list(last_motors))
                else:
                    last_motors, last_on_off = [0] * NUM_MOTORS, (0, 0)
                    haptic.stop()

            # ---- dashboard ----
            if now - last_dashboard >= 1.0 / DASHBOARD_HZ:
                last_dashboard = now
                bx, by, bz = last_bxyz
                mode_label = "CALIBRATING..." if calibrating else "RUNNING"
                if last_sample is None:
                    last_sample = TactileSample(0, 0, 0, 0, 0, 0, 0, 0,
                                                 TactileState.NO_CONTACT, 0.0, False, False)
                render_dashboard(sensor.rate_hz, bx, by, bz, last_sample, last_motors,
                                  last_on_off, sensor.connected, haptic.is_connected(), mode_label)

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


def run_sensor_only_mode(args):
    sensor = SensorReader(args.sensor_port)
    cfg = load_or_default_config(args.calibration_file)

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
    running = True
    print("Sensor-only mode: motors will NOT be commanded. Ctrl+C to quit.")
    try:
        while running:
            bx = by = bz = 0.0
            while not sensor.out_queue.empty():
                ts, bx, by, bz = sensor.out_queue.get()
                if calibrating:
                    if calibrator.feed(bx, by, bz):
                        calibrating = False
                    continue
                last_sample = pipeline.process(bx, by, bz)

            now = time.monotonic()
            if now - last_dashboard >= 1.0 / DASHBOARD_HZ:
                last_dashboard = now
                mode_label = "CALIBRATING..." if calibrating else "SENSOR-ONLY"
                if last_sample is None:
                    last_sample = TactileSample(0, 0, 0, 0, 0, 0, 0, 0,
                                                 TactileState.NO_CONTACT, 0.0, False, False)
                render_dashboard(sensor.rate_hz, bx, by, bz, last_sample,
                                  [0] * NUM_MOTORS, (0, 0), sensor.connected, False, mode_label)
            time.sleep(MAIN_LOOP_SLEEP_S)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop()


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
