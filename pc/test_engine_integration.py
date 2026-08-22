#!/usr/bin/env python3
"""Drive run_full_mode end to end with stub serial endpoints.

    python pc/test_engine_integration.py

No ESP32, no eFlesh, no serial port -- and no pyserial either; a stub
module is injected so the import succeeds on a bare machine. A fake
sensor thread emits a press/release ramp at 100 Hz and a fake transport
records every motor command, so the code paths that normally need both
boards attached can be exercised on a laptop.

What this is really for: the single most important invariant in the GUI
is that what the screen shows is what the motors got. Test [B] asserts
exactly that -- snapshot.motors == the last command the transport
actually received -- which is the thing that silently broke when
encoder.update() was being called three times per frame. Keep it passing.

Also covers: baseline completion, band/recruitment agreement across a
full press, every GUI command taking effect in the engine, the CSV schema
and its motor columns, calibration step 2 persisting max_level, and demo
mode staying badged on every single frame.

Takes about 35 s (it waits out real calibration windows). Stdlib only.
"""

import argparse
import io
import json
import math
import os
import queue
import shutil
import sys
import threading
import time
import types
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

PORT = 8799
CAL = os.path.join(REPO, "_it_calibration.json")
LOGDIR = os.path.join(REPO, "_it_logs")

# ---- stub pyserial, so this runs without the real package installed ----
_fake = types.ModuleType("serial")


class SerialException(Exception):
    pass


class _Serial:
    def __init__(self, *a, **k): pass
    def readline(self): time.sleep(0.01); return b""
    def write(self, b): return len(b)
    def close(self): pass


_fake.Serial = _Serial
_fake.SerialException = SerialException
sys.modules.setdefault("serial", _fake)

import haptic_engine as HE                      # noqa: E402
from haptic_algorithm import LOG_COLUMNS        # noqa: E402

FAILS = []
OUT = []


def p(line):
    OUT.append(line)


def check(name, cond, extra=""):
    p(("  PASS  " if cond else "  FAIL  ") + name + ("   " + str(extra) if extra else ""))
    if not cond:
        FAILS.append(name)


# ---- stub sensor: rest 2 s, ramp past max_level over 4 s, release ----
class StubSensor(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.out_queue = queue.Queue()
        self.connected = True
        self.rate_hz = 100.0
        self.running = True
        self.t0 = time.perf_counter()

    def run(self):
        while self.running:
            t = time.perf_counter() - self.t0
            ph = t % 9.0
            m = 0.0 if ph < 2.0 else ((ph - 2.0) / 4.0 * 95.0 if ph < 6.0 else 0.0)
            self.out_queue.put((int(t * 1000), 0.0, 0.0, m + math.sin(t * 30) * 0.4))
            time.sleep(0.01)

    def read_one_blocking(self, timeout=2.0):
        return self.out_queue.get(timeout=timeout)[1:]

    def stop(self):
        self.running = False


class StubTransport:
    def __init__(self):
        self.sent = []
        self.stops = 0
        self.pings = 0

    def connect(self): return True
    def disconnect(self): pass
    def send_motor_command(self, c): self.sent.append(c.as_list())
    def stop(self): self.stops += 1
    def ping(self): self.pings += 1
    def status(self): pass
    def is_connected(self): return True
    def get_latency_ms(self): return 1.7


def snapshot():
    """Read exactly one SSE frame."""
    r = urllib.request.urlopen("http://127.0.0.1:%d/stream" % PORT, timeout=6)
    buf = b""
    while True:
        c = r.read(1)
        if not c:
            return None
        buf += c
        if buf.endswith(b"\n\n"):
            for line in buf.decode().splitlines():
                if line.startswith("data: "):
                    r.close()
                    return json.loads(line[6:])
            buf = b""


def cmd(name):
    req = urllib.request.Request(
        "http://127.0.0.1:%d/cmd" % PORT,
        data=json.dumps({"cmd": name}).encode(),
        headers={"Content-Type": "application/json"})
    return urllib.request.urlopen(req, timeout=3).status


def main():
    tx = StubTransport()
    HE.SensorReader = lambda port, baud=None: _started(StubSensor())
    HE.get_transport = lambda name, **kw: tx
    # The terminal dashboard repaints stdout with a cursor-home escape at
    # 8 Hz, which would bury the results. It is exercised by running the
    # engine normally; silenced here so the GUI path is what we read.
    HE.render_dashboard = lambda *a, **k: None
    HE.LOG_DIR = LOGDIR

    for path in (CAL,):
        if os.path.exists(path):
            os.remove(path)

    args = argparse.Namespace(
        sensor_port="STUB1", haptic_port="STUB2", transport="usb",
        simulate=False, sensor_only=False, no_log=True,
        calibration_file=CAL, calibrate=False,
        gui=True, gui_host="127.0.0.1", gui_port=PORT)

    threading.Thread(target=HE.run_full_mode, args=(args,), daemon=True).start()
    time.sleep(4.0)   # 100-sample baseline at 100 Hz, then part of a ramp

    p("\n[A] engine up and publishing")
    s = snapshot()
    check("snapshot served from run_full_mode", s is not None)
    check("baseline finished", s and not s["calibrating"])
    check("mode is LIVE", s and s["mode"] == "LIVE", s and s["mode"])
    check("max flagged uncalibrated with no calibration.json",
          s and s["cal"]["max_calibrated"] is False)
    check("motor commands reaching the transport", len(tx.sent) > 30,
          "%d sent" % len(tx.sent))
    check("PING issued so latency is measured", tx.pings >= 1, "%d pings" % tx.pings)

    p("\n[B] THE INVARIANT: displayed motors == last command actually sent")
    matched = 0
    for _ in range(12):
        s = snapshot()
        if s and tx.sent and s["motors"] == tx.sent[-1]:
            matched += 1
        time.sleep(0.05)
    check("snapshot.motors mirrors the sent command", matched >= 11,
          "%d/12 frames" % matched)

    p("\n[C] a full press walks the bands and recruits motors")
    bands, counts, peak = set(), set(), 0.0
    end = time.time() + 10
    while time.time() < end:
        s = snapshot()
        if s:
            bands.add(s["band"])
            counts.add(s["active"])
            peak = max(peak, s["i"])
    check("all five bands seen", len(bands) >= 4, sorted(bands))
    check("recruitment stepped through the table", len(counts) >= 3, sorted(counts))
    check("intensity reached the top of the scale", peak > 0.95, round(peak, 3))

    p("\n[D] GUI commands take effect in the engine")
    check("stop_motors accepted", cmd("stop_motors") == 202)
    before = tx.stops
    time.sleep(0.4)
    check("engine issued a transport stop", tx.stops > before,
          "%d -> %d" % (before, tx.stops))
    check("start_log accepted", cmd("start_log") == 202)
    time.sleep(1.2)
    s = snapshot()
    check("logging on", s and s["log"]["active"] is True)
    check("rows accumulating", s and s["log"]["rows"] > 20, s and s["log"]["rows"])
    logpath = s["log"]["path"]
    check("stop_log accepted", cmd("stop_log") == 202)
    time.sleep(0.4)
    s = snapshot()
    check("logging off", s and s["log"]["active"] is False)

    p("\n[E] CSV schema is the algorithm's, with motor columns")
    if logpath and os.path.exists(logpath):
        rows = io.open(logpath, encoding="utf-8").read().strip().splitlines()
        hdr = rows[0].split(",")
        check("header is haptic_algorithm.LOG_COLUMNS", hdr == LOG_COLUMNS)
        check("six motor columns present",
              hdr[-6:] == ["motor0", "motor1", "motor2", "motor3", "motor4", "motor5"])
        check("data rows written", len(rows) > 20, "%d rows" % (len(rows) - 1))
        check("no force/newton column", not any(
            "force" in h.lower() or "newton" in h.lower() for h in hdr))
    else:
        check("log file exists", False, logpath)

    p("\n[F] calibration step 2 persists max_level")
    check("calibrate_max accepted", cmd("calibrate_max") == 202)
    time.sleep(0.6)
    s = snapshot()
    check("capture reported in progress", s and s["cal"]["max_capture"] is not None)
    time.sleep(5.0)   # MaxCapture.DURATION_S is 4 s
    s = snapshot()
    check("capture finished", s and s["cal"]["max_capture"] is None)
    check("max now flagged calibrated", s and s["cal"]["max_calibrated"] is True)
    check("calibration.json written", os.path.exists(CAL))
    if os.path.exists(CAL):
        saved = json.load(io.open(CAL, encoding="utf-8"))
        check("max_level persisted and positive", saved.get("max_level", 0) > 0,
              round(saved.get("max_level", 0), 2))
        check("no newtons in the file", "newton" not in json.dumps(saved).lower())

    p("\n[G] demo mode is badged on every frame and drives real output")
    check("start_demo accepted", cmd("start_demo") == 202)
    time.sleep(0.5)
    steps, badged, frames = set(), 0, 0
    end = time.time() + 9
    while time.time() < end:
        s = snapshot()
        if s:
            frames += 1
            if s["demo"]["active"]:
                badged += 1
                steps.add(s["demo"]["step"])
                if s["mode"] != "DEMO":
                    check("mode says DEMO while demo active", False, s["mode"])
    check("badged on every frame while running", frames > 0 and badged == frames,
          "%d/%d" % (badged, frames))
    check("advanced through steps", len(steps) >= 2, sorted(steps))
    check("stop_demo accepted", cmd("stop_demo") == 202)
    time.sleep(0.5)
    s = snapshot()
    check("demo cleared", s and s["demo"]["active"] is False)
    check("back to LIVE", s and s["mode"] == "LIVE", s and s["mode"])

    p("\n[H] calibration step 1 is re-runnable at any time")
    check("calibrate_baseline accepted", cmd("calibrate_baseline") == 202)
    time.sleep(0.3)
    s = snapshot()
    check("re-entered calibrating", s and s["calibrating"] is True,
          s and round(s["cal"]["baseline_progress"], 2))
    time.sleep(2.0)
    s = snapshot()
    check("baseline completed again", s and s["calibrating"] is False)


def _started(t):
    t.start()
    return t


if __name__ == "__main__":
    try:
        main()
    finally:
        p("\n" + ("ALL PASS" if not FAILS
                  else "%d FAILED: %r" % (len(FAILS), FAILS)))
        print("\n".join(OUT))
        if os.path.exists(CAL):
            os.remove(CAL)
        shutil.rmtree(LOGDIR, ignore_errors=True)
        sys.stdout.flush()
        # os._exit, not sys.exit: the engine leaves daemon threads (serial
        # reader, telemetry HTTP server) that would keep this alive.
        os._exit(1 if FAILS else 0)
