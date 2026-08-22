#!/usr/bin/env python3
"""Offline check for telemetry.py -- no ESP32, no eFlesh, no serial port.

    python pc/test_telemetry.py

Verifies the whole browser-facing path: the server starts, /stream
delivers snapshots, /cmd whitelists and enqueues commands, band labels
line up with the recruitment table, the stream really is decimated, and
publish() never blocks on a client that stopped reading.

Run this before blaming the hardware. If this passes and the dashboard
is still empty, the problem is between the engine and the hub, not in
the hub -- check gui/raw.html next.

Plain stdlib asserts and prints, deliberately: the repo pins no test
framework and a demo-day check should not need one installed.
"""
import json
import os
import sys
import threading
import time
import urllib.request
import urllib.error

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import telemetry
from haptic_algorithm import (HapticConfig, TactilePipeline, HapticEncoder,
                              step_table)

fails = []


def check(name, cond, extra=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + ("   " + str(extra) if extra else ""))
    if not cond:
        fails.append(name)


cfg = HapticConfig()

print("\n[1] intensity_band vs recruitment_table")
# Default table: 1 motor at .05, 2 at .25, 4 at .5, 6 at .75
cases = [
    (0.00, False, "NO CONTACT"),
    (0.01, True,  "LIGHT"),     # in contact, below first recruitment step
    (0.10, True,  "LIGHT"),
    (0.30, True,  "MEDIUM"),
    (0.60, True,  "STRONG"),
    (0.95, True,  "MAXIMUM"),
    (1.00, True,  "MAXIMUM"),
]
for i, contact, expect in cases:
    got = telemetry.intensity_band(cfg, i, contact)
    check(f"I={i:.2f} contact={contact!s:5} -> {got}", got == expect, f"expected {expect}")

print("\n[2] band boundary == motor recruitment boundary")
# The whole point of deriving bands from the table: the label must change
# on the same threshold that recruits another motor.
#
# Exception, by design: the FIRST recruitment step (I=0.05, 0->1 motors).
# Between contact_level and that step, contact is latched but nothing is
# recruited yet. intensity_band() clamps that sliver to LIGHT rather than
# reporting "NO CONTACT", which would contradict the state field displayed
# right beside it. The motor array is the authority on what is actually
# running, and in that window it correctly shows all six dark.
first_step = min(r[0] for r in cfg.recruitment_table if r[0] > 0)
for thresh in [r[0] for r in cfg.recruitment_table if r[0] > first_step]:
    lo = telemetry.intensity_band(cfg, thresh - 1e-6, True)
    hi = telemetry.intensity_band(cfg, thresh + 1e-6, True)
    n_lo = step_table(cfg.recruitment_table, thresh - 1e-6)
    n_hi = step_table(cfg.recruitment_table, thresh + 1e-6)
    check(f"at I={thresh}: band {lo}->{hi} and motors {n_lo}->{n_hi} change together",
          (lo != hi) and (n_lo != n_hi))

lo = telemetry.intensity_band(cfg, first_step - 1e-6, True)
check(f"sub-recruitment sliver (I<{first_step}, in contact) reads LIGHT, not NO CONTACT",
      lo == "LIGHT", lo)

print("\n[3] snapshot builds and is JSON-serialisable")
pipeline = TactilePipeline(cfg)
encoder = HapticEncoder(cfg)
sample = pipeline.process(10, 5, 60)
motors, on_off = encoder.update(sample)
snap = telemetry.build_snapshot(
    seq=1, cfg=cfg, sample=sample, motors=motors, on_off=on_off,
    bx=10, by=5, bz=60, mode="LIVE", calibrating=False,
    sensor_ok=True, sensor_rate_hz=99.4, haptic_ok=True,
    haptic_latency_ms=1.8, transport_name="usb", stream_hz=30.0)
body = json.dumps(snap, allow_nan=False)
check("serialises without NaN/Inf", len(body) > 100, f"{len(body)} bytes")
check("motors mirrors the encoder result", snap["motors"] == list(motors))
check("no 'force' or 'newton' anywhere in the payload",
      "force" not in body.lower() and "newton" not in body.lower())
check("features present", set(snap["features"]) == {"M", "N", "S", "dM", "hf"})

print("\n[4] HTTP server: /stream, /cmd, whitelist, 404s")
hub = telemetry.TelemetryHub(port=8791)
hub.start()
time.sleep(0.3)

frames = []
err = []


def reader():
    try:
        r = urllib.request.urlopen("http://127.0.0.1:8791/stream", timeout=6)
        buf = b""
        while len(frames) < 3:
            chunk = r.read(1)
            if not chunk:
                break
            buf += chunk
            if buf.endswith(b"\n\n"):
                for line in buf.decode().splitlines():
                    if line.startswith("data: "):
                        frames.append(json.loads(line[6:]))
                buf = b""
    except Exception as e:
        err.append(repr(e))


t = threading.Thread(target=reader, daemon=True)
t.start()
time.sleep(0.5)
check("client registered", hub.client_count == 1, f"count={hub.client_count}")

deadline = time.time() + 5
n = 0
while len(frames) < 3 and time.time() < deadline:
    n += 1
    snap["seq"] = n
    hub.publish(snap)
    time.sleep(0.005)
t.join(timeout=2)
check("received >=3 snapshots over SSE", len(frames) >= 3, f"got {len(frames)} err={err}")
if frames:
    check("frame round-trips intact", frames[0]["motors"] == list(motors))

req = urllib.request.Request("http://127.0.0.1:8791/cmd",
                             data=json.dumps({"cmd": "stop_motors"}).encode(),
                             headers={"Content-Type": "application/json"})
code = urllib.request.urlopen(req, timeout=3).status
check("POST /cmd stop_motors accepted", code == 202, f"HTTP {code}")

bad = urllib.request.Request("http://127.0.0.1:8791/cmd",
                             data=json.dumps({"cmd": "rm -rf"}).encode())
try:
    urllib.request.urlopen(bad, timeout=3)
    check("non-whitelisted command rejected", False, "was accepted!")
except urllib.error.HTTPError as e:
    check("non-whitelisted command rejected", e.code == 400, f"HTTP {e.code}")

cmds = hub.drain_commands()
check("engine drains exactly the whitelisted command",
      cmds == [("stop_motors", {})], cmds)

try:
    urllib.request.urlopen("http://127.0.0.1:8791/nope", timeout=3)
    check("unknown path 404s", False)
except urllib.error.HTTPError as e:
    check("unknown path 404s", e.code == 404, f"HTTP {e.code}")

print("\n[5] decimation: a 100+ Hz caller becomes a ~30 Hz stream")
import queue as _q
sink = _q.Queue(maxsize=10000)
hub._add_client(sink)
calls = 0
t0 = time.perf_counter()
while time.perf_counter() - t0 < 1.0:
    hub.publish(snap)          # called as fast as the loop can go
    calls += 1
    # NB: no time.sleep() here. On Windows + Python < 3.11, sleep(0.001)
    # actually sleeps ~15.6 ms, which would throttle this test harness to
    # ~65 Hz and make it look like publish() was under-delivering.
got = sink.qsize()
hub._remove_client(sink)
check(f"{calls} publish calls in 1.0 s -> {got} frames on the wire",
      abs(got - telemetry.STREAM_HZ) <= 3 and calls > got * 3,
      f"target {telemetry.STREAM_HZ} Hz")

print("\n[6] publish() never blocks even with a stalled client")
stalled = __import__("queue").Queue(maxsize=telemetry.QUEUE_DEPTH)
hub._add_client(stalled)
t0 = time.perf_counter()
for i in range(400):
    hub._last_push = 0.0          # defeat decimation, force every frame through
    hub.publish(snap)
dt = time.perf_counter() - t0
check("400 publishes to a client that never reads", dt < 1.0, f"{dt*1000:.0f} ms total")
check("stalled client queue stayed bounded", stalled.qsize() <= telemetry.QUEUE_DEPTH,
      f"qsize={stalled.qsize()}")

hub.stop()
time.sleep(0.2)

print("\n" + ("ALL PASS" if not fails else f"{len(fails)} FAILED: {fails}"))
sys.exit(1 if fails else 0)
