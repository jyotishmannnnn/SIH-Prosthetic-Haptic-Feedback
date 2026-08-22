#!/usr/bin/env python3
"""
telemetry.py -- the GUI's plumbing: a snapshot formatter plus a tiny
stdlib HTTP server that streams those snapshots to a browser and accepts
a fixed whitelist of commands back.

============================================================
THIS MODULE CONTAINS NO SIGNAL PROCESSING AND NO HAPTIC ENCODING
============================================================

haptic_algorithm.py is the single source of truth for both. Every number
in a snapshot is *copied* from a value the algorithm already computed --
build_snapshot() is a pure formatter over inputs it is handed. It does
not filter, threshold, integrate, or re-derive anything, and it never
touches a serial port. If a number the GUI wants does not exist in the
algorithm's output, the correct fix is to plumb it through, not to
compute it here: two implementations of the same math drift, and then
the screen stops matching the motors.

The two derived values in here are deliberate and are labelling only,
never motor output:

  * intensity_band()      -- picks a human label (LIGHT/MEDIUM/...) by
                             looking up which row of the config's own
                             recruitment_table is currently active, so
                             the label changes on exactly the same
                             threshold that recruits another motor.
  * active_motor_count()  -- calls haptic_algorithm.step_table() against
                             the same recruitment_table the encoder uses.

Transport: Server-Sent Events over the stdlib http.server, chosen over a
WebSocket because it needs no third-party package (pc/requirements.txt
pins only pyserial), reconnects on its own when the engine restarts, and
serves the single-file dashboard from the same origin -- so there is one
process, one port, and nothing to install at a venue with no wifi.

    GET  /          -> gui/index.html   (the dashboard)
    GET  /raw       -> gui/raw.html     (bare JSON dump, data-path check)
    GET  /stream    -> text/event-stream, one JSON snapshot per event
    POST /cmd       -> {"cmd": "<name>", "args": {...}}

Binds to 127.0.0.1 by default: this endpoint can stop and start motors,
so it is not exposed to the network unless you explicitly ask for it.
"""

import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from haptic_algorithm import NUM_MOTORS, TactileState, step_table

DEFAULT_PORT = 8770
DEFAULT_HOST = "127.0.0.1"

# The sensor runs at ~100 Hz. A browser cannot usefully draw that, and a
# 60 Hz display cannot show it, so snapshots are decimated to this rate.
# Nothing downstream of the serial thread is allowed to depend on it.
STREAM_HZ = 30

# Per-client outbound depth. Deliberately tiny: a slow or paused browser
# tab must never become backpressure on the serial loop, so publish()
# drops the oldest frame instead of waiting. Stale telemetry is useless
# telemetry -- a dropped frame is always the right trade here.
QUEUE_DEPTH = 3

# Inbound commands are whitelisted. The engine is the only thing that
# touches a serial port; this is the complete set of things a browser is
# allowed to ask it to do.
ALLOWED_COMMANDS = frozenset({
    "calibrate_baseline",
    "calibrate_max",
    "start_demo",
    "stop_demo",
    "start_log",
    "stop_log",
    "stop_motors",
})

# Five labels for the five rows of the default recruitment_table. See
# intensity_band() -- these are display labels for an already-computed
# intensity, and are never inputs to anything.
BAND_LABELS = ["NO CONTACT", "LIGHT", "MEDIUM", "STRONG", "MAXIMUM"]


def intensity_band(cfg, intensity, in_contact):
    """Human-readable band for an already-computed intensity.

    The boundaries are not invented here: they are the I_at values of
    HapticConfig.recruitment_table, the same thresholds the encoder uses
    to decide how many motors to recruit. So the label flips to STRONG on
    exactly the frame a fourth motor lights up, and recalibrating the
    table moves the label and the motors together.
    """
    if not in_contact:
        return BAND_LABELS[0]

    table = cfg.recruitment_table or []
    idx = 0
    for i, row in enumerate(table):
        if intensity >= row[0]:
            idx = i
        else:
            break

    # In contact but below the first recruitment step (no motor running
    # yet): report the lightest contact label rather than "NO CONTACT",
    # which would contradict the state field shown beside it.
    idx = max(1, min(idx, len(BAND_LABELS) - 1))
    return BAND_LABELS[idx]


def active_motor_count(cfg, intensity):
    """Motor count the encoder would recruit at this intensity -- via the
    algorithm's own step_table() over its own table, not a copy of it."""
    return step_table(cfg.recruitment_table, intensity)


def build_snapshot(*, seq, cfg, sample, motors, on_off, bx, by, bz,
                   mode, calibrating,
                   sensor_ok, sensor_rate_hz,
                   haptic_ok, haptic_latency_ms, transport_name,
                   baseline_progress=1.0,
                   max_calibrated=False, max_capture=None,
                   log_active=False, log_path=None, log_rows=0,
                   demo=None, stream_hz=0.0, events=None, notice=None):
    """Format one frame for the GUI. Pure: every argument is a value the
    caller already had. Nothing is computed that the algorithm did not
    already produce, except the two labelling helpers above."""
    on_ms, off_ms = on_off
    in_contact = sample.state is not TactileState.NO_CONTACT
    intensity = sample.intensity
    ev = events or {}

    return {
        "seq": seq,
        "t": time.time(),
        "mode": mode,
        "calibrating": bool(calibrating),

        # Raw field, straight off the wire, baseline NOT subtracted.
        "b": {"x": bx, "y": by, "z": bz},
        # Baseline-subtracted and EMA-filtered, from TactileSample.
        "d": {"x": sample.dx, "y": sample.dy, "z": sample.dz},

        # NOTE ON "M": the algorithm computes exactly one magnitude, from
        # the *filtered* deltas (haptic_algorithm.TactilePipeline.process,
        # Stage 2). There is no separate unfiltered M anywhere in the
        # pipeline, so none is reported here -- inventing one would mean
        # duplicating the algorithm's math in this file. All of M/N/S/dM
        # are relative deformation signals in raw MLX90393 delta units.
        # They are NOT newtons and must never be labelled as force.
        "features": {
            "M": sample.M,
            "N": sample.N,
            "S": sample.S,
            "dM": sample.dM,
            "hf": sample.hf_energy,
        },

        "i": intensity,
        "band": intensity_band(cfg, intensity, in_contact),
        "state": sample.state.value,
        "events": {
            "onset": bool(ev.get("onset")),
            "release": bool(ev.get("release")),
        },

        # The command that was actually sent to the haptic ESP32 this
        # frame -- not a recomputed value. See run_full_mode's single
        # encoder.update() call.
        "motors": list(motors),
        "active": active_motor_count(cfg, intensity) if in_contact else 0,
        "pattern": {
            "on_ms": on_ms,
            "off_ms": off_ms,
            "continuous": bool(off_ms <= 0 and any(motors)),
        },

        "link": {
            "sensor_ok": bool(sensor_ok),
            "sensor_rate_hz": sensor_rate_hz,
            "haptic_ok": bool(haptic_ok),
            "haptic_latency_ms": haptic_latency_ms,
            "transport": transport_name,
            "stream_hz": stream_hz,
        },

        "cal": {
            "baseline": [cfg.baseline_bx, cfg.baseline_by, cfg.baseline_bz],
            "deadband": cfg.deadband,
            "contact_level": cfg.contact_level,
            "max_level": cfg.max_level,
            "alpha": cfg.alpha,
            "pwm_min": cfg.pwm_min,
            "pwm_max": cfg.pwm_max,
            "gamma": cfg.gamma,
            "baseline_progress": baseline_progress,
            "max_calibrated": bool(max_calibrated),
            "max_capture": max_capture,   # {"progress":f,"peak":f} while running
        },

        "log": {"active": bool(log_active), "path": log_path, "rows": log_rows},
        "demo": demo or {"active": False},

        # Short-lived engine message (calibration result, log path, refusal).
        # The engine decides when it expires; the GUI just displays it.
        "notice": notice,
    }


# ======================================================================
# SSE HUB
# ======================================================================

class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class TelemetryHub:
    """Serves the dashboard, streams snapshots, collects commands.

    Threading contract, which is the whole point of this class:
    publish() is called from the engine's main loop and MUST NOT block.
    It decimates to stream_hz, serialises once, and hands the bytes to
    each client's bounded queue, dropping the oldest frame when a client
    is not keeping up. A wedged browser tab, a paused debugger, or a
    client that vanished without closing its socket therefore cannot
    stall the serial read loop or delay a motor command.
    """

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT,
                 gui_dir=None, stream_hz=STREAM_HZ):
        self.host = host
        self.port = port
        self.gui_dir = gui_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), os.pardir, "gui")
        self._min_dt = 1.0 / float(stream_hz) if stream_hz > 0 else 0.0

        self._clients = []
        self._lock = threading.Lock()
        self._commands = queue.Queue()

        # perf_counter, NOT monotonic: on Windows time.monotonic() has a
        # ~15.6 ms resolution, which quantizes a 33 ms decimation gate into
        # alternating 31/47 ms gaps and lands the stream at ~22 Hz instead
        # of 30. perf_counter is sub-microsecond on every platform we run on.
        self._last_push = 0.0
        self._pushed = 0
        self._rate_t = time.perf_counter()
        self._rate = 0.0

        self._server = None
        self._thread = None

    # ---- lifecycle ----

    def start(self):
        self._server = _Server((self.host, self.port), _make_handler(self))
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        name="telemetry-http", daemon=True)
        self._thread.start()
        return self.url

    @property
    def url(self):
        return f"http://{self.host}:{self.port}/"

    def stop(self):
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None

    # ---- outbound ----

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)

    @property
    def stream_hz(self):
        return self._rate

    def publish(self, snapshot):
        """Non-blocking. Safe to call every frame; decimates internally."""
        now = time.perf_counter()
        if self._min_dt and (now - self._last_push) < self._min_dt:
            return
        self._last_push = now

        self._pushed += 1
        if now - self._rate_t >= 1.0:
            self._rate = self._pushed / (now - self._rate_t)
            self._pushed = 0
            self._rate_t = now

        with self._lock:
            clients = list(self._clients)
        if not clients:
            return

        try:
            body = json.dumps(snapshot, separators=(",", ":"), allow_nan=False)
        except (ValueError, TypeError):
            return  # never let a bad frame take down the engine
        payload = ("data: " + body + "\n\n").encode("utf-8")

        for q in clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                # Drop the oldest frame and take the newest. Never wait.
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(payload)
                except queue.Full:
                    pass

    # ---- inbound ----

    def drain_commands(self):
        """Returns [(name, args_dict), ...] received since the last call."""
        out = []
        while True:
            try:
                out.append(self._commands.get_nowait())
            except queue.Empty:
                return out

    # ---- client registry (used by the request handler) ----

    def _add_client(self, q):
        with self._lock:
            self._clients.append(q)

    def _remove_client(self, q):
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)


def _make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "eflesh-telemetry/1"

        # The terminal dashboard owns stdout and repaints with a cursor-home
        # escape. Letting http.server log request lines there would shred it,
        # so access logging is off entirely.
        def log_message(self, *args, **kwargs):
            pass

        def do_GET(self):
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            if path == "/":
                self._send_file("index.html")
            elif path == "/raw":
                self._send_file("raw.html")
            elif path == "/stream":
                self._stream()
            elif path == "/favicon.ico":
                self._send_status(204)
            else:
                self._send_text(404, "not found")

        def do_POST(self):
            if self.path.split("?", 1)[0].rstrip("/") != "/cmd":
                self._send_text(404, "not found")
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = 0
            raw = self.rfile.read(n) if n > 0 else b"{}"
            try:
                body = json.loads(raw.decode("utf-8") or "{}")
            except (ValueError, UnicodeDecodeError):
                self._send_text(400, "bad json")
                return
            if not isinstance(body, dict):
                self._send_text(400, "bad body")
                return
            name = str(body.get("cmd", "")).strip()
            if name not in ALLOWED_COMMANDS:
                self._send_text(400, "unknown command")
                return
            args = body.get("args")
            hub._commands.put((name, args if isinstance(args, dict) else {}))
            self._send_text(202, "accepted")

        # ---- SSE ----

        def _stream(self):
            q = queue.Queue(maxsize=QUEUE_DEPTH)
            hub._add_client(q)
            try:
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Accel-Buffering", "no")
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(b": connected\n\n")
                self.wfile.flush()
                while True:
                    try:
                        payload = q.get(timeout=1.0)
                    except queue.Empty:
                        # Keepalive comment. Also how a client that went
                        # away without closing cleanly gets noticed: the
                        # write raises and we unregister below.
                        payload = b": ping\n\n"
                    self.wfile.write(payload)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError, OSError, ValueError):
                pass
            finally:
                hub._remove_client(q)

        # ---- helpers ----

        def _send_file(self, name):
            path = os.path.normpath(os.path.join(hub.gui_dir, name))
            try:
                with open(path, "rb") as f:
                    data = f.read()
            except OSError:
                self._send_text(404, f"missing {name} (looked in {hub.gui_dir})")
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _send_text(self, code, msg):
            data = msg.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _send_status(self, code):
            self.send_response(code)
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler
