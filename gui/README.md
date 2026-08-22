# GUI

**Status: built.** A browser dashboard for the eFlesh/MLX90393 pipeline,
served by `pc/haptic_engine.py` itself.

```
python pc/haptic_engine.py --sensor-port COM7 --haptic-port COM8 --gui
```

Then open <http://127.0.0.1:8770/>. The terminal dashboard keeps running
in parallel — it is deliberately the fallback if the browser dies
mid-demo, so nothing was moved out of it.

| File | What it is |
|---|---|
| `index.html` | The dashboard. Single file, no CDN, no web fonts, no external assets. |
| `raw.html` | Unstyled JSON dump at `/raw`. The data-path check. |
| `../pc/telemetry.py` | Snapshot formatter + the SSE server thread. |
| `../pc/test_telemetry.py` | Offline verification of the hub, no hardware needed. |
| `../pc/test_engine_integration.py` | Drives `run_full_mode` with stub serial endpoints, no hardware needed. |

## The rule this GUI is built around

**The GUI computes nothing about the tactile signal or the haptic
encoding.** `pc/haptic_algorithm.py` is the single source of truth for
both. Every number on screen is copied from a value the algorithm already
produced.

This is not stylistic. The motor array renders `snapshot.motors`, which is
the exact `M,<m0>..<m5>` command sent to the haptic ESP32 on that frame.
If the GUI recomputed intensity or motor states from raw field values, the
two implementations would drift and the screen would stop matching what
the motors actually do — the one failure mode that would wreck a demo,
because you would be narrating a number nobody is feeling.

Likewise the GUI never touches a serial port, a GPIO, or a PWM register.
It POSTs a command name; the engine remains the only thing that talks to
hardware, and the haptic ESP32 still owns motor output.

Two values are derived browser-side, and both are labels only, never
motor output:

- the band label (NO CONTACT / LIGHT / MEDIUM / STRONG / MAXIMUM)
- the band tick marks on the intensity bar

Both come from `HapticConfig.recruitment_table`, shipped in each snapshot.
That table already defines 1/2/4/6 motors at I ≥ .05/.25/.5/.75, which is
five steps for five labels, so **the label changes on exactly the frame
another motor is recruited.** Recalibrate the table and the label, the
ticks and the motors all move together. Nothing is hardcoded in the
JavaScript.

One deliberate exception: between `contact_level` and the first
recruitment step, contact is latched but no motor runs yet. That sliver
reads LIGHT rather than NO CONTACT, which would contradict the state field
beside it. The motor array is the authority on what is actually running,
and there it correctly shows all six dark.

## Transport

Server-Sent Events over the stdlib `http.server`, not a WebSocket.
`pc/requirements.txt` pins only `pyserial` and a venue with no wifi is no
place to discover a missing package, so: no third-party dependency, no
hand-rolled RFC 6455 framing, `EventSource` reconnects on its own when the
engine restarts, and the dashboard is served from the same origin as its
data. One process, one port.

```
GET  /        -> index.html
GET  /raw     -> raw.html
GET  /stream  -> text/event-stream, one JSON snapshot per event, ~30 Hz
POST /cmd     -> {"cmd": "<name>"}
```

Wire format is documented in `../docs/serial-protocol.md`.

Commands are whitelisted in `telemetry.ALLOWED_COMMANDS`:
`calibrate_baseline`, `calibrate_max`, `start_demo`, `stop_demo`,
`start_log`, `stop_log`, `stop_motors`. Anything else gets a 400.

**Binds to `127.0.0.1` by default.** This endpoint can start and stop
motors. `--gui-host 0.0.0.0` makes it reachable from other machines, which
is occasionally useful (a tablet as a second screen) and should be a
conscious choice.

### Why the serial loop can't be stalled by the browser

`TelemetryHub.publish()` is called from the engine's main loop and never
blocks. It decimates ~100 Hz frames to 30 Hz, serialises once, and pushes
into per-client bounded queues (depth 3) that **drop the oldest frame**
when a client isn't keeping up. A wedged tab, a paused debugger, or a
client that vanished without closing its socket cannot delay a motor
command. Stale telemetry is useless telemetry — dropping is always the
right trade here.

## Order to debug in

1. `python pc/test_telemetry.py` — passes with no hardware attached. If
   this fails, the problem is in the hub, not your wiring.
2. `python pc/test_engine_integration.py` — drives the real
   `run_full_mode` against stub serial endpoints (~35 s). Its test `[B]`
   asserts the invariant that matters most here: `snapshot.motors` equals
   the last command the transport actually received. If that ever fails,
   the dashboard is lying about the hardware — fix it before demoing.
3. `/raw` — press the patch. If `M` and `I` move, the whole chain works
   and anything still wrong is presentation.
4. `/` — the dashboard.

`raw.html` is kept in the repo after the fact precisely so that when the
dashboard looks wrong you can answer "is it the data or the drawing?"
in one click.

## Calibration, in the GUI

Two steps, both required, both writing to the same `calibration.json`
`HapticConfig` already uses:

1. **Baseline** — 100 untouched samples, median per axis. Per-run; it
   drifts.
2. **Maximum** — hold the hardest press the demo will use for 4 s; the
   peak `M` becomes `max_level`, which is what makes `I = 1.0` mean
   anything.

Without step 2 there is no calibrated top to the scale, so the dashboard
shows **"MAXIMUM NOT CALIBRATED — I is undefined"** and names the
placeholder it is currently dividing by, rather than showing a confident
percentage against a number that means nothing for your patch. Step 2
refuses to save a peak at or below `contact_level`.

Neither step ever displays a newton. There is no force sensor in this
pipeline. See `../docs/calibration.md`.

## Demo mode

A scripted walk: NO CONTACT → LIGHT → MEDIUM → STRONG → MAXIMUM →
release, badged `DEMO MODE` on both the dashboard and the terminal for as
long as it runs.

It synthesizes **sensor input** — Bx/By/Bz — and pushes it through the
real `TactilePipeline`. The deadband, EMA filter, feature extraction,
state machine, intensity normalization and haptic encoder running in demo
mode are the ones running live, and the motors are really driven. No
pipeline stage is reimplemented and no motor value is invented. Real
sensor samples are discarded for the duration so live and scripted input
can never mix.

**Live mode never synthesizes anything.**

## Logging

The GUI's log buttons drive the engine's existing `DemoLogger` — the
schema is still `haptic_algorithm.LOG_COLUMNS` and there is deliberately
no second writer. Motor columns are the last command actually sent.

## Transport selection (still CLI-only)

`pc/transports/` exposes `get_transport("usb"|"wifi"|"ble"|"espnow", ...)`.
A transport picker in the GUI is **not built** — `--transport <name>`
remains the selector. Only USB is implemented in firmware today
(`../docs/transport-options.md`), so a picker would mostly offer choices
that cannot work yet.

## Legacy GUI (different project)

`legacy-alpaca-fsr-glove/` is a complete, working embedded browser GUI for
a **separate, earlier prototype** — the ALPACA FSR-glove demo, WiFi-based,
different hardware and pin layout. It is not part of this pipeline. This
dashboard borrows its visual language deliberately (dark panels, thin
borders, cyan accent, tabular figures) so the two read as siblings, minus
the gradients and blur. See `legacy-alpaca-fsr-glove/README.md`.
