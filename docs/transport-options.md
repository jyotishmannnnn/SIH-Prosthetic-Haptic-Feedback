# Haptic transport options

Comparison of the four PC -> Haptic ESP32 communication methods
implemented or evaluated in `pc/transports/`. See
`docs/wireless-setup.md` for setup instructions on the ones that are
actually runnable, and `docs/architecture.md` for the per-transport data
flow diagrams.

**No fabricated results.** Only USB has been run against real hardware
in this project. Everything else is marked `NOT TESTED` and stays marked
that way until it's actually been measured — do not treat the numbers in
this doc as measured unless the row says so explicitly.

## Comparison

| | USB Serial | Wi-Fi (UDP) | Bluetooth LE | ESP-NOW |
|---|---|---|---|---|
| **Status** | Working, tested | Implemented, NOT TESTED | Implemented, NOT TESTED | Architectural stub, not implemented |
| **Latency** | Measured: see below | NOT TESTED | NOT TESTED | NOT TESTED |
| **Reliability** | High — physical cable, no interference | Depends on RF environment; UDP has no delivery guarantee (by design, see below) | Depends on RF environment, range ~10m typical for BLE | Reported very low latency in Espressif's own docs, unverified here |
| **Setup complexity** | Lowest — plug in, select COM port | Medium — needs ESP32 in AP or STA mode, IP/port config | Medium — needs pairing/scanning by device name, `bleak` dependency | High — needs new gateway firmware on Sensor ESP32 (not built) |
| **PC compatibility** | Universal (any OS with a serial driver) | Universal (stdlib `socket`) | Needs `bleak` (Windows/macOS/Linux supported) | **None** — PC cannot speak ESP-NOW at all |
| **ESP32-S3 compatibility** | Native, no library needed beyond Arduino core | Native WiFi (AP or STA) | BLE only — **no Bluetooth Classic on ESP32-S3** (see `docs/wireless-setup.md`) | Native, part of the WiFi driver, no separate radio |
| **Internet required** | No | No | No | No |
| **Router required** | No | No (ESP32 can run its own SoftAP) | No | No (no router involved at all — that's the point of ESP-NOW) |
| **Packet loss** | Effectively none (wired) | NOT TESTED — UDP does not retransmit by design | NOT TESTED | NOT TESTED |
| **Reconnect behavior** | Reopen serial port | Re-send PING, wait for PONG (implemented, untested) | Re-scan by device name, reconnect (implemented, untested) | N/A (stub) |
| **Power implications** | None beyond USB (already powering the board) | Wi-Fi radio draws meaningfully more current than idle — matters if the Haptic ESP32 is ever battery-powered | Lower power than Wi-Fi, still more than wired-only | Lower power than a full Wi-Fi connection (no AP association overhead) |
| **Wearer mobility** | Tethered by USB cable | Untethered | Untethered | Untethered (but PC still needs a wired gateway) |
| **Suitability for SIH demo** | **Best** — zero unknowns, already works | Usable as a backup if tested and validated before demo day | Usable as a backup if tested and validated before demo day | Not usable as-is — would need the gateway firmware built and tested first |

## Measured latency — USB

The existing `pc/haptic_engine.py` pushes `M,` commands to the Haptic
ESP32 at a fixed 50 Hz (`HAPTIC_SEND_HZ`). This is a **send rate**, not a
measured round-trip latency — no round-trip latency benchmark has been
run yet (see `pc/transports/benchmark.py` for the tool to do this; it
has not been executed against real hardware in this environment because
no ESP32 hardware is attached to the environment this change was made
in). Mark USB round-trip latency as **NOT TESTED** until
`benchmark.py` has actually been run and this line is updated with real
numbers.

## Why UDP for Wi-Fi, not TCP

Motor commands are a continuously-refreshed **state**, not a queue of
discrete events that each matter individually — `haptic_engine.py`
already re-sends the current PWM values ~50 times per second regardless
of whether they changed. If one Wi-Fi datagram is lost, the next one
~20ms later completely supersedes it; there is nothing to retransmit.
TCP's guaranteed-in-order delivery buys nothing here, and a lost/delayed
TCP segment would head-of-line-block every command behind it, adding
latency UDP simply doesn't have. UDP also avoids TCP's per-connection
state and handshake overhead on a resource-constrained microcontroller.
The accepted tradeoff: no delivery guarantee at all — which is exactly
why the Haptic ESP32's 500ms watchdog must remain in force regardless of
transport (a lost UDP datagram must look no different to the firmware
than a lost USB byte: motors go to a safe state if commands stop
arriving).

## Why ESP-NOW is not simply "the best option"

ESP-NOW gives fast, connectionless ESP32-to-ESP32 communication with no
router — genuinely attractive for a Haptic ESP32 that needs to be
untethered. But the PC has no way to speak ESP-NOW directly (it's an
Espressif-proprietary protocol implemented in the ESP32 WiFi
driver/radio, not a general network protocol any OS or WiFi adapter
supports). The only architecture that works is routing through the
Sensor ESP32 as a USB<->ESP-NOW gateway:

```
PC --USB--> Sensor ESP32 --ESP-NOW--> Haptic ESP32 --> motors
```

This requires the Sensor ESP32 to do three things at once: read the
MLX90393 at ~100Hz, service USB serial to the PC, and now also relay
motor commands over ESP-NOW. Whether it can sustain all three
concurrently at the target rate **has not been evaluated with real code
or hardware** — see "Sensor ESP32 concurrent-load evaluation" below.
Building and testing that gateway firmware is explicitly out of scope
for this task (task instructions: don't modify the sensor firmware's
core eFlesh reading functionality). Until that gateway exists and is
measured, ESP-NOW is a documented future option, not a demo-ready one.

## Sensor ESP32 concurrent-load evaluation (ESP-NOW gateway feasibility)

Not measured — no gateway firmware exists to measure. What can be said
without new measurement, from the existing, tested sensor firmware
(`firmware/sensor/eFlesh_sensor_v1/eFlesh_sensor_v1.ino`) and public
ESP32-S3 characteristics:

- The current sensor loop is I2C-read-bound at ~100Hz and already has
  serial-write headroom (each `S,...` line is ~30 bytes at 115200 baud,
  a small fraction of available bandwidth).
- ESP-NOW transmission is asynchronous and typically sub-millisecond on
  ESP32 hardware per Espressif's documentation, so in principle it could
  be added without disturbing the 100Hz I2C loop timing — but "in
  principle" is not a measurement, and WiFi-driver-level ESP-NOW
  activity is known to be able to introduce jitter into tightly-timed
  loops on some ESP32 workloads. This needs to be measured on the actual
  gateway firmware once built, not assumed.
- Recommendation if this is pursued later: prototype the ESP-NOW relay
  in isolation first (no MLX90393 involved), measure any impact on a
  dummy 100Hz loop, then integrate.

## Recommendation

- **Primary (demo): USB.** It's the only transport that's actually been
  run against real hardware, and the existing watchdog/failsafe design
  around it is proven.
- **Wireless backup: none yet, pending testing.** Neither Wi-Fi nor BLE
  has been validated against real Haptic ESP32 firmware in this
  environment — that firmware doesn't exist yet (see
  `docs/wireless-setup.md`). Do not select a wireless "backup" for the
  actual SIH demo until one has been built, flashed, and benchmarked.
- **ESP-NOW: future work, not demo-ready.** Promising on paper for
  Haptic-ESP32 untethering, but requires new gateway firmware on the
  Sensor ESP32 that doesn't exist and hasn't been load-tested.
- **Demo-day fallback plan: USB, always.** If a wireless transport is
  tested and works reliably before the demo, it can be offered as a
  visual "look, it's wireless too" moment — but the actual judged
  demonstration should run on USB, the only transport with zero unknowns.
