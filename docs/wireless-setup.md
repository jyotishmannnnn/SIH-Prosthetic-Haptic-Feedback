# Wireless transport setup

Setup instructions for the wireless transports that have PC-side client
code implemented (`pc/transports/wifi.py`, `pc/transports/bluetooth.py`).

**Neither has matching ESP32 firmware in this repository yet.** The
existing `firmware/haptic-controller/haptic_controller_v1/` is USB-only
and has NOT been modified by this change (per the task's explicit "don't
break USB" requirement). Everything below describes the PC side (real,
runnable) and the firmware CONTRACT that a future firmware would need to
implement to match it — building/flashing/testing that firmware is
future work.

## Wi-Fi (UDP)

### PC side (implemented)
`pc/transports/wifi.py`, selected via `--transport wifi`. Configure the
target in `pc/.env` (copy from `pc/.env.example`):
```
HAPTIC_WIFI_HOST=192.168.4.1
HAPTIC_WIFI_PORT=4210
```
No Wi-Fi credentials are needed on the PC side if the ESP32 runs its own
access point (SoftAP) — the PC just joins that AP like any other Wi-Fi
network via the OS, then talks UDP to the ESP32's known SoftAP IP
(`192.168.4.1` is the ESP32 SoftAP default). If a future firmware instead
joins an existing network (station mode), that network's SSID/password
would need to go in `.env` too — see the comment in `.env.example`.

### Firmware contract (NOT YET BUILT)
A future WiFi-capable haptic firmware would need to:
- Start a UDP listener on the configured port (default 4210).
- Accept the exact same ASCII commands as the USB protocol:
  `M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>`, `S`, `PING` (reply `PONG`), `STATUS`.
- Keep the same 500ms watchdog — no valid command for 500ms means motors
  off, regardless of transport.
- Run either as a SoftAP (simplest, no router/existing network needed —
  matches the "no router required" goal) or join an existing network in
  station mode (requires credentials, adds a dependency on that network
  being present at demo time — SoftAP is recommended for a demo).

### Security considerations
- No cloud/internet access needed either way.
- If run as a SoftAP, set a real WiFi password (WPA2) on the ESP32's
  access point — do not run an open network. Never hard-code that
  password in a committed `.ino` file (see `gui/legacy-alpaca-fsr-glove/`
  for a past example of exactly this mistake, already fixed).
- UDP has no built-in authentication — anyone on the same WiFi network
  could send motor commands to the Haptic ESP32. Acceptable for an
  isolated demo SoftAP; not acceptable if this were ever deployed on a
  shared/public network.

### Timeout / reconnect behavior (implemented)
`WiFiTransport.connect()` sends a `PING` and waits up to `timeout_s`
(default 0.5s) for a `PONG`; no reply means `is_connected()` returns
`False`. There is no persistent "connection" in UDP — every send is
independent, so `haptic_engine.py`'s existing failsafe behavior (stop
motors, mark connection lost) already covers a dead Wi-Fi link the same
way it covers a dead USB link.

## Bluetooth LE

### Critical platform fact
The **ESP32-S3** (used by the XIAO ESP32-S3 in this project) has a
**BLE-only** radio. It does **not** have Bluetooth Classic, so Bluetooth
Classic SPP ("serial over Bluetooth", the simplest possible Bluetooth
transport) is **not available** on this hardware. This is why
`transports/bluetooth.py` is a GATT client, not a serial-port-over-
Bluetooth client.

### PC side (implemented)
`pc/transports/bluetooth.py`, selected via `--transport ble`. Requires
`pip install bleak` (cross-platform BLE client library; not a default
dependency, see `pc/requirements.txt`). Configure the target device name
in `pc/.env`:
```
HAPTIC_BLE_DEVICE_NAME=HapticESP32
```

Proposed GATT service (placeholder UUIDs, Nordic-UART-style, chosen for
convention only — a real firmware could reuse or replace these):
```
Service:        6e400001-b5a3-f393-e0a9-e50e24dcca9e  (HAPTIC_SERVICE_UUID)
Characteristic:  6e400002-b5a3-f393-e0a9-e50e24dcca9e  (MOTOR_COMMAND_UUID, write)
```
The PC writes the same ASCII command text used everywhere else
(`M,120,120,0,0,0,0`, `S`, ...) as the characteristic's raw bytes — no
new binary protocol, per the task's explicit "don't add unnecessary
complexity" guidance. A binary-packed command (6 raw bytes instead of an
ASCII string) would save a little BLE payload size, but at ~50Hz and
BLE's typical ~20ms connection interval, that saving is not worth the
protocol divergence from USB/Wi-Fi for a demo.

### Firmware contract (NOT YET BUILT)
- Advertise as `HapticESP32` (or whatever `HAPTIC_BLE_DEVICE_NAME` is set
  to) with the GATT service/characteristic above.
- On a characteristic write, parse the same ASCII commands as USB.
- Same 500ms watchdog.
- Likely library: NimBLE-Arduino (lower RAM/flash footprint than the
  stock ESP32 BLE Arduino library, commonly recommended for ESP32-S3).

### Latency / packet rate considerations
BLE connection intervals are typically negotiated in the 15-45ms range
unless specifically tuned lower, which puts a practical ceiling on
command rate noticeably below the 50Hz USB send rate — this has **not
been measured** here, but should be expected and verified before relying
on BLE for a demo requiring the same responsiveness as USB.

### Reconnect / failsafe (implemented on PC side)
`BLETransport.connect()` re-scans by device name and reconnects; a
failed characteristic write marks the transport disconnected rather than
silently continuing. As with every transport, the ESP32's own 500ms
watchdog is the actual safety guarantee, independent of what the PC
believes its connection state is.

### Windows compatibility
`bleak` supports Windows (via the WinRT BLE APIs), macOS, and Linux —
chosen specifically because this project's development environment is
Windows. Not yet tested end-to-end (no BLE firmware exists), but the
library choice itself is not a Windows-specific risk.

## ESP-NOW

Not implemented — see `pc/transports/espnow.py` module docstring and
`docs/transport-options.md` ("Why ESP-NOW is not simply 'the best
option'") for why, and what a future gateway firmware would need to do.

## Testing checklist (once firmware exists)

For each wireless transport, before trusting it for a demo:
1. Run `python -m transports.benchmark --transport wifi` (or `ble`) and
   record real latency/packet-loss numbers in `docs/transport-options.md`
   — replace the `NOT TESTED` markers with actual measurements.
2. Confirm the 500ms watchdog still stops motors within spec when the
   link is physically cut (turn off WiFi, walk BLE device out of range).
3. Confirm reconnect works without a firmware reboot.
4. Run the same `haptic_engine.py --simulate` motor-recruitment check
   used for USB (see `docs/demo.md`) to confirm identical haptic
   behavior across transports — the algorithm doesn't change, only the
   wire.
