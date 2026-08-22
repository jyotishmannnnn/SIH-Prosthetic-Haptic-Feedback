# Serial protocol

Two independent USB-serial links, both at **115200 baud, 8N1**. The two
ESP32 boards never talk to each other — everything routes through the PC.

## Sensor ESP32 -> PC

One line per sample, ~100 Hz (`SAMPLE_RATE_HZ` in
`firmware/sensor/eFlesh_sensor_v1/eFlesh_sensor_v1.ino`), newline-terminated:

```
S,<millis_timestamp>,<Bx>,<By>,<Bz>
```

Example:
```
S,123456,1023.40,-521.70,284.20
```

- `<millis_timestamp>`: unsigned integer, milliseconds since the sensor
  ESP32 booted (`millis()`), **not** synchronized with PC wall-clock time.
- `<Bx>/<By>/<Bz>`: floats, 2 decimal places, raw MLX90393 burst-read
  values in the library's native units (not gauss/tesla-calibrated,
  not baseline-subtracted — that happens on the PC).

When `DEBUG_SERIAL` is `0` (default), this is the **only** line format on
this stream. Nothing else is printed.

Fault line (rare, distinguishable by prefix):
```
F,<message>
```
Example: `F,MLX90393 repeated read failure`. The PC's `SensorReader`
parser only acts on lines starting with `S,`; anything else (including
`F,...` and, if `DEBUG_SERIAL=1`, `#DBG ...` lines) is captured
separately and never parsed as sensor data.

### Error handling (sensor side)
- Malformed `S,` packets (wrong field count, non-numeric value) are
  silently dropped by the PC parser — no corrupted data reaches the
  processing pipeline.
- The sensor firmware counts consecutive I2C read failures
  (`MAX_CONSECUTIVE_I2C_FAILS`, default 10). On exceeding that count it
  emits an `F,` line and attempts to re-initialize the MLX90393. It never
  sends fabricated or stale readings while faulted.

## PC -> Haptic ESP32

Newline-terminated ASCII commands:

```
M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>\n
```
Sets all 6 motor PWM values (0-255 each) in one command. Example:
```
M,120,120,0,0,0,0
M,210,210,210,210,210,210
```

```
S\n
```
Immediately stops (zeroes) all motors.

```
PING\n
```
Haptic ESP32 replies `PONG`.

```
STATUS\n
```
Haptic ESP32 replies:
```
STATUS,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>,age_ms=<n>
```
where `age_ms` is milliseconds since the last valid command was received
(useful for confirming the watchdog is/isn't about to trip).

### Error handling (haptic side)
- Values outside 0-255 are clamped in firmware.
- A malformed `M,` line (wrong field count, non-numeric value) is dropped
  in its entirety — no partial motor update happens.
- Unknown commands are silently ignored.
- Overlength lines (>63 chars) are discarded to prevent buffer overrun.

### Watchdog (mandatory failsafe)
If no valid `M,` or `S` command is received for `COMM_TIMEOUT_MS`
(**500 ms** in the current firmware), the Haptic ESP32 immediately zeroes
all motors, independent of whatever the PC last commanded. This protects
against PC crash, Python crash, USB disconnect, or a frozen processing
loop. `S` also counts as a valid command for watchdog purposes (an
explicit stop is not "silence").

## Practical note: send rate

`pc/haptic_engine.py` pushes `M,` commands to the Haptic ESP32 at a fixed
50 Hz (`HAPTIC_SEND_HZ`), decoupled from the ~100 Hz sensor sample rate,
so pulse timing accuracy doesn't depend on sensor arrival jitter. 50 Hz
is comfortably above the 500 ms watchdog window.
