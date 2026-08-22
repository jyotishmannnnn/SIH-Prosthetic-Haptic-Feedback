# Legacy: ALPACA FSR-glove demo (separate project)

**This is not part of the current eFlesh/MLX90393 two-ESP32 pipeline**
documented in `../../docs/`. It's kept here because it's real, working
code and design reasoning worth preserving — not because it's the current
architecture.

## What this actually is

An earlier/parallel prototype for a different hand design (ALPACA, Team
Mirai) using a **sensing glove** (5x FSR-402 pressure sensors) instead of
eFlesh/MLX90393, communicating over **WiFi** (the current eFlesh pipeline
explicitly does not use WiFi) with a browser-based GUI embedded directly
in the firmware (`INDEX_HTML` in `fsr_gui.ino` — no separate GUI files,
no LittleFS, single-file Arduino upload).

Per `ALPACA_haptic_handoff.md` (included here verbatim as project
history), this glove firmware is a real working asset — "best asset in
repo" per the team's own audit — but the haptic-band code snippet in that
handoff doc has **known, unresolved pin conflicts** with the shipped
`fsr_gui.ino` (documented in the handoff doc's "known open issues" /
the project's internal hardware audit): the glove's 8 FSR channels
occupy D0-D5+D8-D9, colliding with the motor pins and I2C pins the
handoff doc assumes are free. That conflict was never fixed in this
snippet — this is exactly why the current eFlesh prototype uses two
separate ESP32 boards instead of one.

## What it does

- 8x FSR-402 channels (5 fingertip + 3 palm) on a Seeed XIAO ESP32-S3
- WiFi access point + WebSocket server, GUI served from flash
- Embedded GUI: thermographic hand heatmap, live per-zone force readout
  (converted to Newtons via a FSR resistance-force curve, in JavaScript),
  sparkline history, simulation mode (works with no hardware attached),
  tare/calibration commands over the WebSocket channel
- Baseline-drift compensation (tracked per channel in firmware)

## Security note

The original file had a hardcoded WiFi AP password. It's been replaced
with a placeholder (`YOUR_WIFI_PASSWORD_HERE`) in `fsr_gui.ino` — set
your own before flashing. Do not commit a real password back into this
file.

## Relationship to the current prototype

Not integrated, not planned to be integrated as-is. Its motor-driver
circuit (BC557 PNP, high-side, tested and working on a 3.3V rail) is
referenced in `../../docs/hardware.md` as a real, bench-verified
alternative if you don't already have a driver board for the current
eFlesh pipeline's Haptic ESP32.
