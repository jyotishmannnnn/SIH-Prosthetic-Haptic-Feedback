# GUI

**Status: WIP.** The current eFlesh/MLX90393 pipeline (documented in
`../docs/`) does not yet have a dedicated graphical GUI. Real-time
visualization for that pipeline is currently the terminal dashboard built
into `pc/haptic_engine.py` (updates at ~8 Hz), which displays:

- sensor connection status, haptic connection status
- sensor sample rate
- Bx/By/Bz and baseline-subtracted dx/dy/dz
- M/N/S/dM/HF_energy features
- normalized tactile intensity (bar graph)
- contact state / tactile state machine state
- all six motor PWM values (bar graph per motor)
- current pulse pattern (ON/OFF ms, or "continuous")
- calibration state ("CALIBRATING..." vs "RUNNING")

A browser/desktop GUI covering the same fields (per the original scope
for this pipeline) has not been built yet — this is intentionally called
out as not-done rather than implied to exist.

## Transport selection (for a future GUI)

`pc/transports/` (see `docs/transport-options.md`) already exposes a
single factory, `transports.get_transport("usb"|"wifi"|"ble"|"espnow", ...)`,
returning an object with `connect()`/`is_connected()`/`get_latency_ms()`/
`send_motor_command()`. A future GUI's "HAPTIC CONNECTION" panel (radio
buttons for USB/Wi-Fi/Bluetooth/ESP-NOW + a Connect button + a
Transport/Status/Latency readout) should call this factory directly —
it must not implement any transport protocol logic itself. Today,
`pc/haptic_engine.py --transport <name>` is the interim selector until a
GUI exists.

## Legacy GUI (different project)

`legacy-alpaca-fsr-glove/` contains a **complete, working** embedded
browser GUI — but it belongs to a separate, earlier prototype (the
ALPACA FSR-glove demo, WiFi-based, different hardware/pin layout). See
`legacy-alpaca-fsr-glove/README.md` for what it actually is and why it's
kept separate from the current eFlesh pipeline's documentation.
