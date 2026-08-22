# Hardware

All pin assignments below are copied verbatim from the `#define`s in the
committed firmware (`firmware/sensor/eFlesh_sensor_v1/eFlesh_sensor_v1.ino`
and `firmware/haptic-controller/haptic_controller_v1/haptic_controller_v1.ino`).
None of this is invented — if you change a pin, update the `.ino` and this
file together.

## Current hardware list

- Seeed Studio XIAO ESP32-S3 x2 (one sensor node, one haptic node)
- 1x MLX90393 3-axis magnetometer
- 1x e-Flesh tactile patch (mounted on the robotic hand, single patch,
  single MLX90393 — this prototype does NOT use the multi-sensor eFlesh
  architecture)
- 6x coin ERM vibration motors
- Motor driver stage (transistor/MOSFET per motor — see below)
- 3D-printed robotic/bionic hand
- PC running `pc/haptic_engine.py`

## Sensor ESP32-S3

| MLX90393 | XIAO ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | D4 / GPIO5 |
| SCL | D5 / GPIO6 |

I2C clock: 400 kHz. I2C address: `0x0C` (`MLX_I2C_ADDR`) — this is the
address the upstream eFlesh reference sketch uses for its first sensor;
verify with an I2C scan if your board doesn't ACK at this address.

MLX90393 register configuration (copied from the upstream eFlesh
reference sketch, `eFlesh-main/arduino/5X_eflesh_stream/5X_eflesh_stream.ino`,
adapted from 5 sensors to 1):

| Setting | Value |
|---|---|
| Gain select | `0x1` |
| Resolution (X/Y/Z) | `0x2 / 0x2 / 0x2` |
| Digital filtering | `0x4` |
| Burst set | `0xF` (temperature + X + Y + Z) |

Library: `MLX90393.h` from
[tesshellebrekers/arduino-MLX90393](https://github.com/tesshellebrekers/arduino-MLX90393)
(the upstream eFlesh project's own submodule — not vendored into this
repo, install it into your Arduino `libraries/` folder manually).

## Haptic ESP32-S3

| Motor | XIAO Pin | GPIO | Status |
|---|---|---|---|
| M0 | D10 | 9 | **Physically bench-verified** — GPIO9 -> 333 ohm -> CPN2222A base, confirmed spinning a real coin motor before this firmware was written. Do not move this pin without re-verifying. |
| M1 | D9 | 8 | Same topology as M0, not yet individually bench-verified at time of writing |
| M2 | D8 | 7 | Same topology as M0, not yet individually bench-verified at time of writing |
| M3 | D3 | 4 | Same topology as M0, not yet individually bench-verified at time of writing |
| M4 | D2 | 3 | Same topology as M0, not yet individually bench-verified at time of writing |
| M5 | D1 | 2 | Same topology as M0, not yet individually bench-verified at time of writing |

D0 (GPIO1) left spare. D4/D5 avoided (this board family's default I2C
pins — not needed on this board since it never reads the MLX90393, kept
clear anyway for consistency with the sensor board's pin docs). D6/D7
(GPIO43/44, UART0) avoided — GPIO43 blips for ~200ms at boot with the ROM
bootloader log, which would visibly glitch a motor at power-up.

This mapping supersedes an earlier placeholder mapping (M0 on D0/GPIO1)
used before Motor 0 was bench-verified on GPIO9 — GPIO9 is now fixed as
M0 because it's the one pin physically proven to work end-to-end.

PWM: ESP32 Arduino core 3.x pin-addressed LEDC API
(`ledcAttach(pin, freq, resolutionBits)` + `ledcWrite(pin, duty)`),
configurable via `PWM_FREQUENCY` (default 20 kHz, above audible range)
and `PWM_RESOLUTION` (default 8-bit, 0-255 duty) in the firmware.

## Motor driver stage — PHYSICALLY VERIFIED (CPN2222A), same circuit on all six channels

**Coin ERM motors must not be driven directly from ESP32 GPIO** — they
draw more current than a GPIO pin can safely source and are inductive
(brushed DC), producing back-EMF on turn-off.

This is no longer an assumption: the following circuit was bench-tested
on Motor 0 with a real coin motor before being scaled to all six
channels, and is the actual topology `haptic_controller_v1.ino` is
written against:

```
XIAO GPIO (digital / PWM, 0-3.3V)
   -> 333 ohm resistor
   -> CPN2222A (2N2222-family NPN) base
      CPN2222A collector -> motor (-) terminal
      motor (+) terminal -> motor supply rail
      CPN2222A emitter -> common GND (shared with ESP32 GND)
```

- Logic: **GPIO HIGH / PWM duty > 0 = motor ON** (NPN, low-side). No
  inversion needed — `MOTOR_PWM_INVERTED` in firmware is `false`.
- All six motor channels use this exact same topology (one CPN2222A +
  one 333 ohm resistor per motor) — nothing motor-specific changes
  between channels other than the GPIO pin.
- **Flyback protection has not been added or measured yet.** The
  single-motor proof-of-concept ran without one. Before extended/high-
  duty-cycle operation, measure the transistor's collector voltage on
  turn-off with a scope/multimeter; add a flyback diode (cathode to
  motor +, anode to motor -) per channel if the turn-off spike is a
  concern for your specific transistor's voltage rating. Not added
  pre-emptively per "don't unnecessarily change the already-working
  setup" — document here if a channel is later found to need one.
- Motor supply rail voltage/current budget across all six channels
  simultaneously has not been measured yet — see `docs/demo.md` for the
  bring-up test order (one motor -> all six digital -> PWM) that will
  surface any supply-side issues before the PC algorithm is connected.

### Related reference: ALPACA driver design (different project, PNP variant)

A separate/earlier prototype (`gui/legacy-alpaca-fsr-glove/`) documents a
similar but distinct driver: BC557 **PNP**, high-side, GPIO LOW = motor
ON (see `gui/legacy-alpaca-fsr-glove/ALPACA_haptic_handoff.md`, "Motor
driver" section). Not used here — this repo's haptic controller uses the
NPN/low-side CPN2222A topology above — kept as a cross-reference only.

## Sensor interference test — REQUIRED, NOT YET PERFORMED

Coin ERM motors contain a magnet; the MLX90393 is a magnetometer. Motor
activation could inject a magnetic signal the tactile pipeline
misreads as contact. **This has not been measured yet** — do not assume
it's fine, and do not compensate for it in software before measuring.

Procedure (run once both boards are wired and firmware flashed):
1. eFlesh untouched, all motors OFF. Record `M` (filtered magnitude) from
   `python haptic_engine.py --sensor-only --sensor-port COMx` for ~10s.
2. Turn Motor 0 ON (steady PWM, e.g. `P,0,200` over the Haptic ESP32's
   serial monitor, or `MOTOR_TEST_MODE=1`). Record `M` again for ~10s,
   eFlesh still untouched.
3. Turn all six motors ON simultaneously (`A,200`). Record `M` again for
   ~10s, eFlesh still untouched.
4. Compare the three recordings. If motors-ON `M` is comparable to or
   exceeds `CONTACT_LEVEL` from your calibration, the motors are
   corrupting the tactile signal.

If interference is found: the fix is physical (increase spacing between
motors and the MLX90393, reorient motors so their magnetic axis doesn't
couple into the sensor's axes, or shield the sensor) — not a software
deadband/threshold hack, since that would silently reduce real
sensitivity along with the interference.

**Status: not yet run.** Fill in actual measured `M` values here once
tested, before relying on simultaneous sensor+motor operation for a demo.

## Onboard status LED

The legacy single-board prototype (`firmware/legacy/eFlesh_haptic_v1_single_board/`)
uses GPIO21 (active-low) as a calibration-status LED, per Seeed's XIAO
ESP32-S3 pinout docs. The current split sensor/haptic firmware does not
use it.
