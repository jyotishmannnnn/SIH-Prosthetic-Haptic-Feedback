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

| Motor | XIAO Pin | GPIO |
|---|---|---|
| M0 | D0 | 1 |
| M1 | D1 | 2 |
| M2 | D2 | 3 |
| M3 | D3 | 4 |
| M4 | D8 | 7 |
| M5 | D9 | 8 |

D4/D5 avoided (this board family's default I2C pins, unused on this
board but kept clear), D6/D7 avoided (UART0).

PWM: ESP32 Arduino core 3.x pin-addressed LEDC API
(`ledcAttach(pin, freq, resolutionBits)` + `ledcWrite(pin, duty)`),
20 kHz carrier (above audible range), 8-bit duty resolution (0-255).

## Motor driver stage — ASSUMPTION, not verified against a schematic

**Coin ERM motors must not be driven directly from ESP32 GPIO** — they
draw more current than a GPIO pin can safely source and are inductive
(brushed DC), producing back-EMF on turn-off. No motor driver schematic
exists in this repo or in the upstream eFlesh source, so the following is
an assumed, generic driver stage per motor:

```
XIAO GPIO (PWM, 0-3.3V)
   -> 220 ohm - 1k ohm gate/base resistor
   -> N-channel MOSFET gate (e.g. AO3400) or NPN base (e.g. 2N2222)
      MOSFET drain / transistor collector -> motor (-) terminal
      motor (+) terminal -> separate motor supply rail (NOT the ESP32 3V3 rail)
      MOSFET source / transistor emitter -> common GND (shared with ESP32 GND)
      flyback diode (1N4148/1N5819) across motor terminals, cathode to +supply
```

If a specific driver board/circuit is later chosen, update this file and
`firmware/haptic-controller/haptic_controller_v1/haptic_controller_v1.ino`'s
`MOTOR_PWM_INVERTED` define together (that's the only place polarity
inversion is applied).

### Related, unresolved reference: ALPACA driver design

A separate/earlier prototype (`gui/legacy-alpaca-fsr-glove/`) documents a
**working, tested** driver circuit for coin ERM motors on a XIAO
ESP32-S3: a single BC557 PNP transistor per motor, high-side, GPIO LOW =
motor ON (see `gui/legacy-alpaca-fsr-glove/ALPACA_haptic_handoff.md`,
"Motor driver" section). It targets a 3.3V motor rail specifically and
notes it would need a second transistor stage on a 5V rail. This is a
different hardware/pin context (single-board FSR glove, not the
two-ESP32 eFlesh split) but is a real, bench-verified circuit worth
reusing or adapting if you don't already have a driver board.

## Onboard status LED

The legacy single-board prototype (`firmware/legacy/eFlesh_haptic_v1_single_board/`)
uses GPIO21 (active-low) as a calibration-status LED, per Seeed's XIAO
ESP32-S3 pinout docs. The current split sensor/haptic firmware does not
use it.
