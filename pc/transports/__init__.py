"""
Transport abstraction for PC -> Haptic ESP32 communication.

haptic_algorithm.py (the tactile-processing/haptic-encoding algorithm)
does not import anything from this package and never will — it only
produces plain motor-intensity values. haptic_engine.py (the runner)
is the only thing that talks to a transport, via the common
HapticTransport interface in base.py:

    haptic_algorithm.py -> [m0..m5] -> MotorCommand -> HapticTransport -> Haptic ESP32

Only USBSerialTransport is implemented against real, tested hardware
(it's a direct refactor of the existing HapticLink class that used to
live in haptic_engine.py — same protocol, same behavior, same watchdog
reliance). WiFiTransport and BLETransport are real, runnable client code
but have NOT been exercised against real ESP32 firmware in this
environment (no such firmware exists yet — see docs/wireless-setup.md).
ESPNowTransport is an architectural stub only — the PC cannot speak
ESP-NOW directly, see its module docstring and docs/transport-options.md.
"""

from .base import MotorCommand, HapticTransport, format_motor_command, format_stop
from .config import load_transport_config, get_default_transport_name
from .usb_serial import USBSerialTransport
from .wifi import WiFiTransport
from .bluetooth import BLETransport
from .espnow import ESPNowTransport

_TRANSPORTS = {
    "usb": USBSerialTransport,
    "wifi": WiFiTransport,
    "ble": BLETransport,
    "bluetooth": BLETransport,
    "espnow": ESPNowTransport,
}


def get_transport(name, **kwargs):
    """Factory: get_transport("usb", port="COM8") -> USBSerialTransport(...).

    Missing kwargs are filled in from environment variables / local_config.json
    (see config.py, load_transport_config()) so the GUI (or CLI) only needs
    to pass what it actually knows (e.g. just the chosen COM port for USB).
    """
    name = name.lower()
    if name not in _TRANSPORTS:
        raise ValueError(f"Unknown transport '{name}'. Valid: {sorted(_TRANSPORTS)}")

    cfg = load_transport_config()
    transport_cfg = dict(cfg.get(name, {}))
    transport_cfg.update({k: v for k, v in kwargs.items() if v is not None})

    return _TRANSPORTS[name](**transport_cfg)
