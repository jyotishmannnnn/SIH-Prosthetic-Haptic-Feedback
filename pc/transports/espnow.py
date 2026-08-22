"""
ESPNowTransport — ARCHITECTURAL STUB ONLY. Not implemented, not tested,
not runnable. Exists so the transport factory (transports/__init__.py)
has a clean slot for ESP-NOW once the gateway firmware exists.

WHY THIS IS A STUB, NOT A REAL TRANSPORT:
ESP-NOW is a proprietary Espressif protocol implemented in the ESP32 WiFi
radio/driver. A PC cannot speak ESP-NOW — there is no Windows/Linux/macOS
network stack support for it, and no commodity WiFi adapter implements
it. The PC cannot directly reach a Haptic ESP32 over ESP-NOW.

The only viable architecture is a GATEWAY:

    PC --USB--> Sensor ESP32 --ESP-NOW--> Haptic ESP32 --> motors

i.e. the existing Sensor ESP32 would need a second responsibility: relay
motor commands it receives over USB (from the PC) onward to the Haptic
ESP32 via ESP-NOW. This is explicitly OUT OF SCOPE for this task — the
task instructions say not to modify the sensor firmware's core eFlesh
reading functionality, and adding a USB-command-relay + ESP-NOW-transmit
path is more than a trivial addition (see docs/transport-options.md for
the technical evaluation of whether the Sensor ESP32 can even sustain
~100Hz MLX90393 reads + USB + ESP-NOW transmission at once).

If/when that gateway firmware is built, this class becomes a thin
USB-serial client (same shape as USBSerialTransport) that talks to the
Sensor ESP32's COM port using a distinct "relay to ESP-NOW" command
prefix, so the Haptic ESP32's own protocol doesn't need to change at all.
"""

from .base import HapticTransport, MotorCommand


class ESPNowTransport(HapticTransport):
    """Not implemented. Every method is inert/no-op so importing and
    listing this transport (e.g. for a GUI dropdown) is safe, but
    actually selecting it does nothing and never claims to be connected."""

    def __init__(self, **kwargs):
        self._connected = False

    def connect(self):
        return False

    def disconnect(self):
        pass

    def send_motor_command(self, command: MotorCommand):
        pass  # intentionally inert -- see module docstring

    def stop(self):
        pass

    def ping(self):
        pass

    def status(self):
        pass

    def is_connected(self):
        return False

    def get_latency_ms(self):
        return None
