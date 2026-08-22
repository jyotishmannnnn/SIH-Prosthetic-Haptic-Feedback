"""
WiFiTransport — UDP client. NOT TESTED against real hardware: no
WiFi-capable Haptic ESP32 firmware exists in this repository yet (only
the USB serial firmware, firmware/haptic-controller/haptic_controller_v1/).
This is real, runnable client code, written against a documented,
not-yet-built firmware contract — see docs/wireless-setup.md before
relying on it for a demo.

WHY UDP, NOT TCP (see docs/transport-options.md for the full comparison):
Motor commands are a continuously-refreshed STATE, not a queue of events
that all matter — haptic_engine.py already resends the current PWM
values ~50x/second (HAPTIC_SEND_HZ in haptic_engine.py). If one packet
is lost, the next one 20ms later supersedes it completely; there is
nothing to retransmit. TCP's guaranteed-in-order delivery buys nothing
here and its head-of-line blocking on a lost/delayed packet would add
latency that UDP simply doesn't have. UDP also avoids TCP's connection
setup/teardown state on a lightweight microcontroller. The tradeoff we
accept: no delivery guarantee — which is exactly why the ESP32-side
500ms watchdog remains mandatory regardless of transport (a lost UDP
motor-command datagram must never look different from a lost USB byte).

Packet format: same ASCII text as the USB protocol ("M,...", "S",
"PING", "STATUS"), one UDP datagram per command, no framing needed since
UDP is already datagram-oriented (unlike the serial byte stream, no
newline terminator required, though the firmware side may choose to
accept one for code reuse with its serial parser).
"""

import socket
import time

from .base import HapticTransport, MotorCommand, format_motor_command, format_stop


class WiFiTransport(HapticTransport):
    def __init__(self, host=None, port=4210, timeout_s=0.5):
        if not host:
            raise ValueError("WiFiTransport requires 'host' (Haptic ESP32's IP address)")
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.sock = None
        self._connected = False
        self._last_ping_sent = None
        self._latency_ms = None
        self._last_send_ok = 0.0

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout_s)
        # UDP has no real "connection"; treat a successful PING/PONG
        # round-trip as the liveness check instead of a socket-level connect.
        self.ping()
        try:
            data, _ = self.sock.recvfrom(64)
            if data.decode("ascii", errors="replace").strip() == "PONG":
                self._connected = True
        except socket.timeout:
            self._connected = False
        return self._connected

    def send_motor_command(self, command: MotorCommand):
        self._send(format_motor_command(command))

    def stop(self):
        self._send(format_stop())

    def ping(self):
        self._last_ping_sent = time.monotonic()
        self._send("PING")

    def status(self):
        self._send("STATUS")

    def is_connected(self):
        return self._connected

    def get_latency_ms(self):
        return self._latency_ms

    def _send(self, text):
        if self.sock is None:
            return
        try:
            self.sock.sendto(text.encode("ascii"), (self.host, self.port))
            self._last_send_ok = time.monotonic()
        except OSError:
            self._connected = False

    def disconnect(self):
        self._connected = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
