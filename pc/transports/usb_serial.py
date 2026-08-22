"""
USBSerialTransport — the EXISTING, WORKING, tested transport, refactored
out of haptic_engine.py's old HapticLink class with no behavior change:
same pyserial usage, same background reader thread draining replies so
the OS buffer never backs up, same 'M,.../S/PING/STATUS' protocol.

This is the source of truth for wired operation. See
docs/serial-protocol.md.
"""

import threading
import time

try:
    import serial
except ImportError:
    serial = None

from .base import HapticTransport, MotorCommand, format_motor_command, format_stop

BAUD_RATE = 115200
SERIAL_READ_TIMEOUT_S = 0.01


class USBSerialTransport(HapticTransport):
    def __init__(self, port=None, baud=BAUD_RATE):
        if serial is None:
            raise ImportError("Missing dependency: pyserial. Install with: pip install pyserial")
        if not port:
            raise ValueError("USBSerialTransport requires 'port' (e.g. COM8 or /dev/ttyACM1)")
        self.port = port
        self.baud = baud
        self.ser = None
        self._connected = False
        self._reader = None
        self._running = False
        self.last_reply = ""
        self._last_ping_sent = None
        self._latency_ms = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=SERIAL_READ_TIMEOUT_S)
        except serial.SerialException:
            self._connected = False
            return False
        self._connected = True
        self._running = True
        self._reader = threading.Thread(target=self._drain, daemon=True)
        self._reader.start()
        return True

    def _drain(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                self._connected = False
                time.sleep(0.1)
                continue
            if raw:
                self.last_reply = raw.decode("ascii", errors="replace").strip()
                if self.last_reply == "PONG" and self._last_ping_sent is not None:
                    self._latency_ms = (time.monotonic() - self._last_ping_sent) * 1000.0
                    self._last_ping_sent = None

    def send_motor_command(self, command: MotorCommand):
        self._write(format_motor_command(command) + "\n")

    def stop(self):
        self._write(format_stop() + "\n")

    def ping(self):
        self._last_ping_sent = time.monotonic()
        self._write("PING\n")

    def status(self):
        self._write("STATUS\n")

    def is_connected(self):
        return self._connected

    def get_latency_ms(self):
        return self._latency_ms

    def _write(self, line):
        if not self._connected or self.ser is None:
            return
        try:
            self.ser.write(line.encode("ascii"))
        except serial.SerialException:
            self._connected = False

    def disconnect(self):
        self._running = False
        self._connected = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
