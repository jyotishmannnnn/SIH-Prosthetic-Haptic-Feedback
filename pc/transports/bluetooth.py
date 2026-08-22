"""
BLETransport — Bluetooth LOW ENERGY only. NOT TESTED against real
hardware: no BLE Haptic ESP32 firmware exists in this repository yet —
see docs/wireless-setup.md for the proposed GATT service design.

IMPORTANT PLATFORM FACT (verified against Espressif/Arduino-ESP32 docs,
not assumed): the ESP32-S3 (used by the XIAO ESP32-S3 in this project)
has a BLE 5.0 radio ONLY. Unlike the original ESP32, it has NO Bluetooth
Classic radio, so Bluetooth Classic SPP ("serial over Bluetooth", the
simplest possible Bluetooth transport) is NOT AVAILABLE on this board.
BLE GATT is the only Bluetooth option here, which is why this transport
sends the ASCII command as a GATT characteristic write instead of a
serial-like byte stream.

Requires the optional 'bleak' package (cross-platform BLE client,
works on Windows/macOS/Linux): pip install bleak
Not added to requirements.txt as a hard dependency since USB-only setups
don't need it — see pc/requirements.txt comments.

bleak is asyncio-based; this class runs a dedicated background event
loop thread so the rest of haptic_engine.py (synchronous, threading-based)
doesn't need to become async just to support one transport.
"""

import asyncio
import threading
import time

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

from .base import HapticTransport, MotorCommand, format_motor_command, format_stop

DEFAULT_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
DEFAULT_CHARACTERISTIC_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"


class BLETransport(HapticTransport):
    def __init__(self, device_name="HapticESP32",
                 service_uuid=DEFAULT_SERVICE_UUID,
                 characteristic_uuid=DEFAULT_CHARACTERISTIC_UUID,
                 scan_timeout_s=5.0):
        if BleakClient is None:
            raise ImportError("Missing dependency: bleak. Install with: pip install bleak")
        self.device_name = device_name
        self.service_uuid = service_uuid
        self.characteristic_uuid = characteristic_uuid
        self.scan_timeout_s = scan_timeout_s

        self._client = None
        self._connected = False
        self._loop = None
        self._loop_thread = None
        self._last_ping_sent = None
        self._latency_ms = None

    # ---- background asyncio loop, so callers stay synchronous ----
    def _start_loop(self):
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

    def _run(self, coro, timeout=10.0):
        if self._loop is None:
            self._start_loop()
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        try:
            return future.result(timeout=timeout)
        except Exception:
            return None

    async def _connect_async(self):
        device = await BleakScanner.find_device_by_name(self.device_name, timeout=self.scan_timeout_s)
        if device is None:
            return False
        self._client = BleakClient(device)
        await self._client.connect()
        return self._client.is_connected

    def connect(self):
        result = self._run(self._connect_async(), timeout=self.scan_timeout_s + 5.0)
        self._connected = bool(result)
        return self._connected

    async def _write_async(self, text):
        await self._client.write_gatt_char(self.characteristic_uuid, text.encode("ascii"), response=False)

    def _send(self, text):
        if not self._connected or self._client is None:
            return
        result = self._run(self._write_async(text), timeout=1.0)
        if result is None:
            # write failed/timed out -- don't assume the link is still good
            self._connected = False

    def send_motor_command(self, command: MotorCommand):
        self._send(format_motor_command(command))

    def stop(self):
        self._send(format_stop())

    def ping(self):
        self._last_ping_sent = time.monotonic()
        self._send("PING")
        # Note: unlike USB/WiFi, reading a PONG reply back over BLE would
        # need a notify subscription on the characteristic (not yet wired
        # up here) -- get_latency_ms() stays None for BLE until that's added.

    def status(self):
        self._send("STATUS")

    def is_connected(self):
        return self._connected

    def get_latency_ms(self):
        return self._latency_ms

    def disconnect(self):
        if self._client is not None:
            self._run(self._client.disconnect(), timeout=5.0)
        self._connected = False
