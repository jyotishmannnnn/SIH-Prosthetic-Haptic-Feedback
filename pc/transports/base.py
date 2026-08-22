"""
Common transport interface + wire-format helpers shared by every
transport. This is the ONLY place the ASCII "M,.../S" wire format is
generated from, so USB/Wi-Fi/BLE all stay byte-for-byte consistent with
docs/serial-protocol.md without duplicating string-building logic.

MotorCommand is the transport-independent output of the haptic
algorithm's encoding step (see haptic_algorithm.HapticEncoder.update(),
which returns a plain [m0..m5] list — MotorCommand.from_list() wraps
that list for transports, nothing more).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

NUM_MOTORS = 6


def _clamp255(v):
    return max(0, min(255, int(v)))


@dataclass(frozen=True)
class MotorCommand:
    m0: int = 0
    m1: int = 0
    m2: int = 0
    m3: int = 0
    m4: int = 0
    m5: int = 0

    @classmethod
    def from_list(cls, values):
        if len(values) != NUM_MOTORS:
            raise ValueError(f"MotorCommand needs exactly {NUM_MOTORS} values, got {len(values)}")
        return cls(*[_clamp255(v) for v in values])

    def as_list(self):
        return [self.m0, self.m1, self.m2, self.m3, self.m4, self.m5]


def format_motor_command(command: MotorCommand) -> str:
    """The wire format documented in docs/serial-protocol.md: 'M,<m0>,...,<m5>'."""
    return "M," + ",".join(str(v) for v in command.as_list())


def format_stop() -> str:
    return "S"


class HapticTransport(ABC):
    """Every transport (USB serial, Wi-Fi, BLE, ESP-NOW) implements this.

    haptic_engine.py is the only caller. The tactile-processing/haptic-
    encoding algorithm (haptic_algorithm.py) never imports this module —
    it stays completely transport-independent.

    Contract every implementation MUST honor:
      - send_motor_command() must never raise on a transient link failure;
        it records the failure (is_connected() becomes False) and returns,
        so the caller's main loop is never blocked or crashed by a dead link.
      - The Haptic ESP32's own 500ms watchdog is the ultimate failsafe
        regardless of what the PC-side transport thinks its state is —
        no transport implementation is allowed to weaken that assumption.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Open the link. Returns True on (best-effort) success."""

    @abstractmethod
    def disconnect(self):
        """Close the link. Safe to call multiple times."""

    @abstractmethod
    def send_motor_command(self, command: MotorCommand):
        """Send a motor command. Must not raise on transient failure."""

    @abstractmethod
    def stop(self):
        """Send the stop-all-motors command."""

    @abstractmethod
    def ping(self):
        """Send a liveness probe (transport-specific; may be a no-op if
        the transport has no PING concept)."""

    @abstractmethod
    def status(self):
        """Request a status report (transport-specific; may be a no-op)."""

    @abstractmethod
    def is_connected(self) -> bool:
        """True only if the link is believed to actually be up right now
        — never just 'was opened successfully once'."""

    def get_latency_ms(self):
        """Optional: last measured round-trip latency in ms, or None if
        not measured/not applicable. Used by the GUI status readout."""
        return None
