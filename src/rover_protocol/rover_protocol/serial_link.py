"""Reusable, hardware-agnostic serial connection manager.

Every bridge node (base, arm, mast, microscope, antenna, power) needs
the same basic
behaviour: open a port, tolerate it not existing yet or disappearing
(USB re-enumeration), write outgoing frames, read incoming lines, and
keep simple health counters for the BoardStatus topic. Implementing
this once here means it can be unit tested against a fake serial
object instead of real hardware, and every bridge gets identical
reconnect behaviour for free.

Boot grace period: opening a serial connection to an Arduino-family
board (Mega/Uno) typically triggers a hardware reset - the DTR line
toggling on open discharges/charges the cap tied to the board's RESET
pin, the same mechanism the IDE relies on for auto-upload. That means
the board spends the first ~1-2 seconds after a fresh connection
rebooting, not running the sketch, so anything written to it during
that window is simply lost - it never reaches a loop() that's actually
running yet. `write_frame()` silently no-ops during `boot_grace_sec`
after any (re)connection so the bridge doesn't waste writes into a
board that isn't listening, and doesn't misread that silence as a
communication failure.

write_timeout: FIXED, a real bug, not a hypothetical hardening. The
underlying pyserial connection used to be constructed with a read
timeout but no write one - pyserial's own default when write_timeout
is left unset is None, meaning a write blocks indefinitely if the
OS-level output buffer fills and the board on the other end isn't
draining its own serial RX fast enough to keep up. This was a real,
previously-unbounded hang, confirmed via a live log capture: a slow,
blocking stretch of firmware work (a real example - the arm's own
homing sequence, which legitimately runs far longer than this
project's usual command-response turnaround, especially against a
high-ratio gearbox) was enough to trigger it, since the bridge node's
own continuous, periodic writes kept arriving while the firmware's
loop() wasn't getting back around to Serial.available() quickly
enough - eventually filling the board's tiny 64-byte hardware RX
buffer and blocking the call forever, with no error and no further
log output at all, since a thread stuck on a blocking syscall
produces neither. See write_frame()'s own docstring for the full
mechanism; the fix (a bounded write_timeout, defaulting to 0.2s to
match the existing read timeout) applies to every board through this
one shared class, not just the one whose homing sequence happened to
be slow enough to actually trigger it during testing.

The real pyserial ``serial.Serial`` class is only imported lazily
(inside ``_default_factory``) so this module - and its unit tests -
have no hard dependency on pyserial being importable in every
environment that merely wants to test the framing logic.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from .framing import RoverFrameError, decode_frame


def _default_factory(port: str, baud: int, timeout: float, write_timeout: float):
    import serial  # local import: keeps pyserial optional for pure logic tests

    return serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=write_timeout)


class SerialLink:
    """Manages one serial connection to one Arduino board.

    ``serial_factory`` defaults to opening a real pyserial port, but
    tests inject a fake object exposing ``write(bytes)``,
    ``readline() -> bytes`` and ``close()`` so the reconnect / framing
    logic can be exercised without hardware.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float = 0.2,
        write_timeout: float = 0.2,
        serial_factory: Callable = _default_factory,
        clock: Callable[[], float] = time.monotonic,
        boot_grace_sec: float = 2.0,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._serial_factory = serial_factory
        self._clock = clock
        self._boot_grace_sec = boot_grace_sec

        self._conn = None
        self.connected = False
        self.rx_frame_count = 0
        self.checksum_error_count = 0
        self.reconnect_count = 0
        self._last_rx_time: Optional[float] = None
        self._connected_at: Optional[float] = None

    # -- connection lifecycle -------------------------------------------------
    def ensure_open(self) -> bool:
        """Try to (re)open the port if it isn't currently connected."""
        if self.connected:
            return True
        try:
            self._conn = self._serial_factory(self.port, self.baud, self.timeout, self.write_timeout)
            self.connected = True
            self.reconnect_count += 1
            self._connected_at = self._clock()
            return True
        except Exception:
            self._conn = None
            self.connected = False
            return False

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
        self._conn = None
        self.connected = False

    def in_boot_grace_period(self) -> bool:
        """True if we (re)connected recently enough that the board may
        still be rebooting from the connection's own auto-reset.
        """
        if self._connected_at is None:
            return False
        return (self._clock() - self._connected_at) < self._boot_grace_sec

    # -- I/O --------------------------------------------------------------
    def write_frame(self, frame_str: str) -> bool:
        """Write a pre-encoded frame (see rover_protocol.framing.encode_frame).

        No-ops (returns False without touching the port) during the
        post-connect boot grace period - see the module docstring.

        FIXED: a real, previously-unbounded hang, not a hypothetical
        one. Before write_timeout existed here, the underlying
        pyserial Serial object had a read timeout but no write one -
        pyserial's own default for an unset write_timeout is None,
        meaning a write blocks indefinitely if the OS-level output
        buffer fills and the board on the other end isn't draining
        its own serial RX fast enough to keep up. A slow, blocking
        stretch of firmware work (a real example: the arm's own
        homing sequence, which can legitimately run far longer than
        this project's usual command-response turnaround, especially
        against a high-ratio gearbox) was enough to trigger this: the
        bridge node's own continuous, periodic writes kept arriving
        while the firmware's loop() wasn't getting back around to
        Serial.available() quickly enough, eventually filling the
        board's tiny 64-byte hardware RX buffer and blocking this
        call forever - not a crash, just silence, since a thread stuck
        on a blocking syscall produces no error and no further log
        output at all, which is exactly what made this so hard to
        previously diagnose from code review alone. Bounding this with
        a real write_timeout means a write that can't complete now
        raises serial.SerialTimeoutException (caught by the same
        `except Exception` below as any other write failure) instead
        of hanging the calling thread - and since every bridge node's
        own timer callback runs on that same thread, an unbounded
        hang here was capable of stalling that node's entire executor,
        not just this one write.
        """
        if not self.ensure_open():
            return False
        if self.in_boot_grace_period():
            return False
        try:
            self._conn.write(frame_str.encode("ascii"))
            return True
        except Exception:
            self.connected = False
            self._conn = None
            return False

    def read_decoded(self):
        """Read one line and decode it. Returns (msg_type, fields) or None
        if no complete/valid frame was available this call. Malformed
        frames are counted and swallowed rather than raised, since a
        dropped frame on a live serial link is expected, recoverable
        noise, not a program error.

        No-ops during the post-connect boot grace period too - some
        boards emit boot-time noise on the UART during their own reset
        transient, which would otherwise get miscounted as checksum
        errors rather than recognized as "board still rebooting."
        """
        if not self.ensure_open():
            return None
        if self.in_boot_grace_period():
            return None
        try:
            raw = self._conn.readline()
        except Exception:
            self.connected = False
            self._conn = None
            return None

        if not raw:
            return None  # timed out with no data

        try:
            msg_type, fields = decode_frame(raw)
        except RoverFrameError:
            self.checksum_error_count += 1
            return None

        self.rx_frame_count += 1
        self._last_rx_time = self._clock()
        return msg_type, fields

    def last_rx_age_sec(self) -> float:
        if self._last_rx_time is None:
            return float("inf")
        return self._clock() - self._last_rx_time
