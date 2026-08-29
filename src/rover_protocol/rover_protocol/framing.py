"""Shared serial line protocol for all Mars-rover Arduino bridges.

Every Arduino board (base Mega #1, arm Mega #2, mast Uno #3, microscope
Uno #4) and every host-side ROS 2 bridge node speaks the same tiny
ASCII framing so the protocol only has to be designed, implemented and
tested once.

Frame format
------------
    <TYPE>,<f1>,<f2>,...,<fn>*<CC>\\n

  TYPE  - single uppercase ASCII letter identifying the message kind
          (board-specific, e.g. 'D' = drive command, 'E' = encoder state)
  f1..fn - signed decimal integers, comma separated. No floats are ever
          sent over the wire: angles are encoded in tenths of a degree
          ("decidegrees") and positions in encoder ticks / motor steps
          so both ends can use plain integer math.
  *CC   - two upper-case hex digits: XOR checksum of every byte in
          "TYPE,f1,f2,...,fn" (i.e. everything before the '*').
  \\n    - line terminator.

Example:  "D,120,120,120,120,120,120,150,-150,150,-150*3F\\n"

Keeping this identical on both the Python side and the Arduino side
(see firmware/common/RoverProtocol.h) means a single unit-tested
implementation defines correctness for the whole fleet of boards.
"""

from __future__ import annotations

from typing import List, Tuple

FRAME_TERMINATOR = "\n"
FIELD_SEP = ","
CHECKSUM_SEP = "*"
MAX_FRAME_LEN = 128  # generous upper bound; guards against garbage on the wire


class RoverFrameError(ValueError):
    """Raised when a line cannot be parsed as a valid rover protocol frame."""


def checksum(payload: str) -> int:
    """XOR checksum over the raw bytes of ``payload`` (the pre-'*' text)."""
    value = 0
    for byte in payload.encode("ascii"):
        value ^= byte
    return value


def encode_frame(msg_type: str, fields: List[int]) -> str:
    """Build a wire-ready frame string for ``msg_type`` and integer ``fields``.

    Raises RoverFrameError if msg_type or fields are malformed.
    """
    if not (isinstance(msg_type, str) and len(msg_type) == 1 and msg_type.isalpha() and msg_type.isupper()):
        raise RoverFrameError(f"msg_type must be a single uppercase letter, got {msg_type!r}")
    for f in fields:
        if not isinstance(f, int):
            raise RoverFrameError(f"field {f!r} is not an int; floats are never sent over the wire")

    payload = FIELD_SEP.join([msg_type] + [str(f) for f in fields])
    cc = checksum(payload)
    frame = f"{payload}{CHECKSUM_SEP}{cc:02X}{FRAME_TERMINATOR}"
    if len(frame) > MAX_FRAME_LEN:
        raise RoverFrameError(f"encoded frame exceeds MAX_FRAME_LEN ({len(frame)} > {MAX_FRAME_LEN})")
    return frame


def decode_frame(line) -> Tuple[str, List[int]]:
    """Parse one line of serial input into (msg_type, [fields]).

    ``line`` may be ``str`` or ``bytes``; trailing CR/LF is tolerated.
    Raises RoverFrameError on any malformed or checksum-mismatched frame.
    """
    if isinstance(line, bytes):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise RoverFrameError(f"non-ASCII bytes on the wire: {line!r}") from exc

    line = line.strip("\r\n")
    if not line:
        raise RoverFrameError("empty line")
    if len(line) > MAX_FRAME_LEN:
        raise RoverFrameError(f"line too long ({len(line)} > {MAX_FRAME_LEN})")

    if CHECKSUM_SEP not in line:
        raise RoverFrameError(f"missing '{CHECKSUM_SEP}' checksum separator in {line!r}")
    payload, _, cc_str = line.rpartition(CHECKSUM_SEP)
    if len(cc_str) != 2:
        raise RoverFrameError(f"checksum must be exactly 2 hex digits, got {cc_str!r}")
    try:
        cc_received = int(cc_str, 16)
    except ValueError as exc:
        raise RoverFrameError(f"checksum {cc_str!r} is not valid hex") from exc

    cc_expected = checksum(payload)
    if cc_received != cc_expected:
        raise RoverFrameError(
            f"checksum mismatch for {line!r}: expected {cc_expected:02X}, got {cc_received:02X}"
        )

    parts = payload.split(FIELD_SEP)
    msg_type, raw_fields = parts[0], parts[1:]
    if len(msg_type) != 1 or not msg_type.isalpha() or not msg_type.isupper():
        raise RoverFrameError(f"invalid message type {msg_type!r} in {line!r}")

    fields: List[int] = []
    for rf in raw_fields:
        try:
            fields.append(int(rf))
        except ValueError as exc:
            raise RoverFrameError(f"non-integer field {rf!r} in {line!r}") from exc

    return msg_type, fields
