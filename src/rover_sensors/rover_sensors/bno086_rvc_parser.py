"""Parser for the BNO08x (BNO080/085/086) UART-RVC output mode.

Board on this rover: SparkFun VR IMU Breakout - BNO086 (Qwiic),
factory I2C address 0x4B (alternate 0x4A via the ADR solder jumper).
That address is only meaningful if the board's PS0/PS1 protocol-select
jumpers are strapped for I2C - on this rover they are NOT: the board
is wired via its UART edge pins straight to a Waveshare "USB TO TTL
(B)" converter (CH343G-based - confirmed via Waveshare's own spec to
be a plain USB-to-UART bridge with no I2C capability, so true I2C
isn't physically possible over this link regardless of the board's
address setting). With PS0/PS1 selecting UART-RVC specifically, no
host microcontroller is needed at all: the sensor free-runs at 115200
baud, 100 Hz, continuously pushing 19-byte binary frames with no
request/response handshake and no addressing of any kind. This module
only implements the parsing side.

If the protocol-select jumpers are ever changed to I2C mode instead
(e.g. to use SparkFun's I2C Arduino library for richer sensor
reports), this UART-RVC driver no longer applies - a different,
address-aware I2C driver would be needed in its place, and note
SparkFun's own hookup guide advises against pairing this chip with an
8-bit AVR (Uno/Mega) over I2C; they recommend an ESP32 or SAMD51.

Frame layout (verified against the CEVA/Hillcrest BNO08X datasheet,
rev 1.16/1.17, section "UART-RVC interface", and cross-checked against
Adafruit's reference driver, adafruit_bno08x_rvc):

    byte 0-1   : sync bytes, always 0xAA 0xAA
    byte 2     : frame index (increments each frame, wraps at 255)
    byte 3-4   : yaw   (int16, little-endian, raw/100.0 = degrees)
    byte 5-6   : pitch (int16, little-endian, raw/100.0 = degrees)
    byte 7-8   : roll  (int16, little-endian, raw/100.0 = degrees)
    byte 9-10  : x acceleration (int16 LE, raw/100.0 = m/s^2)
    byte 11-12 : y acceleration (int16 LE, raw/100.0 = m/s^2)
    byte 13-14 : z acceleration (int16 LE, raw/100.0 = m/s^2)
    byte 15-17 : reserved
    byte 18    : checksum = sum(byte[2:18]) % 256 (payload only, sync bytes excluded)

The datasheet specifies the yaw/pitch/roll rotations should be applied
in that order (yaw about Z, then pitch about Y', then roll about X'')
to reconstruct orientation - see quaternion_from_yaw_pitch_roll below.

No angular velocity (gyro rate) is present in this frame format, so
callers publishing sensor_msgs/Imu should mark angular_velocity_covariance[0]
= -1 (REP 103 "data unavailable" convention) rather than fabricate zeros.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

SYNC_BYTE = 0xAA
FRAME_LEN = 19
# Checksum = sum(frame[2:18]) % 256 - i.e. the 16 bytes immediately
# AFTER the two 0xAA sync bytes (index + yaw/pitch/roll + accel x/y/z
# + the first reserved byte), NOT including the sync bytes themselves.
# Verified against the datasheet's own worked example
# (0xAA 0xAA DE 01 00 92 FF 25 08 8D FE EC FF D1 03 00 00 00 E7): summing
# frame[0:16] (sync-inclusive, as a naive reading of some reference
# drivers suggests) gives 0x3B, not the message's actual trailing 0xE7;
# summing frame[2:18] gives 0xE7, matching exactly.
_CHECKSUM_START = 2
_CHECKSUM_END = 18
_STRUCT_FMT = "<BhhhhhhBBBB"  # index, yaw,pitch,roll,ax,ay,az (int16 LE) + 3 reserved + checksum


@dataclass(frozen=True)
class Bno086Reading:
    index: int
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    accel_x_mps2: float
    accel_y_mps2: float
    accel_z_mps2: float


def parse_frame(frame: bytes) -> Optional[Bno086Reading]:
    """Parse one already-synchronized 19-byte RVC frame.

    Returns None (never raises) if the frame is the wrong length, the
    sync bytes are missing, or the checksum doesn't match - malformed
    frames on a live UART are expected, recoverable noise.
    """
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != SYNC_BYTE or frame[1] != SYNC_BYTE:
        return None

    checksum_expected = sum(frame[_CHECKSUM_START:_CHECKSUM_END]) % 256
    checksum_received = frame[FRAME_LEN - 1]
    if checksum_expected != checksum_received:
        return None

    (_index, yaw_raw, pitch_raw, roll_raw, ax_raw, ay_raw, az_raw, _r1, _r2, _r3, _cc) = (
        struct.unpack_from(_STRUCT_FMT, frame, offset=2)
    )

    return Bno086Reading(
        index=frame[2],
        yaw_deg=yaw_raw / 100.0,
        pitch_deg=pitch_raw / 100.0,
        roll_deg=roll_raw / 100.0,
        accel_x_mps2=ax_raw / 100.0,
        accel_y_mps2=ay_raw / 100.0,
        accel_z_mps2=az_raw / 100.0,
    )


class StreamSync:
    """Feed raw serial bytes in as they arrive; get back zero or more
    parsed readings. Resyncs automatically after any garbage, dropped
    byte, or corrupted frame by searching for the next 0xAA 0xAA pair.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[Bno086Reading]:
        self._buf.extend(data)
        readings: List[Bno086Reading] = []

        while True:
            sync_index = self._buf.find(bytes([SYNC_BYTE, SYNC_BYTE]))
            if sync_index == -1:
                # Keep at most the trailing byte, in case it's the first
                # half of a sync pair that will arrive on the next feed().
                if len(self._buf) > 1:
                    del self._buf[: len(self._buf) - 1]
                break

            if sync_index > 0:
                del self._buf[:sync_index]

            if len(self._buf) < FRAME_LEN:
                break  # wait for the rest of the frame to arrive

            frame = bytes(self._buf[:FRAME_LEN])
            reading = parse_frame(frame)
            if reading is not None:
                readings.append(reading)
                del self._buf[:FRAME_LEN]
            else:
                # Sync bytes matched by coincidence, or checksum failed:
                # drop just the leading sync byte and search again.
                del self._buf[:1]

        return readings


def quaternion_from_yaw_pitch_roll(yaw_deg: float, pitch_deg: float, roll_deg: float) -> Tuple[float, float, float, float]:
    """Standard intrinsic Z (yaw) -> Y' (pitch) -> X'' (roll) Euler-to-
    quaternion conversion, matching the rotation order the BNO08X
    datasheet specifies for reconstructing orientation from this frame.
    Returns (x, y, z, w).
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w
