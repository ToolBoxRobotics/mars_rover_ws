import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rover_sensors.bno086_rvc_parser import (
    FRAME_LEN,
    StreamSync,
    parse_frame,
    quaternion_from_yaw_pitch_roll,
)

# Golden vector straight from the CEVA/Hillcrest BNO08X datasheet's own
# worked checksum example (section "UART-RVC interface").
DATASHEET_EXAMPLE_FRAME = bytes(
    [0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7]
)


def build_frame(index, yaw_raw, pitch_raw, roll_raw, ax_raw, ay_raw, az_raw):
    """Independently packs a frame (not sharing code with the parser's
    own checksum implementation) so the round-trip test is a genuine
    cross-check rather than testing the module against itself.
    """
    body = struct.pack("<BhhhhhhBBB", index, yaw_raw, pitch_raw, roll_raw, ax_raw, ay_raw, az_raw, 0, 0, 0)
    payload_for_checksum = body  # bytes 2..17 of the full frame, i.e. everything after the sync pair
    checksum = sum(payload_for_checksum) % 256
    return bytes([0xAA, 0xAA]) + body + bytes([checksum])


def test_datasheet_worked_example_parses_successfully():
    reading = parse_frame(DATASHEET_EXAMPLE_FRAME)
    assert reading is not None
    assert reading.index == 0xDE


def test_datasheet_worked_example_checksum_matches_by_construction():
    # The frame's own trailing byte must equal our computed checksum
    # over frame[2:18]; parse_frame returning non-None already proves
    # this, but assert it explicitly for documentation purposes.
    payload = DATASHEET_EXAMPLE_FRAME[2:18]
    assert sum(payload) % 256 == DATASHEET_EXAMPLE_FRAME[18]


def test_parse_frame_roundtrip_known_values():
    frame = build_frame(index=5, yaw_raw=9000, pitch_raw=-4500, roll_raw=1800, ax_raw=100, ay_raw=-200, az_raw=981)
    reading = parse_frame(frame)
    assert reading is not None
    assert reading.index == 5
    assert reading.yaw_deg == 90.0
    assert reading.pitch_deg == -45.0
    assert reading.roll_deg == 18.0
    assert reading.accel_x_mps2 == 1.0
    assert reading.accel_y_mps2 == -2.0
    assert abs(reading.accel_z_mps2 - 9.81) < 1e-9


def test_parse_frame_rejects_wrong_length():
    assert parse_frame(b"\xAA\xAA\x00") is None


def test_parse_frame_rejects_missing_sync():
    frame = bytearray(build_frame(0, 0, 0, 0, 0, 0, 0))
    frame[0] = 0x00
    assert parse_frame(bytes(frame)) is None


def test_parse_frame_rejects_bad_checksum():
    frame = bytearray(build_frame(0, 100, 200, 300, 1, 2, 3))
    frame[-1] ^= 0xFF  # corrupt the checksum byte
    assert parse_frame(bytes(frame)) is None


def test_stream_sync_single_frame_across_chunks():
    frame = build_frame(index=1, yaw_raw=100, pitch_raw=200, roll_raw=300, ax_raw=0, ay_raw=0, az_raw=0)
    sync = StreamSync()
    readings = []
    # feed it in small, arbitrary-sized chunks to exercise buffering
    for i in range(0, len(frame), 3):
        readings.extend(sync.feed(frame[i : i + 3]))
    assert len(readings) == 1
    assert readings[0].index == 1


def test_stream_sync_recovers_from_leading_garbage():
    frame = build_frame(index=2, yaw_raw=0, pitch_raw=0, roll_raw=0, ax_raw=0, ay_raw=0, az_raw=0)
    sync = StreamSync()
    readings = sync.feed(b"\x01\x02\x03\xAA garbage-not-a-real-sync" + frame)
    assert len(readings) == 1
    assert readings[0].index == 2


def test_stream_sync_handles_multiple_consecutive_frames():
    f1 = build_frame(1, 100, 0, 0, 0, 0, 0)
    f2 = build_frame(2, 200, 0, 0, 0, 0, 0)
    sync = StreamSync()
    readings = sync.feed(f1 + f2)
    assert [r.index for r in readings] == [1, 2]
    assert readings[1].yaw_deg == 2.0


def test_quaternion_identity_at_zero_orientation():
    x, y, z, w = quaternion_from_yaw_pitch_roll(0.0, 0.0, 0.0)
    assert (round(x, 9), round(y, 9), round(z, 9), round(w, 9)) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_is_normalized():
    x, y, z, w = quaternion_from_yaw_pitch_roll(37.0, -12.5, 5.0)
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    assert abs(norm - 1.0) < 1e-9


def test_frame_len_constant_matches_struct_layout():
    # 2 sync + struct payload must equal the documented 19-byte frame.
    assert FRAME_LEN == 19
