import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import (
    RoverFrameError,
    checksum,
    decode_frame,
    encode_frame,
)


def test_checksum_known_value():
    # XOR of ASCII bytes of "D,1,2" computed by hand: 'D'^','^'1'^','^'2'
    payload = "D,1,2"
    expected = 0
    for b in payload.encode("ascii"):
        expected ^= b
    assert checksum(payload) == expected


def test_encode_decode_roundtrip_simple():
    frame = encode_frame("D", [120, -120, 0])
    msg_type, fields = decode_frame(frame)
    assert msg_type == "D"
    assert fields == [120, -120, 0]


def test_encode_decode_roundtrip_negative_and_zero():
    frame = encode_frame("E", [0, -1, 32767, -32768])
    msg_type, fields = decode_frame(frame)
    assert msg_type == "E"
    assert fields == [0, -1, 32767, -32768]


def test_encode_decode_no_fields():
    frame = encode_frame("H", [])
    msg_type, fields = decode_frame(frame)
    assert msg_type == "H"
    assert fields == []


def test_encode_rejects_lowercase_type():
    with pytest.raises(RoverFrameError):
        encode_frame("d", [1])


def test_encode_rejects_multichar_type():
    with pytest.raises(RoverFrameError):
        encode_frame("DD", [1])


def test_encode_rejects_float_field():
    with pytest.raises(RoverFrameError):
        encode_frame("D", [1.5])  # type: ignore[list-item]


def test_decode_rejects_missing_checksum_separator():
    with pytest.raises(RoverFrameError):
        decode_frame("D,1,2\n")


def test_decode_rejects_bad_checksum():
    good = encode_frame("D", [1, 2, 3])
    payload, _, _cc = good.strip("\n").rpartition("*")
    corrupted = f"{payload}*00\n"
    with pytest.raises(RoverFrameError):
        decode_frame(corrupted)


def test_decode_rejects_non_integer_field():
    with pytest.raises(RoverFrameError):
        decode_frame("D,1.5*00\n")


def test_decode_rejects_empty_line():
    with pytest.raises(RoverFrameError):
        decode_frame("")


def test_decode_accepts_bytes_and_crlf():
    frame_str = encode_frame("S", [7, 8])
    frame_bytes = frame_str.replace("\n", "\r\n").encode("ascii")
    msg_type, fields = decode_frame(frame_bytes)
    assert msg_type == "S"
    assert fields == [7, 8]


def test_decode_rejects_line_too_long():
    huge = "D," + ",".join(["1"] * 100)
    with pytest.raises(RoverFrameError):
        decode_frame(huge + "*00\n")


def test_encode_rejects_oversize_frame():
    with pytest.raises(RoverFrameError):
        encode_frame("D", list(range(60)))
