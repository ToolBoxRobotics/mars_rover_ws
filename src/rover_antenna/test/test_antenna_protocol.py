import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "rover_protocol"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import RoverFrameError, decode_frame
from rover_antenna import antenna_protocol


def test_encode_gimbal_command_roundtrip():
    frame = antenna_protocol.encode_gimbal_command(1500, 900, True)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "G"
    assert fields == [1500, 900, 1]


def test_encode_gimbal_command_driver_disabled():
    frame = antenna_protocol.encode_gimbal_command(0, 0, False)
    _msg_type, fields = decode_frame(frame)
    assert fields[2] == 0


def test_encode_home_request_roundtrip():
    frame = antenna_protocol.encode_home_request()
    msg_type, fields = decode_frame(frame)
    assert msg_type == "Z"
    assert fields == []


def test_parse_gimbal_state_valid():
    (
        azimuth,
        elevation,
        az_limit,
        el_limit,
        homed,
        voltage_mv,
        driver_enabled,
        temperature_deci_c,
        fan_duty_percent,
    ) = antenna_protocol.parse_gimbal_state([1500, 900, 0, 1, 1, 12600, 1, 235, 65])
    assert azimuth == 1500
    assert elevation == 900
    assert az_limit is False
    assert el_limit is True
    assert homed is True
    assert voltage_mv == 12600
    assert driver_enabled is True
    assert temperature_deci_c == 235
    assert fan_duty_percent == 65


def test_parse_gimbal_state_driver_disabled():
    *_rest, driver_enabled, _temperature_deci_c, _fan_duty_percent = antenna_protocol.parse_gimbal_state(
        [0, 0, 0, 0, 0, 12000, 0, -9999, 0]
    )
    assert driver_enabled is False


def test_parse_gimbal_state_fan_off_when_not_running():
    *_rest, fan_duty_percent = antenna_protocol.parse_gimbal_state([0, 0, 0, 0, 1, 12000, 1, 200, 0])
    assert fan_duty_percent == 0


def test_parse_gimbal_state_fields_are_bool_typed():
    # Not just truthy/falsy - genuinely `bool`, so downstream ROS
    # message assignment (AntennaState.azimuth_limit_triggered etc.,
    # all `bool` fields) gets exactly the type it expects.
    _, _, az_limit, el_limit, homed, _, driver_enabled, _, _ = antenna_protocol.parse_gimbal_state(
        [0, 0, 1, 0, 1, 0, 1, -9999, 0]
    )
    assert isinstance(az_limit, bool)
    assert isinstance(el_limit, bool)
    assert isinstance(homed, bool)
    assert isinstance(driver_enabled, bool)


def test_parse_gimbal_state_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        antenna_protocol.parse_gimbal_state([1, 2])
    with pytest.raises(RoverFrameError):
        # Old 8-field format from before fan_duty_percent was added -
        # must not be silently accepted as if still valid.
        antenna_protocol.parse_gimbal_state([1500, 900, 0, 1, 1, 12600, 1, 235])
