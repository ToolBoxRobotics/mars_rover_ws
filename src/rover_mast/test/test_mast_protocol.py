import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "rover_protocol"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import RoverFrameError, decode_frame
from rover_mast import mast_protocol


def test_encode_mast_command_roundtrip():
    frame = mast_protocol.encode_mast_command(450, -100, mast_protocol.LIFT_ERECT, True)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "M"
    assert fields == [450, -100, 1, 1]


def test_encode_mast_command_driver_disabled():
    frame = mast_protocol.encode_mast_command(0, 0, mast_protocol.LIFT_HOLD, False)
    _msg_type, fields = decode_frame(frame)
    assert fields[3] == 0


def test_encode_mast_command_rejects_bad_lift_mode():
    with pytest.raises(RoverFrameError):
        mast_protocol.encode_mast_command(0, 0, 5, True)


def test_encode_home_request_roundtrip():
    frame = mast_protocol.encode_home_request()
    msg_type, fields = decode_frame(frame)
    assert msg_type == "Z"
    assert fields == []


def test_parse_mast_state_valid():
    (
        yaw,
        pitch,
        lift_state,
        yaw_limit,
        pitch_limit,
        homed,
        voltage_mv,
        driver_enabled,
        temperature_deci_c,
        fan_duty_percent,
    ) = mast_protocol.parse_mast_state(
        [100, -50, mast_protocol.STATE_SERVICE, 0, 1, 1, 12600, 1, 235, 65]
    )
    assert yaw == 100
    assert pitch == -50
    assert lift_state == 2
    assert yaw_limit is False
    assert pitch_limit is True
    assert homed is True
    assert voltage_mv == 12600
    assert driver_enabled is True
    assert temperature_deci_c == 235
    assert fan_duty_percent == 65


def test_parse_mast_state_driver_disabled():
    *_rest, driver_enabled, _temperature_deci_c, _fan_duty_percent = mast_protocol.parse_mast_state(
        [0, 0, mast_protocol.STATE_UNKNOWN, 0, 0, 1, 12000, 0, -9999, 0]
    )
    assert driver_enabled is False


def test_parse_mast_state_fan_off_when_not_running():
    # 0 is a real, expected duty cycle (the thermostat hasn't decided
    # to run the fan), not a sentinel or an error - distinct from
    # temperature_deci_c's own -9999 "sensor not found" convention.
    *_rest, fan_duty_percent = mast_protocol.parse_mast_state(
        [0, 0, mast_protocol.STATE_UNKNOWN, 0, 0, 1, 12000, 0, 200, 0]
    )
    assert fan_duty_percent == 0


def test_parse_mast_state_limit_and_homed_fields_are_bool_typed():
    # Not just truthy/falsy - genuinely `bool`, so downstream ROS
    # message assignment (MastState.yaw_limit_triggered etc., all
    # `bool` fields) gets exactly the type it expects.
    _, _, _, yaw_limit, pitch_limit, homed, _, driver_enabled, _, _ = mast_protocol.parse_mast_state(
        [0, 0, mast_protocol.STATE_UNKNOWN, 1, 0, 0, 0, 1, -9999, 0]
    )
    assert isinstance(yaw_limit, bool)
    assert isinstance(pitch_limit, bool)
    assert isinstance(homed, bool)
    assert isinstance(driver_enabled, bool)


def test_parse_mast_state_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        mast_protocol.parse_mast_state([1, 2])
    with pytest.raises(RoverFrameError):
        # Old 9-field format from before fan_duty_percent was added -
        # must not be silently accepted as if still valid.
        mast_protocol.parse_mast_state([100, -50, 2, 0, 1, 1, 12600, 1, 235])


def test_parse_mast_state_rejects_unknown_lift_state():
    # Correct (10-field) length, but a lift_state value that isn't one
    # of the four known constants - this must fail on the lift_state
    # check specifically, not on field count (using an already-correct
    # length here matters: a wrong-length input would raise for the
    # wrong reason and this test would stop actually exercising the
    # validation it's named for).
    with pytest.raises(RoverFrameError):
        mast_protocol.parse_mast_state([0, 0, 99, 0, 0, 1, 12000, 1, -9999, 0])
