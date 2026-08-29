"""Regression test for a real, previously-hit bug: rclpy backs every
fixed-size ROS array message field (e.g. ArmState's own
``joint_position_steps``, BaseState's own ``encoder_ticks``) with
``numpy.ndarray`` internally - so ``list(msg.some_array_field)``
produces ``numpy.int32``/``numpy.bool_`` elements, not plain Python
``int``/``bool``. ``json.dumps()`` cannot serialize either numpy
scalar type at all.

This bit ros_bridge.py's own ``_on_base_state()``/``_on_arm_state()``
for real - see README's "Explicit assumptions" for the full incident,
including why it took multiple sessions to actually trace (the
resulting exception was uncaught inside server.py's own single,
combined-snapshot asyncio telemetry task, silently killing telemetry
for every board at once, with nothing the browser's own console would
ever show).

Doesn't import ros_bridge.py or rclpy at all - both bring in a real
ROS 2 install this test suite is designed to run without (see this
package's own historical absence of any test/ contents until this
file). Tests the underlying principle directly instead, with real
numpy arrays standing in for what rclpy actually produces, which is
enough to pin the exact failure mode and its fix without needing the
full ROS stack running.
"""

import json

import pytest

np = pytest.importorskip("numpy", reason="numpy isn't a direct rover_web_gui dependency - only pulled in transitively via rclpy, which this test suite runs without")


def test_bare_list_of_numpy_int32_is_not_json_serializable():
    # This is the ORIGINAL bug, reproduced directly - list(msg.field) on
    # a real ROS int32[] field produces exactly this.
    fake_ros_array = np.array([100, -200, 300], dtype=np.int32)
    broken = list(fake_ros_array)
    with pytest.raises(TypeError):
        json.dumps({"encoder_ticks": broken})


def test_bare_list_of_numpy_bool_is_not_json_serializable():
    # Same underlying issue, the bool[] version - ArmState's own
    # limit_switch_triggered/joint_homed are exactly this shape.
    fake_ros_array = np.array([True, False, True], dtype=np.bool_)
    broken = list(fake_ros_array)
    with pytest.raises(TypeError):
        json.dumps({"joint_homed": broken})


def test_per_element_int_cast_fixes_serialization():
    # This is the actual fix applied in ros_bridge.py's own
    # _on_base_state()/_on_arm_state(): [int(t) for t in ...], not
    # list(...) directly.
    fake_ros_array = np.array([100, -200, 300], dtype=np.int32)
    fixed = [int(t) for t in fake_ros_array]
    result = json.dumps({"encoder_ticks": fixed})
    assert json.loads(result)["encoder_ticks"] == [100, -200, 300]


def test_per_element_bool_cast_fixes_serialization():
    fake_ros_array = np.array([True, False, True], dtype=np.bool_)
    fixed = [bool(t) for t in fake_ros_array]
    result = json.dumps({"joint_homed": fixed})
    assert json.loads(result)["joint_homed"] == [True, False, True]


def test_scalar_numpy_fields_are_not_affected():
    # Only fixed-size ARRAY fields hit this - a scalar numpy value
    # (which ROS message fields for a single int/float/bool are NOT
    # backed by; only the array case is) would be a different bug
    # entirely if it ever occurred. This test documents that
    # distinction rather than assumes it - plain Python int/bool
    # serialize fine on their own, no cast needed.
    assert json.dumps({"supply_voltage_mv": 12600}) == '{"supply_voltage_mv": 12600}'
    assert json.dumps({"homed": True}) == '{"homed": true}'
