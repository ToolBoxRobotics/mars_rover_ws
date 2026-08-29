"""Base (Mega #1) serial message layer, built on rover_protocol.framing.

Frame types
-----------
  'D' (host -> Mega1): drive command
      fields = [w_fl, w_fr, w_ml, w_mr, w_rl, w_rr, s_fl, s_fr, s_rl, s_rr]
      w_*  = wheel throttle, -1000..1000 (maps to PWM + direction pins on
             the DRI0002 drivers) - all 6 wheels are driven
      s_*  = corner steering angle in decidegrees (tenths of a degree),
             sent to the servos (via a PCA9685 16-channel I2C PWM
             driver, address 0x40 - see base_mega1.ino's own header
             comment for the full history; no longer direct Mega pins)
             for the 4 corner wheels
             (ML/MR are fixed and have no steer field)

  'E' (Mega1 -> host): encoder state, ML and MR only, plus two supply
      voltages, board temperature, and cooling fan speed
      fields = [e_ml, e_mr, drive_voltage_mv, steering_voltage_mv,
                temperature_deci_c, fan_duty_percent]
      e_*  = cumulative ticks. Only the two fixed middle wheels are
             physically encoded - see rover_base/odometry.py for why
             they're the only pair useful for wheel odometry (their
             rolling axis never changes with corner steering, unlike
             FL/FR/RL/RR). Wiring individual encoders on the 4
             steerable corners would add hardware complexity for data
             nothing consumes.
      drive_voltage_mv = the drive motors' own supply rail voltage in
             millivolts, from an FZ0430 sensor (5:1 resistive divider)
             on analog pin A0 - see base_mega1.ino's readVoltageMv()
             for the conversion math (shared with steering_voltage_mv
             below; same function, different pin argument).
      steering_voltage_mv = the steering servos' own supply rail
             voltage in millivolts, from a second, identically-
             configured FZ0430 on analog pin A1 - added specifically
             to give the two rails independent readings rather than
             one shared value covering both; see base_mega1.ino's own
             header comment for the full reasoning.
      temperature_deci_c = board/enclosure temperature in tenths of a
             degree Celsius, from a DS18B20 on 1-Wire (Mega pin 20) -
             -9999 ("sensor not found") if the sensor didn't respond
             on its most recent read - checked fresh every cycle, not
             just once at boot; see base_mega1.ino's own header
             comment for the pull-up requirement this depends on.
      fan_duty_percent = 0-100, cooling fan PWM duty cycle - entirely
             automatic, a function of temperature_deci_c (see
             base_mega1.ino's updateFanControl()), never operator-
             commanded, so there's no corresponding field on 'D'
             above. 0 means the fan is currently off, not necessarily
             broken.

  'H' (either direction): heartbeat / keep-alive, no fields. The bridge
      sends this instead of 'D' when there is no fresh cmd_vel, so the
      Mega's watchdog (see firmware/base_mega1) knows the link is alive
      without re-issuing a drive command.
"""

from __future__ import annotations

from typing import List, Tuple

from rover_protocol.framing import RoverFrameError, decode_frame, encode_frame

NUM_WHEELS = 6
NUM_STEER = 4
NUM_ENCODERS = 2  # ML, MR only - see the 'E' frame docs above


def encode_drive(wheel_throttle: List[int], steer_decideg: List[int]) -> str:
    if len(wheel_throttle) != NUM_WHEELS:
        raise RoverFrameError(f"expected {NUM_WHEELS} wheel throttles, got {len(wheel_throttle)}")
    if len(steer_decideg) != NUM_STEER:
        raise RoverFrameError(f"expected {NUM_STEER} steer angles, got {len(steer_decideg)}")
    # Defensive int() cast - see arm_protocol.encode_joint_command's
    # comment for why; not currently exercised here (base_bridge_node
    # always computes fresh Python ints via kinematics.py rather than
    # reading an array field off a ROS message), but consistent with
    # the other three protocol modules rather than leaving this one
    # the odd one out.
    return encode_frame("D", [int(t) for t in wheel_throttle] + [int(s) for s in steer_decideg])


def encode_heartbeat() -> str:
    return encode_frame("H", [])


def decode_line(line) -> Tuple[str, List[int]]:
    """Decode any line from the base Mega. Raises RoverFrameError on bad frames."""
    return decode_frame(line)


def parse_encoder_state(fields: List[int]) -> Tuple[List[int], int, int, int, int]:
    """Returns (encoder_ticks[2], drive_voltage_mv, steering_voltage_mv, temperature_deci_c, fan_duty_percent)."""
    expected = NUM_ENCODERS + 4
    if len(fields) != expected:
        raise RoverFrameError(f"'E' frame expected {expected} fields, got {len(fields)}")
    return (
        list(fields[:NUM_ENCODERS]),
        fields[NUM_ENCODERS],
        fields[NUM_ENCODERS + 1],
        fields[NUM_ENCODERS + 2],
        fields[NUM_ENCODERS + 3],
    )
