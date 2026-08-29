"""Mast (Uno #3) serial message layer, built on rover_protocol.framing.

Frame types
-----------
  'M' (host -> Uno3): mast command
      fields = [head_yaw_decideg, head_pitch_decideg, lift_mode, driver_enable]
      lift_mode: -1 = stow (drive toward the horizontal/transport limit
                        switch), 0 = hold (stop the lift motor where it
                        is), 1 = erect (drive toward the vertical/
                        service limit switch)
      driver_enable: 1 to energize the yaw/pitch TB6600 drivers, 0 to
                        de-energize them. Only takes effect once homed
                        is true - see mast_uno3.ino's
                        handleMastCommand() for why applying this
                        unconditionally, the way ArmCommand's enable
                        field works, would strand a stepper mid-seek
                        or mid-move-to-zero instead.
      head_yaw_decideg/head_pitch_decideg are ignored by the firmware
      (not silently accepted-but-wrong) until homed is true - see 'Z'
      below for what that actually requires now (reaching true zero,
      not just triggering a limit switch) - the same convention
      rover_arm's 'A' frame uses for homing alone.
      lift_mode always applies immediately regardless of homing state:
      the lift has no step-relative position to home, just directly-
      read limit switches.

  'Z' (host -> Uno3): home request, no fields. Starts the yaw/pitch
      calibration sequence (mirrors rover_arm's own 'Z' frame) - seeks
      each axis toward its limit switch in turn. Each switch is
      physically at that axis's minimum (most-negative) bound, not
      its center, so triggering it sets the axis's position to that
      minimum (-170 deg yaw, -180 deg pitch - real, bench-confirmed
      values), not zero - then both axes drive from there to true
      zero (center of travel). Only once both arrive there does
      "home" actually get established (see 'S' below) and the drivers
      disable themselves. Doesn't touch the lift at any point.

  'S' (Uno3 -> host): mast state
      fields = [head_yaw_decideg, head_pitch_decideg, lift_state,
                yaw_limit_triggered, pitch_limit_triggered, homed,
                voltage_mv, driver_enabled, temperature_deci_c,
                fan_duty_percent]
      lift_state: 0 = unknown, 1 = transport (at the stowed limit),
                  2 = service (at the erect limit), 3 = moving
      yaw_limit_triggered/pitch_limit_triggered: 1 if that axis's
                  calibration switch is currently triggered, else 0
      homed: 1 once both axes have actually arrived at true zero -
                  not the instant their limit switches trigger (that
                  position is each axis's minimum, not home; see 'Z'
                  above). Movement and enable/disable commands are
                  both ignored by the firmware until this is 1.
      voltage_mv: main battery supply voltage in millivolts, from an
                  FZ0430 sensor (5:1 resistive divider) on analog pin
                  A0 - see mast_uno3.ino's readSupplyVoltageMv() for
                  the conversion math.
      driver_enabled: 1 if the yaw/pitch TB6600 drivers are currently
                  energized, else 0. Starts true (homing needs them
                  energized) and goes false automatically once the
                  post-calibration sequence completes, or whenever
                  explicitly disabled via a driver_enable=0 command.
      temperature_deci_c: board/enclosure temperature in tenths of a
                  degree Celsius, from a DS18B20 on 1-Wire (Uno pin
                  A4) - -9999 ("sensor not found") if the sensor
                  didn't respond on its most recent read - checked
                  fresh every cycle, not just once at boot; see
                  base_mega1.ino's own header comment for the pull-up
                  requirement this depends on.
      fan_duty_percent: 0-100, the cooling fan's current PWM duty
                  cycle - entirely automatic, a function of
                  temperature_deci_c (see mast_uno3.ino's
                  updateFanControl()), never operator-commanded, so
                  there's no corresponding field on 'M' above. 0 means
                  the fan is off, not necessarily that it's broken -
                  see mast_uno3.ino's kFanOnTempDeciC/kFanOffTempDeciC
                  for the thermostat thresholds.
"""

from __future__ import annotations

from typing import List, Tuple

from rover_protocol.framing import RoverFrameError, decode_frame, encode_frame

LIFT_STOW = -1
LIFT_HOLD = 0
LIFT_ERECT = 1

STATE_UNKNOWN = 0
STATE_TRANSPORT = 1
STATE_SERVICE = 2
STATE_MOVING = 3


def encode_mast_command(
    head_yaw_decideg: int, head_pitch_decideg: int, lift_mode: int, driver_enable: bool
) -> str:
    if lift_mode not in (LIFT_STOW, LIFT_HOLD, LIFT_ERECT):
        raise RoverFrameError(f"lift_mode must be -1/0/1, got {lift_mode}")
    return encode_frame(
        "M",
        [
            int(head_yaw_decideg),
            int(head_pitch_decideg),
            int(lift_mode),
            1 if driver_enable else 0,
        ],
    )


def encode_home_request() -> str:
    return encode_frame("Z", [])


def decode_line(line) -> Tuple[str, List[int]]:
    return decode_frame(line)


def parse_mast_state(fields: List[int]):
    """Returns (head_yaw_decideg, head_pitch_decideg, lift_state,
    yaw_limit_triggered, pitch_limit_triggered, homed, voltage_mv,
    driver_enabled, temperature_deci_c, fan_duty_percent).
    """
    if len(fields) != 10:
        raise RoverFrameError(f"'S' frame expected 10 fields, got {len(fields)}")
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
    ) = fields
    if lift_state not in (STATE_UNKNOWN, STATE_TRANSPORT, STATE_SERVICE, STATE_MOVING):
        raise RoverFrameError(f"unknown lift_state value {lift_state}")
    return (
        yaw,
        pitch,
        lift_state,
        bool(yaw_limit),
        bool(pitch_limit),
        bool(homed),
        voltage_mv,
        bool(driver_enabled),
        temperature_deci_c,
        fan_duty_percent,
    )
