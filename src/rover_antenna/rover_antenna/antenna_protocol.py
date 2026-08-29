"""Antenna gimbal (Uno #5) serial message layer, built on
rover_protocol.framing.

Frame types
-----------
  'G' (host -> Uno5): gimbal command
      fields = [azimuth_decideg, elevation_decideg, driver_enable]
      azimuth_decideg/elevation_decideg = target position, decidegrees
                  (tenths of a degree). Ignored by the firmware
                  (not silently accepted-but-wrong) until homed is
                  true - see 'Z' below - the same convention
                  rover_arm's 'A' frame and rover_mast's 'M' frame use.
      driver_enable = 1 to energize the shared TB6600 enable pin, 0 to
                  de-energize it. Only takes effect once homed is
                  true, for the same reason MastCommand's own
                  driver_enable field works this way - see
                  antenna_uno5.ino's handleGimbalCommand().

  'Z' (host -> Uno5): home request, no fields. Starts the azimuth/
      elevation calibration sequence (mirrors rover_arm's and
      rover_mast's own 'Z' frames) - seeks each axis toward its limit
      switch in turn. Unlike the mast, each switch here sits at that
      axis's own operational minimum (not offset from a separately-
      centered zero), so triggering it directly establishes "home" -
      no follow-on move-to-a-different-reference step needed.

  'S' (Uno5 -> host): gimbal state
      fields = [azimuth_decideg, elevation_decideg,
                azimuth_limit_triggered, elevation_limit_triggered,
                homed, voltage_mv, driver_enabled, temperature_deci_c,
                fan_duty_percent]
      azimuth_limit_triggered/elevation_limit_triggered: 1 if that
                  axis's calibration switch is currently triggered,
                  else 0
      homed: 1 once both axes have found their calibration switch
      voltage_mv: main battery supply voltage in millivolts, from an
                  FZ0430 sensor (5:1 resistive divider) on analog pin
                  A0 - see antenna_uno5.ino's readSupplyVoltageMv()
                  for the conversion math.
      driver_enabled: 1 if the shared TB6600 enable pin is currently
                  energized, else 0. Starts true (homing needs it
                  energized).
      temperature_deci_c: board/enclosure temperature in tenths of a
                  degree Celsius, from a DS18B20 on 1-Wire (Uno pin
                  A4) - -9999 ("sensor not found") if the sensor
                  didn't respond on its most recent read - checked
                  fresh every cycle, not just once at boot; see
                  base_mega1.ino's own header comment for the pull-up
                  requirement this depends on.
      fan_duty_percent: 0-100, cooling fan PWM duty cycle - entirely
                  automatic, a function of temperature_deci_c (see
                  antenna_uno5.ino's updateFanControl()), never
                  operator-commanded, so there's no corresponding
                  field on 'G' above. 0 means the fan is currently
                  off, not necessarily broken.
"""

from __future__ import annotations

from typing import List, Tuple

from rover_protocol.framing import RoverFrameError, decode_frame, encode_frame


def encode_gimbal_command(azimuth_decideg: int, elevation_decideg: int, driver_enable: bool) -> str:
    return encode_frame(
        "G",
        [int(azimuth_decideg), int(elevation_decideg), 1 if driver_enable else 0],
    )


def encode_home_request() -> str:
    return encode_frame("Z", [])


def decode_line(line) -> Tuple[str, List[int]]:
    return decode_frame(line)


def parse_gimbal_state(fields: List[int]):
    """Returns (azimuth_decideg, elevation_decideg, azimuth_limit_triggered,
    elevation_limit_triggered, homed, voltage_mv, driver_enabled, temperature_deci_c, fan_duty_percent).
    """
    if len(fields) != 9:
        raise RoverFrameError(f"'S' frame expected 9 fields, got {len(fields)}")
    (
        azimuth,
        elevation,
        azimuth_limit,
        elevation_limit,
        homed,
        voltage_mv,
        driver_enabled,
        temperature_deci_c,
        fan_duty_percent,
    ) = fields
    return (
        azimuth,
        elevation,
        bool(azimuth_limit),
        bool(elevation_limit),
        bool(homed),
        voltage_mv,
        bool(driver_enabled),
        temperature_deci_c,
        fan_duty_percent,
    )
