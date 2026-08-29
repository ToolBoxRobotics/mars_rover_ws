"""Arm (Mega #2) serial message layer, built on rover_protocol.framing.

Frame types
-----------
  'A' (host -> Mega2): joint-space position command
      fields = [j1, j2, j3, j4, j5, enable]
      j*     = absolute target position in motor steps, relative to
               each joint's homed zero
      enable = 1 to energize all five drivers (3x TB6600, 2x A4988),
               0 to disable them (free-spin, e.g. before manually
               stowing the arm)
      Ignored entirely (not queued) unless every joint is individually
      homed, no homing run is currently in progress, AND no emergency
      stop is currently latched - see arm_mega2.ino's
      handleJointCommand() for why partial-homed motion isn't
      supported, and handleEmergencyStop() for the e-stop gate.

  'Z' (host -> Mega2): trigger the calibration-switch homing sequence.
      fields = [joint_index]
      joint_index = -1 to home all 5 joints in the order
               arm_mega2.ino's own kHomingOrder specifies (originally
               always J1 through J5; now independently configurable
               there, see that constant's own comment), or 0-4 to home
               just that one joint on its own, leaving every other
               joint's homed status and position untouched. Ignored if
               a homing run is already in progress. Each joint's own
               homing DIRECTION (kHomingDirection) and its post-limit-
               switch OFFSET to its actual defined zero
               (kHomingOffsetSteps) are also independently configurable
               firmware-side constants now - both are set entirely in
               arm_mega2.ino, not sent over the wire at all, since
               they're physical-calibration facts about the hardware
               itself, not something an operator chooses per homing
               run.

  'P' (host -> Mega2): move to a predefined pose
      fields = [preset]
      preset = 0 (PRESET_INITIAL), 1 (PRESET_TRANSPORT), or 2
               (PRESET_SERVICE) - see encode_preset_request()'s own
               module-level constants. The actual joint-angle targets
               for each preset live in arm_mega2.ino as fixed
               constants (kInitialPoseSteps/kTransportPoseSteps/
               kServicePoseSteps), not sent over the wire - this frame
               only selects which one. Same "fully homed, not
               mid-homing, not e-stopped" gate as a regular 'A' command
               - see arm_mega2.ino's handlePresetRequest().

  'X' (host -> Mega2): emergency stop, latching
      fields = [engage]
      engage = 1 to latch the e-stop (immediate controlled stop on
               every joint - see arm_mega2.ino's handleEmergencyStop()
               for why this is a fast, accel-profiled deceleration via
               AccelStepper's own stop(), not an instantaneous halt),
               0 to clear it. While latched, blocks every source of
               new movement ('A' and 'P' above) until explicitly
               cleared - the firmware itself is the source of truth
               for this, not whatever's upstream sending this frame,
               specifically so a bridge-node restart or hiccup after
               an e-stop can't silently resume motion. Does NOT
               de-energize the drivers - a deliberate departure from
               conventional power-cutting e-stop behavior, since this
               is a gravity-loaded arm where an uncontrolled drop from
               de-energizing mid-air is judged the worse outcome; see
               arm_mega2.ino's own header comment and
               handleEmergencyStop() for the full reasoning.

  'S' (Mega2 -> host): joint state
      fields = [j1, j2, j3, j4, j5, lim1, lim2, lim3, lim4, lim5,
                homed1, homed2, homed3, homed4, homed5, voltage_mv,
                temperature_deci_c, fan_duty_percent, estop_active]
      j*     = current position in steps
      lim*   = 1 if that joint's calibration limit switch is currently
               triggered, else 0
      homed* = 1 once that specific joint's homing has completed
               successfully - per-joint now, not one shared flag,
               since a joint can be individually re-homed without
               touching the others.
      voltage_mv = main battery supply voltage in millivolts, from an
               FZ0430 sensor (5:1 resistive divider) on analog pin A0 -
               see arm_mega2.ino's readSupplyVoltageMv() for the
               conversion math.
      temperature_deci_c = board/enclosure temperature in tenths of a
               degree Celsius, from a DS18B20 on 1-Wire (Mega pin 20) -
               -9999 ("sensor not found") if the sensor didn't respond
               on its most recent read - checked fresh every cycle, not
               just once at boot; see base_mega1.ino's own header
               comment for the pull-up requirement this depends on.
      fan_duty_percent = 0-100, cooling fan PWM duty cycle - entirely
               automatic, a function of temperature_deci_c (see
               arm_mega2.ino's updateFanControl()), never operator-
               commanded, so there's no corresponding field on 'A'
               above. 0 means the fan is currently off, not
               necessarily broken.
      estop_active = 1 if the emergency stop is currently latched
               (see the 'X' frame above), else 0 - the firmware's own
               authoritative state, not inferred from anything the
               host last sent.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from rover_protocol.framing import RoverFrameError, decode_frame, encode_frame

NUM_JOINTS = 5


def encode_joint_command(joint_target_steps: List[int], enable: bool) -> str:
    if len(joint_target_steps) != NUM_JOINTS:
        raise RoverFrameError(f"expected {NUM_JOINTS} joint targets, got {len(joint_target_steps)}")
    # Defensive int() cast, matching mast_protocol.encode_mast_command
    # and microscope_protocol's encode function - guards against
    # numpy.int32 (or similar int-valued-but-not-int-typed) elements
    # slipping in from a caller that read them straight off a ROS
    # message array field without converting first. See
    # arm_bridge_node.py's own comment on this for the actual story.
    return encode_frame("A", [int(t) for t in joint_target_steps] + [1 if enable else 0])


def encode_home_request(joint_index: Optional[int] = None) -> str:
    """joint_index=None (or -1) homes all 5 joints in sequence; 0-4
    homes just that one joint.
    """
    index = -1 if joint_index is None else int(joint_index)
    if index < -1 or index >= NUM_JOINTS:
        raise RoverFrameError(f"joint_index must be -1..{NUM_JOINTS - 1} or None, got {joint_index}")
    return encode_frame("Z", [index])


# Preset indices - must match arm_mega2.ino's own handlePresetRequest()
# exactly (0=INITIAL, 1=TRANSPORT, 2=SERVICE); kept as named constants
# here rather than left as bare 0/1/2 at every call site.
PRESET_INITIAL = 0
PRESET_TRANSPORT = 1
PRESET_SERVICE = 2
_VALID_PRESETS = (PRESET_INITIAL, PRESET_TRANSPORT, PRESET_SERVICE)


def encode_preset_request(preset: int) -> str:
    """preset: PRESET_INITIAL, PRESET_TRANSPORT, or PRESET_SERVICE.
    Firmware-side pose is a fixed constant (see arm_mega2.ino's own
    kInitialPoseSteps/kTransportPoseSteps/kServicePoseSteps) - this
    just selects which one, not the joint targets themselves.
    """
    preset = int(preset)
    if preset not in _VALID_PRESETS:
        raise RoverFrameError(f"preset must be one of {_VALID_PRESETS}, got {preset}")
    return encode_frame("P", [preset])


def encode_emergency_stop(engage: bool) -> str:
    """engage=True latches the e-stop (immediate controlled stop on
    every joint, drivers left energized - see arm_mega2.ino's own
    handleEmergencyStop() for the full reasoning); engage=False clears
    it, allowing new joint/preset commands again.
    """
    return encode_frame("X", [1 if engage else 0])


def decode_line(line) -> Tuple[str, List[int]]:
    return decode_frame(line)


def parse_joint_state(fields: List[int]):
    """Returns (positions[5], limit_switches[5]bool, joint_homed[5]bool, voltage_mv:int, temperature_deci_c:int, fan_duty_percent:int, estop_active:bool)."""
    expected = NUM_JOINTS * 3 + 4
    if len(fields) != expected:
        raise RoverFrameError(f"'S' frame expected {expected} fields, got {len(fields)}")
    positions = fields[0:NUM_JOINTS]
    limits = [bool(v) for v in fields[NUM_JOINTS : 2 * NUM_JOINTS]]
    joint_homed = [bool(v) for v in fields[2 * NUM_JOINTS : 3 * NUM_JOINTS]]
    voltage_mv = fields[3 * NUM_JOINTS]
    temperature_deci_c = fields[3 * NUM_JOINTS + 1]
    fan_duty_percent = fields[3 * NUM_JOINTS + 2]
    estop_active = bool(fields[3 * NUM_JOINTS + 3])
    return positions, limits, joint_homed, voltage_mv, temperature_deci_c, fan_duty_percent, estop_active
