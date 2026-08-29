"""Pure Xbox-360-to-rover-command mapping logic, kept free of any
rclpy/sensor_msgs dependency so it can be unit tested with plain
Python lists standing in for a sensor_msgs/Joy message's axes/buttons
arrays.

A single gamepad does not have enough independent axes to drive the
base, arm, mast, microscope, and antenna gimbal all at once, so
control is split into five SUBSYSTEM modes cycled with one button (LB
by default):

    DRIVE -> ARM -> MAST -> MICROSCOPE -> ANTENNA -> DRIVE -> ...

Within DRIVE mode specifically, the base's steering geometry is a
second, independent selection (X/Y by default, unused elsewhere in
DRIVE mode until now):

    X: toggle ACKERMANN <-> POINT_TURN
    Y: force STOP immediately (one-way - resuming always takes an
       explicit X press, not a toggle that could bump back off)

See DriveGeometryMode / DriveGeometrySwitcher below. This local enum's
integer values are kept in sync with rover_msgs/DriveMode.msg's
constants by hand (deliberately not importing rover_msgs here, so
this module keeps testing with plain `pytest` and no colcon build).

A dedicated deadman button (RB by default) must be held for ANY
command to leave neutral/zero/hold, in every mode - releasing it is
always safe.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Sequence, Tuple


class Mode(IntEnum):
    DRIVE = 0
    ARM = 1
    MAST = 2
    MICROSCOPE = 3
    ANTENNA = 4


def next_mode(mode: Mode) -> Mode:
    return Mode((int(mode) + 1) % len(Mode))


def apply_deadzone(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def _safe_axis(axes: Sequence[float], index: int) -> float:
    return axes[index] if 0 <= index < len(axes) else 0.0


def _safe_button(buttons: Sequence[int], index: int) -> bool:
    return bool(buttons[index]) if 0 <= index < len(buttons) else False


@dataclass(frozen=True)
class TeleopConfig:
    deadzone: float = 0.12

    axis_left_x: int = 0
    axis_left_y: int = 1
    axis_left_trigger: int = 2
    axis_right_x: int = 3
    axis_right_y: int = 4
    axis_right_trigger: int = 5

    button_a: int = 0
    button_b: int = 1
    button_x: int = 2
    button_y: int = 3
    button_lb: int = 4
    button_rb: int = 5
    button_back: int = 6
    button_start: int = 7

    invert_left_y: bool = False
    invert_right_y: bool = False

    max_linear_mps: float = 0.65
    max_angular_radps: float = 1.5

    arm_jog_steps_per_sec: float = 800.0

    max_head_yaw_deg: float = 170.0
    max_head_pitch_deg: float = 180.0

    microscope_focus_jog_steps_per_sec: float = 400.0

    # ANTENNA mode: azimuth/elevation are jogged (like the arm), not
    # positioned absolutely (like the mast) - the antenna's real
    # operational range (15-285 deg azimuth, 0-180 deg elevation) isn't
    # centered around 0 the way the mast's is, so mapping stick
    # deflection straight to an absolute angle the way compute_mast_command
    # does would be a much less natural control feel here. Degrees, not
    # steps, matching the wire protocol's own units for this axis.
    antenna_jog_deg_per_sec: float = 30.0


def deadman_engaged(buttons: Sequence[int], cfg: TeleopConfig) -> bool:
    return _safe_button(buttons, cfg.button_rb)


class ModeSwitcher:
    """Edge-triggers on the mode button so a held press doesn't cycle
    through every mode in one frame.
    """

    def __init__(self, initial: Mode = Mode.DRIVE) -> None:
        self.mode = initial
        self._button_was_down = False

    def update(self, buttons: Sequence[int], cfg: TeleopConfig) -> Mode:
        is_down = _safe_button(buttons, cfg.button_lb)
        if is_down and not self._button_was_down:
            self.mode = next_mode(self.mode)
        self._button_was_down = is_down
        return self.mode


class DriveGeometryMode(IntEnum):
    """Mirrors rover_msgs/DriveMode.msg's constants - keep numerically
    identical if either changes.
    """

    ACKERMANN = 0
    POINT_TURN = 1
    STOP = 2


_DRIVE_GEOMETRY_CYCLE = (DriveGeometryMode.ACKERMANN, DriveGeometryMode.POINT_TURN)


def next_drive_geometry_mode(mode: DriveGeometryMode) -> DriveGeometryMode:
    """Toggles ACKERMANN <-> POINT_TURN. STOP is intentionally not part
    of this cycle: toggling while stopped resumes at ACKERMANN rather
    than silently skipping over the stop.
    """
    if mode not in _DRIVE_GEOMETRY_CYCLE:
        return DriveGeometryMode.ACKERMANN
    idx = _DRIVE_GEOMETRY_CYCLE.index(mode)
    return _DRIVE_GEOMETRY_CYCLE[(idx + 1) % len(_DRIVE_GEOMETRY_CYCLE)]


class DriveGeometrySwitcher:
    """Edge-triggers on two buttons: X toggles ACKERMANN/POINT_TURN
    (and resumes from STOP into ACKERMANN), Y unconditionally forces
    STOP. Y is deliberately one-way - resuming from STOP always takes
    an explicit X press rather than a toggle that could bump back off
    by accident.
    """

    def __init__(self, initial: DriveGeometryMode = DriveGeometryMode.ACKERMANN) -> None:
        self.mode = initial
        self._x_was_down = False
        self._y_was_down = False

    def update(self, buttons: Sequence[int], cfg: TeleopConfig) -> DriveGeometryMode:
        x_down = _safe_button(buttons, cfg.button_x)
        y_down = _safe_button(buttons, cfg.button_y)

        if y_down and not self._y_was_down:
            self.mode = DriveGeometryMode.STOP
        elif x_down and not self._x_was_down:
            self.mode = next_drive_geometry_mode(self.mode)

        self._x_was_down = x_down
        self._y_was_down = y_down
        return self.mode


def compute_drive_twist(axes: Sequence[float], cfg: TeleopConfig) -> Tuple[float, float]:
    """ACKERMANN mode. Returns (linear_x_mps, angular_z_radps). Left
    stick: Y = throttle, X = turn.
    """
    ly = apply_deadzone(_safe_axis(axes, cfg.axis_left_y), cfg.deadzone)
    lx = apply_deadzone(_safe_axis(axes, cfg.axis_left_x), cfg.deadzone)
    if cfg.invert_left_y:
        ly = -ly
    linear_x = ly * cfg.max_linear_mps
    angular_z = lx * cfg.max_angular_radps
    return linear_x, angular_z


def compute_point_turn_rate(axes: Sequence[float], cfg: TeleopConfig) -> float:
    """POINT_TURN mode. Returns angular_z_radps from left stick X only -
    there is no forward component in a pivot, so Y is ignored entirely
    (unlike ACKERMANN, which uses the full stick).
    """
    lx = apply_deadzone(_safe_axis(axes, cfg.axis_left_x), cfg.deadzone)
    return lx * cfg.max_angular_radps


def compute_arm_jog(
    axes: Sequence[float],
    cfg: TeleopConfig,
    dt_sec: float,
    current_targets_steps: Sequence[int],
) -> List[int]:
    """Jogs all 5 joints from the current targets: left stick -> J1/J2,
    right stick -> J3/J4, trigger pair -> J5. Triggers on a typical
    xpad mapping rest at -1 (unpressed) and move to +1 (fully pressed);
    we take (right_trigger - left_trigger) so either one alone jogs J5
    in one direction while both together cancel out.
    """
    targets = list(current_targets_steps)
    if len(targets) != 5:
        raise ValueError(f"expected 5 current joint targets, got {len(targets)}")

    j1 = apply_deadzone(_safe_axis(axes, cfg.axis_left_x), cfg.deadzone)
    j2 = apply_deadzone(_safe_axis(axes, cfg.axis_left_y), cfg.deadzone)
    j3 = apply_deadzone(_safe_axis(axes, cfg.axis_right_x), cfg.deadzone)
    j4 = apply_deadzone(_safe_axis(axes, cfg.axis_right_y), cfg.deadzone)

    lt = _safe_axis(axes, cfg.axis_left_trigger)
    rt = _safe_axis(axes, cfg.axis_right_trigger)
    j5 = apply_deadzone((rt - lt) / 2.0, cfg.deadzone)

    jog = cfg.arm_jog_steps_per_sec * dt_sec
    deltas = [j1, j2, j3, j4, j5]
    return [int(round(t + d * jog)) for t, d in zip(targets, deltas)]


def compute_mast_command(axes: Sequence[float], buttons: Sequence[int], cfg: TeleopConfig) -> Tuple[int, int, int]:
    """Right stick sets an absolute head yaw/pitch target (proportional
    to deflection, not a jog). A = erect, B = stow, otherwise hold.
    Returns (head_yaw_decideg, head_pitch_decideg, lift_mode).
    """
    rx = apply_deadzone(_safe_axis(axes, cfg.axis_right_x), cfg.deadzone)
    ry = apply_deadzone(_safe_axis(axes, cfg.axis_right_y), cfg.deadzone)
    if cfg.invert_right_y:
        ry = -ry

    yaw_decideg = int(round(rx * cfg.max_head_yaw_deg * 10))
    pitch_decideg = int(round(ry * cfg.max_head_pitch_deg * 10))

    if _safe_button(buttons, cfg.button_a):
        lift_mode = 1  # erect / service
    elif _safe_button(buttons, cfg.button_b):
        lift_mode = -1  # stow / transport
    else:
        lift_mode = 0  # hold

    return yaw_decideg, pitch_decideg, lift_mode


@dataclass
class MicroscopeJogState:
    focus_steps: int = 0
    cover_open: bool = False
    _button_a_was_down: bool = field(default=False, repr=False)


def compute_microscope_command(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: TeleopConfig,
    dt_sec: float,
    state: MicroscopeJogState,
) -> Tuple[int, int, bool]:
    """Left stick Y jogs focus/zoom. Right trigger sets LED brightness
    (0 unpressed -> 255 fully pressed). A toggles the lens cover
    (edge-triggered). Mutates `state` in place and also returns the
    resulting (focus_steps, led_pwm, cover_open).
    """
    ly = apply_deadzone(_safe_axis(axes, cfg.axis_left_y), cfg.deadzone)
    if cfg.invert_left_y:
        ly = -ly
    state.focus_steps += int(round(ly * cfg.microscope_focus_jog_steps_per_sec * dt_sec))

    rt = _safe_axis(axes, cfg.axis_right_trigger)
    # Typical xpad resting value is -1.0 (unpressed) .. +1.0 (fully pressed).
    led_frac = max(0.0, min(1.0, (rt + 1.0) / 2.0))
    led_pwm = int(round(led_frac * 255))

    a_down = _safe_button(buttons, cfg.button_a)
    if a_down and not state._button_a_was_down:
        state.cover_open = not state.cover_open
    state._button_a_was_down = a_down

    return state.focus_steps, led_pwm, state.cover_open


def compute_antenna_jog(
    axes: Sequence[float],
    cfg: TeleopConfig,
    dt_sec: float,
    current_azimuth_decideg: int,
    current_elevation_decideg: int,
) -> Tuple[int, int]:
    """Left stick jogs the antenna gimbal from its current targets:
    X -> azimuth (G1), Y -> elevation (G2). No client-side clamping to
    the antenna's real 15-285/0-180 deg operational range - same as
    compute_arm_jog, this relies entirely on the firmware's own
    constrain() to reject/clip out-of-range targets, rather than
    duplicating that range here and risking the two drifting apart.
    Returns (azimuth_decideg, elevation_decideg).
    """
    lx = apply_deadzone(_safe_axis(axes, cfg.axis_left_x), cfg.deadzone)
    ly = apply_deadzone(_safe_axis(axes, cfg.axis_left_y), cfg.deadzone)
    if cfg.invert_left_y:
        ly = -ly

    jog_decideg = cfg.antenna_jog_deg_per_sec * 10.0 * dt_sec
    azimuth_decideg = int(round(current_azimuth_decideg + lx * jog_decideg))
    elevation_decideg = int(round(current_elevation_decideg + ly * jog_decideg))
    return azimuth_decideg, elevation_decideg
