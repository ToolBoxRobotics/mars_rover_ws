import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_teleop.joy_mapping import (
    DriveGeometryMode,
    DriveGeometrySwitcher,
    Mode,
    ModeSwitcher,
    MicroscopeJogState,
    TeleopConfig,
    apply_deadzone,
    compute_antenna_jog,
    compute_arm_jog,
    compute_drive_twist,
    compute_mast_command,
    compute_microscope_command,
    compute_point_turn_rate,
    deadman_engaged,
    next_drive_geometry_mode,
    next_mode,
)

CFG = TeleopConfig()


def axes(**overrides):
    """Build a 6-element axes array (matching CFG's default indices),
    all zero except overrides, e.g. axes(axis_left_y=1.0).
    """
    values = [0.0] * 6
    names = ["axis_left_x", "axis_left_y", "axis_left_trigger", "axis_right_x", "axis_right_y", "axis_right_trigger"]
    for key, value in overrides.items():
        values[names.index(key)] = value
    return values


def buttons(**overrides):
    values = [0] * 8
    names = ["button_a", "button_b", "button_x", "button_y", "button_lb", "button_rb", "button_back", "button_start"]
    for key, value in overrides.items():
        values[names.index(key)] = value
    return values


# -- deadzone / mode cycling -------------------------------------------------
def test_deadzone_zeros_small_values():
    assert apply_deadzone(0.05, 0.12) == 0.0
    assert apply_deadzone(-0.05, 0.12) == 0.0


def test_deadzone_passes_through_large_values():
    assert apply_deadzone(0.5, 0.12) == 0.5


def test_next_mode_cycles_through_all_five_and_wraps():
    m = Mode.DRIVE
    seen = [m]
    for _ in range(5):
        m = next_mode(m)
        seen.append(m)
    assert seen == [Mode.DRIVE, Mode.ARM, Mode.MAST, Mode.MICROSCOPE, Mode.ANTENNA, Mode.DRIVE]


def test_mode_switcher_only_advances_on_press_edge_not_while_held():
    switcher = ModeSwitcher()
    assert switcher.mode == Mode.DRIVE
    # button held down across two updates should only advance once
    switcher.update(buttons(button_lb=1), CFG)
    assert switcher.mode == Mode.ARM
    switcher.update(buttons(button_lb=1), CFG)
    assert switcher.mode == Mode.ARM  # unchanged: still held
    switcher.update(buttons(button_lb=0), CFG)
    assert switcher.mode == Mode.ARM  # released, unchanged
    switcher.update(buttons(button_lb=1), CFG)
    assert switcher.mode == Mode.MAST  # new press: advances again


def test_deadman_reflects_rb_button():
    assert deadman_engaged(buttons(button_rb=1), CFG) is True
    assert deadman_engaged(buttons(button_rb=0), CFG) is False


# -- drive mode ---------------------------------------------------------
def test_drive_twist_forward_and_turn():
    linear_x, angular_z = compute_drive_twist(axes(axis_left_y=1.0, axis_left_x=0.5), CFG)
    assert linear_x == pytest.approx(CFG.max_linear_mps)
    assert angular_z == pytest.approx(0.5 * CFG.max_angular_radps)


def test_drive_twist_deadzone_yields_zero():
    linear_x, angular_z = compute_drive_twist(axes(axis_left_y=0.05, axis_left_x=0.05), CFG)
    assert linear_x == 0.0
    assert angular_z == 0.0


def test_drive_twist_invert_left_y():
    cfg = TeleopConfig(invert_left_y=True)
    linear_x, _ = compute_drive_twist(axes(axis_left_y=1.0), cfg)
    assert linear_x == pytest.approx(-cfg.max_linear_mps)


# -- arm mode -----------------------------------------------------------
def test_arm_jog_moves_only_deflected_axes():
    targets = [0, 0, 0, 0, 0]
    new_targets = compute_arm_jog(axes(axis_left_x=1.0), CFG, dt_sec=1.0, current_targets_steps=targets)
    assert new_targets[0] == int(round(CFG.arm_jog_steps_per_sec))
    assert new_targets[1:] == [0, 0, 0, 0]


def test_arm_jog_j5_from_trigger_difference():
    targets = [0, 0, 0, 0, 100]
    # right trigger fully pressed (+1), left trigger unpressed (-1) -> (rt-lt)/2 = 1.0
    new_targets = compute_arm_jog(
        axes(axis_right_trigger=1.0, axis_left_trigger=-1.0), CFG, dt_sec=0.5, current_targets_steps=targets
    )
    assert new_targets[4] == 100 + int(round(CFG.arm_jog_steps_per_sec * 0.5))


def test_arm_jog_rejects_wrong_target_length():
    with pytest.raises(ValueError):
        compute_arm_jog(axes(), CFG, dt_sec=1.0, current_targets_steps=[0, 0, 0])


# -- mast mode -----------------------------------------------------------
def test_mast_command_full_right_stick_deflection_maps_to_max_angle():
    yaw, pitch, lift = compute_mast_command(axes(axis_right_x=1.0, axis_right_y=1.0), buttons(), CFG)
    assert yaw == int(round(CFG.max_head_yaw_deg * 10))
    assert pitch == int(round(CFG.max_head_pitch_deg * 10))
    assert lift == 0


def test_mast_command_lift_buttons():
    _, _, lift_erect = compute_mast_command(axes(), buttons(button_a=1), CFG)
    _, _, lift_stow = compute_mast_command(axes(), buttons(button_b=1), CFG)
    _, _, lift_hold = compute_mast_command(axes(), buttons(), CFG)
    assert lift_erect == 1
    assert lift_stow == -1
    assert lift_hold == 0


# -- microscope mode ------------------------------------------------------
def test_microscope_focus_jog_accumulates_across_calls():
    state = MicroscopeJogState()
    compute_microscope_command(axes(axis_left_y=1.0), buttons(), CFG, dt_sec=1.0, state=state)
    first = state.focus_steps
    compute_microscope_command(axes(axis_left_y=1.0), buttons(), CFG, dt_sec=1.0, state=state)
    assert state.focus_steps == first * 2
    assert first == int(round(CFG.microscope_focus_jog_steps_per_sec))


def test_microscope_led_pwm_from_trigger():
    state = MicroscopeJogState()
    _, led_unpressed, _ = compute_microscope_command(axes(axis_right_trigger=-1.0), buttons(), CFG, 1.0, state)
    _, led_full, _ = compute_microscope_command(axes(axis_right_trigger=1.0), buttons(), CFG, 1.0, state)
    assert led_unpressed == 0
    assert led_full == 255


def test_microscope_cover_toggle_is_edge_triggered():
    state = MicroscopeJogState()
    assert state.cover_open is False
    _, _, cover1 = compute_microscope_command(axes(), buttons(button_a=1), CFG, 1.0, state)
    assert cover1 is True
    # holding the button should not toggle again
    _, _, cover2 = compute_microscope_command(axes(), buttons(button_a=1), CFG, 1.0, state)
    assert cover2 is True
    # release then press again toggles back
    compute_microscope_command(axes(), buttons(button_a=0), CFG, 1.0, state)
    _, _, cover3 = compute_microscope_command(axes(), buttons(button_a=1), CFG, 1.0, state)
    assert cover3 is False


# -- drive geometry mode (within DRIVE subsystem mode) ---------------------
def test_next_drive_geometry_mode_cycles_and_wraps():
    m = DriveGeometryMode.ACKERMANN
    seen = [m]
    for _ in range(2):
        m = next_drive_geometry_mode(m)
        seen.append(m)
    assert seen == [
        DriveGeometryMode.ACKERMANN,
        DriveGeometryMode.POINT_TURN,
        DriveGeometryMode.ACKERMANN,
    ]


def test_next_drive_geometry_mode_from_stop_resumes_at_ackermann():
    assert next_drive_geometry_mode(DriveGeometryMode.STOP) == DriveGeometryMode.ACKERMANN


def test_drive_geometry_switcher_x_toggles_on_press_edge_only():
    switcher = DriveGeometrySwitcher()
    assert switcher.mode == DriveGeometryMode.ACKERMANN
    switcher.update(buttons(button_x=1), CFG)
    assert switcher.mode == DriveGeometryMode.POINT_TURN
    switcher.update(buttons(button_x=1), CFG)  # held: no further advance
    assert switcher.mode == DriveGeometryMode.POINT_TURN
    switcher.update(buttons(button_x=0), CFG)
    switcher.update(buttons(button_x=1), CFG)  # new press: toggles back
    assert switcher.mode == DriveGeometryMode.ACKERMANN


def test_drive_geometry_switcher_y_forces_stop_immediately():
    switcher = DriveGeometrySwitcher()
    switcher.update(buttons(button_x=1), CFG)  # -> POINT_TURN
    assert switcher.mode == DriveGeometryMode.POINT_TURN
    switcher.update(buttons(button_y=1), CFG)
    assert switcher.mode == DriveGeometryMode.STOP


def test_drive_geometry_switcher_resume_from_stop_needs_explicit_x_press():
    switcher = DriveGeometrySwitcher()
    switcher.update(buttons(button_y=1), CFG)
    assert switcher.mode == DriveGeometryMode.STOP
    switcher.update(buttons(), CFG)  # releasing Y alone must not resume
    assert switcher.mode == DriveGeometryMode.STOP
    switcher.update(buttons(button_x=1), CFG)
    assert switcher.mode == DriveGeometryMode.ACKERMANN


def test_compute_point_turn_rate_uses_only_left_x():
    rate = compute_point_turn_rate(axes(axis_left_x=0.6, axis_left_y=1.0), CFG)
    assert rate == pytest.approx(0.6 * CFG.max_angular_radps)


def test_compute_point_turn_rate_deadzone_yields_zero():
    assert compute_point_turn_rate(axes(axis_left_x=0.05), CFG) == 0.0


# -- antenna mode ----------------------------------------------------------
def test_antenna_jog_azimuth_from_left_x():
    azimuth, elevation = compute_antenna_jog(
        axes(axis_left_x=1.0), CFG, dt_sec=1.0, current_azimuth_decideg=150, current_elevation_decideg=0
    )
    assert azimuth == 150 + int(round(CFG.antenna_jog_deg_per_sec * 10))
    assert elevation == 0


def test_antenna_jog_elevation_from_left_y():
    azimuth, elevation = compute_antenna_jog(
        axes(axis_left_y=1.0), CFG, dt_sec=0.5, current_azimuth_decideg=150, current_elevation_decideg=900
    )
    assert azimuth == 150
    assert elevation == 900 + int(round(CFG.antenna_jog_deg_per_sec * 10 * 0.5))


def test_antenna_jog_deadzone_yields_no_change():
    azimuth, elevation = compute_antenna_jog(
        axes(axis_left_x=0.05, axis_left_y=0.05),
        CFG,
        dt_sec=1.0,
        current_azimuth_decideg=1500,
        current_elevation_decideg=900,
    )
    assert azimuth == 1500
    assert elevation == 900


def test_antenna_jog_does_not_clamp_to_operational_range():
    # Deliberately verifies compute_antenna_jog does NOT self-clamp -
    # see its own docstring for why: clamping is the firmware's job
    # (antenna_uno5.ino's constrain() calls), and duplicating the
    # 15-285/0-180 range here would risk it silently drifting out of
    # sync with the firmware's own copy.
    azimuth, _elevation = compute_antenna_jog(
        axes(axis_left_x=1.0), CFG, dt_sec=100.0, current_azimuth_decideg=2800, current_elevation_decideg=0
    )
    assert azimuth > 2850  # past max_azimuth_deg*10 - deliberately unclamped here
