"""Thin ROS wiring around :mod:`rover_teleop.joy_mapping`. Requires the
standard `joy` package's joy_node to already be running and publishing
sensor_msgs/Joy on `joy` (see rover_bringup's teleop launch file,
which starts both together).

Subsystem mode cycles DRIVE -> ARM -> MAST -> MICROSCOPE -> ANTENNA on
each LB press. Within DRIVE mode specifically, the base's steering geometry is
a second, independent selection: X toggles ACKERMANN <-> POINT_TURN,
Y forces STOP immediately (see joy_mapping.py for why Y is one-way).
RB must be held for any command to leave neutral - release it any time
to stop everything immediately, in every mode.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from rover_msgs.msg import AntennaCommand, ArmCommand, DriveMode, MastCommand, MicroscopeCommand

from .joy_mapping import (
    DriveGeometryMode,
    DriveGeometrySwitcher,
    Mode,
    MicroscopeJogState,
    ModeSwitcher,
    TeleopConfig,
    compute_antenna_jog,
    compute_arm_jog,
    compute_drive_twist,
    compute_mast_command,
    compute_microscope_command,
    compute_point_turn_rate,
    deadman_engaged,
)


class XboxTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("xbox_teleop_node")

        self._cfg = self._load_config()
        self._mode_switcher = ModeSwitcher()
        self._drive_geometry_switcher = DriveGeometrySwitcher()
        self._arm_targets = [0, 0, 0, 0, 0]
        self._microscope_state = MicroscopeJogState()
        # (150, 0) decideg = (15.0, 0.0) deg - the antenna's actual
        # post-homing position (each axis's own operational minimum,
        # see antenna_uno5.ino), not an arbitrary (0, 0) the way the
        # arm's own initial targets are - azimuth's real range
        # (15-285 deg) doesn't include 0 the way the arm's does, so
        # starting there would mean the first several jog inputs get
        # silently clamped by the firmware before ever reaching a
        # valid position.
        self._antenna_targets = (150, 0)
        self._last_joy_time = time.monotonic()

        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._drive_mode_pub = self.create_publisher(DriveMode, "rover_base/drive_mode", 10)
        self._arm_pub = self.create_publisher(ArmCommand, "rover_arm/command", 10)
        self._mast_pub = self.create_publisher(MastCommand, "rover_mast/command", 10)
        self._microscope_pub = self.create_publisher(MicroscopeCommand, "rover_microscope/command", 10)
        self._antenna_pub = self.create_publisher(AntennaCommand, "rover_antenna/command", 10)
        self._mode_pub = self.create_publisher(String, "rover_teleop/mode", 10)
        self._drive_geometry_pub = self.create_publisher(String, "rover_teleop/drive_geometry_mode", 10)

        self.create_subscription(Joy, "joy", self._on_joy, 10)
        self.get_logger().info(
            "xbox_teleop_node ready - hold RB to enable, LB to cycle subsystem mode, "
            "X/Y in DRIVE mode to cycle/stop steering geometry"
        )

    def _load_config(self) -> TeleopConfig:
        defaults = TeleopConfig()
        for field_name, default_value in defaults.__dict__.items():
            self.declare_parameter(field_name, default_value)
        values = {
            field_name: self.get_parameter(field_name).value for field_name in defaults.__dict__
        }
        return TeleopConfig(**values)

    def _on_joy(self, msg: Joy) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_joy_time)
        self._last_joy_time = now

        mode = self._mode_switcher.update(msg.buttons, self._cfg)
        self._mode_pub.publish(String(data=mode.name))

        if not deadman_engaged(msg.buttons, self._cfg):
            self._publish_neutral()
            return

        if mode == Mode.DRIVE:
            self._handle_drive(msg.axes, msg.buttons)
        elif mode == Mode.ARM:
            self._handle_arm(msg.axes, dt)
        elif mode == Mode.MAST:
            self._handle_mast(msg.axes, msg.buttons)
        elif mode == Mode.MICROSCOPE:
            self._handle_microscope(msg.axes, msg.buttons, dt)
        elif mode == Mode.ANTENNA:
            self._handle_antenna(msg.axes, dt)

    def _publish_neutral(self) -> None:
        self._cmd_vel_pub.publish(Twist())  # all-zero Twist: stop

    def _handle_drive(self, axes, buttons) -> None:
        geometry_mode = self._drive_geometry_switcher.update(buttons, self._cfg)
        self._drive_geometry_pub.publish(String(data=geometry_mode.name))
        self._drive_mode_pub.publish(DriveMode(mode=int(geometry_mode)))

        twist = Twist()
        if geometry_mode == DriveGeometryMode.STOP:
            pass  # leave twist all-zero; rover_base ignores it in STOP anyway
        elif geometry_mode == DriveGeometryMode.POINT_TURN:
            twist.angular.z = compute_point_turn_rate(axes, self._cfg)
        else:  # ACKERMANN
            twist.linear.x, twist.angular.z = compute_drive_twist(axes, self._cfg)

        self._cmd_vel_pub.publish(twist)

    def _handle_arm(self, axes, dt: float) -> None:
        self._arm_targets = compute_arm_jog(axes, self._cfg, dt, self._arm_targets)
        cmd = ArmCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_target_steps = self._arm_targets
        cmd.enable = True
        self._arm_pub.publish(cmd)

    def _handle_mast(self, axes, buttons) -> None:
        yaw, pitch, lift_mode = compute_mast_command(axes, buttons, self._cfg)
        cmd = MastCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.head_yaw_decideg = yaw
        cmd.head_pitch_decideg = pitch
        cmd.lift_mode = lift_mode
        # Same reasoning as _handle_arm's cmd.enable = True: the
        # controller actively driving this axis should also ensure its
        # drivers are enabled, not leave driver_enable at its unset
        # default (False) - the firmware gates target application and
        # enable/disable on the same command, so a stray False here
        # would silently prevent the very movement being commanded.
        cmd.driver_enable = True
        self._mast_pub.publish(cmd)

    def _handle_microscope(self, axes, buttons, dt: float) -> None:
        focus, led_pwm, cover_open = compute_microscope_command(
            axes, buttons, self._cfg, dt, self._microscope_state
        )
        cmd = MicroscopeCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.focus_target_steps = focus
        cmd.led_pwm = led_pwm
        cmd.cover_open = cover_open
        self._microscope_pub.publish(cmd)

    def _handle_antenna(self, axes, dt: float) -> None:
        self._antenna_targets = compute_antenna_jog(axes, self._cfg, dt, *self._antenna_targets)
        cmd = AntennaCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.azimuth_decideg, cmd.elevation_decideg = self._antenna_targets
        # Same reasoning as _handle_arm/_handle_mast: actively driving
        # this axis should also keep its drivers enabled, not leave
        # driver_enable at its unset default (False), which would
        # silently prevent the very movement being commanded.
        cmd.driver_enable = True
        self._antenna_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
