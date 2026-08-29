"""rover_base bridge node.

Subscribes to geometry_msgs/Twist on ``cmd_vel`` and to
rover_msgs/DriveMode on ``rover_base/drive_mode`` (default ACKERMANN
if nothing has been received yet), reinterprets the Twist per the
active mode via :mod:`rover_base.kinematics`, and streams the result
to the base Mega (#1) over a simple checksummed ASCII serial protocol
(see :mod:`rover_base.base_protocol`). Reads back quadrature-encoder
ticks from the Mega and republishes them as ``rover_msgs/BaseState``
for an odometry node to consume.

If no ``cmd_vel`` has been received within ``command_timeout_sec`` the
node treats the twist as all-zero (which, in every mode, already
produces a full stop - see kinematics.py), so a dropped teleop/nav
connection cannot leave the rover driving blind. DriveMode itself has
no staleness timeout: unlike velocity, a "stuck" mode selection is not
unsafe on its own, and STOP mode is a stronger, unconditional override
anyway.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_msgs.msg import BaseCommand, BaseState, BoardStatus, DriveMode
from rover_protocol import SerialLink

from . import base_protocol
from .kinematics import (
    BaseGeometry,
    point_turn_wheel_commands,
    stop_wheel_commands,
    twist_to_wheel_commands,
)


class BaseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_base_bridge")

        self.declare_parameter("serial_port", "/dev/rover/base")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter(
            "boot_grace_sec", 2.0
        )  # Arduino auto-resets on connect; see rover_protocol.serial_link
        self.declare_parameter("wheelbase_front_m", 0.30)
        self.declare_parameter("wheelbase_rear_m", 0.30)
        self.declare_parameter("track_m", 0.46)
        self.declare_parameter("wheel_radius_m", 0.075)
        self.declare_parameter("max_wheel_rpm", 83.0)
        self.declare_parameter("max_steer_deg", 60.0)

        self._geometry = BaseGeometry(
            wheelbase_front_m=self.get_parameter("wheelbase_front_m").value,
            wheelbase_rear_m=self.get_parameter("wheelbase_rear_m").value,
            track_m=self.get_parameter("track_m").value,
            wheel_radius_m=self.get_parameter("wheel_radius_m").value,
            max_wheel_rpm=self.get_parameter("max_wheel_rpm").value,
            max_steer_deg=self.get_parameter("max_steer_deg").value,
        )
        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)

        self._link = SerialLink(
            port=self.get_parameter("serial_port").value,
            baud=int(self.get_parameter("serial_baud").value),
            timeout=0.05,
            boot_grace_sec=float(self.get_parameter("boot_grace_sec").value),
        )

        self._last_twist = Twist()
        self._last_twist_time = 0.0
        self._drive_mode = DriveMode.ACKERMANN
        self._prev_encoder_ticks = None

        self._cmd_pub = self.create_publisher(BaseCommand, "rover_base/command_echo", 10)
        self._state_pub = self.create_publisher(BaseState, "rover_base/state", 10)
        self._status_pub = self.create_publisher(BoardStatus, "rover_base/board_status", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(DriveMode, "rover_base/drive_mode", self._on_drive_mode, 10)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"rover_base bridge starting on {self._link.port} @ {self._link.baud} baud"
        )

    # -- subscriptions ------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_twist = msg
        self._last_twist_time = time.monotonic()

    def _on_drive_mode(self, msg: DriveMode) -> None:
        self._drive_mode = msg.mode

    # -- main loop ------------------------------------------------------
    def _on_timer(self) -> None:
        self._drain_incoming()

        stale = (time.monotonic() - self._last_twist_time) > self._command_timeout_sec
        linear_x = 0.0 if stale else self._last_twist.linear.x
        angular_z = 0.0 if stale else self._last_twist.angular.z

        if self._drive_mode == DriveMode.STOP:
            throttle, steer = stop_wheel_commands()
        elif self._drive_mode == DriveMode.POINT_TURN:
            throttle, steer = point_turn_wheel_commands(angular_z, self._geometry)
        else:  # DriveMode.ACKERMANN, and the default before any mode is received
            throttle, steer = twist_to_wheel_commands(linear_x, angular_z, self._geometry)

        frame = base_protocol.encode_drive(list(throttle), list(steer))
        self._link.write_frame(frame)

        cmd_msg = BaseCommand()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.drive_mode = self._drive_mode
        cmd_msg.wheel_throttle = list(throttle)
        cmd_msg.steer_decideg = list(steer)
        self._cmd_pub.publish(cmd_msg)

        self._publish_status()

    def _drain_incoming(self, max_reads: int = 8) -> None:
        for _ in range(max_reads):
            result = self._link.read_decoded()
            if result is None:
                return
            msg_type, fields = result
            if msg_type == "E":
                self._publish_encoder_state(fields)

    def _publish_encoder_state(self, fields) -> None:
        try:
            ticks, drive_voltage_mv, steering_voltage_mv, temperature_deci_c, fan_duty_percent = (
                base_protocol.parse_encoder_state(fields)
            )
        except Exception as exc:  # malformed 'E' frame despite valid checksum/type
            self.get_logger().warn(f"dropping malformed encoder frame: {exc}")
            return

        state = BaseState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.encoder_ticks = ticks
        state.drive_voltage_mv = drive_voltage_mv
        state.steering_voltage_mv = steering_voltage_mv
        state.board_temperature_decic = temperature_deci_c
        state.fan_duty_percent = fan_duty_percent
        if self._prev_encoder_ticks is None:
            state.encoder_delta_ticks = [0] * len(ticks)
        else:
            state.encoder_delta_ticks = [
                cur - prev for cur, prev in zip(ticks, self._prev_encoder_ticks)
            ]
        self._prev_encoder_ticks = ticks
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.board_name = "base_mega1"
        status.port = self._link.port
        status.connected = self._link.connected
        status.rx_frame_count = self._link.rx_frame_count
        status.checksum_error_count = self._link.checksum_error_count
        status.reconnect_count = self._link.reconnect_count
        age = self._link.last_rx_age_sec()
        status.last_rx_age_sec = float(age) if age != float("inf") else -1.0
        self._status_pub.publish(status)

    def destroy_node(self) -> bool:
        # Fail-safe: always send an explicit stop before the link closes.
        try:
            stop_frame = base_protocol.encode_drive([0] * 6, [0] * 4)
            self._link.write_frame(stop_frame)
        except Exception:
            pass
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
