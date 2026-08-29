"""rover_microscope bridge node.

Subscribes to ``rover_msgs/MicroscopeCommand`` and streams it to the
microscope Uno (#4). Republishes focus/LED/cover state as
``rover_msgs/MicroscopeState``. The USB microscope camera itself is
handled separately by :mod:`rover_microscope.camera_publisher_node` -
this node only talks to the Uno's actuators (stepper focus/zoom, LED,
lens-cover servo).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from rover_msgs.msg import BoardStatus, MicroscopeCommand, MicroscopeState
from rover_protocol import SerialLink

from . import microscope_protocol


class MicroscopeBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_microscope_bridge")

        self.declare_parameter("serial_port", "/dev/rover/microscope")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("boot_grace_sec", 2.0)

        self._link = SerialLink(
            port=self.get_parameter("serial_port").value,
            baud=int(self.get_parameter("serial_baud").value),
            timeout=0.05,
            boot_grace_sec=float(self.get_parameter("boot_grace_sec").value),
        )

        self._last_command = MicroscopeCommand()
        self._last_command.led_pwm = 0
        self._last_command.cover_open = False
        self._last_command.driver_enable = False

        self._state_pub = self.create_publisher(MicroscopeState, "rover_microscope/state", 10)
        self._status_pub = self.create_publisher(BoardStatus, "rover_microscope/board_status", 10)
        self.create_subscription(MicroscopeCommand, "rover_microscope/command", self._on_command, 10)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"rover_microscope bridge starting on {self._link.port} @ {self._link.baud} baud"
        )

    def _on_command(self, msg: MicroscopeCommand) -> None:
        self._last_command = msg

    def _on_timer(self) -> None:
        self._drain_incoming()

        # UPDATED, at the user's own explicit request: this used to
        # force led_pwm=0/cover_open=False whenever no ROS command had
        # arrived recently (a "protect the optics if the link drops"
        # fail-safe) - removed entirely, alongside the equivalent
        # watchdog-triggered behavior in microscope_uno4.ino's own
        # firmware (see that file's own header comment for the fuller
        # reasoning and the trade-off this removal carries). The LED
        # and cover now always reflect whatever was last explicitly
        # commanded, with nothing at this layer overriding that if the
        # command stream goes stale - command_timeout_sec and the
        # staleness tracking it depended on are gone along with it,
        # not left declared-but-unused.
        focus = self._last_command.focus_target_steps
        led_pwm = self._last_command.led_pwm
        cover_open = self._last_command.cover_open
        driver_enable = self._last_command.driver_enable

        frame = microscope_protocol.encode_microscope_command(focus, led_pwm, cover_open, driver_enable)
        self._link.write_frame(frame)
        self._publish_status()

    def _drain_incoming(self, max_reads: int = 8) -> None:
        for _ in range(max_reads):
            result = self._link.read_decoded()
            if result is None:
                return
            msg_type, fields = result
            if msg_type == "S":
                self._publish_microscope_state(fields)

    def _publish_microscope_state(self, fields) -> None:
        try:
            focus, led_pwm, cover_open, homed, driver_enabled, temperature_deci_c, fan_duty_percent = (
                microscope_protocol.parse_microscope_state(fields)
            )
        except Exception as exc:
            self.get_logger().warn(f"dropping malformed microscope state frame: {exc}")
            return

        state = MicroscopeState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.focus_position_steps = focus
        state.led_pwm = led_pwm
        state.cover_open = cover_open
        state.homed = homed
        state.driver_enabled = driver_enabled
        state.board_temperature_decic = temperature_deci_c
        state.fan_duty_percent = fan_duty_percent
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.board_name = "microscope_uno4"
        status.port = self._link.port
        status.connected = self._link.connected
        status.rx_frame_count = self._link.rx_frame_count
        status.checksum_error_count = self._link.checksum_error_count
        status.reconnect_count = self._link.reconnect_count
        age = self._link.last_rx_age_sec()
        status.last_rx_age_sec = float(age) if age != float("inf") else -1.0
        self._status_pub.publish(status)

    def destroy_node(self) -> bool:
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicroscopeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
