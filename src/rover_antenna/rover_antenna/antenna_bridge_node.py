"""rover_antenna bridge node.

Subscribes to ``rover_msgs/AntennaCommand`` (azimuth/elevation target
plus a driver enable/disable flag) and streams it to the antenna
gimbal Uno (#5). Republishes gimbal position, calibration-switch
state, and driver-enabled state as ``rover_msgs/AntennaState``.

Sends a one-shot homing request ('Z') on startup if
``home_on_startup`` is true, mirroring rover_mast_bridge - the
azimuth/elevation NEMA17+EBA-17-M+TB6600 axes have no absolute
encoders and need their calibration switches to establish a position
reference after every power-up. Unlike the mast, each switch here sits
at that axis's own operational minimum rather than an offset from a
separately-centered zero, so homing establishes ``homed`` as soon as
both switches are found - no follow-on move-to-zero sequence, and
correspondingly no risk of the bridge's own routine command resends
interrupting one (though the same ``if (homed)`` gating the mast
needed is still applied here too, for the same underlying reason:
this node sends ``AntennaCommand`` frames continuously at its own
control rate once past the one-shot homing request, using whatever's
in its last-known command, which defaults to driver_enable=false
before the operator has ever touched the antenna panel - applying
that unconditionally during homing would disable the drivers
mid-seek).
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from rover_msgs.msg import AntennaCommand, AntennaState, BoardStatus
from rover_protocol import SerialLink

from . import antenna_protocol


class AntennaBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_antenna_bridge")

        self.declare_parameter("serial_port", "/dev/rover/antenna")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("command_timeout_sec", 1.0)
        self.declare_parameter("boot_grace_sec", 2.0)
        self.declare_parameter("home_on_startup", True)

        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)

        self._link = SerialLink(
            port=self.get_parameter("serial_port").value,
            baud=int(self.get_parameter("serial_baud").value),
            timeout=0.05,
            boot_grace_sec=float(self.get_parameter("boot_grace_sec").value),
        )

        self._last_command = AntennaCommand()
        # False matches the firmware's own gating (see this module's
        # docstring) - not that this default actually reaches the
        # firmware before homing anyway, since antenna_uno5.ino
        # ignores driver_enable entirely until homed is true.
        self._last_command.driver_enable = False
        self._last_command_time = 0.0
        self._sent_home_request = False

        self._state_pub = self.create_publisher(AntennaState, "rover_antenna/state", 10)
        self._status_pub = self.create_publisher(BoardStatus, "rover_antenna/board_status", 10)
        self.create_subscription(AntennaCommand, "rover_antenna/command", self._on_command, 10)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"rover_antenna bridge starting on {self._link.port} @ {self._link.baud} baud"
        )

    def _on_command(self, msg: AntennaCommand) -> None:
        self._last_command = msg
        self._last_command_time = time.monotonic()

    def _on_timer(self) -> None:
        self._drain_incoming()

        if self.get_parameter("home_on_startup").value and not self._sent_home_request:
            if self._link.write_frame(antenna_protocol.encode_home_request()):
                self._sent_home_request = True
                self.get_logger().info("sent homing request to antenna gimbal Uno")
            return  # wait for the next tick before sending a normal gimbal command

        stale = (time.monotonic() - self._last_command_time) > self._command_timeout_sec
        if stale:
            # Fail-safe: hold the last commanded position rather than
            # continuing toward a stale target, same pattern as the
            # arm/mast bridges.
            azimuth = self._last_command.azimuth_decideg
            elevation = self._last_command.elevation_decideg
            driver_enable = self._last_command.driver_enable
        else:
            azimuth = self._last_command.azimuth_decideg
            elevation = self._last_command.elevation_decideg
            driver_enable = self._last_command.driver_enable

        frame = antenna_protocol.encode_gimbal_command(azimuth, elevation, driver_enable)
        self._link.write_frame(frame)
        self._publish_status()

    def _drain_incoming(self, max_reads: int = 8) -> None:
        for _ in range(max_reads):
            result = self._link.read_decoded()
            if result is None:
                return
            msg_type, fields = result
            if msg_type == "S":
                self._publish_gimbal_state(fields)

    def _publish_gimbal_state(self, fields) -> None:
        try:
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
            ) = antenna_protocol.parse_gimbal_state(fields)
        except Exception as exc:
            self.get_logger().warn(f"dropping malformed antenna state frame: {exc}")
            return

        state = AntennaState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.azimuth_decideg = azimuth
        state.elevation_decideg = elevation
        state.azimuth_limit_triggered = azimuth_limit
        state.elevation_limit_triggered = elevation_limit
        state.homed = homed
        state.supply_voltage_mv = voltage_mv
        state.driver_enabled = driver_enabled
        state.board_temperature_decic = temperature_deci_c
        state.fan_duty_percent = fan_duty_percent
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.board_name = "antenna_uno5"
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
    node = AntennaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
