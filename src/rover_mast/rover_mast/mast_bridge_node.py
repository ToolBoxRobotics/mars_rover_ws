"""rover_mast bridge node.

Subscribes to ``rover_msgs/MastCommand`` (head yaw/pitch target, a
lift mode: stow/hold/erect, and a driver enable/disable flag) and
streams it to the mast Uno (#3). Republishes head orientation, lift
limit-switch state, yaw/pitch calibration state, and head-driver
enabled state as ``rover_msgs/MastState``.

Sends a one-shot homing request ('Z') on startup if
``home_on_startup`` is true, mirroring rover_arm_bridge - the yaw/pitch
NEMA17+TB6600 axes have no absolute encoders and need their calibration
switches to establish a position reference after every power-up. Each
switch sits at that axis's minimum bound, not its center, so the
firmware treats triggering it as reaching that minimum rather than
zero, then drives from there to true zero - only once both axes
actually arrive does ``homed`` become true and the head drivers
disable themselves. This node has no active role in that sequence
beyond passing through whatever the firmware reports via
``driver_enabled``, and its own ``driver_enable`` commands are ignored
by the firmware until ``homed`` is true (see mast_protocol.py and
mast_uno3.ino's handleMastCommand() for why). The lift axis is
unaffected either way: it has no step-relative position to home, just
directly-read limit switches, so lift commands always take effect
immediately regardless of head-axis homing state.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from rover_msgs.msg import BoardStatus, MastCommand, MastState
from rover_protocol import SerialLink

from . import mast_protocol


class MastBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_mast_bridge")

        self.declare_parameter("serial_port", "/dev/rover/mast")
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

        self._last_command = MastCommand()
        self._last_command.lift_mode = mast_protocol.LIFT_HOLD
        # False matches the firmware's own eventual resting state (the
        # post-calibration sequence disables the drivers once it
        # finishes) - not that this default actually gets applied
        # before then anyway, since mast_uno3.ino's handleMastCommand()
        # ignores driver_enable entirely while homing or the post-cal
        # sequence is active, for exactly the reason this default
        # exists: nothing here has any way to know when a genuine
        # operator command shows up versus this node's own routine
        # resend of a value nobody's actually touched yet.
        self._last_command.driver_enable = False
        self._last_command_time = 0.0
        self._sent_home_request = False

        self._state_pub = self.create_publisher(MastState, "rover_mast/state", 10)
        self._status_pub = self.create_publisher(BoardStatus, "rover_mast/board_status", 10)
        self.create_subscription(MastCommand, "rover_mast/command", self._on_command, 10)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"rover_mast bridge starting on {self._link.port} @ {self._link.baud} baud"
        )

    def _on_command(self, msg: MastCommand) -> None:
        self._last_command = msg
        self._last_command_time = time.monotonic()

    def _on_timer(self) -> None:
        self._drain_incoming()

        if self.get_parameter("home_on_startup").value and not self._sent_home_request:
            if self._link.write_frame(mast_protocol.encode_home_request()):
                self._sent_home_request = True
                self.get_logger().info("sent homing request to mast Uno")
            return  # wait for the next tick before sending a normal mast command

        stale = (time.monotonic() - self._last_command_time) > self._command_timeout_sec
        if stale:
            # Fail-safe: stop the lift motor and hold the last head
            # pose rather than continuing to drive toward a stale target.
            yaw, pitch = self._last_command.head_yaw_decideg, self._last_command.head_pitch_decideg
            lift_mode = mast_protocol.LIFT_HOLD
            driver_enable = self._last_command.driver_enable
        else:
            yaw = self._last_command.head_yaw_decideg
            pitch = self._last_command.head_pitch_decideg
            lift_mode = self._last_command.lift_mode
            driver_enable = self._last_command.driver_enable

        frame = mast_protocol.encode_mast_command(yaw, pitch, lift_mode, driver_enable)
        self._link.write_frame(frame)
        self._publish_status()

    def _drain_incoming(self, max_reads: int = 8) -> None:
        for _ in range(max_reads):
            result = self._link.read_decoded()
            if result is None:
                return
            msg_type, fields = result
            if msg_type == "S":
                self._publish_mast_state(fields)

    def _publish_mast_state(self, fields) -> None:
        try:
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
            ) = mast_protocol.parse_mast_state(fields)
        except Exception as exc:
            self.get_logger().warn(f"dropping malformed mast state frame: {exc}")
            return

        state = MastState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.head_yaw_decideg = yaw
        state.head_pitch_decideg = pitch
        state.lift_state = lift_state
        state.yaw_limit_triggered = yaw_limit
        state.pitch_limit_triggered = pitch_limit
        state.homed = homed
        state.supply_voltage_mv = voltage_mv
        state.driver_enabled = driver_enabled
        state.board_temperature_decic = temperature_deci_c
        state.fan_duty_percent = fan_duty_percent
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.board_name = "mast_uno3"
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
    node = MastBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
