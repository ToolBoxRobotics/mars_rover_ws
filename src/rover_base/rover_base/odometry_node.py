"""rover_base odometry node.

Subscribes to rover_msgs/BaseState (which already carries the ML/MR
encoder deltas the base bridge reads off the Mega) and publishes
nav_msgs/Odometry on `odom`, plus broadcasts the `odom` -> `base_link`
TF transform - the two things any SLAM/Nav2 stack needs from
odometry. See odometry.py for the integration math.

Uses each BaseState message's own header.stamp (set by the base
bridge at its own control-loop rate) to compute dt between updates,
rather than local receipt time, so integration accuracy doesn't
degrade under subscriber-side scheduling jitter.

Publishes explicit covariance (not the message-default all-zeros) on
both pose and twist, since this feeds an EKF (rover_navigation's
`ekf_local_params.yaml`) that specifically fuses this rover's
nonholonomic zero lateral velocity (vy) as a genuine constraint, not
just an omitted field - robot_localization's own documentation is
explicit that doing so is correct *only* if vy's covariance isn't
literally zero (a Kalman filter reads exact zero as infinite
confidence, not "very confident") or implausibly huge (which would
just discard it). See _TWIST_VARIANCE_* below.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from rover_msgs.msg import BaseState

from .odometry import OdometryConfig, Pose2D, diagonal_covariance, integrate_odometry, quaternion_from_yaw

# Diagonal covariance placeholders - reasonable starting values, not
# independently characterized against the real hardware yet. Tune
# against actual encoder/IMU noise once on the bench; these exist so
# the EKF has *something* better than an ambiguous all-zero default to
# weigh this sensor against the IMU with.
_POSE_VARIANCE_UNKNOWN = 1e3  # this node doesn't estimate absolute pose covariance growth; say so honestly
_TWIST_VARIANCE_VX = 0.02  # (m/s)^2
_TWIST_VARIANCE_VY = 0.001  # (m/s)^2 - small: encodes the nonholonomic zero-lateral-velocity constraint,
#                             not a real measurement, so it's more confident than vx, not less
_TWIST_VARIANCE_VYAW = 0.03  # (rad/s)^2
_TWIST_VARIANCE_UNUSED = 1e3  # vz, vroll, vpitch - not estimated at all


# BaseState.encoder_ticks/encoder_delta_ticks are 2-wide now: only the
# fixed middle wheels are physically encoded (see rover_msgs/BaseState.msg
# and odometry.py's module docstring for why).
_ML_INDEX = 0
_MR_INDEX = 1


class OdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_odometry_node")

        self.declare_parameter("wheel_radius_m", 0.075)
        self.declare_parameter("track_m", 0.46)
        self.declare_parameter("encoder_ticks_per_rev", 663)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self._cfg = OdometryConfig(
            wheel_radius_m=self.get_parameter("wheel_radius_m").value,
            track_m=self.get_parameter("track_m").value,
            encoder_ticks_per_rev=int(self.get_parameter("encoder_ticks_per_rev").value),
        )
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._pose = Pose2D(0.0, 0.0, 0.0)
        self._last_stamp: Optional[Time] = None

        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(BaseState, "rover_base/state", self._on_base_state, 10)

        self.get_logger().info(
            f"rover_odometry_node ready (odom_frame={self._odom_frame}, "
            f"base_frame={self._base_frame}, publish_tf={self._publish_tf})"
        )

    def _on_base_state(self, msg: BaseState) -> None:
        if len(msg.encoder_delta_ticks) != 2:
            self.get_logger().warn(
                f"expected 2 encoder_delta_ticks (ML, MR), got {len(msg.encoder_delta_ticks)}; dropping"
            )
            return

        stamp = Time.from_msg(msg.header.stamp)
        if self._last_stamp is None:
            self._last_stamp = stamp
            return  # need a previous timestamp to compute dt; skip the first message

        dt_sec = (stamp - self._last_stamp).nanoseconds / 1e9
        self._last_stamp = stamp
        if dt_sec <= 0:
            return  # stale/duplicate/out-of-order message; skip rather than divide oddly

        ml_delta = msg.encoder_delta_ticks[_ML_INDEX]
        mr_delta = msg.encoder_delta_ticks[_MR_INDEX]
        update = integrate_odometry(self._pose, ml_delta, mr_delta, dt_sec, self._cfg)
        self._pose = update.pose

        qx, qy, qz, qw = quaternion_from_yaw(self._pose.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = self._pose.x
        odom_msg.pose.pose.position.y = self._pose.y
        odom_msg.pose.covariance = diagonal_covariance([_POSE_VARIANCE_UNKNOWN] * 6)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = update.linear_x_mps
        odom_msg.twist.twist.angular.z = update.angular_z_radps
        # linear.y is left at its default 0.0 - not a real measurement,
        # but a genuine constraint: this rover cannot move sideways in
        # either of its two drive modes (ACKERMANN, POINT_TURN). Fused
        # deliberately (see module docstring) with a small, non-zero
        # variance rather than omitted or left at an ambiguous zero.
        odom_msg.twist.covariance = diagonal_covariance(
            [
                _TWIST_VARIANCE_VX,
                _TWIST_VARIANCE_VY,
                _TWIST_VARIANCE_UNUSED,
                _TWIST_VARIANCE_UNUSED,
                _TWIST_VARIANCE_UNUSED,
                _TWIST_VARIANCE_VYAW,
            ]
        )
        self._odom_pub.publish(odom_msg)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._pose.x
            t.transform.translation.y = self._pose.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
