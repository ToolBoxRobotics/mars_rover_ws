"""FollowJointTrajectory action server bridging MoveIt2's planned
trajectories into this rover's existing joint-space ArmCommand
protocol.

Why this exists instead of a full ros2_control hardware interface:
rover_arm's firmware (arm_mega2.ino) already accepts absolute
per-joint step targets and handles its own velocity/acceleration
profiling on-device (AccelStepper) - there's no need to reimplement
that on the ROS side just to satisfy ros2_control's hardware_interface
plugin API, which would mean writing and compiling a C++ plugin for
very little actual benefit here. moveit_simple_controller_manager's
FollowJointTrajectory support is explicitly designed for exactly this
case - MoveIt's own docs: "the included MoveItSimpleControllerManager
is sufficient if your robot controllers already provide ROS actions
for FollowJointTrajectory." This node IS that action: it translates
each planned waypoint into an ArmCommand and uses ArmState feedback to
know when the arm has arrived closely enough to advance to the next one.

Action name: arm_controller/follow_joint_trajectory - this must match
rover_arm_moveit_config/config/moveit_controllers.yaml's controller
name ("arm_controller") + action_ns ("follow_joint_trajectory")
exactly, concatenated, or MoveIt's trajectory execution manager won't
find this server and will report "Unable to identify any set of
controllers that can actuate the specified joints."

Execution strategy: rather than finely replicating MoveIt's planned
velocity profile (which would mean either fighting AccelStepper's own
on-device profiling or reimplementing it here), each waypoint's target
position is sent as-is and this node waits until the arm is within
tolerance of it - or the waypoint's own time_from_start has elapsed,
whichever comes first - before advancing to the next one. This is
simpler and more robust than exact profile-matching, at the cost of
not guaranteeing MoveIt's planned intermediate velocities are hit
precisely - acceptable for a slow-moving sample-inspection arm, not
necessarily for a fast pick-and-place cycle.
"""

from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rover_msgs.msg import ArmCommand, ArmState

from .joint_conversion import joint_radians_to_steps, joint_steps_to_radians, reorder_by_name

_DEFAULT_JOINT_NAMES = ["shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll"]
# 200 full steps * 1/16 microstepping * 120:1 EBA-17-M planetary
# gearbox = 384000 steps/rev - see rover_arm/config/arm_topology.yaml
# for the authoritative value; this is only the fallback used if that
# yaml isn't loaded (e.g. running this node standalone).
_DEFAULT_STEPS_PER_JOINT_REV = [384000, 384000, 384000, 384000, 384000]


class ArmTrajectoryActionServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_trajectory_action_server")

        self.declare_parameter("joint_names", _DEFAULT_JOINT_NAMES)
        self.declare_parameter("steps_per_joint_rev", _DEFAULT_STEPS_PER_JOINT_REV)
        self.declare_parameter("goal_tolerance_rad", 0.02)
        self.declare_parameter("waypoint_poll_hz", 20.0)
        self.declare_parameter("waypoint_timeout_margin_sec", 2.0)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._steps_per_joint_rev = list(self.get_parameter("steps_per_joint_rev").value)
        self._goal_tolerance_rad = float(self.get_parameter("goal_tolerance_rad").value)
        self._poll_period_sec = 1.0 / float(self.get_parameter("waypoint_poll_hz").value)
        self._timeout_margin_sec = float(self.get_parameter("waypoint_timeout_margin_sec").value)

        self._latest_state: Optional[ArmState] = None
        self._cmd_pub = self.create_publisher(ArmCommand, "rover_arm/command", 10)
        self.create_subscription(ArmState, "rover_arm/state", self._on_state, 10)

        # ReentrantCallbackGroup + MultiThreadedExecutor (see main()):
        # _execute() blocks its own thread while a trajectory runs, but
        # goal/cancel callbacks and the ArmState subscription still
        # need to run concurrently with that - a single-threaded
        # executor would deadlock the moment a trajectory started.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"arm_trajectory_action_server ready, joints={self._joint_names}"
        )

    def _on_state(self, msg: ArmState) -> None:
        self._latest_state = msg

    def _on_goal(self, goal_request) -> GoalResponse:
        names = list(goal_request.trajectory.joint_names)
        if set(names) != set(self._joint_names):
            self.get_logger().warn(
                f"rejecting goal: joint_names {names} don't match {self._joint_names}"
            )
            return GoalResponse.REJECT
        if self._latest_state is None:
            self.get_logger().warn(
                "rejecting goal: no ArmState received yet (is rover_arm_bridge running?)"
            )
            return GoalResponse.REJECT
        if not self._latest_state.homed:
            self.get_logger().warn("rejecting goal: arm has not completed homing yet")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _send_command(self, positions_rad: List[float]) -> None:
        steps = joint_radians_to_steps(positions_rad, self._steps_per_joint_rev)
        msg = ArmCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_target_steps = steps
        msg.enable = True
        self._cmd_pub.publish(msg)

    def _current_positions_rad(self) -> List[float]:
        return joint_steps_to_radians(
            list(self._latest_state.joint_position_steps), self._steps_per_joint_rev
        )

    def _within_tolerance(self, target_rad: List[float]) -> bool:
        current = self._current_positions_rad()
        return all(abs(c - t) <= self._goal_tolerance_rad for c, t in zip(current, target_rad))

    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        result = FollowJointTrajectory.Result()

        start_time = time.monotonic()

        for point in traj.points:
            target_rad = reorder_by_name(names, list(point.positions), self._joint_names)
            self._send_command(target_rad)

            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            # Some slack past the planned time before giving up on this
            # waypoint and moving on anyway - AccelStepper's own accel
            # profile won't exactly match whatever MoveIt planned for,
            # so arriving a bit later than planned is expected, not a fault.
            deadline = start_time + time_from_start + self._timeout_margin_sec

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    # No CANCELED entry exists in Result's error_code
                    # enum (only failure modes do) - SUCCESSFUL here
                    # just means "no fault occurred," matching how
                    # MoveIt's own controller handle primarily reads
                    # the action's goal status, not this field, to
                    # tell success from cancellation.
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result

                if self._within_tolerance(target_rad) or time.monotonic() >= deadline:
                    break

                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = self._joint_names
                feedback.actual.positions = self._current_positions_rad()
                feedback.desired.positions = target_rad
                goal_handle.publish_feedback(feedback)
                time.sleep(self._poll_period_sec)

        if traj.points:
            final_target = reorder_by_name(names, list(traj.points[-1].positions), self._joint_names)
            if not self._within_tolerance(final_target):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = "final position outside tolerance"
                return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmTrajectoryActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
