#!/usr/bin/env python3
"""Send the rover to a GPS coordinate.

Converts a lat/lon into the current map-frame position via
navsat_transform_node's `/fromLL` service (part of `robot_localization`,
launched by localization.launch.py), then sends that straight to
Nav2's `navigate_to_pose` action as an ordinary map-frame goal - Nav2
and AMCL never need to know GPS was involved at all. This is
deliberate: see navsat_transform_params.yaml's header for why a
GPS-fusing second EKF isn't used here instead (it would fight
slam_toolbox/AMCL for ownership of the map->odom TF).

Requires `navsat_transform_node` running (see localization.launch.py)
and Nav2 running against a saved, geo-referenced map (see
navigation.launch.py) - this only makes sense once you're navigating
against a map that was built (or at least started) somewhere with a
GPS fix, since the GPS -> map-frame conversion is relative to wherever
the datum got set.

For following a whole list of GPS waypoints in one mission rather than
a single goal: newer Nav2 releases (Iron and later - per Nav2's own
migration notes, this was added after Humble, which is what this
workspace targets) have a built-in `FollowGPSWaypoints` action in
`nav2_waypoint_follower` for exactly that, built on this same `/fromLL`
service. Worth switching to if this workspace ever moves off Humble;
not assumed available here.

Usage:
    ros2 run rover_navigation gps_goal.py 42.4373 -86.9436
"""

from __future__ import annotations

import argparse
import sys

from gps_coordinate_validation import validate_coordinates


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("latitude", type=float)
    parser.add_argument("longitude", type=float)
    parser.add_argument("--altitude", type=float, default=0.0)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for services/servers")
    args = parser.parse_args()

    error = validate_coordinates(args.latitude, args.longitude)
    if error:
        print(f"Invalid coordinate: {error}", file=sys.stderr)
        sys.exit(1)

    # Imported here, not at module level, so `validate_coordinates` and
    # its tests never need rclpy/nav2_msgs/robot_localization importable.
    import rclpy
    from geographic_msgs.msg import GeoPoint
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from robot_localization.srv import FromLL

    rclpy.init()
    node = Node("gps_goal")
    logger = node.get_logger()

    from_ll_client = node.create_client(FromLL, "/fromLL")
    if not from_ll_client.wait_for_service(timeout_sec=args.timeout):
        logger.error("/fromLL service not available - is navsat_transform_node running? (localization.launch.py)")
        rclpy.shutdown()
        sys.exit(1)

    request = FromLL.Request()
    request.ll_point = GeoPoint(latitude=args.latitude, longitude=args.longitude, altitude=args.altitude)
    future = from_ll_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    if future.result() is None:
        logger.error("/fromLL service call timed out or failed")
        rclpy.shutdown()
        sys.exit(1)

    map_point = future.result().map_point
    logger.info(
        f"GPS ({args.latitude}, {args.longitude}) -> {args.frame_id} frame "
        f"({map_point.x:.2f}, {map_point.y:.2f})"
    )

    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not nav_client.wait_for_server(timeout_sec=args.timeout):
        logger.error("navigate_to_pose action server not available - is Nav2 running? (navigation.launch.py)")
        rclpy.shutdown()
        sys.exit(1)

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = args.frame_id
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = map_point.x
    goal.pose.pose.position.y = map_point.y
    goal.pose.pose.orientation.w = 1.0  # no particular arrival heading requested

    logger.info("Sending navigate_to_pose goal...")
    send_goal_future = nav_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=args.timeout)
    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        logger.error("Goal rejected by Nav2")
        rclpy.shutdown()
        sys.exit(1)

    logger.info("Goal accepted - navigating (this call blocks until arrival or failure)...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    logger.info("Navigation finished.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
