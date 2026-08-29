"""Entry point for `ros2 run rover_web_gui web_gui_node`.

Brings up rclpy and the RosBridge first, wires it into the FastAPI
app, then hands control to uvicorn. Uvicorn owns the main thread's
asyncio loop; the ROS executor runs in its own background thread (see
RosBridge.start()).
"""

from __future__ import annotations

import argparse

import rclpy
import uvicorn

from .ros_bridge import RosBridge
from .server import app, set_bridge


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args, _ros_args = parser.parse_known_args()

    rclpy.init()
    bridge = RosBridge()
    bridge.start()
    set_bridge(bridge)

    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    finally:
        bridge.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
