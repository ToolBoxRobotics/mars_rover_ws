"""Publishes the USB microscope camera (mounted at the arm's 3rd
wrist joint, alongside the focus/LED/cover actuators handled by
:mod:`rover_microscope.microscope_bridge_node`) as a ROS 2
CompressedImage stream, and exposes snapshot / video-recording as
Trigger services so the web GUI's microscope tab can drive them
directly instead of talking to OpenCV itself.
"""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger


class MicroscopeCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("microscope_camera")

        self.declare_parameter("camera_device", "/dev/rover/microscope_cam")
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("frame_rate_hz", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("snapshot_dir", "~/rover_captures/microscope/snapshots")
        self.declare_parameter("recording_dir", "~/rover_captures/microscope/recordings")

        self._snapshot_dir = Path(os.path.expanduser(self.get_parameter("snapshot_dir").value))
        self._recording_dir = Path(os.path.expanduser(self.get_parameter("recording_dir").value))
        self._snapshot_dir.mkdir(parents=True, exist_ok=True)
        self._recording_dir.mkdir(parents=True, exist_ok=True)

        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._frame_rate_hz = float(self.get_parameter("frame_rate_hz").value)

        self._capture = None
        self._latest_frame = None
        self._video_writer = None
        self._recording_path = None

        self._image_pub = self.create_publisher(CompressedImage, "rover_microscope/image/compressed", 5)
        self.create_service(Trigger, "rover_microscope/take_snapshot", self._on_take_snapshot)
        self.create_service(Trigger, "rover_microscope/toggle_recording", self._on_toggle_recording)

        self._open_capture()
        self.create_timer(1.0 / self._frame_rate_hz, self._on_timer)

    def _open_capture(self) -> None:
        device = self.get_parameter("camera_device").value
        # Explicit cv2.CAP_V4L2 - see main_camera_node.py's own copy of
        # this same fix/comment for the full reasoning (OpenCV's default
        # backend auto-detection tries FFMPEG then GStreamer before ever
        # reaching V4L2, and a udev symlink can fail against those
        # first two even though it's a perfectly normal V4L2 device,
        # cascading all the way to a confusing CV_IMAGES error).
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warn(
                f"could not open microscope camera at {device}; will keep retrying"
            )
            self._capture = None
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("frame_width").value))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("frame_height").value))
        self._capture = cap

    def _on_timer(self) -> None:
        if self._capture is None:
            self._open_capture()
            return

        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn("microscope camera read failed; reopening")
            self._capture.release()
            self._capture = None
            return

        self._latest_frame = frame

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self._image_pub.publish(msg)

        if self._video_writer is not None:
            self._video_writer.write(frame)

    def _on_take_snapshot(self, _request, response):
        if self._latest_frame is None:
            response.success = False
            response.message = "no frame available yet"
            return response

        filename = f"microscope_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        path = self._snapshot_dir / filename
        cv2.imwrite(str(path), self._latest_frame)
        response.success = True
        response.message = str(path)
        return response

    def _on_toggle_recording(self, _request, response):
        if self._video_writer is None:
            if self._latest_frame is None:
                response.success = False
                response.message = "no frame available yet; cannot start recording"
                return response
            height, width = self._latest_frame.shape[:2]
            filename = f"microscope_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
            self._recording_path = self._recording_dir / filename
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._video_writer = cv2.VideoWriter(
                str(self._recording_path), fourcc, self._frame_rate_hz, (width, height)
            )
            response.success = True
            response.message = f"recording started: {self._recording_path}"
        else:
            self._video_writer.release()
            self._video_writer = None
            response.success = True
            response.message = f"recording stopped: {self._recording_path}"
        return response

    def destroy_node(self) -> bool:
        if self._video_writer is not None:
            self._video_writer.release()
        if self._capture is not None:
            self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicroscopeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
