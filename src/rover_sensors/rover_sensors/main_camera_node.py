"""Publishes the rover's main forward-facing USB perception camera as
a ROS 2 CompressedImage stream for OpenCV-based vision and for the
web GUI's live view. Unlike the microscope camera (see
rover_microscope.camera_publisher_node), this one has no
snapshot/recording services - just a live feed - since the spec calls
those out specifically for the microscope, not general navigation.
"""

from __future__ import annotations

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class MainCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("main_camera_node")

        self.declare_parameter("camera_device", "/dev/rover/main_cam")
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("frame_rate_hz", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("frame_id", "main_camera_optical_frame")

        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._frame_rate_hz = float(self.get_parameter("frame_rate_hz").value)
        self._frame_id = self.get_parameter("frame_id").value

        self._capture = None
        self._image_pub = self.create_publisher(CompressedImage, "rover_sensors/main_camera/image/compressed", 5)

        self._open_capture()
        self.create_timer(1.0 / self._frame_rate_hz, self._on_timer)

    def _open_capture(self) -> None:
        device = self.get_parameter("camera_device").value
        # Explicit cv2.CAP_V4L2 matters here, not just style - without a
        # backend hint, OpenCV tries backends in a fixed priority order
        # (FFMPEG, then GStreamer, then V4L2, then CV_IMAGES, ...) and a
        # udev symlink like this one can fail against FFMPEG/GStreamer's
        # own device-detection heuristics even though it's a perfectly
        # normal V4L2 device - the failure then cascades all the way to
        # CV_IMAGES, which is for reading numbered image-file sequences,
        # not video devices, and fails confusingly ("can't find starting
        # number"). V4L2 is the correct, reliable backend for a UVC
        # webcam on Linux - asking for it directly skips the cascade
        # rather than hoping auto-detection lands there anyway.
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warn(f"could not open main camera at {device}; will keep retrying")
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
            self.get_logger().warn("main camera read failed; reopening")
            self._capture.release()
            self._capture = None
            return

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self._image_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._capture is not None:
            self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MainCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
