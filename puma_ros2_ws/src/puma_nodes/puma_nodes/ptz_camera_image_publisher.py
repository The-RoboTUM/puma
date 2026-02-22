#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time


class RTSPImagePublisher(Node):
    def __init__(self):
        super().__init__("rtsp_image_publisher")

        # Parameters
        self.declare_parameter("rtsp_url", "rtsp://admin:puma2126@192.168.1.108:554/stream1")
        self.declare_parameter("topic", "/ptz/image_raw")
        self.declare_parameter("frame_id", "ptz_camera")
        self.declare_parameter("publish_fps", 15.0)
        self.declare_parameter("force_tcp", True)

        self.rtsp_url = self.get_parameter("rtsp_url").value
        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_fps = float(self.get_parameter("publish_fps").value)
        self.force_tcp = bool(self.get_parameter("force_tcp").value)

        self.pub = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()

        # OpenCV / FFMPEG options
        if self.force_tcp:
            # Force RTSP over TCP (more stable)
            self.get_logger().info("Forcing RTSP over TCP via OPENCV_FFMPEG_CAPTURE_OPTIONS")
            # This environment var is read by OpenCV's FFmpeg backend in many builds
            import os
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

        self.cap = None
        self._open_capture()

        period = 1.0 / max(self.publish_fps, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.last_reopen_time = 0.0

        self.get_logger().info(f"Publishing RTSP video to topic: {self.topic}")
        self.get_logger().info(f"RTSP URL: {self.rtsp_url}")

    def _open_capture(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        self.cap = cv2.VideoCapture(self.rtsp_url)
        # Reduce latency if supported (not always effective depending on backend)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open RTSP stream (VideoCapture not opened). Will retry...")
        else:
            self.get_logger().info("RTSP stream opened successfully.")

    def _tick(self):
        if self.cap is None or not self.cap.isOpened():
            self._maybe_reopen()
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to read frame. Will try reopening...")
            self._maybe_reopen()
            return

        # OpenCV gives BGR by default
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.pub.publish(msg)

    def _maybe_reopen(self):
        now = time.time()
        # avoid reopening too frequently
        if now - self.last_reopen_time < 1.0:
            return
        self.last_reopen_time = now
        self._open_capture()


def main():
    rclpy.init()
    node = RTSPImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.cap is not None:
                node.cap.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()