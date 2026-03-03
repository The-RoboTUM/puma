#!/usr/bin/env python3
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DualRTSPImagePublisher(Node):
    def __init__(self):
        super().__init__("ptz_dual_image_publisher")

        # -----------------------
        # Parameters
        # -----------------------
        # Default URLs from your ONVIF discovery (Main streams)
        self.declare_parameter(
            "rtsp_url_rgb",
            "rtsp://admin:puma2126@192.168.1.108:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif",
        )
        self.declare_parameter(
            "rtsp_url_thermal",
            "rtsp://admin:puma2126@192.168.1.108:554/cam/realmonitor?channel=2&subtype=0&unicast=true&proto=Onvif",
        )

        self.declare_parameter("topic_rgb", "/ptz/rgb/image_raw")
        self.declare_parameter("topic_thermal", "/ptz/thermal/image_raw")
        self.declare_parameter("frame_id_rgb", "ptz_rgb")
        self.declare_parameter("frame_id_thermal", "ptz_thermal")

        self.declare_parameter("publish_fps", 15.0)
        self.declare_parameter("force_tcp", True)

        # Optional: if OpenCV's default backend (often GStreamer) is flaky,
        # you can force GStreamer pipeline by setting use_gstreamer_pipeline=True
        # (requires gstreamer plugins installed)
        self.declare_parameter("use_gstreamer_pipeline", False)

        # -----------------------
        # Read parameters
        # -----------------------
        self.rtsp_url_rgb = self.get_parameter("rtsp_url_rgb").value
        self.rtsp_url_thermal = self.get_parameter("rtsp_url_thermal").value

        self.topic_rgb = self.get_parameter("topic_rgb").value
        self.topic_thermal = self.get_parameter("topic_thermal").value

        self.frame_id_rgb = self.get_parameter("frame_id_rgb").value
        self.frame_id_thermal = self.get_parameter("frame_id_thermal").value

        self.publish_fps = float(self.get_parameter("publish_fps").value)
        self.force_tcp = bool(self.get_parameter("force_tcp").value)
        self.use_gst = bool(self.get_parameter("use_gstreamer_pipeline").value)

        # -----------------------
        # Publishers / Bridge
        # -----------------------
        self.pub_rgb = self.create_publisher(Image, self.topic_rgb, 10)
        self.pub_thermal = self.create_publisher(Image, self.topic_thermal, 10)
        self.bridge = CvBridge()

        # -----------------------
        # Force RTSP over TCP (FFmpeg backend)
        # -----------------------
        if self.force_tcp:
            # Many OpenCV builds (FFmpeg backend) honor this env var
            self.get_logger().info("Forcing RTSP over TCP via OPENCV_FFMPEG_CAPTURE_OPTIONS")
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

        # -----------------------
        # Open streams
        # -----------------------
        self.cap_rgb = None
        self.cap_thermal = None
        self.last_reopen_time = 0.0

        self._open_captures()

        period = 1.0 / max(self.publish_fps, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"Publishing RGB to: {self.topic_rgb}")
        self.get_logger().info(f"RGB RTSP URL: {self.rtsp_url_rgb}")

        self.get_logger().info(f"Publishing Thermal to: {self.topic_thermal}")
        self.get_logger().info(f"Thermal RTSP URL: {self.rtsp_url_thermal}")

    def _make_gst_pipeline(self, rtsp_url: str) -> str:
        # A robust low-latency pipeline for RTSP -> decode -> appsink
        # protocols=tcp enforces TCP at the RTSP level (different from FFmpeg env var)
        # latency can be tuned (0~200). Too low may stutter; 50~200 often stable.
        return (
            f'rtspsrc location="{rtsp_url}" protocols=tcp latency=100 ! '
            "decodebin ! videoconvert ! appsink drop=true sync=false"
        )

    def _open_one_capture(self, rtsp_url: str):
        if self.use_gst:
            pipeline = self._make_gst_pipeline(rtsp_url)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(rtsp_url)

        # Reduce latency if supported
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        return cap

    def _open_captures(self):
        # Release old ones
        for cap in (self.cap_rgb, self.cap_thermal):
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass

        self.cap_rgb = self._open_one_capture(self.rtsp_url_rgb)
        self.cap_thermal = self._open_one_capture(self.rtsp_url_thermal)

        if not self.cap_rgb.isOpened():
            self.get_logger().error("Failed to open RGB RTSP stream (VideoCapture not opened).")
        else:
            self.get_logger().info("RGB RTSP stream opened successfully.")

        if not self.cap_thermal.isOpened():
            self.get_logger().error("Failed to open Thermal RTSP stream (VideoCapture not opened).")
        else:
            self.get_logger().info("Thermal RTSP stream opened successfully.")

    def _maybe_reopen(self):
        now = time.time()
        # avoid reopening too frequently
        if now - self.last_reopen_time < 1.0:
            return
        self.last_reopen_time = now
        self.get_logger().warn("Reopening RTSP streams...")
        self._open_captures()

    def _read_publish_one(self, cap, publisher, frame_id: str) -> bool:
        if cap is None or not cap.isOpened():
            return False

        ret, frame = cap.read()
        if not ret or frame is None:
            return False

        # Most thermal "video" streams are false-color BGR, so bgr8 is correct.
        # If your thermal stream is grayscale, you can switch encoding to "mono8"
        # (but only if the frame really is 1-channel).
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        publisher.publish(msg)
        return True

    def _tick(self):
        ok_rgb = self._read_publish_one(self.cap_rgb, self.pub_rgb, self.frame_id_rgb)
        ok_th = self._read_publish_one(self.cap_thermal, self.pub_thermal, self.frame_id_thermal)

        if not ok_rgb:
            self.get_logger().warn("RGB: failed to read frame.")
        if not ok_th:
            self.get_logger().warn("Thermal: failed to read frame.")

        if (not ok_rgb) or (not ok_th):
            self._maybe_reopen()


def main():
    rclpy.init()
    node = DualRTSPImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.cap_rgb is not None:
                node.cap_rgb.release()
            if node.cap_thermal is not None:
                node.cap_thermal.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()