#!/usr/bin/env python3
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image

# GStreamer (PyGObject)
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402


class RtspToRos2Image(Node):
    def __init__(self):
        super().__init__("rtsp_to_ros2_image")

        # ---------------- Parameters ----------------
        self.declare_parameter("rtsp_url", "rtsp://10.21.31.103:8554/video1")
        self.declare_parameter("topic", "/camera/front/image_raw")
        self.declare_parameter("frame_id", "camera_front_optical")
        self.declare_parameter("fps", 15.0)          # 0 = publish every frame (no limit)
        self.declare_parameter("use_tcp", True)      # RTSP over TCP is usually more stable
        self.declare_parameter("latency_ms", 100)    # rtspsrc latency buffer
        self.declare_parameter("drop", True)         # appsink drop frames if slow
        self.declare_parameter("max_buffers", 1)     # appsink buffer queue

        self.rtsp_url: str = self.get_parameter("rtsp_url").value
        self.topic: str = self.get_parameter("topic").value
        self.frame_id: str = self.get_parameter("frame_id").value
        self.fps: float = float(self.get_parameter("fps").value)
        self.use_tcp: bool = bool(self.get_parameter("use_tcp").value)
        self.latency_ms: int = int(self.get_parameter("latency_ms").value)
        self.drop: bool = bool(self.get_parameter("drop").value)
        self.max_buffers: int = int(self.get_parameter("max_buffers").value)

        # ---------------- QoS ----------------
        # 图像通常用 BEST_EFFORT + VOLATILE（RViz/工具更顺）
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Image, self.topic, qos)

        # ---------------- GStreamer ----------------
        Gst.init(None)

        # We run GLib mainloop in another thread (GStreamer callbacks live there)
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)

        self.pipeline = None
        self.appsink = None

        # publish throttling
        self._last_pub_t = 0.0

        self._build_pipeline()

        self.get_logger().info(
            f"RTSP -> ROS2 Image\n"
            f"  url: {self.rtsp_url}\n"
            f"  topic: {self.topic}\n"
            f"  frame_id: {self.frame_id}\n"
            f"  fps_limit: {self.fps} (0=no limit)\n"
            f"  latency_ms: {self.latency_ms}, tcp: {self.use_tcp}\n"
        )

        self.loop_thread.start()
        self._start_pipeline()

    def _build_pipeline(self):
        # transport=TCP(4) in rtspsrc if use_tcp else default
        # We force output to BGR for easy ROS Image (bgr8)
        protocols = " protocols=tcp" if self.use_tcp else ""
        pipeline_str = (
            f"rtspsrc location={self.rtsp_url} latency={self.latency_ms}{protocols} ! "
            f"decodebin ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink name=appsink emit-signals=true sync=false "
            f"max-buffers={self.max_buffers} drop={'true' if self.drop else 'false'}"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        if self.appsink is None:
            raise RuntimeError("Failed to get appsink from pipeline")

        # Connect callback for each new frame
        self.appsink.connect("new-sample", self._on_new_sample)

        # Bus watch for errors
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _start_pipeline(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to set pipeline to PLAYING")

    def _stop_pipeline(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass

    def _on_bus_message(self, bus, message):
        mtype = message.type
        if mtype == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer ERROR: {err} debug={debug}")
        elif mtype == Gst.MessageType.EOS:
            self.get_logger().warn("GStreamer EOS (end of stream)")
        elif mtype == Gst.MessageType.STATE_CHANGED:
            # you can enable debug prints if you want
            pass

    def _should_publish(self, now_t: float) -> bool:
        if self.fps <= 0.0:
            return True
        period = 1.0 / self.fps
        return (now_t - self._last_pub_t) >= period

    def _on_new_sample(self, sink):
        # Called in GStreamer/GLib thread
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        now_t = time.time()
        if not self._should_publish(now_t):
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK

        try:
            data = mapinfo.data  # bytes-like
            # Build ROS Image
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height = int(height)
            msg.width = int(width)
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(width) * 3
            msg.data = bytes(data)

            self.pub.publish(msg)
            self._last_pub_t = now_t
        finally:
            buf.unmap(mapinfo)

        return Gst.FlowReturn.OK

    def destroy_node(self):
        self._stop_pipeline()
        try:
            if self.loop.is_running():
                self.loop.quit()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node: Optional[RtspToRos2Image] = None
    try:
        node = RtspToRos2Image()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
