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
gi.require_version("GLib", "2.0")
from gi.repository import Gst, GLib  # noqa: E402


class RtspToRos2Image(Node):
    """
    Subscribe to an RTSP stream using GStreamer and publish sensor_msgs/Image (bgr8).
    Runs GLib main loop in a dedicated thread; GStreamer appsink callback publishes images.
    """

    def __init__(self) -> None:
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

        # Reliability: auto restart
        self.declare_parameter("restart_on_error", True)
        self.declare_parameter("restart_delay_s", 2.0)

        self.rtsp_url: str = str(self.get_parameter("rtsp_url").value)
        self.topic: str = str(self.get_parameter("topic").value)
        self.frame_id: str = str(self.get_parameter("frame_id").value)
        self.fps: float = float(self.get_parameter("fps").value)
        self.use_tcp: bool = bool(self.get_parameter("use_tcp").value)
        self.latency_ms: int = int(self.get_parameter("latency_ms").value)
        self.drop: bool = bool(self.get_parameter("drop").value)
        self.max_buffers: int = int(self.get_parameter("max_buffers").value)

        self.restart_on_error: bool = bool(self.get_parameter("restart_on_error").value)
        self.restart_delay_s: float = float(self.get_parameter("restart_delay_s").value)

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

        # GLib mainloop in another thread (GStreamer callbacks live there)
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)

        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsink = None

        # publish throttling
        self._last_pub_t = 0.0

        # restart control
        self._closing = False
        self._restart_lock = threading.Lock()
        self._restart_pending = False

        # build & start
        self._build_pipeline()

        self.get_logger().info(
            "RTSP -> ROS2 Image\n"
            f"  url: {self.rtsp_url}\n"
            f"  topic: {self.topic}\n"
            f"  frame_id: {self.frame_id}\n"
            f"  fps_limit: {self.fps} (0=no limit)\n"
            f"  latency_ms: {self.latency_ms}, tcp: {self.use_tcp}\n"
            f"  drop: {self.drop}, max_buffers: {self.max_buffers}\n"
            f"  restart_on_error: {self.restart_on_error}, restart_delay_s: {self.restart_delay_s}\n"
        )

        self.loop_thread.start()
        self._start_pipeline()

    # ---------------- Thread / Loop ----------------
    def _run_loop(self) -> None:
        try:
            self.loop.run()
        except Exception as e:
            # In practice, loop.run rarely throws, but keep it safe.
            self.get_logger().error(f"GLib loop exception: {e}")

    # ---------------- Pipeline lifecycle ----------------
    def _build_pipeline(self) -> None:
        # transport=TCP in rtspsrc if use_tcp else default
        # Force output to BGR for ROS Image (bgr8)
        protocols = " protocols=tcp" if self.use_tcp else ""

        # NOTE: quote rtsp_url to avoid parsing issues when url contains ?, &, etc.
        pipeline_str = (
            f"rtspsrc location=\"{self.rtsp_url}\" latency={self.latency_ms}{protocols} ! "
            f"decodebin ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink name=appsink emit-signals=true sync=false "
            f"max-buffers={self.max_buffers} drop={'true' if self.drop else 'false'}"
        )

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            raise RuntimeError(f"Failed to parse pipeline: {e}\nPipeline: {pipeline_str}") from e

        self.appsink = self.pipeline.get_by_name("appsink")
        if self.appsink is None:
            raise RuntimeError("Failed to get appsink from pipeline")

        # Connect callback for each new frame
        self.appsink.connect("new-sample", self._on_new_sample)

        # Bus watch for errors
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _start_pipeline(self) -> None:
        if self.pipeline is None:
            self.get_logger().error("Pipeline is None; cannot start.")
            return

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set pipeline to PLAYING.")
            self._schedule_restart("start_failure")

    def _stop_pipeline(self) -> None:
        if self.pipeline is None:
            return
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass

    def _schedule_restart(self, reason: str) -> None:
        if not self.restart_on_error or self._closing:
            return

        with self._restart_lock:
            if self._restart_pending:
                return
            self._restart_pending = True

        self.get_logger().warn(f"Scheduling pipeline restart (reason={reason}) in {self.restart_delay_s:.1f}s")

        def _do_restart():
            if self._closing:
                return
            try:
                self._restart_pipeline()
            finally:
                with self._restart_lock:
                    self._restart_pending = False

        # Use ROS timer (runs in rclpy executor thread) to restart safely
        self.create_timer(self.restart_delay_s, _do_restart)

    def _restart_pipeline(self) -> None:
        if self._closing:
            return

        self.get_logger().warn("Restarting GStreamer pipeline...")
        try:
            self._stop_pipeline()
        except Exception as e:
            self.get_logger().warn(f"Stop pipeline exception: {e}")

        try:
            # Rebuild pipeline from scratch to recover from broken state
            self._build_pipeline()
            self._start_pipeline()
            self.get_logger().info("Pipeline restarted.")
        except Exception as e:
            self.get_logger().error(f"Pipeline restart failed: {e}")
            # If restart fails, schedule another attempt.
            self._schedule_restart("restart_failed")

    # ---------------- Bus / Callback ----------------
    def _on_bus_message(self, bus, message) -> None:
        mtype = message.type
        if mtype == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer ERROR: {err} debug={debug}")
            self._schedule_restart("gst_error")
        elif mtype == Gst.MessageType.EOS:
            self.get_logger().warn("GStreamer EOS (end of stream)")
            self._schedule_restart("eos")
        elif mtype == Gst.MessageType.STATE_CHANGED:
            # You can enable debug if needed:
            # old, new, pending = message.parse_state_changed()
            # self.get_logger().debug(f"STATE_CHANGED: {old.value_nick}->{new.value_nick}")
            pass

    def _should_publish(self, now_t: float) -> bool:
        if self.fps <= 0.0:
            return True
        period = 1.0 / self.fps
        return (now_t - self._last_pub_t) >= period

    def _on_new_sample(self, sink):
        # Called in GStreamer/GLib thread
        if self._closing:
            return Gst.FlowReturn.OK

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
            try:
                buf.unmap(mapinfo)
            except Exception:
                pass

        return Gst.FlowReturn.OK

    # ---------------- Shutdown ----------------
    def destroy_node(self) -> None:
        self._closing = True

        # Stop pipeline
        try:
            self._stop_pipeline()
        except Exception:
            pass

        # Stop GLib loop
        try:
            if self.loop.is_running():
                self.loop.quit()
        except Exception:
            pass

        # Join thread (best-effort)
        try:
            if self.loop_thread.is_alive():
                self.loop_thread.join(timeout=1.0)
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
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
