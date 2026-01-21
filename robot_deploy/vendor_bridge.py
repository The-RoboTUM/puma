#!/usr/bin/env python3
"""
ROS2 to Vendor UDP Bridge for PUMA Robot (DeepRobotics Lynx M20)
Run this ON THE ROBOT via SSH.

Subscribes to ROS2 topics and translates them to the robot's UDP protocol.

Topics:
    /cmd_vel (geometry_msgs/Twist) - Velocity commands
    /puma/control (std_msgs/String) - Motion state: stand, sit, damping, standard
    /puma/gait (std_msgs/String) - Gait: basic, stair
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import json
import struct
import time
import subprocess
import threading

# Robot network configuration - send to robot's documented UDP server IP
ROBOT_IP = "10.21.31.103"
ROBOT_PORT = 30000

# Protocol constants
SYNC_BYTES = bytes([0xEB, 0x91, 0xEB, 0x90])

# Motion state mapping (vendor controller)
MOTION_STATES = {
    "stand": 1,
    "sit": 4,
    "damping": 3,
    "standard": 6,  # This enables walking with vendor controller!
}

# Gait mapping (vendor controller)
GAITS = {
    "basic": 1,
    "stair": 14,
}


def build_header(payload_len: int, msg_id: int) -> bytes:
    """Build 16-byte protocol header."""
    return (
        SYNC_BYTES +
        struct.pack('<H', payload_len) +
        struct.pack('<H', msg_id) +
        bytes([0x01]) +  # JSON flag
        bytes(7)
    )


def build_payload(type_code: int, cmd_code: int, items: dict = None) -> bytes:
    """Build JSON payload."""
    payload = {
        "PatrolDevice": {
            "Type": type_code,
            "Command": cmd_code,
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Items": items or {}
        }
    }
    return json.dumps(payload).encode('utf-8')



class VendorBridge(Node):
    def __init__(self, enable_video=True, rtsp_url='rtsp://10.21.31.103:8554/video1'):
        super().__init__('puma_vendor_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.msg_id = 0
        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/puma/control', self._on_control, 10)
        self.create_subscription(String, '/puma/gait', self._on_gait, 10)
        # Heartbeat timer
        self.create_timer(0.5, self._send_heartbeat)
        # Keep-alive for DDS discovery
        self.create_timer(0.1, lambda: None)
        self.get_logger().info(
            f"Vendor Bridge started -> {ROBOT_IP}:{ROBOT_PORT}\n"
            f"  /cmd_vel      -> velocity\n"
            f"  /puma/control -> stand, sit, damping, standard\n"
            f"  /puma/gait    -> basic, stair"
        )
        # Video publisher using gscam
        self.enable_video = enable_video
        if self.enable_video:
            self.rtsp_url = rtsp_url
            self._video_process = None
            self._start_gscam()

    def _start_gscam(self):
        """Start gscam for RTSP video streaming."""
        self.get_logger().info(f"Starting gscam for RTSP stream: {self.rtsp_url}")
        # GStreamer pipeline for RTSP HEVC (H.265)
        pipeline = f"rtspsrc location={self.rtsp_url} protocols=tcp latency=100 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! videorate ! video/x-raw,framerate=30/1"
        
        try:
            # Launch gscam with the pipeline
            cmd = [
                'ros2', 'run', 'gscam', 'gscam_node',
                '__params:=', f'gscam_config:="{pipeline}"'
            ]
            self._video_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Check if process started
            import time
            time.sleep(2)  # Wait a bit for startup
            if self._video_process.poll() is None:
                self.get_logger().info("gscam started successfully")
                # Check stderr for errors
                if self._video_process.stderr:
                    stderr_data = self._video_process.stderr.read(1024)
                    if stderr_data:
                        self.get_logger().warn(f"gscam stderr: {stderr_data.decode()}")
            else:
                stdout, stderr = self._video_process.communicate()
                self.get_logger().error(f"gscam failed: stdout={stdout.decode()}, stderr={stderr.decode()}")
        except Exception as e:
            self.get_logger().error(f"Failed to start gscam: {e}")

    def __del__(self):
        """Clean up video process on shutdown."""
        if self._video_process:
            self._video_process.terminate()
            self._video_process.wait()

    def _send(self, type_code: int, cmd_code: int, items: dict = None):
        """Send UDP message to robot."""
        try:
            payload = build_payload(type_code, cmd_code, items)
            header = build_header(len(payload), self.msg_id)
            self.sock.sendto(header + payload, (ROBOT_IP, ROBOT_PORT))
            self.msg_id = (self.msg_id + 1) % 65536
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")

    def _send_heartbeat(self):
        """Send heartbeat to keep connection alive."""
        self._send(100, 100)

    def _on_cmd_vel(self, msg: Twist):
        """Handle velocity commands (Type 2, Cmd 21)."""
        items = {
            "X": float(msg.linear.x),
            "Y": float(msg.linear.y),
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": float(msg.angular.z)
        }
        self._send(2, 21, items)

    def _on_control(self, msg: String):
        """Handle motion state commands (Type 2, Cmd 22)."""
        cmd = msg.data.lower()
        if cmd in MOTION_STATES:
            self._send(2, 22, {"MotionParam": MOTION_STATES[cmd]})
            self.get_logger().info(f"Motion: {cmd} ({MOTION_STATES[cmd]})")
        else:
            self.get_logger().warn(f"Unknown control: {cmd}. Use: stand, sit, damping, standard")

    def _on_gait(self, msg: String):
        """Handle gait commands (Type 2, Cmd 23)."""
        cmd = msg.data.lower()
        if cmd in GAITS:
            self._send(2, 23, {"GaitParam": GAITS[cmd]})
            self.get_logger().info(f"Gait: {cmd} ({GAITS[cmd]})")
        else:
            self.get_logger().warn(f"Unknown gait: {cmd}. Use: basic, stair")



def main(args=None):
    rclpy.init(args=args)
    node = VendorBridge(enable_video=True, rtsp_url='rtsp://10.21.31.103:8554/video1')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up video process
        if hasattr(node, '_video_process') and node._video_process:
            node._video_process.terminate()
            node._video_process.wait()
        node.sock.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
