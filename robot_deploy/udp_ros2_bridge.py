#!/usr/bin/env python3
"""
ROS2 to UDP Bridge for PUMA Robot (DeepRobotics Lynx M20)

Subscribes to ROS2 topics and translates them to the robot's UDP protocol.

Topics:
    /cmd_vel (geometry_msgs/Twist) - Velocity commands
    /puma/control (std_msgs/String) - Motion state: stand, sit, damping, standard, rl
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import json
import struct
import time

# Robot network configuration
ROBOT_IP = "10.21.31.103"
ROBOT_PORT = 30000

# Protocol constants
SYNC_BYTES = bytes([0xEB, 0x91, 0xEB, 0x90])

# Motion state mapping
MOTION_STATES = {
    "stand": 1,
    "sit": 4,
    "damping": 3,
    "standard": 6,
    "rl": 17,
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


class UdpBridge(Node):
    def __init__(self):
        super().__init__('puma_udp_bridge')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.msg_id = 0
        
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/puma/control', self._on_control, 10)
        self.create_timer(0.5, self._send_heartbeat)
        
        self.get_logger().info(f"Bridge started -> {ROBOT_IP}:{ROBOT_PORT}")

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
        """Handle velocity commands."""
        items = {
            "X": msg.linear.x,
            "Y": msg.linear.y,
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": msg.angular.z
        }
        self._send(2, 21, items)

    def _on_control(self, msg: String):
        """Handle motion state commands."""
        cmd = msg.data.lower()
        if cmd in MOTION_STATES:
            self._send(2, 22, {"MotionParam": MOTION_STATES[cmd]})
            self.get_logger().info(f"Motion: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
