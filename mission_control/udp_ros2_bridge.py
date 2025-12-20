#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import json
import struct
import time
import threading

# Robot Configuration
ROBOT_IP = "10.21.31.103"
ROBOT_PORT = 30000

# Protocol Constants (matching protocol.py)
SYNC_BYTES = bytes([0xEB, 0x91, 0xEB, 0x90])

def get_timestamp():
    """Returns current time formatted for the robot protocol."""
    return time.strftime("%Y-%m-%d %H:%M:%S")

def build_header(payload_len, msg_id, is_json=True):
    """Constructs the 16-byte protocol header (matching protocol.py)."""
    return (
        SYNC_BYTES +
        struct.pack('<H', payload_len) +
        struct.pack('<H', msg_id) +
        bytes([0x01 if is_json else 0x00]) +
        bytes(7)  # Reserved
    )

def create_json_payload(type_code, cmd_code, items=None):
    """Creates a JSON payload encoded in bytes."""
    payload = {
        "PatrolDevice": {
            "Type": type_code,
            "Command": cmd_code,
            "Time": get_timestamp(),
            "Items": items or {}
        }
    }
    return json.dumps(payload).encode('utf-8')

class UdpBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        
        
        # Message ID counter (matching teleop_robot.py)
        self.message_id = 0
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/puma/control', self.control_callback, 10)
        
        # Heartbeat Timer (2 Hz to match HEARTBEAT_FREQ in protocol.py)
        self.create_timer(0.5, self.send_heartbeat)
        
        self.get_logger().info(f"UDP Bridge Started. Target: {ROBOT_IP}:{ROBOT_PORT}")

    def send_message(self, type_code, command_code, items=None):
        """Send a message using the same protocol as teleop_robot.py"""
        try:
            payload = create_json_payload(type_code, command_code, items)
            header = build_header(len(payload), self.message_id, is_json=True)
            data = header + payload
            
            self.sock.sendto(data, (ROBOT_IP, ROBOT_PORT))
            self.message_id = (self.message_id + 1) % 65536
        except Exception as e:
            self.get_logger().error(f"UDP Send Error: {e}")

    def send_heartbeat(self):
        """Sends Type 100 Heartbeat to keep connection alive."""
        self.send_message(100, 100)

    def cmd_vel_callback(self, msg):
        """Translates /cmd_vel to Type 2 Command 21 (Motion Control)."""
        # Use same field names as teleop_robot.py: X, Y, Z, Roll, Pitch, Yaw
        items = {
            "X": float(msg.linear.x),
            "Y": float(msg.linear.y),
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": float(msg.angular.z)
        }
        self.send_message(2, 21, items)
        self.get_logger().info(f"cmd_vel: X={msg.linear.x:.2f} Y={msg.linear.y:.2f} Yaw={msg.angular.z:.2f}")

    def control_callback(self, msg):
        """Translates /puma/control to Type 2 Command 22 (Motion State)."""
        self.get_logger().info(f"Received control command: {msg.data}")
        cmd = msg.data.lower()
        state_id = None
        
        if cmd == "stand":
            state_id = 1
        elif cmd == "sit":
            state_id = 4
        elif cmd == "damping":
            state_id = 3
        elif cmd == "standard":
            state_id = 6
        elif cmd == "rl":
            state_id = 17
            
        if state_id is not None:
            # Use "MotionParam" instead of "MotionState" (matching teleop_robot.py)
            self.send_message(2, 22, {"MotionParam": state_id})
            self.get_logger().info(f"Sent State Change: {cmd} ({state_id})")

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
