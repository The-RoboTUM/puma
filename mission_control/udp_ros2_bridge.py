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

class UdpBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/puma/control', self.control_callback, 10)
        
        # Heartbeat Timer (10 Hz)
        self.create_timer(0.5, self.send_heartbeat)
        
        self.get_logger().info(f"UDP Bridge Started. Target: {ROBOT_IP}:{ROBOT_PORT}")

    def pack_message(self, json_data):
        """Packs JSON data into the binary protocol format."""
        json_str = json.dumps(json_data)
        data_bytes = json_str.encode('utf-8')
        length = len(data_bytes)
        
        # Header (16 bytes)
        # 0-3: Sync (EB 91 EB 90)
        # 4-5: Length (Little Endian)
        # 6: 0x01
        # 7: 0x00
        # 8: 0x01
        # 9-15: Reserved (0)
        header = bytearray(16)
        header[0] = 0xEB
        header[1] = 0x91
        header[2] = 0xEB
        header[3] = 0x90
        header[4] = length & 0xFF
        header[5] = (length >> 8) & 0xFF
        header[6] = 0x01
        header[7] = 0x00
        header[8] = 0x01
        
        return header + data_bytes

    def send_udp(self, payload):
        try:
            packet = self.pack_message(payload)
            self.sock.sendto(packet, (ROBOT_IP, ROBOT_PORT))
        except Exception as e:
            self.get_logger().error(f"UDP Send Error: {e}")

    def send_heartbeat(self):
        """Sends Type 100 Heartbeat to keep connection alive."""
        payload = {
            "PatrolDevice": {
                "Type": 100,
                "Command": 100,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "Items": {}
            }
        }
        self.send_udp(payload)

    def cmd_vel_callback(self, msg):
        """Translates /cmd_vel to Type 2 Command 21 (Motion Control)."""
        # Map Twist to Robot Velocity
        # Linear.x -> VelocityX
        # Linear.y -> VelocityY
        # Angular.z -> VelocityYaw
        
        payload = {
            "PatrolDevice": {
                "Type": 2,
                "Command": 21,
                "Items": {
                    "VelocityX": float(msg.linear.x),
                    "VelocityY": float(msg.linear.y),
                    "VelocityYaw": float(msg.angular.z)
                }
            }
        }
        self.send_udp(payload)

    def control_callback(self, msg):
        """Translates /puma/control to Type 2 Command 22 (Motion State)."""
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
            payload = {
                "PatrolDevice": {
                    "Type": 2,
                    "Command": 22,
                    "Items": {
                        "MotionState": state_id
                    }
                }
            }
            self.send_udp(payload)
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
