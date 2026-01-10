#!/usr/bin/env python3
"""
UDP Bridge with configurable robot IP
支持通过树莓派网关连接机器人
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import json
import struct
import time
import sys
import os

# 从环境变量读取机器人 IP，或使用默认值
ROBOT_IP = os.getenv('PUMA_ROBOT_IP', '10.21.31.103')
ROBOT_PORT = int(os.getenv('PUMA_ROBOT_PORT', '30000'))

# Protocol Constants
SYNC_BYTES = bytes([0xEB, 0x91, 0xEB, 0x90])

def get_timestamp():
    """Returns current time formatted for the robot protocol."""
    return time.strftime("%Y-%m-%d %H:%M:%S")

def build_header(payload_len, msg_id, is_json=True):
    """Constructs the 16-byte protocol header."""
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
        
        # Message ID counter
        self.message_id = 0
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/puma/control', self.control_callback, 10)
        
        # Heartbeat Timer (2 Hz)
        self.create_timer(0.5, self.send_heartbeat)
        
        self.get_logger().info(f"=" * 50)
        self.get_logger().info(f"UDP Bridge Started")
        self.get_logger().info(f"Target Robot: {ROBOT_IP}:{ROBOT_PORT}")
        self.get_logger().info(f"Listening on topics: /cmd_vel, /puma/control")
        self.get_logger().info(f"=" * 50)
        
        # 测试连接
        self.test_connection()

    def test_connection(self):
        """测试到机器人的 UDP 连接"""
        try:
            # 发送一个测试心跳包
            test_payload = create_json_payload(100, 100)
            test_header = build_header(len(test_payload), 0, is_json=True)
            test_data = test_header + test_payload
            
            self.sock.sendto(test_data, (ROBOT_IP, ROBOT_PORT))
            self.get_logger().info(f"✓ 测试数据包已发送到 {ROBOT_IP}:{ROBOT_PORT}")
        except Exception as e:
            self.get_logger().error(f"✗ 连接测试失败: {e}")
            self.get_logger().error(f"  请检查:")
            self.get_logger().error(f"  1. 机器人 IP 是否正确: {ROBOT_IP}")
            self.get_logger().error(f"  2. 网络连接是否正常")
            self.get_logger().error(f"  3. 防火墙是否阻止 UDP 端口 {ROBOT_PORT}")

    def send_message(self, type_code, command_code, items=None):
        """Send a message using the robot protocol"""
        try:
            payload = create_json_payload(type_code, command_code, items)
            header = build_header(len(payload), self.message_id, is_json=True)
            data = header + payload
            
            self.sock.sendto(data, (ROBOT_IP, ROBOT_PORT))
            self.message_id = (self.message_id + 1) % 65536
            return True
        except Exception as e:
            self.get_logger().error(f"UDP Send Error: {e}")
            return False

    def send_heartbeat(self):
        """Sends Type 100 Heartbeat to keep connection alive."""
        self.send_message(100, 100)

    def cmd_vel_callback(self, msg):
        """Translates /cmd_vel to Type 2 Command 21 (Motion Control)."""
        items = {
            "X": float(msg.linear.x),
            "Y": float(msg.linear.y),
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": float(msg.angular.z)
        }
        if self.send_message(2, 21, items):
            self.get_logger().info(
                f"→ cmd_vel: X={msg.linear.x:.2f} Y={msg.linear.y:.2f} Yaw={msg.angular.z:.2f}"
            )

    def control_callback(self, msg):
        """Translates /puma/control to Type 2 Command 22 (Motion State)."""
        cmd = msg.data.lower()
        state_id = None
        
        state_map = {
            "stand": 1,
            "sit": 4,
            "damping": 3,
            "standard": 6,
            "rl": 17
        }
        
        state_id = state_map.get(cmd)
        
        if state_id is not None:
            if self.send_message(2, 22, {"MotionParam": state_id}):
                self.get_logger().info(f"→ Control: {cmd.upper()} (State {state_id})")
        else:
            self.get_logger().warning(f"Unknown control command: {cmd}")

def main(args=None):
    print("\n" + "=" * 50)
    print("PUMA UDP ROS2 Bridge")
    print("=" * 50)
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Robot Port: {ROBOT_PORT}")
    print("")
    print("提示: 可通过环境变量配置:")
    print("  export PUMA_ROBOT_IP=<树莓派IP或机器人IP>")
    print("  export PUMA_ROBOT_PORT=<端口号>")
    print("=" * 50 + "\n")
    
    rclpy.init(args=args)
    node = UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
