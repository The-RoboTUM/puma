#!/usr/bin/env python3
"""
Bidirectional Topic Relay Client with Integrated Teleop

This runs on the PC and:
1. Connects to the robot's topic_relay_server via TCP
2. Has integrated keyboard teleop that sends commands over TCP
3. Receives /IMU data and republishes it locally for RViz

Architecture:
  [PC: This Client] --TCP:9999--> [Robot: topic_relay_server] --> [Robot: udp_ros2_bridge] --> [Robot]
       ^                                    |
       |                                    |
   [RViz] <-- local /IMU             Receives /cmd_vel, /puma/control
                                     Publishes to local ROS2
"""

import socket
import threading
import struct
import json
import sys
import termios
import tty
import select
import signal
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Configuration
ROBOT_IP = "10.21.41.1"  # Robot's WiFi hotspot IP
RELAY_PORT = 9999

# Teleop key bindings
BINDINGS = {
    'w': ('linear', 0.5),   # Forward
    's': ('linear', -0.5),  # Backward
    'a': ('angular', 0.5),  # Turn left
    'd': ('angular', -0.5), # Turn right
    'q': ('control', 'stand'),
    'e': ('control', 'sit'),
    'r': ('control', 'walk'),
    't': ('control', 'stop'),
    '1': ('control', 'mode1'),
    '2': ('control', 'mode2'),
    '3': ('control', 'mode3'),
    ' ': ('stop', None),    # Emergency stop
}


class TopicRelayClient(Node):
    """ROS2 node that connects to robot via TCP and provides local topic access."""
    
    def __init__(self):
        super().__init__('topic_relay_client')
        
        # Publishers for received data
        self.imu_pub = self.create_publisher(Imu, '/IMU', 10)
        
        # TCP connection
        self.socket = None
        self.connected = False
        self.recv_thread = None
        self.running = True
        
        # Teleop state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Stats
        self.msgs_received = 0
        self.msgs_sent = 0
        
        # Connect to robot
        self.connect_to_robot()
        
        # Start receiver thread
        if self.connected:
            self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.recv_thread.start()
        
        self.get_logger().info(f"Topic Relay Client initialized")
        self.get_logger().info(f"Connected to robot at {ROBOT_IP}:{RELAY_PORT}")
    
    def connect_to_robot(self):
        """Establish TCP connection to robot."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((ROBOT_IP, RELAY_PORT))
            self.socket.settimeout(None)  # Remove timeout for normal operation
            self.connected = True
            self.get_logger().info(f"Connected to robot at {ROBOT_IP}:{RELAY_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")
            self.connected = False
    
    def receive_loop(self):
        """Receive messages from robot and republish locally."""
        buffer = b""
        
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    self.get_logger().warn("Connection closed by robot")
                    self.connected = False
                    break
                
                buffer += data
                
                # Process complete messages (4-byte length prefix + JSON)
                while len(buffer) >= 4:
                    msg_len = struct.unpack('>I', buffer[:4])[0]
                    if len(buffer) < 4 + msg_len:
                        break  # Wait for more data
                    
                    msg_data = buffer[4:4+msg_len]
                    buffer = buffer[4+msg_len:]
                    
                    try:
                        msg = json.loads(msg_data.decode('utf-8'))
                        self.handle_message(msg)
                    except json.JSONDecodeError as e:
                        self.get_logger().warn(f"Invalid JSON: {e}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Receive error: {e}")
                break
    
    def handle_message(self, msg):
        """Handle received message from robot."""
        topic = msg.get('topic', '')
        data = msg.get('data', {})
        
        if topic == '/IMU':
            self.publish_imu(data)
            self.msgs_received += 1
        else:
            self.get_logger().debug(f"Unknown topic: {topic}")
    
    def publish_imu(self, data):
        """Republish IMU data locally."""
        imu_msg = Imu()
        
        # Orientation
        imu_msg.orientation.x = data.get('orientation', {}).get('x', 0.0)
        imu_msg.orientation.y = data.get('orientation', {}).get('y', 0.0)
        imu_msg.orientation.z = data.get('orientation', {}).get('z', 0.0)
        imu_msg.orientation.w = data.get('orientation', {}).get('w', 1.0)
        
        # Angular velocity
        imu_msg.angular_velocity.x = data.get('angular_velocity', {}).get('x', 0.0)
        imu_msg.angular_velocity.y = data.get('angular_velocity', {}).get('y', 0.0)
        imu_msg.angular_velocity.z = data.get('angular_velocity', {}).get('z', 0.0)
        
        # Linear acceleration
        imu_msg.linear_acceleration.x = data.get('linear_acceleration', {}).get('x', 0.0)
        imu_msg.linear_acceleration.y = data.get('linear_acceleration', {}).get('y', 0.0)
        imu_msg.linear_acceleration.z = data.get('linear_acceleration', {}).get('z', 0.0)
        
        self.imu_pub.publish(imu_msg)
    
    def send_message(self, topic: str, data: dict):
        """Send a message to the robot."""
        if not self.connected:
            return False
        
        try:
            msg = {'topic': topic, 'data': data}
            msg_bytes = json.dumps(msg).encode('utf-8')
            length_prefix = struct.pack('>I', len(msg_bytes))
            self.socket.sendall(length_prefix + msg_bytes)
            self.msgs_sent += 1
            return True
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")
            self.connected = False
            return False
    
    def send_velocity(self, linear: float, angular: float):
        """Send velocity command to robot."""
        data = {
            'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
        }
        self.send_message('/cmd_vel', data)
    
    def send_control(self, command: str):
        """Send control command to robot."""
        data = {'data': command}
        self.send_message('/puma/control', data)
    
    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass


def get_key(timeout=0.1):
    """Get a single keypress (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            return key
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_status(client, linear, angular):
    """Print current status."""
    status = "CONNECTED" if client.connected else "DISCONNECTED"
    print(f"\r[{status}] Vel: lin={linear:+.2f} ang={angular:+.2f} | Rx:{client.msgs_received} Tx:{client.msgs_sent}   ", end='', flush=True)


def main():
    print("""
╔═══════════════════════════════════════════════════════════════╗
║            PUMA Topic Relay Client with Teleop                ║
╠═══════════════════════════════════════════════════════════════╣
║  MOVEMENT:                      COMMANDS:                     ║
║    W - Forward                    Q - Stand                   ║
║    S - Backward                   E - Sit                     ║
║    A - Turn Left                  R - Walk Mode               ║
║    D - Turn Right                 T - Stop Mode               ║
║    SPACE - Emergency Stop         1/2/3 - Mode 1/2/3          ║
║                                                               ║
║  Press Ctrl+C to exit                                         ║
╚═══════════════════════════════════════════════════════════════╝
""")
    
    # Initialize ROS2
    rclpy.init()
    client = TopicRelayClient()
    
    if not client.connected:
        print("\n❌ Failed to connect to robot. Check that:")
        print(f"   1. Robot's topic_relay_server is running on {ROBOT_IP}")
        print(f"   2. You're connected to the robot's WiFi")
        print(f"   3. Port {RELAY_PORT} is accessible")
        rclpy.shutdown()
        return
    
    print(f"\n✅ Connected to robot at {ROBOT_IP}:{RELAY_PORT}")
    print("   Topics available locally: /IMU")
    print("   Use 'ros2 topic list' in another terminal to see them")
    print("\n   Start RViz2: rviz2")
    print("\n   Waiting for commands...")
    
    # Velocity state
    linear_vel = 0.0
    angular_vel = 0.0
    
    # ROS2 spinner in background
    spin_thread = threading.Thread(target=lambda: rclpy.spin(client), daemon=True)
    spin_thread.start()
    
    # Handle Ctrl+C
    def signal_handler(sig, frame):
        print("\n\nShutting down...")
        client.send_velocity(0.0, 0.0)  # Stop robot
        client.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Main loop
    try:
        while client.connected:
            key = get_key(0.1)
            
            if key:
                key = key.lower()
                
                if key in BINDINGS:
                    action, value = BINDINGS[key]
                    
                    if action == 'linear':
                        linear_vel = value
                        client.send_velocity(linear_vel, angular_vel)
                    elif action == 'angular':
                        angular_vel = value
                        client.send_velocity(linear_vel, angular_vel)
                    elif action == 'control':
                        client.send_control(value)
                        print(f"\n>>> Sent control command: {value}")
                    elif action == 'stop':
                        linear_vel = 0.0
                        angular_vel = 0.0
                        client.send_velocity(0.0, 0.0)
                        print(f"\n>>> EMERGENCY STOP")
                
                elif key == '\x03':  # Ctrl+C
                    break
            
            else:
                # No key pressed, decay velocity
                if abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01:
                    linear_vel *= 0.8
                    angular_vel *= 0.8
                    if abs(linear_vel) < 0.01:
                        linear_vel = 0.0
                    if abs(angular_vel) < 0.01:
                        angular_vel = 0.0
                    client.send_velocity(linear_vel, angular_vel)
            
            print_status(client, linear_vel, angular_vel)
            
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\n\nStopping robot and disconnecting...")
        client.send_velocity(0.0, 0.0)
        client.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
