#!/usr/bin/env python3
"""
Keyboard Teleop for PUMA Robot via ROS2
Run this ON THE ROBOT via SSH (requires vendor_bridge.py running).

Publishes to /cmd_vel and /puma/control for the UDP bridge.
"""
import sys
import time
import termios
import tty
import select
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String

HELP_TEXT = """
PUMA Keyboard Teleop (Vendor Controller)
=========================================
Movement:        Commands:
  q   w   e      u - Stand
  a   s   d      z - Sit
                 m - Damping
w/s: forward     6 - Standard (walk mode)
a/d: strafe      
q/e: rotate      Gaits:
                 b - Basic
+/-: speed       t - Stair

                 Space: Stop
                 Ctrl+C: Quit

Workflow: u (stand) → 6 (standard) → wasd (move)
"""

SPEED = 0.3
TURN = 0.5

MOVE_BINDINGS = {
    'w': (SPEED, 0, 0),
    's': (-SPEED, 0, 0),
    'a': (0, SPEED, 0),
    'd': (0, -SPEED, 0),
    'q': (0, 0, TURN),
    'e': (0, 0, -TURN),
}

CONTROL_BINDINGS = {
    'u': 'stand',
    'z': 'sit',
    'm': 'damping',
    '6': 'standard',
}

GAIT_BINDINGS = {
    'b': 'basic',
    't': 'stair',
}


def get_key(settings, timeout=0.1):
    """Read a single keypress."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('puma_teleop')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ctrl = self.create_publisher(String, '/puma/control', 10)
        self.pub_gait = self.create_publisher(String, '/puma/gait', 10)
        self.create_timer(0.1, lambda: None)  # Keep-alive for DDS discovery

    def send_velocity(self, x, y, yaw):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(yaw)
        self.pub_vel.publish(msg)

    def send_control(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub_ctrl.publish(msg)

    def send_gait(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub_gait.publish(msg)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    
    print(HELP_TEXT)
    print("Waiting for DDS discovery...")
    time.sleep(2)
    print("Ready!\n")
    
    speed_mult = 1.0
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key in MOVE_BINDINGS:
                x, y, yaw = MOVE_BINDINGS[key]
                node.send_velocity(x * speed_mult, y * speed_mult, yaw * speed_mult)
                print(f"\rVel: x={x*speed_mult:+.2f} y={y*speed_mult:+.2f} yaw={yaw*speed_mult:+.2f}  ", end="")
            
            elif key in CONTROL_BINDINGS:
                cmd = CONTROL_BINDINGS[key]
                node.send_control(cmd)
                print(f"\rCommand: {cmd:<12}", end="")
            
            elif key in GAIT_BINDINGS:
                cmd = GAIT_BINDINGS[key]
                node.send_gait(cmd)
                print(f"\rGait: {cmd:<12}", end="")
            
            elif key == '+' or key == '=':
                speed_mult = min(speed_mult + 0.1, 2.0)
                print(f"\rSpeed: {speed_mult:.1f}x        ", end="")
            
            elif key == '-':
                speed_mult = max(speed_mult - 0.1, 0.1)
                print(f"\rSpeed: {speed_mult:.1f}x        ", end="")
            
            elif key == ' ':
                node.send_velocity(0, 0, 0)
                print("\rSTOPPED              ", end="")
            
            elif key == '\x03':
                break
    
    except Exception as e:
        print(f"\nError: {e}")
    
    finally:
        node.send_velocity(0, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        executor.shutdown()
        rclpy.shutdown()
        print("\n")


if __name__ == '__main__':
    main()
