#!/usr/bin/env python3
"""
Keyboard Teleop for PUMA Robot via ROS2

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
PUMA Keyboard Teleop
====================
Movement:        Commands:
  q   w   e      u - Stand
  a   s   d      z - Sit
                 m - Damping
w/s: forward     r - Standard
a/d: strafe      p - RL Mode
q/e: rotate      
                 Space: Stop
                 Ctrl+C: Quit
"""

MOVE_BINDINGS = {
    'w': (0.2, 0, 0),
    's': (-0.2, 0, 0),
    'a': (0, 0.2, 0),
    'd': (0, -0.2, 0),
    'q': (0, 0, 0.5),
    'e': (0, 0, -0.5),
}

CONTROL_BINDINGS = {
    'u': 'stand',
    'z': 'sit',
    'm': 'damping',
    'r': 'standard',
    'p': 'rl',
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
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key in MOVE_BINDINGS:
                x, y, yaw = MOVE_BINDINGS[key]
                node.send_velocity(x, y, yaw)
                print(f"\rVel: x={x:+.1f} y={y:+.1f} yaw={yaw:+.1f}  ", end="")
            
            elif key in CONTROL_BINDINGS:
                cmd = CONTROL_BINDINGS[key]
                node.send_control(cmd)
                print(f"\rCommand: {cmd:<10}", end="")
            
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
