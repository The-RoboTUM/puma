#!/usr/bin/env python3
"""
Simple keyboard teleop for PUMA robot via ROS2 topics.
Publishes to /cmd_vel and /puma/control which are forwarded by udp_patrol_bridge_ros2.py
"""
import sys
import termios
import tty
import select
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String

USAGE = """
═══════════════════════════════════════
   PUMA Robot Keyboard Teleop
═══════════════════════════════════════

Movement Controls:
   q    w    e       u : Stand Up
   a    s    d       z : Sit Down  
                     m : Damping
w/s : Forward/Back    r : Standard Mode
a/d : Left/Right      p : RL Control
q/e : Turn Left/Right

SPACE : Emergency Stop
CTRL-C : Quit
═══════════════════════════════════════
"""

# Movement speed settings
LINEAR_SPEED = 0.3   # m/s
ANGULAR_SPEED = 0.5  # rad/s

moveBindings = {
    'w': (LINEAR_SPEED, 0, 0),
    's': (-LINEAR_SPEED, 0, 0),
    'a': (0, LINEAR_SPEED, 0),
    'd': (0, -LINEAR_SPEED, 0),
    'q': (0, 0, ANGULAR_SPEED),
    'e': (0, 0, -ANGULAR_SPEED),
}

controlBindings = {
    'u': 'stand',
    'z': 'sit',
    'm': 'damping',
    'r': 'standard',
    'p': 'rl',
}


def getKey(settings):
    """Get a single keypress from terminal."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('puma_teleop_keyboard')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ctrl = self.create_publisher(String, '/puma/control', 10)
        self.get_logger().info('Teleop node ready - publishing to /cmd_vel and /puma/control')

    def send_velocity(self, x: float, y: float, theta: float):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(theta)
        self.pub_vel.publish(twist)

    def send_control(self, command: str):
        """Publish control command."""
        msg = String()
        msg.data = command
        self.pub_ctrl.publish(msg)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    
    # Spin ROS2 in background thread
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    print(USAGE)
    print("Waiting for ROS2 discovery...")
    import time
    time.sleep(1.5)
    print("✓ Ready! Press keys to control the robot.\n")
    
    x = 0.0
    y = 0.0
    th = 0.0
    
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings:
                x, y, th = moveBindings[key]
                node.send_velocity(x, y, th)
                print(f"\r→ Velocity: x={x:+.2f} y={y:+.2f} θ={th:+.2f}   ", end="", flush=True)
                
            elif key in controlBindings:
                cmd = controlBindings[key]
                node.send_control(cmd)
                print(f"\r→ Command: {cmd.upper()}                    ", end="", flush=True)
                
            elif key == ' ':
                x = y = th = 0.0
                node.send_velocity(0, 0, 0)
                print("\r→ EMERGENCY STOP                              ", end="", flush=True)
                
            elif key == '\x03':  # CTRL-C
                break

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        node.send_velocity(0, 0, 0)  # Stop robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        executor.shutdown()
        rclpy.shutdown()
        print("\n\nTeleop stopped. Robot halted.")


if __name__ == '__main__':
    main()
