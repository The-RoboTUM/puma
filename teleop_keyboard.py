#!/usr/bin/env python3
"""Keyboard teleop for PUMA robot via ROS2 /cmd_vel and /puma/control topics."""
import sys
import termios
import tty
import select
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String

USAGE = """
═══════════════════════════════════════
   PUMA Robot Keyboard Teleop
═══════════════════════════════════════

Movement:
  w/s : Forward/Back   |  q/e : Turn Left/Right
  a/d : Left/Right     |  SPACE : Stop

Motion:
  u : Stand  |  z : Sit

Gaits:
  1 : Basic  |  2 : Obstacles  |  3 : Flat  |  4 : Stair

Safety:
  x (3x) : Soft E-Stop (robot falls!)

CTRL-C : Quit
═══════════════════════════════════════
"""

LINEAR_SPEED = 1.0
ANGULAR_SPEED = 1.0
PUBLISH_RATE = 20.0
ESTOP_TIMEOUT = 2.0

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
}

gaitBindings = {
    '1': 'gait_basic',
    '2': 'gait_obstacles',
    '3': 'gait_flat',
    '4': 'gait_stair',
}

estop_press_count = 0
estop_last_time = 0.0


def getKey(settings, timeout=0.05):
    """Read single keypress from terminal."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('puma_teleop_keyboard')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ctrl = self.create_publisher(String, '/puma/control', 10)
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        self.vel_timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_velocity)
        self.get_logger().info('Teleop ready')

    def publish_velocity(self):
        """Publish velocity at fixed rate (robot requires continuous commands)."""
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.vth
        self.pub_vel.publish(twist)

    def set_velocity(self, x: float, y: float, theta: float):
        self.vx = float(x)
        self.vy = float(y)
        self.vth = float(theta)

    def stop(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

    def send_control(self, command: str):
        msg = String()
        msg.data = command
        self.pub_ctrl.publish(msg)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    print(USAGE)
    time.sleep(1)
    print("\u2713 Ready!\n")
    
    global estop_press_count, estop_last_time
    
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings:
                x, y, th = moveBindings[key]
                node.set_velocity(x, y, th)
                print(f"\r→ Vel: x={x:+.1f} y={y:+.1f} ω={th:+.1f}   ", end="", flush=True)
                
            elif key in controlBindings:
                node.send_control(controlBindings[key])
                print(f"\r→ {controlBindings[key].upper()}                         ", end="", flush=True)
                
            elif key in gaitBindings:
                node.send_control(gaitBindings[key])
                gait = key.replace('gait_', '').upper()
                print(f"\r→ Gait: {gait}                           ", end="", flush=True)
            
            elif key == 'x':
                now = time.time()
                if now - estop_last_time > ESTOP_TIMEOUT:
                    estop_press_count = 0
                estop_press_count += 1
                estop_last_time = now
                
                if estop_press_count >= 3:
                    node.stop()
                    node.send_control('estop')
                    print("\r→ E-STOP ACTIVATED!                        ", end="", flush=True)
                    estop_press_count = 0
                else:
                    remaining = 3 - estop_press_count
                    print(f"\r→ E-Stop: press 'x' {remaining}x more           ", end="", flush=True)
                
            elif key == ' ':
                node.stop()
                print("\r→ Stopped                                 ", end="", flush=True)
                
            elif key == '\x03':
                break
            
            elif key == '':
                if node.vx != 0.0 or node.vy != 0.0 or node.vth != 0.0:
                    node.stop()
                    print("\r→ Released                               ", end="", flush=True)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        executor.shutdown()
        rclpy.shutdown()
        print("\n\nTeleop stopped.")


if __name__ == '__main__':
    main()
