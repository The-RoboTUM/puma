#!/usr/bin/env python3
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

msg = """
PUMA Local Teleop (ROS2 -> UDP Bridge)
---------------------------
Moving:
   q    w    e
   a    s    d

w/s : linear X (forward/back)
a/d : linear Y (left/right)
q/e : angular Z (turn)

Functions:
u : Stand Up (Motion State 1)
z : Sit Down (Motion State 4)
m : Damping (Motion State 3)
r : Standard Mode (Motion State 6)
p : RL Control (Motion State 17)

Space : FORCE STOP
CTRL-C to quit
"""

moveBindings = {
    'w': (0.2, 0, 0),
    's': (-0.2, 0, 0),
    'a': (0, 0.2, 0),
    'd': (0, -0.2, 0),
    'q': (0, 0, 0.5),
    'e': (0, 0, -0.5),
}

controlBindings = {
    'u': 'stand',
    'z': 'sit',
    'm': 'damping',
    'r': 'standard',
    'p': 'rl',
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class LocalTeleop(Node):
    def __init__(self):
        super().__init__('puma_local_teleop')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ctrl = self.create_publisher(String, '/puma/control', 10)
        # Create a timer to keep the node "alive" for discovery
        self.create_timer(0.1, lambda: None)
        self.get_logger().info('Teleop node started, publishing to /cmd_vel and /puma/control')

    def send_vel(self, x, y, th):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(th)
        self.pub_vel.publish(twist)

    def send_control(self, cmd):
        control_msg = String()
        control_msg.data = cmd
        self.pub_ctrl.publish(control_msg)
        print(f"\rSent Command: {cmd}            ", end="")

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = LocalTeleop()
    
    # Create executor and spin in background thread
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    print(msg)
    print("[INFO] Waiting for ROS2 discovery (2 seconds)...")
    import time
    time.sleep(2)  # Give DDS time to discover other nodes
    print("[INFO] Ready! Press keys to control.")
    
    x = 0.0
    y = 0.0
    th = 0.0
    
    try:
        while rclpy.ok():
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                node.send_vel(x, y, th)
                print(f"\rVel: x={x} y={y} th={th}", end="")
                
            elif key in controlBindings.keys():
                node.send_control(controlBindings[key])
                
            elif key == ' ':
                x = 0.0
                y = 0.0
                th = 0.0
                node.send_vel(0, 0, 0)
                print("\rSTOPPED                   ", end="")
                
            elif key == '\x03':
                break

    except Exception as e:
        print(e)

    finally:
        node.send_vel(0, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
