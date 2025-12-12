#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Robot-specific imports
from drdds.msg import NavCmd, NavCmdValue, MetaType, Timestamp, MotionState, MotionStateValue

class PumaBridge(Node):
    def __init__(self):
        super().__init__('puma_bridge')

        # Subscribers (Input from Laptop)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/puma/control', self.control_callback, 10)
        
        # Publishers (Output to Robot Firmware)
        self.pub_nav = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.pub_motion = self.create_publisher(MotionState, '/MOTION_STATE', 10)
        
        self.get_logger().info("PUMA Bridge Started.")

    def create_header(self):
        """Creates the required DeepRobotics message header with timestamp."""
        header = MetaType()
        header.frame_id = 0
        
        ts = Timestamp()
        now = self.get_clock().now()
        ts.sec = int(now.seconds_nanoseconds()[0])
        ts.nsec = int(now.seconds_nanoseconds()[1])
        
        header.timestamp = ts
        return header

    def cmd_vel_callback(self, msg):
        """Translates geometry_msgs/Twist -> drdds/NavCmd"""
        nav_msg = NavCmd()
        nav_msg.header = self.create_header()
        
        val = NavCmdValue()
        val.x_vel = float(msg.linear.x)
        val.y_vel = float(msg.linear.y)
        val.yaw_vel = float(msg.angular.z)
        
        nav_msg.data = val
        self.pub_nav.publish(nav_msg)

    def control_callback(self, msg):
        """Translates std_msgs/String -> drdds/MotionState"""
        command = msg.data.lower()
        state_map = {
            "stand": 1,
            "sit": 4,
            "damping": 3,
            "standard": 6
        }
        
        state_id = state_map.get(command)
        
        if state_id is not None:
            motion_msg = MotionState()
            motion_msg.header = self.create_header()
            
            val = MotionStateValue()
            val.state = state_id
            
            motion_msg.data = val
            self.pub_motion.publish(motion_msg)
            self.get_logger().info(f"Bridge: Switching State -> {command} ({state_id})")

def main(args=None):
    rclpy.init(args=args)
    node = PumaBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
