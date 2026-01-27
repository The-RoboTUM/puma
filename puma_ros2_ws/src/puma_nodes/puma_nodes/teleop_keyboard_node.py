#!/usr/bin/env python3
"""
Keyboard teleop for PUMA robot via ROS2 /cmd_vel and /puma/control topics.

- Publishes Twist continuously (fixed rate) because robot may require streaming commands.
- Publishes String commands on a control topic (stand/sit/gaits/estop).
- Parameters:
    cmd_vel_topic (str)
    control_topic (str)
    linear_speed (float)
    angular_speed (float)
    publish_rate (float)
    estop_timeout (float)
    estop_presses (int)
    stop_on_key_release (bool)
"""

import sys
import termios
import tty
import select
import threading
import time
from typing import Optional

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


def get_key(settings, timeout: float = 0.05) -> str:
    """Read single keypress from terminal (non-blocking with timeout)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("puma_teleop_keyboard")

        # ---------------- Parameters ----------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_topic", "/puma/control")

        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("publish_rate", 20.0)

        self.declare_parameter("estop_timeout", 2.0)
        self.declare_parameter("estop_presses", 3)

        self.declare_parameter("stop_on_key_release", True)

        self.cmd_vel_topic: str = str(self.get_parameter("cmd_vel_topic").value)
        self.control_topic: str = str(self.get_parameter("control_topic").value)

        self.linear_speed: float = float(self.get_parameter("linear_speed").value)
        self.angular_speed: float = float(self.get_parameter("angular_speed").value)
        self.publish_rate: float = float(self.get_parameter("publish_rate").value)

        self.estop_timeout: float = float(self.get_parameter("estop_timeout").value)
        self.estop_presses: int = int(self.get_parameter("estop_presses").value)

        self.stop_on_key_release: bool = bool(self.get_parameter("stop_on_key_release").value)

        # ---------------- Publishers ----------------
        self.pub_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_ctrl = self.create_publisher(String, self.control_topic, 10)

        # ---------------- State ----------------
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self._estop_press_count = 0
        self._estop_last_time = 0.0

        # Key bindings depend on parameters -> create after reading params
        self.move_bindings = {
            "w": (self.linear_speed, 0.0, 0.0),
            "s": (-self.linear_speed, 0.0, 0.0),
            "a": (0.0, self.linear_speed, 0.0),
            "d": (0.0, -self.linear_speed, 0.0),
            "q": (0.0, 0.0, self.angular_speed),
            "e": (0.0, 0.0, -self.angular_speed),
        }

        self.control_bindings = {
            "u": "stand",
            "z": "sit",
        }

        self.gait_bindings = {
            "1": "gait_basic",
            "2": "gait_obstacles",
            "3": "gait_flat",
            "4": "gait_stair",
        }

        # Timer for continuous velocity publishing
        if self.publish_rate <= 0.0:
            self.get_logger().warn("publish_rate <= 0, defaulting to 20.0 Hz")
            self.publish_rate = 20.0
        self.vel_timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)

        self.get_logger().info(
            "Teleop ready\n"
            f"  cmd_vel_topic: {self.cmd_vel_topic}\n"
            f"  control_topic: {self.control_topic}\n"
            f"  linear_speed: {self.linear_speed}\n"
            f"  angular_speed: {self.angular_speed}\n"
            f"  publish_rate: {self.publish_rate}\n"
            f"  estop: {self.estop_presses} presses within {self.estop_timeout}s\n"
        )

    # ---------------- Publishers ----------------
    def publish_velocity(self) -> None:
        """Publish velocity at fixed rate (robot requires continuous commands)."""
        twist = Twist()
        twist.linear.x = float(self.vx)
        twist.linear.y = float(self.vy)
        twist.angular.z = float(self.vth)
        self.pub_vel.publish(twist)

    def set_velocity(self, x: float, y: float, theta: float) -> None:
        self.vx = float(x)
        self.vy = float(y)
        self.vth = float(theta)

    def stop(self) -> None:
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

    def send_control(self, command: str) -> None:
        msg = String()
        msg.data = str(command)
        self.pub_ctrl.publish(msg)

    # ---------------- E-Stop logic ----------------
    def handle_estop_key(self) -> str:
        """
        Returns a status string for UI print.
        """
        now = time.time()
        if now - self._estop_last_time > self.estop_timeout:
            self._estop_press_count = 0
        self._estop_press_count += 1
        self._estop_last_time = now

        if self._estop_press_count >= self.estop_presses:
            self.stop()
            self.send_control("estop")
            self._estop_press_count = 0
            return "E-STOP ACTIVATED!"
        remaining = self.estop_presses - self._estop_press_count
        return f"E-Stop: press 'x' {remaining}x more"


def main(args=None) -> None:
    # Terminal settings must be restored even if ROS crashes
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node: Optional[TeleopNode] = None
    executor: Optional[SingleThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None

    try:
        node = TeleopNode()

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        print(USAGE)
        time.sleep(0.2)
        print("\u2713 Ready!\n")

        while rclpy.ok():
            key = get_key(settings)

            if key in node.move_bindings:
                x, y, th = node.move_bindings[key]
                node.set_velocity(x, y, th)
                print(f"\r→ Vel: x={x:+.2f} y={y:+.2f} ω={th:+.2f}   ", end="", flush=True)

            elif key in node.control_bindings:
                cmd = node.control_bindings[key]
                node.send_control(cmd)
                print(f"\r→ {cmd.upper():<30}", end="", flush=True)

            elif key in node.gait_bindings:
                cmd = node.gait_bindings[key]
                node.send_control(cmd)
                # FIX: display based on cmd, not key
                gait_name = cmd.replace("gait_", "").upper()
                print(f"\r→ Gait: {gait_name:<24}", end="", flush=True)

            elif key == "x":
                status = node.handle_estop_key()
                print(f"\r→ {status:<30}", end="", flush=True)

            elif key == " ":
                node.stop()
                print("\r→ Stopped                          ", end="", flush=True)

            elif key == "\x03":  # CTRL-C
                break

            elif key == "":
                # key release / idle
                if node.stop_on_key_release and (node.vx != 0.0 or node.vy != 0.0 or node.vth != 0.0):
                    node.stop()
                    print("\r→ Released                         ", end="", flush=True)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Always restore terminal
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        except Exception:
            pass

        if node is not None:
            try:
                node.stop()
            except Exception:
                pass

        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass

        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        rclpy.shutdown()
        print("\n\nTeleop stopped.")


if __name__ == "__main__":
    main()
