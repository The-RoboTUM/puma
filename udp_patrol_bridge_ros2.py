#!/usr/bin/env python3
import socket
import json
import math
import time
import select
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


SYNC = b"\xeb\x91\xeb\x90"  # 4 bytes


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (rad) to quaternion (x,y,z,w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def apply_deadband(x: float, deadband: float) -> float:
    """Zero-out small values within deadband."""
    return 0.0 if abs(x) < deadband else x


class PatrolUdpBridge(Node):
    def __init__(self):
        super().__init__("patrol_udp_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 32000)

        # Robot UDP endpoint (where we send heartbeat)
        self.declare_parameter("robot_ip", "10.21.31.103")
        self.declare_parameter("robot_port", 30000)

        # Frames
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # Loop rate / Heartbeat
        self.declare_parameter("poll_rate_hz", 200.0)      # how often we poll UDP socket (non-blocking)
        self.declare_parameter("send_heartbeat", True)
        self.declare_parameter("heartbeat_rate_hz", 2.0)   # 2 Hz heartbeats
        self.declare_parameter("heartbeat_type", 100)      # Type=100
        self.declare_parameter("heartbeat_cmd", 100)       # Command=100

        # --- Drift fix (方案 A) ---
        self.declare_parameter("vel_deadband", 0.01)          # m/s
        self.declare_parameter("yaw_rate_deadband", 0.02)     # rad/s
        self.declare_parameter("freeze_when_stationary", True)

        # Debug
        self.declare_parameter("debug_log", False)
        self.declare_parameter("debug_log_every_n", 200)   # print every N motion packets

        # Read params
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)

        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_port = int(self.get_parameter("robot_port").value)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)
        self.send_heartbeat = bool(self.get_parameter("send_heartbeat").value)
        self.heartbeat_rate_hz = float(self.get_parameter("heartbeat_rate_hz").value)
        self.heartbeat_type = int(self.get_parameter("heartbeat_type").value)
        self.heartbeat_cmd = int(self.get_parameter("heartbeat_cmd").value)

        self.vel_deadband = float(self.get_parameter("vel_deadband").value)
        self.yaw_rate_deadband = float(self.get_parameter("yaw_rate_deadband").value)
        self.freeze_when_stationary = bool(self.get_parameter("freeze_when_stationary").value)

        self.debug_log = bool(self.get_parameter("debug_log").value)
        self.debug_log_every_n = int(self.get_parameter("debug_log_every_n").value)

        # ---------------- UDP socket ----------------
        # 用「同一個 socket」同時 listen + send heartbeat，確保回包回到同一個本地端口
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 方便重啟（注意：不是讓多進程同時分到相同流量）
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass

        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

        # ---------------- QoS (sensor friendly) ----------------
        # /imu /odom /tf 通常用 BEST_EFFORT 更穩（特別是視覺化/工具端）
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_imu = self.create_publisher(Imu, "/imu", sensor_qos)
        self.pub_odom = self.create_publisher(Odometry, "/odom", sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Subscribers for teleop ----------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(String, "/puma/control", self.control_callback, 10)

        # ---------------- State for odom integration ----------------
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.last_t: Optional[float] = None

        # Debug counters
        self.rx_total = 0
        self.rx_json = 0
        self.rx_motion = 0
        self.pub_count = 0
        self.last_sender = None

        # JSON decoder for robust partial parsing
        self._decoder = json.JSONDecoder()

        # ---------------- Timers ----------------
        self.poll_timer = self.create_timer(1.0 / max(self.poll_rate_hz, 1.0), self.poll_udp)

        if self.send_heartbeat and self.heartbeat_rate_hz > 0.0:
            self.hb_msg_id = 0
            self.hb_timer = self.create_timer(1.0 / self.heartbeat_rate_hz, self.send_heartbeat_once)
        else:
            self.hb_timer = None

        self.get_logger().info(
            f"Listening UDP on {self.listen_ip}:{self.listen_port} ; "
            f"robot={self.robot_ip}:{self.robot_port} ; "
            f"publishing /imu /odom /tf ({self.odom_frame}->{self.base_frame}) ; "
            f"subscribing /cmd_vel /puma/control ; "
            f"heartbeat={'ON' if self.hb_timer else 'OFF'} ; "
            f"deadband(v={self.vel_deadband}, yaw_rate={self.yaw_rate_deadband}) ; "
            f"freeze_when_stationary={self.freeze_when_stationary}"
        )

    # ---------------- ROS2 -> Robot (Teleop Commands) ----------------
    def cmd_vel_callback(self, msg: Twist):
        """Handle /cmd_vel messages and send Type 2 Command 21 to robot."""
        items = {
            "X": float(msg.linear.x),
            "Y": float(msg.linear.y),
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": float(msg.angular.z)
        }
        self._send_control_message(2, 21, items)

    def control_callback(self, msg: String):
        """Handle /puma/control messages and send Type 2 Command 22 to robot."""
        cmd = msg.data.lower()
        state_map = {
            "stand": 1,
            "sit": 4,
            "damping": 3,
            "standard": 6,
            "rl": 17
        }

        state_id = state_map.get(cmd)
        if state_id is not None:
            self._send_control_message(2, 22, {"MotionParam": state_id})
            self.get_logger().info(f"Sent motion state: {cmd} ({state_id})")
        else:
            self.get_logger().warn(f"Unknown control command: {cmd}")

    def _send_control_message(self, type_code: int, command_code: int, items: dict):
        """Send control message to robot using same protocol as heartbeat."""
        try:
            payload = json.dumps({
                "PatrolDevice": {
                    "Type": type_code,
                    "Command": command_code,
                    "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "Items": items
                }
            }).encode("utf-8")

            header = (
                SYNC +
                int.to_bytes(len(payload), 2, "little", signed=False) +
                int.to_bytes(self.hb_msg_id & 0xFFFF, 2, "little", signed=False) +
                b"\x01" +
                b"\x00" * 7
            )

            packet = header + payload
            self.sock.sendto(packet, (self.robot_ip, self.robot_port))
            self.hb_msg_id = (self.hb_msg_id + 1) & 0xFFFF

        except Exception as e:
            self.get_logger().error(f"Failed to send control message: {e}")

    # ---------------- Heartbeat ----------------
    def _make_heartbeat_packet(self, msg_id: int) -> bytes:
        payload = json.dumps({
            "PatrolDevice": {
                "Type": self.heartbeat_type,
                "Command": self.heartbeat_cmd,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "Items": {}
            }
        }).encode("utf-8")

        header = (
            SYNC +
            int.to_bytes(len(payload), 2, "little", signed=False) +
            int.to_bytes(msg_id & 0xFFFF, 2, "little", signed=False) +
            b"\x01" +
            b"\x00" * 7
        )
        return header + payload

    def send_heartbeat_once(self):
        try:
            pkt = self._make_heartbeat_packet(self.hb_msg_id)
            self.sock.sendto(pkt, (self.robot_ip, self.robot_port))
            self.hb_msg_id = (self.hb_msg_id + 1) & 0xFFFF
        except Exception as e:
            self.get_logger().warn(f"Heartbeat send error: {e}")

    # ---------------- JSON extraction ----------------
    def _extract_json(self, data: bytes) -> Optional[dict]:
        """
        Robustly decode: find first '{' then parse exactly one JSON object using raw_decode.
        This survives: header(16B)+JSON + trailing bytes, or direct JSON.
        """
        try:
            s = data.decode("utf-8", errors="ignore")
        except Exception:
            return None

        i = s.find("{")
        if i < 0:
            return None

        try:
            obj, _end = self._decoder.raw_decode(s[i:])
            if isinstance(obj, dict):
                return obj
            return None
        except Exception:
            return None

    # ---------------- UDP polling ----------------
    def poll_udp(self):
        # Non-blocking: drain all available datagrams
        while True:
            r, _, _ = select.select([self.sock], [], [], 0.0)
            if not r:
                break

            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"UDP recv error: {e}")
                break

            self.rx_total += 1
            self.last_sender = addr

            msg = self._extract_json(data)
            if not msg:
                continue
            self.rx_json += 1

            pd = msg.get("PatrolDevice", {})
            t = pd.get("Type", None)
            c = pd.get("Command", None)
            items = pd.get("Items", {})

            # Command=4 has MotionStatus
            if t == 1002 and c == 4:
                motion = items.get("MotionStatus", {})
                self.rx_motion += 1
                self.handle_motion(motion)

                if self.debug_log and (self.rx_motion % max(self.debug_log_every_n, 1) == 0):
                    self.get_logger().info(
                        f"rx_total={self.rx_total} rx_json={self.rx_json} rx_motion={self.rx_motion} "
                        f"pub={self.pub_count} last_sender={self.last_sender}"
                    )

    # ---------------- Motion handling / Publishing ----------------
    def handle_motion(self, motion: dict):
        roll = float(motion.get("Roll", 0.0))
        pitch = float(motion.get("Pitch", 0.0))
        yaw = float(motion.get("Yaw", 0.0))
        omega_z = float(motion.get("OmegaZ", 0.0))
        vx_body = float(motion.get("LinearX", 0.0))
        vy_body = float(motion.get("LinearY", 0.0))
        height = float(motion.get("Height", 0.0))

        # --- 方案 A：deadband，杀掉静止偏置 ---
        vx_body = apply_deadband(vx_body, self.vel_deadband)
        vy_body = apply_deadband(vy_body, self.vel_deadband)
        omega_z = apply_deadband(omega_z, self.yaw_rate_deadband)

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.last_t is None:
            self.last_t = now_sec
            self.z = height
            # 初次不积分，但仍可发布一次（这里选择不发布，避免 dt=0）
            return

        dt = now_sec - self.last_t
        # 防止积分爆炸
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0

        # body -> world (use yaw)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        vx = cy * vx_body - sy * vy_body
        vy = sy * vx_body + cy * vy_body

        # --- 方案 A：静止时不积分（freeze），防漂 ---
        stationary = (vx_body == 0.0 and vy_body == 0.0 and omega_z == 0.0)
        if self.freeze_when_stationary and stationary:
            # 不更新 x/y（不积分）；但高度还是更新（如果你希望高度也冻结，可以把这行注释掉）
            self.z = height
            vx = 0.0
            vy = 0.0
            omega_z = 0.0
        else:
            self.x += vx * dt
            self.y += vy * dt
            self.z = height

        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)

        # ---- Publish Imu ----
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.base_frame

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.z = omega_z
        self.pub_imu.publish(imu)

        # ---- Publish Odometry ----
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # twist：如果静止被判定，vx/vy 会是 0（更符合“静止”）
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega_z

        self.pub_odom.publish(odom)

        # ---- Broadcast TF ----
        tfm = TransformStamped()
        tfm.header.stamp = now.to_msg()
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = self.base_frame
        tfm.transform.translation.x = self.x
        tfm.transform.translation.y = self.y
        tfm.transform.translation.z = self.z
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tfm)

        self.pub_count += 1
        self.last_t = now_sec


def main():
    rclpy.init()
    node = PatrolUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.sock.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
