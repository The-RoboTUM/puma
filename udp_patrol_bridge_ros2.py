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


class PatrolUdpBridge(Node):
    def __init__(self):
        super().__init__("patrol_udp_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 32000)

        self.declare_parameter("robot_ip", "10.21.31.103")
        self.declare_parameter("robot_port", 30000)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("poll_rate_hz", 200.0)

        self.declare_parameter("send_heartbeat", True)
        self.declare_parameter("heartbeat_rate_hz", 2.0)
        self.declare_parameter("heartbeat_type", 100)
        self.declare_parameter("heartbeat_cmd", 100)

        # ---- Drift control (deadband + stationary freeze) ----
        self.declare_parameter("deadband_v", 0.03)          # m/s
        self.declare_parameter("deadband_yaw_rate", 0.05)   # rad/s
        self.declare_parameter("freeze_when_stationary", True)
        self.declare_parameter("stationary_count_required", 5)

        # ---- Cmd gating (方案 A) ----
        self.declare_parameter("use_cmd_gating", True)
        self.declare_parameter("cmd_timeout_sec", 0.5)          # 多久内的 cmd 认为有效
        self.declare_parameter("cmd_linear_eps", 0.03)          # |linear| < eps 认为“没平移”
        self.declare_parameter("cmd_angular_min", 0.10)         # |angular| > min 认为“在转向”
        self.declare_parameter("gate_translation_when_turning", True)

        # ---- NEW: latch after turning ----
        # 一旦发生“纯转向”，就锁存：后续没有平移指令前，都不积分 x/y
        self.declare_parameter("hold_translation_after_turn", True)
        self.declare_parameter("release_hold_on_linear_cmd", True)

        # Debug
        self.declare_parameter("debug_log", False)
        self.declare_parameter("debug_log_every_n", 50)

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

        self.deadband_v = float(self.get_parameter("deadband_v").value)
        self.deadband_yaw_rate = float(self.get_parameter("deadband_yaw_rate").value)
        self.freeze_when_stationary = bool(self.get_parameter("freeze_when_stationary").value)
        self.stationary_count_required = int(self.get_parameter("stationary_count_required").value)

        self.use_cmd_gating = bool(self.get_parameter("use_cmd_gating").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.cmd_linear_eps = float(self.get_parameter("cmd_linear_eps").value)
        self.cmd_angular_min = float(self.get_parameter("cmd_angular_min").value)
        self.gate_translation_when_turning = bool(self.get_parameter("gate_translation_when_turning").value)

        self.hold_translation_after_turn = bool(self.get_parameter("hold_translation_after_turn").value)
        self.release_hold_on_linear_cmd = bool(self.get_parameter("release_hold_on_linear_cmd").value)

        self.debug_log = bool(self.get_parameter("debug_log").value)
        self.debug_log_every_n = int(self.get_parameter("debug_log_every_n").value)

        # ---------------- UDP socket ----------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

        # ---------------- QoS ----------------
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_imu = self.create_publisher(Imu, "/imu", sensor_qos)
        self.pub_odom = self.create_publisher(Odometry, "/odom", sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Subscribers ----------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(String, "/puma/control", self.control_callback, 10)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.last_t: Optional[float] = None
        self.last_yaw: float = 0.0

        # 连续静止计数
        self.stationary_count = 0

        # 最近 cmd_vel（用于门控）
        self.last_cmd_time_sec: float = -1.0
        self.last_cmd_linear_x: float = 0.0
        self.last_cmd_linear_y: float = 0.0
        self.last_cmd_angular_z: float = 0.0

        # NEW: latch flag
        self.hold_translation: bool = False
        self.hold_since_sec: float = -1.0
        self.hold_reason: str = ""

        # Debug counters
        self.rx_total = 0
        self.rx_json = 0
        self.rx_motion = 0
        self.pub_count = 0
        self.last_sender = None

        self._decoder = json.JSONDecoder()

        # outgoing message id (always available)
        self.tx_msg_id = 0

        # ---------------- Timers ----------------
        self.poll_timer = self.create_timer(1.0 / max(self.poll_rate_hz, 1.0), self.poll_udp)

        if self.send_heartbeat and self.heartbeat_rate_hz > 0.0:
            self.hb_timer = self.create_timer(1.0 / self.heartbeat_rate_hz, self.send_heartbeat_once)
        else:
            self.hb_timer = None

        self.get_logger().info(
            f"Listening UDP on {self.listen_ip}:{self.listen_port} ; "
            f"robot={self.robot_ip}:{self.robot_port} ; "
            f"publishing /imu /odom /tf ({self.odom_frame}->{self.base_frame}) ; "
            f"subscribing /cmd_vel /puma/control ; "
            f"heartbeat={'ON' if self.hb_timer else 'OFF'} ; "
            f"deadband(v={self.deadband_v}, yaw_rate={self.deadband_yaw_rate}) ; "
            f"freeze_when_stationary={self.freeze_when_stationary} N={self.stationary_count_required} ; "
            f"use_cmd_gating={self.use_cmd_gating} gate_translation_when_turning={self.gate_translation_when_turning} ; "
            f"hold_translation_after_turn={self.hold_translation_after_turn}"
        )

    # ---------------- Helpers: cmd intent ----------------
    def _cmd_is_recent(self, now_sec: float) -> bool:
        if not self.use_cmd_gating:
            return False
        if self.last_cmd_time_sec < 0:
            return False
        return (now_sec - self.last_cmd_time_sec) <= self.cmd_timeout_sec

    def _cmd_lin_mag(self) -> float:
        return math.hypot(self.last_cmd_linear_x, self.last_cmd_linear_y)

    def _recent_cmd_is_turning_in_place(self, now_sec: float) -> bool:
        if not self._cmd_is_recent(now_sec):
            return False
        lin_mag = self._cmd_lin_mag()
        ang = abs(self.last_cmd_angular_z)
        return (lin_mag < self.cmd_linear_eps) and (ang > self.cmd_angular_min)

    def _recent_cmd_has_translation(self, now_sec: float) -> bool:
        if not self._cmd_is_recent(now_sec):
            return False
        return self._cmd_lin_mag() >= self.cmd_linear_eps

    # ---------------- /cmd_vel tracking ----------------
    def cmd_vel_callback(self, msg: Twist):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.last_cmd_time_sec = now_sec
        self.last_cmd_linear_x = float(msg.linear.x)
        self.last_cmd_linear_y = float(msg.linear.y)
        self.last_cmd_angular_z = float(msg.angular.z)

        # NEW: latch logic
        lin_mag = self._cmd_lin_mag()
        ang = abs(self.last_cmd_angular_z)

        # 如果是“纯转向”，开启 hold_translation（转向后锁存平移）
        if self.hold_translation_after_turn and (lin_mag < self.cmd_linear_eps) and (ang > self.cmd_angular_min):
            if not self.hold_translation:
                self.hold_translation = True
                self.hold_since_sec = now_sec
                self.hold_reason = "turn_in_place_cmd"
                if self.debug_log:
                    self.get_logger().info("Hold translation ON (reason: turn_in_place_cmd)")

        # 如果有明显平移指令，解除 hold_translation
        if self.release_hold_on_linear_cmd and (lin_mag >= self.cmd_linear_eps):
            if self.hold_translation:
                self.hold_translation = False
                self.hold_reason = ""
                if self.debug_log:
                    self.get_logger().info("Hold translation OFF (reason: linear_cmd_detected)")

        # 同时把 cmd 转发给机器人
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

    # ---------------- Robot send ----------------
    def _send_control_message(self, type_code: int, command_code: int, items: dict):
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
                int.to_bytes(self.tx_msg_id & 0xFFFF, 2, "little", signed=False) +
                b"\x01" +
                b"\x00" * 7
            )
            self.sock.sendto(header + payload, (self.robot_ip, self.robot_port))
            self.tx_msg_id = (self.tx_msg_id + 1) & 0xFFFF
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
            pkt = self._make_heartbeat_packet(self.tx_msg_id)
            self.sock.sendto(pkt, (self.robot_ip, self.robot_port))
            self.tx_msg_id = (self.tx_msg_id + 1) & 0xFFFF
        except Exception as e:
            self.get_logger().warn(f"Heartbeat send error: {e}")

    # ---------------- JSON extraction ----------------
    def _extract_json(self, data: bytes) -> Optional[dict]:
        try:
            s = data.decode("utf-8", errors="ignore")
        except Exception:
            return None
        i = s.find("{")
        if i < 0:
            return None
        try:
            obj, _end = self._decoder.raw_decode(s[i:])
            return obj if isinstance(obj, dict) else None
        except Exception:
            return None

    # ---------------- UDP polling ----------------
    def poll_udp(self):
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

            if t == 1002 and c == 4:
                motion = items.get("MotionStatus", {})
                self.rx_motion += 1
                self.handle_motion(motion)

                if self.debug_log and (self.rx_motion % max(self.debug_log_every_n, 1) == 0):
                    self.get_logger().info(
                        f"rx_total={self.rx_total} rx_json={self.rx_json} rx_motion={self.rx_motion} "
                        f"pub={self.pub_count} last_sender={self.last_sender} hold_translation={self.hold_translation}"
                    )

    # ---------------- Motion handling ----------------
    def handle_motion(self, motion: dict):
        roll = float(motion.get("Roll", 0.0))
        pitch = float(motion.get("Pitch", 0.0))
        yaw = float(motion.get("Yaw", 0.0))
        omega_z = float(motion.get("OmegaZ", 0.0))
        lin_x = float(motion.get("LinearX", 0.0))
        lin_y = float(motion.get("LinearY", 0.0))
        height = float(motion.get("Height", 0.0))

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        # --- deadband on measured values ---
        if abs(lin_x) < self.deadband_v:
            lin_x = 0.0
        if abs(lin_y) < self.deadband_v:
            lin_y = 0.0
        if abs(omega_z) < self.deadband_yaw_rate:
            omega_z = 0.0

        turning_in_place_cmd = self._recent_cmd_is_turning_in_place(now_sec)

        # gate_translation_when_turning：只在“近期纯转向 cmd”时生效（短时）
        gate_now = self.gate_translation_when_turning and turning_in_place_cmd

        # NEW: latch — 只要 hold_translation=True，就一直 gate
        if self.hold_translation_after_turn and self.hold_translation:
            gate_now = True

        # 方案 A 实际作用：用于积分的线速度
        if gate_now:
            lin_x_for_int = 0.0
            lin_y_for_int = 0.0
        else:
            lin_x_for_int = lin_x
            lin_y_for_int = lin_y

        if self.last_t is None:
            self.last_t = now_sec
            self.last_yaw = yaw
            self.z = height
            return

        dt = now_sec - self.last_t
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0

        # stationary 判定（基于用于积分的值）
        stationary = (lin_x_for_int == 0.0 and lin_y_for_int == 0.0 and omega_z == 0.0)
        if stationary:
            self.stationary_count += 1
        else:
            self.stationary_count = 0

        freeze = self.freeze_when_stationary and (self.stationary_count >= max(1, self.stationary_count_required))

        # 积分位置（如果 freeze 或 gate_now，vx/vy 已经是 0，积分不会漂）
        if not freeze:
            self.x += lin_x_for_int * dt
            self.y += lin_y_for_int * dt

        self.z = height

        if self.debug_log and (self.rx_motion % max(self.debug_log_every_n, 1) == 0):
            cmd_recent = self._cmd_is_recent(now_sec)
            self.get_logger().info(
                f"motion: raw_lin=({motion.get('LinearX')},{motion.get('LinearY')}) "
                f"after_db=({lin_x:.4f},{lin_y:.4f}) for_int=({lin_x_for_int:.4f},{lin_y_for_int:.4f}) "
                f"omega_z={omega_z:.4f} yaw={yaw:.3f} dt={dt:.4f} "
                f"cmd_recent={cmd_recent} turn_cmd={turning_in_place_cmd} hold={self.hold_translation} "
                f"stationary_cnt={self.stationary_count} freeze={freeze} "
                f"pose=({self.x:.3f},{self.y:.3f})"
            )

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

        # 给消费者的 twist：如果 gate_now，就发布 0 线速度
        odom.twist.twist.linear.x = 0.0 if gate_now else lin_x
        odom.twist.twist.linear.y = 0.0 if gate_now else lin_y
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
        self.last_yaw = yaw


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
