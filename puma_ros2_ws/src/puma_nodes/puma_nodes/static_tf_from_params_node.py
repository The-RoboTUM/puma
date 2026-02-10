#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quat_from_rpy(roll: float, pitch: float, yaw: float):
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


class StaticTfFromParams(Node):
    def __init__(self):
        super().__init__("lidar_static_tf_pub")
        self.broadcaster = StaticTransformBroadcaster(self)

        # 可扩展：transform_names = ["front", "back", ...]
        self.declare_parameter("transform_names", ["front", "back"])
        names = self.get_parameter("transform_names").get_parameter_value().string_array_value

        transforms = []
        for name in names:
            parent_key = f"transforms.{name}.parent"
            child_key = f"transforms.{name}.child"
            xyz_key = f"transforms.{name}.xyz"
            rpy_key = f"transforms.{name}.rpy"

            # self.declare_parameter(parent_key, "base_link")
            self.declare_parameter(parent_key, "puma_base_link")
            self.declare_parameter(child_key, f"{name}_link")
            self.declare_parameter(xyz_key, [0.0, 0.0, 0.0])
            self.declare_parameter(rpy_key, [0.0, 0.0, 0.0])

            parent = self.get_parameter(parent_key).value
            child = self.get_parameter(child_key).value
            xyz = self.get_parameter(xyz_key).value
            rpy = self.get_parameter(rpy_key).value

            if not (isinstance(xyz, (list, tuple)) and len(xyz) == 3):
                raise ValueError(f"{xyz_key} must be a list of 3 numbers")
            if not (isinstance(rpy, (list, tuple)) and len(rpy) == 3):
                raise ValueError(f"{rpy_key} must be a list of 3 numbers")

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child

            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])

            qx, qy, qz, qw = quat_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            transforms.append(t)

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published {len(transforms)} static transforms on /tf_static")
        for t in transforms:
            self.get_logger().info(
                f"  {t.header.frame_id} -> {t.child_frame_id}  xyz=({t.transform.translation.x:.5f},"
                f"{t.transform.translation.y:.5f},{t.transform.translation.z:.5f})"
            )


def main():
    rclpy.init()
    node = StaticTfFromParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
