 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math

TOPIC = "/rslidar_points_6691"  # 需要的话改成 6692

class ZRange(Node):
    def __init__(self):
        super().__init__('pc2_zrange')
        self.sub = self.create_subscription(PointCloud2, TOPIC, self.cb, 10)
        self.done = False

    def cb(self, msg: PointCloud2):
        zmin, zmax = math.inf, -math.inf
        n = 0
        for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            z = p[2]
            zmin = min(zmin, z)
            zmax = max(zmax, z)
            n += 1
            if n >= 200000:  # 防止太慢，采样前20万点够用了
                break
        self.get_logger().info(f"{TOPIC}: sampled {n} pts, zmin={zmin:.3f} m, zmax={zmax:.3f} m, zspan={(zmax-zmin):.3f} m")
        self.done = True

def main():
    rclpy.init()
    node = ZRange()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

