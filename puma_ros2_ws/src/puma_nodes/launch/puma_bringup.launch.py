from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("puma_nodes")
    lidar_tf_yaml = os.path.join(pkg_share, "config", "lidar_tf.yaml")

    # 1) two cameras (front + rear)
    front = Node(
        package="puma_nodes",
        executable="rtsp_to_ros2_image",
        name="rtsp_front",
        output="screen",
        parameters=[{
            "rtsp_url": "rtsp://10.21.31.103:8554/video1",
            "topic": "/camera/front/image_raw",
            "frame_id": "camera_front_optical",
            "fps": 15.0,
        }],
    )

    rear = Node(
        package="puma_nodes",
        executable="rtsp_to_ros2_image",
        name="rtsp_rear",
        output="screen",
        parameters=[{
            "rtsp_url": "rtsp://10.21.31.103:8554/video2",
            "topic": "/camera/rear/image_raw",
            "frame_id": "camera_rear_optical",
            "fps": 15.0,
        }],
    )

    # 2) UDP bridge
    udp_bridge = Node(
        package="puma_nodes",
        executable="udp_patrol_bridge",
        name="udp_patrol_bridge",
        output="screen",
        # 如果你想把 ip/port 配置化，也可以在这里 parameters=[...]
    )

    # 3) Static TF from params file
    static_tf = Node(
        package="puma_nodes",
        executable="static_tf_from_params",
        name="lidar_static_tf_pub",
        output="screen",
        parameters=[lidar_tf_yaml],
    )

    return LaunchDescription([
        front,
        rear,
        udp_bridge,
        static_tf,
    ])
