from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    front = Node(
        package="puma_nodes",
        executable="rtsp_to_ros2_image",
        name="rtsp_front",
        output="screen",
        parameters=[{
            "rtsp_url": "rtsp://10.21.31.103:8554/video1",
            "topic": "/camera/front/image_raw",
            "frame_id": "camera_front_optical",
            'qos_reliability': 'best_effort',
            'qos_durability': 'volatile',
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
            'qos_reliability': 'best_effort',
            'qos_durability': 'volatile',
            "fps": 15.0,
        }],
    )

    return LaunchDescription([front, rear])
