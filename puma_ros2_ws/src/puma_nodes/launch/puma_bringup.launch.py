from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("puma_nodes")
    rslidar_params = os.path.join(pkg_share, "config", "rslidar_rosparams.yaml")
    
    # Path to your RViz config file (create this in your package's 'rviz' folder)
    rviz_config_path = os.path.join(pkg_share, "config", "puma_config.rviz")

    # 1. Sensors & Drivers
    front_cam = Node(
        package="puma_nodes",
        executable="rtsp_to_ros2_image",
        name="rtsp_front",
        parameters=[{"rtsp_url": "rtsp://10.21.31.103:8554/video1", "topic": "/camera/front/image_raw", "frame_id": "camera_front_optical"}]
    )

    rear_cam = Node(
        package="puma_nodes",
        executable="rtsp_to_ros2_image",
        name="rtsp_rear",
        parameters=[{"rtsp_url": "rtsp://10.21.31.103:8554/video2", "topic": "/camera/rear/image_raw", "frame_id": "camera_rear_optical"}]
    )

    udp_bridge = Node(package="puma_nodes", executable="udp_patrol_bridge", name="udp_patrol_bridge")
    rslidar_driver = Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='rslidar_driver', parameters=[rslidar_params])

    # 2. Continuous TF Broadcaster
    robot_description_content = f"""
    <robot name="puma">
        <link name="odom" />
        <link name="base_link" />

        <joint name="odom_to_base" type="floating">
            <parent link="odom"/>
            <child link="base_link"/>
        </joint>

        <link name="lidar_link" />
        <joint name="tf_front_lidar" type="fixed">
            <parent link="base_link"/>
            <child link="lidar_link"/>
            <origin xyz="0.32028 0 -0.013" rpy="0 -1.57079 -3.14159"/>
        </joint>

        <link name="lidar_link_2" />
        <joint name="tf_rear_lidar" type="fixed">
            <parent link="base_link"/>
            <child link="lidar_link_2"/>
            <origin xyz="-0.32028 0 -0.013" rpy="0 -1.57079 0"/>
        </joint>

        <link name="camera_front_optical" />
        <joint name="tf_front_cam" type="fixed">
            <parent link="base_link"/>
            <child name="camera_front_optical"/>
            <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
        </joint>

        <link name="camera_rear_optical" />
        <joint name="tf_rear_cam" type="fixed">
            <parent link="base_link"/>
            <child link="camera_rear_optical"/>
            <origin xyz="-0.2 0 0.5" rpy="0 0 3.14159"/>
        </joint>
    </robot>
    """

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. RViz Node
    if os.path.exists(rviz_config_path):
        rviz_args = ['-d', rviz_config_path]
        print("test")
    else:
        # If file is missing, launch empty RViz so you can create/save it
        rviz_args = [] 
        print(f"Config not found at {rviz_config_path}, launching empty RViz.")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    return LaunchDescription([
        front_cam, 
        rear_cam, 
        udp_bridge, 
        rslidar_driver,
        robot_state_publisher,
        rviz_node
    ])