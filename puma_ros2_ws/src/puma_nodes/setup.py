from setuptools import setup
import os
from glob import glob

package_name = "puma_nodes"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ament resource index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package manifest
        ("share/" + package_name, ["package.xml"]),
        # install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # install config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pedro",
    maintainer_email="todo@example.com",
    description="PUMA ROS2 nodes (RTSP image, keyboard teleop, UDP patrol bridge).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rtsp_to_ros2_image = puma_nodes.rtsp_to_ros2_image_node:main",
            "teleop_keyboard = puma_nodes.teleop_keyboard_node:main",
            "udp_patrol_bridge = puma_nodes.udp_patrol_bridge_node:main",
            'static_tf_from_params = puma_nodes.static_tf_from_params_node:main',

        ],
    },
)
