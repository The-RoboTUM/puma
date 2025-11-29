from setuptools import find_packages, setup

package_name = 'puma_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotum',
    maintainer_email='j.andregon15@gmail.com',
    description='ROS2 driver for PUMA robot using TCP protocol',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ros2 run puma_driver puma_driver
            'puma_driver = puma_driver.driver_node:main',
        ],
    },
)
