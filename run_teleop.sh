#!/bin/bash
# Wrapper script to run teleop client with correct Python

# Unset conda/ROS variables that might interfere
unset ROS_LOCALHOST_ONLY
unset PYTHONPATH

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Run with system Python (not conda)
exec /usr/bin/python3 "$(dirname "$0")/mission_control/topic_relay_client.py"
