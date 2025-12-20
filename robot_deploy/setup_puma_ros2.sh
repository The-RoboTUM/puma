#!/bin/bash
# Setup ROS2 environment for PUMA with UDP-only FastDDS (fixes inter-process discovery)

source /opt/robot/scripts/setup_ros2.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp_only.xml"

echo "PUMA ROS2 ready (FastDDS: $FASTRTPS_DEFAULT_PROFILES_FILE)"
