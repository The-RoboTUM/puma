#!/bin/bash
# Run the ROS2 Teleop (requires vendor_bridge.py running)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source robot ROS2 environment
source /opt/robot/scripts/setup_ros2.sh

# Use default ROS2 settings (same as other robot nodes)
export ROS_DOMAIN_ID=0

# Override with UDP-only config (same interfaces, no SHM)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp_only.xml"

python3 "$SCRIPT_DIR/teleop_ros2.py"
