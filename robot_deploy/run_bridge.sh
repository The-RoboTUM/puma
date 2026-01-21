#!/bin/bash
# Run the Vendor Bridge (ROS2 → UDP)
# This allows any ROS2 node to control the vendor controller

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source robot ROS2 environment (includes its FastDDS config)
source /opt/robot/scripts/setup_ros2.sh

# Use same domain as robot
export ROS_DOMAIN_ID=0

# Override with UDP-only config (same interfaces, no SHM)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp_only.xml"

echo "════════════════════════════════════════════════════════"
echo "  PUMA Vendor Bridge (ROS2 → UDP)"
echo "════════════════════════════════════════════════════════"
echo ""
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo ""
echo "Subscribed topics:"
echo "  /cmd_vel           → Velocity commands (Twist)"
echo "  /puma/control      → Stand/Sit/Standard (String)"  
echo "  /puma/gait         → Gait selection (String)"
echo ""
echo "════════════════════════════════════════════════════════"
echo ""

python3 "$SCRIPT_DIR/vendor_bridge.py"
