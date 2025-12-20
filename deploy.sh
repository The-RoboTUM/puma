#!/bin/bash
# Deploy robot files to the PUMA robot (aos)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_HOST="m20-aos"
REMOTE_DIR="~/puma_ros2"

echo "Deploying robot_deploy/ to $ROBOT_HOST:$REMOTE_DIR ..."

# Create remote directory and copy all files
ssh $ROBOT_HOST "mkdir -p $REMOTE_DIR"
scp -r "$SCRIPT_DIR/robot_deploy/"* $ROBOT_HOST:$REMOTE_DIR/

echo ""
echo "Done! Files deployed to $ROBOT_HOST:$REMOTE_DIR"
echo ""
echo "To run on robot:"
echo "  cd ~/puma_ros2"
echo "  source setup_puma_ros2.sh"
echo "  python3 udp_ros2_bridge.py"
