#!/bin/bash
# Run local teleop on PC (jenny) to control robot via SSH tunnel

echo "========================================="
echo "PUMA Remote Teleop via SSH Tunnel"
echo "========================================="
echo ""

# Check if bridge is running on robot
echo "Checking if bridge is running on robot..."
if ! ssh m20-aos "pgrep -f udp_ros2_bridge.py" > /dev/null 2>&1; then
    echo "⚠️  Bridge not detected on robot!"
    echo ""
    echo "Please start the bridge on the robot first:"
    echo "  ssh m20-aos"
    echo "  cd ~/puma_ros2"
    echo "  source setup_puma_ros2.sh"
    echo "  python3 udp_ros2_bridge.py &"
    echo ""
    read -p "Press Enter once bridge is running, or Ctrl+C to exit..."
fi

echo "✓ Bridge appears to be running"
echo ""

# Set up environment
export ROS_DOMAIN_ID=0

# Configure FastDDS for cross-network discovery
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_client.xml"
echo "Using FastDDS config: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo ""

# Option 1: Try direct communication (if on same network)
echo "Testing network connectivity to robot..."
if ping -c 1 -W 2 10.21.31.103 > /dev/null 2>&1; then
    echo "✓ Direct network access to robot available"
    echo ""
    echo "Starting teleop in direct mode..."
    echo "Make sure FastDDS is configured for cross-network communication"
    echo ""
    
    # Run teleop directly
    python3 mission_control/local_ros2_teleop.py
else
    echo "✗ Cannot reach robot network directly"
    echo ""
    echo "Setting up ROS2 communication via SSH tunnel..."
    echo ""
    
    # Create SSH tunnel for ROS2 DDS ports
    echo "Creating SSH tunnels for ROS2 DDS discovery..."
    
    # Forward multicast discovery ports
    ssh -f -N -L 7400:localhost:7400 \
              -L 7401:localhost:7401 \
              -L 7410:localhost:7410 \
              -L 7411:localhost:7411 \
              -L 7412:localhost:7412 \
              m20-aos 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo "✓ SSH tunnels established"
    else
        echo "Note: Some tunnels may already exist (this is OK)"
    fi
    
    echo ""
    echo "Setting ROS_LOCALHOST_ONLY to use tunneled connection..."
    export ROS_LOCALHOST_ONLY=1
    
    echo ""
    echo "Starting teleop..."
    echo ""
    
    # Run teleop
    python3 mission_control/local_ros2_teleop.py
    
    # Cleanup tunnels
    echo ""
    echo "Cleaning up SSH tunnels..."
    pkill -f "ssh -f -N -L 7400"
fi

echo ""
echo "Done!"
