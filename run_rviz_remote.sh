#!/bin/bash
# Run RViz on the robot with X11 forwarding

echo "Connecting to robot and launching RViz..."
echo "Make sure the robot is running topic_relay_server and udp_ros2_bridge"
echo ""

# Connect to robot with X11 forwarding and run RViz
ssh -X user@10.21.41.1 "
    source /opt/ros/foxy/setup.bash
    export DISPLAY=:0
    rviz2
"
