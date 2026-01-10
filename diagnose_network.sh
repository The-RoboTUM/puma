#!/bin/bash
# 网络诊断脚本 - 检查树莓派网关配置

echo "========================================="
echo "PUMA 网络诊断工具"
echo "========================================="
echo ""

# 1. 检查本地 IP 地址
echo "1. 本地网络接口："
ip addr show | grep -E "inet " | grep -v "127.0.0.1"
echo ""

# 2. 检查到机器人的连接（通过树莓派）
echo "2. 请输入树莓派的 IP 地址（网关）："
read GATEWAY_IP
echo ""

echo "   测试到树莓派的连接..."
ping -c 3 $GATEWAY_IP
echo ""

# 3. 检查机器人 IP
echo "3. 请输入机器人的 IP 地址："
read ROBOT_IP
echo ""

echo "   测试到机器人的连接（通过网关）..."
ping -c 3 $ROBOT_IP
echo ""

# 4. 检查 UDP 端口
echo "4. 测试 UDP 端口 30000（机器人控制端口）..."
echo "   发送测试数据包..."
timeout 2 nc -u -w 1 $ROBOT_IP 30000 <<< "test" && echo "   端口可达" || echo "   端口不可达或无响应"
echo ""

# 5. 检查 ROS2 环境
echo "5. ROS2 环境变量："
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-未设置}"
echo "   FASTRTPS_DEFAULT_PROFILES_FILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-未设置}"
echo ""

# 6. 检查进程
echo "6. 相关进程检查："
echo "   udp_ros2_bridge 进程："
pgrep -af udp_ros2_bridge || echo "   未运行"
echo ""
echo "   local_ros2_teleop 进程："
pgrep -af local_ros2_teleop || echo "   未运行"
echo ""

echo "========================================="
echo "诊断建议："
echo "========================================="
echo "1. 确保您能 ping 通树莓派和机器人"
echo "2. 确保 udp_ros2_bridge.py 中的 ROBOT_IP 指向正确的地址"
echo "3. 确保两台机器（PC 和树莓派）在同一 ROS_DOMAIN_ID"
echo "4. 如果通过树莓派，可能需要配置端口转发或代理"
echo ""
