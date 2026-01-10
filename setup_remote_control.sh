#!/bin/bash
# SSH 隧道方式控制机器人（适用于远程场景）

echo "========================================="
echo "PUMA SSH 隧道控制方案"
echo "========================================="
echo ""

# 检测当前网络状态
LOCAL_IP=$(ip route get 1 | awk '{print $7;exit}')
echo "本地 IP: $LOCAL_IP"

# 检查是否能直接访问机器人
if ping -c 1 -W 1 10.21.31.103 > /dev/null 2>&1; then
    echo "✓ 可以直接访问机器人 (10.21.31.103)"
    ROBOT_ACCESSIBLE="direct"
    ROBOT_IP="10.21.31.103"
else
    echo "✗ 无法直接访问机器人网络 (10.21.31.x)"
    echo ""
    echo "远程控制方案："
    echo "1. 在机器人上运行 bridge"
    echo "2. 在您的 PC 上运行 teleop"
    echo "3. 使用 SSH 隧道转发 ROS2 DDS 流量"
    echo ""
    ROBOT_ACCESSIBLE="remote"
fi

if [ "$ROBOT_ACCESSIBLE" = "remote" ]; then
    echo "========================================="
    echo "设置方案"
    echo "========================================="
    echo ""
    echo "步骤 1: SSH 连接到机器人"
    echo "  ssh m20-aos"
    echo ""
    echo "步骤 2: 在机器人上运行（在 SSH 会话中）:"
    echo "  cd /path/to/puma"
    echo "  export ROS_DOMAIN_ID=0"
    echo "  python3 mission_control/udp_ros2_bridge.py"
    echo ""
    echo "步骤 3: 在您的 PC 上（新终端）运行:"
    echo "  export ROS_DOMAIN_ID=0"
    echo "  python3 mission_control/local_ros2_teleop.py"
    echo ""
    echo "注意: 这种方式要求机器人和 PC 都连接到互联网"
    echo "     或在同一局域网内"
    echo ""
    echo "========================================="
    echo "或者使用 ROS2 over SSH（高级）"
    echo "========================================="
    echo ""
    echo "如果需要通过 SSH 隧道，需要配置:"
    echo "1. SSH 端口转发 (比较复杂)"
    echo "2. 或者使用 VPN 连接两个网络"
    echo ""
    
    read -p "是否要生成 SSH 隧道配置脚本? (y/n): " GENERATE
    
    if [ "$GENERATE" = "y" ]; then
        cat > /tmp/setup_ssh_tunnel.sh << 'TUNNEL_EOF'
#!/bin/bash
# SSH 隧道设置 - 将 ROS2 DDS 端口转发到机器人

echo "建立 SSH 隧道到机器人..."
echo "这将转发以下端口:"
echo "  - 7400-7500 (ROS2 DDS discovery)"
echo "  - 11811 (ROS2 daemon)"
echo ""

# 创建多个端口转发
ssh -f -N \
  -L 7400:localhost:7400 \
  -L 7401:localhost:7401 \
  -L 7402:localhost:7402 \
  -L 7410:localhost:7410 \
  -L 7411:localhost:7411 \
  -L 7412:localhost:7412 \
  -L 11811:localhost:11811 \
  m20-aos

if [ $? -eq 0 ]; then
    echo "✓ SSH 隧道已建立"
    echo ""
    echo "现在可以运行:"
    echo "  export ROS_LOCALHOST_ONLY=1"
    echo "  python3 mission_control/local_ros2_teleop.py"
else
    echo "✗ SSH 隧道建立失败"
fi
TUNNEL_EOF
        chmod +x /tmp/setup_ssh_tunnel.sh
        echo ""
        echo "✓ 已生成隧道脚本: /tmp/setup_ssh_tunnel.sh"
        echo ""
    fi
else
    # 直接访问模式
    echo ""
    echo "选择运行模式:"
    echo "  1) 本地模式（PC 和机器人在同一网络）"
    echo "  2) 生成机器人端部署脚本"
    read -p "选择 (1-2): " MODE
    
    if [ "$MODE" = "2" ]; then
        # 生成在机器人上运行的脚本
        cat > /tmp/deploy_to_robot.sh << 'DEPLOY_EOF'
#!/bin/bash
# 在机器人上部署和运行 bridge

ROBOT_HOST="m20-aos"
PUMA_DIR="/home/user/puma"  # 根据实际路径修改

echo "部署 bridge 到机器人..."

# 复制文件
scp mission_control/udp_ros2_bridge.py $ROBOT_HOST:$PUMA_DIR/
scp mission_control/protocol.py $ROBOT_HOST:$PUMA_DIR/

echo ""
echo "启动 bridge (在机器人上)..."

ssh $ROBOT_HOST << 'REMOTE_SCRIPT'
cd /home/user/puma
export ROS_DOMAIN_ID=0
export ROBOT_IP=127.0.0.1  # 本地回环，因为在机器人上运行
python3 udp_ros2_bridge.py
REMOTE_SCRIPT
DEPLOY_EOF
        chmod +x /tmp/deploy_to_robot.sh
        echo "✓ 已生成部署脚本: /tmp/deploy_to_robot.sh"
    fi
fi

echo ""
echo "========================================="
