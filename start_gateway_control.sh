#!/bin/bash
# 通过树莓派网关启动 PUMA 控制系统

echo "========================================="
echo "PUMA 远程控制启动脚本（树莓派网关）"
echo "========================================="
echo ""

# 1. 获取树莓派/机器人 IP
echo "请输入机器人的 IP 地址（或树莓派网关 IP）："
echo "（直接回车使用默认值: 10.21.31.103）"
read -p "IP: " ROBOT_IP
ROBOT_IP=${ROBOT_IP:-10.21.31.103}

export PUMA_ROBOT_IP=$ROBOT_IP
export PUMA_ROBOT_PORT=30000

echo ""
echo "配置信息："
echo "  机器人 IP: $PUMA_ROBOT_IP"
echo "  机器人端口: $PUMA_ROBOT_PORT"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo ""

# 2. 测试连接
echo "测试网络连接..."
if ping -c 2 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "✓ 网络连接正常"
else
    echo "✗ 警告: 无法 ping 通 $ROBOT_IP"
    echo "  继续尝试连接..."
fi
echo ""

# 3. 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 4. 检查是否需要配置 FastDDS
if [ -z "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "配置 FastDDS UDP-only 模式..."
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/robot_deploy/fastdds_udp_only.xml"
    
    # 更新 FastDDS 配置中的 IP 地址
    LOCAL_IP=$(ip route get 1 | awk '{print $7;exit}')
    echo "  本地 IP: $LOCAL_IP"
    
    # 创建临时配置文件（如果需要）
    TEMP_FASTDDS="/tmp/puma_fastdds_$(date +%s).xml"
    sed "s/10.21.33.103/$LOCAL_IP/g" "$SCRIPT_DIR/robot_deploy/fastdds_udp_only.xml" > "$TEMP_FASTDDS"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$TEMP_FASTDDS"
    echo "  FastDDS 配置: $FASTRTPS_DEFAULT_PROFILES_FILE"
fi
echo ""

# 5. 启动选项
echo "选择启动模式："
echo "  1) 仅启动 Bridge（在机器人端或树莓派上运行）"
echo "  2) 仅启动 Teleop（在控制端运行）"
echo "  3) 同时启动 Bridge 和 Teleop（本地测试）"
read -p "选择 (1-3): " MODE

echo ""
echo "启动中..."
echo "========================================="
echo ""

case $MODE in
    1)
        echo "启动 UDP Bridge..."
        python3 "$SCRIPT_DIR/mission_control/udp_ros2_bridge_gateway.py"
        ;;
    2)
        echo "启动 Teleop..."
        echo "确保 Bridge 已在目标机器上运行！"
        sleep 2
        python3 "$SCRIPT_DIR/mission_control/local_ros2_teleop.py"
        ;;
    3)
        echo "同时启动 Bridge 和 Teleop..."
        # 在后台启动 bridge
        python3 "$SCRIPT_DIR/mission_control/udp_ros2_bridge_gateway.py" &
        BRIDGE_PID=$!
        
        echo "Bridge PID: $BRIDGE_PID"
        echo "等待 3 秒让 Bridge 初始化..."
        sleep 3
        
        # 启动 teleop
        python3 "$SCRIPT_DIR/mission_control/local_ros2_teleop.py"
        
        # 清理
        echo "关闭 Bridge..."
        kill $BRIDGE_PID 2>/dev/null
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
