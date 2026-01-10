#!/bin/bash
# 快速测试 - 在机器人上运行所有代码

echo "========================================="
echo "PUMA 快速测试（机器人本地模式）"
echo "========================================="
echo ""
echo "此脚本将："
echo "1. SSH 连接到机器人"
echo "2. 在机器人上启动 bridge 和 teleop"
echo ""
read -p "按 Enter 继续..."

echo ""
echo "连接到机器人 (m20-aos)..."
echo ""

ssh -t m20-aos << 'EOF'
echo "========================================="
echo "已连接到机器人"
echo "========================================="
echo ""

# 设置环境
export ROS_DOMAIN_ID=0
export ROBOT_IP=127.0.0.1  # 本地回环

# 检查 Python
which python3 || echo "警告: python3 未找到"

# 检查文件
if [ -f "/home/user/puma/mission_control/udp_ros2_bridge.py" ]; then
    PUMA_DIR="/home/user/puma"
elif [ -f "$HOME/puma/mission_control/udp_ros2_bridge.py" ]; then
    PUMA_DIR="$HOME/puma"
else
    echo "错误: 找不到 puma 目录"
    echo "请手动指定路径"
    read -p "puma 目录路径: " PUMA_DIR
fi

echo "使用目录: $PUMA_DIR"
cd "$PUMA_DIR" || exit 1

echo ""
echo "启动 UDP Bridge (后台)..."
python3 mission_control/udp_ros2_bridge.py > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
echo "Bridge PID: $BRIDGE_PID"

echo "等待 2 秒..."
sleep 2

# 检查 bridge 是否运行
if kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "✓ Bridge 运行中"
    echo ""
    echo "Bridge 日志 (最后10行):"
    tail -10 /tmp/bridge.log
    echo ""
else
    echo "✗ Bridge 启动失败"
    echo ""
    echo "错误日志:"
    cat /tmp/bridge.log
    exit 1
fi

echo ""
echo "========================================="
echo "现在启动 Teleop (按 Ctrl+C 退出)"
echo "========================================="
echo ""

# 启动 teleop
python3 mission_control/local_ros2_teleop.py

# 清理
echo ""
echo "关闭 Bridge..."
kill $BRIDGE_PID 2>/dev/null

echo "完成"
EOF

echo ""
echo "========================================="
echo "SSH 会话已结束"
echo "========================================="
