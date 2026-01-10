# PUMA 树莓派网关故障排除指南

## 问题：通过树莓派连接机器人无响应

### 症状
- `local_ros2_teleop.py` 显示 "Ready!"
- 发送命令后机器人无反应
- 直接 SSH 连接机器人工作正常
- 通过树莓派网关无法控制

---

## 根本原因分析

### 1. **架构理解**
```
[您的 PC] <--ROS2 DDS--> [树莓派/Bridge] <--UDP--> [机器人]
    ↓                           ↓                    ↓
local_ros2_teleop.py    udp_ros2_bridge.py      机器人控制器
```

### 2. **可能的问题点**

#### A. UDP Bridge 中的 IP 配置错误
- **问题**: `udp_ros2_bridge.py` 中的 `ROBOT_IP = "10.21.31.103"` 硬编码
- **影响**: Bridge 发送 UDP 包到错误的地址
- **验证**: 检查机器人当前的实际 IP 地址

#### B. ROS2 DDS 发现失败
- **问题**: PC 和树莓派之间的 DDS 节点无法相互发现
- **影响**: teleop 发布的消息无法到达 bridge
- **原因**: 
  - 不同的 `ROS_DOMAIN_ID`
  - 防火墙阻止 UDP 多播
  - FastDDS 配置的网络接口不正确

#### C. 树莓派未正确转发 UDP 流量
- **问题**: 树莓派作为网关但未配置路由/NAT
- **影响**: UDP 包无法从树莓派到达机器人

---

## 解决方案

### 方案 1: 使用改进的 Bridge（推荐）

使用新创建的 `udp_ros2_bridge_gateway.py`，支持环境变量配置：

```bash
# 1. 设置机器人/树莓派 IP
export PUMA_ROBOT_IP=<树莓派或机器人的实际IP>

# 2. 在树莓派上启动 bridge（如果 bridge 运行在树莓派上）
python3 mission_control/udp_ros2_bridge_gateway.py

# 3. 在您的 PC 上启动 teleop
python3 mission_control/local_ros2_teleop.py
```

### 方案 2: 使用快速启动脚本

```bash
./start_gateway_control.sh
```

脚本会引导您完成配置。

### 方案 3: 修改现有代码

编辑 `mission_control/udp_ros2_bridge.py`：

```python
# 修改第 13 行
ROBOT_IP = "10.21.31.103"  # 改为实际的 IP
```

---

## 诊断步骤

### 1. 运行网络诊断
```bash
./diagnose_network.sh
```

### 2. 手动检查连接

#### 测试 1: Ping 测试
```bash
# 从您的 PC ping 树莓派
ping <树莓派IP>

# 从您的 PC ping 机器人（如果可达）
ping <机器人IP>

# 从树莓派 ping 机器人（SSH 到树莓派后）
ping <机器人IP>
```

#### 测试 2: ROS2 节点发现
```bash
# 终端 1: 在树莓派上启动 bridge
python3 udp_ros2_bridge.py

# 终端 2: 在您的 PC 上查看 ROS2 节点
ros2 node list
# 应该看到: /udp_bridge

# 终端 3: 查看 topics
ros2 topic list
# 应该看到: /cmd_vel, /puma/control
```

#### 测试 3: 手动发送 ROS2 消息
```bash
# 测试 cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 测试 control
ros2 topic pub /puma/control std_msgs/msg/String "data: 'stand'" --once
```

如果手动发送有效，说明问题在 `local_ros2_teleop.py`。

---

## 常见配置问题

### 问题 1: ROS_DOMAIN_ID 不匹配

**检查**:
```bash
# 在 PC 上
echo $ROS_DOMAIN_ID

# 在树莓派上（SSH 连接）
echo $ROS_DOMAIN_ID
```

**修复**: 确保两边使用相同的值（默认为 0）
```bash
export ROS_DOMAIN_ID=0
```

### 问题 2: FastDDS 网络接口配置错误

**检查** `robot_deploy/fastdds_udp_only.xml`:
```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
    <address>10.21.33.103</address>  <!-- 这个 IP 需要匹配您的网络 -->
</interfaceWhiteList>
```

**修复**: 更新为您的实际网络 IP
```bash
# 查看您的 IP
ip addr show

# 更新配置文件中的地址
```

### 问题 3: 防火墙阻止 UDP

**检查**:
```bash
# Ubuntu/Debian
sudo ufw status

# 如果启用了防火墙
sudo ufw allow 30000/udp  # 机器人控制端口
sudo ufw allow 7400:7500/udp  # ROS2 DDS 端口范围
```

---

## 部署架构建议

### 选项 A: Bridge 运行在树莓派上
```
[您的 PC]           [树莓派]              [机器人]
teleop.py  --ROS2--> bridge.py  --UDP-->  控制器
```

**优点**: 
- PC 只需要 ROS2 通信
- 树莓派直接连接机器人，延迟低

**配置**:
- 树莓派: 运行 `udp_ros2_bridge_gateway.py`
- 树莓派: `export PUMA_ROBOT_IP=<机器人IP>`
- PC: 运行 `local_ros2_teleop.py`
- 确保 PC 和树莓派的 `ROS_DOMAIN_ID` 相同

### 选项 B: Bridge 运行在您的 PC 上
```
[您的 PC]                    [树莓派]    [机器人]
teleop.py + bridge.py  --UDP--> 网关 --> 控制器
```

**优点**:
- 树莓派仅作为网络转发
- 不需要在树莓派上安装 ROS2

**配置**:
- PC: 运行 `udp_ros2_bridge_gateway.py` 和 `local_ros2_teleop.py`
- PC: `export PUMA_ROBOT_IP=<树莓派IP 或 机器人IP>`
- 树莓派: 可能需要配置 NAT/端口转发

---

## 验证清单

完成以下检查后再次测试：

- [ ] 能 ping 通树莓派
- [ ] 能 ping 通机器人（从树莓派或 PC）
- [ ] `PUMA_ROBOT_IP` 设置正确
- [ ] `ROS_DOMAIN_ID` 在所有机器上一致
- [ ] Bridge 启动时显示 "UDP Bridge Started"
- [ ] `ros2 node list` 能看到 `/udp_bridge`
- [ ] `ros2 topic list` 能看到 `/cmd_vel` 和 `/puma/control`
- [ ] Bridge 日志中能看到心跳包发送
- [ ] 手动 `ros2 topic pub` 能触发机器人响应

---

## 仍然无法工作？

### 收集调试信息

1. **Bridge 输出**:
```bash
python3 udp_ros2_bridge_gateway.py 2>&1 | tee bridge.log
```

2. **Teleop 输出**:
```bash
python3 local_ros2_teleop.py 2>&1 | tee teleop.log
```

3. **ROS2 节点图**:
```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel  # 查看是否有消息发布
```

4. **网络抓包**（高级）:
```bash
# 在树莓派上抓取发往机器人的 UDP 包
sudo tcpdump -i any -n udp port 30000 -X
```

### 联系支持时提供
- 网络拓扑图（PC、树莓派、机器人的 IP 和连接方式）
- `diagnose_network.sh` 的输出
- Bridge 和 Teleop 的完整日志
- `ros2 node list` 和 `ros2 topic list` 的输出
