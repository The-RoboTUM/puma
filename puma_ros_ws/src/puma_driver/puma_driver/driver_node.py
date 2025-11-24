import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import asyncio
import threading

from .robot_client import RobotClient   # 使用你刚提取的 RobotClient


class PumaRosDriver(Node):
    def __init__(self):
        super().__init__("puma_driver")

        # === 初始化 TCP 客户端 ===
        # 使用 RobotClient 默认的 ROBOT_IP / ROBOT_PORT（来自 protocol_utils）
        # 这样行为和原来的 teleop_robot.py 完全一致
        self.client = RobotClient()

        # === 订阅 /cmd_vel ===
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # === 发布 /puma/status ===
        self.status_pub = self.create_publisher(String, "/puma/status", 10)

        # === 把 robot_client 的回调绑定为 ROS 发布函数 ===
        self.client.on_message_received = self.on_robot_message

        # === 在后台线程启动 asyncio loop（运行 TCP）===
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._start_loop, daemon=True).start()

        # 在 loop 中启动机器人任务
        asyncio.run_coroutine_threadsafe(self.start_robot_client(), self.loop)

        self.get_logger().info("Puma ROS Driver started!")

    def _start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def start_robot_client(self):
        # Connect + start loops
        await self.client.connect()

        tasks = [
            asyncio.create_task(self.client.heartbeat_loop()),
            asyncio.create_task(self.client.listen_loop()),
            asyncio.create_task(self.client.control_loop()),
        ]

        await asyncio.gather(*tasks)

    def cmd_vel_callback(self, msg: Twist):
        # 将 /cmd_vel 转换为底层期望的速度（目标速度由 control_loop 平滑发送）
        self.client.target_vx = msg.linear.x
        self.client.target_vy = msg.linear.y
        self.client.target_vw = msg.angular.z

    def on_robot_message(self, payload_bytes):
        # 将机器人反馈发布到 ROS
        txt = payload_bytes.decode("utf-8", errors="ignore")

        # 简单打印前 200 个字符，帮助你确认确实收到了东西
        print(f"[RX] from robot: {txt[:200]}")

        ros_msg = String()
        ros_msg.data = txt
        self.status_pub.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PumaRosDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
