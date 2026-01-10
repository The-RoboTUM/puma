#!/usr/bin/env python3
"""
ROS2 Topic Relay - Robot Side (Bidirectional TCP Server)
- Sends robot topics (IMU, status, etc.) TO PC clients
- Receives commands (cmd_vel, control) FROM PC clients and publishes locally
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import socket
import threading
import json
import struct
import time

TCP_HOST = '0.0.0.0'
TCP_PORT = 9999


class BidirectionalRelayServer(Node):
    def __init__(self):
        super().__init__('relay_server')
        
        self._tcp_clients = []
        self._clients_lock = threading.Lock()
        
        # Publishers - for commands received FROM PC
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_control = self.create_publisher(String, '/puma/control', 10)
        
        # Subscribers - for topics to send TO PC
        self.create_subscription(Imu, '/IMU', self.imu_callback, 10)
        
        # Heartbeat timer
        self.create_timer(5.0, self.heartbeat)
        
        # Start TCP server
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f'Bidirectional Relay Server on port {TCP_PORT}')
        self.get_logger().info('  PC -> Robot: /cmd_vel, /puma/control')
        self.get_logger().info('  Robot -> PC: /IMU')

    def heartbeat(self):
        with self._clients_lock:
            n = len(self._tcp_clients)
        if n > 0:
            self.get_logger().info(f'{n} client(s) connected')

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_HOST, TCP_PORT))
        server.listen(5)
        self.get_logger().info(f'TCP listening on {TCP_HOST}:{TCP_PORT}')
        
        while True:
            try:
                client, addr = server.accept()
                self.get_logger().info(f'Client connected: {addr}')
                with self._clients_lock:
                    self._tcp_clients.append(client)
                t = threading.Thread(target=self.receive_from_client, args=(client, addr), daemon=True)
                t.start()
            except Exception as e:
                self.get_logger().error(f'Server error: {e}')

    def receive_from_client(self, client, addr):
        buffer = b''
        try:
            while True:
                data = client.recv(4096)
                if not data:
                    break
                buffer += data
                
                while len(buffer) >= 4:
                    msg_len = struct.unpack('>I', buffer[:4])[0]
                    if len(buffer) < 4 + msg_len:
                        break
                    json_data = buffer[4:4+msg_len]
                    buffer = buffer[4+msg_len:]
                    self.process_command(json.loads(json_data))
        except Exception as e:
            self.get_logger().warn(f'Client {addr} error: {e}')
        finally:
            with self._clients_lock:
                if client in self._tcp_clients:
                    self._tcp_clients.remove(client)
            self.get_logger().info(f'Client disconnected: {addr}')

    def process_command(self, packet):
        topic = packet.get('topic')
        data = packet.get('data', {})
        
        self.get_logger().debug(f'Received: topic={topic}, data={data}')
        
        if topic == '/cmd_vel':
            msg = Twist()
            msg.linear.x = float(data.get('linear', {}).get('x', 0.0))
            msg.linear.y = float(data.get('linear', {}).get('y', 0.0))
            msg.linear.z = float(data.get('linear', {}).get('z', 0.0))
            msg.angular.x = float(data.get('angular', {}).get('x', 0.0))
            msg.angular.y = float(data.get('angular', {}).get('y', 0.0))
            msg.angular.z = float(data.get('angular', {}).get('z', 0.0))
            self.pub_cmd_vel.publish(msg)
            self.get_logger().info(f'cmd_vel: lin={msg.linear.x:.2f} ang={msg.angular.z:.2f}')
            
        elif topic == '/puma/control':
            # Handle both {'data': 'stand'} and direct string
            cmd = data.get('data', data) if isinstance(data, dict) else str(data)
            msg = String()
            msg.data = str(cmd)
            self.pub_control.publish(msg)
            self.get_logger().info(f'Control: {msg.data}')

    def send_to_clients(self, topic, msg_type, data):
        packet = {'topic': topic, 'type': msg_type, 'data': data, 'ts': time.time()}
        json_data = json.dumps(packet).encode('utf-8')
        frame = struct.pack('>I', len(json_data)) + json_data
        
        with self._clients_lock:
            dead = []
            for c in self._tcp_clients:
                try:
                    c.sendall(frame)
                except:
                    dead.append(c)
            for c in dead:
                self._tcp_clients.remove(c)

    def imu_callback(self, msg):
        data = {
            'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y, 
                           'z': msg.orientation.z, 'w': msg.orientation.w},
            'angular_velocity': {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y,
                                'z': msg.angular_velocity.z},
            'linear_acceleration': {'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y,
                                   'z': msg.linear_acceleration.z}
        }
        self.send_to_clients('/IMU', 'sensor_msgs/Imu', data)


def main(args=None):
    rclpy.init(args=args)
    node = BidirectionalRelayServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
