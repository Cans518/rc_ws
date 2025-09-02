#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import socket
import threading
import json
from datetime import datetime, timedelta

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # --- 参数：电机增益 ---
        self.declare_parameter('left_motor_gain', 1.0)
        self.declare_parameter('right_motor_gain', 1.0)
        self.left_gain = self.get_parameter('left_motor_gain').get_parameter_value().double_value
        self.right_gain = self.get_parameter('right_motor_gain').get_parameter_value().double_value

        # --- 参数：UDP 端口 ---
        self.declare_parameter('udp_port', 8080)
        udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        # --- 参数：超时时间（毫秒）---
        self.declare_parameter('command_timeout_ms', 500)
        self.timeout = timedelta(milliseconds=self.get_parameter('command_timeout_ms').get_parameter_value().integer_value)

        # 电机控制发布者
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)

        # 共享状态
        self.command = "stop"
        self.speed = 1.0  # 默认速度
        self.last_command_time = datetime.now()
        self.command_lock = threading.Lock()

        # 启动 UDP 服务器线程
        self.udp_thread = threading.Thread(target=self.udp_server, args=(udp_port,), daemon=True)
        self.udp_thread.start()

        # 定时发布电机指令（10Hz）
        self.timer = self.create_timer(0.1, self.publish_motor_command)

        self.get_logger().info(
            f"Motor control node started with UDP on port {udp_port}, "
            f"timeout={self.timeout.total_seconds()*1000:.0f}ms, "
            f"left_gain={self.left_gain}, right_gain={self.right_gain}"
        )

    def udp_server(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('127.0.0.1', port)
        try:
            sock.bind(server_address)
            self.get_logger().info(f"UDP server listening on {server_address}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket: {e}")
            return

        while rclpy.ok():
            try:
                sock.settimeout(1.0)  # 防止无限阻塞
                data, address = sock.recvfrom(1024)
                message = data.decode('utf-8').strip()

                try:
                    # 解析 JSON
                    data = json.loads(message)
                    cmd = data.get("cmd", "").lower()
                    speed = data.get("speed", 1.0)

                    if cmd not in ["forward", "backward", "left", "right", "stop"]:
                        self.get_logger().warn(f"Invalid command: {cmd}")
                        continue

                    if not isinstance(speed, (int, float)) or speed < 0:
                        self.get_logger().warn(f"Invalid speed value: {speed}")
                        continue

                    # 更新命令
                    with self.command_lock:
                        self.command = cmd
                        self.speed = float(speed)
                        self.last_command_time = datetime.now()

                    self.get_logger().info(f"Received from {address}: {data}")

                except json.JSONDecodeError:
                    self.get_logger().warn(f"Invalid JSON: {message}")

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP receive error: {e}")

    def publish_motor_command(self):
        now = datetime.now()
        elapsed = now - self.last_command_time

        with self.command_lock:
            cmd = self.command
            speed = self.speed
            active = elapsed < self.timeout

        # 超时则强制停止
        if not active and cmd != "stop":
            cmd = "stop"
            self.get_logger().warn("Command timeout, stopping motors.")

        msg = MotorsState()
        motor1 = MotorState()  # 假设 motor1 是左轮
        motor2 = MotorState()  # 假设 motor2 是右轮
        motor1.id = 1
        motor2.id = 2

        base_speed = speed

        if cmd == "forward":
            motor1.rps = base_speed * self.left_gain
            motor2.rps = -base_speed * self.right_gain
        elif cmd == "backward":
            motor1.rps = -base_speed * self.left_gain
            motor2.rps = base_speed * self.right_gain
        elif cmd == "left":
            motor1.rps = 0.0
            motor2.rps = -base_speed * self.right_gain
        elif cmd == "right":
            motor1.rps = base_speed * self.left_gain
            motor2.rps = 0.0
        else:  # stop or timeout
            motor1.rps = 0.0
            motor2.rps = 0.0

        msg.data = [motor1, motor2]
        self.motor_pub.publish(msg)

        # 只有在有效命令期间才打印（避免刷屏）
        if elapsed < self.timeout:
            self.get_logger().debug(
                f'Pub: L={motor1.rps:.2f}, R={motor2.rps:.2f} | '
                f'Cmd={cmd}, Speed={speed}, Elapsed={elapsed.total_seconds()*1000:.0f}ms'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()