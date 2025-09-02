#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from battery_soc import get_battery_status_mv
import socket
import json
import time

class BatteryUDPNode(Node):
    def __init__(self):
        super().__init__('battery_control')

        # 参数：UDP 目标地址和端口
        self.declare_parameter('udp_ip', '127.0.0.1')
        self.declare_parameter('udp_port', 5555)

        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"UDP destination: {self.udp_ip}:{self.udp_port}")

        self.subscription = self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery', 
            self.battery_callback,
            10
        )
        self.latest_mv = None
        self.get_logger().info(f"Subscribed to '/ros_robot_controller/battery' topic.")

        # 每 30 秒发送一次 UDP
        self.timer = self.create_timer(30.0, self.timer_callback)

    def battery_callback(self, msg):
        self.latest_mv = msg.data
        self.get_logger().debug(f"Received battery voltage: {self.latest_mv} mV")

    def timer_callback(self):
        if self.latest_mv is None:
            self.get_logger().warn("No battery data received yet.")
            return

        # 计算状态
        status = get_battery_status_mv(self.latest_mv)

        # 转为 JSON
        json_data = json.dumps(status, ensure_ascii=False)

        # 发送 UDP
        try:
            self.sock.sendto(json_data.encode('utf-8'), (self.udp_ip, self.udp_port))
            self.get_logger().info(f"Sent via UDP: {json_data}")
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")

# ========================
# 主函数
# ========================
def main(args=None):
    rclpy.init(args=args)
    node = BatteryUDPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()