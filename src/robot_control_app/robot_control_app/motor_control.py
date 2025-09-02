#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        self.timer = self.create_timer(0.5, self.publish_motor_command)
        self.direction = 1
        
    def publish_motor_command(self):
        msg = MotorsState()
        
        motor1 = MotorState()
        motor1.id = 1
        motor1.rps = 1.0 * self.direction
        
        motor2 = MotorState()
        motor2.id = 2
        motor2.rps = -1.0 * self.direction
        
        msg.data = [motor1, motor2]
        self.motor_pub.publish(msg)
        
        self.direction *= -1  # 反转方向
        self.get_logger().info(f'Published motor command: {motor1.rps}, {motor2.rps}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()