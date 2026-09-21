#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import ApplyBodyWrench
import time

class DeepscanControl(Node):
    def __init__(self):
        super().__init__('deepscan_control')
        self.sub = self.create_subscription(
            Twist,
            '/deepscan_boat/cmd_vel',
            self.cmd_callback,
            10)
        self.client = self.create_client(ApplyBodyWrench, '/apply_body_wrench')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /apply_body_wrench...')
        self.get_logger().info('Ready! Use keyboard to move.')

    def cmd_callback(self, msg: Twist):
        req = ApplyBodyWrench.Request()
        req.body_name = 'deepscan_boat::base_link'
        # Преобразуем линейную скорость в силу (чем больше скорость, тем сильнее толкаем)
        req.wrench.force.x = msg.linear.x * 1000.0   # 0.5 м/с → 500 Н
        req.wrench.force.y = msg.linear.y * 1000.0
        req.wrench.torque.z = msg.angular.z * 500.0 # 1.0 рад/с → 500 Н·м
        req.duration.sec = 0.1                       # короткий импульс, чтобы не перегружать
        req.start_time = self.get_clock().now().to_msg()
        self.client.call_async(req)

def main():
    rclpy.init()
    node = DeepscanControl()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
