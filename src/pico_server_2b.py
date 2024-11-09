#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PicoServer(Node):
    def __init__(self):
        super().__init__('pico_server')
        self.publisher_ = self.create_publisher(String, 'server_topic', 10)
        self.get_logger().info('Pico Server is ready.')

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {message}')

def main(args=None):
    rclpy.init(args=args)
    server_node = PicoServer()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
