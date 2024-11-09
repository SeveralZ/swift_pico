#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PicoClient(Node):
    def __init__(self):
        super().__init__('pico_client')
        self.subscription = self.create_subscription(
            String,
            'server_topic',
            self.listener_callback,
            10)
        self.get_logger().info('Pico Client is ready.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    client_node = PicoClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
