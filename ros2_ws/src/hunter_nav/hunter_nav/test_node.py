#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f'Timer callback called {self.counter_} times.')

def main(args=None):
    rclpy.init(args=args)

    node = TestNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()