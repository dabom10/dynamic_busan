import os
import time
import sys
import numpy as np
import rclpy
from rclpy.node import Node

class ShakeController(Node):
    def __init__(self):
        super().__init__("shake_node")


    def shake_start(self):
        self.get_logger().info("Shake Start")


def main(args=None):
    rclpy.init(args=args)
    node = ShakeController()

    try:
        while rclpy.ok():
            node.shake_start()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()