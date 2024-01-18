#!/usr/bin/python3
#
# Copyright (c) 2023 Gmaker
#
# Cybertail setup
# Open source on https://github.com/GMaker-git

import rclpy
from rclpy.executors import MultiThreadedExecutor

from . import CyberTailNode


def main(args=None):
    rclpy.init(args=args)
    bt_node = CyberTailNode.CyberTailNode('cyberdog_tail')
    bt_node.get_logger().info("cyberdog_tail is comming.")
    rclpy.spin(bt_node)
    bt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
