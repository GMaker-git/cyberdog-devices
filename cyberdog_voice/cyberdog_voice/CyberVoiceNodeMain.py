#!/usr/bin/python3
#
# Copyright (c) 2023 Gmaker
#
# Cybertail setup
# Open source on https://github.com/GMaker-git

import rclpy
from rclpy.executors import MultiThreadedExecutor

from . import GVoiceNode


def main(args=None):
    rclpy.init(args=args)
    bt_node = GVoiceNode.GVoiceNode('cyberdog_voice')
    bt_node.get_logger().info("cyberdog_voice is comming.")
    rclpy.spin(bt_node)
    # try:
    #     bt_node.startListen()
    # except KeyboardInterrupt:
    #     bt_node.stopListen()

    bt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
