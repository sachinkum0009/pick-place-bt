#!/usr/bin/env python3

import rclpy

from pick_place_bt.ar_det_server import ARDetServer
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    ar_det_server = ARDetServer()
    executor = MultiThreadedExecutor()
    executor.add_node(ar_det_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        ar_det_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()