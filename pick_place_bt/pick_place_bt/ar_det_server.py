#!/usr/bin/env python3

"""
AR Tag Detection Server for Pick and Place Behavior Tree
This server detects AR tags in the environment and provides their poses to the behavior tree.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import cv2


class ARDetServer(Node):
    def __init__(self):
        super().__init__("ar_det_server")
        self.get_logger().info("AR Detection Server has been started.")
        self.callback_group = ReentrantCallbackGroup()
        self.img_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10,
            callback_group=self.callback_group,
        )
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        if ids is not None:
            for corner, id in zip(corners, ids):
                self.get_logger().info(
                    f"Detected AR Tag ID: {id[0]} at corners: {corner}"
                )
        else:
            self.get_logger().info("No AR tags detected.")
        self.get_logger().info("Received an image for AR tag detection.")


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
