#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Publish a static transform from frame "J6" -> "zed_camera_link" at (0,0,0) with identity rotation.
    args = ["0", "0", "0", "0", "-1.571", "0.0", "J6", "zed_camera_link"]
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_j6_to_camera",
        output="screen",
        arguments=args,
    )

    return LaunchDescription([static_tf_node])
