from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_place_bt',
            executable='bt_executor_node',
            name='bt_executor',
            output='screen'
        )
    ])