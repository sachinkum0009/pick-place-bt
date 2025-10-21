from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_place_bt',
            executable='pick_place_bt_node',
            name='pick_place_bt_node',
            output='screen'
        )
    ])