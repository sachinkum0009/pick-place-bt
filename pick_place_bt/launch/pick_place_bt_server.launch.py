from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_place_bt',
            executable='pick_place_bt_server_node',
            name='pick_place_bt_server_node',
            output='screen'
        )
    ])