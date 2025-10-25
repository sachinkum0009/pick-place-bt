from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ar_det_server_node = Node(
        package='pick_place_bt',
        executable='ar_det_server_node.py',
        name='ar_det_server',
        output='screen'
    )
    ld.add_action(ar_det_server_node)
    return ld