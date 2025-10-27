from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    tag_pose_server_node = Node(
        package='pick_place_bt',
        executable='tag_pose_server_node',
        name='tag_pose_server',
        output='screen'
    )
    ld.add_action(tag_pose_server_node)
    return ld