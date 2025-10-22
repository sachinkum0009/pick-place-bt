from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # allow overriding which robot description and moveit config to use
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('model', default_value='cobotta', description='robot model name'))
    declared_arguments.append(
        DeclareLaunchArgument('description_package', default_value='denso_robot_descriptions',
                               description='Package that contains the robot urdf/xacro'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file', default_value='denso_robot.urdf.xacro',
                               description='URDF/XACRO description file'))
    declared_arguments.append(
        DeclareLaunchArgument('moveit_config_package', default_value='denso_robot_moveit_config',
                               description='MoveIt config package'))
    declared_arguments.append(
        DeclareLaunchArgument('moveit_config_file', default_value='denso_robot.srdf.xacro',
                               description='SRDF/XACRO file for MoveIt'))

    model = LaunchConfiguration('model')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')

    # Build robot_description (URDF) from xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]), ' ',
        'model:=', model, ' '
    ])
    robot_description = {'robot_description': robot_description_content}

    # Build robot_description_semantic (SRDF) from xacro if available
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare(moveit_config_package), 'srdf', moveit_config_file]), ' ',
        'model:=', model, ' '
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    node = Node(
        package='pick_place_bt',
        executable='move_group_server_node',
        name='move_group_server_node',
        output='screen',
        namespace="cobotta",
        parameters=[robot_description, robot_description_semantic]
    )

    return LaunchDescription(declared_arguments + [node])