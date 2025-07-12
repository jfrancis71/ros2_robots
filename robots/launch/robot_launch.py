from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    robot_name = LaunchConfiguration('robot')
    robot_launch_arg = DeclareLaunchArgument(
        'robot',
        description='package name of robot',
        default_value='thomas'
    )
    camera_launch_arg = DeclareLaunchArgument(
        'camera',
        description='Launch camera node.',
        default_value='false'
    )
    motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_name),
                'launch',
                'brickpi3_motors_launch.py'
            ])
        ]),
    )
    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        parameters=[{"frequency": 10.0}],
        remappings=[
            ('/image', PathJoinSubstitution([robot_name, 'image']))],
        condition=IfCondition(LaunchConfiguration('camera'))
    )
    compress_node = Node(
        package='image_transport',
        executable='republish',
        parameters=[{"in_transport": "raw", "out_transport": "compressed"}],
        remappings=[
            ('in', PathJoinSubstitution([robot_name, 'image'])),
            ('out/compressed', PathJoinSubstitution([robot_name, 'compressed']))]
    )
    return LaunchDescription([
        robot_launch_arg,
        camera_launch_arg,
        motors_launch,
        camera_node,
        compress_node])
