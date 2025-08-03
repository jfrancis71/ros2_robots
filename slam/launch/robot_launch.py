from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    camera_launch_arg = DeclareLaunchArgument(
        'camera',
        description='Launch camera node.',
        default_value='false'
    )
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robots"),
                'launch',
                'robot_launch.py'
            ])
        ]),
        launch_arguments={
                'robot': 'thomas',
                'camera': LaunchConfiguration('camera'),
                'lidar': 'true'
            }.items()
        )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
                'slam_params_file': PathJoinSubstitution([
                    FindPackageShare("slam"),
                    'config',
                    'mapper_params_online_async.yaml'
                    ])
            }.items()
        )

    return LaunchDescription([
        camera_launch_arg,
        robot_launch,
        slam_launch])
