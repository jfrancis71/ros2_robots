from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

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

    return LaunchDescription([slam_launch])
