from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    decompress_node = Node(
        package='image_transport',
        executable='republish',
        parameters=[{"in_transport": "compressed", "out_transport": "raw"}],
        remappings=[
            ('in/compressed', 'thomas/compressed'),
            ('out', '/desktop/image')],
    )
    stereo_split_node = Node(
        package='stereo_demo',
        executable='stereo_split_node',
        parameters=[{"left_camera_info_url": "file:///root/ros2_config/calibration_30_01_2026/left.yaml"},
            {"right_camera_info_url": "file:///root/ros2_config/calibration_30_01_2026/right.yaml"}],
        remappings=[
        ('/image', "/desktop/image")],
    )
    stereo_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_image_proc'),
                'launch',
                'stereo_image_proc.launch.py'
            ])
        ]),
        launch_arguments = [('sgbm_mode', '2')]
    )
    base_link_to_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['--x','0.0', '--y','0.0', '--z','0.0', '--roll','-1.57', '--pitch','0' , '--yaw','-1.57', '--frame-id','base_link', '--child-frame-id','camera_frame'],
    )

    return LaunchDescription([
        decompress_node,
        stereo_split_node,
        stereo_proc_launch,
        base_link_to_camera_tf_node])
