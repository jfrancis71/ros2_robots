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
    lidar_launch_arg = DeclareLaunchArgument(
        'lidar',
        description='Launch lidar node.',
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
            ('out/compressed', PathJoinSubstitution([robot_name, 'compressed']))],
        condition=IfCondition(LaunchConfiguration('camera'))
    )
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ],
        condition=IfCondition(LaunchConfiguration('lidar'))
    )
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0','0','0.0','1.57','0','0','base_link','base_laser'],
        condition=IfCondition(LaunchConfiguration('lidar'))
    )

    return LaunchDescription([
        robot_launch_arg,
        camera_launch_arg,
        motors_launch,
        camera_node,
        compress_node,
        lidar_node,
        base_link_to_laser_tf_node])
