from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        parameters=[{"frequency": 10.0}],
        remappings=[
            ('/image', '/robot/image')]
    )
    compress_node = Node(
        package='image_transport',
        executable='republish',
        parameters=[{"in_transport": "raw", "out_transport": "compressed"}],
        remappings=[
            ('in', '/robot/image'),
            ('out/compressed', '/robot/compressed')]
    )
    return LaunchDescription([
        camera_node,
        compress_node])
