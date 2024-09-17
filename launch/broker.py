from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_package',
            executable='sensor',
        ),
        Node(
            package='test_package',
            executable='broker'
        ),
    ])
