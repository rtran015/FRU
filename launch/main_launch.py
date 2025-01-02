from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
                package="test_package",
                executable="main_joy",
            ),
        Node(
            package="test_package", 
            executable="main_drive"
            ),
        Node(
            package='autonomous',
            executable='auto',
        ),
        Node(
            package='sensors',
            executable='sensor'
        ),
        Node(
            package='db_broker',
            executable='broker',
            arguments=['-p', ['can_id:=15']]
        ),
        Node(
            package='db_broker',
            executable='broker',
            arguments=['-p', ['can_id:=16']]
        ),         
         Node(
            package='db_broker',
            executable='broker',
            arguments=['-p', ['can_id:=17']]
        ),         
         Node(
            package='db_broker',
            executable='broker',
            arguments=['-p', ['can_id:=18']]
        ),
    ])
