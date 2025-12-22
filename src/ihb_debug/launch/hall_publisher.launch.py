from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ihb_debug',
            executable='hall_publisher',
            name='hall_sensor_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ]) 