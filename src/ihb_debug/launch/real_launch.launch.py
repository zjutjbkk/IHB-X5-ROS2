import os
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        # 启动 realsense2_camera 节点
        LogInfo(msg='启动 realsense2_camera 节点'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={'depth_module.profile': '640x480x15'}.items()
        ),


        # 等待 20 秒后启动 mono2d_body_detection 节点
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='启动 ROS2 mono2d_body_detection 节点'),
                Node(
                    package='mono2d_body_detection',
                    executable='mono2d_body_detection',
                    output='screen',
                    parameters=[
                        {'is_shared_mem_sub': 0},
                        {'ros_img_topic_name': '/camera/camera/color/image_raw'},
                        ],
                    arguments=['--log-level', 'error']
                ),
            ],
        ),


    ])
