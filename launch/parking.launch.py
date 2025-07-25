from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    description_prefix = get_package_share_directory("ibt_ros2_description")
    arm_type = LaunchConfiguration("arm_type", default="robofox_61814v3")
    arm_prefix = LaunchConfiguration("arm_prefix", default="robofox")

    ibt_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ibt_ros2_driver'),
                'launch',
                'ibt_ros2_driver.launch.py'
            )
        ),
        launch_arguments={
            'ns': 'robofox',
            'url': 'wss://192.168.1.100:5568:5567',
            'prefix': 'robofox',
            'timeout_ms': '10000',
            'login': 'admin',
            'password': 'admin',
            'source_frame': 'base_link',
            'target_frame': 'robofox_link6',
        }.items()
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_prefix, 'launch', 'combofox_display.launch.py'
            )
        ),
        launch_arguments={
            'use_gui': 'false',
            'use_rviz': LaunchConfiguration('use_rviz', default='false'),
            'arm_type': arm_type,
            'arm_prefix': arm_prefix
        }.items()
    )

    controller_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='combofox_parking',
                executable='parking_executor',
                name='combofox_controller',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz for camera.'),
        DeclareLaunchArgument('arm_type', default_value='robofox_61814v3', description='Type of the arm.'),
        DeclareLaunchArgument('arm_prefix', default_value='robofox', description='Prefix for the arm.'),

        robot_description_launch,
        ibt_driver_launch,
        controller_node
    ])
