from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('thermal_camera_monitor'), 'config', 'camera_params.yaml')
    return LaunchDescription([
        Node(
            package='thermal_camera_monitor',
            executable='hik_alarm_node',
            name='hik_alarm_node',
            output='screen',
            parameters=[params],
        )
    ])
