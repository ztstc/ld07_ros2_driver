from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('ld07_ros2_driver'),
        'config',
        'params.yaml'
    )

    if not os.path.isfile(config_file):
        raise FileNotFoundError(f"Parameter file not found: {config_file}")

    return LaunchDescription([
        Node(
            package='ld07_ros2_driver',
            executable='ld07_node',
            name='ld07_node',
            parameters=[config_file]
        )
    ])