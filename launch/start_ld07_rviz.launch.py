from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('ld07_ros2_driver'),
        'rviz',
        'ld07.rviz'
    )
    
    print(f"Loading RViz config from: {rviz_config_path}")

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
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])