from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('conveyor_sort')
    vision_config = os.path.join(pkg, 'config', 'vision_params.yaml')

    return LaunchDescription([
        Node(package='conveyor_sort', executable='camera_node',
             name='camera_node', output='screen'),
        Node(package='conveyor_sort', executable='vision_node',
             name='vision_node', output='screen',
             parameters=[vision_config]),
        Node(package='conveyor_sort', executable='object_tracker_node',
             name='object_tracker_node', output='screen'),
    ])