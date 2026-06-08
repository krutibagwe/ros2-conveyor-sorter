from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg        = get_package_share_directory('conveyor_sort')
    config     = os.path.join(pkg, 'config', 'pid_params.yaml')
    vision_cfg = os.path.join(pkg, 'config', 'vision_params.yaml')
    rviz_cfg   = os.path.join(pkg, 'rviz', 'conveyor_viz.rviz')

    return LaunchDescription([

        # Static TF — fixes "Frame [world] does not exist" in RViz
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        Node(package='conveyor_sort', executable='camera_node',
             name='camera_node', output='screen',
             parameters=[{'use_gazebo_camera': False}]),

        Node(package='conveyor_sort', executable='vision_node',
             name='vision_node', output='screen',
             parameters=[vision_cfg]),

        Node(package='conveyor_sort', executable='object_tracker_node',
             name='object_tracker_node', output='screen'),

        Node(package='conveyor_sort', executable='controller_node',
             name='controller_node', output='screen',
             parameters=[config]),

        Node(package='conveyor_sort', executable='conveyor_node',
             name='conveyor_node', output='screen',
             parameters=[config]),

        Node(package='conveyor_sort', executable='sorting_node',
             name='sorting_node', output='screen'),

        Node(package='conveyor_sort', executable='jam_monitor_node',
             name='jam_monitor_node', output='screen'),

        Node(package='conveyor_sort', executable='rviz_display_node',
             name='rviz_display_node', output='screen'),

        Node(package='rviz2', executable='rviz2',
             name='rviz2', output='screen',
             arguments=['-d', rviz_cfg]),
    ])