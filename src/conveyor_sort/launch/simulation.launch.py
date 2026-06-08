import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             ExecuteProcess, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('conveyor_sort')
    gazebo_pkg  = get_package_share_directory('gazebo_ros')
    world_file  = os.path.join(pkg, 'worlds', 'conveyor_world.sdf')
    config      = os.path.join(pkg, 'config', 'pid_params.yaml')
    vision_cfg  = os.path.join(pkg, 'config', 'vision_params.yaml')
    rviz_cfg    = os.path.join(pkg, 'rviz', 'conveyor_viz.rviz')
    models_path = os.path.join(pkg, 'models')
    spawn_script = os.path.join(
        get_package_share_directory('conveyor_sort'),
        '..', '..', '..', '..',
        'src', 'conveyor_sort',
        'scripts', 'spawn_boxes.py')

    return LaunchDescription([

        # Environment variables
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path),

        # 1. Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world':   world_file,
                'verbose': 'false',
                'pause':   'false',
            }.items()
        ),

        # 2. Static TF — fixes "Frame [world] does not exist" in RViz
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # 3. Camera node — simulated camera (Gazebo camera unreliable on WSL)
        TimerAction(period=6.0, actions=[
            Node(
                package='conveyor_sort',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[{'use_gazebo_camera': False}]
            ),
        ]),

        # 4. Vision + tracker
        TimerAction(period=7.0, actions=[
            Node(package='conveyor_sort', executable='vision_node',
                 name='vision_node', output='screen',
                 parameters=[vision_cfg]),
            Node(package='conveyor_sort', executable='object_tracker_node',
                 name='object_tracker_node', output='screen'),
        ]),

        # 5. Control nodes
        TimerAction(period=8.0, actions=[
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
        ]),

        # 6. Box spawner — moves boxes along belt in Gazebo
        TimerAction(period=10.0, actions=[
            ExecuteProcess(
                cmd=['python3', spawn_script],
                output='screen'
            ),
        ]),

        # 7. RViz
        TimerAction(period=11.0, actions=[
            Node(package='rviz2', executable='rviz2',
                 name='rviz2', output='screen',
                 arguments=['-d', rviz_cfg]),
        ]),

    ])