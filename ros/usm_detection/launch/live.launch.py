"""Start the live detector on the simulation clock and log the GPU's state beside it.

Needs bridge.launch.py and the XRCE agent. Run from the repo root (the weights and data/live
are relative paths); the run's name gives data/live/<run>.csv and data/live/<run>_gpu.csv:

    ros2 launch usm_detection live.launch.py
    ros2 launch usm_detection live.launch.py rerun:=false run:=no_viewer
"""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run = LaunchConfiguration('run')
    return LaunchDescription([
        DeclareLaunchArgument('rerun', default_value='true', description='open the Rerun viewer'),
        DeclareLaunchArgument('run', default_value=f'{datetime.now():%Y%m%d_%H%M%S}',
                              description='name of this run (must be new)'),
        Node(
            package='usm_detection',
            executable='live_detector',
            parameters=[{'use_sim_time': True, 'rerun': LaunchConfiguration('rerun'),
                         'csv': ['data/live/', run, '.csv']}],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', 'tools/sample_gpu.py', ['data/live/', run, '_gpu.csv']],
            output='screen',
        ),
    ])
