"""Bridge the Gazebo camera and true poses into ROS 2 and record a survey to MCAP.

Run from the repo root while the simulation and the XRCE agent are up; stop with Ctrl+C
after landing:

    ros2 launch usm_mission record.launch.py                 # -> data/bags/<YYYYmmdd_HHMMSS>
    ros2 launch usm_mission record.launch.py out:=data/bags/alt30 world:=village
"""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLE = 'usm_x500_gimbal_0'  # model name PX4 spawns: <PX4_SIM_MODEL without gz_>_<instance>

# All stamps share the simulation clock (UXRCE_DDS_SYNCT=0 in the airframe).
PX4_TOPICS = [
    '/fmu/out/vehicle_local_position_v1',      # EKF2 position estimate (NED)
    '/fmu/out/vehicle_attitude',               # vehicle orientation
    '/fmu/out/gimbal_device_attitude_status',  # measured camera orientation
    '/fmu/out/vehicle_status_v1',              # arming state and flight mode
]
SIM_TOPICS = [
    '/camera/image',
    '/camera/camera_info',  # intrinsics
    # True poses for evaluation. Known gap: the Pose_V -> TFMessage conversion drops the
    # model names and stamps (only the order survives); to be replaced by a stamped source.
    '/gz/dynamic_pose',
    '/clock',
    '/rosout',              # mission log
]


def _bridge_and_record(context):
    world = LaunchConfiguration('world').perform(context)
    out = LaunchConfiguration('out').perform(context)
    camera = f'/world/{world}/model/{VEHICLE}/link/camera_link/sensor/camera'
    poses = f'/world/{world}/dynamic_pose/info'

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[  # gz -> ROS only ('[')
            f'{camera}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{camera}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'{poses}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            (f'{camera}/image', '/camera/image'),
            (f'{camera}/camera_info', '/camera/camera_info'),
            (poses, '/gz/dynamic_pose'),
        ],
        output='screen',
    )
    # Lossless zstd: images of the empty test world shrink ~380x, so the real ratio has
    # to be measured in a textured world before choosing a lossy format.
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
             '-o', out, '--topics', *SIM_TOPICS, *PX4_TOPICS],
        output='screen',
    )
    return [bridge, record]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='default', description='Gazebo world name'),
        DeclareLaunchArgument(
            'out', default_value=f'data/bags/{datetime.now():%Y%m%d_%H%M%S}',
            description='output bag directory (must not exist)'),
        OpaqueFunction(function=_bridge_and_record),
    ])
