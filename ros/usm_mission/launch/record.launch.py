"""Record a survey to MCAP, with the camera compressed to JPEG.

Needs bridge.launch.py and the XRCE agent. Run from the repo root while the simulation is
up; stop with Ctrl+C after landing:

    ros2 launch usm_mission record.launch.py                 # -> data/bags/<YYYYmmdd_HHMMSS>
    ros2 launch usm_mission record.launch.py out:=data/bags/alt30
"""
from datetime import datetime
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# All stamps share the simulation clock (UXRCE_DDS_SYNCT=0 in the airframe).
PX4_TOPICS = [
    '/fmu/out/vehicle_local_position_v1',      # EKF2 position estimate (NED)
    '/fmu/out/vehicle_attitude',               # vehicle orientation
    '/fmu/out/gimbal_device_attitude_status',  # measured camera orientation
    '/fmu/out/vehicle_status_v1',              # arming state and flight mode
]
JPEG_QUALITY = 95  # 0.9 MB per 1080p frame in the village world, PSNR 38 dB (raw: 6.2 MB)

SIM_TOPICS = [
    '/camera/image/compressed',  # JPEG; the raw /camera/image is ~10 GB per survey
    '/camera/camera_info',       # intrinsics
    '/gz/odometry',         # true vehicle pose (ENU) and velocity, for evaluation only
    '/clock',
    '/rosout',              # mission log
    '/detections',          # live detector's boxes, if it is running
]


def _camera_is_bridged() -> bool:
    info = subprocess.run(['ros2', 'topic', 'info', '/camera/image'],
                          capture_output=True, text=True, timeout=20)
    return 'Publisher count: 0' not in info.stdout and info.returncode == 0


def _record(context):
    out = LaunchConfiguration('out').perform(context)

    jpeg = Node(
        package='image_transport',
        executable='republish',
        name='camera_jpeg',
        parameters=[{'in_transport': 'raw', 'out_transport': 'compressed',
                     'out.compressed.format': 'jpeg',
                     'out.compressed.jpeg_quality': JPEG_QUALITY}],
        # Remapping 'out' alone does not reach the transport's sub-topic 'out/compressed'.
        remappings=[('in', '/camera/image'), ('out/compressed', '/camera/image/compressed')],
        output='screen',
    )
    # zstd on top of JPEG only helps the small topics.
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
             '-o', out, '--topics', *SIM_TOPICS, *PX4_TOPICS],
        output='screen',
    )
    actions = [jpeg, record]
    if not _camera_is_bridged():  # the bag would stay silently empty of camera frames
        actions.insert(0, LogInfo(msg='WARNING: /camera/image has no publisher. '
                                      'Is bridge.launch.py running?'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'out', default_value=f'data/bags/{datetime.now():%Y%m%d_%H%M%S}',
            description='output bag directory (must not exist)'),
        OpaqueFunction(function=_record),
    ])
