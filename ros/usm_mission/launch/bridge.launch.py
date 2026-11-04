"""Bridge the Gazebo camera, true vehicle pose and clock into ROS 2.

Everything that needs them (recording, live detection) relies on this one bridge: a second
one would publish every frame twice. Run it while the simulation is up:

    ros2 launch usm_mission bridge.launch.py world:=village
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLE = 'usm_x500_gimbal_0'  # model name PX4 spawns: <PX4_SIM_MODEL without gz_>_<instance>


def _bridge(context):
    world = LaunchConfiguration('world').perform(context)
    camera = f'/world/{world}/model/{VEHICLE}/link/camera_link/sensor/camera'
    odometry = f'/model/{VEHICLE}/odometry'  # OdometryPublisher in usm_x500_gimbal

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[  # gz -> ROS only ('[')
            f'{camera}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{camera}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'{odometry}@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            (f'{camera}/image', '/camera/image'),
            (f'{camera}/camera_info', '/camera/camera_info'),
            (odometry, '/gz/odometry'),
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='default', description='Gazebo world name'),
        OpaqueFunction(function=_bridge),
    ])
