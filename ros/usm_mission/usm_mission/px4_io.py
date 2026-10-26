"""Talking to PX4 over the uXRCE-DDS bridge (px4_msgs topics).

All positions are in PX4's local NED frame: x north, y east, z down [m].
"""
from px4_msgs.msg import (
    GimbalManagerSetAttitude, OffboardControlMode, TrajectorySetpoint, VehicleCommand,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# PX4 publishes best-effort; transient-local hands a late subscriber the last sample.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6  # src/modules/commander/px4_custom_mode.h
SOURCE_SYSTEM = 1     # sysid/compid we send commands as (same as the first SITL instance)
SOURCE_COMPONENT = 1
NAN = float('nan')


class Px4Commander:
    """Publishes offboard heartbeats, position setpoints and vehicle commands."""

    def __init__(self, node: Node):
        self._node = node
        self._mode_pub = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._setpoint_pub = node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._command_pub = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def _now_us(self) -> int:
        return self._node.get_clock().now().nanoseconds // 1000

    def heartbeat(self):
        """Offboard keep-alive; PX4 leaves offboard if it stops arriving at >= 2 Hz."""
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        self._mode_pub.publish(msg)

    def position_setpoint(self, x: float, y: float, z: float, yaw: float,
                          velocity: tuple[float, float, float] | None = None):
        """Fly to NED position [m] with heading yaw [rad]; NaN fields are left to PX4.

        `velocity` [m/s] is a feed-forward for a moving setpoint: the position controller
        adds it to its correction, so the vehicle keeps pace instead of lagging behind.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [NAN, NAN, NAN] if velocity is None else [float(v) for v in velocity]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = float(yaw)
        msg.yawspeed = NAN
        self._setpoint_pub.publish(msg)

    def command(self, command: int, param1: float = 0.0, param2: float = 0.0,
                param3: float = 0.0, param4: float = 0.0, param5: float = 0.0,
                param6: float = 0.0, param7: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1  # MAV_SYS_ID of the first SITL instance
        msg.target_component = 1
        msg.source_system = SOURCE_SYSTEM
        msg.source_component = SOURCE_COMPONENT
        msg.from_external = True
        self._command_pub.publish(msg)

    def set_offboard_mode(self):
        # param1 = 1: "custom mode" flag; param2: PX4 main mode
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def arm(self):
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land(self):
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_to_launch(self):
        self.command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def gimbal_take_control(self):
        """Become the gimbal's primary controller; PX4 denies pitch/yaw commands otherwise.

        param1/param2: sysid/compid given primary control; param3/param4 = -1 leave the
        secondary controller unchanged (src/modules/gimbal/input_mavlink.cpp).
        """
        self.command(VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                     SOURCE_SYSTEM, SOURCE_COMPONENT, -1.0, -1.0)

    def gimbal_pitch_yaw(self, pitch_deg: float, yaw_deg: float):
        """Point the gimbal: pitch -90 is nadir, yaw 0 is the vehicle's nose.

        Roll and pitch are locked to the horizon (the camera stays level while the vehicle
        tilts to accelerate); yaw follows the vehicle. param3/param4 are rate setpoints,
        NaN = not used.
        """
        flags = (GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
                 | GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_PITCH_LOCK)
        self.command(VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                     pitch_deg, yaw_deg, NAN, NAN, flags)
