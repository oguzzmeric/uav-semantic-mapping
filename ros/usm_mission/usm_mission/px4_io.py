"""Talking to PX4 over the uXRCE-DDS bridge (px4_msgs topics).

All positions are in PX4's local NED frame: x north, y east, z down [m].
"""
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
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

    def position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Fly to NED position [m] with heading yaw [rad]; NaN fields are left to PX4."""
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = float(yaw)
        msg.yawspeed = NAN
        self._setpoint_pub.publish(msg)

    def command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1  # MAV_SYS_ID of the first SITL instance
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
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
