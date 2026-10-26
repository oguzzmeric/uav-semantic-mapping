"""Arm, climb to a fixed altitude in offboard mode, hover, then land with PX4's NAV_LAND.

First flight-control node of the project: proves the ROS 2 -> PX4 offboard path.

    ros2 run usm_mission offboard_hover --ros-args -p altitude:=5.0 -p hover_time:=10.0
"""
from enum import Enum, auto

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from usm_mission.px4_io import PX4_QOS, Px4Commander


class State(Enum):
    WAIT_FOR_PX4 = auto()      # until status and a valid local position arrive
    STREAM_SETPOINTS = auto()  # PX4 only accepts offboard once setpoints already flow
    ENGAGE = auto()            # request offboard mode + arm, retry until confirmed
    CLIMB = auto()
    HOVER = auto()
    LAND = auto()              # PX4's own land mode takes over
    DONE = auto()


class OffboardHover(Node):
    RATE_HZ = 10.0
    PRESTREAM_S = 1.0
    RETRY_S = 1.0
    ENGAGE_TIMEOUT_S = 10.0
    ALT_TOLERANCE_M = 0.3

    def __init__(self):
        super().__init__('offboard_hover')
        self.altitude = self.declare_parameter('altitude', 5.0).value
        self.hover_time = self.declare_parameter('hover_time', 10.0).value

        self.px4 = Px4Commander(self)
        self.status: VehicleStatus | None = None
        self.position: VehicleLocalPosition | None = None
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, PX4_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._on_position, PX4_QOS)

        self.done = Future()
        self.target = None  # (x, y, z, yaw) in NED
        self.state = State.WAIT_FOR_PX4
        self.state_since = self.get_clock().now()
        self.last_command = None
        self.create_timer(1.0 / self.RATE_HZ, self._step)

    def _on_status(self, msg: VehicleStatus):
        self.status = msg

    def _on_position(self, msg: VehicleLocalPosition):
        self.position = msg

    @property
    def armed(self) -> bool:
        return self.status is not None and \
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def nav_state(self) -> int | None:
        return None if self.status is None else self.status.nav_state

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_since).nanoseconds * 1e-9

    def _enter(self, state: State):
        self.get_logger().info(f'{self.state.name} -> {state.name}')
        self.state = state
        self.state_since = self.get_clock().now()
        self.last_command = None

    def _retry_due(self) -> bool:
        """True at most once per RETRY_S; used to resend commands PX4 may have dropped."""
        now = self.get_clock().now()
        if self.last_command is None or \
                (now - self.last_command).nanoseconds * 1e-9 >= self.RETRY_S:
            self.last_command = now
            return True
        return False

    def _step(self):
        # Keep offboard alive until we hand control back to PX4.
        if self.state not in (State.LAND, State.DONE):
            self.px4.heartbeat()
            if self.target is not None:
                self.px4.position_setpoint(*self.target)

        # If PX4 (failsafe) or a pilot leaves offboard mid-flight, do not fight it.
        if self.state in (State.CLIMB, State.HOVER) and \
                self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn(f'PX4 left offboard (nav_state={self.nav_state}); stopping')
            self._finish()
            return

        if self.state == State.WAIT_FOR_PX4:
            p = self.position
            if self.status is not None and p is not None and p.xy_valid and p.z_valid:
                # Straight above the current spot, keeping the current heading.
                self.target = (p.x, p.y, p.z - self.altitude, p.heading)
                self.get_logger().info(
                    f'target NED x={p.x:.2f} y={p.y:.2f} z={p.z - self.altitude:.2f}')
                self._enter(State.STREAM_SETPOINTS)

        elif self.state == State.STREAM_SETPOINTS:
            if self._elapsed() >= self.PRESTREAM_S:
                self._enter(State.ENGAGE)

        elif self.state == State.ENGAGE:
            if self.armed and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._enter(State.CLIMB)
            elif self._elapsed() > self.ENGAGE_TIMEOUT_S:
                self.get_logger().error(
                    'PX4 did not accept offboard + arm (check `commander check` in pxh)')
                self._finish()
            elif self._retry_due():
                self.px4.set_offboard_mode()
                self.px4.arm()

        elif self.state == State.CLIMB:
            if abs(self.position.z - self.target[2]) < self.ALT_TOLERANCE_M:
                self.get_logger().info(f'reached z={self.position.z:.2f} m (NED)')
                self._enter(State.HOVER)

        elif self.state == State.HOVER:
            if self._elapsed() >= self.hover_time:
                self._enter(State.LAND)

        elif self.state == State.LAND:
            if not self.armed:  # PX4 disarms by itself after touchdown
                self.get_logger().info('landed and disarmed')
                self._finish()
            elif self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND and self._retry_due():
                self.px4.land()

    def _finish(self):
        self._enter(State.DONE)
        if not self.done.done():
            self.done.set_result(True)


def main():
    rclpy.init()
    node = OffboardHover()
    try:
        rclpy.spin_until_future_complete(node, node.done)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
