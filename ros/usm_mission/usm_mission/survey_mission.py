"""Fly a lawnmower survey with a nadir camera, then return home with PX4's RTL.

The plan comes from usm_core.coverage.lawnmower. The area is given in local NED
relative to the take-off point. Lanes are flown with a setpoint that moves along the
path at constant speed (position + velocity feed-forward), heading fixed north. At the
end the node logs how closely the plan was tracked.

    ros2 run usm_mission survey_mission --ros-args -p altitude:=30.0 -p speed:=5.0
"""
from enum import Enum, auto
import math

from px4_msgs.msg import GimbalDeviceAttitudeStatus, VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from usm_core.coverage import Area, Camera, lawnmower
from usm_mission.px4_io import PX4_QOS, Px4Commander

Point = tuple[float, float, float]


class State(Enum):
    WAIT_FOR_PX4 = auto()      # until status and a valid local position arrive
    STREAM_SETPOINTS = auto()  # PX4 only accepts offboard once setpoints already flow
    ENGAGE = auto()            # request offboard mode + arm, retry until confirmed
    CLIMB = auto()             # straight up to survey altitude
    POINT_GIMBAL = auto()      # camera to nadir, wait until the measured angle agrees
    SURVEY = auto()            # fly to the first waypoint, then the lanes
    RETURN = auto()            # PX4's RTL flies home and lands
    DONE = auto()


def off_nadir_deg(q) -> float:
    """Angle [deg] between the camera axis and straight down.

    q = (w, x, y, z) rotates the gimbal's FRD frame into NED; the camera looks along the
    gimbal's x axis, whose NED z component (row 2, column 0 of the rotation matrix) is
    2(xz - wy). Nadir: that component is 1, the angle 0.
    """
    w, x, y, z = q
    down = max(-1.0, min(1.0, 2.0 * (x * z - w * y)))
    return math.degrees(math.acos(down))


class MovingSetpoint:
    """A setpoint sliding along a polyline with a trapezoidal speed profile.

    On every segment it speeds up at `accel`, cruises at `speed` and slows down so that it
    stops exactly at the vertex: v = min(speed, v_prev + accel*dt, sqrt(2*accel*remaining)).
    A jump from standstill to full speed made the vehicle fall behind, sprint to catch up
    (9.3 m/s at 5 m/s) and tilt hard, which the gimbal could not fully follow.
    It stops at every vertex until the vehicle is within `accept_radius`, so each lane
    starts on its line rather than with the vehicle still cutting the previous corner.
    """

    def __init__(self, points: list[Point], speed: float, accel: float, accept_radius: float):
        self.points = points
        self.speed = speed
        self.accel = accel
        self.accept_radius = accept_radius
        self.segment = 0   # flying points[segment] -> points[segment + 1]
        self.along = 0.0   # distance of the setpoint from the segment start [m]
        self.v = 0.0       # current setpoint speed [m/s]

    def duration(self) -> float:
        """Time [s] the setpoint needs for the whole path, not counting waits at vertices."""
        total = 0.0
        ramp = self.speed ** 2 / self.accel  # distance to reach cruise speed and stop again
        for a, b in zip(self.points, self.points[1:]):
            length = math.dist(a, b)
            if length >= ramp:  # trapezoid: L/v cruising plus v/a lost to the two ramps
                total += length / self.speed + self.speed / self.accel
            else:  # triangle: never reaches cruise speed, peak sqrt(a*L)
                total += 2.0 * math.sqrt(length / self.accel)
        return total

    @property
    def finished(self) -> bool:
        return self.segment >= len(self.points) - 1

    def step(self, dt: float, vehicle: Point) -> tuple[Point, Point]:
        """Advance by dt [s]; returns (position, velocity feed-forward) in NED."""
        a, b = self.points[self.segment], self.points[self.segment + 1]
        length = math.dist(a, b)
        remaining = length - self.along
        self.v = min(self.speed, self.v + self.accel * dt, math.sqrt(2.0 * self.accel * remaining))
        self.along = min(self.along + self.v * dt, length)
        if self.along >= length:  # waiting at the vertex
            if math.dist(vehicle, b) < self.accept_radius:
                self.segment += 1
                self.along = 0.0
                self.v = 0.0
            return b, (0.0, 0.0, 0.0)
        u = [(bi - ai) / length for ai, bi in zip(a, b)]
        position = tuple(ai + ui * self.along for ai, ui in zip(a, u))
        return position, tuple(ui * self.v for ui in u)


class TrackStats:
    """Running mean and max of a sampled value."""

    def __init__(self):
        self.n = 0
        self.total = 0.0
        self.max = 0.0

    def add(self, value: float):
        self.n += 1
        self.total += value
        self.max = max(self.max, value)

    def __str__(self) -> str:
        if self.n == 0:
            return 'no samples'
        return f'mean {self.total / self.n:.2f}, max {self.max:.2f} (n={self.n})'


class SurveyMission(Node):
    RATE_HZ = 20.0
    PRESTREAM_S = 1.0
    RETRY_S = 1.0
    ENGAGE_TIMEOUT_S = 10.0
    GIMBAL_TIMEOUT_S = 10.0
    ALT_TOLERANCE_M = 0.3
    NADIR_TOLERANCE_DEG = 2.0
    ACCEPT_RADIUS_M = 1.0
    YAW = 0.0  # heading fixed north on every lane: the image keeps its orientation

    def __init__(self):
        super().__init__('survey_mission')
        p = self.declare_parameter
        self.altitude = p('altitude', 30.0).value
        self.speed = p('speed', 5.0).value
        self.accel = p('accel', 2.0).value  # setpoint acceleration [m/s^2]
        # Survey area [m], local NED relative to the take-off point
        area = Area(p('x_min', -30.0).value, p('x_max', 30.0).value,
                    p('y_min', -30.0).value, p('y_max', 30.0).value)
        side_overlap = p('side_overlap', 0.7).value
        front_overlap = p('front_overlap', 0.8).value
        edge_margin = p('edge_margin', -1.0).value  # < 0: lawnmower default (Gw/2)
        # Must match sim/gz_models/usm_gimbal
        camera = Camera(p('image_width', 1920).value, p('image_height', 1080).value,
                        math.radians(p('hfov_deg', 60.0).value))

        self.plan = lawnmower(area, camera, self.altitude, side_overlap, front_overlap,
                              None if edge_margin < 0 else edge_margin)
        self.get_logger().info(
            f'plan: {self.plan.n_lanes} lanes, spacing {self.plan.lane_spacing:.2f} m, '
            f'photo every {self.plan.photo_spacing:.2f} m, path {self.plan.path_length:.0f} m '
            f'(~{MovingSetpoint(self.plan.waypoints, self.speed, self.accel, 0.0).duration():.0f} s '
            f'at {self.speed} m/s, {self.accel} m/s^2)')

        self.px4 = Px4Commander(self)
        self.status: VehicleStatus | None = None
        self.position: VehicleLocalPosition | None = None
        self.gimbal_tilt: float | None = None  # measured off-nadir angle [deg]
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, PX4_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._on_position, PX4_QOS)
        self.create_subscription(
            GimbalDeviceAttitudeStatus, '/fmu/out/gimbal_device_attitude_status',
            self._on_gimbal, PX4_QOS)

        self.done = Future()
        self.home: Point | None = None
        self.target: Point | None = None
        self.velocity: Point | None = None
        self.path: MovingSetpoint | None = None
        self.cross_track = TrackStats()  # horizontal distance from the lane line [m]
        self.height_error = TrackStats()  # |z - survey z| on lanes [m]
        self.lane_speed = TrackStats()  # horizontal speed on lanes [m/s]
        self.tilt = TrackStats()  # gimbal off-nadir angle during the whole survey [deg]
        self.lane_tilt = TrackStats()  # ... on lanes only, where the images are used [deg]
        self.times: dict[State, float] = {}

        self.state = State.WAIT_FOR_PX4
        self.state_since = self._now()
        self.last_command = None
        self.last_step = self._now()
        self.create_timer(1.0 / self.RATE_HZ, self._step)

    # --- inputs ------------------------------------------------------------------------

    def _on_status(self, msg: VehicleStatus):
        self.status = msg

    def _on_position(self, msg: VehicleLocalPosition):
        self.position = msg

    def _on_gimbal(self, msg: GimbalDeviceAttitudeStatus):
        self.gimbal_tilt = off_nadir_deg(msg.q)

    @property
    def armed(self) -> bool:
        return self.status is not None and \
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def nav_state(self) -> int | None:
        return None if self.status is None else self.status.nav_state

    @property
    def vehicle(self) -> Point:
        return (self.position.x, self.position.y, self.position.z)

    # --- state machine helpers ---------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self.state_since

    def _enter(self, state: State):
        self.get_logger().info(f'{self.state.name} -> {state.name}')
        self.state = state
        self.state_since = self._now()
        self.times[state] = self.state_since
        self.last_command = None

    def _retry_due(self) -> bool:
        """True at most once per RETRY_S; used to resend commands PX4 may have dropped."""
        now = self._now()
        if self.last_command is None or now - self.last_command >= self.RETRY_S:
            self.last_command = now
            return True
        return False

    # --- main loop ---------------------------------------------------------------------

    def _step(self):
        now = self._now()
        dt, self.last_step = now - self.last_step, now

        # Keep offboard alive until we hand control back to PX4.
        if self.state not in (State.RETURN, State.DONE):
            self.px4.heartbeat()
            if self.target is not None:
                self.px4.position_setpoint(*self.target, self.YAW, self.velocity)

        # If PX4 (failsafe) or a pilot leaves offboard mid-flight, do not fight it.
        if self.state in (State.CLIMB, State.POINT_GIMBAL, State.SURVEY) and \
                self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn(f'PX4 left offboard (nav_state={self.nav_state}); stopping')
            self._finish()
            return

        if self.state == State.WAIT_FOR_PX4:
            p = self.position
            if self.status is not None and p is not None and p.xy_valid and p.z_valid:
                self.home = self.vehicle
                self.target = (p.x, p.y, p.z - self.altitude)
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
                self._enter(State.POINT_GIMBAL)

        elif self.state == State.POINT_GIMBAL:
            if self.gimbal_tilt is not None and self.gimbal_tilt < self.NADIR_TOLERANCE_DEG:
                self.get_logger().info(f'gimbal at nadir ({self.gimbal_tilt:.2f} deg off)')
                self._start_survey()
            elif self._elapsed() > self.GIMBAL_TIMEOUT_S:
                tilt = 'no gimbal status' if self.gimbal_tilt is None \
                    else f'{self.gimbal_tilt:.1f} deg off nadir'
                self.get_logger().error(f'gimbal did not reach nadir ({tilt}); returning')
                self._enter(State.RETURN)
            elif self._retry_due():
                self.px4.gimbal_take_control()
                self.px4.gimbal_pitch_yaw(-90.0, 0.0)

        elif self.state == State.SURVEY:
            self._sample()
            self.target, self.velocity = self.path.step(dt, self.vehicle)
            if self.path.finished:
                self.velocity = None
                self._enter(State.RETURN)

        elif self.state == State.RETURN:
            if not self.armed:  # PX4 disarms by itself after touchdown
                self.get_logger().info('landed and disarmed')
                self._report()
                self._finish()
            elif self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_RTL and self._retry_due():
                self.px4.return_to_launch()

    def _start_survey(self):
        hx, hy, hz = self.home
        z = hz - self.altitude
        # lawnmower plans relative to the take-off point; shift to local NED
        waypoints = [(hx + x, hy + y, z) for x, y, _ in self.plan.waypoints]
        self.path = MovingSetpoint([self.target] + waypoints, self.speed, self.accel,
                                   self.ACCEPT_RADIUS_M)
        self._enter(State.SURVEY)

    def _sample(self):
        """Tracking errors; lanes are the odd segments (even ones are transits)."""
        if self.gimbal_tilt is not None:
            self.tilt.add(self.gimbal_tilt)
        seg = self.path.segment
        if seg % 2 == 0 or self.path.finished:
            return
        a, b = self.path.points[seg], self.path.points[seg + 1]
        length = math.dist(a[:2], b[:2])
        if length == 0.0:
            return
        ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        dx, dy = self.position.x - a[0], self.position.y - a[1]
        if 0.0 <= dx * ux + dy * uy <= length:  # only alongside the lane, not past its ends
            self.cross_track.add(abs(dx * uy - dy * ux))
            self.height_error.add(abs(self.position.z - a[2]))
            self.lane_speed.add(math.hypot(self.position.vx, self.position.vy))
            if self.gimbal_tilt is not None:
                self.lane_tilt.add(self.gimbal_tilt)

    def _report(self):
        t = self.times
        survey_s = t.get(State.RETURN, math.nan) - t.get(State.SURVEY, math.nan)
        mission_s = t.get(State.DONE, self._now()) - t.get(State.CLIMB, math.nan)
        log = self.get_logger().info
        log(f'survey time {survey_s:.1f} s (setpoint profile: {self.path.duration():.1f} s), '
            f'mission time {mission_s:.1f} s')
        log(f'cross-track error [m]: {self.cross_track}')
        log(f'height error [m]: {self.height_error}')
        log(f'lane speed [m/s]: {self.lane_speed}')
        log(f'gimbal off-nadir [deg]: {self.tilt}')
        log(f'gimbal off-nadir on lanes [deg]: {self.lane_tilt}')

    def _finish(self):
        self._enter(State.DONE)
        if not self.done.done():
            self.done.set_result(True)


def main():
    rclpy.init()
    node = SurveyMission()
    try:
        rclpy.spin_until_future_complete(node, node.done)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
