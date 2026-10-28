"""Reading survey bags recorded by ros/usm_mission/launch/record.launch.py.

All stamps in the bag share the simulation clock (UXRCE_DDS_SYNCT=0), in seconds here:
camera and Gazebo messages carry header stamps, PX4 messages `timestamp(_sample)` in us.
Gazebo poses are ENU world / FLU body; PX4 and everything we produce are NED / FRD.
"""
import bisect
import math

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions

CAMERA_TOPIC = '/camera/image'
CAMERA_INFO_TOPIC = '/camera/camera_info'
TRUTH_TOPIC = '/gz/odometry'
POSITION_TOPIC = '/fmu/out/vehicle_local_position_v1'
ATTITUDE_TOPIC = '/fmu/out/vehicle_attitude'
GIMBAL_TOPIC = '/fmu/out/gimbal_device_attitude_status'
STATUS_TOPIC = '/fmu/out/vehicle_status_v1'

NAV_STATE_OFFBOARD = 14


def read(bag: str, topic: str, msg_type):
    """Yield deserialized messages of one topic, in recording order."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    reader.set_filter(StorageFilter(topics=[topic]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        yield deserialize_message(data, msg_type)


def header_time(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    """Rotation about the frame's third axis (z-y-x Euler yaw) of a unit quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def enu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """(east, north, up) -> (north, east, down)."""
    return y, x, -z


def enu_flu_yaw_to_ned_frd(yaw_enu: float) -> float:
    """Heading of a FLU body in ENU (from east, counter-clockwise) as a NED heading of the
    matching FRD body (from north, clockwise)."""
    return wrap_angle(math.pi / 2.0 - yaw_enu)


def quat_mul(p: tuple, q: tuple) -> tuple:
    """Hamilton product of (w, x, y, z) quaternions."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (pw * qw - px * qx - py * qy - pz * qz,
            pw * qx + px * qw + py * qz - pz * qy,
            pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw)


_Q_ENU_TO_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
_Q_FRD_TO_FLU = (0.0, -1.0, 0.0, 0.0)  # inverse of q_FLU_to_FRD = (0, 1, 0, 0)


def enu_flu_quat_to_ned_frd(q: tuple) -> tuple:
    """Orientation of a FLU body in ENU as the orientation of its FRD body in NED, (w, x, y, z).
    Same conversion as PX4's GZGimbal, so the truth matches the gimbal status convention."""
    return quat_mul(quat_mul(_Q_ENU_TO_NED, q), _Q_FRD_TO_FLU)


class Series:
    """Time-sorted samples with linear interpolation; angle columns interpolate on the circle."""

    def __init__(self, times: list[float], values: list[tuple], angle_columns: tuple[int, ...] = ()):
        order = sorted(range(len(times)), key=times.__getitem__)
        self.t = [times[i] for i in order]
        self.v = [values[i] for i in order]
        self.angle_columns = set(angle_columns)

    def at(self, t: float):
        """Value at time t, or None outside the sampled interval."""
        i = bisect.bisect_left(self.t, t)
        if i < len(self.t) and self.t[i] == t:
            return self.v[i]
        if i == 0 or i == len(self.t):
            return None
        a = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return self._blend(self.v[i - 1], self.v[i], a)

    def _blend(self, lo: tuple, hi: tuple, a: float) -> tuple:
        return tuple(
            wrap_angle(u + a * wrap_angle(w - u)) if k in self.angle_columns else u + a * (w - u)
            for k, (u, w) in enumerate(zip(lo, hi)))

    def last_before(self, t: float):
        """Most recent sample at or before t (for step-like data such as flight modes)."""
        i = bisect.bisect_right(self.t, t) - 1
        return None if i < 0 else self.v[i]


class QuaternionSeries(Series):
    """Unit (w, x, y, z) quaternions, interpolated by normalized lerp. Between samples a few
    ms apart it matches slerp to well below the attitude noise."""

    def _blend(self, lo: tuple, hi: tuple, a: float) -> tuple:
        if sum(u * w for u, w in zip(lo, hi)) < 0.0:  # q and -q are the same rotation
            hi = tuple(-w for w in hi)
        q = [u + a * (w - u) for u, w in zip(lo, hi)]
        norm = math.sqrt(sum(c * c for c in q))
        return tuple(c / norm for c in q)
