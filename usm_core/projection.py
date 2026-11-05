"""World -> pixel projection, for ground-truth boxes in simulation.

The evaluation's side of the camera geometry, written separately from the pixel -> ground
code on purpose: each can then check the other.

Frames and conventions:
- World: local NED [m], x north, y east, z down.
- Camera orientation: quaternion (w, x, y, z) rotating the gimbal's FRD frame into NED. The
  camera looks along FRD x; image right is FRD y, image down is FRD z.
- Pixel: pinhole without distortion, u = cx + fx * y / x, v = cy + fy * z / x.
"""
from dataclasses import dataclass

import numpy as np


# Camera relative to the vehicle's base_link at nadir, in the vehicle's FRD frame [m]: from the
# poses in sim/gz_models (gimbal mount 0.26 up and yawed 180 deg; pitch joint 0.0412 forward,
# 0.162 down; camera 0.0328 from the joint along its axis, which then points down).
CAMERA_OFFSET_FRD = np.array([0.0412, 0.0, -0.26 + 0.162 + 0.0328])


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Box3D:
    """An upright box standing on the ground, in NED."""
    n: float
    e: float
    d: float        # base height, usually 0
    yaw_deg: float  # direction of the length axis, clockwise from north
    length: float
    width: float
    height: float


def rotation_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def off_nadir_deg(q) -> float:
    """Angle [deg] between the camera axis and straight down."""
    return float(np.degrees(np.arccos(np.clip(rotation_matrix(q)[2, 0], -1.0, 1.0))))


def box_corners(box: Box3D) -> np.ndarray:
    """The 8 corners, shape (8, 3); the 4 at the base first."""
    c, s = np.cos(np.radians(box.yaw_deg)), np.sin(np.radians(box.yaw_deg))
    corners = []
    for z in (box.d, box.d - box.height):
        for u, v in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            du, dv = u * box.length / 2, v * box.width / 2
            corners.append((box.n + c * du - s * dv, box.e + s * du + c * dv, z))
    return np.array(corners)


def project_points(points, camera_position, q, k: Intrinsics) -> np.ndarray:
    """Pixels (u, v) of NED points, shape (N, 2); NaN for points not in front of the camera."""
    frd = (np.asarray(points) - np.asarray(camera_position)) @ rotation_matrix(q)  # R^T p, row-wise
    with np.errstate(divide='ignore', invalid='ignore'):
        u = k.cx + k.fx * frd[:, 1] / frd[:, 0]
        v = k.cy + k.fy * frd[:, 2] / frd[:, 0]
    behind = frd[:, 0] <= 0
    u[behind] = np.nan
    v[behind] = np.nan
    return np.column_stack([u, v])


def place_on_ground(local, n: float, e: float, d: float, yaw_deg: float) -> np.ndarray:
    """Model frame (x forward, y left, z up) -> NED; yaw is x's heading, clockwise from north."""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    x, y, z = np.asarray(local, float).T
    return np.column_stack([n + c * x + s * y, e + s * x - c * y, d - z])


def image_box(points, camera_position, q, k: Intrinsics) -> tuple[np.ndarray, float] | None:
    """Image rectangle around NED points, clipped to the frame, and the share of it inside.

    Returns None if any point is behind the camera or nothing is inside the frame.
    """
    pixels = project_points(points, camera_position, q, k)
    if np.isnan(pixels).any():
        return None
    x1, y1 = pixels.min(axis=0)
    x2, y2 = pixels.max(axis=0)
    clipped = np.array([max(x1, 0.0), max(y1, 0.0), min(x2, k.width), min(y2, k.height)])
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        return None
    share = (clipped[2] - clipped[0]) * (clipped[3] - clipped[1]) / ((x2 - x1) * (y2 - y1))
    return clipped, float(share)


def segment_hits_box(start, end, box: Box3D) -> bool:
    """Does the straight line from start to end pass through the box? (slab test in its frame)"""
    c, s = np.cos(np.radians(box.yaw_deg)), np.sin(np.radians(box.yaw_deg))
    to_box = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    centre = np.array([box.n, box.e, box.d - box.height / 2])
    half = np.array([box.length, box.width, box.height]) / 2
    a = to_box @ (np.asarray(start, float) - centre)
    direction = to_box @ (np.asarray(end, float) - np.asarray(start, float))
    t_in, t_out = 0.0, 1.0
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if abs(a[axis]) > half[axis]:
                return False
            continue
        t1 = (-half[axis] - a[axis]) / direction[axis]
        t2 = (half[axis] - a[axis]) / direction[axis]
        t_in = max(t_in, min(t1, t2))
        t_out = min(t_out, max(t1, t2))
        if t_in > t_out:
            return False
    return True


def hidden_share(box: Box3D, camera_position, others: list[Box3D]) -> float:
    """Share of the box's top-face points whose line of sight is blocked by one of the others."""
    top = box_corners(box)[4:]
    centre = top.mean(axis=0)
    samples = [centre, *top, *((top + centre) / 2)]
    blocked = sum(any(segment_hits_box(camera_position, p, o) for o in others) for p in samples)
    return blocked / len(samples)
