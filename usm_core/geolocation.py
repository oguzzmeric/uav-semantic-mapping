"""Pixel -> world: where on the ground a detection is.

Frames and conventions as in usm_core.projection: world is local NED [m], the camera quaternion
rotates its FRD frame (x along the optical axis, y image right, z image down) into NED.

The ground is taken as flat at d = 0. A box's centre shows an object's middle, not its base, so
the ray is cut with the plane at half the object's height rather than with the ground.
"""
import numpy as np

from usm_core.detection import PERSON, VEHICLE
from usm_core.projection import Intrinsics, rotation_matrix

# Assumed heights per class [m], as a real system would have them: not the true height of each
# object. Rounded means of the models in the village (people 1.67-1.92, vehicles 1.57-2.16).
CLASS_HEIGHTS = {PERSON: 1.8, VEHICLE: 1.8}


def pixel_ray(u: float, v: float, k: Intrinsics) -> np.ndarray:
    """The direction a pixel looks in, in the camera's FRD frame (1 along the optical axis)."""
    return np.array([1.0, (u - k.cx) / k.fx, (v - k.cy) / k.fy])


def intersect_plane(origin, direction, plane_d: float) -> np.ndarray | None:
    """Where a NED ray from origin meets the horizontal plane d = plane_d, or None if it never does."""
    origin, direction = np.asarray(origin, float), np.asarray(direction, float)
    if abs(direction[2]) < 1e-9:
        return None
    t = (plane_d - origin[2]) / direction[2]
    return origin + t * direction if t > 0 else None


def geolocate(u: float, v: float, camera_position, q, k: Intrinsics,
              object_height: float = 0.0) -> np.ndarray | None:
    """NED point at the middle height of an object seen at pixel (u, v); None above the horizon."""
    ray = rotation_matrix(q) @ pixel_ray(u, v, k)
    return intersect_plane(camera_position, ray, -object_height / 2)
