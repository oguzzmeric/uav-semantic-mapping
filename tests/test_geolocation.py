"""Spec for usm_core.geolocation. Expected values are worked out by hand in the comments.

Reference camera: 1920x1080, HFOV 60 deg: fx = 960 / tan(30 deg) = 1662.768 px.
At 30 m straight down, 1 m on the ground is fx / 30 = 55.426 px.
"""
import math

import numpy as np
import pytest

from usm_core.geolocation import geolocate, intersect_plane, pixel_ray
from usm_core.projection import Intrinsics, project_points

FX = 960 / math.tan(math.radians(30))
K = Intrinsics(1920, 1080, FX, FX, 960.0, 540.0)
CAMERA = (0.0, 0.0, -30.0)
S = math.sqrt(0.5)
NADIR_NORTH = (S, 0.0, -S, 0.0)  # camera x = down, y = east, z = south
NADIR_EAST = (0.5, 0.5, -0.5, 0.5)  # the same, with the vehicle turned to the east
PX_PER_M = FX / 30


def test_pixel_ray_at_centre_is_the_optical_axis():
    assert pixel_ray(960, 540, K) == pytest.approx((1, 0, 0))


def test_pixel_ray_right_and_down_are_positive():
    assert pixel_ray(960 + FX, 540 - FX / 2, K) == pytest.approx((1, 1, -0.5))


def test_intersect_plane():
    assert intersect_plane((1, 2, -30), (0, 0.5, 1), 0) == pytest.approx((1, 17, 0))


def test_intersect_plane_rejects_rays_that_miss_it():
    assert intersect_plane((0, 0, -30), (1, 0, 0), 0) is None      # horizontal
    assert intersect_plane((0, 0, -30), (0, 0, -1), 0) is None     # pointing up


def test_image_centre_at_nadir_is_straight_below():
    assert geolocate(960, 540, CAMERA, NADIR_NORTH, K) == pytest.approx((0, 0, 0))


def test_nadir_height_error_does_not_move_the_centre_pixel():
    # straight below stays straight below, whatever height is assumed
    for height in (0.0, 1.8, 10.0):
        assert geolocate(960, 540, CAMERA, NADIR_NORTH, K, height)[:2] == pytest.approx((0, 0))


def test_ten_metres_east_and_north_heading_north():
    assert geolocate(960 + 10 * PX_PER_M, 540, CAMERA, NADIR_NORTH, K)[:2] == pytest.approx((0, 10))
    assert geolocate(960, 540 - 10 * PX_PER_M, CAMERA, NADIR_NORTH, K)[:2] == pytest.approx((10, 0))


def test_heading_east_turns_the_image():
    # image up is east, image right is south
    assert geolocate(960, 540 - 10 * PX_PER_M, CAMERA, NADIR_EAST, K)[:2] == pytest.approx((0, 10))
    assert geolocate(960 + 10 * PX_PER_M, 540, CAMERA, NADIR_EAST, K)[:2] == pytest.approx((-10, 0))


def test_camera_position_shifts_the_result():
    assert geolocate(960, 540, (5, -3, -30), NADIR_NORTH, K)[:2] == pytest.approx((5, -3))


def test_object_height_uses_the_plane_at_half_height():
    # seen at 572 px off centre from 30 m. On the ground: 572 / FX * 30 = 10.32 m.
    # A 2 m object's middle is 29 m below the camera: 572 / FX * 29 = 9.98 m.
    u = 960 + 572
    assert geolocate(u, 540, CAMERA, NADIR_NORTH, K)[1] == pytest.approx(572 / FX * 30)
    point = geolocate(u, 540, CAMERA, NADIR_NORTH, K, object_height=2.0)
    assert point[1] == pytest.approx(572 / FX * 29)
    assert point[2] == pytest.approx(-1)


def test_tilted_camera_centre_pixel_follows_tan():
    # pitched 10 deg forward of nadir, heading north: the axis meets the ground 30 tan(10) ahead
    angle = math.radians(-80 / 2)
    q = (math.cos(angle), 0.0, math.sin(angle), 0.0)
    assert geolocate(960, 540, CAMERA, q, K)[:2] == pytest.approx((30 * math.tan(math.radians(10)), 0))


def test_camera_looking_at_the_horizon_gives_nothing():
    assert geolocate(960, 540, CAMERA, (1.0, 0.0, 0.0, 0.0), K) is None


@pytest.mark.parametrize('q', [NADIR_NORTH, NADIR_EAST, (0.70, 0.03, -0.71, 0.05), (0.66, 0.26, -0.65, 0.27)])
@pytest.mark.parametrize('point', [(3.0, -7.0, -0.9), (-12.5, 4.0, -0.9), (0.0, 0.0, -0.9)])
def test_round_trip_with_the_projection(q, point):
    # world -> pixel -> world returns the point, on its own plane
    norm = np.linalg.norm(q)
    q = tuple(c / norm for c in q)
    u, v = project_points(np.array([point]), CAMERA, q, K)[0]
    assert geolocate(u, v, CAMERA, q, K, object_height=1.8) == pytest.approx(point)
