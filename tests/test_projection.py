"""Spec for usm_core.projection. Expected values are worked out by hand in the comments.

Reference camera: 1920x1080, HFOV 60 deg: fx = 960 / tan(30 deg) = 1662.768 px.
At 30 m straight down, 1 m on the ground is fx / 30 = 55.426 px.
"""
import math

import numpy as np
import pytest

from usm_core.projection import (
    Box3D, Intrinsics, box_corners, hidden_share, image_box, off_nadir_deg, place_on_ground,
    project_points, segment_hits_box,
)

FX = 960 / math.tan(math.radians(30))
K = Intrinsics(1920, 1080, FX, FX, 960.0, 540.0)
CAMERA = (0.0, 0.0, -30.0)
S = math.sqrt(0.5)
NADIR_NORTH = (S, 0.0, -S, 0.0)  # pitch -90 deg: camera x = down, y = east, z = south
NADIR_EAST = (0.5, 0.5, -0.5, 0.5)  # the same, after turning the vehicle 90 deg to the east
PX_PER_M = FX / 30


def pixel(point, q=NADIR_NORTH):
    return project_points(np.array([point], float), CAMERA, q, K)[0]


def test_point_below_camera_is_image_centre():
    assert pixel((0, 0, 0)) == pytest.approx((960, 540))


def test_nadir_heading_north_image_up_is_north_and_right_is_east():
    assert pixel((10, 0, 0)) == pytest.approx((960, 540 - 10 * PX_PER_M))
    assert pixel((0, 10, 0)) == pytest.approx((960 + 10 * PX_PER_M, 540))


def test_nadir_heading_east_image_up_is_east_and_right_is_south():
    assert pixel((0, 10, 0), NADIR_EAST) == pytest.approx((960, 540 - 10 * PX_PER_M))
    assert pixel((-10, 0, 0), NADIR_EAST) == pytest.approx((960 + 10 * PX_PER_M, 540))


def test_frame_edge_is_half_the_footprint():
    # half-width on the ground: 30 * tan(30 deg) = 17.3205 m -> right edge of the image
    assert pixel((0, 30 * math.tan(math.radians(30)), 0))[0] == pytest.approx(1920)


def test_higher_point_appears_further_from_centre():
    # 2 m tall, 10 m east: seen from 28 m instead of 30 m
    assert pixel((0, 10, -2))[0] == pytest.approx(960 + FX * 10 / 28)


def test_point_behind_camera_is_nan():
    assert np.isnan(pixel((0, 0, -40))).all()


def test_off_nadir():
    assert off_nadir_deg(NADIR_NORTH) == pytest.approx(0, abs=1e-6)
    assert off_nadir_deg((1.0, 0.0, 0.0, 0.0)) == pytest.approx(90)  # looking at the horizon


def test_box_corners_follow_yaw():
    corners = box_corners(Box3D(0, 0, 0, 0, length=4, width=2, height=1))
    assert corners[:, 0].max() == pytest.approx(2)  # length along north
    assert corners[:, 1].max() == pytest.approx(1)
    corners = box_corners(Box3D(0, 0, 0, 90, length=4, width=2, height=1))
    assert corners[:, 0].max() == pytest.approx(1)
    assert corners[:, 1].max() == pytest.approx(2)  # yaw 90: length along east
    assert corners[:, 2].min() == pytest.approx(-1)  # top is at negative down


def test_image_box_of_centred_car():
    # 4 x 2 m, 0 tall, straight below: 4 m north-south, 2 m east-west
    result = image_box(box_corners(Box3D(0, 0, 0, 0, 4, 2, 0.0)), CAMERA, NADIR_NORTH, K)
    box, share = result
    assert box == pytest.approx([960 - PX_PER_M, 540 - 2 * PX_PER_M, 960 + PX_PER_M, 540 + 2 * PX_PER_M])
    assert share == pytest.approx(1)


def test_image_box_clipped_at_the_edge_reports_share():
    # centred on the right edge of the frame: half of it is outside
    east = 30 * math.tan(math.radians(30))
    box, share = image_box(box_corners(Box3D(0, east, 0, 0, 2, 2, 0.0)), CAMERA, NADIR_NORTH, K)
    assert box[2] == pytest.approx(1920)
    assert share == pytest.approx(0.5)


def test_image_box_outside_the_frame_is_none():
    assert image_box(box_corners(Box3D(0, 100, 0, 0, 2, 2, 0.0)), CAMERA, NADIR_NORTH, K) is None


def test_place_on_ground_follows_heading():
    # heading north: model forward is north, model left is west, model up is negative down
    assert place_on_ground([[1, 0, 0], [0, 1, 0], [0, 0, 2]], 10, 20, 0, 0) == pytest.approx(
        np.array([[11, 20, 0], [10, 19, 0], [10, 20, -2]]))
    # heading east: model forward is east, model left is north
    assert place_on_ground([[1, 0, 0], [0, 1, 0]], 10, 20, 0, 90) == pytest.approx(
        np.array([[10, 21, 0], [11, 20, 0]]))


def test_segment_hits_box():
    house = Box3D(0, 0, 0, 0, 10, 10, 8)
    assert segment_hits_box((0, 0, -30), (0, 0, 0), house)
    assert not segment_hits_box((20, 0, -30), (20, 0, 0), house)
    assert not segment_hits_box((0, 0, -30), (0, 0, -9), house)  # stops above the roof


def test_hidden_share_under_a_roof_and_in_the_open():
    roof = [Box3D(0, 0, 3, 0, 10, 10, 1)]  # a roof high above the person's head
    person_below = Box3D(0, 0, 10, 0, 0.5, 0.5, 1.8)
    assert hidden_share(person_below, CAMERA, roof) == 1
    assert hidden_share(Box3D(30, 0, 0, 0, 0.5, 0.5, 1.8), CAMERA, roof) == 0
