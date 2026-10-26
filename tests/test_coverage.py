"""Spec for usm_core.coverage. Expected values are worked out by hand in the comments.

Reference camera: 1920x1080, HFOV 60 deg, at 30 m:
  Gw = 2 * 30 * tan(30 deg)     = 34.641016 m   (across-track)
  Gh = Gw * 1080 / 1920         = 19.485572 m   (along-track)
  GSD = Gw / 1920               = 0.0180422 m/px
  lane spacing @ 70 % side      = 0.3 * Gw = 10.392305 m
  photo spacing @ 80 % front    = 0.2 * Gh = 3.897114 m
"""
import math

import pytest

from usm_core.coverage import (
    Area, Camera, ground_footprint, gsd, lane_spacing, lawnmower, photo_spacing,
)

CAM = Camera(width_px=1920, height_px=1080, hfov_rad=math.radians(60))
H = 30.0
GW = 34.641016
GH = 19.485572
TOL = 1e-4


def assert_waypoints_close(got, want):
    # pytest.approx does not apply its tolerance inside nested tuples, so compare per point
    assert len(got) == len(want)
    for g, w in zip(got, want):
        assert g == pytest.approx(w, abs=TOL)


# --- footprint, GSD, spacings ---------------------------------------------------------

def test_footprint_reference_camera():
    gw, gh = ground_footprint(CAM, H)
    assert gw == pytest.approx(GW, abs=TOL)
    assert gh == pytest.approx(GH, abs=TOL)


def test_footprint_90deg_hfov_is_twice_the_altitude():
    # tan(45 deg) = 1, so the footprint width is exactly 2h
    gw, gh = ground_footprint(Camera(1000, 500, math.radians(90)), 10.0)
    assert gw == pytest.approx(20.0, abs=TOL)
    assert gh == pytest.approx(10.0, abs=TOL)  # keeps the 2:1 image aspect


def test_footprint_scales_linearly_with_altitude():
    gw30, gh30 = ground_footprint(CAM, 30.0)
    gw60, gh60 = ground_footprint(CAM, 60.0)
    assert gw60 == pytest.approx(2 * gw30, abs=TOL)
    assert gh60 == pytest.approx(2 * gh30, abs=TOL)


def test_gsd_reference_camera():
    assert gsd(CAM, H) == pytest.approx(0.0180422, abs=1e-7)


def test_spacings_reference_camera():
    assert lane_spacing(CAM, H, 0.7) == pytest.approx(10.392305, abs=TOL)
    assert photo_spacing(CAM, H, 0.8) == pytest.approx(3.897114, abs=TOL)


def test_zero_overlap_spacing_equals_footprint():
    assert lane_spacing(CAM, H, 0.0) == pytest.approx(GW, abs=TOL)
    assert photo_spacing(CAM, H, 0.0) == pytest.approx(GH, abs=TOL)


@pytest.mark.parametrize('overlap', [-0.1, 1.0, 1.5])
def test_overlap_out_of_range_raises(overlap):
    with pytest.raises(ValueError):
        lane_spacing(CAM, H, overlap)
    with pytest.raises(ValueError):
        photo_spacing(CAM, H, overlap)


@pytest.mark.parametrize('altitude', [0.0, -5.0])
def test_non_positive_altitude_raises(altitude):
    with pytest.raises(ValueError):
        ground_footprint(CAM, altitude)


# --- lawnmower: rule (i), edge_margin = 0 --------------------------------------------
# 60 x 60 m area. Lanes must keep the footprint inside: first lane at y = Gw/2.
#   room for lanes  = 60 - Gw = 25.358984
#   n = ceil(25.358984 / 10.392305) + 1 = ceil(2.440) + 1 = 4
#   actual spacing  = 25.358984 / 3 = 8.452995
#   lanes y         = 17.3205, 25.7735, 34.2265, 42.6795
#   lane ends x     = Gh/2 = 9.742786 ... 60 - Gh/2 = 50.257214
#   path            = 4 * 40.514428 + 3 * 8.452995 = 187.4167

AREA_60 = Area(x_min=0.0, x_max=60.0, y_min=0.0, y_max=60.0)


def test_rule_i_lanes_and_spacing():
    plan = lawnmower(AREA_60, CAM, H, 0.7, 0.8, edge_margin=0.0)
    assert plan.n_lanes == 4
    assert plan.lane_spacing == pytest.approx(8.452995, abs=TOL)
    assert plan.photo_spacing == pytest.approx(3.897114, abs=TOL)
    assert plan.path_length == pytest.approx(187.4167, abs=1e-3)


def test_rule_i_waypoints_in_flight_order():
    plan = lawnmower(AREA_60, CAM, H, 0.7, 0.8, edge_margin=0.0)
    lo, hi = 9.742786, 50.257214
    expected = [
        (lo, 17.320508, -H), (hi, 17.320508, -H),  # lane 0: northbound
        (hi, 25.773503, -H), (lo, 25.773503, -H),  # lane 1: southbound
        (lo, 34.226497, -H), (hi, 34.226497, -H),  # lane 2: northbound
        (hi, 42.679492, -H), (lo, 42.679492, -H),  # lane 3: southbound
    ]
    assert_waypoints_close(plan.waypoints, expected)


# --- lawnmower: rule (ii), default edge_margin = Gw/2 --------------------------------
# Area grown by Gw/2 = 17.320508 on every side -> y in [-17.32, 77.32], width 94.641016
#   room for lanes  = 94.641016 - Gw = 60.0
#   n = ceil(60 / 10.392305) + 1 = ceil(5.774) + 1 = 7, spacing = 60 / 6 = 10.0
#   lanes y         = 0, 10, ..., 60  (first and last exactly on the area boundary)
#   lane ends x     = -17.320508 + Gh/2 = -7.577722 ... 77.320508 - Gh/2 = 67.577722
#   path            = 7 * 75.155444 + 6 * 10 = 586.0881

def test_rule_ii_is_the_default():
    default = lawnmower(AREA_60, CAM, H, 0.7, 0.8)
    explicit = lawnmower(AREA_60, CAM, H, 0.7, 0.8, edge_margin=GW / 2)
    assert_waypoints_close(default.waypoints, explicit.waypoints)


def test_rule_ii_lanes_on_area_boundary():
    plan = lawnmower(AREA_60, CAM, H, 0.7, 0.8)
    assert plan.n_lanes == 7
    assert plan.lane_spacing == pytest.approx(10.0, abs=TOL)
    lane_ys = [wp[1] for wp in plan.waypoints[::2]]
    assert lane_ys == pytest.approx([0, 10, 20, 30, 40, 50, 60], abs=TOL)
    assert plan.waypoints[0] == pytest.approx((-7.577722, 0.0, -H), abs=TOL)
    assert plan.waypoints[1] == pytest.approx((67.577722, 0.0, -H), abs=TOL)
    assert plan.path_length == pytest.approx(586.0881, abs=1e-3)


# --- lawnmower: degenerate areas -----------------------------------------------------

def test_area_narrower_than_footprint_gives_one_central_lane():
    narrow = Area(x_min=0.0, x_max=60.0, y_min=0.0, y_max=20.0)  # 20 m < Gw
    plan = lawnmower(narrow, CAM, H, 0.7, 0.8, edge_margin=0.0)
    assert plan.n_lanes == 1
    assert plan.lane_spacing == 0.0
    assert_waypoints_close(plan.waypoints, [(9.742786, 10.0, -H), (50.257214, 10.0, -H)])
    assert plan.path_length == pytest.approx(40.514428, abs=TOL)


def test_area_shorter_than_footprint_puts_lane_ends_in_the_middle():
    short = Area(x_min=0.0, x_max=10.0, y_min=0.0, y_max=60.0)  # 10 m < Gh
    plan = lawnmower(short, CAM, H, 0.7, 0.8, edge_margin=0.0)
    assert plan.n_lanes == 4
    assert all(wp[0] == pytest.approx(5.0, abs=TOL) for wp in plan.waypoints)
    assert plan.path_length == pytest.approx(3 * 8.452995, abs=1e-3)


def test_exact_lane_count_is_not_rounded_up_by_float_noise():
    # 90 deg HFOV at 10 m: Gw = 20, side 50 % -> max spacing 10. On a 40 m area the
    # room is 40 - 20 = 20 = exactly 2 gaps -> 3 lanes. tan(45 deg) is 0.9999999999999999
    # in floating point, so the ratio comes out as 2.0000000000000004 and a bare ceil()
    # adds a needless 4th lane.
    cam = Camera(1000, 500, math.radians(90))
    plan = lawnmower(Area(0.0, 40.0, 0.0, 40.0), cam, 10.0, 0.5, 0.5, edge_margin=0.0)
    assert plan.n_lanes == 3
    assert plan.lane_spacing == pytest.approx(10.0, abs=TOL)
    assert plan.path_length == pytest.approx(110.0, abs=TOL)  # 3 * 30 + 2 * 10


# --- lawnmower: invariants on a realistic mission ------------------------------------

def test_invariants_150m_mission():
    area = Area(x_min=-75.0, x_max=75.0, y_min=-75.0, y_max=75.0)
    side, front = 0.7, 0.8
    plan = lawnmower(area, CAM, H, side, front)
    wps = plan.waypoints

    assert len(wps) == 2 * plan.n_lanes
    assert all(z == pytest.approx(-H) for _, _, z in wps)
    # spacing is spread evenly and never wider than requested
    assert plan.lane_spacing <= lane_spacing(CAM, H, side) + 1e-9
    lane_ys = [wp[1] for wp in wps[::2]]
    gaps = [b - a for a, b in zip(lane_ys, lane_ys[1:])]
    assert gaps == pytest.approx([plan.lane_spacing] * len(gaps), abs=TOL)
    # each lane is a straight north/south line, directions alternate
    for i in range(plan.n_lanes):
        start, end = wps[2 * i], wps[2 * i + 1]
        assert start[1] == pytest.approx(end[1])
        assert (end[0] > start[0]) == (i % 2 == 0)
    # path length is the polyline length through the waypoints
    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(wps, wps[1:]))
    assert plan.path_length == pytest.approx(length, abs=1e-6)


# --- lawnmower: input validation -----------------------------------------------------

@pytest.mark.parametrize('area', [
    Area(x_min=10.0, x_max=10.0, y_min=0.0, y_max=60.0),
    Area(x_min=0.0, x_max=60.0, y_min=5.0, y_max=-5.0),
])
def test_empty_area_raises(area):
    with pytest.raises(ValueError):
        lawnmower(area, CAM, H, 0.7, 0.8)


def test_negative_edge_margin_raises():
    with pytest.raises(ValueError):
        lawnmower(AREA_60, CAM, H, 0.7, 0.8, edge_margin=-1.0)
