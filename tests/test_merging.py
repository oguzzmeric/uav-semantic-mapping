"""Spec for usm_core.merging and usm_core.metrics."""
import numpy as np
import pytest

from usm_core.merging import ObjectMerger, touches_frame_edge
from usm_core.metrics import score_objects


def merged(points, radius=1.0, min_frames=1):
    merger = ObjectMerger(radius)
    for i, (cls, n, e) in enumerate(points):
        merger.add(cls, n, e, frame=i, score=0.5)
    return merger.objects(min_frames)


def test_near_points_become_one_object():
    objects = merged([('person', 10.0, 0.0), ('person', 10.2, 0.0), ('person', 9.9, 0.0)])
    assert len(objects) == 1
    assert objects[0].position() == pytest.approx((10.0, 0.0))


def test_far_points_stay_apart():
    assert len(merged([('person', 10.0, 0.0), ('person', 30.0, 0.0)])) == 2


def test_classes_never_merge():
    assert len(merged([('person', 10.0, 0.0), ('vehicle', 10.0, 0.0)])) == 2


def test_a_point_joins_the_nearest_cluster():
    objects = merged([('person', 0.0, 0.0), ('person', 3.0, 0.0), ('person', 2.5, 0.0)], radius=2.0)
    # 2.5 is within 2 m... of the cluster at 3.0 only; the cluster at 0.0 keeps one point
    assert sorted(len(o.points) for o in objects) == [1, 2]


def test_median_position_ignores_one_bad_point():
    objects = merged([('person', 10.0, 0.0), ('person', 10.0, 0.0), ('person', 10.0, 0.0),
                      ('person', 10.0, 0.0), ('person', 10.9, 0.0)])
    assert objects[0].position() == pytest.approx((10.0, 0.0))


def test_min_frames_counts_different_frames():
    merger = ObjectMerger(1.0)
    merger.add('person', 0.0, 0.0, frame=7, score=0.5)
    merger.add('person', 0.1, 0.0, frame=7, score=0.9)  # a second box in the same frame
    assert merger.objects(min_frames=2) == []
    merger.add('person', 0.1, 0.0, frame=8, score=0.6)
    objects = merger.objects(min_frames=2)
    assert len(objects) == 1
    assert objects[0].best_score == 0.9


def test_touches_frame_edge():
    assert touches_frame_edge([0, 500, 50, 50], 1920, 1080)
    assert touches_frame_edge([500, 1, 50, 50], 1920, 1080)
    assert touches_frame_edge([1870, 500, 50, 50], 1920, 1080)
    assert touches_frame_edge([500, 1030, 50, 50], 1920, 1080)
    assert not touches_frame_edge([500, 500, 50, 50], 1920, 1080)


def test_score_counts_matches_misses_and_false_alarms():
    score = score_objects([(0.0, 0.0), (10.0, 1.0), (50.0, 50.0)], [(0.5, 0.0), (10.0, 0.0), (90.0, 90.0)])
    assert (score.truths, score.reports, score.matched) == (3, 3, 2)
    assert score.recall == pytest.approx(2 / 3)
    assert score.precision == pytest.approx(2 / 3)
    assert sorted(score.errors) == pytest.approx([0.5, 1.0])
    assert score.missed == [2]
    assert score.false_alarms == [2]


def test_score_limit_is_inclusive_and_strict_beyond():
    assert score_objects([(2.0, 0.0)], [(0.0, 0.0)]).matched == 1
    assert score_objects([(2.01, 0.0)], [(0.0, 0.0)]).matched == 0


def test_two_reports_of_one_object_count_once():
    score = score_objects([(0.1, 0.0), (0.2, 0.0)], [(0.0, 0.0)])
    assert score.matched == 1
    assert score.errors == pytest.approx([0.1])
    assert score.false_alarms == [1]


def test_pairing_finds_the_assignment_that_matches_most():
    # a greedy nearest-first pairing would give the first report to truth 1 and match only one
    score = score_objects([(1.0, 0.0), (2.5, 0.0)], [(0.0, 0.0), (1.5, 0.0)])
    assert score.matched == 2


def test_score_with_nothing_on_one_side():
    assert score_objects([], [(0.0, 0.0)]).missed == [0]
    assert score_objects([(0.0, 0.0)], []).false_alarms == [0]
    assert score_objects(np.empty((0, 2)), np.empty((0, 2))).matched == 0
