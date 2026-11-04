"""Spec for usm_core.detection: the class mapping, without the model or torch."""
import sys

import pytest

from usm_core.detection import (
    CLASS_MAP, PERSON, VEHICLE, Detection, iou, merge_tiles, project_class, tile_offsets,
)

# Class names stored in models/teknofest_yolo11l_nadir.pt, in index order.
MODEL_CLASSES = ['UAP', 'INSAN', 'MOTOR', 'ARABA', 'KAMYON', 'OTOBUS', 'UAI', 'MAKINE', 'GEMI', 'TREN']


def test_person_and_vehicle_classes():
    assert project_class('INSAN') == PERSON
    for name in ('ARABA', 'KAMYON', 'OTOBUS', 'MOTOR'):
        assert project_class(name) == VEHICLE


def test_other_classes_are_dropped():
    for name in ('UAP', 'UAI', 'MAKINE', 'GEMI', 'TREN'):
        assert project_class(name) is None


def test_map_only_names_model_classes():
    # a typo in CLASS_MAP would silently drop a class
    assert set(CLASS_MAP) <= set(MODEL_CLASSES)


def person(score, x1, y1=100.0, x2=None, y2=150.0):
    return Detection(PERSON, score, x1, y1, x1 + 50 if x2 is None else x2, y2)


def test_tile_offsets_of_a_1080p_frame():
    assert tile_offsets(1920) == [0, 640]


def test_tile_offsets_reach_the_right_edge():
    assert tile_offsets(2500) == [0, 640, 1220]
    assert tile_offsets(1000) == [0]


def test_iou():
    # 50 x 50 boxes shifted by 25: shared 25 x 50 = 1250, union 5000 - 1250 = 3750
    assert iou(person(1, 0, 100, 50, 150), person(1, 25, 100, 75, 150)) == pytest.approx(1 / 3)
    assert iou(person(1, 0), person(1, 100)) == 0


def test_merge_shifts_boxes_to_image_coordinates():
    merged = merge_tiles([(640, [person(0.9, 500)])], 1920, 0.7)
    assert (merged[0].x1, merged[0].x2) == (1140, 1190)


def test_merge_keeps_the_better_of_a_duplicate_across_tiles():
    # the same object in the overlap: x 700..750 in the image
    merged = merge_tiles([(0, [person(0.8, 700)]), (640, [person(0.9, 60)])], 1920, 0.7)
    assert [d.score for d in merged] == [0.9]


def test_merge_drops_boxes_cut_by_an_edge_between_tiles():
    cut_right = person(0.9, 1240, x2=1280)  # right edge of the left tile
    cut_left = person(0.9, 0, x2=40)        # left edge of the right tile
    assert merge_tiles([(0, [cut_right]), (640, [cut_left])], 1920, 0.7) == []


def test_merge_keeps_boxes_at_the_edges_of_the_image():
    at_image_left = person(0.9, 0, x2=40)
    at_image_right = person(0.8, 1240, x2=1280)
    merged = merge_tiles([(0, [at_image_left]), (640, [at_image_right])], 1920, 0.7)
    assert [(d.x1, d.x2) for d in merged] == [(0, 40), (1880, 1920)]


def test_merge_keeps_different_classes_on_the_same_spot():
    car = Detection(VEHICLE, 0.5, 700, 100, 750, 150)
    assert len(merge_tiles([(0, [person(0.9, 700), car])], 1920, 0.7)) == 2


def test_import_does_not_load_ultralytics():
    assert 'ultralytics' not in sys.modules
