"""Spec for usm_core.detection: the class mapping, without the model or torch."""
import sys

from usm_core.detection import CLASS_MAP, PERSON, VEHICLE, project_class

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


def test_import_does_not_load_ultralytics():
    assert 'ultralytics' not in sys.modules
