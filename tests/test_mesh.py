"""Spec for usm_core.mesh, on tiny hand-written files."""
import math

import numpy as np
import pytest

from usm_core.mesh import apply_visual_pose, load_dae, load_mesh, load_obj

OBJ = """# a comment
v 1 2 3
vn 0 0 1
v 4 5 6
f 1 2 1
"""
DAE = """<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="0.5"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="g"><mesh>
      <source id="g-pos"><float_array id="a" count="6">1 0 0 0 1 0</float_array></source>
      <vertices id="g-v"><input semantic="POSITION" source="#g-pos"/></vertices>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="s">
    <node id="parent"><translate>10 0 0</translate>
      <node id="child"><scale>2 2 2</scale><instance_geometry url="#g"/></node>
    </node>
  </visual_scene></library_visual_scenes>
</COLLADA>
"""


def test_load_obj_reads_only_vertices(tmp_path):
    path = tmp_path / 'm.obj'
    path.write_text(OBJ)
    assert load_obj(path).tolist() == [[1, 2, 3], [4, 5, 6]]
    assert load_mesh(path).shape == (2, 3)


def test_load_dae_applies_nodes_and_unit(tmp_path):
    path = tmp_path / 'm.dae'
    path.write_text(DAE)
    # scale 2, then move 10 along x, then the file's unit of 0.5 m
    assert load_dae(path) == pytest.approx(np.array([[6, 0, 0], [5, 1, 0]]))


def test_visual_pose_scales_then_rotates_then_moves():
    out = apply_visual_pose([[1.0, 0.0, 0.0]], scale=2.0, translation=(0, 0, 3), yaw=math.pi / 2)
    assert out[0] == pytest.approx((0, 2, 3))


def test_visual_roll_turns_y_towards_z():
    assert apply_visual_pose([[0.0, 1.0, 0.0]], roll=math.pi / 2)[0] == pytest.approx((0, 0, 1))
