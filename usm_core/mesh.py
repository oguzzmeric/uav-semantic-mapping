"""Vertices of the simulated objects' meshes, for ground-truth boxes.

Reads only what a bounding box needs: vertex positions, with the Collada node transforms and
unit applied. Skeletons and materials are ignored.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

_COLLADA = '{http://www.collada.org/2005/11/COLLADASchema}'


def load_obj(path) -> np.ndarray:
    """Vertices of a Wavefront OBJ file, shape (N, 3)."""
    with open(path) as f:
        return np.array([line.split()[1:4] for line in f if line.startswith('v ')], float)


def _node_matrix(node) -> np.ndarray:
    m = np.eye(4)
    for element in node:
        tag = element.tag.replace(_COLLADA, '')
        values = [float(v) for v in (element.text or '').split()]
        if tag == 'matrix':
            m = m @ np.array(values).reshape(4, 4)
        elif tag == 'translate':
            t = np.eye(4)
            t[:3, 3] = values
            m = m @ t
        elif tag == 'scale':
            m = m @ np.diag([*values, 1.0])
        elif tag == 'rotate':
            x, y, z = values[:3]
            a = np.radians(values[3])
            k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            r = np.eye(4)
            r[:3, :3] = np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k
            m = m @ r
    return m


def load_dae(path) -> np.ndarray:
    """Vertices of a Collada file in metres, Z up as authored, shape (N, 3)."""
    root = ET.parse(path).getroot()
    unit_element = root.find(f'{_COLLADA}asset/{_COLLADA}unit')
    unit = float(unit_element.get('meter', 1.0)) if unit_element is not None else 1.0

    geometries = {}
    for geometry in root.iter(f'{_COLLADA}geometry'):
        mesh = geometry.find(f'{_COLLADA}mesh')
        if mesh is None:
            continue
        source_id = mesh.find(f'{_COLLADA}vertices/{_COLLADA}input[@semantic="POSITION"]') \
            .get('source')[1:]
        for source in mesh.findall(f'{_COLLADA}source'):
            if source.get('id') == source_id:
                values = source.find(f'{_COLLADA}float_array').text.split()
                geometries[geometry.get('id')] = np.array(values, float).reshape(-1, 3)

    placed = []

    def walk(node, parent):
        m = parent @ _node_matrix(node)
        for instance in node.findall(f'{_COLLADA}instance_geometry'):
            placed.append(geometries[instance.get('url')[1:]] @ m[:3, :3].T + m[:3, 3])
        for child in node.findall(f'{_COLLADA}node'):
            walk(child, m)

    for scene in root.iter(f'{_COLLADA}visual_scene'):
        for node in scene.findall(f'{_COLLADA}node'):
            walk(node, np.eye(4))
    return np.vstack(placed) * unit


def load_mesh(path) -> np.ndarray:
    return load_dae(path) if Path(path).suffix.lower() == '.dae' else load_obj(path)


def apply_visual_pose(vertices, scale: float = 1.0, translation=(0.0, 0.0, 0.0),
                      roll: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    """Mesh frame -> model frame, as an SDF <visual> does it: scale, then rotate, then move."""
    cr, sr = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) \
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return (np.asarray(vertices) * scale) @ rotation.T + np.asarray(translation)
