"""Spec for usm_core.alignment, on known transforms."""
import math

import numpy as np
import pytest

from usm_core.alignment import umeyama

POINTS = np.array([[0, 0, 0], [10, 0, 0], [0, 5, 0], [0, 0, 3], [7, 2, 1], [-4, 6, 2]], float)


def yaw(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_identity():
    sim = umeyama(POINTS, POINTS)
    assert sim.scale == pytest.approx(1)
    assert sim.rotation == pytest.approx(np.eye(3))
    assert sim.translation == pytest.approx(np.zeros(3), abs=1e-9)


def test_recovers_a_known_similarity():
    rotation = yaw(0.7) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
    target = 2.5 * POINTS @ rotation.T + [3, -1, 20]
    sim = umeyama(POINTS, target)
    assert sim.scale == pytest.approx(2.5)
    assert sim.rotation == pytest.approx(rotation)
    assert sim.translation == pytest.approx((3, -1, 20))
    assert sim.apply(POINTS) == pytest.approx(target)


def test_scale_only():
    assert umeyama(POINTS, 3 * POINTS).scale == pytest.approx(3)


def test_never_returns_a_mirror():
    # the target is a mirror image of the source: the best proper rotation still has det +1
    sim = umeyama(POINTS, POINTS * [1, 1, -1])
    assert np.linalg.det(sim.rotation) == pytest.approx(1)


def test_noise_averages_out():
    rng = np.random.default_rng(0)
    source = rng.uniform(-30, 30, (300, 3))
    target = 1.5 * source + [1, 2, 3] + rng.normal(0, 0.1, source.shape)
    sim = umeyama(source, target)
    assert sim.scale == pytest.approx(1.5, abs=1e-3)
    assert sim.translation == pytest.approx((1, 2, 3), abs=0.05)
