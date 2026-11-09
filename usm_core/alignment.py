"""Similarity alignment: the scale, rotation and shift that put one point set onto another.

A map built from photos alone has no metres and no north. Given the same cameras in the map and
in the world, Umeyama's closed-form least-squares solution (1991) gives y = s R x + t.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Sim3:
    scale: float
    rotation: np.ndarray  # 3 x 3
    translation: np.ndarray

    def apply(self, points) -> np.ndarray:
        """Transform points, shape (N, 3)."""
        return self.scale * np.asarray(points, float) @ self.rotation.T + self.translation


def umeyama(source, target) -> Sim3:
    """The similarity minimising the squared distances from source to target; shapes (N, 3), N >= 3."""
    source, target = np.asarray(source, float), np.asarray(target, float)
    source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
    a, b = source - source_mean, target - target_mean

    # Rotation from the cross-covariance; the sign keeps it a rotation rather than a reflection.
    u, d, vt = np.linalg.svd(b.T @ a / len(source))
    sign = np.diag([1.0, 1.0, np.sign(np.linalg.det(u) * np.linalg.det(vt))])
    rotation = u @ sign @ vt
    scale = float(np.trace(np.diag(d) @ sign) / (a * a).sum(axis=1).mean())
    return Sim3(scale, rotation, target_mean - scale * rotation @ source_mean)


def robust_umeyama(source, target, rounds: int = 6, spread: float = 3.0,
                   floor: float = 0.5) -> tuple[Sim3, np.ndarray]:
    """Umeyama that ignores outliers; returns the fit and a mask of the points it kept.

    Least squares lets a few wrong points pull the whole fit, so it is repeated on the points
    within `spread` times the median distance of the last fit (never under `floor`, in target units).
    """
    source, target = np.asarray(source, float), np.asarray(target, float)
    keep = np.ones(len(source), bool)
    for _ in range(rounds):
        sim = umeyama(source[keep], target[keep])
        distances = np.linalg.norm(sim.apply(source) - target, axis=1)
        keep = distances < max(spread * np.median(distances), floor)
    return sim, keep
