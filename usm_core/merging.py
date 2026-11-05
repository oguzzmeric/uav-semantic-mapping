"""Merge the ground points of many frames into one record per object.

Points are taken one at a time, in flight order, so the same code can run live: each joins the
nearest cluster of its class within a radius, or opens a new one. A cluster is compared by the
mean of its points, which is cheap to update; the reported position is their median, which one
bad point cannot pull away.
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Cluster:
    cls: str
    points: list[tuple[float, float]] = field(default_factory=list)
    frames: set[int] = field(default_factory=set)
    best_score: float = 0.0
    _sum_n: float = 0.0
    _sum_e: float = 0.0

    def add(self, n: float, e: float, frame: int, score: float):
        self.points.append((n, e))
        self.frames.add(frame)
        self.best_score = max(self.best_score, score)
        self._sum_n += n
        self._sum_e += e

    def distance_to(self, n: float, e: float) -> float:
        count = len(self.points)
        return float(np.hypot(self._sum_n / count - n, self._sum_e / count - e))

    def position(self) -> tuple[float, float]:
        n, e = np.median(np.array(self.points), axis=0)
        return float(n), float(e)


class ObjectMerger:
    def __init__(self, radius: float):
        self.radius = radius
        self.clusters: list[Cluster] = []

    def add(self, cls: str, n: float, e: float, frame: int, score: float):
        same_class = [c for c in self.clusters if c.cls == cls]
        nearest = min(same_class, key=lambda c: c.distance_to(n, e), default=None)
        if nearest is None or nearest.distance_to(n, e) > self.radius:
            nearest = Cluster(cls)
            self.clusters.append(nearest)
        nearest.add(n, e, frame, score)

    def objects(self, min_frames: int) -> list[Cluster]:
        """Clusters seen in at least min_frames different frames; fewer is likely a false alarm."""
        return [c for c in self.clusters if len(c.frames) >= min_frames]


def touches_frame_edge(bbox, width: int, height: int, margin: float = 2.0) -> bool:
    """Is a box (x, y, w, h) cut by the frame? Its centre is then not the object's centre."""
    x, y, w, h = bbox
    return x <= margin or y <= margin or x + w >= width - margin or y + h >= height - margin
