"""Object-level metrics: reports against the true positions, in simulation.

A report is correct if it lies within the match limit of a true object of its class. Each true
object and each report can match only once, with the pairing that minimises the total distance.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment

MATCH_LIMIT = 2.0  # m, fixed before any measurement


@dataclass(frozen=True)
class ObjectScore:
    truths: int
    reports: int
    matched: int
    errors: np.ndarray                 # horizontal error of each matched report [m]
    missed: list[int]                  # indices of true objects without a report
    false_alarms: list[int]            # indices of reports without a true object

    @property
    def recall(self) -> float:
        return self.matched / self.truths if self.truths else 0.0

    @property
    def precision(self) -> float:
        return self.matched / self.reports if self.reports else 0.0


def score_objects(reports, truths, limit: float = MATCH_LIMIT) -> ObjectScore:
    """Both are (n, e) arrays of one class."""
    reports, truths = np.asarray(reports, float).reshape(-1, 2), np.asarray(truths, float).reshape(-1, 2)
    if not len(reports) or not len(truths):
        return ObjectScore(len(truths), len(reports), 0, np.array([]), list(range(len(truths))),
                           list(range(len(reports))))
    distances = np.linalg.norm(reports[:, None] - truths[None], axis=2)
    # A pair over the limit costs more than any set of valid pairs, so it is only used when forced.
    cost = np.where(distances <= limit, distances, limit * (len(reports) + len(truths) + 1))
    rows, columns = linear_sum_assignment(cost)
    pairs = [(r, t) for r, t in zip(rows, columns) if distances[r, t] <= limit]
    return ObjectScore(
        truths=len(truths), reports=len(reports), matched=len(pairs),
        errors=np.array([distances[r, t] for r, t in pairs]),
        missed=sorted(set(range(len(truths))) - {t for _, t in pairs}),
        false_alarms=sorted(set(range(len(reports))) - {r for r, _ in pairs}),
    )
