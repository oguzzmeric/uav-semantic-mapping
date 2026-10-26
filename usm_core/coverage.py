"""Lawnmower (boustrophedon) survey planning for a nadir camera.

Frames and conventions:
- Local NED [m]: x north, y east, z down. Waypoints carry z = -altitude.
- Camera points straight down (nadir). The image width is across-track, the image
  height is along-track.
- Lanes run along x (north/south) and are stacked along y (east). The first lane is
  at the lowest y and is flown towards +x; directions then alternate.
"""
from dataclasses import dataclass
import math
from turtle import width


@dataclass(frozen=True)
class Camera:
    width_px: int
    height_px: int
    hfov_rad: float


@dataclass(frozen=True)
class Area:
    """Axis-aligned rectangle in local NED [m]."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class SurveyPlan:
    waypoints: list[tuple[float, float, float]]  # NED (x, y, z); 2 per lane, in flight order
    n_lanes: int
    lane_spacing: float   # actual distance between neighbouring lanes [m]; 0.0 if n_lanes == 1
    photo_spacing: float  # distance between photos along a lane [m]
    path_length: float    # length of the polyline through all waypoints [m]


def ground_footprint(cam: Camera, altitude: float) -> tuple[float, float]:
    """Ground footprint (across-track width, along-track depth) [m] of a nadir image."""
    if altitude <= 0:
        raise ValueError(f'altitude must be positive, got {altitude}')
    width = 2.0 * altitude * math.tan(cam.hfov_rad / 2.0)
    depth = width * cam.height_px / cam.width_px  # same f for both axes: aspect is kept
    return width, depth


def gsd(cam: Camera, altitude: float) -> float:
    """Ground sampling distance [m/px] at nadir."""
    width, _ = ground_footprint(cam, altitude)
    return width / cam.width_px


def _check_overlap(overlap: float, name: str) -> None:
    if not (0 <= overlap < 1):
        raise ValueError(f'{name} must be in [0, 1), got {overlap}')


def lane_spacing(cam: Camera, altitude: float, side_overlap: float) -> float:
    """Largest lane spacing [m] that still gives `side_overlap` (0 <= overlap < 1)."""
    _check_overlap(side_overlap, 'side_overlap')
    width, _ = ground_footprint(cam, altitude)
    return width * (1 - side_overlap)


def photo_spacing(cam: Camera, altitude: float, front_overlap: float) -> float:
    """Distance between photos along a lane [m] for `front_overlap` (0 <= overlap < 1)."""
    _check_overlap(front_overlap, 'front_overlap')

    _, depth = ground_footprint(cam, altitude)
    return depth * (1 - front_overlap)


def _lane_positions(lo: float, hi: float, footprint: float,
                    max_spacing: float) -> tuple[list[float], float]:
    """Evenly spaced lane centres in [lo, hi] keeping the footprint inside."""
    room = hi - lo
    if room <= footprint:
        return [(lo + hi) / 2.0], 0.0
    n = math.ceil((room - footprint) / max_spacing - 1e-9) + 1
    spacing = (room - footprint) / (n - 1)
    positions = [lo + footprint / 2.0 + i * spacing for i in range(n)]
    return positions, spacing


def lawnmower(area: Area, cam: Camera, altitude: float, side_overlap: float,
              front_overlap: float, edge_margin: float | None = None) -> SurveyPlan:
    """Plan a lawnmower survey over `area`.

    The area is grown by `edge_margin` on every side; lanes and lane ends are then placed
    so the camera footprint stays inside the grown area, with lanes spread evenly (the
    actual spacing may be smaller than `lane_spacing(...)`, never larger).

    edge_margin = 0      -> footprint stays inside the area (edges seen by fewer lanes)
    edge_margin = Gw / 2 -> first and last lanes lie on the area boundary (default, None)

    If the grown area is narrower than the footprint, a single lane runs through its
    middle; if it is shorter than the footprint depth, both lane ends sit at its middle.
    Raises ValueError for altitude <= 0, overlaps outside [0, 1), an empty area or a
    negative edge_margin.
    """
    if area.x_max <= area.x_min or area.y_max <= area.y_min:
        raise ValueError(f'area must be non-empty, got {area}')
    width, depth = ground_footprint(cam, altitude)  # also rejects altitude <= 0
    if edge_margin is None:
        edge_margin = width / 2.0
    if edge_margin < 0:
        raise ValueError(f'edge_margin must be non-negative, got {edge_margin}')
    max_lane_spacing = lane_spacing(cam, altitude, side_overlap)
    photo_step = photo_spacing(cam, altitude, front_overlap)

    # grown area
    x_lo, x_hi = area.x_min - edge_margin, area.x_max + edge_margin
    y_lo, y_hi = area.y_min - edge_margin, area.y_max + edge_margin

    # lanes are stacked along y: across-track the image covers its width
    lane_ys, actual_spacing = _lane_positions(y_lo, y_hi, width, max_lane_spacing)

    # lane ends along x: along-track the image covers its depth
    x_start, x_end = x_lo + depth / 2.0, x_hi - depth / 2.0
    if x_start > x_end:  # area shorter than the footprint depth
        x_start = x_end = (x_lo + x_hi) / 2.0

    waypoints = []
    for i, y in enumerate(lane_ys):
        a, b = (x_start, x_end) if i % 2 == 0 else (x_end, x_start)
        waypoints += [(a, y, -altitude), (b, y, -altitude)]
    path_length = sum(math.dist(p, q) for p, q in zip(waypoints, waypoints[1:]))

    return SurveyPlan(waypoints=waypoints, n_lanes=len(lane_ys),
                      lane_spacing=actual_spacing, photo_spacing=photo_step,
                      path_length=path_length)
