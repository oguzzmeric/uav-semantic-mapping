"""Person and vehicle detection with the project's YOLO11l model.

One place for the detection settings and the class mapping, shared by the live ROS node
and the offline tools, so live and offline results differ only by how they are run.

ultralytics is imported when a Detector is created, not at module import: the class
mapping stays testable without torch (CI runs without the 'detect' dependency group).
"""
from dataclasses import dataclass

import numpy as np

PERSON = 'person'
VEHICLE = 'vehicle'

# The model's 10 classes -> project classes. Fixed before any measurement.
# Not mapped: UAP and UAI (landing pads), MAKINE (machinery), GEMI (ship), TREN (train).
CLASS_MAP = {
    'INSAN': PERSON,
    'ARABA': VEHICLE,
    'KAMYON': VEHICLE,
    'OTOBUS': VEHICLE,
    'MOTOR': VEHICLE,
}


# Offline tiling of a 1080p frame: 2 tiles of 1280 x 1080, so the model gets native pixels. Fixed
# before any tiled result was seen. The overlap is wider than the largest object (~340 px).
TILE_WIDTH = 1280
TILE_OVERLAP = 640
TILE_EDGE_MARGIN = 2.0  # px; a box this close to an edge between tiles counts as cut


@dataclass(frozen=True)
class Settings:
    imgsz: int = 1280   # the model was trained at 1280; a 1080p frame enters as 1280x736
    conf: float = 0.25  # minimum class score
    iou: float = 0.7    # NMS drops a box overlapping a higher-scored one by more than this
    half: bool = False  # fp16 gives NaN at 1280 on the GTX 1650 and is 4x slower there


@dataclass(frozen=True)
class Detection:
    cls: str      # PERSON or VEHICLE
    score: float
    x1: float     # box corners [px] in the input image
    y1: float
    x2: float
    y2: float


def project_class(model_class: str) -> str | None:
    """The project class for a model class name, or None if the class is not used."""
    return CLASS_MAP.get(model_class)


def tile_offsets(image_width: int, tile_width: int = TILE_WIDTH) -> list[int]:
    """Left edges of side-by-side tiles covering the width with TILE_OVERLAP between them."""
    if image_width <= tile_width:
        return [0]
    step = tile_width - TILE_OVERLAP
    offsets = list(range(0, image_width - tile_width, step))
    return [*offsets, image_width - tile_width]


def iou(a: Detection, b: Detection) -> float:
    w = min(a.x2, b.x2) - max(a.x1, b.x1)
    h = min(a.y2, b.y2) - max(a.y1, b.y1)
    if w <= 0 or h <= 0:
        return 0.0
    shared = w * h
    return shared / ((a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - shared)


def merge_tiles(tiles: list[tuple[int, list[Detection]]], image_width: int, iou_limit: float,
                tile_width: int = TILE_WIDTH) -> list[Detection]:
    """One set of full-image boxes from each tile's (left edge, boxes in tile coordinates).

    A box touching an edge between tiles shows a cut object; the overlap is wider than any
    object, so the whole object is in the neighbouring tile and the cut box is dropped. What
    both tiles saw is then reduced by per-class NMS, highest score first.
    """
    boxes = []
    for offset, detections in tiles:
        inner_left = offset > 0
        inner_right = offset + tile_width < image_width
        for d in detections:
            if (inner_left and d.x1 <= TILE_EDGE_MARGIN) \
                    or (inner_right and d.x2 >= tile_width - TILE_EDGE_MARGIN):
                continue
            boxes.append(Detection(d.cls, d.score, d.x1 + offset, d.y1, d.x2 + offset, d.y2))
    kept = []
    for box in sorted(boxes, key=lambda d: -d.score):
        if all(box.cls != k.cls or iou(box, k) <= iou_limit for k in kept):
            kept.append(box)
    return kept


class Detector:
    """YOLO on one BGR image at a time; returns project-class boxes and stage timings."""

    def __init__(self, weights: str, settings: Settings = Settings(), device: str = 'cuda:0'):
        from ultralytics import YOLO
        self.settings = settings
        self.device = device
        self._model = YOLO(weights)

    def warmup(self, shape: tuple[int, int, int] = (1080, 1920, 3)):
        """Run once on a blank frame: the first call loads CUDA kernels and is slow."""
        self(np.zeros(shape, np.uint8))

    def __call__(self, image_bgr: np.ndarray) -> tuple[list[Detection], dict[str, float]]:
        s = self.settings
        r = self._model.predict(image_bgr, imgsz=s.imgsz, conf=s.conf, iou=s.iou, half=s.half,
                                device=self.device, verbose=False)[0]
        detections = []
        for (x1, y1, x2, y2), c, p in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(),
                                          r.boxes.conf.tolist()):
            cls = project_class(self._model.names[int(c)])
            if cls is not None:
                detections.append(Detection(cls, p, x1, y1, x2, y2))
        # Wall-clock ms per stage; ultralytics synchronises CUDA before reading the clock.
        timings = {'preprocess_ms': r.speed['preprocess'], 'inference_ms': r.speed['inference'],
                   'postprocess_ms': r.speed['postprocess']}
        return detections, timings

    def detect_tiled(self, image_bgr: np.ndarray) -> tuple[list[Detection], dict[str, float]]:
        """Detect on full-resolution tiles, for the offline tools: nothing is scaled down."""
        width = image_bgr.shape[1]
        tiles, totals = [], {}
        for offset in tile_offsets(width):
            detections, timings = self(image_bgr[:, offset:offset + TILE_WIDTH])
            tiles.append((offset, detections))
            for stage, ms in timings.items():
                totals[stage] = totals.get(stage, 0.0) + ms
        return merge_tiles(tiles, width, self.settings.iou), totals
