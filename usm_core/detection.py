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

# The model's 10 classes (TEKNOFEST 2025) -> project classes. Fixed before any measurement.
# Not mapped: UAP and UAI (landing pads), MAKINE (machinery), GEMI (ship), TREN (train).
CLASS_MAP = {
    'INSAN': PERSON,
    'ARABA': VEHICLE,
    'KAMYON': VEHICLE,
    'OTOBUS': VEHICLE,
    'MOTOR': VEHICLE,
}


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
