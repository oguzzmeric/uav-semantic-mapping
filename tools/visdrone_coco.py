"""A VisDrone-DET split as a COCO ground-truth file, with the project's class mapping.

    .venv/bin/python -I tools/visdrone_coco.py datasets/visdrone/VisDrone2019-DET-test-dev \\
        datasets/visdrone/eval_test_dev

Writes <out>/truth_coco.json and links <out>/images to the split's images, so detect_offline.py
and eval_detections.py run on it as on a simulated survey. Fixed before any detection was compared
(PROGRESS.md, 2026-11-09).
"""
import argparse
import json
from pathlib import Path

import cv2

CATEGORIES = {'person': 1, 'vehicle': 2}
# VisDrone category -> project class; None: kept as an ignored region, neither missed nor a false
# alarm when detected. Bicycles and tricycles have no counterpart in the model's classes.
VISDRONE_CLASSES = {
    0: None,         # ignored region
    1: 'person',     # pedestrian
    2: 'person',     # people (sitting, lying, ...)
    3: None,         # bicycle
    4: 'vehicle',    # car
    5: 'vehicle',    # van
    6: 'vehicle',    # truck
    7: None,         # tricycle
    8: None,         # awning-tricycle
    9: 'vehicle',    # bus
    10: 'vehicle',   # motor (the model's MOTOR counts as a vehicle)
    11: None,        # others
}


def read_annotations(path: Path) -> list[tuple[list[float], int, int]]:
    """(bbox, category, score) per line; score 0 marks a box VisDrone itself ignores."""
    boxes = []
    for line in path.read_text().splitlines():
        fields = line.strip().rstrip(',').split(',')
        if len(fields) >= 6:
            x, y, w, h = (float(v) for v in fields[:4])
            if w > 0 and h > 0:
                boxes.append(([x, y, w, h], int(fields[5]), int(fields[4])))
    return boxes


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('split')
    parser.add_argument('out')
    args = parser.parse_args()

    split, out = Path(args.split), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if not (out / 'images').exists():
        (out / 'images').symlink_to((split / 'images').resolve())

    images, annotations = [], []
    for index, image_path in enumerate(sorted((split / 'images').glob('*.jpg')), start=1):
        height, width = cv2.imread(str(image_path)).shape[:2]
        images.append({'id': index, 'file_name': image_path.name, 'width': width, 'height': height})
        for bbox, category, score in read_annotations(split / 'annotations' / f'{image_path.stem}.txt'):
            cls = VISDRONE_CLASSES.get(category)
            ignore = cls is None or score == 0
            for name in (CATEGORIES if cls is None else [cls]):  # an ignored region spans both classes
                annotations.append({'id': len(annotations) + 1, 'image_id': index,
                                    'category_id': CATEGORIES[name], 'bbox': bbox,
                                    'area': bbox[2] * bbox[3], 'iscrowd': int(ignore)})

    (out / 'truth_coco.json').write_text(json.dumps({
        'images': images, 'annotations': annotations,
        'categories': [{'id': i, 'name': n} for n, i in CATEGORIES.items()]}))
    counted = [a for a in annotations if not a['iscrowd']]
    print(f'{len(images)} images, {len(counted)} counted boxes '
          + ', '.join(f"{n} {sum(a['category_id'] == i for a in counted)}" for n, i in CATEGORIES.items())
          + f', {len(annotations) - len(counted)} ignored -> {out / "truth_coco.json"}')


if __name__ == '__main__':
    main()
