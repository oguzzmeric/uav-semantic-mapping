"""Detect people and vehicles in the frames of an exported survey, as a COCO results file.

    .venv/bin/python tools/detect_offline.py data/export/<name> --run full_frame
    .venv/bin/python tools/detect_offline.py data/export/<name> --run tiled --tiled

Runs on the frames listed in truth_coco.json (tools/truth_boxes.py) and writes
data/export/<name>/detections_<run>.json. Boxes are kept down to a score of SCORE_FLOOR because
AP sweeps over all scores; the working threshold (Settings.conf) is applied in the evaluation.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import cv2
import numpy as np

from usm_core.detection import PERSON, VEHICLE, Detector, Settings

WEIGHTS = 'models/teknofest_yolo11l_nadir.pt'
SCORE_FLOOR = 0.001  # the usual floor for computing AP (also the default of Ultralytics' val)
CATEGORY_IDS = {PERSON: 1, VEHICLE: 2}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--run', default='full_frame', help='name of this detection run')
    parser.add_argument('--tiled', action='store_true', help='detect on full-resolution tiles')
    args = parser.parse_args()

    export = Path(args.export)
    out = export / f'detections_{args.run}.json'
    if out.exists():
        raise SystemExit(f'{out} exists')
    images = json.loads((export / 'truth_coco.json').read_text())['images']

    detector = Detector(WEIGHTS, replace(Settings(), conf=SCORE_FLOOR))
    detector.warmup()
    detect = detector.detect_tiled if args.tiled else detector
    results, times = [], []
    for i, image in enumerate(images):
        frame = cv2.imread(str(export / 'images' / image['file_name']))
        t0 = time.perf_counter()
        detections, _ = detect(frame)
        times.append(time.perf_counter() - t0)
        results += [{'image_id': image['id'], 'category_id': CATEGORY_IDS[d.cls],
                     'bbox': [round(d.x1, 2), round(d.y1, 2), round(d.x2 - d.x1, 2),
                              round(d.y2 - d.y1, 2)],
                     'score': round(d.score, 4)} for d in detections]
        if i % 200 == 0:
            print(f'{i}/{len(images)} frames, {np.median(times) * 1e3:.0f} ms/frame', flush=True)

    out.write_text(json.dumps(results))
    print(f'{len(images)} frames, {len(results)} boxes, median {np.median(times) * 1e3:.0f} ms/frame '
          f'-> {out}')


if __name__ == '__main__':
    main()
