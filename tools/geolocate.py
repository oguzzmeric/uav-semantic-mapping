"""Step 6 of the offline pipeline: the ground position of every box of a detection run.

    .venv/bin/python tools/geolocate.py data/export/<name> sim/gz_worlds/village_objects.csv \\
        --run full_frame [--pose ekf|true] [--no-height]

Writes data/export/<name>/geolocated_<run>_<pose>[_noheight].json. With --run truth the boxes are
the ground-truth ones: no detection error is left, only geometry.

The printed errors are a check of this step, in simulation, per box: a truth box against its own
object, a model box against the nearest object of its class. The object-level numbers come after
merging.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from usm_core.detection import Settings
from usm_core.geolocation import CLASS_HEIGHTS, geolocate
from usm_core.projection import CAMERA_OFFSET_FRD, Intrinsics, rotation_matrix


def camera_position(row, pose: str) -> np.ndarray:
    position = np.array([float(row[f'{pose}_{c}']) for c in 'ned'])
    return position + rotation_matrix([float(row[f'{pose}_q{c}']) for c in 'wxyz']) @ CAMERA_OFFSET_FRD


def load_boxes(export: Path, run: str, truth: dict) -> list[dict]:
    """Boxes as {frame, category_id, score, bbox, object}; object is None for model boxes."""
    if run == 'truth':
        return [{'frame': a['image_id'], 'category_id': a['category_id'], 'score': 1.0,
                 'bbox': a['bbox'], 'object': a['object']}
                for a in truth['annotations'] if not a['iscrowd']]
    results = json.loads((export / f'detections_{run}.json').read_text())
    return [{'frame': r['image_id'], 'category_id': r['category_id'], 'score': r['score'],
             'bbox': r['bbox'], 'object': None}
            for r in results if r['score'] >= Settings().conf]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('objects')
    parser.add_argument('--run', default='full_frame', help="detection run, or 'truth'")
    parser.add_argument('--pose', choices=['ekf', 'true'], default='ekf', help='vehicle position')
    parser.add_argument('--no-height', action='store_true', help='cut the rays with the ground')
    args = parser.parse_args()

    export = Path(args.export)
    camera = json.loads((export / 'camera.json').read_text())
    k = Intrinsics(camera['width'], camera['height'], camera['fx'], camera['fy'],
                   camera['cx'], camera['cy'])
    truth = json.loads((export / 'truth_coco.json').read_text())
    categories = {c['id']: c['name'] for c in truth['categories']}
    with open(export / 'frames.csv', newline='') as f:
        frames = {int(r['frame']): r for r in csv.DictReader(f)}
    with open(args.objects, newline='') as f:
        objects = [r for r in csv.DictReader(f) if r['class'] in categories.values()]

    located = []
    for box in load_boxes(export, args.run, truth):
        row = frames[box['frame']]
        x, y, w, h = box['bbox']
        cls = categories[box['category_id']]
        point = geolocate(x + w / 2, y + h / 2, camera_position(row, args.pose),
                          [float(row[f'gimbal_q{c}']) for c in 'wxyz'], k,
                          0.0 if args.no_height else CLASS_HEIGHTS[cls])
        if point is not None:
            located.append({**box, 'cls': cls, 'n': round(float(point[0]), 3),
                            'e': round(float(point[1]), 3)})

    suffix = f'{args.run}_{args.pose}' + ('_noheight' if args.no_height else '')
    out = export / f'geolocated_{suffix}.json'
    out.write_text(json.dumps(located))
    print(f'{suffix}: {len(located)} boxes -> {out}')

    positions = {o['name']: (float(o['n']), float(o['e'])) for o in objects}
    for cls in categories.values():
        boxes = [b for b in located if b['cls'] == cls]
        names = [o['name'] for o in objects if o['class'] == cls]
        grid = np.array([positions[n] for n in names])
        points = np.array([(b['n'], b['e']) for b in boxes])
        if args.run == 'truth':
            own = np.array([positions[b['object']] for b in boxes])
            errors = np.hypot(*(points - own).T)
        else:
            errors = np.linalg.norm(points[:, None] - grid[None], axis=2).min(axis=1)
        print(f'  {cls:8s} {len(boxes):5d} boxes: error mean {errors.mean():.3f}, median '
              f'{np.median(errors):.3f}, p95 {np.percentile(errors, 95):.3f}, '
              f'max {errors.max():.2f} m; within 2 m: {100 * (errors <= 2).mean():.1f} %')


if __name__ == '__main__':
    main()
