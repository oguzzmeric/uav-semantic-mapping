"""Ground-truth boxes per frame for an exported simulation survey, as a COCO file.

    python3 tools/truth_boxes.py data/export/<name> sim/gz_worlds/village_objects.csv \\
        [--altitude 30] [--overlay 1150 1600 --overlay-dir <folder>]

Writes data/export/<name>/truth_coco.json. Each object's mesh is projected with the true camera
pose, so these boxes come from the geometry, not from what is seen in the frame. Boxes of the
footprint and height alone were too loose for people (PROGRESS.md, 2026-11-04).

Rules, fixed before any detection was compared with them (PROGRESS.md, 2026-11-04):
- frames: tools/frames.py;
- an object is marked ignore (iscrowd = 1), neither missed nor a false alarm when detected, if
  under MIN_VISIBLE_SHARE of its box is inside the frame or if buildings hide over
  MAX_HIDDEN_SHARE of it.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from usm_core.mesh import apply_visual_pose, load_mesh
from frames import MAX_OFF_NADIR_DEG, is_survey_frame, read_frames  # noqa: F401
from usm_core.projection import (
    CAMERA_OFFSET_FRD, Box3D, Intrinsics, hidden_share, image_box, place_on_ground,
)

MIN_VISIBLE_SHARE = 0.5
MAX_HIDDEN_SHARE = 0.5
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'sim/gz_worlds'))
from make_village import PEOPLE, VEHICLES  # noqa: E402  (where the world puts each mesh)

CATEGORIES = {'person': 1, 'vehicle': 2}
FUEL_CACHE = Path.home() / '.gz/fuel/fuel.gazebosim.org/openrobotics/models'
# Model -> mesh in the Fuel cache and its <visual> settings: (file, scale, translation, roll, yaw).
# Same versions as sim/gz_worlds/make_village.py; the hatchbacks share one mesh.
_HATCHBACK = ('hatchback/3/meshes/hatchback.obj', 0.0254, (0, 0, 0), 0.0, math.pi / 2)
MESHES = {
    'Walking person': ('walking%20person/3/meshes/walking.dae', 1.0, (0, 0, -0.02), 0.0, 0.0),
    'Standing person': ('standing%20person/3/meshes/standing.dae', 1.0, (0, 0, 0.02), 0.04, 0.0),
    'Casual female': ('casual%20female/4/meshes/casual_female.dae', 1.0, (0, 0, 0.02), 0.04, 0.0),
    'MaleVisitorOnPhone': ('malevisitoronphone/2/meshes/MaleVisitorStatic.obj', 1.0, (0, 0, 0), 0.0, 0.0),
    'Hatchback': _HATCHBACK,
    'Hatchback blue': _HATCHBACK,
    'Hatchback red': _HATCHBACK,
    'Pickup': ('pickup/4/meshes/pickup.dae', 1.0, (0, 0, 0), 0.0, -math.pi / 2),
    'SUV': ('suv/4/meshes/suv.obj', 0.06, (0, 0, 0), 0.0, -math.pi / 2),
}
# m, mesh size against village_objects.csv: its sizes leave out the 0.04 rad lean of two models.
SIZE_TOLERANCE = 0.05
COLORS = {'person': (60, 60, 255), 'vehicle': (255, 160, 40), 'ignored': (160, 160, 160)}


def load_objects(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    boxes = {r['name']: Box3D(float(r['n']), float(r['e']), float(r['d']), float(r['yaw_deg']),
                              float(r['length']), float(r['width']), float(r['height']))
             for r in rows}
    return {r['name']: r for r in rows}, boxes


def model_points(row) -> np.ndarray:
    """A model's convex hull vertices, relative to the point the csv gives as its position.

    The world places a model so that its footprint centre, as listed in make_village.py, lands
    on that position. A pinhole projection keeps a convex shape convex, so the hull gives the
    same image box as every vertex, with a few hundred points instead of thousands.
    """
    file, scale, translation, roll, yaw = MESHES[row['model']]
    points = apply_visual_pose(load_mesh(FUEL_CACHE / file), scale, translation, roll, yaw)
    size = points.max(axis=0) - points.min(axis=0)
    expected = [float(row['length']), float(row['width']), float(row['height'])]
    if np.abs(size - expected).max() > SIZE_TOLERANCE:
        raise SystemExit(f"{row['model']}: mesh size {size.round(2)} is not {expected} of the csv")
    center = next(m.center for m in (*PEOPLE, *VEHICLES)
                  if m.uri.rsplit('/', 2)[1].replace('%20', ' ') == row['model'])
    return points[ConvexHull(points).vertices] - [center[0], center[1], 0.0]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('objects')
    parser.add_argument('--altitude', type=float, default=30.0, help='survey altitude (m)')
    parser.add_argument('--overlay', type=int, nargs='*', default=[], help='frames to draw')
    parser.add_argument('--overlay-dir', type=Path)
    args = parser.parse_args()

    export = Path(args.export)
    camera = json.loads((export / 'camera.json').read_text())
    k = Intrinsics(camera['width'], camera['height'], camera['fx'], camera['fy'],
                   camera['cx'], camera['cy'])
    objects, boxes = load_objects(args.objects)
    classes = {n: r['class'] for n, r in objects.items()}
    buildings = [boxes[n] for n, c in classes.items() if c == 'building']
    targets = [n for n, c in classes.items() if c in CATEGORIES]
    hulls = {model: model_points(next(r for r in objects.values() if r['model'] == model))
             for model in {objects[n]['model'] for n in targets}}
    points = {n: place_on_ground(hulls[objects[n]['model']], boxes[n].n, boxes[n].e, boxes[n].d,
                                 boxes[n].yaw_deg) for n in targets}

    images, annotations = [], []
    frames = read_frames(export / 'frames.csv')
    for row in frames:
        if not is_survey_frame(row, args.altitude):
            continue
        gimbal = [float(row[f'gimbal_q{c}']) for c in 'wxyz']
        frame = int(row['frame'])
        images.append({'id': frame, 'file_name': row['image'],
                       'width': k.width, 'height': k.height})
        vehicle = Rotation.from_quat([float(row[f'true_q{c}']) for c in 'xyzw'])
        position = np.array([float(row[f'true_{c}']) for c in 'ned']) \
            + vehicle.apply(CAMERA_OFFSET_FRD)
        for name in targets:
            result = image_box(points[name], position, gimbal, k)
            if result is None:
                continue
            (x1, y1, x2, y2), visible = result
            hidden = hidden_share(boxes[name], position, buildings)
            ignore = visible < MIN_VISIBLE_SHARE or hidden > MAX_HIDDEN_SHARE
            annotations.append({
                'id': len(annotations) + 1, 'image_id': frame, 'object': name,
                'category_id': CATEGORIES[classes[name]],
                'bbox': [round(x1, 2), round(y1, 2), round(x2 - x1, 2), round(y2 - y1, 2)],
                'area': round((x2 - x1) * (y2 - y1), 2), 'iscrowd': int(ignore),
                'visible_share': round(visible, 3), 'hidden_share': round(hidden, 3),
            })

    out = export / 'truth_coco.json'
    out.write_text(json.dumps({
        'images': images, 'annotations': annotations,
        'categories': [{'id': i, 'name': n} for n, i in CATEGORIES.items()],
    }))

    counted = [a for a in annotations if not a['iscrowd']]
    print(f'{len(images)} frames of {len(frames)}; {len(annotations)} boxes, '
          f'{len(annotations) - len(counted)} ignored -> {out}')
    for name, category in CATEGORIES.items():
        selected = [a for a in counted if a['category_id'] == category]
        seen = {a['object'] for a in selected}
        sizes = np.array([a['bbox'][2:] for a in selected])
        print(f'  {name}: {len(selected)} boxes, {len(seen)} objects seen of '
              f'{sum(c == name for c in classes.values())}, median size '
              f'{np.median(sizes[:, 0]):.0f} x {np.median(sizes[:, 1]):.0f} px')
    missing = sorted(set(targets) - {a['object'] for a in counted})
    print(f'  objects never counted: {missing or "none"}')

    if args.overlay:
        args.overlay_dir.mkdir(parents=True, exist_ok=True)
        by_frame = {i['id']: i for i in images}
        for frame in args.overlay:
            if frame not in by_frame:
                print(f'  frame {frame} is not in the survey window, not drawn')
                continue
            image = cv2.imread(str(export / 'images' / by_frame[frame]['file_name']))
            for a in (a for a in annotations if a['image_id'] == frame):
                x, y, w, h = a['bbox']
                cls = next(n for n, i in CATEGORIES.items() if i == a['category_id'])
                color = COLORS['ignored' if a['iscrowd'] else cls]
                cv2.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
                cv2.putText(image, a['object'], (int(x), max(int(y) - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imwrite(str(args.overlay_dir / f'truth_{frame:06d}.jpg'), image)


if __name__ == '__main__':
    main()
