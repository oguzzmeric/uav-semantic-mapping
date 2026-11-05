"""Step 7 of the offline pipeline: one record per object from the located boxes.

    .venv/bin/python tools/merge_objects.py data/export/<name> --located full_frame_ekf

Reads geolocated_<located>.json, writes objects_<located>.json. Boxes cut by the frame are left
out: their centre is not the object's centre (vehicles: median error 0.94 m against 0.12 m).
"""
import argparse
import json
from pathlib import Path

from usm_core.merging import ObjectMerger, touches_frame_edge

# Chosen on the development flight (PROGRESS.md, 2026-11-05), fixed before the test flight.
MERGE_RADIUS = 2.0  # m; half the 4 m minimum spacing between objects in the village
MIN_FRAMES = 5


def merge(located: list[dict], width: int, height: int, radius: float, min_frames: int):
    merger = ObjectMerger(radius)
    for box in sorted(located, key=lambda b: b['frame']):
        if not touches_frame_edge(box['bbox'], width, height):
            merger.add(box['cls'], box['n'], box['e'], box['frame'], box['score'])
    return merger.objects(min_frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--located', default='full_frame_ekf', help='suffix of the geolocated file')
    parser.add_argument('--radius', type=float, default=MERGE_RADIUS, help='merge radius (m)')
    parser.add_argument('--min-frames', type=int, default=MIN_FRAMES)
    args = parser.parse_args()

    export = Path(args.export)
    camera = json.loads((export / 'camera.json').read_text())
    located = json.loads((export / f'geolocated_{args.located}.json').read_text())
    objects = merge(located, camera['width'], camera['height'], args.radius, args.min_frames)
    out = export / f'objects_{args.located}.json'
    records = [{'cls': o.cls, 'n': round(o.position()[0], 3), 'e': round(o.position()[1], 3),
                'frames': len(o.frames), 'best_score': o.best_score} for o in objects]
    out.write_text(json.dumps(records))
    print(f'{len(located)} boxes -> {len(records)} objects -> {out}')


if __name__ == '__main__':
    main()
