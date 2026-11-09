"""Open the aligned 3D map in Rerun, with the drone's path and the reported objects on it.

    .venv/bin/python tools/view_map.py data/export/<name> --map map_gated --located full_frame_ekf
"""
import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np
import rerun as rr

PERSON_COLOR = (255, 60, 60)
VEHICLE_COLOR = (40, 160, 255)
PATH_COLOR = (40, 200, 80)
MAX_REACH = 60.0  # m from the origin: leaves out the few points triangulated far away
HEIGHT_RANGE = (-2.0, 15.0)  # m


def to_view(ned) -> np.ndarray:
    """NED -> the viewer's Z-up frame: x east, y north, z up."""
    ned = np.asarray(ned, float)
    return np.column_stack([ned[:, 1], ned[:, 0], -ned[:, 2]])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--map', default='map_gated', help='map folder inside the export')
    parser.add_argument('--located', default='full_frame_ekf', help='suffix of the objects file')
    args = parser.parse_args()

    export = Path(args.export)
    data = np.load(export / args.map / 'aligned.npz')
    points, colors = data['points'], data['colors']
    height = -points[:, 2]
    inside = (np.abs(points[:, :2]).max(axis=1) < MAX_REACH) \
        & (height > HEIGHT_RANGE[0]) & (height < HEIGHT_RANGE[1])

    # rr.spawn looks for the 'rerun' program on PATH; the venv's bin is there only if activated.
    os.environ['PATH'] = f"{Path(sys.executable).parent}{os.pathsep}{os.environ['PATH']}"
    rr.init('usm_map')
    rr.spawn()
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log('world/map', rr.Points3D(to_view(points[inside]), colors=colors[inside], radii=0.03), static=True)
    rr.log('world/path', rr.LineStrips3D([to_view(data['cameras'])], colors=[PATH_COLOR]), static=True)

    objects = json.loads((export / f'objects_{args.located}.json').read_text())
    for cls, color in (('person', PERSON_COLOR), ('vehicle', VEHICLE_COLOR)):
        found = [o for o in objects if o['cls'] == cls]
        ned = [(o['n'], o['e'], -3.0) for o in found]  # drawn above the object, to stand out
        rr.log(f'world/{cls}', rr.Points3D(to_view(ned), colors=[color], radii=0.5,
                                           labels=[f"{cls} {o['frames']} frames" for o in found],
                                           show_labels=False), static=True)
    print(f'{inside.sum()} of {len(points)} points, {len(objects)} objects')


if __name__ == '__main__':
    main()
