"""Step 9 of the offline pipeline: the COLMAP map in world coordinates, in metres.

    .venv/bin/python tools/align_map.py data/export/<name> sim/gz_worlds/village_objects.csv [--map map]

Fits map cameras to EKF2 camera positions (Sim(3), Umeyama, ignoring cameras the map misplaced),
then checks the map against what it did not use: the true camera positions, the ground plane and
the true roof heights. Writes
data/export/<name>/<map>/aligned.npz (world points, their colours, the transform).
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import pycolmap

from frames import read_frames
from geolocate import camera_position
from usm_core.alignment import robust_umeyama

OPEN_GROUND_CLEARANCE = 3.0  # m from any object's footprint centre beyond its own size


def summary(values) -> str:
    return (f'median {np.median(values):.3f}, p95 {np.percentile(values, 95):.3f}, '
            f'max {np.max(values):.3f} m')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('objects')
    parser.add_argument('--map', default='map', help='map folder inside the export')
    args = parser.parse_args()

    export = Path(args.export)
    folder = export / args.map
    models = [pycolmap.Reconstruction(p) for p in sorted(p.parent for p in folder.glob('*/points3D.bin'))]
    model = max(models, key=lambda m: m.num_reg_images())
    rows = {r['image']: r for r in read_frames(export / 'frames.csv')}

    images = list(model.images.values())
    print(f'{len(models)} model(s); largest: {model.num_reg_images()} photos, '
          f'{model.num_points3D()} points')
    centres = np.array([i.projection_center() for i in images])
    estimated = np.array([camera_position(rows[i.name], 'ekf') for i in images])
    true = np.array([camera_position(rows[i.name], 'true') for i in images])

    sim, keep = robust_umeyama(centres, estimated)
    aligned = sim.apply(centres)
    print(f'scale {sim.scale:.4f} m per map unit; {keep.sum()} cameras fitted, {(~keep).sum()} misplaced')
    print(f'cameras to EKF2 (fitted):        {summary(np.linalg.norm((aligned - estimated)[keep], axis=1))}')
    print(f'cameras to the truth (not used): {summary(np.linalg.norm((aligned - true)[keep], axis=1))}')
    print(f'  of which horizontal:           {summary(np.linalg.norm((aligned - true)[keep, :2], axis=1))}')
    print(f'  of which vertical:             {summary(np.abs((aligned - true)[keep, 2]))}')
    if (~keep).any():
        off = np.linalg.norm((aligned - true)[~keep], axis=1)
        print(f'misplaced cameras to the truth:  {summary(off)}')

    points = np.array([p.xyz for p in model.points3D.values()])
    colors = np.array([p.color for p in model.points3D.values()])
    world = sim.apply(points)

    with open(args.objects, newline='') as f:
        objects = list(csv.DictReader(f))
    n, e, d = world.T
    clear = np.ones(len(world), bool)
    for o in objects:
        reach = np.hypot(float(o['length']), float(o['width'])) / 2 + OPEN_GROUND_CLEARANCE
        clear &= np.hypot(n - float(o['n']), e - float(o['e'])) > reach
    clear &= (np.abs(n) < 40) & (np.abs(e) < 40)
    print(f'ground: {clear.sum()} points in the open, height {summary(np.abs(d[clear]))}; '
          f'mean d {d[clear].mean():+.3f} m')

    for o in (o for o in objects if o['class'] == 'building'):
        c, s = np.cos(np.radians(float(o['yaw_deg']))), np.sin(np.radians(float(o['yaw_deg'])))
        u = c * (n - float(o['n'])) + s * (e - float(o['e']))
        v = -s * (n - float(o['n'])) + c * (e - float(o['e']))
        inside = (np.abs(u) < float(o['length']) / 2) & (np.abs(v) < float(o['width']) / 2)
        if inside.sum():
            top = np.percentile(-d[inside], 99)
            print(f"  {o['name']}: {inside.sum()} points, top {top:.2f} m (true {float(o['height']):.2f} m)")

    np.savez_compressed(folder / 'aligned.npz', points=world, colors=colors, cameras=aligned[keep],
                        scale=sim.scale,
                        rotation=sim.rotation, translation=sim.translation)
    print(f'-> {folder / "aligned.npz"}')


if __name__ == '__main__':
    main()
