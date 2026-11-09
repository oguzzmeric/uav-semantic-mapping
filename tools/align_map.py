"""Step 9 of the offline pipeline: the COLMAP map in world coordinates, in metres.

    .venv/bin/python tools/align_map.py data/export/<name> [--map map] [--objects <truth csv>]

Fits map cameras to EKF2 camera positions (Sim(3), Umeyama, ignoring cameras the map misplaced).
In simulation it then checks the map against what it did not use: the true camera positions, the
ground plane and, with --objects, the true roof heights. Writes data/export/<name>/<map>/aligned.npz
(world points, their colours, the cameras, the transform) and alignment.json (the numbers).
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pycolmap

from frames import read_frames
from geolocate import camera_position
from usm_core.alignment import robust_umeyama

OPEN_GROUND_CLEARANCE = 3.0  # m from any object's footprint centre beyond its own size


def stats(values) -> dict:
    return {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'max': float(np.max(values))}


def summary(values) -> str:
    s = stats(values)
    return f"median {s['median']:.3f}, p95 {s['p95']:.3f}, max {s['max']:.3f} m"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--map', default='map', help='map folder inside the export')
    parser.add_argument('--objects', help='true object positions, in simulation')
    args = parser.parse_args()

    export = Path(args.export)
    folder = export / args.map
    models = [pycolmap.Reconstruction(p) for p in sorted(p.parent for p in folder.glob('*/points3D.bin'))]
    model = max(models, key=lambda m: m.num_reg_images())
    rows = {r['image']: r for r in read_frames(export / 'frames.csv')}
    images = sorted(model.images.values(), key=lambda i: int(rows[i.name]['frame']))  # flight order
    print(f'{len(models)} model(s); largest: {model.num_reg_images()} photos, '
          f'{model.num_points3D()} points')

    centres = np.array([i.projection_center() for i in images])
    estimated = np.array([camera_position(rows[i.name], 'ekf') for i in images])
    sim, keep = robust_umeyama(centres, estimated)
    aligned = sim.apply(centres)
    fit = np.linalg.norm((aligned - estimated)[keep], axis=1)
    print(f'scale {sim.scale:.4f} m per map unit; {keep.sum()} cameras fitted, {(~keep).sum()} misplaced')
    print(f'cameras to EKF2 (fitted):        {summary(fit)}')
    with pycolmap.Database.open(folder / 'database.db') as db:
        photos_used = db.num_images()
    result = {
        'models': len(models), 'photos_used': photos_used, 'photos_registered': model.num_reg_images(),
        'points': model.num_points3D(), 'reprojection_px': model.compute_mean_reprojection_error(),
        'scale': sim.scale, 'cameras_fitted': int(keep.sum()), 'cameras_misplaced': int((~keep).sum()),
        'cameras_to_ekf': stats(fit),
    }

    points = np.array([p.xyz for p in model.points3D.values()])
    colors = np.array([p.color for p in model.points3D.values()])
    world = sim.apply(points)

    if 'true_n' in next(iter(rows.values())):  # simulation: check against the truth
        true = np.array([camera_position(rows[i.name], 'true') for i in images])
        to_truth = np.linalg.norm((aligned - true)[keep], axis=1)
        print(f'cameras to the truth (not used): {summary(to_truth)}')
        print(f'  of which horizontal:           {summary(np.linalg.norm((aligned - true)[keep, :2], axis=1))}')
        print(f'  of which vertical:             {summary(np.abs((aligned - true)[keep, 2]))}')
        result['cameras_to_truth'] = stats(to_truth)
        if (~keep).any():
            print(f'misplaced cameras to the truth:  {summary(np.linalg.norm((aligned - true)[~keep], axis=1))}')

    if args.objects:
        with open(args.objects, newline='') as f:
            objects = list(csv.DictReader(f))
        n, e, d = world.T
        clear = (np.abs(n) < 40) & (np.abs(e) < 40)
        for o in objects:
            reach = np.hypot(float(o['length']), float(o['width'])) / 2 + OPEN_GROUND_CLEARANCE
            clear &= np.hypot(n - float(o['n']), e - float(o['e'])) > reach
        print(f'ground: {clear.sum()} points in the open, height {summary(np.abs(d[clear]))}; '
              f'mean d {d[clear].mean():+.3f} m')
        result['ground_height'] = stats(np.abs(d[clear]))
        result['roofs'] = []
        for o in (o for o in objects if o['class'] == 'building'):
            c, s = np.cos(np.radians(float(o['yaw_deg']))), np.sin(np.radians(float(o['yaw_deg'])))
            u = c * (n - float(o['n'])) + s * (e - float(o['e']))
            v = -s * (n - float(o['n'])) + c * (e - float(o['e']))
            inside = (np.abs(u) < float(o['length']) / 2) & (np.abs(v) < float(o['width']) / 2)
            if inside.sum():
                top = float(np.percentile(-d[inside], 99))
                print(f"  {o['name']}: {inside.sum()} points, top {top:.2f} m (true {float(o['height']):.2f} m)")
                result['roofs'].append({'name': o['name'], 'top': top, 'true': float(o['height'])})

    np.savez_compressed(folder / 'aligned.npz', points=world, colors=colors, cameras=aligned[keep],
                        scale=sim.scale, rotation=sim.rotation, translation=sim.translation)
    (folder / 'alignment.json').write_text(json.dumps(result, indent=1))
    print(f'-> {folder / "aligned.npz"}, {folder / "alignment.json"}')


if __name__ == '__main__':
    main()
