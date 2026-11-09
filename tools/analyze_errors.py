"""Where the position error comes from, and how it grows with angle and height errors.

    .venv/bin/python tools/analyze_errors.py sim/gz_worlds/village_seed2_objects.csv \\
        data/export/village_seed2_alt20 data/export/village_seed2_alt30 data/export/village_seed2_alt40

Simulation only. Method fixed before the first run (PROGRESS.md, 2026-11-09):
- error budget: truth or model boxes, true or EKF2 pose, with or without the height correction;
- sensitivity: the camera tilted by delta (the same way every frame, or a random way per frame),
  or believed delta_h higher; each box's shift compared with delta_d ~ h delta / cos^2 theta and
  delta_d = delta_h tan theta.
Writes tables and plots to data/analysis/.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
import numpy as np

from frames import read_frames
from geolocate import camera_position, load_boxes
from merge_objects import MERGE_RADIUS, MIN_FRAMES, merge
from usm_core.geolocation import CLASS_HEIGHTS, intersect_plane, pixel_ray
from usm_core.metrics import score_objects
from usm_core.projection import Intrinsics, rotation_matrix

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (after choosing the file-only backend)

TILTS = (1.0, 3.0, 5.0)              # deg
HEIGHT_OFFSETS = (0.5, 1.0, 2.0, 5.0)  # m
SEED = 0
OUT = Path('data/analysis')


def tilt(axis, angle_rad) -> np.ndarray:
    """Rotation about a world (NED) axis, Rodrigues."""
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle_rad) * k + (1 - math.cos(angle_rad)) * k @ k


class Survey:
    def __init__(self, export: Path):
        self.export = export
        camera = json.loads((export / 'camera.json').read_text())
        self.k = Intrinsics(camera['width'], camera['height'], camera['fx'], camera['fy'],
                            camera['cx'], camera['cy'])
        self.truth = json.loads((export / 'truth_coco.json').read_text())
        self.categories = {c['id']: c['name'] for c in self.truth['categories']}
        self.rows = {int(r['frame']): r for r in read_frames(export / 'frames.csv')}
        self.altitude = round(float(np.median([-float(r['ekf_d']) for r in self.rows.values()
                                              if int(r['nav_state']) == 14 and -float(r['ekf_d']) > 5])))

    def locate(self, run: str, pose: str = 'ekf', height: bool = True, tilt_deg: float = 0.0,
               tilt_mode: str = 'systematic', height_offset: float = 0.0, rng=None):
        """Ground points of every box; also each box's ray angle off nadir and camera height."""
        out = []
        for box in load_boxes(self.export, run, self.truth):
            row = self.rows[box['frame']]
            cls = self.categories[box['category_id']]
            rotation = rotation_matrix([float(row[f'gimbal_q{c}']) for c in 'wxyz'])
            if tilt_deg:
                azimuth = 0.0 if tilt_mode == 'systematic' else rng.uniform(0, 2 * math.pi)
                axis = np.array([math.sin(azimuth), math.cos(azimuth), 0.0])  # horizontal
                rotation = tilt(axis, math.radians(tilt_deg)) @ rotation
            position = camera_position(row, pose) - [0.0, 0.0, height_offset]
            x, y, w, h = box['bbox']
            ray = rotation @ pixel_ray(x + w / 2, y + h / 2, self.k)
            object_height = CLASS_HEIGHTS[cls] if height else 0.0
            point = intersect_plane(position, ray, -object_height / 2)
            if point is not None:
                out.append({**box, 'cls': cls, 'n': float(point[0]), 'e': float(point[1]),
                            'theta': math.atan2(math.hypot(ray[0], ray[1]), ray[2]),
                            'h': -position[2] - object_height / 2})
        return out


def objects_score(located, objects, k: Intrinsics) -> dict:
    merged = merge(located, k.width, k.height, MERGE_RADIUS, MIN_FRAMES)
    result = {}
    for cls in ('person', 'vehicle'):
        s = score_objects([o.position() for o in merged if o.cls == cls],
                          [(float(o['n']), float(o['e'])) for o in objects if o['class'] == cls])
        result[cls] = {'recall': s.recall, 'precision': s.precision,
                       'median': float(np.median(s.errors)) if s.matched else float('nan')}
    return result


def box_errors(located, objects) -> dict:
    """Per class: each box's horizontal error to its own object (truth boxes) or nearest (model)."""
    positions = {o['name']: (float(o['n']), float(o['e'])) for o in objects}
    result = {}
    for cls in ('person', 'vehicle'):
        boxes = [b for b in located if b['cls'] == cls]
        points = np.array([(b['n'], b['e']) for b in boxes])
        if boxes and boxes[0]['object']:
            own = np.array([positions[b['object']] for b in boxes])
            errors = np.hypot(*(points - own).T)
        else:
            grid = np.array([positions[o['name']] for o in objects if o['class'] == cls])
            errors = np.linalg.norm(points[:, None] - grid[None], axis=2).min(axis=1)
        result[cls] = (float(np.median(errors)), float(np.percentile(errors, 95)))
    return result


def shifts(base, moved):
    """Horizontal shift of each box between two runs over the same boxes."""
    return np.array([math.hypot(m['n'] - b['n'], m['e'] - b['e']) for b, m in zip(base, moved)])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('objects')
    parser.add_argument('exports', nargs='+')
    parser.add_argument('--run', default='full_frame')
    args = parser.parse_args()
    with open(args.objects, newline='') as f:
        objects = [o for o in csv.DictReader(f) if o['class'] in ('person', 'vehicle')]
    OUT.mkdir(parents=True, exist_ok=True)
    surveys = sorted((Survey(Path(e)) for e in args.exports), key=lambda s: s.altitude)

    print('ERROR BUDGET: per-box horizontal error median / p95 (m), and object-level median (m)')
    budget = [('A truth box, true pose', 'truth', 'true', True),
              ('B truth box, EKF2', 'truth', 'ekf', True),
              ('C model box, EKF2', args.run, 'ekf', True),
              ('B without height', 'truth', 'ekf', False),
              ('C without height', args.run, 'ekf', False)]
    rows = []
    for s in surveys:
        for label, run, pose, height in budget:
            located = s.locate(run, pose, height)
            per_box, per_object = box_errors(located, objects), objects_score(located, objects, s.k)
            rows.append({'altitude': s.altitude, 'case': label,
                         **{f'{c}_box_median': per_box[c][0] for c in per_box},
                         **{f'{c}_box_p95': per_box[c][1] for c in per_box},
                         **{f'{c}_object_median': per_object[c]['median'] for c in per_object}})
            print(f"  {s.altitude:2d} m  {label:24s}  person {per_box['person'][0]:.3f} / "
                  f"{per_box['person'][1]:.3f}  obj {per_object['person']['median']:.3f}   vehicle "
                  f"{per_box['vehicle'][0]:.3f} / {per_box['vehicle'][1]:.3f}  obj "
                  f"{per_object['vehicle']['median']:.3f}")
    with open(OUT / 'error_budget.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print('\nSENSITIVITY: per-box shift median, measured vs theory (m); object recall / median error')
    rng = np.random.default_rng(SEED)
    angle, height_rows, objects_rows = {}, {}, []
    for s in surveys:
        base = s.locate(args.run)
        for mode in ('systematic', 'random'):
            for delta in TILTS:
                moved = s.locate(args.run, tilt_deg=delta, tilt_mode=mode, rng=rng)
                measured = shifts(base, moved)
                theory = np.array([b['h'] * math.radians(delta) / math.cos(b['theta']) ** 2 for b in base])
                angle[(s.altitude, mode, delta)] = (np.median(measured), np.median(theory))
                score = objects_score(moved, objects, s.k)
                objects_rows.append((s.altitude, f'tilt {mode}', delta, score))
                print(f'  {s.altitude:2d} m  tilt {delta:.0f} deg {mode:10s}  shift {np.median(measured):.3f} '
                      f'vs {np.median(theory):.3f}   person R {100 * score["person"]["recall"]:5.1f} % '
                      f'err {score["person"]["median"]:.3f}   vehicle R {100 * score["vehicle"]["recall"]:5.1f} % '
                      f'err {score["vehicle"]["median"]:.3f}')
        for offset in HEIGHT_OFFSETS:
            moved = s.locate(args.run, height_offset=offset)
            measured = shifts(base, moved)
            theory = np.array([offset * math.tan(b['theta']) for b in base])
            height_rows[(s.altitude, offset)] = (np.median(measured), np.median(theory))
            score = objects_score(moved, objects, s.k)
            objects_rows.append((s.altitude, 'height', offset, score))
            print(f'  {s.altitude:2d} m  height +{offset:.1f} m           shift {np.median(measured):.3f} '
                  f'vs {np.median(theory):.3f}   person R {100 * score["person"]["recall"]:5.1f} % '
                  f'err {score["person"]["median"]:.3f}   vehicle R {100 * score["vehicle"]["recall"]:5.1f} % '
                  f'err {score["vehicle"]["median"]:.3f}')

    colors = {20: '#2b8fe0', 30: '#e8433b', 40: '#46a86a'}
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    for altitude in colors:
        if (altitude, 'systematic', TILTS[0]) not in angle:
            continue
        for mode, style in (('systematic', '-o'), ('random', '--s')):
            axes[0].plot(TILTS, [angle[(altitude, mode, d)][0] for d in TILTS], style,
                         color=colors[altitude], label=f'{altitude} m measured ({mode})')
        axes[0].plot(TILTS, [angle[(altitude, 'systematic', d)][1] for d in TILTS], ':',
                     color=colors[altitude], lw=2.5, alpha=0.6, label=f'{altitude} m theory h·δ/cos²θ')
        axes[1].plot(HEIGHT_OFFSETS, [height_rows[(altitude, o)][0] for o in HEIGHT_OFFSETS], '-o',
                     color=colors[altitude], label=f'{altitude} m measured')
        axes[1].plot(HEIGHT_OFFSETS, [height_rows[(altitude, o)][1] for o in HEIGHT_OFFSETS], ':',
                     color=colors[altitude], lw=2.5, alpha=0.6, label=f'{altitude} m theory Δh·tanθ')
        for mode, style in (('tilt systematic', '-o'), ('tilt random', '--s')):
            points = [(d, sc['person']['recall']) for a, kind, d, sc in objects_rows if a == altitude and kind == mode]
            axes[2].plot([0] + [p[0] for p in points], [1.0] + [p[1] for p in points], style,
                         color=colors[altitude], label=f'{altitude} m ({mode.split()[1]})')
    axes[0].set(xlabel='camera tilt error δ (deg)', ylabel='median shift per box (m)', title='Angle error')
    axes[1].set(xlabel='height error Δh (m)', ylabel='median shift per box (m)', title='Height error')
    axes[2].set(xlabel='camera tilt error δ (deg)', ylabel='object-level recall, people', title='Recall (2 m rule)',
                ylim=(-0.05, 1.05))
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle('Sensitivity of the ground position, simulation (village_seed2)', fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / 'sensitivity.png', dpi=120)
    print(f'\n-> {OUT / "error_budget.csv"}, {OUT / "sensitivity.png"}')


if __name__ == '__main__':
    main()
