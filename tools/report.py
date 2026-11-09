"""Step 10 of the offline pipeline: one self-contained HTML report of a survey.

    .venv/bin/python tools/report.py data/export/<name> --located full_frame_ekf [--map map] \\
        [--truth sim/gz_worlds/<world>_objects.csv]

Mission, top-down map, object list and map quality always; with --truth (simulation) also the
object-level evaluation. Writes data/export/<name>/report.html, images embedded, so the one file
can be sent on its own.
"""
import argparse
import base64
import csv
from datetime import datetime
import io
import json
from pathlib import Path

import jinja2
import matplotlib
import warnings
import numpy as np

from frames import is_survey_frame, read_frames
from usm_core.metrics import MATCH_LIMIT, score_objects

matplotlib.use('Agg')
with warnings.catch_warnings():  # the system's own matplotlib makes the 3D toolkit complain; unused
    warnings.simplefilter('ignore')
    import matplotlib.pyplot as plt  # noqa: E402  (after choosing the file-only backend)

ALTITUDE = 30.0
CLASSES = ('person', 'vehicle')
COLORS = {'person': '#e8433b', 'vehicle': '#2b8fe0'}
MARKERS = {'person': 'o', 'vehicle': 's'}
MAP_REACH = 55.0       # m shown around the origin
MAX_POINTS = 200_000   # points drawn; more only slows the figure down


def mission(rows: list[dict], camera: dict, altitude: float) -> dict:
    survey = [r for r in rows if is_survey_frame(r, altitude)]
    t = [float(r['t']) for r in survey]
    path = np.array([(float(r['ekf_n']), float(r['ekf_e'])) for r in survey])
    height = float(np.median([-float(r['ekf_d']) for r in survey]))
    across = height * camera['width'] / camera['fx']   # ground footprint at nadir [m]
    along = height * camera['height'] / camera['fy']
    extent = path.max(axis=0) - path.min(axis=0)
    return {
        'frames': len(survey), 'duration': t[-1] - t[0], 'altitude': height,
        'path_length': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
        'area': float((extent[0] + along) * (extent[1] + across)),
        'footprint': (across, along),
    }


def figure_png(fig) -> str:
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buffer.getvalue()).decode()


def top_down(aligned: dict, objects: list[dict], truth: list[dict] | None) -> str:
    points, colors = aligned['points'], aligned['colors']
    keep = (np.abs(points[:, :2]).max(axis=1) < MAP_REACH) & (np.abs(points[:, 2]) < 15)
    order = np.argsort(-points[keep, 2])[::-1]  # draw low points first, roofs on top
    chosen = np.flatnonzero(keep)[order]
    if len(chosen) > MAX_POINTS:
        chosen = chosen[np.linspace(0, len(chosen) - 1, MAX_POINTS).astype(int)]

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_facecolor('#1b1b1b')
    ax.scatter(points[chosen, 1], points[chosen, 0], c=colors[chosen] / 255, s=0.4, linewidths=0)
    cameras = aligned['cameras']
    ax.plot(cameras[:, 1], cameras[:, 0], color='#46c86a', lw=1, alpha=0.8, label='drone path (map)')
    for cls in CLASSES:
        found = [o for o in objects if o['cls'] == cls]
        ax.scatter([o['e'] for o in found], [o['n'] for o in found], s=90, marker=MARKERS[cls],
                   facecolors='none', edgecolors=COLORS[cls], linewidths=2, label=f'{cls} (reported)')
    if truth:
        ax.scatter([float(o['e']) for o in truth], [float(o['n']) for o in truth], s=14, marker='x',
                   color='white', linewidths=1, label='true position')
    ax.set_xlim(-MAP_REACH, MAP_REACH)
    ax.set_ylim(-MAP_REACH, MAP_REACH)
    ax.set_aspect('equal')
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.legend(loc='upper right', fontsize=8, facecolor='white', framealpha=0.85)
    return figure_png(fig)


def evaluate(objects: list[dict], truth: list[dict]) -> tuple[list[dict], dict]:
    """Per-class scores, and for each reported object the true object it matched."""
    rows, matched_to = [], {}
    for cls in CLASSES:
        reports = [o for o in objects if o['cls'] == cls]
        true = [t for t in truth if t['class'] == cls]
        s = score_objects([(o['n'], o['e']) for o in reports], [(float(t['n']), float(t['e'])) for t in true])
        positions = np.array([(o['n'], o['e']) for o in reports]).reshape(-1, 2)
        for r_index, report in enumerate(reports):
            if r_index in s.false_alarms:
                continue
            distances = [np.hypot(*(positions[r_index] - (float(t['n']), float(t['e'])))) for t in true]
            matched_to[id(report)] = (true[int(np.argmin(distances))]['name'], float(min(distances)))
        rows.append({
            'cls': cls, 'truths': s.truths, 'reports': s.reports, 'matched': s.matched,
            'recall': s.recall, 'precision': s.precision,
            'mean': float(s.errors.mean()) if s.matched else None,
            'median': float(np.median(s.errors)) if s.matched else None,
            'max': float(s.errors.max()) if s.matched else None,
            'missed': [true[i]['name'] for i in s.missed],
        })
    return rows, matched_to


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--located', default='full_frame_ekf', help='suffix of the objects file')
    parser.add_argument('--map', default='map', help='map folder inside the export')
    parser.add_argument('--truth', help='true object positions (simulation only)')
    parser.add_argument('--altitude', type=float, default=ALTITUDE, help='survey altitude (m)')
    args = parser.parse_args()

    export = Path(args.export)
    camera = json.loads((export / 'camera.json').read_text())
    objects = json.loads((export / f'objects_{args.located}.json').read_text())
    aligned = dict(np.load(export / args.map / 'aligned.npz'))
    alignment = json.loads((export / args.map / 'alignment.json').read_text())
    truth = None
    if args.truth:
        with open(args.truth, newline='') as f:
            truth = [o for o in csv.DictReader(f) if o['class'] in CLASSES]

    evaluation, matched_to = evaluate(objects, truth) if truth else (None, {})
    table = []
    for i, o in enumerate(sorted(objects, key=lambda o: (o['cls'], -o['n'], o['e'])), start=1):
        match = matched_to.get(id(o))
        table.append({**o, 'id': i, 'truth': match[0] if match else None,
                      'error': match[1] if match else None})

    environment = jinja2.Environment(
        loader=jinja2.FileSystemLoader(Path(__file__).parent / 'templates'),
        autoescape=True, undefined=jinja2.StrictUndefined)
    html = environment.get_template('report.html.j2').render(
        name=export.name, created=datetime.now().strftime('%Y-%m-%d %H:%M'),
        mission=mission(read_frames(export / 'frames.csv'), camera, args.altitude),
        counts={cls: sum(o['cls'] == cls for o in objects) for cls in CLASSES},
        objects=table, alignment=alignment, evaluation=evaluation, match_limit=MATCH_LIMIT,
        map_png=top_down(aligned, objects, truth), located=args.located, map_folder=args.map)
    out = export / 'report.html'
    out.write_text(html)
    print(f'{len(objects)} objects -> {out} ({out.stat().st_size / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
