"""Step 11 of the offline pipeline: object-level recall, precision and position error.

    .venv/bin/python tools/eval_objects.py data/export/<name> sim/gz_worlds/village_objects.csv \\
        --located full_frame_ekf

Simulation only. A report is correct within 2 m of a true object of its class (usm_core.metrics).
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from usm_core.metrics import MATCH_LIMIT, score_objects


def report(records: list[dict], objects: list[dict]):
    for cls in ('person', 'vehicle'):
        truths = [o for o in objects if o['class'] == cls]
        reports = [r for r in records if r['cls'] == cls]
        s = score_objects([(r['n'], r['e']) for r in reports],
                          [(float(o['n']), float(o['e'])) for o in truths])
        error = (f'error mean {s.errors.mean():.3f}, median {np.median(s.errors):.3f}, '
                 f'max {s.errors.max():.3f} m' if s.matched else 'no matches')
        print(f'  {cls:8s} truth {s.truths:3d}, reports {s.reports:3d}, matched {s.matched:3d}: '
              f'recall {100 * s.recall:5.1f} %, precision {100 * s.precision:5.1f} %, {error}')
        if s.missed:
            print(f'    missed: {[truths[i]["name"] for i in s.missed]}')
        if s.false_alarms:
            print('    false alarms (n, e, frames): '
                  + ', '.join(f'({reports[i]["n"]:.1f}, {reports[i]["e"]:.1f}, '
                              f'{reports[i]["frames"]})' for i in s.false_alarms))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('objects')
    parser.add_argument('--located', default='full_frame_ekf', help='suffix of the objects file')
    args = parser.parse_args()

    records = json.loads((Path(args.export) / f'objects_{args.located}.json').read_text())
    with open(args.objects, newline='') as f:
        objects = list(csv.DictReader(f))
    print(f'{args.located}: object level, match limit {MATCH_LIMIT} m')
    report(records, objects)


if __name__ == '__main__':
    main()
