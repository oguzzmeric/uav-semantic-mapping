"""Frame-level precision, recall and AP of a detection run, against the ground-truth boxes.

    .venv/bin/python tools/eval_detections.py data/export/<name> --run full_frame

Simulation only. Rules (PROGRESS.md, 2026-11-04): a box is correct at IoU >= 0.5; people and
vehicles are reported separately; boxes matching an ignored object (cut by the frame edge or
hidden by a building) count neither way; a box on anything else, buildings included, is a false
alarm. Matching and AP are pycocotools'. Object-level recall needs geolocation and comes later.
"""
import argparse
from collections import defaultdict
import contextlib
import io
import json
from pathlib import Path

import numpy as np
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval

from usm_core.detection import Settings

IOU_INDEX = 0  # pycocotools' first IoU threshold is 0.5


def evaluate(truth: COCO, results: list, category: int) -> COCOeval:
    with contextlib.redirect_stdout(io.StringIO()):  # pycocotools prints its progress
        evaluator = COCOeval(truth, truth.loadRes(results), 'bbox')
        evaluator.params.catIds = [category]
        evaluator.evaluate()
        evaluator.accumulate()
        evaluator.summarize()
    return evaluator


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--run', default='full_frame', help='name of the detection run')
    args = parser.parse_args()

    export = Path(args.export)
    with contextlib.redirect_stdout(io.StringIO()):
        truth = COCO(str(export / 'truth_coco.json'))
    results = json.loads((export / f'detections_{args.run}.json').read_text())
    threshold = Settings().conf
    print(f'{args.run}: {len(truth.imgs)} frames; P and R at score >= {threshold}, IoU >= 0.5\n')
    print(f'{"class":8s} {"truth":>6s} {"TP":>6s} {"FP":>6s} {"FN":>6s} {"P":>6s} {"R":>6s} '
          f'{"AP50":>6s} {"AP50-95":>8s}')

    for category in truth.getCatIds():
        name = truth.cats[category]['name']
        ap = evaluate(truth, results, category)
        working = evaluate(truth, [r for r in results if r['score'] >= threshold], category)

        tp = fp = 0
        total = 0
        frames_found, frames_counted = defaultdict(int), defaultdict(int)
        # evalImgs holds one entry per frame and area range; index 0 of the area ranges is 'all'
        for e in working.evalImgs[:len(truth.imgs)]:
            if e is None:
                continue
            matches = e['dtMatches'][IOU_INDEX]
            ignored = e['dtIgnore'][IOU_INDEX].astype(bool)
            tp += int(((matches > 0) & ~ignored).sum())
            fp += int(((matches == 0) & ~ignored).sum())
            found = {int(m) for m, skip in zip(matches, ignored) if m > 0 and not skip}
            for gt_id, skip in zip(e['gtIds'], e['gtIgnore']):
                if not skip:
                    total += 1
                    obj = truth.anns[gt_id]['object']
                    frames_counted[obj] += 1
                    frames_found[obj] += gt_id in found
        fn = total - tp
        print(f'{name:8s} {total:6d} {tp:6d} {fp:6d} {fn:6d} {tp / max(tp + fp, 1):6.3f} '
              f'{tp / max(total, 1):6.3f} {ap.stats[1]:6.3f} {ap.stats[0]:8.3f}')
        rates = {o: frames_found[o] / frames_counted[o] for o in frames_counted}
        per_object[name] = [(o, frames_found[o], frames_counted[o]) for o in
                            sorted(rates, key=rates.get)]

    print('\nFrames each object was found in, lowest first (found / counted):')
    for name, rows in per_object.items():
        print(f'  {name}: ' + ', '.join(f'{o.split("_")[1]} {f}/{c}' for o, f, c in rows))
    never = [o for rows in per_object.values() for o, f, _ in rows if f == 0]
    print(f'Objects never found in any counted frame: {never or "none"}')


per_object = {}

if __name__ == '__main__':
    main()
