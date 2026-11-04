"""Speed, delay and GPU numbers of a live detection run, over the survey part of the flight.

    python3 tools/live_stats.py data/live/<run>.csv [--altitude 30]

The method was fixed before the first measurement (PROGRESS.md, 2026-11-03). Survey window:
offboard mode at survey altitude, as in check_pose.py. Delays in sim time are measured from
the frame's stamp; stage durations are wall-clock. data/live/<run>_gpu.csv is read if present.
"""
import argparse
import csv
from pathlib import Path

import numpy as np

NAV_STATE_OFFBOARD = 14
ALTITUDE_TOLERANCE = 0.5  # m, survey window: |z + altitude| below this
CAMERA_HZ = 10.0          # sim/gz_models/usm_gimbal
STAGES = ['convert_ms', 'preprocess_ms', 'inference_ms', 'postprocess_ms', 'viewer_ms']


def load(path) -> dict[str, np.ndarray]:
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def summary(values) -> str:
    return (f'median {np.median(values):6.1f}   p95 {np.percentile(values, 95):6.1f}   '
            f'max {np.max(values):6.1f}')


def rate(stamps) -> float:
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def report_gpu(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    peak = {}
    for r in rows:
        for item in filter(None, r['processes'].split(';')):
            name, mib = item.rsplit('=', 1)
            peak[name] = max(peak.get(name, 0), int(mib))
    total = [int(r['total_mib']) for r in rows]
    temp = [int(r['temp_c']) for r in rows]
    throttled = sum(int(r['throttle'], 16) != 0 for r in rows)
    print(f'\nGPU, whole run ({len(rows)} samples)')
    print(f'  VRAM total peak: {max(total)} MiB')
    for name, mib in sorted(peak.items(), key=lambda p: -p[1]):
        print(f'    {name:24s} {mib:5d} MiB')
    print(f'  load median {np.median([int(r["util_percent"]) for r in rows]):.0f} %, '
          f'temperature median {np.median(temp):.0f} C, max {max(temp)} C, '
          f'clock median {np.median([int(r["clock_mhz"]) for r in rows]):.0f} MHz')
    print(f'  samples with a throttle reason: {throttled}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv')
    parser.add_argument('--altitude', type=float, default=30.0, help='survey altitude (m)')
    args = parser.parse_args()

    d = load(args.csv)
    window = (d['nav_state'] == NAV_STATE_OFFBOARD) & \
        (np.abs(d['ekf_z'] + args.altitude) < ALTITUDE_TOLERANCE)
    if window.sum() < 2:
        raise SystemExit('no survey frames: wrong --altitude, or the flight was not in this run')
    s = {k: v[window] for k, v in d.items()}

    duration = s['stamp_s'][-1] - s['stamp_s'][0]
    processed = len(s['stamp_s'])
    print(f'Survey window: {processed} frames in {duration:.1f} s of sim time')
    print(f'Processed: {rate(s["stamp_s"]):.2f} Hz; '
          f'skipped: {100 * (1 - processed / (duration * CAMERA_HZ)):.0f} % '
          f'(camera {CAMERA_HZ:.0f} Hz)')
    if 'wall_s' in s:
        print(f'RTF: {duration / (s["wall_s"][-1] - s["wall_s"][0]):.3f}')
    print(f'Boxes per frame: person {s["n_person"].mean():.2f}, vehicle {s["n_vehicle"].mean():.2f}')

    print('\nDelay from the frame\'s stamp [ms, sim time]')
    print(f'  total (boxes published)  {summary((s["done_s"] - s["stamp_s"]) * 1e3)}')
    print(f'  until processing starts  {summary((s["start_s"] - s["stamp_s"]) * 1e3)}')

    print('\nStages [ms, wall clock]')
    for stage in STAGES:
        print(f'  {stage[:-3]:24s} {summary(s[stage])}')

    print('\nDrift over the survey, by quarter')
    for i, part in enumerate(np.array_split(np.arange(processed), 4), start=1):
        print(f'  {i}/4: {rate(s["stamp_s"][part]):.2f} Hz, '
              f'inference {np.median(s["inference_ms"][part]):.0f} ms, '
              f'viewer {np.median(s["viewer_ms"][part]):.0f} ms')

    gpu = Path(args.csv).with_name(f'{Path(args.csv).stem}_gpu.csv')
    if gpu.exists():
        report_gpu(gpu)


if __name__ == '__main__':
    main()
