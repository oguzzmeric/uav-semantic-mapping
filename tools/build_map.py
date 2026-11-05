"""Step 8 of the offline pipeline: a sparse 3D map from the survey's photos, with COLMAP on the CPU.

    .venv/bin/python tools/build_map.py data/export/<name> [--spacing 2] [--limit N]

Writes data/export/<name>/map/ (database and model). The map is in COLMAP's own frame, with no
metres yet; tools/align_map.py puts it in the world.

EKF2 positions only choose which photo pairs are compared. They do not enter the reconstruction,
so comparing the map's cameras with EKF2 afterwards compares two independent sources.
"""
import argparse
import json
from pathlib import Path
import shutil
import time

import numpy as np
import pycolmap

from frames import is_survey_frame, read_frames

ALTITUDE = 30.0
# Pairs: photos overlap up to one footprint apart (34.6 x 19.5 m at 30 m). Compare with the nearest
# neighbours only; on a repeating texture, far-apart photos mostly add false matches.
MAX_PAIR_DISTANCE = 25.0  # m
MAX_NEIGHBOURS = 30


def select_photos(rows: list[dict], spacing: float) -> list[dict]:
    """Survey frames at least `spacing` metres apart along the path."""
    selected = []
    for row in (r for r in rows if is_survey_frame(r, ALTITUDE)):
        here = np.array([float(row['ekf_n']), float(row['ekf_e'])])
        if not selected or np.linalg.norm(here - selected[-1][1]) >= spacing:
            selected.append((row, here))
    return [row for row, _ in selected]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export')
    parser.add_argument('--spacing', type=float, default=2.0, help='metres between photos')
    parser.add_argument('--limit', type=int, help='use only the first N photos, for a quick test')
    parser.add_argument('--out', default='map', help='output folder inside the export')
    args = parser.parse_args()

    export = Path(args.export)
    camera = json.loads((export / 'camera.json').read_text())
    photos = select_photos(read_frames(export / 'frames.csv'), args.spacing)[:args.limit]
    names = [p['image'] for p in photos]
    print(f'{len(names)} photos, {args.spacing} m apart', flush=True)

    out = export / args.out
    if out.exists():
        raise SystemExit(f'{out} exists')
    out.mkdir()
    database = out / 'database.db'
    intrinsics = f"{camera['fx']},{camera['fy']},{camera['cx']},{camera['cy']}"

    t0 = time.perf_counter()
    pycolmap.extract_features(
        database, export / 'images', image_names=names, camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=pycolmap.ImageReaderOptions(camera_model='PINHOLE', camera_params=intrinsics),
        device=pycolmap.Device.cpu)
    print(f'features: {time.perf_counter() - t0:.0f} s', flush=True)

    t0 = time.perf_counter()
    position = {p['image']: [float(p['ekf_n']), float(p['ekf_e']), float(p['ekf_d'])] for p in photos}
    with pycolmap.Database.open(database) as db:
        for image in db.read_all_images():
            db.write_pose_prior(pycolmap.PosePrior(
                corr_data_id=image.data_id, position=position[image.name],
                coordinate_system=pycolmap.PosePriorCoordinateSystem.CARTESIAN))
    pycolmap.match_spatial(
        database,
        pairing_options=pycolmap.SpatialPairingOptions(
            max_distance=MAX_PAIR_DISTANCE, max_num_neighbors=MAX_NEIGHBOURS, ignore_z=True),
        device=pycolmap.Device.cpu)
    print(f'matching: {time.perf_counter() - t0:.0f} s', flush=True)

    t0 = time.perf_counter()
    options = pycolmap.IncrementalPipelineOptions(
        ba_refine_focal_length=False, ba_refine_principal_point=False, ba_refine_extra_params=False)
    models = pycolmap.incremental_mapping(database, export / 'images', out, options)
    print(f'mapping: {time.perf_counter() - t0:.0f} s', flush=True)

    if not models:
        raise SystemExit('no model: the photos could not be connected')
    for index, model in sorted(models.items(), key=lambda m: -m[1].num_reg_images()):
        print(f'model {index}: {model.num_reg_images()}/{len(names)} photos, '
              f'{model.num_points3D()} points, mean track {model.compute_mean_track_length():.1f}, '
              f'reprojection error {model.compute_mean_reprojection_error():.2f} px')


if __name__ == '__main__':
    main()
