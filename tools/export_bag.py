"""Export a recorded survey to images + per-frame poses for offline processing.

Run with ROS and px4_msgs sourced:

    python3 tools/export_bag.py data/bags/<name> [--out data/export/<name>]

Writes
    images/000000.png ...  every camera frame, lossless
    frames.csv             one row per image, all values interpolated to the image stamp
    camera.json            intrinsics from camera_info

frames.csv columns (NED world, FRD body, quaternions as w, x, y, z):
    frame, image, t          index, file name, simulation time (s)
    nav_state                PX4 flight mode (14 = offboard)
    ekf_n/e/d                EKF2 position (m)
    ekf_qw..qz               EKF2 vehicle attitude
    gimbal_qw..qz            measured camera orientation, in the world frame
    true_n/e/d, true_qw..qz  Gazebo pose of base_link, for evaluation only
Frames before the first or after the last pose sample are skipped.
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from px4_msgs.msg import GimbalDeviceAttitudeStatus, VehicleAttitude, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import CameraInfo, Image

import bagio

COLUMNS = ['frame', 'image', 't', 'nav_state',
           'ekf_n', 'ekf_e', 'ekf_d', 'ekf_qw', 'ekf_qx', 'ekf_qy', 'ekf_qz',
           'gimbal_qw', 'gimbal_qx', 'gimbal_qy', 'gimbal_qz',
           'true_n', 'true_e', 'true_d', 'true_qw', 'true_qx', 'true_qy', 'true_qz']


def load_series(bag):
    """All pose streams as interpolable series, keyed by name."""
    def collect(topic, msg_type, stamp, value):
        times, values = [], []
        for msg in bagio.read(bag, topic, msg_type):
            times.append(stamp(msg))
            values.append(value(msg))
        return times, values

    px4_sample = lambda m: m.timestamp_sample * 1e-6
    truth_t, truth = collect(bagio.TRUTH_TOPIC, Odometry, bagio.header_time, lambda m: m.pose.pose)
    return {
        'ekf_pos': bagio.Series(*collect(bagio.POSITION_TOPIC, VehicleLocalPosition, px4_sample,
                                         lambda m: (m.x, m.y, m.z))),
        'ekf_q': bagio.QuaternionSeries(*collect(bagio.ATTITUDE_TOPIC, VehicleAttitude, px4_sample,
                                                 lambda m: tuple(m.q))),
        # The status message has no sample stamp; the simulated gimbal publishes right after
        # reading its IMU.
        'gimbal_q': bagio.QuaternionSeries(*collect(bagio.GIMBAL_TOPIC, GimbalDeviceAttitudeStatus,
                                                    lambda m: m.timestamp * 1e-6,
                                                    lambda m: tuple(m.q))),
        'true_pos': bagio.Series(truth_t, [bagio.enu_to_ned(p.position.x, p.position.y, p.position.z)
                                           for p in truth]),
        'true_q': bagio.QuaternionSeries(truth_t, [bagio.enu_flu_quat_to_ned_frd(
            (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)) for p in truth]),
        'nav_state': bagio.Series(*collect(bagio.STATUS_TOPIC, VehicleStatus,
                                           lambda m: m.timestamp * 1e-6, lambda m: (m.nav_state,))),
    }


def write_camera_json(bag, path):
    info = next(bagio.read(bag, bagio.CAMERA_INFO_TOPIC, CameraInfo), None)
    if info is None:
        raise SystemExit(f'no {bagio.CAMERA_INFO_TOPIC} in the bag')
    camera = {
        'width': info.width, 'height': info.height,
        'fx': info.k[0], 'fy': info.k[4], 'cx': info.k[2], 'cy': info.k[5],
        'distortion_model': info.distortion_model, 'distortion': list(info.d),
        'frame_id': info.header.frame_id,
    }
    with open(path, 'w') as f:
        json.dump(camera, f, indent=2)


def to_bgr(msg):
    """sensor_msgs/Image (rgb8 or bgr8) as an OpenCV BGR array."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise SystemExit(f'unsupported image encoding {msg.encoding}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else image


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('--out', help='output directory, must not exist '
                                      '(default: data/export/<bag name>)')
    args = parser.parse_args()
    out = args.out or os.path.join('data', 'export', os.path.basename(os.path.normpath(args.bag)))
    os.makedirs(os.path.join(out, 'images'))  # fails if out exists: never overwrite an export

    write_camera_json(args.bag, os.path.join(out, 'camera.json'))
    series = load_series(args.bag)

    written = skipped = 0
    with open(os.path.join(out, 'frames.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        for msg in bagio.read(args.bag, bagio.CAMERA_TOPIC, Image):
            t = bagio.header_time(msg)
            values = {name: s.at(t) for name, s in series.items() if name != 'nav_state'}
            nav_state = series['nav_state'].last_before(t)
            if nav_state is None or any(v is None for v in values.values()):
                skipped += 1
                continue
            name = f'{written:06d}.png'
            cv2.imwrite(os.path.join(out, 'images', name), to_bgr(msg))
            writer.writerow(
                [written, name, f'{t:.6f}', nav_state[0]]
                + [f'{c:.4f}' for c in values['ekf_pos']] + [f'{c:.6f}' for c in values['ekf_q']]
                + [f'{c:.6f}' for c in values['gimbal_q']]
                + [f'{c:.4f}' for c in values['true_pos']] + [f'{c:.6f}' for c in values['true_q']])
            written += 1

    print(f'{written} frames written to {out} ({skipped} skipped outside the pose data)')


if __name__ == '__main__':
    main()
