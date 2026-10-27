"""Compare PX4's EKF2 estimate with the Gazebo ground truth in a recorded survey.

Run with ROS and px4_msgs sourced:

    python3 tools/check_pose.py data/bags/<name> [--altitude 30]

Only the survey part of the flight is used: offboard mode at survey altitude. Prints
- position error per NED axis: mean (systematic) and std (scatter), horizontal percentiles,
- a time-shift scan: the shift of the truth that minimises the error (0 if the clocks agree),
- heading error over the survey and on the straight lane segments only.
"""
import argparse
import math

import numpy as np
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus

import bagio

ALTITUDE_TOLERANCE = 0.5  # m, survey window: |z + altitude| below this
LANE_MIN_SPEED = 2.0      # m/s along x: the vehicle is on a lane, not at a turn
LANE_MAX_CROSS = 0.5      # m/s along y
SHIFTS = np.arange(-0.300, 0.301, 0.020)  # s, truth time minus estimate time


def load_truth(bag):
    times, values = [], []
    for msg in bagio.read(bag, bagio.TRUTH_TOPIC, Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = bagio.enu_flu_yaw_to_ned_frd(bagio.yaw_from_quat(q.w, q.x, q.y, q.z))
        times.append(bagio.header_time(msg))
        values.append((*bagio.enu_to_ned(p.x, p.y, p.z), yaw))
    return bagio.Series(times, values, angle_columns=(3,))


def load_estimate(bag):
    times, values = [], []
    for msg in bagio.read(bag, bagio.POSITION_TOPIC, VehicleLocalPosition):
        times.append(msg.timestamp_sample * 1e-6)
        values.append((msg.x, msg.y, msg.z, msg.vx, msg.vy))
    return times, values


def load_attitude_yaw(bag):
    times, yaws = [], []
    for msg in bagio.read(bag, bagio.ATTITUDE_TOPIC, VehicleAttitude):
        times.append(msg.timestamp_sample * 1e-6)
        yaws.append(bagio.yaw_from_quat(*msg.q))
    return times, yaws


def load_nav_state(bag):
    times, values = [], []
    for msg in bagio.read(bag, bagio.STATUS_TOPIC, VehicleStatus):
        times.append(msg.timestamp * 1e-6)
        values.append((msg.nav_state,))
    return bagio.Series(times, values)


def in_survey(t, z, nav_state, altitude):
    state = nav_state.last_before(t)
    return (state is not None and state[0] == bagio.NAV_STATE_OFFBOARD
            and abs(z + altitude) < ALTITUDE_TOLERANCE)


def position_errors(times, estimates, truth, shift=0.0):
    """Rows of estimate - truth (north, east, down), truth taken at t + shift."""
    rows = []
    for t, (x, y, z, _, _) in zip(times, estimates):
        true = truth.at(t + shift)
        if true is not None:
            rows.append((x - true[0], y - true[1], z - true[2]))
    return np.array(rows)


def report_position(times, estimates, truth):
    err = position_errors(times, estimates, truth)
    horizontal = np.hypot(err[:, 0], err[:, 1])
    print(f'position ({len(err)} samples, {times[-1] - times[0]:.0f} s)')
    for k, axis in enumerate(('north', 'east', 'down')):
        print(f'  {axis:5s}  mean {err[:, k].mean():+.3f} m  std {err[:, k].std():.3f} m')
    print(f'  horizontal  median {np.median(horizontal):.3f} m  '
          f'p95 {np.percentile(horizontal, 95):.3f} m  max {horizontal.max():.3f} m')


def report_time_shift(times, estimates, truth):
    means = []
    for shift in SHIFTS:
        err = position_errors(times, estimates, truth, shift)
        means.append(np.hypot(err[:, 0], err[:, 1]).mean())
    best = int(np.argmin(means))
    zero = int(np.argmin(np.abs(SHIFTS)))
    print(f'time shift  best {SHIFTS[best] * 1e3:+.0f} ms (mean horizontal {means[best]:.3f} m, '
          f'{means[zero]:.3f} m at 0 ms)')


def report_yaw(label, errors_deg):
    e = np.array(errors_deg)
    print(f'  {label:7s} ({len(e)} samples)  mean {e.mean():+.2f} deg  std {e.std():.2f} deg  '
          f'max |err| {np.abs(e).max():.2f} deg')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('--altitude', type=float, default=30.0, help='survey altitude (m)')
    args = parser.parse_args()

    truth = load_truth(args.bag)
    nav_state = load_nav_state(args.bag)
    est_times, est_values = load_estimate(args.bag)
    survey = [(t, v) for t, v in zip(est_times, est_values)
              if in_survey(t, v[2], nav_state, args.altitude)]
    if not survey:
        raise SystemExit('no survey samples: wrong --altitude, or the bag has no offboard flight')
    times, estimates = map(list, zip(*survey))

    report_position(times, estimates, truth)
    report_time_shift(times, estimates, truth)

    # Heading: vehicle_attitude against the truth; the velocity and altitude come from the
    # position estimate at the same time.
    estimate = bagio.Series(est_times, est_values)
    all_yaw, lane_yaw = [], []
    for t, yaw in zip(*load_attitude_yaw(args.bag)):
        true, est = truth.at(t), estimate.at(t)
        if true is None or est is None or not in_survey(t, est[2], nav_state, args.altitude):
            continue
        err = math.degrees(bagio.wrap_angle(yaw - true[3]))
        all_yaw.append(err)
        if abs(est[3]) >= LANE_MIN_SPEED and abs(est[4]) < LANE_MAX_CROSS:
            lane_yaw.append(err)
    print('heading')
    report_yaw('survey', all_yaw)
    report_yaw('lanes', lane_yaw)


if __name__ == '__main__':
    main()
