"""Fly one recorded survey with no window open, from a clean start to the exported frames.

    python3 tools/fly_survey.py --world village_seed2 --altitude 30 --name village_seed2_alt30

Starts the headless simulation, the XRCE agent, the Gazebo bridge and the recorder, flies the
mission (it lands and exits by itself), stops everything it started and exports the bag to
data/export/<name>. Logs go to data/flights/<name>/. Refuses to start if a simulation is already
running, so it never touches someone else's session.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parent.parent
ROS = 'source /opt/ros/jazzy/setup.bash && source ~/px4_ws/install/setup.bash && source install/setup.bash'
AGENT = Path.home() / 'Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'


def running(name: str) -> bool:
    return subprocess.run(['pgrep', '-x', name], capture_output=True).returncode == 0


class Process:
    """A command in its own process group, so stopping it stops everything it started."""

    def __init__(self, name: str, command: str, logs: Path):
        self.name = name
        self.log = logs / f'{name}.log'
        self.popen = subprocess.Popen(['bash', '-c', command], cwd=REPO, start_new_session=True,
                                      stdout=open(self.log, 'w'), stderr=subprocess.STDOUT)

    def wait_for(self, text: str, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if text in self.log.read_text(errors='replace'):
                return
            if self.popen.poll() is not None:
                raise SystemExit(f'{self.name} exited early; see {self.log}')
            time.sleep(1)
        raise SystemExit(f'{self.name}: no "{text}" within {timeout:.0f} s; see {self.log}')

    def stop(self, sig=signal.SIGINT, timeout: float = 30):
        if self.popen.poll() is not None:
            return
        for s in (sig, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.popen.pid, s)
                self.popen.wait(timeout)
                return
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--world', required=True)
    parser.add_argument('--altitude', type=float, default=30.0)
    parser.add_argument('--name', required=True, help='bag and export name')
    args = parser.parse_args()

    if running('px4') or running('gz') or running('ruby'):
        raise SystemExit('a simulation is already running; not touching it')
    bag, export = REPO / 'data/bags' / args.name, REPO / 'data/export' / args.name
    if bag.exists() or export.exists():
        raise SystemExit(f'{bag} or {export} exists')
    logs = REPO / 'data/flights' / args.name
    logs.mkdir(parents=True, exist_ok=True)

    started = []
    try:
        sim = Process('sim', f'setup/run_sim.sh HEADLESS=1 PX4_GZ_WORLD={args.world}', logs)
        started.append(sim)
        print('simulation starting...', flush=True)
        sim.wait_for('Ready for takeoff', 600)
        started.append(Process('agent', f'{AGENT} udp4 -p 8888', logs))
        started.append(Process('bridge', f'{ROS} && ros2 launch usm_mission bridge.launch.py '
                                         f'world:={args.world}', logs))
        time.sleep(8)
        recorder = Process('record', f'{ROS} && ros2 launch usm_mission record.launch.py '
                                     f'out:={bag.relative_to(REPO)}', logs)
        started.append(recorder)
        time.sleep(10)
        if 'has no publisher' in recorder.log.read_text(errors='replace'):
            raise SystemExit(f'the camera is not bridged; see {logs}')
        print('flying...', flush=True)
        mission = Process('mission', f'{ROS} && ros2 run usm_mission survey_mission --ros-args '
                                     f'-p altitude:={args.altitude}', logs)
        started.append(mission)
        if mission.popen.wait(timeout=1800) != 0:
            raise SystemExit(f'the mission failed; see {mission.log}')
        time.sleep(3)
    finally:
        for process in reversed(started):
            # The agent ignores SIGINT; ROS launch files and the simulation stop cleanly on it.
            process.stop(signal.SIGTERM if process.name == 'agent' else signal.SIGINT)
    if running('px4') or running('gz'):
        print('warning: a simulation process is still running', file=sys.stderr)

    for line in (logs / 'mission.log').read_text(errors='replace').splitlines():
        if any(k in line for k in ('survey time', 'cross-track', 'off-nadir on lanes', 'landed')):
            print(line.split(']: ')[-1])
    print('exporting...', flush=True)
    subprocess.run(['bash', '-c', f'{ROS} && python3 tools/export_bag.py {bag.relative_to(REPO)}'],
                   cwd=REPO, check=True)
    print(f'done: {export}')


if __name__ == '__main__':
    main()
