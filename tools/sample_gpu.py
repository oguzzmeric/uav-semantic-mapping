"""Log the GPU's memory per process, load, temperature and clock about once a second.

Started by live.launch.py; stop with Ctrl+C. Summarised by tools/live_stats.py.

    python3 tools/sample_gpu.py data/live/<run>_gpu.csv
"""
import argparse
import csv
from pathlib import Path
import signal
import subprocess
import time

COLUMNS = ['wall_s', 'total_mib', 'util_percent', 'temp_c', 'clock_mhz', 'throttle', 'processes']
GPU_QUERY = 'memory.used,utilization.gpu,temperature.gpu,clocks.gr,clocks_throttle_reasons.active'


def gpu_state() -> list[str]:
    out = subprocess.run(['nvidia-smi', f'--query-gpu={GPU_QUERY}', '--format=csv,noheader,nounits'],
                         capture_output=True, text=True, check=True).stdout
    return [v.strip() for v in out.split(',')]


def process_memory() -> str:
    """'name=MiB;name=MiB'; pmon also lists graphics processes such as Gazebo, unlike query-compute-apps."""
    out = subprocess.run(['nvidia-smi', 'pmon', '-c', '1', '-s', 'm'],
                         capture_output=True, text=True, check=True).stdout
    processes = []
    for line in out.splitlines():
        fields = line.split()  # gpu pid type fb ccpm name...
        if len(fields) >= 6 and not line.startswith('#') and fields[3] != '-':
            processes.append(f"{' '.join(fields[5:])}={fields[3]}")
    return ';'.join(processes)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('out')
    out = Path(parser.parse_args().out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'x', newline='') as f:  # never overwrite a measurement
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        try:
            while True:
                writer.writerow([f'{time.time():.1f}', *gpu_state(), process_memory()])
                f.flush()
                time.sleep(0.5)  # pmon itself takes about half a second
        except (KeyboardInterrupt, subprocess.CalledProcessError):
            # Ctrl+C reaches this process twice, from the terminal and from ros2 launch, and
            # can stop nvidia-smi mid-call.
            signal.signal(signal.SIGINT, signal.SIG_IGN)


if __name__ == '__main__':
    main()
