"""Live person and vehicle detection on the gimbal camera, shown in Rerun.

Takes the newest frame whenever the model is free: the model is slower than the camera, so
a queue would make the delay grow without bound. Skipped frames are not lost, the recorder
subscribes separately. The official numbers come from the offline tools; this node is for
the live preview and for measuring speed, delay and VRAM.

Run from the repo root with bridge.launch.py up (live.launch.py sets use_sim_time):

    ros2 launch usm_detection live.launch.py
    ros2 launch usm_detection live.launch.py rerun:=false
"""
from collections import deque
import csv
from datetime import datetime
import os
from pathlib import Path
import signal
import sys
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import rerun as rr
import rerun.blueprint as rrb
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from usm_core.detection import PERSON, Detection, Detector

# Best effort also matches PX4's publishers. Depth 1 keeps only the newest message.
NEWEST_ONLY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
CSV_COLUMNS = ['stamp_s', 'start_s', 'done_s', 'wall_s', 'convert_ms', 'preprocess_ms',
               'inference_ms', 'postprocess_ms', 'viewer_ms', 'n_person', 'n_vehicle',
               'nav_state', 'ekf_z']
VIEW_SCALE = 0.5   # images go to the viewer at 960x540; it keeps everything it receives in RAM
VIEW_JPEG_QUALITY = 80
VIEWER_MEMORY_LIMIT = '2GB'  # the viewer drops its oldest data at this limit
TRAIL_PERIOD = 1.0  # s of sim time between trail updates; each update resends the whole trail
PERSON_COLOR = (255, 60, 60)
VEHICLE_COLOR = (40, 160, 255)
DRONE_COLOR = (40, 200, 80)
TOTAL_DELAY_COLOR = (255, 170, 40)
INFERENCE_COLOR = (90, 200, 255)
VIEWER_LAYOUT = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Spatial2DView(origin='camera', name='Camera'),
        rrb.Vertical(
            rrb.Spatial2DView(origin='map', name='Top-down track (EKF2, m)'),
            rrb.TimeSeriesView(origin='delay', name='Delay (ms)'),
            rrb.TimeSeriesView(origin='rate', name='Processed frames (Hz)'),
        ),
        column_shares=[2, 1],
    ),
    collapse_panels=True,
)


def _seconds(t: Time) -> float:
    return t.nanoseconds / 1e9


def _start_viewer():
    # rr.spawn looks for the 'rerun' program on PATH; the venv's bin is there only if activated.
    os.environ['PATH'] = f"{Path(sys.executable).parent}{os.pathsep}{os.environ['PATH']}"
    rr.init('usm_live')
    rr.spawn(memory_limit=VIEWER_MEMORY_LIMIT)
    # Made active, not only default: the viewer otherwise restores the layout it saved last time.
    rr.send_blueprint(VIEWER_LAYOUT, make_active=True, make_default=True)
    rr.log('delay/total_ms', rr.SeriesLines(colors=[TOTAL_DELAY_COLOR]), static=True)
    rr.log('delay/inference_ms', rr.SeriesLines(colors=[INFERENCE_COLOR]), static=True)


def _to_message(header, detections: list[Detection]) -> Detection2DArray:
    """Boxes stamped with their frame's header: they describe the moment it was taken."""
    msg = Detection2DArray(header=header)
    for d in detections:
        item = Detection2D(header=header)
        item.bbox.center.position.x = (d.x1 + d.x2) / 2
        item.bbox.center.position.y = (d.y1 + d.y2) / 2
        item.bbox.size_x = d.x2 - d.x1
        item.bbox.size_y = d.y2 - d.y1
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = d.cls
        result.hypothesis.score = d.score
        item.results.append(result)
        msg.detections.append(item)
    return msg


class LiveDetector(Node):

    def __init__(self):
        super().__init__('live_detector')
        weights = self.declare_parameter('weights', 'models/yolo11l_nadir.pt').value
        csv_path = Path(self.declare_parameter(
            'csv', f'data/live/{datetime.now():%Y%m%d_%H%M%S}.csv').value)
        self._use_viewer = self.declare_parameter('rerun', True).value
        if not Path(weights).is_file():  # ultralytics would try to download an unknown name
            raise FileNotFoundError(f'{weights} not found; run from the repo root')

        self._detector = Detector(weights)
        self._detector.warmup()
        self._bridge = CvBridge()
        self._nav_state = -1
        self._ekf_position = None
        self._trail = []
        self._trail_sent_at = -TRAIL_PERIOD
        self._recent_stamps = deque(maxlen=10)

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(csv_path, 'x', newline='')  # never overwrite a measurement
        self._csv = csv.DictWriter(self._csv_file, CSV_COLUMNS)
        self._csv.writeheader()
        if self._use_viewer:
            _start_viewer()

        self._publisher = self.create_publisher(Detection2DArray, '/detections', 10)
        # /clock is delivered in the default callback group. Frames get their own group and the
        # executor a second thread, otherwise the sim clock would stand still during inference
        # and every delay would read as about zero.
        self.create_subscription(Image, '/camera/image', self._on_image, NEWEST_ONLY_QOS,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self._on_position, NEWEST_ONLY_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, NEWEST_ONLY_QOS)
        self.get_logger().info(f'ready, writing {csv_path}')

    def _on_position(self, msg: VehicleLocalPosition):
        self._ekf_position = (msg.x, msg.y, msg.z)

    def _on_status(self, msg: VehicleStatus):
        self._nav_state = msg.nav_state

    def _on_image(self, msg: Image):
        start = self.get_clock().now()
        t0 = time.perf_counter()
        image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        convert_ms = (time.perf_counter() - t0) * 1e3

        detections, timings = self._detector(image)
        self._publisher.publish(_to_message(msg.header, detections))
        done = self.get_clock().now()

        stamp = _seconds(Time.from_msg(msg.header.stamp))
        viewer_ms = self._log_to_viewer(stamp, image, detections, timings, done) \
            if self._use_viewer else 0.0

        n_person = sum(d.cls == PERSON for d in detections)
        ekf_z = self._ekf_position[2] if self._ekf_position else float('nan')
        self._csv.writerow({
            'stamp_s': f'{stamp:.3f}', 'start_s': f'{_seconds(start):.3f}',
            'done_s': f'{_seconds(done):.3f}', 'wall_s': f'{time.time():.3f}',
            'convert_ms': f'{convert_ms:.1f}',
            **{k: f'{v:.1f}' for k, v in timings.items()}, 'viewer_ms': f'{viewer_ms:.1f}',
            'n_person': n_person, 'n_vehicle': len(detections) - n_person,
            'nav_state': self._nav_state, 'ekf_z': f'{ekf_z:.3f}',
        })
        self._csv_file.flush()  # keep every row if the node is stopped mid-flight

    def _log_to_viewer(self, stamp: float, image: np.ndarray, detections: list[Detection],
                       timings: dict[str, float], done: Time) -> float:
        """Send one frame's data to Rerun on the sim-time timeline; returns the time spent [ms]."""
        t0 = time.perf_counter()
        rr.set_time('sim_time', duration=stamp)

        small = cv2.resize(image, None, fx=VIEW_SCALE, fy=VIEW_SCALE, interpolation=cv2.INTER_AREA)
        rr.log('camera', rr.Image(small, color_model='BGR').compress(jpeg_quality=VIEW_JPEG_QUALITY))
        if detections:
            rr.log('camera/boxes', rr.Boxes2D(
                array=np.array([[d.x1, d.y1, d.x2, d.y2] for d in detections]) * VIEW_SCALE,
                array_format=rr.Box2DFormat.XYXY,
                labels=[f'{d.cls} {d.score:.2f}' for d in detections],
                colors=[PERSON_COLOR if d.cls == PERSON else VEHICLE_COLOR for d in detections]))
        else:
            rr.log('camera/boxes', rr.Clear(recursive=False))

        if self._ekf_position:
            x, y, _ = self._ekf_position
            here = (y, -x)  # top-down map: east to the right, north up
            rr.log('map/drone', rr.Points2D([here], radii=rr.Radius.ui_points(5.0),
                                            colors=[DRONE_COLOR]))
            if stamp - self._trail_sent_at >= TRAIL_PERIOD:
                self._trail.append(here)
                self._trail_sent_at = stamp
                rr.log('map/track', rr.LineStrips2D([self._trail], colors=[DRONE_COLOR]))

        self._recent_stamps.append(stamp)
        if len(self._recent_stamps) > 1:
            span = self._recent_stamps[-1] - self._recent_stamps[0]
            if span > 0:
                rr.log('rate/processed_hz', rr.Scalars((len(self._recent_stamps) - 1) / span))
        rr.log('delay/total_ms', rr.Scalars((_seconds(done) - stamp) * 1e3))
        rr.log('delay/inference_ms', rr.Scalars(timings['inference_ms']))
        return (time.perf_counter() - t0) * 1e3

    def close(self):
        self._csv_file.close()


def main():
    rclpy.init()
    node = LiveDetector()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C reaches the node twice, from the terminal and from ros2 launch.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
