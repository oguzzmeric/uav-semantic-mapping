"""Which recorded frames belong to the survey, shared by the offline tools."""
import csv

from usm_core.projection import off_nadir_deg

NAV_STATE_OFFBOARD = 14
ALTITUDE_TOLERANCE = 0.5   # m, as in check_pose.py
MAX_OFF_NADIR_DEG = 10.0   # drops the frames taken while the gimbal is still turning down


def read_frames(path) -> list[dict]:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def is_survey_frame(row: dict, altitude: float) -> bool:
    """Offboard, at survey altitude, camera about straight down."""
    return (int(row['nav_state']) == NAV_STATE_OFFBOARD
            and abs(float(row['ekf_d']) + altitude) < ALTITUDE_TOLERANCE
            and off_nadir_deg([float(row[f'gimbal_q{c}']) for c in 'wxyz']) <= MAX_OFF_NADIR_DEG)
