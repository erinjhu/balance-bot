import math
import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from edge_robot.kalman_filter import KalmanFilter
from foxglove_case_loader import load_case, load_expected, sample_to_imu  # noqa: E402
from test_foxglove_cases import discover_cases, CASES_DIR

expected_samples_count = 5

@pytest.fixture
def kalman_filter(scope="class"):
    angle_uncert = 1.0
    bias_uncert = 1.0
    kf = KalmanFilter(angle_uncert, bias_uncert)

    yield kf

def test_load_last_case():
    print("test")
    input_path = CASES_DIR / "test_1" / "test_1"
    description, samples = load_case(input_path)
    assert description == "One line describing what this scenario represents."
    assert len(samples) == expected_samples_count
    assert samples[expected_samples_count - 1]["title"] == "falling_bwd"
    assert samples[expected_samples_count - 1]["expected_pitch"] == "massive negative"
    assert samples[expected_samples_count - 1]["expected_pwm"] == "massive negative"
    assert samples[expected_samples_count - 1]["angular_velocity"] == { "x": 0.0, "y": -2.5, "z": 0.0 }
    assert samples[expected_samples_count - 1]["linear_acceleration"] == { "x": -1.70, "y": 9.66, "z": 0 }

# todos