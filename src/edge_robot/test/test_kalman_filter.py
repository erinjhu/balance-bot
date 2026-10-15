"""Unit tests for the KalmanFilter.

The KalmanFilter is pure math: no ROS, no spinning, no timing. That means
every input is under our control and every output is deterministic, so we
can assert on real numbers (unlike the node tests, which only check wiring
and direction).

Each test should set up a known state, call predict_next_angle and/or
update, then assert on angle / bias / the P-matrix entries.
"""

import math
import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from edge_robot.kalman_filter import KalmanFilter


# Default uncertainties for most tests. Pick values you understand;
# large initial P means "trust incoming measurements heavily".
ANGLE_UNCERT = 1.0
BIAS_UNCERT = 1.0


@pytest.fixture
def kf():
    """A fresh filter for each test (function scope -> no state leaks between tests)."""
    return KalmanFilter(ANGLE_UNCERT, BIAS_UNCERT)


# ---------------------------------------------------------------------------
# Construction / initial state
# ---------------------------------------------------------------------------

def test_initial_state(kf):
    assert kf.angle == 0.0 and kf.bias == 0.0


def test_initial_covariance(kf):
    assert kf.p00 == ANGLE_UNCERT and kf.p11 == BIAS_UNCERT
    assert kf.p01 == 0.0 and kf.p10 == 0.0


# ---------------------------------------------------------------------------
# predict_next_angle  (gyro integration step)
# ---------------------------------------------------------------------------

ang_velo_values = [
    0.5,
    1.0
]

dt_values = [
    0.0,
    0.5
]

next_angle_values = [
    0.0,
    0.5
]

bias_values = [
    0.0,
    1.0
]

def test_predict_integrates_angle(kf):
    kf.predict_next_angle(ang_velo_values[1], dt_values[1])
    assert kf.angle == pytest.approx(next_angle_values[1])


def test_predict_subtracts_bias():
    kf0 = KalmanFilter(ANGLE_UNCERT, BIAS_UNCERT)
    kf0.bias = bias_values[0]
    kf0.predict_next_angle(ang_velo_values[1], dt_values[1])
    angle_0 = kf0.angle
    kf1 = KalmanFilter(ANGLE_UNCERT, BIAS_UNCERT)
    kf1.bias = bias_values[1]
    kf1.predict_next_angle(ang_velo_values[1], dt_values[1])
    angle_1 = kf1.angle
    assert angle_1 < angle_0


def test_predict_grows_uncertainty(kf):
    prev_p00 = kf.p00
    kf.predict_next_angle(ang_velo_values[1], dt_values[1])
    assert kf.p00 > prev_p00
    # prediction alone adds doubt. Capture p00 before, call predict,
    # assert p00 afterward is strictly larger.


def test_predict_develops_cross_covariance(kf):
    kf.predict_next_angle(ang_velo_values[1], dt_values[1])
    assert kf.p01 != 0
    assert kf.p10 != 0
    # p01 / p10 start at 0.0 but become non-zero after a predict with
    # non-zero p11 and dt (the filter derives the correlation itself).


def test_predict_zero_dt_leaves_angle_unchanged(kf):
    # dt == 0 means no time passed, so angle should not move.
    prev_angle = kf.angle
    kf.predict_next_angle(ang_velo_values[0], dt_values[0])
    new_angle = kf.angle
    assert prev_angle == new_angle  


# ---------------------------------------------------------------------------
# update  (accelerometer correction step)
# ---------------------------------------------------------------------------

def test_update_reduces_uncertainty(kf):
    # HINT: a measurement always lowers doubt. Capture p00 before update,
    # assert it is smaller afterward.
    assert False  # TODO


def test_update_with_zero_error_leaves_state_unchanged(kf):
    # HINT: if the measured angle already equals kf.angle, error == 0, so
    # angle and bias should not move.
    assert False  # TODO


def test_update_moves_estimate_toward_measurement(kf):
    # HINT: predict to some angle, then update with a *larger* measured
    # angle. The new angle should land strictly between the prediction and
    # the measurement (correction, never overshoot in one step).
    assert False  # TODO


# ---------------------------------------------------------------------------
# Behavior over many iterations (convergence) -- optional, higher value
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("true_angle", [0.0, 0.3, -0.5])
def test_converges_to_true_angle(kf, true_angle):
    # HINT: loop many times feeding predict(0 velocity, small dt) + update(true_angle).
    # After enough iterations kf.angle should approach true_angle.
    # Use pytest.approx with an abs= tolerance.
    assert False  # TODO
