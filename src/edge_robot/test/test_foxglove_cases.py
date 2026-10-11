"""Data-driven pipeline tests: JSON IMU time series in, sign/range checks out.

Every folder under foxglove_imu_TCs/ (except "template") is one test case:
  <case>/<case>           input  (see foxglove_case_loader.py)
  <case>/<case>_expected  expected sign/range for the final pitch and pwm

To add a test, drop in a new folder with those two files. No Python needed.
"""

import sys
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from std_msgs.msg import Empty, Float32, Int16MultiArray
from sensor_msgs.msg import Imu

sys.path.insert(0, str(Path(__file__).resolve().parent))
from foxglove_case_loader import load_case, load_expected, sample_to_imu  # noqa: E402

# note: noqa (no quality assurance) tells linters to ignore the linting error
#   where the import isn't at the top of the file. in this case, the import
#   comes after changing the system path

CASES_DIR = Path(__file__).resolve().parents[1] / 'foxglove_imu_TCs'

# How long to keep spinning after the last sample so the 100 Hz PID timer
# can react to the final pitch before we read the result.
SETTLE_SEC = 0.5

# Each case's timestamps are shifted by a unique offset so the estimator
# never sees time go backwards between cases. The jump is > 1 s, so the
# estimator skips the first frame of every case (a known, deterministic
# behaviour) instead of computing a bogus dt.
CASE_TIME_STRIDE = 100.0


def discover_cases():
    """Return sorted (name, input_path, expected_path) for each case folder."""
    cases = []
    for folder in sorted(CASES_DIR.iterdir()):
        if not folder.is_dir() or folder.name == 'template':
            continue
        cases.append((folder.name, folder / folder.name,
                      folder / f'{folder.name}_expected'))
    return cases


def check_value(name, value, check):
    """Return a list of failure strings for one value vs. its sign/range check."""
    failures = []
    sign = check.get('sign', 'any')
    lo = check.get('min')
    hi = check.get('max')

    if sign == 'positive' and not value > 0:
        failures.append(f'{name}={value} expected positive')
    elif sign == 'negative' and not value < 0:
        failures.append(f'{name}={value} expected negative')
    elif sign == 'zero' and lo is None and hi is None and abs(value) > 1e-3:
        # "zero" with no explicit bounds means "approximately zero".
        failures.append(f'{name}={value} expected ~0')

    if lo is not None and value < lo:
        failures.append(f'{name}={value} below min {lo}')
    if hi is not None and value > hi:
        failures.append(f'{name}={value} above max {hi}')
    return failures

# 

@pytest.mark.launch_test
def generate_test_description():
    state_estimator = launch_ros.actions.Node(
        package='edge_robot', executable='state_estimator',
        name='state_estimator_node')
    pid_controller = launch_ros.actions.Node(
        package='edge_robot', executable='pid', name='pid_node')

    return launch.LaunchDescription([
        state_estimator,
        pid_controller,
        launch_testing.actions.ReadyToTest(),
    ])


class TestFoxgloveCases(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    # - Make a node for the test harness
    # - Make publishers for IMU mock data and the signal to reset the PID error
    # - Make subscribers for the PWM and pitch to check the output data later

    def setUp(self):
        self.node = rclpy.create_node('foxglove_case_harness')
        self.imu_pub = self.node.create_publisher(Imu, '/imu/data_raw', 10)
        self.reset_pub = self.node.create_publisher(Empty, '/pid/reset', 10)
        self.node.create_subscription(
            Int16MultiArray, '/motor/pwm', self._on_pwm, 10)
        self.node.create_subscription(
            Float32, '/robot/state/pitch', self._on_pitch, 10)
        self.pwm = None
        self.pitch = None

    def tearDown(self):
        self.node.destroy_node()

    def _on_pwm(self, msg):
        self.pwm = msg.data[0]

    def _on_pitch(self, msg):
        self.pitch = msg.data

    # time.monotonic() 
    #   - time.time() uses the system clock, which can jump forwards or backwards
    #     when the computer syncs its time with the internet
    #   - time.monotonic() is just elapsed time from an arbitraty starting point.
    #     the value will only increase and won't go backwards. this will prevent it
    #     from messing up the dt values which are needed for PID.

    # rclpy.spin_once()
    #   - rclpy.spin() makes the node take over the thread in an infinite loop that
    #     runs the callback functions. no other code will run until the node shuts
    #     down
    #   - rclpy.spin_once() runs one cycle where it processes only the first 
    #     callback that gets triggered.

    # - since the script needs to publish mock IMU data but also check the pitch and
    #   PWM values that the node publishes, don't use rclpy.spin()
    # - the loop ensures that the test publishes the mock data, listens for the node
    #   outputs, and repeats. 
    # - if you used rclpy.spin(), it would only be listening for node outputs, and the
    #   mock data wouldn't get published. the timeout ensures that it stops publishing 
    #   mock data so that it can listen.
    # - the timeout_sec prevents busy-waiting. if there are no messages in the queue,
    #   continue to the next line of code in the python script. if there are messages
    #   in the queue, trigger the callback and then go to the next iteration of the 
    #   while loop.


    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def _run_case(self, samples, time_offset):
        """Replay samples in (roughly) real time and return (pitch, pwm)."""
        # Fresh PID integral/derivative state for this case.
        for _ in range(10):
            self.reset_pub.publish(Empty())
            self._spin_for(0.02)
        self.pwm = None
        self.pitch = None

        prev_t = samples[0].get('t', 0.0)
        for sample in samples:
            t = sample.get('t', 0.0)
            shifted = dict(sample, t=t + time_offset)
            self.imu_pub.publish(sample_to_imu(shifted))
            # Wait the real gap between samples so the PID timer keeps up.
            self._spin_for(max(t - prev_t, 0.0))
            prev_t = t

        self._spin_for(SETTLE_SEC)
        return self.pitch, self.pwm

    def test_all_cases(self):
        cases = discover_cases()
        self.assertTrue(cases, f'No test cases found in {CASES_DIR}')

        for index, (name, input_path, expected_path) in enumerate(cases):
            with self.subTest(case=name):
                _, samples = load_case(input_path)
                expected = load_expected(expected_path)
                self.assertTrue(samples, f'{name}: no samples')

                pitch, pwm = self._run_case(
                    samples, time_offset=CASE_TIME_STRIDE * (index + 1))

                self.assertIsNotNone(pitch, f'{name}: no pitch received')
                self.assertIsNotNone(pwm, f'{name}: no pwm received')

                failures = []
                failures += check_value('pitch', pitch, expected['pitch'])
                failures += check_value('pwm', pwm, expected['pwm'])
                self.assertFalse(failures, f'{name}: ' + '; '.join(failures))
