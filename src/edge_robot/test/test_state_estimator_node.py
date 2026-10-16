import math
import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from edge_robot.state_estimator_node import StateEstimatorNode
from foxglove_case_loader import load_case, load_expected, sample_to_imu  # noqa: E402
from test_foxglove_cases import discover_cases, CASES_DIR

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32, Int16MultiArray
from sensor_msgs.msg import Imu 

EXPECTED_SAMPLES_COUNT = 5
ANGLE_UNCERT = 1.0
BIAS_UNCERT = 1.0 

class HarnessNode(Node):
    
    def __init__(self):
        super().__init__('test_harness_node')
        self.mock_imu_pub = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )
        self.pitch_sub = self.create_subscription(
            Float32,
            '/robot/state/pitch',
            self.pitch_callback,
            10
        )
        self.received_pitch = None

    def pitch_callback(self, msg):
        print("pitch_callback")
        self.received_pitch = msg
        return

@pytest.fixture(scope="module")
def ros_env():

    rclpy.init()
    state_estimator_node = StateEstimatorNode(ANGLE_UNCERT, BIAS_UNCERT)
    test_harness_node = HarnessNode()
    executor = SingleThreadedExecutor()
    executor.add_node(state_estimator_node)
    executor.add_node(test_harness_node)
    
    yield state_estimator_node, test_harness_node, executor

    executor.shutdown()
    executor.remove_node(state_estimator_node)
    executor.remove_node(test_harness_node)
    state_estimator_node.destroy_node()
    test_harness_node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def test_data():
    input_path = CASES_DIR / "test_1" / "test_1"
    description, samples = load_case(input_path)
    return samples

def test_data_parsing(test_data, ros_env):
    print("Test data parsing")
    state_estimator_node, test_harness_node, executor = ros_env
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["title"] == "stationary_upright"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["expected_pitch"] == 0
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["expected_pwm"] == 0
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["angular_velocity"] == { "x": 0.0, "y": 0.0, "z": 0.0 }
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["linear_acceleration"] == { "x": 0.0, "y": 0.0, "z": 9.81 }    
    executor.spin_once(timeout_sec=0.1)
    
def test_node(test_data, ros_env):
    import time

    state_estimator_node, test_harness_node, executor = ros_env
    test_harness_node.received_pitch = None

    # The estimator skips the very first frame (it only establishes the clock
    # baseline, since dt needs a previous timestamp). So publish two frames a
    # small dt apart: the first sets the baseline, the second produces a usable
    # dt and triggers a pitch publish.
    first_msg = sample_to_imu(dict(test_data[0], t=0.0))
    second_msg = sample_to_imu(dict(test_data[0], t=0.01))

    test_harness_node.mock_imu_pub.publish(first_msg)
    # Spin so the estimator processes the baseline frame before the next one.
    baseline_end = time.monotonic() + 0.5
    while time.monotonic() < baseline_end:
        executor.spin_once(timeout_sec=0.05)

    test_harness_node.mock_imu_pub.publish(second_msg)

    # The round trip spans two callbacks on two nodes (harness publishes IMU
    # -> estimator publishes pitch -> harness receives pitch). A single
    # spin_once only drives one work item, so keep spinning until the pitch
    # arrives or we time out.
    end = time.monotonic() + 2.0
    while test_harness_node.received_pitch is None and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)

    print(f"received: {test_harness_node.received_pitch}")
    assert test_harness_node.received_pitch is not None

