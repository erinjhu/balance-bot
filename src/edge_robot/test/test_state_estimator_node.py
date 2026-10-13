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

class TestHarnessNode(Node):
    
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
        self.received_pitch = msg
        return

@pytest.fixture(scope="module")
def ros_env():

    rclpy.init()
    state_estimator_node = StateEstimatorNode(ANGLE_UNCERT, BIAS_UNCERT)
    test_harness_node = TestHarnessNode()
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

# def test_load_last_case():
#     print("test_load_last_case")
#     input_path = CASES_DIR / "test_1" / "test_1"
#     description, samples = load_case(input_path)
#     assert description == "One line describing what this scenario represents."
#     assert len(samples) == EXPECTED_SAMPLES_COUNT
#     assert samples[EXPECTED_SAMPLES_COUNT - 1]["title"] == "falling_bwd"
#     assert samples[EXPECTED_SAMPLES_COUNT - 1]["expected_pitch"] == "massive negative"
#     assert samples[EXPECTED_SAMPLES_COUNT - 1]["expected_pwm"] == "massive negative"
#     assert samples[EXPECTED_SAMPLES_COUNT - 1]["angular_velocity"] == { "x": 0.0, "y": -2.5, "z": 0.0 }
#     assert samples[EXPECTED_SAMPLES_COUNT - 1]["linear_acceleration"] == { "x": -1.70, "y": 9.66, "z": 0 }

# @pytest.mark.parametrize("label, ang_velo, accel, expected_pitch", [
#     ("stationary_upright", {"x": 0.0, "y": 0.0, "z": 0.0 }, { "x": 1.70, "y": 0, "z": 9.66}, 0),
#     ("still_tilt_10_deg_fwd", { "x": 0.0, "y": 0.0, "z": 0.0 }, { "x": 1.70, "y": 0, "z": 9.66}, 10),
#     ("still_tilt_10_deg_bwd", { "x": 0.0, "y": 0.0, "z": 0.0 }, { "x": -1.70, "y": 0, "z": 9.66}, -10),
#     ("falling_fwd", { "x": 0.0, "y": 2.5, "z": 0.0 }, { "x": 1.70, "y": 9.66, "z": 0 }, "massive_positive"),
#     ("falling_bwd", { "x": 0.0, "y": 2.5, "z": 0.0 }, { "x": 1.70, "y": 9.66, "z": 0 }, "massive_negative"),
# ])
def test_node(test_data, ros_env):
    print("test_node")
    state_estimator_node, test_harness_node, executor = ros_env
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["title"] == "falling_bwd"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["expected_pitch"] == "massive negative"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["expected_pwm"] == "massive negative"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["angular_velocity"] == { "x": 0.0, "y": -2.5, "z": 0.0 }
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["linear_acceleration"] == { "x": -1.70, "y": 9.66, "z": 0 }    
    executor.spin_once(timeout_sec=0.1)
