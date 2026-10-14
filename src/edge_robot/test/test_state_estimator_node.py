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
        print("pitch_callback")
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

def test_data_parsing(test_data, ros_env):
    print("Test data parsing")
    state_estimator_node, test_harness_node, executor = ros_env
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["title"] == "falling_bwd"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["expected_pitch"] == "massive negative"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["expected_pwm"] == "massive negative"
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["angular_velocity"] == { "x": 0.0, "y": -2.5, "z": 0.0 }
    assert test_data[EXPECTED_SAMPLES_COUNT - 1]["linear_acceleration"] == { "x": -1.70, "y": 9.66, "z": 0 }    
    executor.spin_once(timeout_sec=0.1)
    
def test_node(test_data, ros_env):
    print("Test node")
    state_estimator_node, test_harness_node, executor = ros_env
    stationary_upright_msg = sample_to_imu(test_data[0])
    test_harness_node.mock_imu_pub.publish(stationary_upright_msg)
    executor.spin_once(timeout_sec=1)
    print(f"received: {test_harness_node.received_pitch}")
    assert test_harness_node.received_pitch is not None
