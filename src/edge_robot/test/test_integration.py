import unittest
import unittest
import rclpy
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import time

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16MultiArray, Empty

# note: pytest.mark is for metadata. in this part, it puts
#   the test under the launch_test custom group
@pytest.mark.launch_test
def generate_test_description():

    state_estimator = launch_ros.actions.Node(
        package='edge_robot',
        executable='state_estimator', 
        name='state_estimator_node'
    )
    
    pid_controller = launch_ros.actions.Node(
        package='edge_robot',
        executable='pid', 
        name='pid_node'
    )

    return launch.LaunchDescription([
        state_estimator,
        pid_controller,
        launch_testing.actions.ReadyToTest()
    ])

# unittest.TestCase: don't need to make a main method
class TestRobotPipeline(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context once for the test suite
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context when finished
        rclpy.shutdown()

    def setUp(self):
        # Create a temporary ROS node to act as the test harness
        self.node = rclpy.create_node('test_harness_node')
        
        self.mock_imu_pub = self.node.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )
        
        self.outputted_pwm_sub = self.node.create_subscription(
            Int16MultiArray,
            '/motor/pwm',
            self.pwm_callback,
            10
        )

        self.pitch_sub = self.node.create_subscription(
            Float32,
            '/robot/state/pitch',
            self.pitch_callback,
            10
        )

        # Lets each test start the PID from a clean slate (no leftover
        # integral windup from a previous test), so results are deterministic.
        self.reset_pub = self.node.create_publisher(
            Empty,
            '/pid/reset',
            10
        )

        self.received_pwm = None
        self.received_pitch = None

        # Trigger a reset and give ROS a moment to deliver it to the PID.
        for _ in range(10):
            self.reset_pub.publish(Empty())
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def tearDown(self):
        # Destroy the temporary node after each test
        self.node.destroy_node()

    def pwm_callback(self, msg):
        self.received_pwm = msg

    def pitch_callback(self, msg):
        self.received_pitch = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def make_imu_msg(self, accel_x, accel_z=9.5, ang_velo_y=0.0):
        """Build an IMU message with a real timestamp.

        accel_x controls the tilt direction:
          positive accel_x -> positive pitch -> one motor direction
          negative accel_x -> negative pitch -> the other direction
        """
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.angular_velocity.y = ang_velo_y
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.z = accel_z
        return msg

    def pump(self, imu_msg, iterations=100):
        """Publish the IMU message repeatedly while spinning the harness.

        Re-stamps each message so the estimator sees an advancing clock
        (a non-zero dt), then spins so our subscription callbacks fire.
        Returns the last PWM message received.
        """
        for _ in range(iterations):
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.mock_imu_pub.publish(imu_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.received_pwm

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------
    def test_imu_to_pwm_flow(self):
        """A simulated tilt should produce a non-zero motor command."""
        pwm = self.pump(self.make_imu_msg(accel_x=2.0))

        self.assertIsNotNone(pwm, "Never received PWM message")
        self.assertNotEqual(pwm.data[0], 0,
                            f"Expected a non-zero PWM for a tilt, got {pwm.data[0]}")

    def test_pitch_is_published(self):
        """The estimator half of the pipeline should publish a pitch angle."""
        self.pump(self.make_imu_msg(accel_x=2.0))

        self.assertIsNotNone(self.received_pitch, "Never received pitch message")
        self.assertNotEqual(self.received_pitch.data, 0.0,
                            "Expected a non-zero pitch estimate for a tilt")

    def test_both_motors_get_same_command(self):
        """The PID publishes [pwm, pwm]; both motors should match."""
        pwm = self.pump(self.make_imu_msg(accel_x=2.0))

        self.assertIsNotNone(pwm, "Never received PWM message")
        self.assertEqual(len(pwm.data), 2, "Expected two motor values")
        self.assertEqual(pwm.data[0], pwm.data[1],
                        f"Motors disagree: {pwm.data[0]} vs {pwm.data[1]}")

    # NOTE: Directional correctness ("bigger tilt -> stronger command",
    # "sign of command follows sign of error") is NOT tested here. Those are
    # properties of the PID math, and the integration pipeline runs against
    # two stateful nodes (the PID integral and the Kalman filter both carry
    # memory and converge over time), so sampling a single PWM value after a
    # fixed number of iterations races that convergence and is unreliable.
    # The directional behaviour is covered deterministically by the pure-math
    # unit tests in test_math.py (test_pid_* ). This file only verifies that
    # the nodes are wired together correctly end to end.
