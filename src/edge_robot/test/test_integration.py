import unittest
import rclpy
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import time

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16MultiArray

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
        self.received_pwm = None

    def tearDown(self):
        # Destroy the temporary node after each test
        self.node.destroy_node()

    def pwm_callback(self, msg):
        self.received_pwm = msg

    def test_imu_to_pwm_flow(self):
        """Test that a simulated tilt generates a corrective PWM command."""
        
        mock_imu_msg = Imu()
        mock_imu_msg.angular_velocity.y = 0.5 
        mock_imu_msg.linear_acceleration.x = 2.0
        mock_imu_msg.linear_acceleration.z = 9.5
        self.mock_imu_pub.publish(mock_imu_msg)

        start_time = time.time()
        
        rclpy.spin_once(self.node, timeout_sec=0.1)
        


        pass