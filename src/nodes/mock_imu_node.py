import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class MockIMUNode(Node):
    def __init__(self):
        super().__init__('mock_imu_node')

        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer callback')
        self.publish_fake_data()

    def publish_fake_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angular_velocity.y = 0.5
        msg.linear_acceleration.x = 2.0     # tilt forward
        msg.linear_acceleration.z = 9.5     # slowly falling down
        self.imu_publisher.publish(msg)
        return

def main(args=None):
    rclpy.init(args=args)
    node = MockIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()