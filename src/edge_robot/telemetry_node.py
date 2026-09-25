import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class TelemetryNode(Node):

    # __init__(self) runs once when the program starts

    def __init__(self):
        super().__init__('telemetry_node')
        
        # publish JointState messages to the 'joint_states' topic

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 10Hz (0.1 seconds)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):

        # empty package of data

        msg = JointState()
        
        # put current time in the message package

        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Name the 6 degrees of freedom

        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Generate sine waves for movement
        #   - output an array of 6 different motor positions
        #       - one for each angle
        #   - motor position is an angle in radians
        #       0: straight
        #       1.0: bend forward
        #       -1.0: bend backward
        #   - example: return value when 'elapsed' = 0
        #       [sin(0), sin(1), sin(2), sin(3), sin(4), sin(5)]


        elapsed = time.time() - self.start_time

        msg.position = [math.sin(elapsed + i) for i in range(6)]

        # publish and log
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published 6-DOF joints at {elapsed:.2f}s")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()