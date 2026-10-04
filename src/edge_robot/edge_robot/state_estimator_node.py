import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray
from sensor_msgs.msg import Imu 
from edge_robot.kalman_filter import KalmanFilter

class StateEstimatorNode(Node):

    def __init__(self):

        super().__init__('state_estimator_node')
        self.kf = KalmanFilter()
        self.last_time = 0.0

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )   

        self.pitch_publisher = self.create_publisher(
            Float32,
            '/robot/state/pitch',
            10
        )

    def imu_callback(self, msg):
       
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        dt = current_time - self.last_time
        self.last_time = current_time

        # in the first iteration of the loop, current_time will be a huge numnber

        if dt > 1.0:      
            self.get_logger().warn(f"Large time jump detected: {dt}s. Skipping frame.")     
            return   
        
        ang_velo = msg.angular_velocity.y                                                         # rad/s
        accel_angle = math.atan2(msg.linear_acceleration.x, msg.linear_acceleration.z)            # arctan
       
        self.kf.predict_next_angle(ang_velo, dt)
        self.kf.update(accel_angle)

        pitch_msg = Float32()
        pitch_msg.data = self.kf.angle
        self.pitch_publisher.publish(pitch_msg)
       
        return

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()