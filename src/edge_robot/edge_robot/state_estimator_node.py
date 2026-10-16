import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray
from sensor_msgs.msg import Imu 
from edge_robot.kalman_filter import KalmanFilter

# Summary: Filter out the noisy 

class StateEstimatorNode(Node):

    def __init__(self, angle_uncert, bias_uncert):

        super().__init__('state_estimator_node')
        self.kf = KalmanFilter(angle_uncert, bias_uncert)
        # None until the first IMU frame arrives. We can't compute a dt
        # without a previous timestamp, so the first frame only establishes
        # the clock baseline instead of being treated as a huge time jump.
        self.last_time = None

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
        print("imu_callback")
       
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        # First frame: no previous timestamp, so just record the baseline
        # and wait for the next frame to produce a usable dt.
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # A large gap means the clock jumped (e.g. between replayed test
        # cases); skip rather than integrating over a bogus dt.
        if dt > 1.0:
            self.get_logger().warn(f"Large time jump detected: {dt}s. Skipping frame.")
            print("return")
            return   
        
        ang_velo = msg.angular_velocity.y                                                         # rad/s
        accel_angle = math.atan2(msg.linear_acceleration.x, msg.linear_acceleration.z)            # arctan
       
        self.kf.predict_next_angle(ang_velo, dt)
        self.kf.update(accel_angle)
        print("kf updated")
        pitch_msg = Float32()
        pitch_msg.data = self.kf.angle
        self.pitch_publisher.publish(pitch_msg)
        print(f"published pitch: {pitch_msg}")
        return

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode(1.0, 1.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()