import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray

class PIDNode(Node):

    def __init__(self):
        
        super().__init__('pid_node')

        self.kp = 1.0
        self.ki = 1.0
        self.kd = 1.0

        self.total_error = 0.0
        self.previous_error = 0.0

        self.current_pitch = 0.0
        self.target_pitch = 0.0

        self.pitch_subscriber = self.create_subscription(
            Float32,                    # message type
            '/robot/state/pitch',       # topic name
            self.pitch_callback,        # callback function
            10                          # QoS / queue size (max number of messages to hold in buffer in case the publisher generates data faster than the subscriber can process it)
        )

        self.pwm_publisher = self.create_publisher(
            Int16MultiArray,                # use Int16MultiArray for multiple PWM values since there are multiple motors
            '/motor/pwm',        # topic name
            10                              
            # QoS / queue size (max number of messages to hold in buffer in case the publisher generates data faster than the system can send it)
        )

        # period: 0.01s, frequency: 100Hz
        self.create_timer(0.01, self.control_loop)

    def calculate_output(self, target_pitch, current_pitch, dt):

        error = target_pitch - current_pitch
        p_output = self.kp * error

        # purpose of Ki:
        #   if there is still error after using Kp, Ki will turn 
        #   the motor until there is no error

        self.total_error += error * dt
        i_output = self.ki * self.total_error

        # purpose of Kd:
        #   Ki might cause it to overshoot and oscillate
        #   By using rate of change, Kd ensures that the error
        #   doesn't close too fast

        slope = (error - self.previous_error) / dt
        d_output = self.kd * slope

        self.previous_error = error

        return p_output + i_output + d_output

    def pitch_callback(self, msg):
        # self.get_logger().info("Called pitch_callback")
        self.current_pitch = msg.data # .data is Float32

    def control_loop(self):

        pwm_val = self.calculate_output(self.target_pitch, self.current_pitch, 0.01)

        # left and right motors
        msg = Int16MultiArray()
        msg.data = [int(pwm_val), int(pwm_val)]
        self.pwm_publisher.publish(msg)
        return 0

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()