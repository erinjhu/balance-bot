import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ObservabilityNode(Node):
    def __init__(self):
        super().__init__('observability_node')
        
        # purpose of this publisher: system metrics
        # this publisher will publish later when you do publisher_.publish(msg)
        # if the network connection lags, ROS 2 will hold 10 messages to publish before discarding the oldest ones

        self.publisher_ = self.create_publisher(Float32MultiArray, 'system_metrics', 10)
        
        # 1Hz timer to publish once every second

        self.timer = self.create_timer(1.0, self.timer_callback)

        # last_total: will be used to track the total number of ticks (as of the previous 
        #   second because it only takes a measurement each second)
        # last_idle: will be used to track the total number of ticks (also as of previous second)
        # example: CPU has done 1000 ticks since it booted up (last_total) and 500 of those ticks
        #   were spent doing nothing
        
        self.last_total, self.last_idle = self.get_cpu_times()
        self.get_logger().info("Observability Node started, publishing at 1Hz.")

    def get_cpu_times(self):
        """Reads /proc/stat to get total and idle CPU times."""

        # /proc/stat is a Linux file with system info
        # example of a the first line: cpu  4705 356 3310 132649 0 0 0 0 0 0
        #   each number is a system state
        #   4705: user time
        #   356: nice time
        #   3310: system time
        #   132649: idle time
        # times = list(map(float, f.readline().split()[1:5]))
        #   f.readline(): read one line from the file 'f'
        #   .split(): make the string into a list ['cpu', '4705', '356', '3310', '132649']
        #   [1:5]: take the elements at indices 1, 2, 3, and 4 to get ['4705', '356', '3310', '132649']
        #   list(): make into a python list of numbers [4705.0, 356.0, 3310.0, 132649.0]
        # return 
        #   sum(times): get the total of all the times to calculate percentage of idle time later
        #   times[3]: idle time

        with open('/proc/stat', 'r') as f:
            times = list(map(float, f.readline().split()[1:5]))
            return sum(times), times[3]

    def get_mem_usage(self):
        """Reads /proc/meminfo to calculate used memory percentage."""

        # /proc/stat is a Linux file with memory info

        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()   
            total = int(lines[0].split()[1]) # total memory
            free = int(lines[1].split()[1])  # free memory
            return 100.0 * (1.0 - (free / total)) # percentage of used memory

    def timer_callback(self):

        # get the total and idle time from the /proc/stat system file

        total, idle = self.get_cpu_times()

        cpu_usage = 0.0

        # make sure that total ticks is always increasing

        if total > self.last_total:

            # get the percentage of cpu usage
            # use the current minus previous idle and total values 
            #   find the CPU usage percentage within the most recent second

            cpu_usage = 100.0 * (1.0 - (idle - self.last_idle) / (total - self.last_total))

        # update the prev variables
        
        self.last_total, self.last_idle = total, idle

        # get the percentage of memory usage

        mem_usage = self.get_mem_usage()

        # publish the message as an array of 32-bit floats

        msg = Float32MultiArray()
        msg.data = [round(cpu_usage, 2), round(mem_usage, 2)]
        self.publisher_.publish(msg)
        
        self.get_logger().info(f"Published CPU: {msg.data[0]}%, Mem: {msg.data[1]}%")

def main(args=None):
    rclpy.init(args=args)
    node = ObservabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()