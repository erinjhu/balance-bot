import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import socket
import json
import time
import numpy as np
import math

use_mock_camera = True

class VideoStreamingNode(Node):
    def __init__(self):

        # super(): initialize parent Node class (imported from ROS2)

        super().__init__('video_node')
        
        # metadata_pub has same functionality as publisher_ but makes naming more clear

        self.metadata_pub = self.create_publisher(String, 'video_metadata', 10)
        
        # Set up the raw UDP socket
        # AF_INET = Internet Protocol (IPv4), SOCK_DGRAM = UDP

        # UDP: SOCK_DGRAM
        #   socket: network communication
        #   UDP is good for speed but is less reliable
        #   will discard lost packets and move on to the next one
        #   ensure video isn't laggy
        # AF_INET
        #   Internet Protocol
        #   use IPv4 addresses (main difference is that v4 is 32 bits and v6 is 128 bits)

        self.udp_ip = "127.0.0.1" 
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # use OpenCV to connect to computer camera

        self.cap = cv2.VideoCapture(0)
        
        # set resolution to 640x480 since UDP packets have a 65KB limit.

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 30 frames per second timer (1/30 of a second per frame)

        if use_mock_camera:
            self.timer = self.create_timer(1.0 / 30.0, self.timer_callback_mock)
        else:
            self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Video Node started. Streaming UDP to port 5005.")
        

    def timer_callback(self):

        # capture raw frame from the hardware
        #   frame: image data (pixels)
        #   ret: T/F - success/fail

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame. (See WSL note below)")
            return
            
        # compress image to 80% quality to save space for UDP

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

        # compress to JPEG

        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        
        if success:
            # convert the image to a raw bytestream

            image_bytes = encoded_image.tobytes()

            # send the image bytes over the network

            try:
                self.sock.sendto(image_bytes, (self.udp_ip, self.udp_port))
            except OSError as e:
                self.get_logger().error(f"Frame too large for single UDP packet: {e}")

            # publish metadata

            metadata = {
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
                "resolution": "640x480",
                "format": "jpeg",
                "size_bytes": len(image_bytes)
            }
            msg = String()

            # dumps(): convert python dictionary to string

            msg.data = json.dumps(metadata)
            self.metadata_pub.publish(msg)

    def timer_callback_mock(self):

        # black frame

        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # moving green circle

        t = time.time()
        x = int(320 + 200 * math.sin(t * 2)) 
        cv2.circle(frame, (x, 240), 50, (0, 255, 0), -1)

        # compress frame

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if success:

            # blast bytes over UDP

            image_bytes = encoded_image.tobytes()
            try:
                self.sock.sendto(image_bytes, (self.udp_ip, self.udp_port))
            except OSError as e:
                self.get_logger().error(f"UDP Error: {e}")

            # publish metadata

            metadata = {
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
                "resolution": "640x480",
                "format": "jpeg",
                "size_bytes": len(image_bytes)
            }
            msg = String()
            msg.data = json.dumps(metadata)
            self.metadata_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamingNode()
    rclpy.spin(node)
    
    # cleanup hardware resources on shutdown
    node.cap.release()
    node.sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()