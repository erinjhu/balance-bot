import socket
import cv2
import numpy as np

# bind to the same IP and port the sender is using

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP video on port {UDP_PORT}...")

while True:
    # catch the incoming bytes (65535 is the max size of a UDP packet)

    data, addr = sock.recvfrom(65535)
    
    # convert the raw bytes back into a NumPy array

    np_data = np.frombuffer(data, dtype=np.uint8)
    
    # decode the JPEG back into an OpenCV image frame

    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
    
    if frame is not None:

        # Pop open a window to show the frame

        cv2.imshow("Received UDP Stream", frame)
        
        # press 'q' to quit

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()