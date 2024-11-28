import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 as cv
import numpy as np
from top_view_visualization.GoProInterface.webcam import GoProWebcamPlayer
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import threading

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        bestEffort = QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(Image, 'scenario_image', bestEffort)
        timer_period = 0.1  # seconds
        
        self.br = CvBridge()
        self.webcam = GoProStream()
        self.item = 1
        self.frame = np.zeros((960, 480, 3), dtype=np.uint8)
        cv.putText(
            self.frame,
            "testing frames",
            (50, 50),
            cv.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),  # Red text color
            2,
            cv.LINE_AA
        )
        
        # Start a separate thread for receiving frames
        self.thread = threading.Thread(target=self.receive_frames, daemon=True)
        self.thread.start()

        # Timer for publishing frames
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def receive_frames(self):
        while True:
            ret, frame = self.webcam.image_capture()
            if ret:
                frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
                cv.putText(
                    frame,
                    "RoboLAND Testing",
                    (50, 50),
                    cv.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),  # Red text color
                    2,
                    cv.LINE_AA
                )
                self.frame = frame

    def timer_callback(self):
        print("publish")
        self.publisher_.publish(self.br.cv2_to_imgmsg(self.frame))

           

            


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




class GoProStream:
    """
    Initialize an object to stream gopro video as a webcam, serial number is the last three numbers of the serial, port is the http port you want to send on
    """
    def __init__(self, serial_number: list[int] = [5,3,7], port: int = 7567):
        self.serial_number = serial_number
        self.port = port
        self.webcam = GoProWebcamPlayer([5,3,7], port)
        self.start_stream()
        self.cap = cv.VideoCapture(self.webcam.player.url+"?overrun_nonfatal=1&fifo_size=50000000", cv.CAP_FFMPEG)
    """
    Starts video stream
    ros2_send toggles between sending over a topic and displaying with opencv.
    """
    def start_stream(self):
        self.webcam.open()
        self.webcam.play()


    def image_capture(self):
        # print(self.webcam.player.url)
        ret, frame = self.cap.read()
        return ret, frame
