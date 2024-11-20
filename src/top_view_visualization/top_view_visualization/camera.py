import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 as cv
import numpy as np
from top_view_visualization.GoProInterface.webcam import GoProWebcamPlayer


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'scenario_image', 10)
        timer_period = 0.5 # seconds
        self.br = CvBridge()
        self.webcam = GoProStream()
        self.timer = self.create_timer(timer_period, self.timer_callback) #publish on the topic "scienario_image" every 0.1 seconds 
        self.item = 1

    def timer_callback(self):
        ret, frame = self.webcam.image_capture()
        if ret:
            half = cv.resize(frame, (0, 0), fx = 0.1, fy = 0.1)
            self.publisher_.publish(self.br.cv2_to_imgmsg(half))
        else:
            print("no")
        
           

            


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
    def __init__(self, serial_number: list[int] = [5,3,7], port: int = 9000):
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
        print(self.webcam.player.url)
        ret, frame = self.cap.read()
        return ret, frame
