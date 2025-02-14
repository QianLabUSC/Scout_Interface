import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge 
from top_view_visualization.GoProInterface.webcam import GoProWebcamPlayer
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import cv2 as cv
import numpy as np
import pickle
import threading
from scipy.spatial.transform import Rotation as R


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        bestEffort = QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.photo_publisher = self.create_publisher(Image, 'scenario_image', bestEffort)
        self.scenario_image_publish_rate = 0.1  # TODO HYPERPARAMETER 1 publish per second 
        self.pose_publisher = self.create_publisher(Pose, 'pose_estimation', bestEffort)
        self.pose_publisher_rate = 0.01 # TODO HYPERPARAMETER 100 publish per second 
        
        self.br = CvBridge()
        self.webcam = GoProStream()
        self.frame = np.zeros((960, 480, 3), dtype=np.uint8)
        
        # Start a separate thread for receiving frames
        self.thread = threading.Thread(target=self.receive_frames, daemon=True)
        self.thread.start()

        # Timer for publishing frames
        self.robot_aruco_id = 1   #TODO REPLACE WITH ARUCO TAG ID ON THE ROBOT!!!!!!!!!
        self.timer = self.create_timer(self.scenario_image_publish_rate, self.publish_scenario_image)
        self.pose_publisher = self.create_timer(self.pose_publisher_rate, self.publish_pose)
        self.robot_pose = Pose()
        

        
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

    def publish_scenario_image(self):
        try:
            self.photo_publisher.publish(self.br.cv2_to_imgmsg(self.frame))
        except:
            print("error publishing IMAGE")

    def publish_pose(self):
        try:
            img, corners, ids = self.webcam.find_aruco_tags(self.frame)
            rvecs, tvecs, ids = self.webcam.calculate_rvecs_tvecs(img, corners, ids)
            distances, quaternions = self.webcam.get_distance_and_quaternion(rvecs=rvecs, tvecs=tvecs)
            quaternion = quaternions[ids.index(self.robot_aruco_id)]
            self.robot_pose.position.x = tvecs[0]
            self.robot_pose.position.y = tvecs[1]
            self.robot_pose.position.z = tvecs[2]
            self.robot_pose.orientation.x = quaternion[0]
            self.robot_pose.orientation.y = quaternion[1]
            self.robot_pose.orientation.z = quaternion[2]
            self.robot_pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(self.robot_pose)
        except:
            print("error publishing POSE")
            pass

            


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
        #Getting Gopro setup
        self.serial_number = serial_number
        self.port = port
        self.webcam = GoProWebcamPlayer([5,3,7], port)
        self.start_stream()
        self.cap = cv.VideoCapture(self.webcam.player.url+"?overrun_nonfatal=1&fifo_size=50000000", cv.CAP_FFMPEG)

        #Load in camera callibration settings
        #TODO CHANGE FILE PATH TO THE PICKLE SETTING FILE
        self.camMatrix, self.distCoeff = pickle.load(open("../calibration_settings/calibration.pkl", "rb"))
        
        #TODO HYPERPARAMETER alpha level for how much of original image to keep. 0 is lose some corners no black spots, 1 is keep corners add black spots (0 to 1)
        self.alpha_level = 0
        
        #AruCo Tag setup usig a 4x4_50 variation
        self.arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters_create()
        self.detector = cv.aruco.ArucoDetector(dictionary=self.arucoDict, detectorParams=self.arucoParams)

    """
    Starts video stream
    ros2_send toggles between sending over a topic and displaying with opencv.
    """
    def start_stream(self):
        self.webcam.open()
        self.webcam.play()

    def image_capture(self):
        ret, frame = self.cap.read()
        frame = self.undistort_image(frame)
        return ret, frame
    
    def undistort_image(self, img):
        try:
            h,  w = img.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.camMatrix, self.distCoeff, (w,h), self.alpha_level, (w,h))
            dst = cv.undistort(img, self.camMatrix, self.distCoeff, None, newcameramtx)
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            return dst
        except:
            print("No calibration/settings found")
            pass
            

    def find_aruco_tags(self, img):
        corners, ids, rejected = self.detector.detectMarkers(img)
        return img, corners, ids

    def calculate_rvecs_tvecs(self, img, corners, ids):
        if ids is None or len(corners) == 0:
            print("No ArUco markers detected!")
            return
        marker_size = 1
        objPoints = np.array([
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0]
        ], dtype=np.float32)
        
        success_ids, rvecs, tvecs = [], [], []
        for i in range(len(corners)):
            success, rvec, tvec = cv.solvePnP(objPoints, corners[i][0], self.camMatrix, self.distCoeff)
            
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
                success_ids.append(ids[i])

        # #uncomment out to draw bounding box + frames.
        # for i in range(len(rvecs)):
        #     img = cv.drawFrameAxes(img, self.camMatrix, self.distCoeff, rvecs[i], tvecs[i], 0.75, 2)
        # if ids is not None:
        #     img = cv.aruco.drawDetectedMarkers(img, corners, ids)
        
        
        # #if you want to see the images
        # cv.imshow("ArUco Detection", img)
        # cv.waitKey(0)
        # cv.destroyAllWindows()
        return rvecs, tvecs, ids

    def quaternion_helper(self, rvec):
        quaternion = R.from_rotvec(rvec.reshape(3,))
        quaternion = quaternion.as_quat()
        return quaternion  
    
    def get_distance_and_quaternion(self, rvecs, tvecs):
        distances = [np.linalg.norm(tvec) for tvec in tvecs]
        quaternions = list(map(self.quaternion_helper, rvecs))
        return distances, quaternions






