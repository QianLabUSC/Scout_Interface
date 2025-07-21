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
import time
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from dataclasses import dataclass
from collections import defaultdict

@dataclass 
class Isometric:
    """Small simple isometric/rigid transformation structure"""
    t:np.ndarray = np.array([[0,0,0]]).transpose()
    R:np.ndarray = np.identity(3)

    def __post_init__(self):
        # Ensure the numpy array has a specific size, for example, size 5
        expected_size = 3
        if self.t.shape != (expected_size,1):
            raise ValueError(f"Translation must have exactly {expected_size}, elements")
        if self.R.shape != (expected_size,expected_size):
            raise ValueError(f"Rotation must have exactly {expected_size},{expected_size} elements")



class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        bestEffort = QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.photo_publisher = self.create_publisher(Image, 'scenario_image', bestEffort)
        self.scenario_image_publish_rate = 0.1  # TODO HYPERPARAMETER period in seconds
        self.pose_publisher = self.create_publisher(Pose, 'spirit/marker_robot', 10)
        self.pose_publisher_rate = 0.25 # TODO HYPERPARAMETER period in seconds
        self.br = CvBridge()
        self.webcam = GoProStream()
        self.frame = np.zeros((960, 480, 3), dtype=np.uint8)
        self.detected_frame_rate = 0
        self.frame_rate = 1/60.0
        self.marker_offsets_T_BMi = defaultdict(Isometric)
        self.robot_pose = Pose()

        # Filter variable
        self.filtered_pose = None
        self.low_pass_alpha = 0.4
        self.low_pass_alpha_R = 0.4

        #TODO Add these configs (origin and robot marker pose) into the yaml
        a = 0.050681
        b = 0.036168  + 0.075
        c = 0.071655 + 0.075
        
        cth = np.cos(np.pi * 70.0/180)
        sth = np.sin(np.pi * 70.0/180)
        
        # self.marker_offsets_T_BMi[10] = Isometric(np.array([[ a, 0, b]]).T,np.array([[cth,0,sth],[0,1,0],[-sth,0,cth]]))
        # self.marker_offsets_T_BMi[11] = Isometric(np.array([[ 0,-a, b]]).T,np.array([[1,0,0],[0,cth,-sth],[0,sth,cth]]))
        self.marker_offsets_T_BMi[12] = Isometric(np.array([[-a, 0, b]]).T,np.array([[cth,0,-sth],[0,1,0],[sth,0,cth]]))
        # self.marker_offsets_T_BMi[13] = Isometric(np.array([[ 0, a, b]]).T,np.array([[1,0,0],[0,cth,sth],[0,-sth,cth]]))
        # self.marker_offsets_T_BMi[14] = Isometric(np.array([[ 0, 0, c]]).T,np.array([[1,0,0],[0,1,0],[0,0,1]]))

        self.origin_backup_T_CO = Isometric(
            np.array([[0.59494335, -0.36280709, 0.98195445]]).T, 
            np.array(
               [[ 0.00611825, -0.55763577, -0.8300632 ],
                [-0.99347451, -0.0979292,   0.05846603],
                [-0.11389017,  0.82428891, -0.55459608]]
            )
        )
        self.origin_backup_T_OC = Isometric(
            -self.origin_backup_T_CO.R.transpose() @ self.origin_backup_T_CO.t,
             self.origin_backup_T_CO.R.transpose())

        # Timer for publishing frames, pose, and receiving frames
        self.timer = self.create_timer(self.scenario_image_publish_rate, self.publish_scenario_image)
        self.pose_timer = self.create_timer(self.pose_publisher_rate, self.publish_pose)
        self.cam_timer = self.create_timer(self.frame_rate, self.receive_frames)

    def receive_frames(self):
        ret, frame = self.webcam.image_capture()
        if ret:
            self.frame = frame
            
    def publish_scenario_image(self):
        try:
            # img = cv.rotate(self.frame, cv.ROTATE_90_CLOCKWISE)
            img = cv.rotate(self.frame, cv.ROTATE_90_COUNTERCLOCKWISE)

            cv.putText(
                    img,
                    "RoboLAND Testing",
                    (50, 50),
                    cv.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),  # Red text color
                    2,
                    cv.LINE_AA
                )
                
            self.photo_publisher.publish(self.br.cv2_to_imgmsg(img))
        except:
            self.get_logger().warn("error publishing IMAGE")

    def publish_pose(self):
        tic = time.time()
        frame = self.webcam.undistort_image(self.frame)
        img, corners, ids = self.webcam.find_aruco_tags(frame)
        if ids is None:
            self.get_logger().warn("No Detection")
            return
        
        self.get_logger().info(", ".join([str(id) for id in ids]))
        robot_id = None
        for key in self.marker_offsets_T_BMi.keys():
            if key in ids:
                robot_id = key
                break
        if robot_id is None:
            self.get_logger().warn("No Robot Marker Found")
            return
        self.get_logger().info("Robot Marker ID: " + str(robot_id))

        rvecs, tvecs, ids = self.webcam.calculate_rvecs_tvecs(img, corners, ids)
        robot_index = np.where(ids == robot_id)[0][0]
        distances, quaternions = self.webcam.get_distance_and_quaternion(rvecs=rvecs, tvecs=tvecs)
        if 0 in ids:
            origin_index = np.where(ids == 0)[0][0]
            self.get_logger().debug("***\norigin, q:"+np.array2string(quaternions[origin_index])+", t:"+np.array2string(tvecs[origin_index])+", R:"+ np.array2string(R.from_quat(quaternions[origin_index]).as_matrix() )+"\n***")

        quaternion = quaternions[robot_index]
        rot_R_CMi = R.from_quat(quaternion).as_matrix()
        translation_p_CMi = tvecs[robot_index]


        rot_R_OMi = self.origin_backup_T_OC.R @ rot_R_CMi
        rot_R_OB = rot_R_OMi @ (self.marker_offsets_T_BMi[robot_id].R.T)
        
        translation_p_OMi = self.origin_backup_T_OC.t + self.origin_backup_T_OC.R @ translation_p_CMi
        translation_p_OB = translation_p_OMi - rot_R_OB @ self.marker_offsets_T_BMi[robot_id].t
        quat_R_OB = R.from_matrix(rot_R_OB).as_quat()
        
        self.get_logger().debug("***\nMarker Pose,\nt:"+
              np.array2string(translation_p_OB.T)+"\nR:\n"+ np.array2string(rot_R_OB)+"***")
        if self.filtered_pose is None:
            self.filtered_pose = Isometric(translation_p_OB, rot_R_OB)
            rotate_filt = R.from_matrix(rot_R_OB)
        else:
            if rot_R_OB[:,2].T@self.filtered_pose.R[:,2] < 0.75:
                self.get_logger().warn("Robot >45 deg rotation, ignoring")
                return
                
            translate_filt = (1-self.low_pass_alpha)*self.filtered_pose.t + self.low_pass_alpha*translation_p_OB
            slerp_for_filter = Slerp([0,1],R.from_matrix(np.array([self.filtered_pose.R, rot_R_OB ])))
            rotate_filt = slerp_for_filter(self.low_pass_alpha_R)
            self.filtered_pose = Isometric(translate_filt, rotate_filt.as_matrix())

        rotate_filt_quat = rotate_filt.as_quat()
        self.robot_pose.position.x = self.filtered_pose.t[0,0]
        self.robot_pose.position.y = self.filtered_pose.t[1,0]
        self.robot_pose.position.z = self.filtered_pose.t[2,0]
        self.robot_pose.orientation.x = rotate_filt_quat[0]
        self.robot_pose.orientation.y = rotate_filt_quat[1]
        self.robot_pose.orientation.z = rotate_filt_quat[2]
        self.robot_pose.orientation.w = rotate_filt_quat[3]
        self.pose_publisher.publish(self.robot_pose)
        
        self.get_logger().info("Pose PUBLISHED")



class GoProStream:
    """
    Initialize an object to stream gopro video as a webcam, serial number is the last three numbers of the serial, port is the http port you want to send on
    """
    RES_DICT = {"1080":12, "720":7, "480":4}
    FOV_DICT = {"wide":0, "linear":4, "narrow":2, "superview":3}
    def __init__(self, serial_number: list[int] = [5,3,7], port: int = 7567):
        #Getting Gopro setup
        self.serial_number = serial_number
        self.port = port
        self.webcam = GoProWebcamPlayer([5,3,7], port)
        
        #TODO to config yaml
        self.resolution = self.RES_DICT["1080"]
        self.fov = self.FOV_DICT["wide"]
        
        self.start_stream()
        self.cap = cv.VideoCapture(self.webcam.player.url+"?overrun_nonfatal=1&fifo_size=50000000", cv.CAP_FFMPEG)
        # self.cap = cv.VideoCapture(self.webcam.player.url+"?overrun_nonfatal=1&flags=low_delay&fifo_size=500", cv.CAP_FFMPEG)
        # self.cap.set(cv.CAP_PROP_BUFFERSIZE,1)

        #Load in camera callibration settings
        #TODO CHANGE FILE PATH TO THE PICKLE SETTING FILE
        self.camMatrix, self.distCoeff = pickle.load(open("/home/qianlab/SpiritHighLevel/src/top_view_visualization/top_view_visualization/camera_calibration/calibration_settings/calibration.pkl", "rb"))

        #TODO HYPERPARAMETER alpha level for how much of original image to keep. 0 is lose some corners no black spots, 1 is keep corners add black spots (0 to 1)
        self.alpha_level = 1
        
        #AruCo Tag Parameters
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.arucoParams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_CONTOUR
        self.detector = cv.aruco.ArucoDetector(dictionary=self.arucoDict, detectorParams=self.arucoParams)

        #TODO Hyperparmeter for the size of the arucotag (Currently, scaling with a meter stick, do better)
        self.marker_size = 0.06153 
        self.newcameramtx, self.roi = cv.getOptimalNewCameraMatrix(self.camMatrix, self.distCoeff, (1920, 1080), self.alpha_level, (3840,2160))
        self.output_image_size = (3840, 2160) #(width, height)
        self.map1, self.map2 = cv.initUndistortRectifyMap(self.camMatrix, self.distCoeff, np.eye(3), self.newcameramtx, self.output_image_size, cv.CV_32FC1)
        
    """
    Starts video stream
    ros2_send toggles between sending over a topic and displaying with opencv.
    """
    def start_stream(self):
        self.webcam.open()
        self.webcam.play(resolution=self.resolution,fov=self.fov)

    def image_capture(self):
        ret, frame = self.cap.read()
        return ret, frame
    
    def undistort_image(self, img):
        try:
            dst = cv.remap(img, self.map1, self.map2, cv.INTER_LINEAR)
            # cv.imshow("Undistorted",dst)
            x, y, w, h = self.roi
            dst = dst[y:y+h, x:x+w]
            # cv.imshow("Undistorted ROI",dst)
            # cv.waitKey(0)
            return dst
        except Exception as e:
            self.get_logger().warn(str(e))
            self.get_logger().warn("No calibration/settings found")
            pass

    def find_aruco_tags(self, img):
        corners, ids, rejected = self.detector.detectMarkers(img)
        return img, corners, ids

    def calculate_rvecs_tvecs(self, img, corners, ids):
        if ids is None or len(corners) == 0:
            self.get_logger().warn("No ArUco markers detected!")
            return None, None, None
        objPoints = np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)
        
        success_ids, rvecs, tvecs = [], [], []
        for i in range(len(corners)):
            success, rvec, tvec = cv.solvePnP(objPoints, corners[i][0], self.camMatrix, self.distCoeff)
            
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
                success_ids.append(ids[i])

        # uncomment to draw bounding box + frames.
        for i in range(len(rvecs)):
            img = cv.drawFrameAxes(img, self.camMatrix, self.distCoeff, rvecs[i], tvecs[i], 0.5, 2)
        if ids is not None:
            img = cv.aruco.drawDetectedMarkers(img, corners, ids)
        # if you want to see the images
        cv.imshow("ArUco Detection", img)
        if cv.waitKey(1) & 0xFF == ord('q'): 
            cv.destroyAllWindows()
        
        return rvecs, tvecs, ids

    def quaternion_helper(self, rvec):
        quaternion = R.from_rotvec(rvec.reshape(3,))
        quaternion = quaternion.as_quat()
        return quaternion  
    
    def get_distance_and_quaternion(self, rvecs, tvecs):
        distances = [np.linalg.norm(tvec) for tvec in tvecs]
        quaternions = list(map(self.quaternion_helper, rvecs))
        return distances, quaternions


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
