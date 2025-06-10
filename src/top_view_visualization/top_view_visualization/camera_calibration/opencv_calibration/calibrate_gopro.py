import numpy as np
import cv2 as cv
import glob
import pickle
import os
from top_view_visualization.GoProInterface.webcam import GoProWebcamPlayer
import time

def camera_check(webcam):
    cap = cv.VideoCapture(
        webcam.player.url+"?overrun_nonfatal=1&flags=low_delay&fifo_size=500",
        cv.CAP_FFMPEG)
    cap.set(cv.CAP_PROP_BUFFERSIZE,1)
    if cap.isOpened():
        return True, cap
    return False,[]

def get_images(cap, delete_prev_photos=False):
    if not os.path.exists("./calibration_images"):
        os.makedirs("./calibration_images")
    if delete_prev_photos:
        files = glob.glob('./calibration_images/*')
        for f in files:
            os.remove(f)
    ret,frame = cap.read()
    image_number = 0;    
    while cap.isOpened():
        ret, frame = cap.read()
        cv.imshow(f'img{image_number}',frame) #display the captured image
        wait_key = cv.waitKey(1)
        if wait_key & 0xFF == ord('y'): #save on pressing 'y' 
            cv.imwrite(f'calibration_images/picture_{image_number}.jpg',frame)
            cv.destroyAllWindows()
            image_number += 1
        elif wait_key & 0xFF == ord('q'): #quit by pressing q
            cv.destroyAllWindows()
            break
    cap.release()

def calibrate_and_save(internal_corner_row, internal_corner_col):
    CHECKERBOARD = (internal_corner_row,internal_corner_col)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('./calibration_images/*.jpg')
    print(images)
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (CHECKERBOARD[0],CHECKERBOARD[1]), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            print(fname)
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            cv.drawChessboardCorners(img, (CHECKERBOARD[0],CHECKERBOARD[1]), corners2, ret)
            # cv.imshow("title",img)
            # cv.waitKey(500)

    cv.destroyAllWindows()
    print("starting callibration")
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("ending callibration")

    if not os.path.exists("../calibration_settings"):
        os.makedirs("../calibration_settings")
    pickle.dump((mtx, dist), open("../calibration_settings/calibration.pkl", "wb"))
    
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (1920, 1080), 1, (3840,2160))

    img = cv.imread("calibration_images/picture_0.jpg")
    cv.imshow("Img", img)
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)
    cv.imshow("Undistorted", dst)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv.imshow("Undistorted ROI",dst)
    cv.waitKey(0)

        
if __name__ == "__main__":
    
    # RESOLUTION = 12 
    # FOV = 0
    # webcam = GoProWebcamPlayer([5,3,7], 7567)
    # webcam.open()
    # webcam.play(RESOLUTION,FOV)
    # status, cap = camera_check(webcam)
    # if status:
    #     get_images(cap, delete_prev_photos=False)
    # webcam.close()
    # del webcam
  
    calibrate_and_save(9,6)


