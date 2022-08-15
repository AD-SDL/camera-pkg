import cv2
import numpy as np


def find_fiducials(img):
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters_create()

    if len(img.shape) == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)

    return corners, ids

def draw_fiducials(img, corners, ids):
    if len(corners) <= 0:
        return

    cv2.aruco.drawDetectedMarkers(img, corners, ids)

    # Made by hand. Should be calculated by calibration for better results
    cameraMatrix = np.array([
        [ 1000,    0, img.shape[0]/2],
        [    0, 1000, img.shape[1]/2],
        [    0,    0,              1],
    ])
    # Distortion coefficients as 0 unless known from calibration
    distCoeffs = np.zeros((4, 1))

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs)
    for rvec, tvec in zip(rvecs, tvecs):
        cv2.aruco.drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 0.05)
