import cv2
import numpy as np


def find_keypoints(img):
    detector = cv2.ORB_create(nfeatures=3000, scaleFactor=2)
    kpt, dsc = detector.detectAndCompute(img, None)

    return kpt, dsc

def keypoint_match(img0, img1):
    kpt0, dsc0 = find_keypoints(img0)
    kpt1, dsc1 = find_keypoints(img1)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(dsc0, dsc1)
    matches = sorted(matches, key=lambda x:x.distance)

    return kpt0, kpt1, matches

def find_homography(img0, img1):
    kpt0, kpt1, matches = keypoint_match(img0, img1)

    # Extract the matched keypoints
    src_pts = np.float32([kpt0[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
    dst_pts = np.float32([kpt1[m.trainIdx].pt for m in matches]).reshape(-1,1,2)

    # Find homography matrix and do perspective transform
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    return M, mask

def draw_matches(img0, img1, maxcount=50):
    kpt0, kpt1, matches = keypoint_match(img0, img1)
    img = cv2.drawMatches(img0, kpt0, img1, kpt1, matches[:maxcount], None, flags=2)

    return img

def draw_keypoints(img, rich_kpt=False):
    if rich_kpt:
        flag = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    else:
        flag = 0

    kpt, _ = find_keypoints(img)
    img = cv2.drawKeypoints(img.copy(), kpt, None, (0, 255, 0), flags=flag)

    return img
