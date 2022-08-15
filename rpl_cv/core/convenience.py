import cv2
import numpy as np

from utils.features import find_homography


def debug_show(img):
    cv2.imshow('', img)
    cv2.waitKey(0)

def rotate(img, angle):
    h, w = img.shape[:2]

    M = cv2.getRotationMatrix2D((w/2, h/2), angle, 1)
    img = cv2.warpAffine(img, M, (w, h))

    return img

def outline_a_on_b(img0, img1):
    M, mask = find_homography(img0, img1)

    h, w = img0.shape[:2]
    pts = np.float32([ [0,0], [0,h-1], [w-1,h-1], [w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts, M)

    # Draw found regions
    img = cv2.polylines(img1, [np.int32(dst)], True, (0,0,255), 1, cv2.LINE_AA)

    return img

def warp_a_to_b(img0, img1):
    M, mask = find_homography(img0, img1)

    img = cv2.warpPerspective(img0, M, (img1.shape[1], img1.shape[0]))

    return img

def blit_a_to_b(img0, img1):
    M, mask = find_homography(img0, img1)

    # Mask out the warped image and place it on the unwarped
    mask = cv2.warpPerspective(np.ones_like(img0)*255, M, (img1.shape[1], img1.shape[0]))
    img0 = cv2.warpPerspective(img0, M, (img1.shape[1], img1.shape[0]))

    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    img1 = cv2.bitwise_and(img1, img1, mask=255-mask)

    img = img0 + img1

    return img

def clone_a_to_b(img0, img1):
    M, mask = find_homography(img0, img1)

    mask = cv2.warpPerspective(np.ones_like(img0)*255, M, (img1.shape[1], img1.shape[0]))
    img0 = cv2.warpPerspective(img0, M, (img1.shape[1], img1.shape[0]))

    img1 = cv2.copyMakeBorder(img1, 10, 10, 10, 10, cv2.BORDER_REPLICATE)

    topleft = [10, 10,]
    img = cv2.seamlessClone(
        img0,
        img1,
        mask,
        (topleft[1]+img0.shape[1]//2, topleft[0]+img0.shape[0]//2),
        cv2.MIXED_CLONE
    )

    return img
