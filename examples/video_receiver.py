#!/usr/bin/env python3

import cv2
import numpy as np
from rpl_cv.core.fiducial import draw_fiducials, find_fiducials
from rpl_cv.core.network import simple_video_receiver


def find_pipette_region(img, corners, draw=None):
    # TODO: Was this loop originally intended to detect multiple pipette regions?
    #       Can it be rewritten?

    for cornerset in corners:
        cornerset = cornerset[0].astype(int)

        down = cornerset[3] - cornerset[0]
        right = cornerset[2] - cornerset[3]

        tl = cornerset[0] + 3*down - right//2
        tr = cornerset[1] + 3*down + right//2
        bl = cornerset[0] + 5*down - right//2
        br = cornerset[1] + 5*down + right//2

        if draw is not None:
            cv2.line(draw, cornerset[0], cornerset[0] + 3*(cornerset[3]-cornerset[0]), 255, 5)
            cv2.line(draw, cornerset[1], cornerset[1] + 3*(cornerset[2]-cornerset[1]), 255, 5)

            cv2.line(draw, tl, tr, 255, 5)
            cv2.line(draw, tr, br, 255, 5)
            cv2.line(draw, br, bl, 255, 5)
            cv2.line(draw, bl, tl, 255, 5)

    if len(corners):
        mn = np.min([tl, tr, bl, br], axis=0)
        mx = np.max([tl, tr, bl, br], axis=0)
    else:
        mn = np.array([0, 0])
        mx = np.array([img.shape[1], img.shape[0]])

    # Make sure mn is >= 0
    mn = np.max([mn, [0, 0]], axis=0)

    return mn, mx

def locate_pipette(img):
    # TODO: Is this necessary?
    shrink = 20
    img = img[shrink:-shrink, shrink:-shrink]
    if img.size == 0:
        return (0, 0), (0, 0)

    if len(img.shape) == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Increase the significance of the pipette features in the image

    img = cv2.erode(img, np.ones((7, 7), np.uint8), iterations=1)
    # cv2.imshow('0 erode', img)

    img = cv2.dilate(img, np.ones((7, 7), np.uint8), iterations=1)
    # cv2.imshow('1 dilate', img)

    img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 51, -13)
    # cv2.imshow('2 threshold', img)

    # Dilate white vertically so that background features vanish, leaving vertical features
    img = cv2.dilate(img, np.ones((31, 5), np.uint8), iterations=1)
    # cv2.imshow('3 dilate', img)

    img = cv2.erode(img, np.ones((61, 3), np.uint8), iterations=1)
    # cv2.imshow('4 erode', img)

    numLabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, 4, cv2.CV_32S)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # TODO: Was this loop originally intended to detect multiple pipettes?
    #       Can it be rewritten?

    # 0 is background
    for i in range(1, numLabels):
        # Extract the connected component statistics and centroid for the current label
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        # area = stats[i, cv2.CC_STAT_AREA]
        # cX, cY = centroids[i]

        # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 3)
        # cv2.circle(img, (int(cX), int(cY)), 4, (0, 0, 255), -1)

    if numLabels == 2:
        return (x+20, y+20), (x+w+20, y+h+20)
    else:
        return (0, 0), (0, 0)

def draw_pipette(img, tl, ptl, pbr):
    # The given image may be very large
    # Crop the image to at most 200x100 around the pipette
    centroid = (
        tl[1] + (ptl[1]+pbr[1])//2,
        tl[0] + (ptl[0]+pbr[0])//2,
    )
    img_crop = img[
        max(0, centroid[0]-100):centroid[0]+100,
        max(0, centroid[1]-50) :centroid[1]+50
    ]

    cv2.imshow('Pipette Draw 1', img_crop)

    # Subtract the average value of nearby pixels
    # This enhances detail
    blur = cv2.GaussianBlur(img_crop, (1, 15), 20)
    sub = cv2.subtract(img_crop, blur) + 150

    # Use the full set of available brightness levels
    norm = cv2.normalize(sub, None, 0, 255, cv2.NORM_MINMAX)

    cv2.imshow('Pipette Draw 2', norm)

    if len(img.shape) == 3:
        color_1 = (0, 0, 255)
        color_2 = (255, 0, 0)
    else:
        color_1 = 255
        color_2 = 255

    cv2.rectangle(
        img,
        (ptl[0]+tl[0], ptl[1]+tl[1]),
        (pbr[0]+tl[0], pbr[1]+tl[1]),
        color_1,
        1,
    )
    cv2.rectangle(
        img,
        (max(0, centroid[1]-50), max(0, centroid[0]-100)),
        (centroid[1]+50, centroid[0]+100),
        color_2,
        1,
    )

def callback(img):
    img_draw = img.copy()
    corners, ids = find_fiducials(img)
    draw_fiducials(img_draw, corners, ids)
    tl, br = find_pipette_region(img, corners, draw=img_draw)

    img_pipette = img[tl[1]:br[1], tl[0]:br[0]]
    if img_pipette.size < 10:
        return

    # Narrow down the pipette top left and bottom right points
    ptl, pbr = locate_pipette(img_pipette)

    # Draw a marker around it if it exists
    if ptl != (0, 0) and pbr != (0, 0):
        draw_pipette(img, tl, ptl, pbr)

    cv2.imshow('Annotated video', img_draw)

simple_video_receiver(callback=callback)
