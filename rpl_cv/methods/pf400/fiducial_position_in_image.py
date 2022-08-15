import cv2
import numpy as np

from ...core.fiducial import find_fiducials


def fiducial_position(img):
    corners, ids = find_fiducials(img)

    pts = []
    if len(corners) > 0:
        # corners.shape = (tag, ?, corner, point)
        pts = np.array(corners)
        pts = np.mean(pts, axis=2)
        pts = np.reshape(pts, (-1, 2))
        pts = pts.astype(int)

        for pt in pts:
            cv2.circle(img, pt, 10, (255, 0, 255), 5)

    return pts

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        cap.release()
        return

    # Certain cameras require a warmup. 10 frames seems to be plenty.
    for i in range(10):
        rv, img = cap.read()

    rv, img = cap.read()

    pts = fiducial_position_in_image(img)

    print(pts)

    cv2.imshow('', img)
    cv2.waitKey(0)

    cap.release()

if __name__ == '__main__':
    main()
