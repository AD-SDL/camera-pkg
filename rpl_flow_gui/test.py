import cv2
import numpy as np
import pyheif
from PIL import Image

def heif_load(fname):
    hf = pyheif.read(fname)

    img = np.array(Image.frombytes(
        hf.mode,
        hf.size,
        hf.data,
        "raw",
        hf.mode,
        hf.stride,
    ))

    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)





img = heif_load('IMG_0246.HEIC')
blur = cv2.GaussianBlur(img, (1, 15), 20)
sub = cv2.subtract(img, blur)
add = sub + 150
norm = cv2.normalize(add, None, 0, 255, cv2.NORM_MINMAX)

cv2.imshow('', norm)
cv2.waitKey(0)