import time

import cv2
from rpl_cv.methods.ot2.plate_read_colors import get_colors, match_size

for i in range(1, 7):
    # Prepare the image
    img = cv2.imread(f'examples/data/ot2_plate_colors/{i}.jpg')
    img = match_size(img, (1280, 1920))
    print(f'Analyzing image examples/data/ot2_plate_colors/{i}.jpg')

    # Get the colors
    s = time.time()
    # Set dbg to 1 or 2 for extra detailed output
    colors = get_colors(img, dbg=0)
    e = time.time()
    print(f'Took {e-s} seconds.')

    # Print some results!
    print(f'Found {len(colors)} plates')
    if len(colors):
        k = list(colors.keys())[0]
        print(f'Plate {k} well A1 had color {colors[k]["A1"]}')
        print(f'Plate {k} had proximity {colors[k]["proximity"]}')
        if colors[k]["proximity"] > 0.2:
            print('Proximity is not good! The plate may be too far from the center of the camera for an accurate reading.')

    print()
