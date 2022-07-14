import cv2
import numpy as np
# import pyheif
from PIL import Image

PATH = 'Test'
COLOR = (0.5, 0.0, 0.5)

# def heif_load(fname):
#     hf = pyheif.read(fname)

#     img = np.array(Image.frombytes(
#         hf.mode,
#         hf.size,
#         hf.data,
#         "raw",
#         hf.mode,
#         hf.stride,
#     ))

#     return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def execcer(string):
    val = []
    string = 'val.append(' + string + ')'

    exec(string)
    val = val[0]

    return val

def register(w):
    # w.register(PATH, {
    #     'title': 'Load HEIC',
    #     'func': heif_load,
    #     'fields': [{'title': 'fname', 'widget': 'file'}],
    #     'outputs': [{'title': ''}],
    #     'color': COLOR,
    # })

    w.register(PATH, {
        'title': 'Image Slice',
        'func': lambda src, a, b, c, d: src[a:b, c:d],
        'inputs': [{'title': 'src'}, {'title': 'a'}, {'title': 'b'}, {'title': 'c'}, {'title': 'd'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Debug',
        'func': lambda *args, **kwargs: ((args, kwargs),),
        'fields': [{'title': '', 'widget': 'debug'}],
        'inputs': [{'title': 'x'}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Exec',
        'func': lambda s: execcer(s),
        'fields': [{'title': 's', 'widget': 'string'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })
