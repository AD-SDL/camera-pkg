import cv2

PATH = 'OpenCV'
PATH_ENUMS = 'OpenCV/Enums'
COLOR = (0.0, 0.0, 0.5)

clone_flags = {
    'cv2.NORMAL_CLONE': cv2.NORMAL_CLONE,
    'cv2.MIXED_CLONE': cv2.MIXED_CLONE,
    'cv2.MONOCHROME_TRANSFER': cv2.MONOCHROME_TRANSFER,
}

imread_modes = {
    'cv2.IMREAD_UNCHANGED': cv2.IMREAD_UNCHANGED,
    'cv2.IMREAD_GRAYSCALE': cv2.IMREAD_GRAYSCALE,
    'cv2.IMREAD_COLOR': cv2.IMREAD_COLOR,
    'cv2.IMREAD_ANYDEPTH': cv2.IMREAD_ANYDEPTH,
    'cv2.IMREAD_ANYCOLOR': cv2.IMREAD_ANYCOLOR,
    'cv2.IMREAD_LOAD_GDAL': cv2.IMREAD_LOAD_GDAL,
    'cv2.IMREAD_REDUCED_GRAYSCALE_2': cv2.IMREAD_REDUCED_GRAYSCALE_2,
    'cv2.IMREAD_REDUCED_COLOR_2': cv2.IMREAD_REDUCED_COLOR_2,
    'cv2.IMREAD_REDUCED_GRAYSCALE_4': cv2.IMREAD_REDUCED_GRAYSCALE_4,
    'cv2.IMREAD_REDUCED_COLOR_4': cv2.IMREAD_REDUCED_COLOR_4,
    'cv2.IMREAD_REDUCED_GRAYSCALE_8': cv2.IMREAD_REDUCED_GRAYSCALE_8,
    'cv2.IMREAD_REDUCED_COLOR_8': cv2.IMREAD_REDUCED_COLOR_8,
    'cv2.IMREAD_IGNORE_ORIENTATION': cv2.IMREAD_IGNORE_ORIENTATION,
}

cvtColor_codes = {
    'cv2.COLOR_BGR2BGRA': cv2.COLOR_BGR2BGRA,
    'cv2.COLOR_RGB2RGBA': cv2.COLOR_RGB2RGBA,
    'cv2.COLOR_BGRA2BGR': cv2.COLOR_BGRA2BGR,
    'cv2.COLOR_RGBA2RGB': cv2.COLOR_RGBA2RGB,
    'cv2.COLOR_BGR2RGBA': cv2.COLOR_BGR2RGBA,
    'cv2.COLOR_RGB2BGRA': cv2.COLOR_RGB2BGRA,
    'cv2.COLOR_RGBA2BGR': cv2.COLOR_RGBA2BGR,
    'cv2.COLOR_BGRA2RGB': cv2.COLOR_BGRA2RGB,
    'cv2.COLOR_BGR2RGB': cv2.COLOR_BGR2RGB,
    'cv2.COLOR_RGB2BGR': cv2.COLOR_RGB2BGR,
    'cv2.COLOR_BGRA2RGBA': cv2.COLOR_BGRA2RGBA,
    'cv2.COLOR_RGBA2BGRA': cv2.COLOR_RGBA2BGRA,
    'cv2.COLOR_BGR2GRAY': cv2.COLOR_BGR2GRAY,
    'cv2.COLOR_RGB2GRAY': cv2.COLOR_RGB2GRAY,
    'cv2.COLOR_GRAY2BGR': cv2.COLOR_GRAY2BGR,
    'cv2.COLOR_GRAY2RGB': cv2.COLOR_GRAY2RGB,
    'cv2.COLOR_GRAY2BGRA': cv2.COLOR_GRAY2BGRA,
    'cv2.COLOR_GRAY2RGBA': cv2.COLOR_GRAY2RGBA,
    'cv2.COLOR_BGRA2GRAY': cv2.COLOR_BGRA2GRAY,
    'cv2.COLOR_RGBA2GRAY': cv2.COLOR_RGBA2GRAY,
}

def register_enums(w):
    #
    w.register(PATH_ENUMS, {
        'title': 'Clone Flags',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': clone_flags}],
        'outputs': [{'title': ''}],
    })

    w.register(PATH_ENUMS, {
        'title': 'Colormap Types',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': {
            'cv2.COLORMAP_AUTUMN': cv2.COLORMAP_AUTUMN,
            'cv2.COLORMAP_BONE': cv2.COLORMAP_BONE,
            'cv2.COLORMAP_JET': cv2.COLORMAP_JET,
            'cv2.COLORMAP_WINTER': cv2.COLORMAP_WINTER,
            'cv2.COLORMAP_RAINBOW': cv2.COLORMAP_RAINBOW,
            'cv2.COLORMAP_OCEAN': cv2.COLORMAP_OCEAN,
            'cv2.COLORMAP_SUMMER': cv2.COLORMAP_SUMMER,
            'cv2.COLORMAP_SPRING': cv2.COLORMAP_SPRING,
            'cv2.COLORMAP_COOL': cv2.COLORMAP_COOL,
            'cv2.COLORMAP_HSV': cv2.COLORMAP_HSV,
            'cv2.COLORMAP_PINK': cv2.COLORMAP_PINK,
            'cv2.COLORMAP_HOT': cv2.COLORMAP_HOT,
            'cv2.COLORMAP_PARULA': cv2.COLORMAP_PARULA,
            'cv2.COLORMAP_MAGMA': cv2.COLORMAP_MAGMA,
            'cv2.COLORMAP_INFERNO': cv2.COLORMAP_INFERNO,
            'cv2.COLORMAP_PLASMA': cv2.COLORMAP_PLASMA,
            'cv2.COLORMAP_VIRIDIS': cv2.COLORMAP_VIRIDIS,
            'cv2.COLORMAP_CIVIDIS': cv2.COLORMAP_CIVIDIS,
            'cv2.COLORMAP_TWILIGHT': cv2.COLORMAP_TWILIGHT,
            'cv2.COLORMAP_TWILIGHT_SHIFTED': cv2.COLORMAP_TWILIGHT_SHIFTED,
            'cv2.COLORMAP_TURBO': cv2.COLORMAP_TURBO,
        }}],
        'outputs': [{'title': ''}],
    })

    w.register(PATH_ENUMS, {
        'title': 'Color Conversion Codes',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': cvtColor_codes}],
        'outputs': [{'title': ''}],
    })

    w.register(PATH_ENUMS, {
        'title': 'Fonts',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': {
            'cv2.FONT_HERSHEY_SIMPLEX': cv2.FONT_HERSHEY_SIMPLEX,
            'cv2.FONT_HERSHEY_PLAIN': cv2.FONT_HERSHEY_PLAIN,
            'cv2.FONT_HERSHEY_DUPLEX': cv2.FONT_HERSHEY_DUPLEX,
            'cv2.FONT_HERSHEY_COMPLEX': cv2.FONT_HERSHEY_COMPLEX,
            'cv2.FONT_HERSHEY_TRIPLEX': cv2.FONT_HERSHEY_TRIPLEX,
            'cv2.FONT_HERSHEY_COMPLEX_SMALL': cv2.FONT_HERSHEY_COMPLEX_SMALL,
            'cv2.FONT_HERSHEY_SCRIPT_SIMPLEX': cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
            'cv2.FONT_HERSHEY_SCRIPT_COMPLEX': cv2.FONT_HERSHEY_SCRIPT_COMPLEX,
            'cv2.FONT_ITALIC': cv2.FONT_ITALIC,
        }}],
        'outputs': [{'title': ''}],
    })

    w.register(PATH_ENUMS, {
        'title': 'Line Types',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': {
            'cv2.FILLED': cv2.FILLED,
            'cv2.LINE_4': cv2.LINE_4,
            'cv2.LINE_8': cv2.LINE_8,
            'cv2.LINE_AA': cv2.LINE_AA,
        }}],
        'outputs': [{'title': ''}],
    })

    w.register(PATH_ENUMS, {
        'title': 'Marker Types',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': {
            'cv2.MARKER_CROSS': cv2.MARKER_CROSS,
            'cv2.MARKER_TILTED_CROSS': cv2.MARKER_TILTED_CROSS,
            'cv2.MARKER_STAR': cv2.MARKER_STAR,
            'cv2.MARKER_DIAMOND': cv2.MARKER_DIAMOND,
            'cv2.MARKER_SQUARE': cv2.MARKER_SQUARE,
            'cv2.MARKER_TRIANGLE_UP': cv2.MARKER_TRIANGLE_UP,
            'cv2.MARKER_TRIANGLE_DOWN': cv2.MARKER_TRIANGLE_DOWN,
        }}],
        'outputs': [{'title': ''}],
    })

    #
    w.register(PATH_ENUMS, {
        'title': 'Imread Modes',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': imread_modes}],
        'outputs': [{'title': ''}],
    })

    w.register(PATH_ENUMS, {
        'title': 'Threshold Types',
        'func': lambda val: val,
        'fields': [{'title': 'val', 'widget': 'choice', 'choices': {
            'cv2.THRESH_BINARY': cv2.THRESH_BINARY,
            'cv2.THRESH_BINARY_INV': cv2.THRESH_BINARY_INV,
            'cv2.THRESH_TRUNC': cv2.THRESH_TRUNC,
            'cv2.THRESH_TOZERO': cv2.THRESH_TOZERO,
            'cv2.THRESH_TOZERO_INV': cv2.THRESH_TOZERO_INV,
            'cv2.THRESH_MASK': cv2.THRESH_MASK,
            'cv2.THRESH_OTSU': cv2.THRESH_OTSU,
            'cv2.THRESH_TRIANGLE': cv2.THRESH_TRIANGLE,
        }}],
        'outputs': [{'title': ''}],
    })

def register(w):
    register_enums(w)

    image_filtering(w)
    operations_on_arrays(w)

    w.register(PATH, {
        'title': 'cv2.imread',
        'func': cv2.imread,
        'fields': [{'title': 'filename', 'widget': 'file'}, {'title': 'flags', 'widget': 'choice', 'choices': imread_modes, 'optional': True}],
        # 'inputs': [{'title': 'flags', 'optional': True}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'cv2.resize',
        'func': cv2.resize,
        'inputs': [
            {'title': 'src'},
            {'title': 'dsize'},
            {'title': 'fx', 'optional': True},
            {'title': 'fy', 'optional': True},
            {'title': 'interpolation', 'optional': True},
        ],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'cv2.cvtColor',
        'func': cv2.cvtColor,
        'fields': [{'title': 'code', 'widget': 'choice', 'choices': cvtColor_codes}],
        'inputs': [{'title': 'src'}],#, {'title': 'code'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'cv2.seamlessClone',
        'func': cv2.seamlessClone,
        'fields': [{'title': 'flags', 'widget': 'choice', 'choices': clone_flags}],
        'inputs': [
            {'title': 'src'},
            {'title': 'dst'},
            {'title': 'mask'},
            {'title': 'p'},
            # {'title': 'flags'},
            {'title': 'blend', 'optional': True},
        ],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'cv2.applyColorMap',
        'func': cv2.applyColorMap,
        'inputs': [
            {'title': 'src'},
            {'title': 'colormap'},
            {'title': 'userColor'},
        ],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'cv2.threshold',
        'func': cv2.threshold,
        'inputs': [
            {'title': 'src'},
            {'title': 'thresh'},
            {'title': 'maxval'},
            {'title': 'type'},
        ],
        'outputs': [
            {'title': 'retval'},
            {'title': 'dst'},
        ],
        'color': COLOR,
    })

    def floodFill(**kwargs):
        if 'image' in kwargs:
            kwargs['image'] = kwargs['image'].copy()
        return cv2.floodFill(**kwargs)

    w.register(PATH, {
        'title': 'cv2.floodFill',
        'func': floodFill,
        'inputs': [
            {'title': 'image'},
            {'title': 'mask'},
            {'title': 'seedPoint'},
            {'title': 'newVal'},
            {'title': 'loDiff', 'optional': True},
            {'title': 'upDiff', 'optional': True},
            {'title': 'flags', 'optional': True},
        ],
        'outputs': [
            {'title': 'retval'},
            {'title': 'image'},
            {'title': 'mask'},
            {'title': 'rect'},
        ],
        'color': COLOR,
    })






def image_filtering(w):
    # https://docs.opencv.org/4.5.2/d4/d86/group__imgproc__filter.html
    path = PATH + '/Image Filtering'

    w.register(path, {
        'title': 'cv2.GaussianBlur',
        'func': cv2.GaussianBlur,
        'inputs': [
            {'title': 'src'},
            {'title': 'ksize'},
            {'title': 'sigmaX'},
            # {'title': 'dst', 'optional': True},
            {'title': 'sigmaY', 'optional': True},
            {'title': 'borderType', 'optional': True},
        ],
        'outputs': [
            {'title': 'dst'},
        ],
        'color': COLOR,
    })

def operations_on_arrays(w):
    # https://docs.opencv.org/3.4/d2/de8/group__core__array.html
    path = PATH + '/Array Operations'

    w.register(path, {
        'title': 'cv2.subtract',
        'func': cv2.subtract,
        'inputs': [
            {'title': 'src1'},
            {'title': 'src2'},
            {'title': 'mask', 'optional': True},
            {'title': 'dtype', 'optional': True},
        ],
        'outputs': [
            {'title': 'dst'},
        ],
        'color': COLOR,
    })

    def normalize(**kwargs):
        kwargs['dst'] = None
        return cv2.normalize(**kwargs)

    w.register(path, {
        'title': 'cv2.normalize',
        'func': normalize,
        'inputs': [
            {'title': 'src'},
            {'title': 'alpha', 'optional': True},
            {'title': 'beta', 'optional': True},
            {'title': 'norm_type', 'optional': True},
            {'title': 'dtype', 'optional': True},
            {'title': 'mask', 'optional': True},
        ],
        'outputs': [
            {'title': 'dst'},
        ],
        'color': COLOR,
    })
