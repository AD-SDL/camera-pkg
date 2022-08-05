import numpy as np

PATH = 'NumPy'
PATH_CREATE = 'NumPy/Create'
COLOR = (0.0, 0.5, 0.0)

def register(w):
    w.register(PATH_CREATE, {
        'title': 'np.array',
        'func': np.array,
        'inputs': [{'title': 'object'}, {'title': 'dtype', 'optional': True}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH_CREATE, {
        'title': 'np.zeros',
        'func': np.zeros,
        'inputs': [{'title': 'shape'}, {'title': 'dtype', 'optional': True}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH_CREATE, {
        'title': 'np.ones',
        'func': np.ones,
        'inputs': [{'title': 'shape'}, {'title': 'dtype', 'optional': True}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Array Attributes',
        'func': lambda arr: (arr.shape, arr.dtype),
        'inputs': [{'title': 'arr'}],
        'outputs': [{'title': 'shape'}, {'title': 'dtype'}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'np.interp',
        'func': np.interp,
        'inputs': [
            {'title': 'x'},
            {'title': 'xp'},
            {'title': 'fp'},
            {'title': 'left', 'optional': True},
            {'title': 'right', 'optional': True},
            {'title': 'period', 'optional': True},
        ],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })
