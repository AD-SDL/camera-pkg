PATH = 'Cast'
COLOR = (0.5, 0.0, 0.5)

def register(w):
    w.register(PATH, {
        'title': 'int',
        'func': lambda x: int(x),
        'inputs': [{'title': 'x'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'float',
        'func': lambda x: float(x),
        'inputs': [{'title': 'x'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'str',
        'func': lambda x: str(x),
        'inputs': [{'title': 'x'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'tuple (2)',
        'func': lambda x,y: (x, y),
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'tuple (3)',
        'func': lambda x,y,z: (x, y, z),
        'inputs': [{'title': 'x'}, {'title': 'y'}, {'title': 'z'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })
