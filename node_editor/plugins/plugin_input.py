PATH = 'Input'
COLOR = (0.5, 0.5, 0.0)

def register(w):
    w.register(PATH, {
        'title': 'New int',
        'func': lambda x: int(x),
        'fields': [{'title': 'x', 'widget': 'string'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'New float',
        'func': lambda x: float(x),
        'fields': [{'title': 'x', 'widget': 'string'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'New str',
        'func': lambda x: str(x),
        'fields': [{'title': 'x', 'widget': 'string'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Slider [0->1]',
        'func': lambda x: x,
        'fields': [{'title': 'x', 'widget': 'slider', 'min': 0.0, 'max': 1.0, 'default': 0.0}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'int tuple (2)',
        'func': lambda x,y: (int(x), int(y)),
        'fields': [{'title': 'x', 'widget': 'string'}, {'title': 'y', 'widget': 'string'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'float tuple (2)',
        'func': lambda x,y: (float(x), float(y)),
        'fields': [{'title': 'x', 'widget': 'string'}, {'title': 'y', 'widget': 'string'}],
        'outputs': [{'title': ''}],
        'simple': True,
        'color': COLOR,
    })
