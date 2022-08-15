PATH = 'Math'
COLOR = (0.5, 0.0, 0.0)

def register(w):
    w.register(PATH, {
        'title': 'Add',
        'func': lambda x,y: x+y,
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Subtract',
        'func': lambda x,y: x-y,
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Multiply',
        'func': lambda x,y: x*y,
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Divide',
        'func': lambda x,y: x/y,
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Integer Divide',
        'func': lambda x,y: x//y,
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })

    w.register(PATH, {
        'title': 'Modulo',
        'func': lambda x,y: x%y,
        'inputs': [{'title': 'x'}, {'title': 'y'}],
        'outputs': [{'title': ''}],
        'color': COLOR,
    })
