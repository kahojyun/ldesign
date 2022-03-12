def to_complex(point):
    if isinstance(point, tuple):
        return complex(*point)
    elif isinstance(point, (float, int)):
        return complex(point, 0)
    elif isinstance(point, complex):
        return point
    else:
        raise TypeError
