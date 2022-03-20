def to_complex(point):
    match point:
        case [r, i]:
            return complex(r, i)
        case float() | int():
            return complex(point, 0)
        case complex():
            return point
        case _:
            raise TypeError
