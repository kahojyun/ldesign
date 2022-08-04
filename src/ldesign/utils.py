from __future__ import annotations

import cmath
import math
from typing import Sequence

import gdstk

from ldesign import elements


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


def find_intersection(
    port1: elements.DockingPort, port2: elements.DockingPort, check_direction=False
):
    p1 = port1.point
    p2 = port2.point
    v1 = cmath.rect(1, port1.angle)
    v2 = cmath.rect(1, port2.angle)
    det = (v1 * v2.conjugate()).imag
    if abs(det) < 1e-5:
        raise ValueError("parallel port")
    a = (((p2 - p1) * v2.conjugate()).imag) / det
    b = (((p2 - p1) * v1.conjugate()).imag) / det
    if check_direction and (a < 1e-5 or b < 1e-5):
        raise ValueError("wrong direction")
    return p1 + a * v1


def fillet_polygons(
    polygons: gdstk.Polygon | Sequence[gdstk.Polygon],
    radius: float,
    tolerance: float = 1e-3,
):
    if isinstance(polygons, gdstk.Polygon):
        polygons.fillet(radius, tolerance)
        return
    for poly in polygons:
        poly.fillet(radius, tolerance)


if __name__ == "__main__":
    print(
        find_intersection(
            elements.DockingPort(0j, math.pi / 4),
            elements.DockingPort(10 + 0j, -2 * math.pi / 4),
        )
    )
