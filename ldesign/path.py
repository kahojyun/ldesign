from __future__ import annotations
import logging
import math
from dataclasses import dataclass, field
from typing import Sequence, Literal

import gdstk
import numpy as np

from ldesign import elements, planning, config
from ldesign.elements import Element
from ldesign.shapes.bondpad import BondPad
from ldesign.shapes.bridge import CpwBridgeArgs
from ldesign.shapes.path import CpwArgs
from ldesign.utils import to_complex

logger = logging.getLogger(__name__)


@dataclass
class PathOptions:
    radius: float = 30
    cpw: CpwArgs = field(default_factory=CpwArgs)
    total_length: float | None = None


class PathOp:
    pass


@dataclass
class Segment(PathOp):
    point: complex
    radius: float

    def __post_init__(self):
        self.point = to_complex(self.point)
        if self.radius < 0:
            raise ValueError(f"Negative turning radius.")


@dataclass
class Start(PathOp):
    port: complex | elements.DockingPort
    radius: float | None = None


@dataclass
class Bridge(PathOp):
    bridge: CpwBridgeArgs = field(default_factory=CpwBridgeArgs)


@dataclass
class AutoMeander(PathOp):
    width: float
    depth: float
    wind_direction: Literal['left', 'right']
    in_position: float
    out_position: float
    radius: float
    length: float | None = None


def _translate_meander(ops: list[PathOp]) -> list[PathOp]:
    new_ops = []
    current_pos: complex = None
    current_angle: float = None
    for op in ops:
        if isinstance(op, Start):
            if isinstance(op.port, complex):
                current_pos = op.port
            else:
                current_pos = op.port.point
                current_angle = op.port.angle
        if isinstance(op, Segment):
            current_angle = np.angle(op.point - current_pos)
            current_pos = op.point
        if isinstance(op, AutoMeander):
            points = planning.plan_fixed_len(op.radius, op.length, 'e', 'n', op.width - op.out_position, op.depth,
                                             op.in_position,
                                             op.width - op.in_position)
            moved_points = np.array(points) * np.exp(1j * (current_angle - np.pi / 2)) + current_pos
            for p in moved_points[1:]:
                new_ops.append(Segment(p, op.radius))
            current_pos = moved_points[-1]
            continue
        new_ops.append(op)
    return new_ops


# TODO support length mark in the middle of the path
# TODO unification of point and port
# lines go into ports except the start port
class CoplanarWaveguideBuilder:
    _ops: list[PathOp]
    options: PathOptions

    def __init__(self, start: complex, radius=None):
        self._ops = [Start(start, radius)]
        self.options = PathOptions()

    def segment(self, point: complex):
        radius = self.options.radius
        self._ops.append(Segment(point, radius))

    def auto_meander(self, width, depth, direction, in_pos, out_pos, radius, length):
        self._ops.append(
            AutoMeander(
                width, depth, direction, in_pos, out_pos, radius, length
            )
        )

    def build(self):
        """Build coplanar waveguide from according to PathOps.

        :return:
        """
        ops = _translate_meander(self._ops)
        print(ops)
        return self._build_gdstk_path(ops)

    def _build_gdstk_path(self, ops: Sequence[PathOp]):
        # points = [start] + [t.point for t in turnings] + [end]
        # min_radius = min(t.radius for t in turnings)
        # if check_radius and min_radius < width / 2:
        #     raise ValueError(f"Minimum turning radius smaller than width/2: {min_radius=}, {width/2=}")
        # last_arc_len = 0
        # if path_type == "auto":
        #     if min_radius - width / 2 < 1:
        #         path_type = "robust"
        #     else:
        #         path_type = "flex"
        # if path_type == "flex":
        #     path = gdstk.FlexPath(start, width)
        # else:
        #     path = gdstk.RobustPath(start, width)
        op0 = ops[0]
        assert isinstance(op0, Start)
        start_pos = op0.port
        start_angle = None
        current_pos = op0.port
        current_radius = None
        last_pos = None
        assert isinstance(current_pos, complex)
        c = config.global_config
        path_inner = gdstk.RobustPath(current_pos, self.options.cpw.width, **c.LD_AL_INNER)
        path_outer = gdstk.RobustPath(current_pos, self.options.cpw.width + 2 * self.options.cpw.gap, **c.LD_AL_OUTER)
        current_path_pos = current_pos
        for op in ops[1:]:
            assert isinstance(op, Segment)
            next_pos = op.point
            if last_pos is not None:
                vec1 = current_pos - last_pos
                vec2 = next_pos - current_pos
                turn_angle = get_turn_angle(vec1, vec2)
                theta = abs(turn_angle / 2)
                arc_len = current_radius * math.tan(theta)
                arc_start = current_pos - vec1 / abs(vec1) * arc_len
                if abs(arc_start - current_path_pos) > 1e-3:
                    path_inner.segment(arc_start)
                    path_outer.segment(arc_start)
                if abs(turn_angle) > 1e-3:
                    path_inner.turn(current_radius, turn_angle)
                    path_outer.turn(current_radius, turn_angle)
                current_path_pos = current_pos + vec2 / abs(vec2) * arc_len
            if start_angle is None:
                start_angle = np.angle(current_pos - next_pos)
            last_pos = current_pos
            current_pos = next_pos
            current_radius = op.radius
        path_inner.segment(current_pos)
        path_outer.segment(current_pos)
        end_angle = np.angle(current_pos - last_pos)
        cpw_elem = elements.CpwWaveguide()
        # clean cut
        # path_inner.set_ends("extended")
        path_outer = gdstk.boolean(path_outer, path_inner, 'not', **c.LD_AL_OUTER)
        # path_inner.set_ends("flush")
        cpw_elem.cell.add(path_inner, *path_outer)
        # cpw_elem.cell.add(path_inner, path_outer)
        cpw_elem.create_port("start", start_pos, start_angle)
        cpw_elem.create_port("end", current_pos, end_angle)
        return cpw_elem
        # for i in range(len(turnings)):
        #     vec1 = points[i + 1] - points[i]
        #     vec2 = points[i + 2] - points[i + 1]
        #     turn_angle = get_turn_angle(vec1, vec2)
        #     logger.debug(f"{turnings[i]=}, {turn_angle*180/np.pi=}")
        #     theta = abs(turn_angle / 2)
        #     arc_len = turnings[i].radius * math.tan(theta)
        #     arc_start = points[i + 1] - vec1 / abs(vec1) * arc_len
        #     logger.debug(f"{arc_start=}, {arc_len=}")
        #     path.segment(arc_start)
        #     path.turn(turnings[i].radius, turn_angle)
        # if last_arc_len + arc_len - abs(vec1) > len_check_tolerance:
        #     raise ValueError(
        #         f"No enough length for turning, line segment: {points[i]} -- {points[i + 1]}, length: {abs(vec1)}, "
        #         f"required length: {last_arc_len + arc_len}")
        # last_arc_len = arc_len
        # if i == len(turnings) - 1 and last_arc_len - abs(vec2) > len_check_tolerance:
        #     raise ValueError(
        #         f"No enough length for turning, line segment: {points[i + 1]} -- {points[i + 2]}, length: {abs(vec2)}, "
        #         f"required length: {last_arc_len}")


def get_turn_angle(vec1: complex, vec2: complex) -> float:
    return np.angle(vec2 / vec1)  # type: ignore


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    elem = Element()
    builder = CoplanarWaveguideBuilder(0+0j)
    builder.segment(200 + 100j)
    builder.segment(-100 + 100j)
    builder.auto_meander(700, 2000, 'right', 100, 100, 30, 6000)
    builder.segment(-5000 + 100j)
    cpw = builder.build()
    cpw = elem.add_element(cpw)
    bp = BondPad()
    bp = elem.add_element(bp, cpw.port_start, bp.port_line)

    elem.write_gds('../ptest.gds')
    elem.view()
