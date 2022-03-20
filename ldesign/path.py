from __future__ import annotations

import cmath
import logging
import math
from collections import deque
from dataclasses import dataclass, field
from itertools import product
from typing import Literal, Sequence

import gdstk
import numpy as np

from ldesign import config, elements, planning
from ldesign.elements import Element
from ldesign.shapes.bridge import CpwBridgeArgs
from ldesign.shapes.path import CpwArgs
from ldesign.utils import to_complex

logger = logging.getLogger(__name__)
LEN_ERR = 1e-4
ANGLE_ERR = 1e-5


@dataclass
class PathOptions:
    radius: float = 30
    cpw: CpwArgs = field(default_factory=CpwArgs)
    total_length: float | None = None


class PathOp:
    pass


@dataclass
class Segment(PathOp):
    point: complex | elements.DockingPort
    radius: float

    def __post_init__(self):
        if not isinstance(self.point, elements.DockingPort):
            self.point = to_complex(self.point)
        if self.radius < 0:
            raise ValueError(f"Negative turning radius.")


@dataclass
class Bridge(PathOp):
    bridge: CpwBridgeArgs = field(default_factory=CpwBridgeArgs)


@dataclass
class AutoMeander(PathOp):
    width: float
    depth: float
    wind_direction: Literal["left", "right"]
    in_position: float
    out_position: float
    radius: float
    length: float | None = None


def _translate_meander(ops: list[PathOp]) -> list[PathOp]:
    new_ops = []
    current_pos: complex | None = None
    current_angle: float | None = None
    for op in ops:
        match op:
            case Segment(
                point=elements.DockingPort(point=current_pos, angle=current_angle)
            ):
                pass
            case Segment(point=complex() as next_pos):
                if current_pos is not None and not is_zero_len(next_pos - current_pos):
                    current_angle = cmath.phase(next_pos - current_pos)
                current_pos = next_pos
            case AutoMeander(
                radius=r, length=l, width=w, depth=d, in_position=i, out_position=o
            ):
                if current_angle is None or l is None or current_pos is None:
                    raise Exception
                points = planning.plan_fixed_len(
                    r,
                    l,
                    "e",
                    "n",
                    w - o,
                    d,
                    i,
                    w - i,
                )
                moved_points = (
                    np.array(points) * cmath.rect(1, current_angle - math.pi / 2)
                    + current_pos
                )
                for p in moved_points[1:]:
                    new_ops.append(Segment(p, r))
                current_pos = moved_points[-1]
                continue
        new_ops.append(op)
    return new_ops


def _create_gdstk_path(pos: complex, cfg: config.Config, cpw: CpwArgs):
    return gdstk.RobustPath(
        to_complex(pos),
        [cpw.width, cpw.gap, cpw.gap],
        [
            0,
            (cpw.width + cpw.gap) / 2,
            -(cpw.width + cpw.gap) / 2,
        ],
        layer=[
            cfg.LD_AL_INNER["layer"],
            cfg.LD_AL_OUTER["layer"],
            cfg.LD_AL_OUTER["layer"],
        ],
        datatype=[
            cfg.LD_AL_INNER["datatype"],
            cfg.LD_AL_OUTER["datatype"],
            cfg.LD_AL_OUTER["datatype"],
        ],
    )


def is_zero_len(v: complex):
    return abs(v) < LEN_ERR


def is_zero_angle(angle: float):
    return abs(angle) < ANGLE_ERR


def _solve_to_single_circle(
    current_pos: complex, next_pos: complex, next_angle: float, next_radius: float
) -> tuple[complex, float]:
    # Determine turn direction
    v_b = cmath.rect(1, next_angle)
    v_ab = next_pos - current_pos
    if is_zero_len(v_ab):
        raise Exception
    angle_diff = cmath.phase(v_b / v_ab)
    if is_zero_angle(angle_diff):
        return next_pos, 0
    # Pointing outwards
    v_b_r_out = -1j * v_b * math.copysign(next_radius, angle_diff)
    p_cb = next_pos - v_b_r_out
    v_a_cb = p_cb - current_pos
    l_a_cb, phi_a_cb = cmath.polar(v_a_cb)
    # Calculate turn angle
    if next_radius > l_a_cb:
        raise Exception
    theta_in = math.asin(next_radius / l_a_cb)
    in_angle = math.copysign(theta_in, -angle_diff) + phi_a_cb
    turn_angle = _get_turn_angle(in_angle, next_angle, angle_diff)
    # Calculate segment point
    v_b_r_in = v_b_r_out * cmath.rect(1, -turn_angle)
    p_b_r_in = p_cb + v_b_r_in
    return p_b_r_in, turn_angle


def _solve_to_two_circle(
    current_pos: complex,
    current_angle: float,
    current_radius: float,
    next_pos: complex,
    next_angle: float,
    next_radius: float,
) -> list[tuple[float, float, complex, complex]]:
    def _solve_angle(d: float, r1: float, r2: float) -> list[tuple[float, float]]:
        if abs(r1 - r2) > d:
            return []
        # outer
        theta = math.acos((r1 - r2) / d)
        angles = [(theta, theta), (-theta, -theta)]
        # inner
        if d > r1 + r2:
            theta = math.acos((r1 + r2) / d)
            angles.extend([(theta, theta - math.pi), (-theta, math.pi - theta)])
        return angles

    results: list[tuple[float, float, complex, complex]] = []
    for turn_sign_a, turn_sign_b in product((-1, 1), repeat=2):
        # Pointing outwards
        v_a_r_in = -1j * turn_sign_a * cmath.rect(current_radius, current_angle)
        v_b_r_out = -1j * turn_sign_b * cmath.rect(next_radius, next_angle)
        p_ca = current_pos - v_a_r_in
        p_cb = next_pos - v_b_r_out
        v_ca_cb = p_cb - p_ca
        if is_zero_len(v_ca_cb):
            if is_zero_len(current_radius - next_radius) and turn_sign_a == turn_sign_b:
                turn_angle = _get_turn_angle(current_angle, next_angle, turn_sign_a)
                results.append((turn_angle, 0, next_pos, next_pos))
            continue
        l_ca_cb, phi_ca_cb = cmath.polar(v_ca_cb)
        angles = _solve_angle(l_ca_cb, current_radius, next_radius)
        for a1, a2 in angles:
            v_r_seg_a = cmath.rect(current_radius, a1 + phi_ca_cb)
            seg_a = p_ca + v_r_seg_a
            v_r_seg_b = cmath.rect(next_radius, a2 + phi_ca_cb)
            seg_b = p_cb + v_r_seg_b
            # check turn direction
            v_sa_sb = seg_b - seg_a
            if (
                cmath.phase(v_sa_sb / v_r_seg_a) * turn_sign_a < 0
                or cmath.phase(v_sa_sb / v_r_seg_b) * turn_sign_b < 0
            ):
                continue
            seg_angle = cmath.phase(v_sa_sb)
            turn_angle_a = _get_turn_angle(current_angle, seg_angle, turn_sign_a)
            turn_angle_b = _get_turn_angle(seg_angle, next_angle, turn_sign_b)
            results.append((turn_angle_a, turn_angle_b, seg_a, seg_b))
    return sorted(
        results,
        key=lambda x: abs(x[0]) * current_radius
        + abs(x[1]) * next_radius
        + abs(x[3] - x[2]),
    )


def _get_turn_angle(start: float, end: float, sign: float):
    ans = (end - start) % (2 * math.pi)
    if sign > 0:
        return ans
    if ans > 0:
        ans -= 2 * math.pi
    return ans


def _extend_path_to_port(
    builder: BasicPathBuilder,
    next_pos: complex,
    next_angle: float,
    next_radius: float,
) -> None:
    current_pos = builder.current_pos
    current_angle = builder.current_angle
    current_radius = builder.current_radius
    v_ab = next_pos - current_pos
    if is_zero_len(v_ab):
        return
    if current_angle is None or is_zero_len(current_radius):
        if is_zero_len(next_radius):
            builder.segment(next_pos)
            return
        turn_pos, turn_angle = _solve_to_single_circle(
            current_pos, next_pos, next_angle, next_radius
        )
        builder.segment(turn_pos)
        builder.turn(next_radius, turn_angle)
        return
    if is_zero_len(next_radius):
        turn_pos, turn_angle = _solve_to_single_circle(
            next_pos, current_pos, current_angle + math.pi, current_radius
        )
        builder.turn(current_radius, -turn_angle)
        builder.segment(next_pos)
        return
    # (turn_angle_a, turn_angle_b, seg_a, seg_b)
    results = _solve_to_two_circle(
        current_pos, current_angle, current_radius, next_pos, next_angle, next_radius
    )
    assert len(results) > 0
    turn_angle_a, turn_angle_b, _, seg_b = results[0]
    builder.turn(current_radius, turn_angle_a)
    builder.segment(seg_b)
    builder.turn(next_radius, turn_angle_b)


def _extend_path_to_point(
    builder: BasicPathBuilder,
    next_pos: complex,
    next_radius: float,
    future_seg: PathOp,
) -> None:
    """
    :return: current_pos, current_angle
    """
    current_pos = builder.current_pos
    current_angle = builder.current_angle
    current_radius = builder.current_radius
    if current_angle is None or is_zero_len(current_radius):
        start_pos = current_pos
    else:
        start_pos, turn_angle = _solve_to_single_circle(
            next_pos, current_pos, current_angle + math.pi, current_radius
        )
        builder.turn(current_radius, -turn_angle)
        current_pos = builder.current_pos
        current_angle = builder.current_angle
    vec1 = next_pos - start_pos
    if is_zero_len(next_radius) or not isinstance(future_seg, Segment):
        builder.segment(next_pos)
        return
    match future_seg:
        case Segment(point=complex() as end_pos):
            pass
        case Segment(
            point=elements.DockingPort(point=future_pos, angle=future_angle),
            radius=future_radius,
        ):
            if is_zero_len(future_radius):
                end_pos = future_pos
            else:
                end_pos, _ = _solve_to_single_circle(
                    next_pos, future_pos, future_angle, future_radius
                )
        case _:
            raise Exception
    vec2 = end_pos - next_pos
    if is_zero_len(vec1) or is_zero_len(vec2):
        raise Exception
    turn_angle = cmath.phase(vec2 / vec1)
    theta = abs(turn_angle / 2)
    arc_len = next_radius * math.tan(theta)
    if (abs(vec1) < arc_len - LEN_ERR) or (abs(vec2) < arc_len - LEN_ERR):
        raise Exception
    arc_start = next_pos - vec1 / abs(vec1) * arc_len
    builder.segment(arc_start)
    builder.turn(next_radius, turn_angle)


class BasicPathBuilder:
    start_pos: complex
    start_angle: float | None
    current_pos: complex
    current_angle: float | None
    current_radius: float
    elems: list[elements.Element]
    paths: list[gdstk.RobustPath]
    current_path: gdstk.RobustPath | None
    options: PathOptions
    cfg: config.Config

    def __init__(
        self,
        pos: complex,
        angle: float | None,
        radius: float,
        options: PathOptions,
        cfg: config.Config,
    ) -> None:
        self.start_pos = pos
        self.start_angle = angle
        self.current_pos = pos
        self.current_angle = angle
        self.current_radius = radius
        self.elems = []
        self.paths = []
        self.current_path = None
        self.options = options
        self.cfg = cfg

    def segment(self, point: complex) -> BasicPathBuilder:
        if self.current_path is None:
            self.current_path = self._create_new_path()
        v = point - self.current_pos
        if not is_zero_len(v):
            self.current_path.segment(point)
            new_angle = cmath.phase(v)
            if self.start_angle is None:
                self.start_angle = new_angle
            self.current_pos = point
            self.current_angle = new_angle
        return self

    def turn(self, radius: float, angle: float) -> BasicPathBuilder:
        if self.current_angle is None:
            self.current_angle = 0
        if self.current_path is None:
            self.current_path = self._create_new_path()
        if not is_zero_angle(angle):
            start_angle = math.copysign(math.pi / 2, -angle) + self.current_angle
            final_angle = start_angle + angle
            self.current_path.arc(radius, start_angle, final_angle)
            if self.start_angle is None:
                self.start_angle = self.current_angle
            self.current_angle = (self.current_angle + angle) % (2 * np.pi)
            self.current_pos += cmath.rect(radius, final_angle) - cmath.rect(
                radius, start_angle
            )
        return self

    def build(self) -> elements.CpwWaveguide:
        if self.start_angle is None or self.current_angle is None:
            raise Exception
        cpw = elements.CpwWaveguide()
        cpw.cell.add(*self.paths)
        if self.current_path is not None:
            cpw.cell.add(self.current_path)
        for elem in self.elems:
            cpw.add_element(elem)
        cpw.create_port("start", self.start_pos, self.start_angle + math.pi)
        cpw.create_port("end", self.current_pos, self.current_angle)
        return cpw

    def _create_new_path(self):
        return _create_gdstk_path(self.current_pos, self.cfg, self.options.cpw)


# TODO support length mark in the middle of the path
# lines go into ports except the start port
class CoplanarWaveguideBuilder:
    _ops: list[PathOp]
    options: PathOptions
    cfg: config.Config

    def __init__(
        self,
        start: complex | elements.DockingPort,
        radius: float | None = None,
        options: PathOptions | None = None,
        cfg: config.Config | None = None,
    ):
        if isinstance(start, elements.DockingPort):
            start = start.copy()
            # rotate start port by pi to unify with other ports
            start.angle += math.pi
        if radius is None:
            radius = self.options.radius
        self._ops = [Segment(start, radius)]
        self.options = PathOptions() if options is None else options
        self.cfg = config.global_config if cfg is None else cfg

    def segment(self, point: complex | elements.DockingPort):
        radius = self.options.radius
        self._ops.append(Segment(point, radius))

    def auto_meander(self, width, depth, direction, in_pos, out_pos, radius, length):
        self._ops.append(
            AutoMeander(width, depth, direction, in_pos, out_pos, radius, length)
        )

    def build(self) -> elements.CpwWaveguide:
        """Build coplanar waveguide from according to PathOps.

        :return:
        """
        ops = _translate_meander(self._ops)
        return self._build_gdstk_path(ops)

    def _build_gdstk_path(self, ops: Sequence[PathOp]):
        pending_ops = deque(ops)
        start_pos: complex
        start_angle: float | None
        start_radius: float

        # Retrive start point from first op
        match pending_ops.popleft():
            case Segment(point=complex() as start_pos, radius=start_radius):
                start_angle = None
            case Segment(
                point=elements.DockingPort(point=start_pos, angle=start_angle),
                radius=start_radius,
            ):
                pass
            case _:
                raise Exception
        builder = BasicPathBuilder(
            start_pos, start_angle, start_radius, self.options, self.cfg
        )

        while len(pending_ops) > 0:
            match pending_ops:
                case [
                    Segment(
                        point=elements.DockingPort(point=next_pos, angle=next_angle),
                        radius=next_radius,
                    ),
                    *_,
                ]:
                    _extend_path_to_port(
                        builder,
                        next_pos,
                        next_angle,
                        next_radius,
                    )
                    builder.current_pos = next_pos
                    builder.current_angle = next_angle
                    builder.current_radius = next_radius
                    pending_ops.popleft()
                case [
                    Segment(point=complex() as next_pos, radius=next_radius),
                    future_seg,
                    *_,
                ]:
                    _extend_path_to_point(
                        builder,
                        next_pos,
                        next_radius,
                        future_seg,
                    )
                    builder.current_radius = next_radius
                    pending_ops.popleft()
                case [Bridge(), *_]:
                    pending_ops.popleft()
        return builder.build()


if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)
    # elem = Element()
    # builder = CoplanarWaveguideBuilder(0 + 0j)
    # builder.segment(200 + 100j)
    # builder.segment(-100 + 100j)
    # builder.auto_meander(700, 2000, "right", 100, 100, 30, 6000)
    # builder.segment(-5000 + 100j)
    # cpw = builder.build()
    # cpw = elem.add_element(cpw)
    # bp = BondPad()
    # bp = elem.add_element(bp, cpw.port_start, bp.port_line)

    # elem.write_gds("../ptest.gds")
    # elem.view()
    # p1 = 0j
    # p3 = 100+0j
    # a3 = -math.pi
    # r = 40
    # p2, ta = _solve_to_single_circle(p1, p3, a3, r)
    # print(p2, ta)
    # builder = BasicPathBuilder(p1, None, 30, PathOptions(), config.global_config)
    # builder.segment(p2)
    # builder.turn(r, ta)
    # p1 = 100j
    # p3 = 100+0j
    # a3 = np.pi*0.9
    # r = 40
    # p2, ta = _solve_to_single_circle(p1, p3, a3 - math.pi, r)
    # print(p2, ta)
    # builder = BasicPathBuilder(p3, a3, r, PathOptions(), config.global_config)
    # builder.turn(r, -ta)
    # builder.segment(p1)
    # elem = Element()
    # elem.add_element(builder.build())
    # elem.view()
    config.use_preset_design()
    p1 = 0j
    a1 = 0
    r1 = 50
    p3 = 50 - 50j
    a3 = -math.pi / 2
    r3 = 49.99999
    builder = BasicPathBuilder(p1, a1, r1, PathOptions(), config.global_config)
    _extend_path_to_port(builder, p3, a3, r3)
    elem = Element()
    elem.add_element(builder.build())
    elem.flatten()
    elem.view()
