from __future__ import annotations
import cmath
from collections import deque
from itertools import product

import logging
import math
from dataclasses import dataclass, field
from typing import Literal, Sequence

import gdstk
import numpy as np

from ldesign import config, elements, planning
from ldesign.elements import Element
from ldesign.shapes.bondpad import BondPad
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
                if current_angle is None:
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
                    np.array(points) * np.exp(1j * (current_angle - np.pi / 2))
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
        pos,
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
    return np.abs(v) < LEN_ERR


def is_zero_angle(angle: float):
    return np.abs(angle) < ANGLE_ERR


def _solve_to_single_circle(
    current_pos: complex, next_pos: complex, next_angle: float, next_radius: float
) -> tuple[complex, float]:
    # Determine turn direction
    v_b = np.exp(1j * next_angle)
    v_ab = next_pos - current_pos
    if is_zero_len(v_ab):
        raise Exception
    angle_diff = cmath.phase(v_b / v_ab)
    if is_zero_angle(angle_diff):
        return next_pos, 0
    turn_sign_b = np.sign(angle_diff)
    # Pointing outwards
    v_b_r_out = -1j * turn_sign_b * v_b * next_radius
    p_cb = next_pos - v_b_r_out
    v_a_cb = p_cb - current_pos
    l_a_cb = np.abs(v_a_cb)
    # Calculate turn angle
    if next_radius > l_a_cb:
        raise Exception
    theta_in = np.arcsin(next_radius / l_a_cb)
    theta_out = cmath.phase(v_b / v_a_cb)
    turn_angle = theta_out + turn_sign_b * theta_in
    # Calculate segment point
    v_b_r_in = v_b_r_out * np.exp(-1j * turn_angle)
    p_b_r_in = p_cb + v_b_r_in
    return p_b_r_in, turn_angle


def _solve_angle_two_circle(
    d: float, r1: float, r2: float
) -> list[tuple[float, float]]:
    if np.abs(r1 - r2) > d:
        return []
    # outer
    theta = np.arccos((r1 - r2) / d)
    results = [(theta, theta), (-theta, -theta)]
    # inner
    if d > r1 + r2:
        theta = np.arccos((r1 + r2) / d)
        results.extend([(theta, theta - np.pi), (-theta, np.pi - theta)])
    return results


def _get_turn_angle(start: float, end: float, sign: float):
    ans = (end - start) % (2 * np.pi)
    if sign > 0:
        return ans
    if ans > 0:
        ans -= 2 * np.pi
    return ans


def _extend_path_to_port(
    path: gdstk.RobustPath,
    next_pos: complex,
    next_angle: float,
    next_radius: float,
    current_pos: complex,
    current_angle: float | None,
    current_radius: float,
):
    v_ab = next_pos - current_pos
    if is_zero_len(v_ab):
        return
    if current_angle is None or is_zero_len(current_radius):
        if is_zero_len(next_radius):
            path.segment(next_pos)
            return
        turn_pos, turn_angle = _solve_to_single_circle(
            current_pos, next_pos, next_angle, next_radius
        )
        if not is_zero_len(turn_pos - current_pos):
            path.segment(turn_pos)
        if not is_zero_angle(turn_angle):
            path.turn(next_radius, turn_angle)
        return
    if is_zero_len(next_radius):
        turn_pos, turn_angle = _solve_to_single_circle(
            next_pos, current_pos, current_angle + np.pi, current_radius
        )
        if not is_zero_angle(turn_angle):
            path.turn(current_radius, -turn_angle)
        if not is_zero_len(next_pos - turn_pos):
            path.segment(next_pos)
        return
    v_a = np.exp(1j * current_angle)
    v_b = np.exp(1j * next_angle)
    # (turn_angle_a, turn_angle_b, seg_a, seg_b)
    results: list[tuple[float, float, complex, complex]] = []
    for turn_sign_a, turn_sign_b in product((-1, 1), repeat=2):
        # Pointing outwards
        v_a_r_in = -1j * turn_sign_a * v_a * current_radius
        v_b_r_out = -1j * turn_sign_b * v_b * next_radius
        p_ca = current_pos - v_a_r_in
        p_cb = next_pos - v_b_r_out
        v_ca_cb = p_cb - p_ca
        if is_zero_len(v_ca_cb):
            if current_radius == next_radius and turn_sign_a == turn_sign_b:
                turn_angle = _get_turn_angle(current_angle, next_angle, turn_sign_a)
                results.append((turn_angle, 0, next_pos, next_pos))
            continue
        l_ca_cb = np.abs(v_ca_cb)
        angles = _solve_angle_two_circle(l_ca_cb, current_radius, next_radius)
        for a1, a2 in angles:
            v_r_seg_a = v_ca_cb / l_ca_cb * current_radius * np.exp(1j * a1)
            seg_a = p_ca + v_r_seg_a
            v_r_seg_b = v_ca_cb / l_ca_cb * next_radius * np.exp(1j * a2)
            seg_b = p_cb + v_r_seg_b
            # check turn direction
            v_sa_sb = seg_b - seg_a
            seg_a_sign = np.sign(v_sa_sb / v_r_seg_a)
            seg_b_sign = np.sign(v_sa_sb / v_r_seg_b)
            if seg_a_sign != turn_sign_a or seg_b_sign != turn_sign_b:
                continue
            seg_angle = cmath.phase(v_sa_sb)
            turn_angle_a = _get_turn_angle(current_angle, seg_angle, turn_sign_a)
            turn_angle_b = _get_turn_angle(seg_angle, next_angle, turn_sign_b)
            results.append((turn_angle_a, turn_angle_b, seg_a, seg_b))
    results = sorted(results, lambda x: abs(x[0]) + abs(x[1]))  # type: ignore
    assert len(results) > 0
    turn_angle_a, turn_angle_b, seg_a, seg_b = results[0]
    if not is_zero_angle(turn_angle_a):
        path.turn(current_radius, turn_angle_a)
    if not is_zero_len(seg_b - seg_a):
        path.segment(seg_b)
    if not is_zero_angle(turn_angle_b):
        path.turn(next_radius, turn_angle_b)


def _extend_path_to_point(
    path: gdstk.RobustPath,
    next_pos: complex,
    next_radius: float,
    future_seg: PathOp,
    current_pos: complex,
    current_angle: float | None,
    current_radius: float,
) -> tuple[complex, float | None]:
    """
    :return: current_pos, current_angle
    """
    if current_angle is None or is_zero_len(current_radius):
        start_pos = current_pos
    else:
        start_pos, turn_angle = _solve_to_single_circle(
            next_pos, current_pos, current_angle + np.pi, current_radius
        )
        if not is_zero_angle(turn_angle):
            path.turn(current_radius, -turn_angle)
            current_pos = start_pos
            current_angle -= turn_angle
    vec1 = next_pos - start_pos
    if is_zero_len(next_radius) or not isinstance(future_seg, Segment):
        if not is_zero_len(vec1):
            path.segment(next_pos)
            return next_pos, cmath.phase(vec1)
        return current_pos, current_angle
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
    if not is_zero_len(arc_start - start_pos):
        path.segment(arc_start)
    if not is_zero_angle(turn_angle):
        path.turn(next_radius, turn_angle)
    return next_pos + vec2 / abs(vec2) * arc_len, cmath.phase(vec2)


@dataclass
class _PathState:
    start_pos: complex
    current_radius: float
    start_angle: float | None = None
    current_angle: float | None = field(init=False)
    current_pos: complex = field(init=False)

    def __post_init__(self):
        self.current_angle = self.start_angle
        self.current_pos = self.start_pos


# TODO support length mark in the middle of the path
# TODO unification of point and port
# lines go into ports except the start port
class CoplanarWaveguideBuilder:
    _ops: list[PathOp]
    options: PathOptions

    def __init__(
        self, start: complex | elements.DockingPort, radius: float | None = None
    ):
        if isinstance(start, elements.DockingPort):
            start = start.copy()
            # rotate start port by pi to unify with other ports
            start.angle += np.pi
        if radius is None:
            radius = self.options.radius
        self._ops = [Segment(start, radius)]
        self.options = PathOptions()

    def segment(self, point: complex):
        radius = self.options.radius
        self._ops.append(Segment(point, radius))

    def auto_meander(self, width, depth, direction, in_pos, out_pos, radius, length):
        self._ops.append(
            AutoMeander(width, depth, direction, in_pos, out_pos, radius, length)
        )

    def build(self):
        """Build coplanar waveguide from according to PathOps.

        :return:
        """
        ops = _translate_meander(self._ops)
        print(ops)
        return self._build_gdstk_path(ops)

    def _build_gdstk_path(self, ops: Sequence[PathOp]):
        pending_ops = deque(ops)
        gdstk_objs = []
        current_pos: complex
        current_angle: float | None
        current_radius: float
        start_pos: complex
        start_angle: float | None

        def get_gdstk_path():
            match gdstk_objs:
                case [*_, gdstk.RobustPath() as path]:
                    return path
                case _:
                    path = _create_gdstk_path(
                        current_pos, config.global_config, self.options.cpw
                    )
                    gdstk_objs.append(path)
                    return path

        # Retrive start point from first op
        match pending_ops.popleft():
            case Segment(point=complex() as current_pos, radius=current_radius):
                current_angle = None
            case Segment(
                point=elements.DockingPort(point=current_pos, angle=current_angle),
                radius=current_radius,
            ):
                pass
            case _:
                raise Exception
        start_pos = current_pos
        start_angle = current_angle

        while len(pending_ops) > 0:
            match pending_ops:
                case [
                    Segment(
                        point=elements.DockingPort(point=next_pos, angle=next_angle),
                        radius=next_radius,
                    ),
                    *_,
                ]:
                    path = get_gdstk_path()
                    _extend_path_to_port(
                        path,
                        next_pos,
                        next_angle,
                        next_radius,
                        current_pos,
                        current_angle,
                        current_radius,
                    )
                    current_pos = next_pos
                    current_angle = next_angle
                    current_radius = next_radius
                    pending_ops.popleft()
                case [
                    Segment(point=complex() as next_pos, radius=next_radius),
                    future_seg,
                    *_,
                ]:
                    path = get_gdstk_path()
                    current_pos, current_angle = _extend_path_to_point(
                        path,
                        next_pos,
                        next_radius,
                        future_seg,
                        current_pos,
                        current_angle,
                        current_radius,
                    )
                    current_radius = next_radius
                    pending_ops.popleft()
                case [Bridge(), *_]:
                    pending_ops.popleft()
        cpw_elem = elements.CpwWaveguide()
        cpw_elem.cell.add(*gdstk_objs)
        if start_angle is None or current_angle is None:
            raise Exception
        cpw_elem.create_port("start", start_pos, start_angle)
        cpw_elem.create_port("end", current_pos, current_angle)
        return cpw_elem


if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)
    elem = Element()
    builder = CoplanarWaveguideBuilder(0 + 0j)
    builder.segment(200 + 100j)
    builder.segment(-100 + 100j)
    builder.auto_meander(700, 2000, "right", 100, 100, 30, 6000)
    builder.segment(-5000 + 100j)
    cpw = builder.build()
    cpw = elem.add_element(cpw)
    bp = BondPad()
    bp = elem.add_element(bp, cpw.port_start, bp.port_line)

    elem.write_gds("../ptest.gds")
    elem.view()
