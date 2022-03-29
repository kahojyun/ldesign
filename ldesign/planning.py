import math
from typing import Literal

import numpy as np
import scipy.optimize as sopt


def _check_standard_meander_params(
    radius: float, width: float, depth: float, in_pos: float, out_pos: float
):
    if (
        radius < 0
        or width < 0
        or depth < 0
        or in_pos < 0
        or out_pos < 0
        or in_pos > width
        or out_pos > width
    ):
        raise ValueError
    if depth < 2 * radius:
        raise ValueError


def _solve_standard_meander_port(
    x: float, y: float, r: float
) -> tuple[float, float, float]:
    """
    :return: segment length, arc angle, total length
    """
    if x < 0:
        sl, aa, tl = _solve_standard_meander_port(-x, y, r)
        return sl, -aa, tl
    d = math.sqrt((x - 2 * r) ** 2 + y * y)
    if d < 2 * r:
        raise ValueError
    seg_len = math.sqrt(x * x + y * y - 4 * x * r)
    theta = math.acos(2 * r / d)
    alpha = math.atan2(y, x - 2 * r)
    arc_angle = math.pi - theta - alpha
    total_len = seg_len + 2 * arc_angle * r
    return seg_len, arc_angle, total_len


def _solve_standard_meander_port_len(x: float, y: float, r: float) -> float:
    *_, result = _solve_standard_meander_port(x, y, r)
    return result


def calc_standard_meander_min_len(
    radius: float, width: float, depth: float, in_pos: float, out_pos: float
):
    _check_standard_meander_params(radius, width, depth, in_pos, out_pos)
    return _solve_standard_meander_port_len(abs(out_pos - in_pos), depth, radius)


def calc_standard_meander_max_len(
    radius: float,
    width: float,
    depth: float,
    in_pos: float,
    out_pos: float,
    direction: Literal["left", "right", "auto"],
) -> tuple[float, Literal["left", "right"]]:
    """
    :return: max length, direction
    """
    _check_standard_meander_params(radius, width, depth, in_pos, out_pos)
    if direction == "auto":
        l_len = _calc_standard_meander_max_len(
            radius, width, depth, in_pos, out_pos, "left"
        )
        r_len = _calc_standard_meander_max_len(
            radius, width, depth, in_pos, out_pos, "right"
        )
        if l_len > r_len:
            return l_len, "left"
        return r_len, "right"
    else:
        return (
            _calc_standard_meander_max_len(
                radius, width, depth, in_pos, out_pos, direction
            ),
            direction,
        )


def _calc_standard_meander_max_len(
    radius: float,
    width: float,
    depth: float,
    in_pos: float,
    out_pos: float,
    direction: Literal["left", "right"],
):
    n = _get_max_n(radius, depth)
    return _calc_standard_meander_total_len_with_ratio(
        n, 1, radius, width, depth, in_pos, out_pos, direction
    )


def _calc_standard_meander_partial_len_to_n(
    n: int,
    radius: float,
    width: float,
    in_pos: float,
    direction: Literal["left", "right"],
) -> float:
    if n == 0:
        return 0
    p1_x = _get_port_pos(1, in_pos, width, direction)
    return _solve_standard_meander_port_len(
        abs(p1_x - in_pos), 2 * radius, radius
    ) + _solve_standard_meander_port_len(width, 2 * radius, radius) * (n - 1)


def _calc_standard_meander_total_len_with_ratio(
    n: int,
    ratio: float,
    radius: float,
    width: float,
    depth: float,
    in_pos: float,
    out_pos: float,
    direction: Literal["left", "right"],
):
    if n > _get_max_n(radius, depth) or n < 1:
        raise ValueError
    if ratio < 0 or ratio > 1:
        raise ValueError
    if n == 1:
        seg_len = (depth - 2 * radius) * ratio
        curve_len = _solve_standard_meander_port_len(
            abs(out_pos - in_pos), depth - seg_len, radius
        )
        return seg_len + curve_len
    last_x = _get_port_pos(n - 2, in_pos, width, direction)
    target_x = _get_port_pos(n - 1, in_pos, width, direction)
    x = _get_x_from_ratio(ratio, out_pos, last_x, target_x)
    return (
        _calc_standard_meander_partial_len_to_n(n - 2, radius, width, in_pos, direction)
        + _solve_standard_meander_port_len(abs(x - last_x), 2 * radius, radius)
        + _solve_standard_meander_port_len(abs(x - out_pos), 2 * radius, radius)
        + (depth - n * 2 * radius)
    )


def _get_x_from_ratio(ratio, x1, x2, target_x):
    if x1 < x2 <= target_x or target_x <= x2 < x1:
        from_x = x2
    else:
        from_x = x1
    x = from_x + ratio * (target_x - from_x)
    return x


def _get_port_pos(
    n: int, in_pos: float, width: float, direction: Literal["left", "right"]
) -> float:
    if n == 0:
        return in_pos
    if (n % 2 == 0) ^ (direction == "right"):
        return width
    else:
        return 0


def _get_n_and_ratio(x, max_n):
    n = min(math.floor(x), max_n)
    r = x - n
    return n, r


def plan_standard_meander(
    radius: float,
    width: float,
    depth: float,
    in_pos: float,
    out_pos: float,
    total_len: float,
    direction: Literal["left", "right", "auto"],
) -> list[tuple]:
    min_len = calc_standard_meander_min_len(radius, width, depth, in_pos, out_pos)
    max_len, direction = calc_standard_meander_max_len(
        radius, width, depth, in_pos, out_pos, direction
    )
    if total_len < min_len or total_len > max_len:
        raise ValueError

    max_n = _get_max_n(radius, depth)

    def func(x: float) -> float:
        _n, _r = _get_n_and_ratio(x, max_n)
        return (
            _calc_standard_meander_total_len_with_ratio(
                _n, _r, radius, width, depth, in_pos, out_pos, direction
            )
            - total_len
        )

    root: float = sopt.brentq(func, 1, max_n + 1)  # type: ignore
    n, r = _get_n_and_ratio(root, max_n)
    return _get_standard_meander_commands(
        n, r, radius, width, depth, in_pos, out_pos, direction
    )


def _get_standard_meander_commands(
    n: int,
    ratio: float,
    radius: float,
    width: float,
    depth: float,
    in_pos: float,
    out_pos: float,
    direction: Literal["left", "right"],
) -> list[tuple]:
    if n > _get_max_n(radius, depth) or n < 1:
        raise ValueError
    if ratio < 0 or ratio > 1:
        raise ValueError
    if n == 1:
        s2_len = (depth - 2 * radius) * ratio
        return _get_port_port_commands(out_pos - in_pos, depth - s2_len, radius) + [
            ("e", s2_len),
        ]
    last_x = _get_port_pos(n - 2, in_pos, width, direction)
    target_x = _get_port_pos(n - 1, in_pos, width, direction)
    x = _get_x_from_ratio(ratio, out_pos, last_x, target_x)
    return (
        _get_commands_to_n(n - 2, radius, width, in_pos, direction)
        + _get_port_port_commands(x - last_x, 2 * radius, radius)
        + _get_port_port_commands(out_pos - x, 2 * radius, radius)
        + [("e", depth - n * 2 * radius)]
    )


def _get_port_port_commands(x: float, y: float, r: float) -> list[tuple]:
    seg_len, angle, _ = _solve_standard_meander_port(x, y, r)
    return [
        ("a", r, -angle),
        ("e", seg_len),
        ("a", r, angle),
    ]


def _get_commands_to_n(
    n: int,
    radius: float,
    width: float,
    in_pos: float,
    direction: Literal["left", "right"],
):
    last_x = in_pos
    result = []
    for i in range(n):
        x = _get_port_pos(i + 1, in_pos, width, direction)
        result.extend(_get_port_port_commands(x - last_x, 2 * radius, radius))
        last_x = x
    return result


def _get_max_n(radius: float, depth: float):
    return math.floor(depth / (2 * radius))


def _plan_fixed_len_ee(
    radius, total_len, out_position, height, left_width, right_width, tolerance
) -> list[complex]:
    def calc_min_len(n_u, n_b):
        return (
            right_width
            + out_position
            + 4 * radius * n_u
            - (2 - np.pi / 2) * radius * n_b
        )

    available_len = []
    # r
    if right_width < radius or out_position < radius:
        raise ValueError("No enough space for first bend.")
    available_len.append(calc_min_len(0, 1))
    if abs(available_len[-1] - total_len) < tolerance:
        return [
            complex(0, 0),
            complex(0, out_position),
            complex(right_width, out_position),
        ]

    # rur
    if right_width < 3 * radius or out_position < 3 * radius:
        raise ValueError("No enough space for first bend.")
    available_len.append(calc_min_len(0, 3))
    if abs(available_len[-1] - total_len) < tolerance:
        return [
            complex(0, 0),
            complex(0, radius),
            complex(right_width - radius, radius),
            complex(right_width - radius, out_position),
            complex(right_width, out_position),
        ]

    u_turn_num = 1
    while (4 * u_turn_num + 1) * radius <= out_position:
        min_len = calc_min_len(u_turn_num, 4 * u_turn_num + 1)
        max_len = min_len + 2 * u_turn_num * (left_width + right_width - 2 * radius)
        available_len.append((min_len, max_len))
        if min_len <= total_len <= max_len:
            residue_len = total_len - min_len
            points = [
                complex(0, 0),
                complex(0, radius),
                complex(right_width, radius),
                complex(right_width, 3 * radius),
            ]
            for i in range(u_turn_num - 1):
                width = min(residue_len / 2 + 2 * radius, left_width + right_width)
                points.extend(
                    [
                        complex(right_width - width, (4 * i + 3) * radius),
                        complex(right_width - width, (4 * i + 5) * radius),
                        complex(right_width, (4 * i + 5) * radius),
                        complex(right_width, (4 * i + 7) * radius),
                    ]
                )
                residue_len -= 2 * (width - 2 * radius)
            width = residue_len / 2 + 2 * radius
            points.extend(
                [
                    complex(right_width - width, (4 * u_turn_num - 1) * radius),
                    complex(right_width - width, out_position),
                    complex(right_width, out_position),
                ]
            )
            return points
        u_turn_num += 1
    raise ValueError(f"Failed. Available length with given arguments: {available_len}")


def _plan_fixed_len_en(
    radius, total_len, out_position, height, left_width, right_width, tolerance
) -> list[complex]:
    if right_width < 2 * radius:
        raise ValueError("No enough space for first two bends.")

    available_len = []
    out_x = right_width - out_position
    if out_x >= 2 * radius:
        available_len.append(height + out_x - (2 - np.pi / 2) * radius * 2)
        if abs(available_len[-1] - total_len) < tolerance:
            return [
                complex(0, 0),
                complex(0, radius),
                complex(right_width - out_position, radius),
                complex(right_width - out_position, height),
            ]

    turn_num = 1
    while (2 * turn_num + 2) * radius <= height:
        # Check space for out bends
        if (turn_num % 2 == 1 and out_position < 2 * radius) or (
            turn_num % 2 == 0 and out_x < 2 * radius - left_width
        ):
            turn_num += 1
            continue
        # Calculate min and max length
        if turn_num % 2 == 1:
            extra_len = (turn_num * 2 + 2) * radius
        else:
            if out_x < 0:
                extra_len = (2 * turn_num + 4) * radius
            elif out_x > 2 * radius:
                extra_len = 2 * turn_num * radius
            else:
                extra_len = (2 * turn_num + 4) * radius - 2 * out_x
        min_len = (
            height
            + abs(out_x)
            + extra_len
            - (2 - np.pi / 2) * (2 * turn_num + 2) * radius
        )
        max_len = (
            height
            + (turn_num - 1) * (left_width + right_width)
            + right_width
            - (2 - np.pi / 2) * (2 * turn_num + 2) * radius
        )
        if turn_num % 2 == 1:
            max_len += out_position
        else:
            max_len += out_x + left_width
        available_len.append((min_len, max_len))
        # Arrange points
        if min_len <= total_len <= max_len:
            residue_len = total_len - min_len
            points = [complex(0, 0), complex(0, radius)]
            for i in range(turn_num):
                # Calculate width range
                if i == turn_num - 1:
                    if i % 2 == 0:
                        max_width_diff = right_width - max(out_x, 0) - 2 * radius
                    else:
                        max_width_diff = left_width + min(out_x - 2 * radius, 0)
                else:
                    if i % 2 == 0:
                        max_width_diff = right_width - 2 * radius
                    else:
                        max_width_diff = left_width
                width_diff = min(residue_len / 2, max_width_diff)
                unused_width = max_width_diff - width_diff
                if i % 2 == 0:
                    points.extend(
                        [
                            complex(right_width - unused_width, (2 * i + 1) * radius),
                            complex(right_width - unused_width, (2 * i + 3) * radius),
                        ]
                    )
                else:
                    points.extend(
                        [
                            complex(-left_width + unused_width, (2 * i + 1) * radius),
                            complex(-left_width + unused_width, (2 * i + 3) * radius),
                        ]
                    )
                residue_len -= 2 * width_diff
            points.extend(
                [complex(out_x, (2 * turn_num + 1) * radius), complex(out_x, height)]
            )
            return points
        turn_num += 1
    raise ValueError(
        f"Failed. Available length with given arguments: {available_len}, request length: {total_len}"
    )


def plan_fixed_len(
    radius: float,
    total_len: float,
    in_direction: Literal["e", "n", "w"],
    out_side: Literal["e", "n", "w", "s"],
    out_position: float,
    height: float,
    left_width: float,
    right_width: float,
):
    tolerance = 1e-2
    if radius <= 0:
        raise ValueError("Radius should be positive.")
    if (out_side in ("e", "w") and height < out_position) or (
        out_side in ("n", "s") and left_width + right_width < out_position
    ):
        raise ValueError("out_position too large.")
    if in_direction == "e":
        if out_side == "e":
            return _plan_fixed_len_ee(
                radius,
                total_len,
                out_position,
                height,
                left_width,
                right_width,
                tolerance,
            )
        if out_side == "n":
            return _plan_fixed_len_en(
                radius,
                total_len,
                out_position,
                height,
                left_width,
                right_width,
                tolerance,
            )
    raise Exception


if __name__ == "__main__":
    commands = plan_standard_meander(30, 500, 1000, 200, 300, 2971.239, "right")
    total_len = 0
    for c in commands:
        match c:
            case "a", radius, angle:
                total_len += radius * abs(angle)
            case "e", length:
                total_len += length
            case _:
                raise Exception
    print(commands)
    print(total_len)
