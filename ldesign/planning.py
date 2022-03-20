from typing import Literal

import numpy as np


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
