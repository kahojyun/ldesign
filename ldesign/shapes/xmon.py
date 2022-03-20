from dataclasses import asdict, dataclass
from typing import Literal

import gdstk
import numpy as np
from ldesign import config, elements
from ldesign.shapes import upad


@dataclass
class XmonArgs:
    gap: float = 24
    z_gap: float = 18
    arm_len: float = 140
    arm_width: float = 24
    z_inner_pad_width: float = 8
    z_inner_pad_height: float = 6
    z_outer_pad_width: float = 4
    z_outer_pad_height: float = 8
    z_outer_pad_dist: float = 14
    xy_port: Literal["n", "e", "w"] = "e"
    r_port: Literal["n", "e", "w"] = "w"


class Xmon(elements.Element):
    def __init__(
        self, args: XmonArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = XmonArgs()
        self._init_cell(**asdict(args))

    def _init_cell(
        self,
        gap,
        z_gap,
        arm_len,
        arm_width,
        xy_port,
        r_port,
        z_inner_pad_width,
        z_inner_pad_height,
        z_outer_pad_width,
        z_outer_pad_height,
        z_outer_pad_dist,
    ):
        ld_inner = self.config.LD_AL_INNER
        ld_outer = self.config.LD_AL_OUTER
        inner = [
            gdstk.cross(
                1j * (arm_len + arm_width / 2),
                2 * arm_len + arm_width,
                arm_width,
                **ld_inner
            ),
            gdstk.rectangle(
                -z_inner_pad_width / 2 - 1j * z_inner_pad_height,
                z_inner_pad_width / 2 + 0j,
                **ld_inner
            ),
        ]
        bottom_x = -arm_width / 2 - gap
        bottom_y = -z_gap
        top_x = -bottom_x
        top_y = 2 * arm_len + arm_width + gap
        left_x = -arm_width / 2 - arm_len - gap
        left_y = arm_len - gap
        right_x = -left_x
        right_y = arm_len + arm_width + gap
        outer = [
            gdstk.rectangle(bottom_x + 1j * bottom_y, top_x + 1j * top_y),
            gdstk.rectangle(left_x + 1j * left_y, right_x + 1j * right_y),
        ]
        outer_cut = [
            gdstk.rectangle(
                -z_outer_pad_dist / 2 - 1j * z_gap,
                -z_outer_pad_dist / 2
                - z_outer_pad_width
                - 1j * (z_gap - z_outer_pad_height),
            ),
            gdstk.rectangle(
                z_outer_pad_dist / 2 - 1j * z_gap,
                z_outer_pad_dist / 2
                + z_outer_pad_width
                - 1j * (z_gap - z_outer_pad_height),
            ),
        ]
        outer = gdstk.boolean(outer, inner + outer_cut, "not", **ld_outer)
        self.cell.add(*outer, *inner)
        point_map = {
            "n": 1j * top_y,
            "e": right_x + 1j * (arm_len + arm_width / 2),
            "w": left_x + 1j * (arm_len + arm_width / 2),
        }
        angle_map = {"n": np.pi / 2, "e": 0, "w": np.pi}
        self.create_port("xy", point_map[xy_port], angle_map[xy_port])
        self.create_port("r", point_map[r_port], angle_map[r_port])
        self.create_port("z", 1j * (-z_gap), np.pi * 3 / 2)

    @property
    def port_xy(self):
        return self.ports["xy"]

    @property
    def port_r(self):
        return self.ports["r"]

    @property
    def port_z(self):
        return self.ports["z"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = Xmon()
    r_pad = upad.UPad()
    elem.add_element(
        r_pad, elem.port_r, r_pad.port_u, elements.Transformation(translation=3)
    )
    elem.view()
