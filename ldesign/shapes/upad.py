from dataclasses import dataclass, asdict

import gdstk
import numpy as np

import ldesign
from ldesign import elements
from ldesign.config import Config


@dataclass
class UPadArgs:
    u_width: float = 88
    u_height: float = 65.5
    inner_gap: float = 5
    outer_gap: float = 6
    side_width: float = 30
    bottom_width: float = 10


class UPad(elements.Element):
    def __init__(self, args: UPadArgs = None, config: Config = None):
        super().__init__(config=config)
        if args is None:
            args = UPadArgs()
        self.args = args
        self._init_cell(**asdict(args))

    def _init_cell(self, u_width, u_height, inner_gap, outer_gap, side_width, bottom_width):
        ld_inner = self.config.LD_AL_INNER
        ld_outer = self.config.LD_AL_OUTER
        inner = gdstk.rectangle(-side_width - u_width / 2 + 0j,
                                side_width + u_width / 2 + 1j * (u_height + bottom_width))
        inner_cut = gdstk.rectangle(-u_width / 2 + 1j * bottom_width, u_width / 2 + 1j * (bottom_width + u_height))
        inner = gdstk.boolean(inner, inner_cut, "not", **ld_inner)
        outer = gdstk.rectangle(-side_width - u_width / 2 - outer_gap - 1j * outer_gap,
                                side_width + u_width / 2 + outer_gap + 1j * (u_height + bottom_width + outer_gap))
        outer_cut = gdstk.rectangle(-u_width / 2 + inner_gap + 1j * (bottom_width + inner_gap),
                                    u_width / 2 - inner_gap + 1j * (u_height + bottom_width + outer_gap))
        outer = gdstk.boolean(outer, [outer_cut] + inner, "not", **ld_outer)
        self.cell.add(*inner, *outer)
        self.create_port("line", -1j * outer_gap, np.pi * 3 / 2)
        self.create_port("u", 1j * (bottom_width + inner_gap), np.pi / 2)

    @property
    def port_line(self):
        return self.ports["line"]

    @property
    def port_u(self):
        return self.ports["u"]


if __name__ == '__main__':
    ldesign.config.use_preset_design()
    elem = UPad()
    elem.view()
