from dataclasses import asdict, dataclass

import gdstk
import numpy as np
from ldesign import config, elements


@dataclass
class UPadArgs:
    u_width: float = 88
    u_height: float = 65.5
    inner_gap: float = 5
    outer_gap: float = 6
    side_width: float = 30
    bottom_width: float = 10


class UPad(elements.Element):
    def __init__(
        self, args: UPadArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = UPadArgs()
        self.args = args
        self._init_cell(**asdict(args))

    def _init_cell(
        self, u_width, u_height, inner_gap, outer_gap, side_width, bottom_width
    ):
        ld_inner = self.config.LD_AL_INNER
        ld_gap = self.config.LD_AL_GAP
        al_inner_poly = gdstk.rectangle(
            -side_width - u_width / 2 + 0j,
            side_width + u_width / 2 + 1j * (u_height + bottom_width),
        )
        inner_cut = gdstk.rectangle(
            -u_width / 2 + 1j * bottom_width,
            u_width / 2 + 1j * (bottom_width + u_height),
        )
        al_inner_poly = gdstk.boolean(al_inner_poly, inner_cut, "not", **ld_inner)
        al_gap_poly = gdstk.rectangle(
            -side_width - u_width / 2 - outer_gap - 1j * outer_gap,
            side_width
            + u_width / 2
            + outer_gap
            + 1j * (u_height + bottom_width + outer_gap),
        )
        gap_cut = gdstk.rectangle(
            -u_width / 2 + inner_gap + 1j * (bottom_width + inner_gap),
            u_width / 2 - inner_gap + 1j * (u_height + bottom_width + outer_gap),
        )
        al_gap_poly = gdstk.boolean(
            al_gap_poly, [gap_cut] + al_inner_poly, "not", **ld_gap
        )
        self.cell.add(*al_inner_poly, *al_gap_poly)
        self.create_port("line", 0j, np.pi * 3 / 2)
        self.create_port("u", 1j * (bottom_width + inner_gap), np.pi / 2)

    @property
    def port_line(self):
        return self.ports["line"]

    @property
    def port_u(self):
        return self.ports["u"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = UPad()
    elem.view()
