from dataclasses import dataclass, field

import gdstk
import numpy as np
from ldesign import config, elements
from ldesign.shapes.path import CpwArgs


@dataclass
class TPadArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    pad_width: float = 274
    pad_height: float = 30
    fillet_radius: float = 5


class TPad(elements.Element):
    def __init__(
        self, args: TPadArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = TPadArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        args = self.args
        cpw = args.cpw
        pad_width = args.pad_width
        pad_height = args.pad_height
        line_gap = cpw.gap
        line_width = cpw.width
        fillet_radius = args.fillet_radius
        ld_inner = self.config.LD_AL_INNER
        ld_outer = self.config.LD_AL_OUTER
        region = gdstk.rectangle(
            (-pad_width / 2 - line_gap, -line_gap),
            (pad_width / 2 + line_gap, pad_height + line_gap + fillet_radius),
        )
        pad_inner = gdstk.Polygon(
            [
                (pad_width / 2, 0),
                (pad_width / 2, pad_height),
                (line_width / 2, pad_height),
                (line_width / 2, pad_height + fillet_radius * 2),
                (-line_width / 2, pad_height + fillet_radius * 2),
                (-line_width / 2, pad_height),
                (-pad_width / 2, pad_height),
                (-pad_width / 2, 0),
            ],
            **ld_inner
        )
        pad_inner.fillet(fillet_radius)
        pad_inner = gdstk.boolean(region, pad_inner, "and", **ld_inner)
        pad_outer = gdstk.Polygon(
            [
                (pad_width / 2 + line_gap, -line_gap),
                (pad_width / 2 + line_gap, pad_height + line_gap),
                (line_width / 2 + line_gap, pad_height + line_gap),
                (line_width / 2 + line_gap, pad_height + line_gap + fillet_radius * 2),
                (-line_width / 2 - line_gap, pad_height + line_gap + fillet_radius * 2),
                (-line_width / 2 - line_gap, pad_height + line_gap),
                (-pad_width / 2 - line_gap, pad_height + line_gap),
                (-pad_width / 2 - line_gap, -line_gap),
            ],
            **ld_outer
        )
        pad_outer.fillet(fillet_radius)
        pad_outer = gdstk.boolean(region, pad_outer, "and", **ld_outer)
        pad_outer = gdstk.boolean(pad_outer, pad_inner, "not", **ld_outer)
        self.cell.add(*pad_inner)
        self.cell.add(*pad_outer)
        self.create_port("line", pad_height * 1j, np.pi / 2)
        self.create_port("pad_center", -line_gap * 1j, 3 * np.pi / 2)
        self.create_port("pad_right", pad_width / 2 - line_gap * 1j, 3 * np.pi / 2)
        self.create_port("pad_left", -pad_width / 2 - line_gap * 1j, 3 * np.pi / 2)

    @property
    def port_line(self):
        return self.ports["line"]

    @property
    def port_pad_center(self):
        return self.ports["pad_center"]

    @property
    def port_pad_right(self):
        return self.ports["pad_right"]

    @property
    def port_pad_left(self):
        return self.ports["pad_left"]
