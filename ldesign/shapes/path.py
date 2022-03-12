from dataclasses import dataclass, field

import gdstk
import numpy as np

import ldesign
from ldesign import elements
from ldesign.config import Config


@dataclass
class CpwArgs:
    width: float = 4
    gap: float = 2


@dataclass
class FluxEndArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    right_gap_len: float = 1
    left_gap_len: float = 7
    back_gap_len: float = 7


class OpenEnd(elements.Element):
    def __init__(self, args: CpwArgs = None, config: Config = None):
        super().__init__(config=config)
        if args is None:
            args = CpwArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        width = 2 * self.args.gap + self.args.width
        height = self.args.gap
        self.cell.add(gdstk.rectangle(0j, width + height * 1j, **self.config.LD_AL_OUTER))
        self.create_port("line", width / 2 + 0j, np.pi * 3 / 2)
        self.create_port("end", width / 2 + 1j * height, np.pi / 2)

    @property
    def port_line(self):
        return self.ports["line"]


class FluxEnd(elements.Element):
    def __init__(self, args: FluxEndArgs = None, config: Config = None):
        super().__init__(config=config)
        if args is None:
            args = FluxEndArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        ld_inner = self.config.LD_AL_INNER
        ld_outer = self.config.LD_AL_OUTER
        line_width = self.args.cpw.width
        gap = self.args.cpw.gap
        l_length = min(self.args.left_gap_len, self.args.right_gap_len)
        inner = [gdstk.rectangle(line_width / 2 - 1j * (gap + line_width), -line_width / 2 + 0j),
                 gdstk.rectangle(line_width / 2 - 1j * (gap + line_width), -line_width / 2 - gap - l_length - 1j * gap)]
        inner = gdstk.boolean(inner, [], 'or', **ld_inner)
        outer = [gdstk.rectangle(0j, -line_width / 2 - gap - self.args.right_gap_len - 1j * gap),
                 gdstk.rectangle(-line_width / 2 - gap - self.args.left_gap_len - 1j * (gap + line_width),
                                 line_width / 2 + gap + self.args.back_gap_len - 1j * (2 * gap + line_width)),
                 gdstk.rectangle(line_width / 2 + 0j, line_width / 2 + gap - 1j * (2 * gap + line_width))]
        outer = gdstk.boolean(outer, inner, 'not', **ld_outer)
        self.cell.add(*inner, *outer)
        self.create_port("line", 0j, np.pi / 2)
        self.create_port("flux", -1j * (2 * gap + line_width), np.pi * 3 / 2)

    @property
    def port_line(self):
        return self.ports["line"]

    @property
    def port_flux(self):
        return self.ports["flux"]


if __name__ == '__main__':
    ldesign.config.use_preset_design()
    elem = OpenEnd(CpwArgs())
    elem.view()
    elem = FluxEnd()
    elem.view()
