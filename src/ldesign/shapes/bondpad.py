from dataclasses import dataclass, field

import gdstk
import numpy as np

from ldesign import config, elements
from ldesign.shapes.path import CpwArgs


@dataclass
class BondPadArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    pad_gap: float = 100
    pad_width: float = 250
    pad_len: float = 250
    trans_len: float = 120
    end_gap: float = 65

@dataclass
class Round_BondPadArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    pad_gap: float = 75
    pad_width: float = 250
    pad_len: float = 500


class BondPad(elements.Element):
    def __init__(
        self, args: BondPadArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = BondPadArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        line_gap = self.args.cpw.gap
        line_width = self.args.cpw.width
        pad_gap = self.args.pad_gap
        pad_width = self.args.pad_width
        pad_len = self.args.pad_len
        trans_len = self.args.trans_len
        end_gap = self.args.end_gap
        ld_inner = self.config.LD_AL_INNER
        ld_gap = self.config.LD_AL_GAP
        al_gap_poly = (
            gdstk.FlexPath((0, 0), line_gap * 2 + line_width, **ld_gap)
            .horizontal(trans_len, width=pad_gap * 2 + pad_width, relative=True)
            .horizontal(pad_len + end_gap, relative=True)
        )
        al_inner_poly = (
            gdstk.FlexPath((0, 0), line_width, **ld_inner)
            .horizontal(trans_len, width=pad_width, relative=True)
            .horizontal(pad_len, relative=True)
        )
        al_gap_poly = gdstk.boolean(al_gap_poly, al_inner_poly, "not", **ld_gap)
        self.cell.add(*al_gap_poly, al_inner_poly)
        self.create_port("line", 0j, np.pi)

    @property
    def port_line(self):
        return self.ports["line"]

class Round_BondPad(elements.Element):
    def __init__(
        self, args: Round_BondPadArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = Round_BondPadArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        line_gap = self.args.cpw.gap
        line_width = self.args.cpw.width
        pad_gap = self.args.pad_gap
        pad_width = self.args.pad_width
        pad_len = self.args.pad_len
        ld_inner = self.config.LD_AL_INNER
        ld_gap = self.config.LD_AL_GAP
        al_gap_poly = (
            gdstk.FlexPath((pad_gap + pad_width / 2, 0), width=pad_gap * 2 + pad_width, ends="round", **ld_gap)
            .horizontal(pad_len, width=pad_gap * 2 + pad_width, relative=True)
        )
        al_gap_poly2 = (
            gdstk.FlexPath((0, 0), line_gap * 2 + line_width, **ld_gap)
            .horizontal(pad_gap + pad_width / 2, relative=True)
        )
        al_inner_poly = (
            gdstk.FlexPath((pad_gap + pad_width/2, 0), pad_width, ends="round", **ld_inner)
            .horizontal(pad_len, width= pad_width, relative=True)
        )
        al_inner_poly2 = (
            gdstk.FlexPath((0, 0), line_width, **ld_inner)
            .horizontal(pad_gap + pad_width/2 + pad_gap, relative=True)
        )
        al_gap_poly = gdstk.boolean(al_gap_poly, al_gap_poly2, "or", **ld_gap)
        al_inner_poly = gdstk.boolean(al_inner_poly, al_inner_poly2, "or", **ld_inner)
        al_gap_poly = gdstk.boolean(al_gap_poly, al_inner_poly, "not", **ld_gap)
        self.cell.add(*al_gap_poly, *al_inner_poly)
        self.create_port("line", 0j, np.pi)

    @property
    def port_line(self):
        return self.ports["line"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = BondPad()
    elem.view()
    elem2 = Round_BondPad()
    elem2.view()
