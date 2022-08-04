from dataclasses import dataclass, field

import gdstk
import numpy as np
from ldesign import config, elements, utils
from ldesign.shapes.path import CpwArgs


@dataclass
class TPadArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    pad_width: float = 274
    pad_height: float = 30
    fillet_radius: float = 5
    with_gap: bool = True


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
        ld_gap = self.config.LD_AL_GAP
        extend_len = max(fillet_radius, line_gap)
        region = gdstk.rectangle(
            (-pad_width / 2 - line_gap, -line_gap),
            (pad_width / 2 + line_gap, pad_height + extend_len),
        )
        pad_inner = [
            gdstk.rectangle(-pad_width / 2 + 0j, pad_width / 2 + pad_height * 1j),
            gdstk.rectangle(
                -line_width / 2 + pad_height * 1j,
                line_width / 2 + (pad_height + extend_len + fillet_radius) * 1j,
            ),
        ]
        pad_inner = gdstk.boolean(pad_inner, [], "or", **ld_inner)
        utils.fillet_polygons(pad_inner, fillet_radius, self.config.tolerance)
        pad_inner = gdstk.boolean(region, pad_inner, "and", **ld_inner)
        self.cell.add(*pad_inner)
        if args.with_gap:
            pad_gap = gdstk.offset(pad_inner, line_gap, **ld_gap)
            pad_gap = gdstk.boolean(region, pad_gap, "and", **ld_gap)
            pad_gap = gdstk.boolean(pad_gap, pad_inner, "not", **ld_gap)
            self.cell.add(*pad_gap)
            self.create_port("pad_center", -line_gap * 1j, 3 * np.pi / 2)
            self.create_port("pad_right", pad_width / 2 - line_gap * 1j, 3 * np.pi / 2)
            self.create_port("pad_left", -pad_width / 2 - line_gap * 1j, 3 * np.pi / 2)
        else:
            self.create_port("pad_center", 0j, 3 * np.pi / 2)
            self.create_port("pad_right", pad_width / 2 + 0j, 3 * np.pi / 2)
            self.create_port("pad_left", -pad_width / 2 + 0j, 3 * np.pi / 2)
        self.create_port("line", (pad_height + extend_len) * 1j, np.pi / 2)

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


if __name__ == "__main__":
    # config.use_preset_design()
    TPad(TPadArgs(with_gap=False)).view()
