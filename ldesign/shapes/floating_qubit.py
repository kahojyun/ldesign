from dataclasses import dataclass

import gdstk
import numpy as np

from ldesign import config, elements


@dataclass
class FloatingQubitArgs:
    gap: float = 30
    pad_width: float = 500
    pad_height: float = 120
    fillet_radius: float = 20


class FloatingQubit(elements.Element):
    def __init__(
        self, args: FloatingQubitArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = FloatingQubitArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        args = self.args
        gap = args.gap
        pad_width = args.pad_width
        pad_height = args.pad_height
        fillet_radius = args.fillet_radius
        ld_inner = self.config.LD_AL_INNER
        ld_outer = self.config.LD_AL_OUTER
        outer = gdstk.rectangle(
            (-gap, -gap), (pad_width + gap, (pad_height + gap) * 2), **ld_outer
        )
        outer.fillet(fillet_radius)
        inner = gdstk.rectangle((0, 0), (pad_width, pad_height), **ld_inner)
        inner.fillet(fillet_radius)
        inner2 = inner.copy().translate(0, pad_height + gap)
        outer = gdstk.boolean(outer, [inner, inner2], "not", **ld_outer)
        self.cell.add(*outer, inner, inner2)
        self.create_port("bottom", pad_width / 2 - 1j * gap, np.pi * 3 / 2)
        self.create_port("top", pad_width / 2 + 1j * (pad_height + gap) * 2, np.pi / 2)
        self.create_port(
            "top_right", pad_width + 1j * (pad_height + gap) * 2, np.pi / 2
        )

    @property
    def port_bottom(self):
        return self.ports["bottom"]

    @property
    def port_top(self):
        return self.ports["top"]

    @property
    def port_top_right(self):
        return self.ports["top_right"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = FloatingQubit()
    elem.view()
