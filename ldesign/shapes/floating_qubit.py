from dataclasses import dataclass
import math

import gdstk
import numpy as np

from ldesign import config, elements
import ldesign.chips


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


@dataclass
class FloatingXmonArgs:
    inner_cross_size: float = 800
    inner_arm_width: float = 100
    inner_gap: float = 40
    inner_square_width: float = 280
    outer_cross_size: float = 840
    outer_arm_width: float = 180
    outer_square_width: float = 400
    fillet_radius: float = 10


class FloatingXmon(elements.Element):
    def __init__(
        self, args: FloatingXmonArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = FloatingXmonArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        c = self.config
        args = self.args
        inner = gdstk.cross(0j, args.inner_cross_size, args.inner_arm_width, **c.LD_AL_INNER)
        v_sq = args.inner_square_width*(1+1j)
        sq = gdstk.rectangle(-v_sq/2, v_sq/2).rotate(math.pi / 4)
        v_cut = args.inner_square_width+args.inner_gap*1j
        cut = gdstk.rectangle(-v_cut/2, v_cut/2).rotate(math.pi / 4)
        inner = gdstk.boolean(inner, sq, "or", **c.LD_AL_INNER)
        inner = gdstk.boolean(inner, cut, "not", **c.LD_AL_INNER)
        for p in inner:
            p.fillet(args.fillet_radius)
        outer = gdstk.cross(0j, args.outer_cross_size, args.outer_arm_width, **c.LD_AL_OUTER)
        v_sq = args.outer_square_width*(1+1j)
        sq = gdstk.rectangle(-v_sq/2, v_sq/2).rotate(math.pi / 4)
        outer = gdstk.boolean(outer, sq, "or", **c.LD_AL_OUTER)
        for p in outer:
            p.fillet(args.fillet_radius)
        outer = gdstk.boolean(outer, inner, "not", **c.LD_AL_OUTER)
        self.cell.add(*inner, *outer)

        self.create_port("s", -0.5j*args.outer_cross_size, math.pi * 1.5)
        self.create_port("n", 0.5j*args.outer_cross_size, math.pi * 0.5)
        self.create_port("w", -0.5*args.outer_cross_size+0j, math.pi)
        self.create_port("e", 0.5*args.outer_cross_size+0j, 0)

    @property
    def port_s(self):
        return self.ports["s"]

    @property
    def port_n(self):
        return self.ports["n"]

    @property
    def port_w(self):
        return self.ports["w"]

    @property
    def port_e(self):
        return self.ports["e"]

@dataclass
class HcouplerArgs:
    inner_width: float = 100
    inner_length: float = 858
    outer_width: float = 180
    outer_length: float = 898
        
    bottleneck_width: float = 40
    bottleneck_length: float = 742
        
    fillet_radius: float = 10


class Hcoupler(elements.Element):
    def __init__(
        self, args: HcouplerArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = HcouplerArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        c = self.config
        args = self.args
        v_inner = args.inner_length+1j*args.inner_width
        inner = gdstk.rectangle(-v_inner/2, v_inner/2)
        
        cut_top =  gdstk.rectangle(-args.bottleneck_length/2 + 1j*args.bottleneck_width/2, args.bottleneck_length/2+1j*args.inner_width/2)
        cut_bot =  gdstk.rectangle(-args.bottleneck_length/2 - 1j*args.inner_width/2, args.bottleneck_length/2-1j*args.bottleneck_width/2)
        inner = gdstk.boolean(inner, [cut_top,cut_bot], "not", **c.LD_AL_INNER)
        for p in inner:
            p.fillet(args.fillet_radius)
        
        v_outer = args.outer_length+1j*args.outer_width
        outer = gdstk.rectangle(-v_outer/2, v_outer/2, **c.LD_AL_OUTER)
#         v_sq = args.outer_square_width*(1+1j)
#         sq = gdstk.rectangle(-v_sq/2, v_sq/2).rotate(math.pi / 4)
#         outer = gdstk.boolean(outer, sq, "or", **c.LD_AL_OUTER)
#         for p in outer:
        outer.fillet(args.fillet_radius)
        outer = gdstk.boolean(outer, inner, "not", **c.LD_AL_OUTER)
        self.cell.add(*inner, *outer)

        self.create_port("w", -0.5*args.outer_length+0j, math.pi)
        self.create_port("e", 0.5*args.outer_length+0j, 0)

    @property
    def port_w(self):
        return self.ports["w"]

    @property
    def port_e(self):
        return self.ports["e"]

if __name__ == "__main__":
    config.use_preset_design()
    # chip = ldesign.chips.Chip24Ports()
    # elem = FloatingXmon()
    # chip.add_element(elem, elements.DockingPort(5000+5000j), transformation=elements.Transformation(rotation=math.pi/4))
    # chip.view()

    elem = FloatingXmon()
    # elem.write_gds("demo.gds", test_region=(-1000-1000j, 1000+1000j))
    elem.view()
