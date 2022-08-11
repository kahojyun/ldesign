import cmath
import copy
import math
from dataclasses import dataclass
from itertools import product

import gdstk

import ldesign
from ldesign import config, elements


@dataclass
class MarkerArgs:
    outer_marker_size: float = 612
    outer_marker_arm_width: float = 10
    inner_marker_gap: float = 2
    inner_marker_size: float = 8
    inner_marker_arm_width: float = 1
    star: bool = False
    star_cut_size: float | None = None


class Marker(elements.Element):
    def __init__(
        self,
        args: MarkerArgs | None = None,
        config: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = MarkerArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        c = self.config
        args = self.args

        def get_mask(size: float):
            return gdstk.rectangle(
                -size * (1 + 1j) / 2, size * (1 + 1j) / 2, **c.LD_AL_GAP
            )

        cross = [
            gdstk.cross(
                (0, 0),
                args.outer_marker_size,
                args.outer_marker_arm_width,
                **c.LD_AL_GAP
            )
        ]

        if args.star:
            star_cross = gdstk.cross(
                (0, 0),
                args.outer_marker_size,
                args.outer_marker_arm_width,
                **c.LD_AL_GAP
            )
            star_cut_size = args.star_cut_size
            if star_cut_size is None:
                star_cut_size = 3 * args.outer_marker_arm_width
            star_cut = get_mask(star_cut_size)
            star_cross.rotate(math.pi / 4, 0j)
            star_cut.rotate(math.pi / 4, 0j)
            star_cross = gdstk.boolean(star_cross, star_cut, "not", **c.LD_AL_GAP)
            cross = gdstk.boolean(cross, star_cross, "or", **c.LD_AL_GAP)
        # make room for inner marker
        mask = get_mask(args.inner_marker_size + 2 * args.inner_marker_gap)
        cross = gdstk.boolean(cross, mask, "not", **c.LD_AL_GAP)
        # inner marker
        cross = gdstk.boolean(
            cross, get_mask(args.inner_marker_size), "or", **c.LD_AL_GAP
        )
        inner_cross = gdstk.cross(
            (0, 0), args.inner_marker_size, args.inner_marker_arm_width, **c.LD_AL_GAP
        )
        cross = gdstk.boolean(cross, inner_cross, "not", **c.LD_AL_GAP)
        self.cell.add(*cross)


@dataclass
class FlipChipMarkerArgs:
    n_tick: int = 6
    major_tick_width: float = 4.5
    major_tick_length: float = 44
    minor_tick_width: float = 4.5
    minor_tick_length: float = 22
    first_tick: float = 20
    tick_spacing: float = 20
    cross_width: float = 60
    cross_arm: float = 4
    cross_pos: float = 40
    with_metal: bool = True
    metal_width: float = 350


class FlipChipMarker(elements.Element):
    def __init__(
        self,
        args: FlipChipMarkerArgs | None = None,
        config: config.Config | None = None,
        config2: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = FlipChipMarkerArgs()
        self.args = args
        if config2 is None:
            config2 = ldesign.config.global_config
        self.config2 = config2
        self._init_cell()

    def _init_cell(
        self,
    ):
        c = self.config
        c2 = self.config2
        args = self.args
        cross = gdstk.cross(0j, args.cross_width, args.cross_arm, **c2.LD_AL_GAP)
        v = args.cross_width * (1 + 1j)
        rect = gdstk.rectangle(-v / 2, v / 2, **c.LD_AL_GAP)
        rect = gdstk.boolean(rect, cross, "not", **c.LD_AL_GAP)
        v = (args.cross_width / 2 + args.cross_pos) * (1 - 1j)
        self.cell.add(cross.copy().translate(v))
        self.cell.add(cross.copy().translate(-v))
        for p in rect:
            self.cell.add(p.copy().translate(v))
            self.cell.add(p.copy().translate(-v))
        v = args.major_tick_width + args.major_tick_length / 2 * 1j
        major1 = gdstk.rectangle(0j, v, **c.LD_AL_GAP)
        major2 = gdstk.rectangle(0j, v.conjugate(), **c2.LD_AL_GAP)
        v = args.minor_tick_width + args.minor_tick_length / 2 * 1j
        minor1 = gdstk.rectangle(0j, v, **c.LD_AL_GAP)
        minor2 = gdstk.rectangle(0j, v.conjugate(), **c2.LD_AL_GAP)
        for i, j in product(range(args.n_tick), range(4)):
            a = math.pi / 2 * j
            v = cmath.rect(args.first_tick + i * args.tick_spacing, a)
            self.cell.add(major1.copy().transform(translation=v, rotation=a))
            self.cell.add(major2.copy().transform(translation=v, rotation=a))
            v = cmath.rect(args.first_tick + (i + 0.5) * args.tick_spacing, a)
            self.cell.add(minor1.copy().transform(translation=v, rotation=a))
            self.cell.add(minor2.copy().transform(translation=v, rotation=a))
        if args.with_metal:
            v = args.metal_width * (1 + 1j)
            metal1 = gdstk.rectangle(-v / 2, v / 2, **c.LD_AL_OUTER)
            metal2 = gdstk.rectangle(-v / 2, v / 2, **c2.LD_AL_OUTER)
            self.cell.add(metal1, metal2)
            self.flatten(chips=[c.LD_AL_OUTER["datatype"], c2.LD_AL_OUTER["datatype"]])


if __name__ == "__main__":
    config.use_preset_design()
    base_config = copy.copy(config.global_config)
    fc_config = copy.copy(base_config)
    fc_config.LD_AL_GAP = {"layer": 1, "datatype": 1}
    fc_config.LD_AL_INNER = {"layer": 101, "datatype": 1}
    fc_config.LD_AL_OUTER = {"layer": 102, "datatype": 1}
    fc_config.LD_BANDAGE = {"layer": 7, "datatype": 1}
    fc_config.LD_JJ_PAD = {"layer": 6, "datatype": 1}
    fc_config.LD_JJ = {"layer": 5, "datatype": 1}
    fc_config.LD_BRIDGE_UNDER = {"layer": 4, "datatype": 1}
    fc_config.LD_BRIDGE = {"layer": 3, "datatype": 1}
    fc_config.LD_SUPPORT = {"layer": 99, "datatype": 1}
    fc_config.LD_BRIDGE_VIA = {"layer": 100, "datatype": 1}
    fc_config.LD_LABEL = {"layer": 0, "texttype": 0}
    FlipChipMarker(config=base_config, config2=fc_config).view()
