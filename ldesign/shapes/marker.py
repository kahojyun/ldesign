import math
from dataclasses import dataclass

import gdstk
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
                -size * (1 + 1j) / 2, size * (1 + 1j) / 2, **c.LD_AL_OUTER
            )

        cross = [
            gdstk.cross(
                (0, 0),
                args.outer_marker_size,
                args.outer_marker_arm_width,
                **c.LD_AL_OUTER
            )
        ]

        if args.star:
            star_cross = gdstk.cross(
                (0, 0),
                args.outer_marker_size,
                args.outer_marker_arm_width,
                **c.LD_AL_OUTER
            )
            star_cut_size = args.star_cut_size
            if star_cut_size is None:
                star_cut_size = 3 * args.outer_marker_arm_width
            star_cut = get_mask(star_cut_size)
            star_cross.rotate(math.pi / 4, 0j)
            star_cut.rotate(math.pi / 4, 0j)
            star_cross = gdstk.boolean(star_cross, star_cut, "not", **c.LD_AL_OUTER)
            cross = gdstk.boolean(cross, star_cross, "or", **c.LD_AL_OUTER)
        # make room for inner marker
        mask = get_mask(args.inner_marker_size + 2 * args.inner_marker_gap)
        cross = gdstk.boolean(cross, mask, "not", **c.LD_AL_OUTER)
        # inner marker
        cross = gdstk.boolean(
            cross, get_mask(args.inner_marker_size), "or", **c.LD_AL_OUTER
        )
        inner_cross = gdstk.cross(
            (0, 0), args.inner_marker_size, args.inner_marker_arm_width, **c.LD_AL_OUTER
        )
        cross = gdstk.boolean(cross, inner_cross, "not", **c.LD_AL_OUTER)
        self.cell.add(*cross)


@dataclass
class FlipChipMarkerArgs:
    outer_marker_size: float = 612
    outer_marker_arm_width: float = 10
    inner_marker_gap: float = 2
    inner_marker_size: float = 8
    inner_marker_arm_width: float = 1
    star: bool = False
    star_cut_size: float | None = None


class FlipChipMarker(elements.Element):
    def __init__(
        self,
        args: FlipChipMarkerArgs | None = None,
        config: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = FlipChipMarkerArgs()
        self.args = args
        self._init_cell()

    def _init_cell(
        self,
    ):
        c = self.config
        args = self.args


if __name__ == "__main__":
    config.use_preset_design()
    Marker(MarkerArgs(star=True)).view()
