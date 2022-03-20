from dataclasses import asdict, dataclass

import gdstk
import numpy as np
from ldesign import config, elements


@dataclass
class BoundaryArgs:
    width: float = 6160
    height: float | None = None
    edge_width: float = 10
    outer_marker_gap: float = 44
    outer_marker_size: float = 612
    inner_marker_gap: float = 2
    inner_marker_size: float = 8
    inner_marker_arm_width: float = 1
    star: bool = False
    star_cut_size: float | None = None


boundary_12port = BoundaryArgs()
boundary_24port = BoundaryArgs(width=10700)


class Boundary(elements.Element):
    def __init__(
        self, args: BoundaryArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = BoundaryArgs()
        self.args = args
        self._init_cell(**asdict(args))

    def _init_cell(
        self,
        edge_width,
        height,
        inner_marker_arm_width,
        inner_marker_gap,
        inner_marker_size,
        outer_marker_gap,
        outer_marker_size,
        width,
        star,
        star_cut_size,
    ):
        ld_outer = self.config.LD_AL_OUTER

        def get_mask(size):
            mask = gdstk.rectangle(
                (-size / 2, -size / 2), (size / 2, size / 2), **ld_outer
            )
            mask.repetition = gdstk.Repetition(2, 2, (width, height))
            return [mask, *mask.apply_repetition()]

        def get_cross(size, cross_width):
            cross = gdstk.cross((0, 0), size, cross_width, **ld_outer)
            cross.repetition = gdstk.Repetition(2, 2, (width, height))
            return [cross, *cross.apply_repetition()]

        if height is None:
            height = width
        boundary = gdstk.FlexPath(
            [(0, 0), (width, 0), (width, height), (0, height), (0, 0)],
            edge_width,
            ends="extended",
            **ld_outer
        )
        # make room for outer marker
        boundary = gdstk.boolean(
            boundary,
            get_mask(outer_marker_size + outer_marker_gap * 2),
            "not",
            **ld_outer
        )
        # outer marker
        boundary = gdstk.boolean(
            boundary, get_cross(outer_marker_size, edge_width), "or", **ld_outer
        )
        if star:
            star_cross = gdstk.cross((0, 0), outer_marker_size, edge_width, **ld_outer)
            if star_cut_size is None:
                star_cut_size = 3 * edge_width
            star_cut = gdstk.rectangle(
                (-star_cut_size / 2, -star_cut_size / 2),
                (star_cut_size / 2, star_cut_size / 2),
                **ld_outer
            )
            star_cross.rotate(np.pi / 4, 0j)
            star_cut.rotate(np.pi / 4, 0j)
            star_cross.repetition = gdstk.Repetition(2, 2, (width, height))
            star_cut.repetition = gdstk.Repetition(2, 2, (width, height))
            star_cross = gdstk.boolean(
                [star_cross, *star_cross.apply_repetition()],
                [star_cut, *star_cut.apply_repetition()],
                "not",
                **ld_outer
            )
            boundary = gdstk.boolean(boundary, star_cross, "or", **ld_outer)
        # make room for inner marker
        boundary = gdstk.boolean(
            boundary,
            get_mask(inner_marker_size + inner_marker_gap * 2),
            "not",
            **ld_outer
        )
        # inner marker
        boundary = gdstk.boolean(
            boundary, get_mask(inner_marker_size), "or", **ld_outer
        )
        boundary = gdstk.boolean(
            boundary,
            get_cross(inner_marker_size, inner_marker_arm_width),
            "not",
            **ld_outer
        )
        self.cell.add(*boundary)
        self.create_port("origin", 0j, 0)

    @property
    def port_origin(self):
        return self.ports["origin"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = Boundary(BoundaryArgs(star=True))
    elem.view()
