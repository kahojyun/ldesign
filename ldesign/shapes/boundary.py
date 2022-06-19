from dataclasses import dataclass, field
from itertools import product

import gdstk
import numpy as np
from ldesign import config, elements
from ldesign.shapes.marker import Marker, MarkerArgs


@dataclass
class BoundaryArgs:
    width: float = 6160
    height: float | None = None
    edge_width: float = 10
    corner_marker_gap: float = 44
    side_marker_gap: float = 0
    side_marker_n: int = 0
    corner_marker: MarkerArgs = field(default_factory=MarkerArgs)
    side_marker: MarkerArgs = field(default_factory=MarkerArgs)


class Boundary(elements.Element):
    def __init__(
        self, args: BoundaryArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = BoundaryArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        ld_outer = self.config.LD_AL_OUTER
        args = self.args
        height = args.height
        width = args.width
        edge_width = args.edge_width

        def get_mask(size):
            mask = gdstk.rectangle(
                (-size / 2, -size / 2), (size / 2, size / 2), **ld_outer
            )
            return mask

        if height is None:
            height = width
        boundary = gdstk.FlexPath(
            [(0, 0), (width, 0), (width, height), (0, height), (0, 0)],
            edge_width,
            ends="extended",
            **ld_outer
        )

        # corner marker
        mask = get_mask(
            args.corner_marker.outer_marker_size + args.corner_marker_gap * 2
        )
        mask.repetition = gdstk.Repetition(2, 2, (width, height))
        masks = [mask, *mask.apply_repetition()]
        boundary = gdstk.boolean(boundary, masks, "not", **ld_outer)
        corner_marker = Marker(args.corner_marker, self.config)
        for x, y in product((0, width), (0, height)):
            self.add_element(corner_marker, elements.DockingPort(x + 1j * y))

        # side marker
        if args.side_marker_n > 0:
            mask = get_mask(
                args.side_marker.outer_marker_size + args.side_marker_gap * 2
            )
            side_marker = Marker(args.side_marker, self.config)
            side_x = np.linspace(0, width, args.side_marker_n + 2)[1:-1]
            side_y = np.linspace(0, height, args.side_marker_n + 2)[1:-1]
            masks = []
            for x in side_x:
                masks.append(mask.copy().translate(x + 0j))
                masks.append(mask.copy().translate(x + height * 1j))
                self.add_element(side_marker, elements.DockingPort(x + 0j))
                self.add_element(side_marker, elements.DockingPort(x + height * 1j))
            for y in side_y:
                masks.append(mask.copy().translate(y * 1j))
                masks.append(mask.copy().translate(width + y * 1j))
                self.add_element(side_marker, elements.DockingPort(y * 1j))
                self.add_element(side_marker, elements.DockingPort(width + y * 1j))
            boundary = gdstk.boolean(boundary, masks, "not", **ld_outer)

        self.cell.add(*boundary)


if __name__ == "__main__":
    config.use_preset_design()
    elem = Boundary(BoundaryArgs(width=30000, side_marker_n=3))
    elem.view()
