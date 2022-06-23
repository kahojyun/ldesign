import math
from typing import Sequence

import gdstk
import numpy as np

from ldesign import config, elements
from ldesign.shapes import bondpad, boundary, path
from ldesign.shapes.marker import MarkerArgs


class Chip24Ports(elements.Element):
    _WIDTH = 10700
    _PORT_SPACE = 1500
    _PORT_MARGIN = 1070
    _PORT_N = 24

    def __init__(self, config: config.Config | None = None) -> None:
        super().__init__(config=config)
        boundary_args = boundary.BoundaryArgs(
            width=self._WIDTH, corner_marker=MarkerArgs(star=True)
        )
        b = boundary.Boundary(boundary_args, config)
        self.add_element(b)

    def place_bondpads(
        self, pads: Sequence[bondpad.BondPad | None]
    ) -> list[bondpad.BondPad | None]:
        if len(pads) != self._PORT_N:
            raise ValueError
        n_port_per_edge = self._PORT_N // 4
        center = self._WIDTH * (0.5 + 0.5j)
        x0 = self._PORT_MARGIN - self._WIDTH / 2
        y0 = -np.arange(n_port_per_edge, dtype=float) * self._PORT_SPACE
        y0 -= np.mean(y0)
        p0 = x0 + 1j * y0
        points = np.outer([1, 1j, -1, -1j], p0).flatten() + center
        angles = np.outer(
            np.arange(4) * math.pi / 2 - math.pi, np.ones(n_port_per_edge)
        ).flatten()
        ports = [elements.DockingPort(p, a, self) for p, a in zip(points, angles)]
        child_pads = []
        for dp, pad in zip(ports, pads):
            if pad is not None:
                pad = self.add_element(pad, dp, pad.port_line)
                child_pads.append(pad)
            else:
                child_pads.append(None)
        return child_pads

    def add_text(
        self, text: str, size: float, position: complex, vertical: bool = False
    ) -> None:
        self.cell.add(
            *gdstk.text(text, size, position, vertical, **self.config.LD_AL_OUTER)
        )

class Chip96Ports(elements.Element):
    _WIDTH = 32000
    _PORT_SPACE = 1200
    _PORT_MARGIN = 1000
    _PORT_N = 96

    def __init__(self, config: config.Config | None = None) -> None:
        super().__init__(config=config)
        boundary_args = boundary.BoundaryArgs(width=self._WIDTH, star=True)
        b = boundary.Boundary(boundary_args, config)
        self.add_element(b)

    def place_bondpads(
        self, pads: Sequence[bondpad.BondPad | None]
    ) -> list[bondpad.BondPad | None]:
        if len(pads) != self._PORT_N:
            raise ValueError
        n_port_per_edge = self._PORT_N // 4
        center = self._WIDTH * (0.5 + 0.5j)
        x0 = self._PORT_MARGIN - self._WIDTH / 2
        y0 = -np.arange(n_port_per_edge, dtype=float) * self._PORT_SPACE
        y0 -= np.mean(y0)
        p0 = x0 + 1j * y0
        points = np.outer([1, 1j, -1, -1j], p0).flatten() + center
        angles = np.outer(
            np.arange(4) * math.pi / 2 - math.pi, np.ones(n_port_per_edge)
        ).flatten()
        ports = [elements.DockingPort(p, a, self) for p, a in zip(points, angles)]
        child_pads = []
        for dp, pad in zip(ports, pads):
            if pad is not None:
                pad = self.add_element(pad, dp, pad.port_line)
                child_pads.append(pad)
            else:
                child_pads.append(None)
        return child_pads

    def add_text(
        self, text: str, size: float, position: complex, vertical: bool = False
    ) -> None:
        self.cell.add(
            *gdstk.text(text, size, position, vertical, **self.config.LD_AL_OUTER)
        )

if __name__ == "__main__":
    config.use_preset_design()
    c = Chip24Ports()
    xy_pad = bondpad.BondPad()
    ro_pad = bondpad.BondPad(bondpad.BondPadArgs(path.CpwArgs(10, 6)))
    pads = [xy_pad] * 24
    for i in [7, 10, 19, 22]:
        pads[i] = ro_pad
    c.place_bondpads(pads)
    c.view()
