import math
from itertools import product
from typing import Sequence

import gdstk
import numpy as np

from ldesign import config, elements
from ldesign.shapes import bondpad, boundary
from ldesign.shapes.marker import FlipChipMarker, MarkerArgs


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
            *gdstk.text(text, size, position, vertical, **self.config.LD_AL_GAP)
        )


class Chip96Ports(elements.Element):
    _WIDTH = 32000
    _PORT_SPACE = 1200
    _PORT_MARGIN = 1100 + 350 - 285
    _PORT_N = 96

    def __init__(self, config: config.Config | None = None) -> None:
        super().__init__(config=config)
        boundary_args = boundary.BoundaryArgs(
            width=self._WIDTH, side_marker_n=1, edge_width=560
        )
        b = boundary.Boundary(boundary_args, config)
        self.add_element(b)
        self.create_port("center", self._WIDTH / 2 * (1 + 1j), 0)

    @property
    def port_center(self):
        return self.ports["center"]

    def place_bondpads(
        self, pads: Sequence[bondpad.BondPad | None]
    ) -> list[bondpad.BondPad | None]:
        """Place `BondPad`s to the chip."""
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
        """Add text polygon to the cell."""
        self.cell.add(
            *gdstk.text(text, size, position, vertical, **self.config.LD_AL_GAP)
        )

    def add_flip_chip(
        self,
        fc_width: float,
        support_width: float,
        support_gap: float,
        marker_offset: complex,
        fc_config: config.Config,
    ):
        """Add flip chip related elements to the chip.

        Arguments:
            fc_width (float): Width of the flip chip.
            support_width (float): Width of the support region.
            support_gap (float): Gap of the support region to the edge of the flip chip.
            marker_offset (complex): Position offset of the flip chip marker.
            fc_config (config.Config): Configurations for the flip chip.
        """
        fc_boundary = boundary.Boundary(
            boundary.BoundaryArgs(width=fc_width), config=fc_config
        )
        self.add_element(
            fc_boundary, self.port_center, fc_boundary.port_center, match_angle=False
        )
        center_point = self.port_center.point
        # Add support
        support = gdstk.rectangle(
            -support_width * (1 + 1j) / 2,
            support_width * (1 + 1j) / 2,
            **self.config.LD_SUPPORT
        )
        for sign, flip in product([1, -1], [False, True]):
            v = (fc_width / 2 - support_gap - support_width / 2) * (1 + 1j)
            if flip:
                v = v.conjugate()
            v *= sign
            s = support.copy().translate(center_point + v)
            self.cell.add(s)
        # Add marker
        fc_marker = FlipChipMarker(config=self.config, config2=fc_config)
        for sign, flip in product([1, -1], [False, True]):
            v = (fc_width / 2 - support_gap - support_width / 4) * (
                1 + 1j
            ) + marker_offset
            if flip:
                v = v.conjugate()
            v *= sign
            self.add_element(
                fc_marker,
                ref_port=elements.DockingPort(center_point + v),
                transformation=elements.Transformation(
                    rotation=math.pi if sign == 1 else 0, x_reflection=flip
                ),
            )

class Chip220Ports(elements.Element):
    '''This is a chip class which has 220 ports

    This class is stilling under developing by Xuandong. 
    There are 60 ports on left and right sides. 
    There are 50 ports on up and down sides. 
    The default port-to-port space(_PORT_SPACE) is 700um
    '''
    _HEIGHT = 44300 # chip height=700*(60-1)+1500*2
    _WIDTH = 37300 # chip width=700*(50-1)+1500*2
    _PORT_SPACE = 700 #distance between pads
    _PORT_MARGIN = 1100 + 350 - 285 # margin between pad and boundary of chip
    _PORT_N_HEIGHT = 60 #number of pads on height
    _PORT_N_WIDTH = 50 #number of pads on width
    _PORT_N = 220 # total number of pads

    def __init__(self, config: config.Config | None = None) -> None:
        super().__init__(config=config)
        boundary_args = boundary.BoundaryArgs(
            width=self._WIDTH, height = self._HEIGHT, side_marker_n=1, edge_width=560
        ) #create dataclass "BoundaryArgs"
        b = boundary.Boundary(boundary_args, config)#instantiate a boundary class
        self.add_element(b)#add boundary cell to chip
        self.create_port(
            name = "center", 
            point = self._WIDTH / 2 + (self._HEIGHT / 2) * 1j, 
            angle = 0)#set the center position of chip

    @property
    def port_center(self):
        return self.ports["center"]

    def place_bondpads(
        self, pads: Sequence[bondpad.BondPad | None]
    ) -> list[bondpad.BondPad | None]:
        """Place `BondPad`s to the chip."""
        if len(pads) != self._PORT_N: #check whether the number of input pads is equal to number of ports 
            raise ValueError
        center = self._WIDTH / 2 + (self._HEIGHT / 2) * 1j # center of chip
        ######set the position of pads on left side
        x0 = self._PORT_MARGIN - self._WIDTH / 2 #relative x distance from center
        y0 = -np.arange(self._PORT_N_HEIGHT, dtype=float) * self._PORT_SPACE
        y0 -= np.mean(y0)#relative y distance from center
        p0 = x0 + 1j * y0# relative position array
        points = p0 + center #record the absolute position of pads
        angles = - math.pi * np.ones(self._PORT_N_HEIGHT)#record port angle of pads
        ######set the position of pads on down side
        y1 = self._PORT_MARGIN - self._HEIGHT / 2 #relative y distance from center
        x1 = -np.arange(self._PORT_N_WIDTH, dtype=float) * self._PORT_SPACE
        x1 -= np.mean(x1)#relative x distance from center
        p1 = x1 + 1j * y1# relative position array
        points = np.append(points, p1 + center) #record the absolute position of pads
        angles = np.append(angles, - 1 / 2 * math.pi * np.ones(self._PORT_N_WIDTH))#record port angle of pads
        ######set the position of pads on right side
        points = np.append(points, - p0 + center) #record the absolute position of pads
        angles = np.append(angles, 0 * np.ones(self._PORT_N_HEIGHT))#record port angle of pads
        ######set the position of pads on up side
        points = np.append(points, - p1 + center) #record the absolute position of pads
        angles = np.append(angles, 1 / 2 * math.pi * np.ones(self._PORT_N_WIDTH))#record port angle of pads

        ports = [elements.DockingPort(p, a, self) for p, a in zip(points, angles)]#set docking ports
        child_pads = []
        for dp, pad in zip(ports, pads):
            if pad is not None:
                pad = self.add_element(pad, dp, pad.port_line) #add pads to chip
                child_pads.append(pad)
            else:
                child_pads.append(None)
        return child_pads

    def add_text(
        self, text: str, size: float, position: complex, vertical: bool = False
    ) -> None:
        """Add text polygon to the cell."""
        self.cell.add(
            *gdstk.text(text, size, position, vertical, **self.config.LD_AL_GAP)
        )

    def add_flip_chip(
        self,
        fc_width: float,
        support_width: float,
        support_gap: float,
        marker_offset: complex,
        fc_config: config.Config,
    ):
        """Add flip chip related elements to the chip.

        Arguments:
            fc_width (float): Width of the flip chip.
            support_width (float): Width of the support region.
            support_gap (float): Gap of the support region to the edge of the flip chip.
            marker_offset (complex): Position offset of the flip chip marker.
            fc_config (config.Config): Configurations for the flip chip.
        """
        fc_boundary = boundary.Boundary(
            boundary.BoundaryArgs(width=fc_width), config=fc_config
        )
        self.add_element(
            fc_boundary, self.port_center, fc_boundary.port_center, match_angle=False
        )
        center_point = self.port_center.point
        # Add support
        support = gdstk.rectangle(
            -support_width * (1 + 1j) / 2,
            support_width * (1 + 1j) / 2,
            **self.config.LD_SUPPORT
        )
        for sign, flip in product([1, -1], [False, True]):
            v = (fc_width / 2 - support_gap - support_width / 2) * (1 + 1j)
            if flip:
                v = v.conjugate()
            v *= sign
            s = support.copy().translate(center_point + v)
            self.cell.add(s)
        # Add marker
        fc_marker = FlipChipMarker(config=self.config, config2=fc_config)
        for sign, flip in product([1, -1], [False, True]):
            v = (fc_width / 2 - support_gap - support_width / 4) * (
                1 + 1j
            ) + marker_offset
            if flip:
                v = v.conjugate()
            v *= sign
            self.add_element(
                fc_marker,
                ref_port=elements.DockingPort(center_point + v),
                transformation=elements.Transformation(
                    rotation=math.pi if sign == 1 else 0, x_reflection=flip
                ),
            )


if __name__ == "__main__":
    config.use_preset_design()
    fc_config = config.Config()
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
    c = Chip96Ports()
    c.add_flip_chip(14000, 2000, 350, -2000 + 0j, fc_config)
    # xy_pad = bondpad.BondPad()
    # ro_pad = bondpad.BondPad(bondpad.BondPadArgs(path.CpwArgs(10, 6)))
    # pads = [xy_pad] * 24
    # for i in [7, 10, 19, 22]:
    #     pads[i] = ro_pad
    # c.place_bondpads(pads)
    c.view()
