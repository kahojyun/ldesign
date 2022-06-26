import math
from dataclasses import dataclass, field

import gdstk
import ldesign
import numpy as np
from ldesign import config, elements


@dataclass
class CpwArgs:
    width: float = 4
    gap: float = 2


@dataclass
class FluxEndArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    right_gap_len: float = 1
    left_gap_len: float = 7
    back_gap_len: float = 7


class OpenEnd(elements.Element):
    def __init__(
        self, args: CpwArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = CpwArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        width = 2 * self.args.gap + self.args.width
        height = self.args.gap
        self.cell.add(
            gdstk.rectangle(0j, width + height * 1j, **self.config.LD_AL_OUTER)
        )
        self.create_port("line", width / 2 + 0j, np.pi * 3 / 2)
        self.create_port("end", width / 2 + 1j * height, np.pi / 2)

    @property
    def port_line(self):
        return self.ports["line"]


class FluxEnd(elements.Element):
    def __init__(
        self, args: FluxEndArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = FluxEndArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        ld_inner = self.config.LD_AL_INNER
        ld_outer = self.config.LD_AL_OUTER
        line_width = self.args.cpw.width
        gap = self.args.cpw.gap
        l_length = min(self.args.left_gap_len, self.args.right_gap_len)
        inner = [
            gdstk.rectangle(
                line_width / 2 - 1j * (gap + line_width), -line_width / 2 + 0j
            ),
            gdstk.rectangle(
                line_width / 2 - 1j * (gap + line_width),
                -line_width / 2 - gap - l_length - 1j * gap,
            ),
        ]
        inner = gdstk.boolean(inner, [], "or", **ld_inner)
        outer = [
            gdstk.rectangle(
                0j, -line_width / 2 - gap - self.args.right_gap_len - 1j * gap
            ),
            gdstk.rectangle(
                -line_width / 2
                - gap
                - self.args.left_gap_len
                - 1j * (gap + line_width),
                line_width / 2
                + gap
                + self.args.back_gap_len
                - 1j * (2 * gap + line_width),
            ),
            gdstk.rectangle(
                line_width / 2 + 0j, line_width / 2 + gap - 1j * (2 * gap + line_width)
            ),
        ]
        outer = gdstk.boolean(outer, inner, "not", **ld_outer)
        self.cell.add(*inner, *outer)
        self.create_port("line", 0j, np.pi / 2)
        self.create_port("flux", -1j * (2 * gap + line_width), np.pi * 3 / 2)

    @property
    def port_line(self):
        return self.ports["line"]

    @property
    def port_flux(self):
        return self.ports["flux"]


@dataclass
class FluxEnd3DArgs:
    cpw: CpwArgs = field(default_factory=CpwArgs)
    square_width: float = 40
    turn_radius: float = 8
    extend_length: float = 2
    end_gap: float = 4
    slant_len: float | None = None
    cut_ground: bool = False


class FluxEnd3D(elements.Element):
    def __init__(
        self, args: FluxEnd3DArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = FluxEnd3DArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        args = self.args
        r = args.turn_radius
        ext = args.extend_length
        w = args.square_width
        cpw = args.cpw
        options = ldesign.path.PathOptions(radius=r, cpw=cpw, parent_element=self)
        if args.slant_len is None:
            gen = ldesign.path.PathOpGenerator((r + ext) * 1j, options=options)
        else:
            gen = ldesign.path.PathOpGenerator(
                (r + args.slant_len) * (-1 + 1j), options=options
            )
            gen.segment(0j)
        gen.segment(-w * 1j)
        gen.segment(w - w * 1j)
        gen.segment(w - r * 1j, angle=math.pi / 2)
        gen.segment(cpw.width + cpw.gap * 2 + args.end_gap + r * 1j, angle=math.pi / 2)
        gen.extend(ext)
        ops = gen.build()
        line = ldesign.path.create_cpw_from_ops(ops, options=options, cfg=self.config)
        line = self.add_element(line)
        line_port = line.port_start.get_transformed_port(self)
        self.create_port("line", line_port.point, line_port.angle)
        self.create_port("flux", w / 2 - w / 2 * 1j, -math.pi / 2)
        if args.cut_ground:
            ew = cpw.width / 2 + cpw.gap
            cut = gdstk.rectangle(
                (ew - self.config.tolerance) * (-1 + 1j),
                (w + ew - self.config.tolerance) * (1 - 1j),
                **self.config.LD_AL_OUTER
            )
            cut = cut.fillet(r + ew, tolerance=self.config.tolerance)
            self.cell.add(cut)
            self.flatten()

    @property
    def port_line(self):
        return self.ports["line"]

    @property
    def port_flux(self):
        return self.ports["flux"]


class StraightCpw(elements.CpwWaveguide):
    def __init__(
        self,
        length: float,
        args: CpwArgs | None = None,
        config: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = CpwArgs()
        self.args = args
        self._init_cell(length)

    def _init_cell(self, length: float):
        inner = gdstk.FlexPath(
            [0j, length + 0j], self.args.width, **self.config.LD_AL_INNER
        )
        outer = gdstk.FlexPath([0j, length + 0j], self.args.width + 2 * self.args.gap)
        outer = gdstk.boolean(outer, inner, "not", **self.config.LD_AL_OUTER)
        self.cell.add(*outer, inner)
        self.create_port("start", 0j, math.pi)
        self.create_port("end", length + 0j, 0)


if __name__ == "__main__":
    config.use_preset_design()
    FluxEnd3D(FluxEnd3DArgs(slant_len=None, cut_ground=True)).view()
