import math
from dataclasses import dataclass, field

import gdstk

from ldesign import config, elements, shapes


@dataclass
class CrossoverArgs:
    cpw_under: shapes.path.CpwArgs = field(default_factory=shapes.path.CpwArgs)
    cpw_over: shapes.path.CpwArgs = field(default_factory=shapes.path.CpwArgs)
    gap: float = 2
    length_in: float = 7.5
    length_pad: float = 2.5
    length_trans: float = 2.5

    @property
    def total_length(self) -> float:
        return (
            self.cpw_under.gap * 2
            + self.cpw_under.width
            + 2 * self.gap
            + 2 * self.cpw_over.gap
            + 2 * self.length_in
        )


class Crossover(elements.Element):
    def __init__(
        self, args: CrossoverArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = CrossoverArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        args = self.args
        cpw_over = args.cpw_over
        total_length = args.total_length
        self.create_port("start", 0j, math.pi)
        self.create_port("end", total_length + 0j, 0)
        self.create_port("center", total_length / 2 + 0j, math.pi)

        length_sub = total_length - 2 * (args.length_pad + args.length_trans)
        bridge = Bridge(
            BridgeArgs(
                width_pad=cpw_over.width,
                width_top=cpw_over.width - 1,
                width_sub=cpw_over.width + 1,
                length_pad=args.length_pad,
                length_trans=args.length_trans,
                length_sub=length_sub,
            ),
            self.config,
        )
        bridge = self.add_element(
            bridge,
            self.port_start,
            bridge.port_start,
            elements.Transformation(rotation=math.pi),
        )

        cpw_gap = [
            gdstk.FlexPath(
                [0j, args.length_in + cpw_over.gap + 0j],
                cpw_over.width + 2 * cpw_over.gap,
            ),
            gdstk.FlexPath(
                [total_length + 0j, total_length - args.length_in - cpw_over.gap + 0j],
                cpw_over.width + 2 * cpw_over.gap,
            ),
        ]
        cpw_inner = [
            gdstk.FlexPath(
                [0j, args.length_in + 0j], cpw_over.width, **self.config.LD_AL_INNER
            ),
            gdstk.FlexPath(
                [total_length + 0j, total_length - args.length_in + 0j],
                cpw_over.width,
                **self.config.LD_AL_INNER
            ),
        ]
        cpw_gap = gdstk.boolean(cpw_gap, cpw_inner, "not", **self.config.LD_AL_GAP)
        self.cell.add(*cpw_inner, *cpw_gap)

    @property
    def port_start(self):
        return self.ports["start"]

    @property
    def port_end(self):
        return self.ports["end"]

    @property
    def port_center(self):
        return self.ports["center"]


@dataclass
class BridgeArgs:
    width_pad: float = 4
    width_top: float = 3
    width_sub: float = 5
    length_pad: float = 2.5
    length_trans: float = 2.5
    length_sub: float = 14


class Bridge(elements.Element):
    def __init__(
        self, args: BridgeArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = BridgeArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        width_pad = self.args.width_pad
        width_top = self.args.width_top
        width_sub = self.args.width_sub
        length_pad = self.args.length_pad
        length_trans = self.args.length_trans
        length_sub = self.args.length_sub
        sub = gdstk.FlexPath(
            [0j, length_sub + 0j], width_sub, **self.config.LD_BRIDGE_UNDER
        )
        top = gdstk.FlexPath(
            [-length_pad - length_trans + 0j], width_pad, **self.config.LD_BRIDGE
        )
        top.horizontal(length_pad, relative=True)
        top.horizontal(length_trans, width_top, relative=True)
        top.horizontal(length_sub, relative=True)
        top.horizontal(length_trans, width_pad, relative=True)
        top.horizontal(length_pad, relative=True)
        via1 = gdstk.rectangle(
            -length_pad - length_trans - width_pad / 2 * 1j,
            -length_trans + width_pad / 2 * 1j,
            **self.config.LD_BRIDGE_VIA
        )
        via2 = gdstk.rectangle(
            length_sub + length_trans - width_pad / 2 * 1j,
            length_sub + length_trans + length_pad + width_pad / 2 * 1j,
            **self.config.LD_BRIDGE_VIA
        )
        self.cell.add(sub, top, via1, via2)
        self.create_port("start", -length_pad - length_trans + 0j, math.pi)
        self.create_port("end", length_sub + length_trans + length_pad + 0j, 0)
        self.create_port("center", length_sub / 2 + 0j, math.pi)

    @property
    def port_start(self):
        return self.ports["start"]

    @property
    def port_end(self):
        return self.ports["end"]

    @property
    def port_center(self):
        return self.ports["center"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = Crossover()
    cpw = shapes.path.StraightCpw(30)
    elem.add_element(
        cpw,
        elem.port_center,
        cpw.port_start,
        elements.Transformation(15j, -math.pi / 2),
    )
    elem.write_gds("demo.gds")
    elem.view()
