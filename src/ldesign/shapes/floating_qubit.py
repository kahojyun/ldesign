import cmath
import math
from dataclasses import dataclass

import gdstk
from ldesign import config, elements


@dataclass
class FloatingQubitArgs:
    gap: float = 30
    pad_width: float = 500
    pad_height: float = 120
    fillet_radius: float = 10
    with_gap: bool = True


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
        ld_gap = self.config.LD_AL_GAP

        al_inner_poly = gdstk.rectangle((0, 0), (pad_width, pad_height), **ld_inner)
        al_inner_poly.fillet(fillet_radius, tolerance=self.config.tolerance)
        al_inner_poly2 = al_inner_poly.copy().translate(0, pad_height + gap)
        if args.with_gap:
            al_gap_poly = gdstk.rectangle(
                (-gap, -gap), (pad_width + gap, (pad_height + gap) * 2), **ld_gap
            )
            al_gap_poly.fillet(fillet_radius, tolerance=self.config.tolerance)
            al_gap_poly = gdstk.boolean(
                al_gap_poly, [al_inner_poly, al_inner_poly2], "not", **ld_gap
            )
            self.cell.add(*al_gap_poly)
            self.create_port("bottom", pad_width / 2 - 1j * gap, math.pi * 3 / 2)
            self.create_port(
                "top", pad_width / 2 + 1j * (pad_height + gap) * 2, math.pi / 2
            )
            self.create_port(
                "top_right", pad_width + 1j * (pad_height + gap) * 2, math.pi / 2
            )
        else:
            self.create_port("bottom", pad_width / 2 + 0j, math.pi * 3 / 2)
            self.create_port(
                "top", pad_width / 2 + 1j * (pad_height * 2 + gap), math.pi / 2
            )
            self.create_port(
                "top_right", pad_width + 1j * (pad_height * 2 + gap), math.pi / 2
            )
        self.cell.add(al_inner_poly, al_inner_poly2)
        self.create_port(
            "squid", pad_width / 2 + 1j * (pad_height + gap / 2), math.pi / 2
        )

    @property
    def port_squid(self):
        return self.ports["squid"]

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
    with_gap: bool = True


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
        al_inner_poly = gdstk.cross(
            0j, args.inner_cross_size, args.inner_arm_width, **c.LD_AL_INNER
        )
        v_sq = args.inner_square_width * (1 + 1j)
        sq = gdstk.rectangle(-v_sq / 2, v_sq / 2).rotate(math.pi / 4)
        # ensure cut region is larger than square
        v_cut = args.inner_square_width + 1 + args.inner_gap * 1j
        cut = gdstk.rectangle(-v_cut / 2, v_cut / 2).rotate(math.pi / 4)
        al_inner_poly = gdstk.boolean(al_inner_poly, sq, "or", **c.LD_AL_INNER)
        al_inner_poly = gdstk.boolean(al_inner_poly, cut, "not", **c.LD_AL_INNER)
        for p in al_inner_poly:
            p.fillet(args.fillet_radius, tolerance=self.config.tolerance)
        self.cell.add(*al_inner_poly)

        if args.with_gap:
            al_gap_poly = gdstk.cross(
                0j, args.outer_cross_size, args.outer_arm_width, **c.LD_AL_GAP
            )
            v_sq = args.outer_square_width * (1 + 1j)
            sq = gdstk.rectangle(-v_sq / 2, v_sq / 2).rotate(math.pi / 4)
            al_gap_poly = gdstk.boolean(al_gap_poly, sq, "or", **c.LD_AL_GAP)
            for p in al_gap_poly:
                p.fillet(args.fillet_radius, tolerance=self.config.tolerance)
            al_gap_poly = gdstk.boolean(
                al_gap_poly, al_inner_poly, "not", **c.LD_AL_GAP
            )
            self.cell.add(*al_gap_poly)

        self.create_port("squid", 0j, math.pi * 0.5)
        self.create_port(
            "rport", cmath.rect(args.inner_square_width / 2, -math.pi / 4), -math.pi / 4
        )

        port_cross_size = (
            args.outer_cross_size if args.with_gap else args.inner_cross_size
        )
        self.create_port("s", -0.5j * port_cross_size, math.pi * 1.5)
        self.create_port("n", 0.5j * port_cross_size, math.pi * 0.5)
        self.create_port("w", -0.5 * port_cross_size + 0j, math.pi)
        self.create_port("e", 0.5 * port_cross_size + 0j, 0)

    @property
    def port_squid(self):
        return self.ports["squid"]

    @property
    def port_rport(self):
        return self.ports["rport"]

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
class HCouplerArgs:
    inner_width: float = 100
    inner_length: float = 858
    outer_width: float = 180
    outer_length: float = 898

    bottleneck_width: float = 40
    bottleneck_length: float = 742

    fillet_radius: float = 10

    with_gap: bool = True
    with_async_pad: bool = False
    async_pad_length: float = 600
    async_pad_width: float = 60
    async_pad_gap: float = 70


class HCoupler(elements.Element):
    def __init__(
        self, args: HCouplerArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = HCouplerArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        c = self.config
        args = self.args
        v_inner = args.inner_length + 1j * args.inner_width
        al_inner_poly = gdstk.rectangle(-v_inner / 2, v_inner / 2)

        cut_top = gdstk.rectangle(
            -args.bottleneck_length / 2 + 1j * args.bottleneck_width / 2,
            args.bottleneck_length / 2 + 1j * args.inner_width / 2,
        )
        cut_bot = gdstk.rectangle(
            -args.bottleneck_length / 2 - 1j * args.inner_width / 2,
            args.bottleneck_length / 2 - 1j * args.bottleneck_width / 2,
        )
        al_inner_poly = gdstk.boolean(
            al_inner_poly, [cut_top, cut_bot], "not", **c.LD_AL_INNER
        )
        for p in al_inner_poly:
            p.fillet(args.fillet_radius, tolerance=c.tolerance)
        self.cell.add(*al_inner_poly)

        if args.with_gap:
            v_gap = args.outer_length + 1j * args.outer_width
            al_gap_poly = gdstk.rectangle(-v_gap / 2, v_gap / 2, **c.LD_AL_GAP)
            al_gap_poly.fillet(args.fillet_radius, tolerance=c.tolerance)
            al_gap_poly = gdstk.boolean(
                al_gap_poly, al_inner_poly, "not", **c.LD_AL_GAP
            )
            self.cell.add(*al_gap_poly)

        if args.with_async_pad:
            pad = gdstk.rectangle(
                0j, args.async_pad_length + args.async_pad_width * 1j, **c.LD_AL_INNER
            ).translate(
                -args.async_pad_length / 2,
                args.async_pad_gap + args.bottleneck_width / 2,
            )
            pad.fillet(args.fillet_radius, tolerance=c.tolerance)
            self.cell.add(pad)

        self.create_port(
            "squid", (args.outer_width + args.bottleneck_width) * 0.25j, math.pi / 2
        )
        port_length = args.outer_length if args.with_gap else args.inner_length
        self.create_port("w", -0.5 * port_length + 0j, math.pi)
        self.create_port("e", 0.5 * port_length + 0j, 0)

    @property
    def port_squid(self):
        return self.ports["squid"]

    @property
    def port_w(self):
        return self.ports["w"]

    @property
    def port_e(self):
        return self.ports["e"]


@dataclass
class YCouplerArgs:
    couple_width: float = 40
    connect_width: float = 36
    couple_length: float = 150
    connect_length: float = 1200
    async_pad_length: float = 960
    async_pad_width: float = 60
    async_pad_gap: float = 60
    fillet_radius: float = 10


class YCoupler(elements.Element):
    def __init__(
        self, args: YCouplerArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = YCouplerArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        c = self.config
        args = self.args
        p_y1 = args.couple_length * (1 + 1j)
        path_y1 = gdstk.FlexPath(
            [p_y1 + args.connect_width / 2, p_y1 - args.couple_length],
            args.couple_width,
            **c.LD_AL_INNER
        )
        path_y2 = gdstk.FlexPath(
            [p_y1 + args.connect_width / 2 * 1j, p_y1 - args.couple_length * 1j],
            args.couple_width,
            **c.LD_AL_INNER
        )
        p_y2 = p_y1 + cmath.rect(args.connect_length, math.pi / 4)
        path_y3 = gdstk.FlexPath(
            [p_y2 - args.connect_width / 2 * 1j, p_y2 + args.couple_length * 1j],
            args.couple_width,
            **c.LD_AL_INNER
        )
        path_y4 = gdstk.FlexPath(
            [p_y2 - args.connect_width / 2, p_y2 + args.couple_length],
            args.couple_width,
            **c.LD_AL_INNER
        )
        path_connect = gdstk.FlexPath([p_y1, p_y2], args.connect_width, **c.LD_AL_INNER)

        p_pad_center = (p_y1 + p_y2) / 2 + cmath.rect(
            args.async_pad_gap + (args.connect_width + args.async_pad_width) / 2,
            -math.pi / 4,
        )
        v_pad = cmath.rect(args.async_pad_length, math.pi / 4)
        path_pad = gdstk.FlexPath(
            [p_pad_center - v_pad / 2, p_pad_center + v_pad / 2],
            args.async_pad_width,
            **c.LD_AL_INNER
        )

        poly_inner = gdstk.boolean(
            [path_y1, path_y2, path_y3, path_y4, path_connect, path_pad],
            [],
            "or",
            **c.LD_AL_INNER
        )
        for p in poly_inner:
            p.fillet(args.fillet_radius, tolerance=c.tolerance)
        self.cell.add(*poly_inner)

        p_squid = (p_y1 + p_y2) / 2 + cmath.rect(
            (args.connect_width + args.async_pad_gap) / 2, -math.pi / 4
        )
        self.create_port("squid", p_squid, -math.pi / 4)
        self.create_port("center", (p_y1 + p_y2) / 2, -3 / 4 * math.pi)
        self.create_port(
            "sw", p_y1 - args.couple_width / 2 * (1 + 1j), -3 / 4 * math.pi
        )
        self.create_port("ne", p_y2 + args.couple_width / 2 * (1 + 1j), 1 / 4 * math.pi)

    @property
    def port_center(self):
        return self.ports["center"]

    @property
    def port_squid(self):
        return self.ports["squid"]

    @property
    def port_sw(self):
        return self.ports["sw"]

    @property
    def port_ne(self):
        return self.ports["ne"]


if __name__ == "__main__":
    config.use_preset_design()
    # chip = ldesign.chips.Chip24Ports()
    # elem = FloatingXmon()
    # chip.add_element(elem, elements.DockingPort(5000+5000j), transformation=elements.Transformation(rotation=math.pi/4))
    # chip.view()

    # elem = FloatingXmon(FloatingXmonArgs(inner_square_width=400, with_gap=False))
    elem = HCoupler(HCouplerArgs(with_gap=False, with_async_pad=True))
    # elem.write_gds("demo.gds", test_region=(-1000-1000j, 1000+1000j))
    elem.view()
