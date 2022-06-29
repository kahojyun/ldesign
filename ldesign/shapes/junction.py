from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Literal

import gdstk
from ldesign import config, elements


@dataclass
class JunctionPadArgs:
    base_width: float = 4.5
    base_length: float = 8.5
    bandage_length: float = 9.5
    bandage_top_margin: float = 1
    bandage_side_margin: float = 1
    jpad_length: float = 6.5
    jpad_top_margin: float = 1
    jpad_side_margin: float = 1


@dataclass
class JunctionArmArgs:
    base_length: float = 12
    base_width: float = 0.5
    cross_length_inner: float = 2.25
    cross_length_outer: float = 2.25
    cross_width: float = 0.2


@dataclass
class SingleJunctionArgs:
    q_pad: JunctionPadArgs = field(default_factory=JunctionPadArgs)
    g_pad: JunctionPadArgs = field(default_factory=JunctionPadArgs)
    q_arm: JunctionArmArgs = field(default_factory=JunctionArmArgs)
    g_arm: JunctionArmArgs = field(default_factory=JunctionArmArgs)
    q_arm_direction: Literal["top", "left", "right"] = "right"
    g_arm_direction: Literal["top", "left", "right"] = "top"


def _get_default_squid_pad_args():
    return JunctionPadArgs(base_width=6, jpad_side_margin=2)


def _get_default_squid_arm_args():
    return JunctionArmArgs(base_length=2, cross_length_inner=2, cross_length_outer=1.5)


@dataclass
class SquidArgs:
    q_pad: JunctionPadArgs = field(default_factory=_get_default_squid_pad_args)
    g_pad: JunctionPadArgs = field(default_factory=_get_default_squid_pad_args)
    q_arm: JunctionArmArgs = field(default_factory=_get_default_squid_arm_args)
    g_arm: JunctionArmArgs = field(default_factory=_get_default_squid_arm_args)
    j_width1: float = 0.2
    j_width2: float = 0.2


def _get_default_square_squid_pad_args():
    return JunctionPadArgs(
        base_width=3,
        bandage_length=10,
        bandage_top_margin=-4,
        bandage_side_margin=0.5,
        jpad_length=8,
        jpad_top_margin=5,
        jpad_side_margin=0.5,
    )


def _get_default_square_squid_arm_args():
    return JunctionArmArgs(
        base_length=10, cross_length_inner=4.75, cross_length_outer=2.25
    )


@dataclass
class SquareSquidArgs:
    pad: JunctionPadArgs = field(default_factory=_get_default_square_squid_pad_args)
    arm: JunctionArmArgs = field(default_factory=_get_default_square_squid_arm_args)
    j_width1: float = 0.2
    j_width2: float = 0.2


class JunctionPad(elements.Element):
    def __init__(
        self, args: JunctionPadArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = JunctionPadArgs()
        self.args = args
        self._init_cell()

    def _init_cell(
        self,
    ):
        args = self.args
        base = gdstk.rectangle(
            args.base_width / 2 + 0j,
            -args.base_width / 2 + args.base_length * 1j,
            **self.config.LD_AL_INNER,
        )

        bandage_width = args.base_width - 2 * args.bandage_side_margin
        bandage_top_right = (
            bandage_width / 2 + (args.base_length - args.bandage_top_margin) * 1j
        )
        bandage_bottom_left = (
            bandage_top_right - bandage_width - args.bandage_length * 1j
        )
        bandage = gdstk.rectangle(
            bandage_bottom_left, bandage_top_right, **self.config.LD_BANDAGE
        )

        jpad_width = args.base_width + 2 * args.jpad_side_margin
        jpad_top_y = args.base_length + args.jpad_top_margin
        jpad_top_right = jpad_width / 2 + jpad_top_y * 1j
        jpad_bottom_left = jpad_top_right - jpad_width - args.jpad_length * 1j
        jpad = gdstk.rectangle(
            jpad_top_right, jpad_bottom_left, **self.config.LD_JJ_PAD
        )
        self.cell.add(base, bandage, jpad)
        self.create_port("base", 0j, -math.pi / 2)
        self.create_port("jpad_top", jpad_top_y * 1j, math.pi / 2)
        jpad_middle_y = jpad_top_y - args.jpad_length / 2
        self.create_port("jpad_left", -jpad_width / 2 + jpad_middle_y * 1j, math.pi)
        self.create_port("jpad_right", jpad_width / 2 + jpad_middle_y * 1j, 0)

    @property
    def port_base(self):
        return self.ports["base"]

    @property
    def port_jpad_top(self):
        return self.ports["jpad_top"]

    @property
    def port_jpad_left(self):
        return self.ports["jpad_left"]

    @property
    def port_jpad_right(self):
        return self.ports["jpad_right"]


class JunctionArm(elements.Element):
    def __init__(
        self, args: JunctionArmArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = JunctionArmArgs()
        self.args = args
        self._init_cell()

    def _init_cell(
        self,
    ):
        args = self.args
        base = gdstk.rectangle(
            args.base_width / 2 + 0j,
            -args.base_width / 2 + args.base_length * 1j,
            **self.config.LD_JJ,
        )

        # If cross_width is odd in nm, shift 0.5nm
        shift = 0
        min_grid = self.config.precision / self.config.unit
        if round(args.cross_width / min_grid) % 2 == 1:
            shift = min_grid / 2
        cross = gdstk.rectangle(
            args.cross_width / 2 + shift + 0j,
            -args.cross_width / 2
            + shift
            + (args.cross_length_inner + args.cross_length_outer) * 1j,
            **self.config.LD_JJ,
        )
        cross.translate(args.base_length * 1j)

        self.cell.add(base, cross)
        self.create_port("base", 0j, -math.pi / 2)
        self.create_port(
            "cross", (args.base_length + args.cross_length_inner) * 1j, math.pi / 2
        )

    @property
    def port_base(self):
        return self.ports["base"]

    @property
    def port_cross(self):
        return self.ports["cross"]


class SingleJunction(elements.Element):
    def __init__(
        self,
        args: SingleJunctionArgs | None = None,
        config: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = SingleJunctionArgs()
        self.args = args
        self._init_cell()

    def _init_cell(
        self,
    ):
        args = self.args
        q_pad = JunctionPad(args.q_pad, self.config)
        q_pad = self.add_element(
            q_pad, elements.DockingPort(0j, math.pi / 2, self), q_pad.port_base
        )
        q_arm = JunctionArm(args.q_arm, self.config)
        match args.q_arm_direction:
            case "top":
                q_arm = self.add_element(q_arm, q_pad.port_jpad_top, q_arm.port_base)
            case "left":
                q_arm = self.add_element(q_arm, q_pad.port_jpad_left, q_arm.port_base)
            case "right":
                q_arm = self.add_element(q_arm, q_pad.port_jpad_right, q_arm.port_base)
            case _:
                raise ValueError(args.q_arm_direction)
        g_arm = JunctionArm(args.g_arm, self.config)
        match args.q_arm_direction, args.g_arm_direction:
            case ("top", "left") | ("right", "top"):
                rot = math.pi / 2
            case ("top", "right") | ("left", "top"):
                rot = -math.pi / 2
            case _:
                raise ValueError
        g_arm = self.add_element(
            g_arm,
            q_arm.port_cross,
            g_arm.port_cross,
            elements.Transformation(rotation=rot),
        )
        g_pad = JunctionPad(args.g_pad, self.config)
        match args.g_arm_direction:
            case "top":
                g_pad = self.add_element(g_pad, g_arm.port_base, g_pad.port_jpad_top)
            case "left":
                g_pad = self.add_element(g_pad, g_arm.port_base, g_pad.port_jpad_left)
            case "right":
                g_pad = self.add_element(g_pad, g_arm.port_base, g_pad.port_jpad_right)
            case _:
                raise ValueError(args.g_arm_direction)

        self.create_port("q_base", 0j, -math.pi / 2)
        g_base_port = g_pad.port_base.as_reference(0j, root_elem=self)
        self.create_port("g_base", g_base_port.point, g_base_port.angle)

    @property
    def port_q_base(self):
        return self.ports["q_base"]

    @property
    def port_g_base(self):
        return self.ports["g_base"]


class Squid(elements.Element):
    def __init__(
        self,
        args: SquidArgs | None = None,
        config: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = SquidArgs()
        self.args = args
        self._init_cell()

    def _init_cell(
        self,
    ):
        args = self.args
        q_pad = JunctionPad(args.q_pad, self.config)
        q_pad = self.add_element(
            q_pad, elements.DockingPort(0j, math.pi / 2, self), q_pad.port_base
        )
        q_arm1_args = dataclasses.replace(args.q_arm, cross_width=args.j_width1)
        q_arm2_args = dataclasses.replace(args.q_arm, cross_width=args.j_width2)
        g_arm1_args = dataclasses.replace(args.g_arm, cross_width=args.j_width1)
        g_arm2_args = dataclasses.replace(args.g_arm, cross_width=args.j_width2)
        q_arm1 = JunctionArm(q_arm1_args, self.config)
        q_arm1 = self.add_element(q_arm1, q_pad.port_jpad_left, q_arm1.port_base)
        q_arm2 = JunctionArm(q_arm2_args, self.config)
        q_arm2 = self.add_element(q_arm2, q_pad.port_jpad_right, q_arm2.port_base)

        g_arm1 = JunctionArm(g_arm1_args, self.config)
        g_arm1 = self.add_element(
            g_arm1,
            q_arm1.port_cross,
            g_arm1.port_cross,
            elements.Transformation(rotation=-math.pi / 2),
        )
        g_arm2 = JunctionArm(g_arm2_args, self.config)
        g_arm2 = self.add_element(
            g_arm2,
            q_arm2.port_cross,
            g_arm2.port_cross,
            elements.Transformation(rotation=math.pi / 2),
        )

        g_pad = JunctionPad(args.g_pad, self.config)
        g_pad1 = self.add_element(g_pad, g_arm1.port_base, g_pad.port_jpad_top)
        g_pad2 = self.add_element(g_pad, g_arm2.port_base, g_pad.port_jpad_top)

        self.create_port("q_base", 0j, -math.pi / 2)
        g_base_port1 = g_pad1.port_base.as_reference(0j, root_elem=self)
        g_base_port2 = g_pad2.port_base.as_reference(0j, root_elem=self)
        self.create_port(
            "g_center",
            (g_base_port1.point + g_base_port2.point) / 2,
            g_base_port1.angle,
        )

    @property
    def port_q_base(self):
        return self.ports["q_base"]

    @property
    def port_g_center(self):
        return self.ports["g_center"]

    @property
    def total_height(self):
        return (self.port_g_center.point - self.port_q_base.point).imag


class SquareSquid(elements.Element):
    def __init__(
        self,
        args: SquareSquidArgs | None = None,
        config: config.Config | None = None,
    ):
        super().__init__(config=config)
        if args is None:
            args = SquareSquidArgs()
        self.args = args
        self._init_cell()

    def _init_cell(
        self,
    ):
        args = self.args
        pad = JunctionPad(args.pad, self.config)
        pad1 = self.add_element(
            pad, elements.DockingPort(0j, math.pi / 2, self), pad.port_base
        )
        pad_corner = pad1.port_jpad_top.as_reference(
            -(args.pad.base_width / 2 + args.pad.jpad_side_margin) * 1j
        )
        arm1_args = dataclasses.replace(args.arm, cross_width=args.j_width1)
        arm2_args = dataclasses.replace(args.arm, cross_width=args.j_width2)
        arm1 = JunctionArm(arm1_args, self.config)
        arm2 = JunctionArm(arm2_args, self.config)
        a1 = self.add_element(
            arm1,
            pad_corner,
            arm1.port_base,
            transformation=elements.Transformation(arm1_args.base_width * 1.5j),
        )
        a2 = self.add_element(
            arm2,
            pad_corner,
            arm2.port_base,
            transformation=elements.Transformation(
                -arm2_args.base_width * 1.5 + 0j, -math.pi / 2
            ),
        )
        pc1 = a1.port_cross.get_transformed_port(self).point
        pc2 = a2.port_cross.get_transformed_port(self).point
        self.create_port("center", (pc1 + pc2) / 2, math.pi / 2)
        a1 = self.add_element(
            arm1,
            a1.port_cross,
            arm1.port_cross,
            transformation=elements.Transformation(rotation=-math.pi / 2),
        )
        a2 = self.add_element(
            arm2,
            a2.port_cross,
            arm2.port_cross,
            transformation=elements.Transformation(rotation=math.pi / 2),
        )
        self.add_element(
            pad,
            a2.port_base,
            pad.port_jpad_top,
            transformation=elements.Transformation(
                (
                    arm1_args.base_width * 1.5
                    - (args.pad.base_width / 2 + args.pad.jpad_side_margin)
                )
                * 1j
            ),
        )

    @property
    def port_center(self):
        return self.ports["center"]


if __name__ == "__main__":
    config.use_preset_design()
    elem = SquareSquid(SquareSquidArgs(j_width1=0.2, j_width2=0.2))
    elem.view()
