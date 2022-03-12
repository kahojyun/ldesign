import cmath
import copy
import operator
import os
import uuid
from collections import defaultdict
from dataclasses import dataclass
from functools import reduce
from typing import Literal, ClassVar, TypeVar, Optional

import gdspy
import gdstk
import numpy as np

from ldesign.config import Config, global_config
from ldesign.planning import plan_fixed_len
from ldesign.utils import to_complex

LD_AL_INNER = {'layer': 1, 'datatype': 1}
LD_AL_OUTER = {'layer': 1, 'datatype': 0}
LD_BANDAGE = {'layer': 7, 'datatype': 0}
LD_JJ_PAD = {'layer': 6, 'datatype': 0}
LD_JJ = {'layer': 5, 'datatype': 0}
LD_BRIDGE_UNDER = {'layer': 4, 'datatype': 0}
LD_BRIDGE = {'layer': 3, 'datatype': 0}
LD_LABEL = {'layer': 127, 'texttype': 0}


@dataclass
class Transformation:
    """Represent transformation of elements.

    Transformation order:
        1. x_reflection
        2. rotation, magnification
        3. translation
    """
    translation: complex = 0j
    rotation: float = 0
    magnification: float = 1
    x_reflection: bool = False

    def __post_init__(self):
        self.translation = to_complex(self.translation)

    def copy(self):
        return copy.copy(self)

    def transform(self, other: 'Transformation'):
        new_t = self.translation
        new_m = self.magnification
        new_rot = self.rotation
        new_ref = self.x_reflection
        if other.x_reflection:
            new_t = new_t.conjugate()
            new_rot = -new_rot
        new_t = new_t * other.magnification * cmath.exp(1j * other.rotation) + other.translation
        new_m *= other.magnification
        new_rot += other.rotation
        new_ref = new_ref != other.x_reflection
        return Transformation(new_t, new_rot, new_m, new_ref)

    def inverse(self):
        new_t = self.translation
        new_m = 1 / self.magnification
        new_rot = -self.rotation
        new_ref = self.x_reflection
        if new_ref:
            new_t = new_t.conjugate()
            new_rot = -new_rot
        new_t = -new_t * new_m * cmath.exp(1j * new_rot)
        return Transformation(new_t, new_rot, new_m, new_ref)

    def with_new_axis(self, origin=0j, axis_angle=0) -> 'Transformation':
        t = Transformation(origin, rotation=axis_angle)
        return t @ self @ t.inverse()

    def __matmul__(self, other):
        if isinstance(other, Transformation):
            return other.transform(self)
        if isinstance(other, DockingPort):
            return other.transform(self)
        if isinstance(other, complex):
            return other * self.magnification * cmath.exp(1j * self.rotation) + self.translation
        raise TypeError


@dataclass
class DockingPort:
    point: complex = 0j
    angle: float = 0
    element: Optional['Element'] = None

    def __post_init__(self):
        self.point = to_complex(self.point)

    def copy(self):
        return copy.copy(self)

    def transform(self, transformation: Transformation):
        new_p = self.point
        new_a = self.angle
        if transformation.x_reflection:
            new_p = new_p.conjugate()
            new_a = -new_a
        translation = transformation.translation
        rotation = transformation.rotation
        magnification = transformation.magnification
        new_p = new_p * magnification * cmath.exp(1j * rotation) + translation
        new_a = (new_a + rotation) % (2 * np.pi)
        return DockingPort(new_p, new_a)

    def get_transformed_port(self, root_elem: 'Element' = None):
        if self.element is None:
            return self.copy()
        transformation = self.element.get_total_transformation(root_elem)
        result = transformation @ self
        result.element = root_elem
        return result


TE = TypeVar('TE', bound='Element')


class Element:
    # Transformation relative to parent
    transformation: Transformation
    cell: gdstk.Cell
    children: list['Element']
    parent: Optional['Element']
    config: Config
    ports: dict[str, DockingPort]
    _prefix_count: ClassVar[defaultdict] = defaultdict(int)

    def __init__(self, name: str = None, prefix: str = None, config: Config = None):
        if name is None:
            if prefix is None:
                prefix = type(self).__name__
            name = f'{prefix}_{Element._prefix_count[prefix]}'
            Element._prefix_count[prefix] += 1
        if config is None:
            config = global_config
        self.cell = gdstk.Cell(name)
        self.ports = {}
        self.children = []
        self.transformation = Transformation()
        self.parent = None
        self.config = config

    @property
    def name(self) -> str:
        return self.cell.name

    def __add__(self, other: 'Element'):
        if isinstance(other, Element):
            new_elem = Element(prefix="CompositeElement")
            new_elem.add_element(self)
            new_elem.add_element(other)
            return new_elem
        else:
            raise TypeError

    def __iadd__(self, other: 'Element'):
        if isinstance(other, Element):
            self.add_element(other)
        else:
            raise TypeError

    def __repr__(self):
        return f"{type(self).__name__}(name={self.name}, transformation={self.transformation})"

    def copy(self):
        new = copy.copy(self)
        new.ports = {}
        for pn, p in self.ports.items():
            new_p = p.copy()
            new_p.element = new
            new.ports[pn] = new_p
        new.children = []
        for c in self.children:
            new_c = c.copy()
            new_c.parent = new
            new.children.append(new_c)
        return new

    def create_port(self, name: str, point: complex, angle: float):
        self.ports[name] = DockingPort(point, angle, self)
        if self.config.label_ports:
            self.cell.add(gdstk.Label(name, point, **self.config.LD_LABEL))
            p_start1 = point + self.config.port_width / 2 * cmath.exp(1j * (angle + np.pi / 2))
            p_start2 = point - self.config.port_width / 2 * cmath.exp(1j * (angle + np.pi / 2))
            p_end = point + self.config.port_length * cmath.exp(1j * angle)
            self.cell.add(gdstk.Polygon([p_start1, p_start2, p_end], layer=self.config.LD_LABEL["layer"]))

    def get_total_transformation(self, root_elem: 'Element' = None):
        # from bottom to top
        transformations = []
        p = self
        while p is not None and p is not root_elem:
            transformations.append(p.transformation)
            p = p.parent
        if p is None and root_elem is not None:
            raise Exception("Can't find root element")
        return reduce(operator.matmul, reversed(transformations), Transformation())

    def add_element(self, element: TE, ref_port: DockingPort = None, elem_port: DockingPort = None,
                    transformation: Transformation = None, match_angle: bool = True) -> TE:
        transformations = []
        # 1. align point
        if ref_port is None:
            transformed_ref_port = DockingPort()
        else:
            transformed_ref_port = ref_port.get_transformed_port(self)
        if elem_port is None:
            transformed_elem_port = DockingPort(angle=np.pi)
        else:
            transformed_elem_port = elem_port.get_transformed_port(element)
        origin = transformed_ref_port.point
        align_point = transformed_elem_port.point
        transformations.append(Transformation(origin - align_point))
        # 2. match port angle
        if match_angle:
            transformations.append(Transformation(
                rotation=transformed_ref_port.angle + np.pi - transformed_elem_port.angle).with_new_axis(origin))
        # 3. extra transformation
        if transformation is None:
            transformation = Transformation()
        transformation = transformation.with_new_axis(origin, transformed_ref_port.angle if match_angle else 0)
        transformations.append(transformation)

        total_transformation = reduce(operator.matmul, reversed(transformations), Transformation())
        translation = total_transformation.translation
        rotation = total_transformation.rotation
        magnification = total_transformation.magnification
        x_reflection = total_transformation.x_reflection
        self.cell.add(
            gdstk.Reference(element.cell, translation, rotation, magnification, x_reflection))
        transformed_element = element.copy()
        transformed_element.transformation = total_transformation
        transformed_element.parent = self
        self.children.append(transformed_element)
        return transformed_element

    def flatten(self):
        cell = self.cell.flatten()
        outer = cell.get_polygons(**LD_AL_OUTER)
        inner = cell.get_polygons(**LD_AL_INNER)
        outer = gdstk.boolean(outer, inner, 'not', **LD_AL_OUTER)
        inner = gdstk.boolean(inner, [], 'or', **LD_AL_INNER)
        cell.filter([LD_AL_OUTER['layer']], [LD_AL_OUTER['datatype']], 'and')
        cell.filter([LD_AL_INNER['layer']], [LD_AL_INNER['datatype']], 'and')
        cell.add(*outer)
        cell.add(*inner)

    def add_to_library(self, lib: gdstk.Library, flatten=True, test_region=None):
        if test_region is not None:
            self.flatten()
            cell = gdstk.Cell(self.cell.name + '_test')
            poly = self.cell.get_polygons(**LD_AL_OUTER)
            cell.add(*gdstk.boolean(gdstk.rectangle(*test_region), poly, 'not', **LD_AL_OUTER))
            lib.add(cell)
        if flatten:
            self.flatten()
            lib.add(self.cell)
        else:
            lib.add(self.cell, *self.cell.dependencies(True))

    def view(self):
        filename = f"{uuid.uuid4()}.gds"
        self.write_gds(filename, flatten=False)

        lib = gdspy.GdsLibrary(infile=filename)
        gdspy.LayoutViewer(library=lib)
        os.remove(filename)

    def write_gds(self, filename: str, flatten=True, test_region=None, libname="library", max_points=4000):
        lib = gdstk.Library(libname)
        self.add_to_library(lib, flatten, test_region)
        lib.write_gds(filename, max_points=max_points)


class CpwWaveguide(Element):
    def __init__(self):
        super().__init__()

    @property
    def port_start(self):
        return self.ports["start"]

    @property
    def port_end(self):
        return self.ports["end"]


class Cpw(Element):
    def __init__(self, points, gap=2, width=4, radius=50, end_gap=False):
        super().__init__()
        if end_gap:
            outer = gdstk.FlexPath(points, gap * 2 + width, bend_radius=radius, ends=(0, gap), **LD_AL_OUTER)
        else:
            outer = gdstk.FlexPath(points, gap * 2 + width, bend_radius=radius, **LD_AL_OUTER)
        inner = gdstk.FlexPath(points, width, bend_radius=radius, **LD_AL_INNER)
        outer = gdstk.boolean(outer, inner, 'not', **LD_AL_OUTER)
        self.cell.add(inner, *outer)
        self.point_begin = to_complex(points[0])
        self.point_end = to_complex(points[-1])


class MeanderAuto(Element):
    def __init__(self, gap, width, radius, total_len, in_direction: Literal['e', 'n', 'w'],
                 out_side: Literal['e', 'n', 'w', 's'], out_position, height, left_width, right_width):
        super().__init__()
        outer_width = width + 2 * gap
        points = plan_fixed_len(radius, total_len - outer_width, in_direction, out_side, out_position,
                                height - outer_width, left_width - outer_width / 2, right_width - outer_width / 2)
        points[0] = (0, -outer_width / 2)
        if out_side == 'e':
            last_point_offset = outer_width / 2
        elif out_side == 'n':
            last_point_offset = outer_width / 2 * 1j
        elif out_side == 'w':
            last_point_offset = -outer_width / 2
        else:
            last_point_offset = -outer_width / 2 * 1j
        points[-1] = to_complex(points[-1]) + last_point_offset
        cpw_elem = Cpw(points, gap, width, radius)
        self.add_element(cpw_elem)
        self.point_begin = cpw_elem.point_begin
        self.point_end = cpw_elem.point_end
