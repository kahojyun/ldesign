from __future__ import annotations

import cmath
import copy
import operator
import os
import uuid
from collections import defaultdict
from dataclasses import dataclass
from functools import reduce
from typing import ClassVar, Optional, TypeVar

import gdspy
import gdstk
import numpy as np

from ldesign.config import Config, global_config
from ldesign.utils import to_complex

_Transformable = TypeVar("_Transformable", "Transformation", "DockingPort", complex)


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

    def transform(self, other: Transformation):
        new_t = self.translation
        new_m = self.magnification
        new_rot = self.rotation
        new_ref = self.x_reflection
        if other.x_reflection:
            new_t = new_t.conjugate()
            new_rot = -new_rot
        new_t = (
            new_t * other.magnification * cmath.exp(1j * other.rotation)
            + other.translation
        )
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

    def with_new_axis(
        self, origin: complex = 0j, axis_angle: float = 0
    ) -> Transformation:
        t = Transformation(origin, rotation=axis_angle)
        return t @ self @ t.inverse()

    def __matmul__(self, other: _Transformable) -> _Transformable:
        if isinstance(other, Transformation):
            return other.transform(self)
        if isinstance(other, DockingPort):
            return other.transform(self)
        if isinstance(other, complex):
            return (
                other * self.magnification * cmath.exp(1j * self.rotation)
                + self.translation
            )
        raise TypeError


@dataclass
class DockingPort:
    point: complex = 0j
    angle: float = 0
    element: Optional[Element] = None

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

    def get_transformed_port(self, root_elem: Element | None = None):
        if self.element is None:
            return self.copy()
        transformation = self.element.get_total_transformation(root_elem)
        result = transformation @ self
        result.element = root_elem
        return result

    def as_reference(
        self, point: complex, angle: float = 0, root_elem: Element | None = None
    ):
        transformed_port = self.get_transformed_port(root_elem)
        transformed_port.point += cmath.rect(1, transformed_port.angle) * point
        transformed_port.angle += angle
        return transformed_port


TE = TypeVar("TE", bound="Element")


class Element:
    # Transformation relative to parent
    transformation: Transformation
    cell: gdstk.Cell
    children: list[Element]
    labeled_children: dict[str, Element]
    parent: Optional[Element]
    config: Config
    ports: dict[str, DockingPort]
    _prefix_count: ClassVar[defaultdict] = defaultdict(int)

    def __init__(
        self,
        name: str | None = None,
        prefix: str | None = None,
        config: Config | None = None,
    ):
        if name is None:
            if prefix is None:
                prefix = type(self).__name__
            name = f"{prefix}_{Element._prefix_count[prefix]}"
            Element._prefix_count[prefix] += 1
        if config is None:
            config = global_config
        self.cell = gdstk.Cell(name)
        self.ports = {}
        self.children = []
        self.labeled_children = {}
        self.transformation = Transformation()
        self.parent = None
        self.config = config

    @property
    def name(self) -> str:
        return self.cell.name

    def __add__(self, other: Element):
        if isinstance(other, Element):
            new_elem = Element(prefix="CompositeElement")
            new_elem.add_element(self)
            new_elem.add_element(other)
            return new_elem
        else:
            raise TypeError

    def __iadd__(self, other: Element):
        if isinstance(other, Element):
            self.add_element(other)
        else:
            raise TypeError

    def __repr__(self):
        return f'{type(self).__name__}(name="{self.name}", transformation={self.transformation})'

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
        new.labeled_children = {}
        for k, c in self.labeled_children.items():
            new_c = c.copy()
            new_c.parent = new
            new.labeled_children[k] = new_c
        return new

    def create_port(self, name: str, point: complex, angle: float):
        self.ports[name] = DockingPort(point, angle, self)
        if self.config.label_ports:
            self.cell.add(gdstk.Label(name, point, **self.config.LD_LABEL))
            p_start1 = point + self.config.port_width / 2 * cmath.exp(
                1j * (angle + np.pi / 2)
            )
            p_start2 = point - self.config.port_width / 2 * cmath.exp(
                1j * (angle + np.pi / 2)
            )
            p_end = point + self.config.port_length * cmath.exp(1j * angle)
            self.cell.add(
                gdstk.Polygon(
                    [p_start1, p_start2, p_end], layer=self.config.LD_LABEL["layer"]
                )
            )

    def get_total_transformation(
        self, root_elem: Element | None = None
    ) -> Transformation:
        # from bottom to top
        transformations = []
        p = self
        while p is not None and p is not root_elem:
            transformations.append(p.transformation)
            p = p.parent
        if p is None and root_elem is not None:
            raise Exception("Can't find root element")
        return reduce(operator.matmul, reversed(transformations), Transformation())

    def add_element(
        self,
        element: TE,
        ref_port: DockingPort | None = None,
        elem_port: DockingPort | None = None,
        transformation: Transformation | None = None,
        label: str | None = None,
        match_angle: bool = True,
        copy: bool = True,
    ) -> TE:
        if not copy and element.parent is not None:
            raise ValueError(
                "Trying to add element without copying but element already has a parent."
            )
        if label is not None and label in self.labeled_children:
            raise ValueError("Duplicate label", label)
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
            transformations.append(
                Transformation(
                    rotation=transformed_ref_port.angle
                    + np.pi
                    - transformed_elem_port.angle
                ).with_new_axis(origin)
            )
        # 3. extra transformation
        if transformation is None:
            transformation = Transformation()
        transformation = transformation.with_new_axis(
            origin, transformed_ref_port.angle if match_angle else 0
        )
        transformations.append(transformation)

        total_transformation = reduce(
            operator.matmul, reversed(transformations), Transformation()
        )
        translation = total_transformation.translation
        rotation = total_transformation.rotation
        magnification = total_transformation.magnification
        x_reflection = total_transformation.x_reflection
        self.cell.add(
            gdstk.Reference(
                element.cell, translation, rotation, magnification, x_reflection
            )
        )
        if copy:
            transformed_element = element.copy()
        else:
            transformed_element = element
        transformed_element.transformation = total_transformation
        transformed_element.parent = self
        if label is not None:
            self.labeled_children[label] = transformed_element
        else:
            self.children.append(transformed_element)
        return transformed_element

    def flatten(self, chips: list[int] | None = None):
        cell = self.cell.flatten()
        ld_outer = self.config.LD_AL_OUTER
        ld_inner = self.config.LD_AL_INNER
        if chips is None:
            outer = cell.get_polygons(**ld_outer)
            inner = cell.get_polygons(**ld_inner)
            outer = gdstk.boolean(outer, inner, "not", **ld_outer)
            inner = gdstk.boolean(inner, [], "or", **ld_inner)
            cell.filter([ld_outer["layer"]], [ld_outer["datatype"]], "and")
            cell.filter([ld_inner["layer"]], [ld_inner["datatype"]], "and")
            cell.add(*outer)
            cell.add(*inner)
        else:
            for chip in chips:
                outer = cell.get_polygons(layer=ld_outer["layer"], datatype=chip)
                inner = cell.get_polygons(layer=ld_inner["layer"], datatype=chip)
                outer = gdstk.boolean(
                    outer, inner, "not", layer=ld_outer["layer"], datatype=chip
                )
                inner = gdstk.boolean(
                    inner, [], "or", layer=ld_inner["layer"], datatype=chip
                )
                cell.filter([ld_outer["layer"]], [chip], "and")
                cell.filter([ld_inner["layer"]], [chip], "and")
                cell.add(*outer)
                cell.add(*inner)

    def add_to_library(
        self,
        lib: gdstk.Library,
        flatten=True,
        test_region: tuple[complex, complex] | None = None,
        chips: list[int] | None = None,
    ):
        if test_region is not None:
            self.flatten(chips)
            if chips is None:
                temp_cell = gdstk.Cell(self.cell.name + "_temp")
                rect = gdstk.rectangle(*test_region)
                for poly in self.cell.get_polygons(include_paths=True):
                    temp_cell.add(
                        *gdstk.boolean(
                            rect, poly, "and", layer=poly.layer, datatype=poly.datatype
                        )
                    )
                ld_outer = self.config.LD_AL_OUTER
                ld_inner = self.config.LD_AL_INNER
                outer_poly = temp_cell.get_polygons(**ld_outer)
                base_al = gdstk.boolean(rect, outer_poly, "not", **ld_outer)
                temp_cell.filter([ld_outer["layer"]], [ld_outer["datatype"]], "and")
                temp_cell.filter([ld_inner["layer"]], [ld_inner["datatype"]], "and")
                temp_cell.add(*base_al)
                new_cell = gdstk.Cell(self.cell.name + "_test")
                new_cell.add(gdstk.Reference(temp_cell, -test_region[0]))
                new_cell.flatten()
                lib.add(new_cell)
            else:
                for chip in chips:
                    temp_cell = gdstk.Cell(self.cell.name + f"_temp_chip{chip}")
                    rect = gdstk.rectangle(*test_region)
                    for poly in self.cell.get_polygons(include_paths=True):
                        if poly.datatype != chip:
                            continue
                        temp_cell.add(
                            *gdstk.boolean(
                                rect,
                                poly,
                                "and",
                                layer=poly.layer,
                                datatype=poly.datatype,
                            )
                        )
                    ld_outer = self.config.LD_AL_OUTER
                    ld_inner = self.config.LD_AL_INNER
                    outer_poly = temp_cell.get_polygons(
                        layer=ld_outer["layer"], datatype=chip
                    )
                    base_al = gdstk.boolean(
                        rect, outer_poly, "not", layer=ld_outer["layer"], datatype=chip
                    )
                    temp_cell.filter([ld_outer["layer"]], [chip], "and")
                    temp_cell.filter([ld_inner["layer"]], [chip], "and")
                    temp_cell.add(*base_al)
                    new_cell = gdstk.Cell(self.cell.name + f"_test_chip{chip}")
                    new_cell.add(gdstk.Reference(temp_cell, -test_region[0]))
                    new_cell.flatten()
                    lib.add(new_cell)
            return
        if flatten:
            self.flatten(chips)
            lib.add(self.cell)
            if chips is not None:
                for chip in chips:
                    new_cell = gdstk.Cell(self.cell.name + f"_chip{chip}")
                    for poly in self.cell.get_polygons(include_paths=True):
                        if poly.datatype != chip:
                            continue
                        new_cell.add(poly)
                    lib.add(new_cell)
        else:
            lib.add(self.cell, *self.cell.dependencies(True))

    def view(self):
        filename = f"{uuid.uuid4()}.gds"
        self.write_gds(filename, flatten=False)

        lib = gdspy.GdsLibrary(infile=filename)
        gdspy.LayoutViewer(library=lib)
        os.remove(filename)

    def write_gds(
        self,
        filename: str,
        flatten=True,
        test_region: tuple[complex, complex] | None = None,
        libname="library",
        max_points=4000,
        chips: list[int] | None = None,
    ):
        lib = gdstk.Library(libname)
        self.add_to_library(lib, flatten, test_region, chips)
        lib.write_gds(filename, max_points=max_points)


class CpwWaveguide(Element):
    def __init__(self, config: Config | None = None):
        super().__init__(config=config)

    @property
    def port_start(self):
        return self.ports["start"]

    @property
    def port_end(self):
        return self.ports["end"]
