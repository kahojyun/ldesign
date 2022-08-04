from dataclasses import dataclass


@dataclass
class Config:
    """Configurations for element.

    Currently, `layer` is used to distinguish shapes for different purpose, while
    `datatype` denotes which chip the shape belongs to.

    All elements should draw GDSII shapes according to the `Config` passed into their
    constructors.

    Arguments:
        label_ports (bool): Whether draw polygons and add text for ports.  Useful
            for shape designing and debugging.
        port_width (float): Width of the port label.
        port_length (float): Length of the port label.
        tolerance (float): Tolerance for curve approximation.
        unit (float): Unit of the GDSII file.
        precision (float): Precision of the GDSII file.
    """

    LD_AL_INNER = {"layer": 101, "datatype": 0}
    LD_AL_OUTER = {"layer": 102, "datatype": 0}
    LD_AL_GAP = {"layer": 1, "datatype": 0}
    LD_AL_TRAP = {"layer": 2, "datatype": 0}
    LD_BANDAGE = {"layer": 7, "datatype": 0}
    LD_JJ_PAD = {"layer": 6, "datatype": 0}
    LD_JJ = {"layer": 5, "datatype": 0}
    LD_BRIDGE_UNDER = {"layer": 4, "datatype": 0}
    LD_BRIDGE = {"layer": 3, "datatype": 0}
    LD_SUPPORT = {"layer": 99, "datatype": 0}
    LD_BRIDGE_VIA = {"layer": 100, "datatype": 0}
    LD_LABEL = {"layer": 0, "texttype": 0}
    label_ports: bool = False
    port_width: float = 0.5
    port_length: float = 2
    tolerance: float = 0.01
    unit: float = 1e-6
    precision: float = 1e-9


global_config = Config()


def use_preset_design():
    """Use a preset configuration for design purpose."""
    global_config.label_ports = True
    global_config.tolerance = 0.1
