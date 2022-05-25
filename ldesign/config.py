from dataclasses import dataclass


@dataclass
class Config:
    LD_AL_INNER = {"layer": 1, "datatype": 1}
    LD_AL_OUTER = {"layer": 1, "datatype": 0}
    LD_BANDAGE = {"layer": 7, "datatype": 0}
    LD_JJ_PAD = {"layer": 6, "datatype": 0}
    LD_JJ = {"layer": 5, "datatype": 0}
    LD_BRIDGE_UNDER = {"layer": 4, "datatype": 0}
    LD_BRIDGE = {"layer": 3, "datatype": 0}
    LD_BRIDGE_VIA = {"layer": 100, "datatype": 0}
    LD_LABEL = {"layer": 0, "texttype": 0}
    label_ports: bool = False
    port_width: float = 0.5
    port_length: float = 2
    tolerance: float = 0.01


global_config = Config()


def use_preset_design():
    global_config.label_ports = True
    global_config.tolerance = 0.5
