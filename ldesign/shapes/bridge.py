import math
from dataclasses import dataclass

import gdstk
from ldesign import config, elements


@dataclass
class CpwBridgeArgs:
    width: float = 40
    length: float = 100


class Bridge(elements.Element):
    def __init__(
        self, args: CpwBridgeArgs | None = None, config: config.Config | None = None
    ):
        super().__init__(config=config)
        if args is None:
            args = CpwBridgeArgs()
        self.args = args
        self._init_cell()

    def _init_cell(self):
        width = self.args.width
        length = self.args.length
        rect = gdstk.rectangle(0 - width / 2 * 1j, length + width / 2 * 1j)
        self.cell.add(rect)
        self.create_port("start", 0j, math.pi)
        self.create_port("end", length + 0j, 0)

    @property
    def port_start(self):
        return self.ports["start"]

    @property
    def port_end(self):
        return self.ports["end"]
