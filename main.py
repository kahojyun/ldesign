from ldesign import config
from ldesign.shapes import boundary
from ldesign.shapes.boundary import Boundary

if __name__ == '__main__':
    config.use_preset_design()

    top_elem = Boundary(boundary.boundary_24port)
    top_elem.view()
