from importlib.metadata import PackageNotFoundError, version

from . import calculation, config, elements, path, planning, shapes, utils
from .config import Config
from .elements import DockingPort, Element, Transformation

# from setuptools-scm docs
try:
    __version__ = version("ldesign")
except PackageNotFoundError:
    # package is not installed
    pass
