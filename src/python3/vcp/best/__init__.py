#!/usr/bin/env python
# coding=utf-8
"""Streaming from multiple (supported) cameras."""

try:
    from .best_cpp import *
except ImportError as e:
    print("[W] Could not load vcp::best C++ wrapper")
    print(str(e))

# from . import config
# from . import configurator
from .storage import *
from .streaming import *
# from . import utils

# Should be imported explicitly due to PyQt5 dependency, which is usually overkill
#from . import config_gui
