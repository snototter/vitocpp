#!/usr/bin/env python
# coding=utf-8
"""Python 3 bindings for the (C++) VCP library 

See VCP_ROOT_DIR/examples/python3 for an overview.
"""

__all__ = ['colormaps', 'config', 'imutils', 'imvis', 'math2d', 'math3d', 'utils']
__author__ = 'snototter'

# Load version
import os
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'version.py')) as vf:
    exec(vf.read())
