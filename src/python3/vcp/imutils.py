#!/usr/bin/env python
# coding=utf-8
"""Utilities you'll often need when working with images."""

from vcp.imutils_cpp import *

import vito
from vito.imutils import *

def fuzzy_resize(image, scaling_factor, output_scaling_factor=False):
    """Rounds scaling_factor to the closest 1/10th before scaling."""
    if scaling_factor == 1.0:
        img, scale = image, 1.0
    else:
        img, scale = fuzzy_resize__(image, scaling_factor)
    if output_scaling_factor:
        return (img, scale)
    else:
        return img
