#!/usr/bin/env python
# coding=utf-8
"""Utilities you'll often need when working with images."""

# import numpy as np
# import cv2
# from PIL import Image
from .imutils_cpp import *

import vito
from vito.imutils import *


# def imread(filename, mode='RGB', flip_channels=False):
#     return vimutils.imread(filename, mode=mode, flip_channels=flip_channels)
#     # """Load an image (using PIL) into a NumPy array.

#     # Optionally specify PIL's loading 'mode', i.e. RGB for color, RGBA for a
#     # transparent image and L for grayscale. You can also flip the channels,
#     # i.e. convert RGB to BGR if you need it."""
#     # if filename is None:
#     #     return None
#     # image = np.asarray(Image.open(filename).convert(mode))
#     # if flip_channels:
#     #     return flip_layers(image)
#     # else:
#     #     return image


# def imsave(filename, image, flip_channels=False):
#     """Store an image, taking care of flipping for you."""
#     if flip_channels:
#         cv2.imwrite(filename, flip_layers(image))
# # crappy JPEG presets, wtf!        Image.fromarray(flip_layers(image)).save(filename)
#     else:
#         cv2.imwrite(filename, image)


# def ndarray2memory_file(np_data, format='png'):
#     return vimutils.ndarray2memory_file(np_data, format=format)


# def memory_file2ndarray(memfile):
#     return vimutils.memory_file2ndarray(memfile)


# def flip_layers(nparray):
#     return vimutils.flip_layers(nparray)


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



# def clip_rect_to_image(rect, img_width, img_height):
#     return vimutils.clip_rect_to_image(rect, img_width, img_height)


# def is_valid_bbox(rect):
#     return vimutils.is_valid_bbox(rect)


# def apply_on_bboxes(image_np, bboxes, func, **func_kwargs):
#     return vimutils.apply_on_bboxes(image_np, bboxes, func, **func_kwargs)


# def roi(image_np, rect):
#     return vimutils.roi(image_np, rect)
