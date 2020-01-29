#!/usr/bin/env python
# coding=utf-8
"""Showcasing image visualizations"""

import os
import sys
import cv2

import numpy as np

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import colormaps
from vcp import imutils
from vcp import imvis
from vito import cam_projections as prjutils


if __name__ == "__main__":
    ############################################################################
    # Render an image sequence as "perspective images"
    rgb = imutils.imread('../data/flamingo.jpg', mode='RGB').copy()
    bgr = imutils.flip_layers(rgb)
    gray = imutils.rgb2gray(rgb, is_bgr=False)
    # rgb[:,:,0] = 255
    # rgb[:,:,2] = 0
    images = [rgb, gray, bgr, gray]

    imvis.imshow(rgb, 'input')
    vis = imvis.render_perspective(rgb, ry=4, rz=3, ty=1, angles_in_deg=True)#, bg_color=(180,0,0))
    imvis.imshow(vis, 'perspective', wait_ms=-1)

    # Add (transparent) border
    #TODO make transparent again
    # images = [imutils.pad(img, 5, color=None) for img in images]
    images = [imutils.pad(img, 5, color=(0, 0, 200)) for img in images]

    imvis.imshow(images[0], 'inputs', wait_ms=-1)
    vis = imvis.make_collage(images, bg_color=(180, 0, 0))
    imvis.imshow(vis, 'inputs', wait_ms=-1)


    vis = imvis.render_image_sequence(images)
    imvis.imshow(vis, 'image sequence', wait_ms=-1)
