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
    rgb = imutils.imread('../data/flamingo.jpg', 'RGB')
    bgr = imutils.flip_layers(rgb)
    gray = np.dstack((rgb[:, :, 0], rgb[:, :, 0], rgb[:, :, 0]))

    images = [rgb, gray, bgr, gray]
    # Add (transparent) border
    images = [imutils.pad(img, 2, color=None) for img in images]

    cv2.imshow('foo', images[0])
    cv2.waitKey(-1)
    vis = imvis.make_collage(images)
    imvis.imshow(vis, 'inputs', wait_ms=-1)

    vis = imvis.render_image_sequence(images)
    imvis.imshow(vis, 'image sequence', wait_ms=-1)
