#!/usr/bin/env python
# coding=utf-8
"""Showcasing image utitlities of vcp"""

import os
import sys
import cv2

import numpy as np

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import imutils
from vcp import imvis


if __name__ == "__main__":
    rgb = imutils.imread('../data/flamingo.jpg', mode='RGB', flip_channels=False) # Load as BGR
    # Discretization
    vis = imutils.transform(rgb, 'rgb2cn')
    imvis.imshow(vis, 'discretization')

    # Cartoonification
    rgb = imutils.cartoonify(rgb, num_pyramid_levels=3,
        num_bilateral_filters=5, diameter_pixel_neighborhood=7,
        sigma_color=9.0, sigma_space=7.0, kernel_size_median=7,
        edge_block_size=9, is_rgb=False)


    # Fuzzy resizing
    for scale in [0.72, 0.729, 0.99]:
        res, actual_scale = imutils.fuzzy_resize(rgb, scale, output_scaling_factor=True)
        imvis.imshow(res, 'Scaling desired {:.3f}, actual {:.3f}'.format(scale, actual_scale), wait_ms=10, flip_channels=False)


    # Rotate image around center
    for angle in range(0, 361, 10):
        vis_img_cropped = imutils.rotate_image(rgb, np.deg2rad(angle), crop=True)
        vis_img_nocrop = imutils.rotate_image(rgb, np.deg2rad(angle), crop=False)
        vis_img = imvis.make_collage([vis_img_nocrop, vis_img_cropped], padding=15, bg_color=(255, 0, 0))

        k = imvis.imshow(vis_img, wait_ms=50)
        if k == ord('q') or k == 27:
            break


    # Rotate image around arbitrary point
    for angle in range(0,361,10):
        vis_img = imutils.rotate_image_anchor(rgb, (rgb.shape[1]//2, rgb.shape[0]//10*2), np.deg2rad(angle), cv2.BORDER_REPLICATE)
        k = imvis.imshow(vis_img, wait_ms=50)
        if k == ord('q') or k == 27:
            break

    cv2.waitKey(-1)


