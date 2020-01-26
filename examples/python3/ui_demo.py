#!/usr/bin/env python
# coding=utf-8
"""Showcasing the simple UI tools of vcp - these are 
very basic user input tools. Consider using my 'iminspect'
python package (especially the image viewer and UI widgets)
instead."""

import os
import sys
import cv2


# Add path to the pypvt3 package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import ui_basics
from vcp import imutils

if __name__ == "__main__":
    img = imutils.imread('../data/flamingo.jpg', flip_channels=True)

    # Select a rectangle
    valid, rect = ui_basics.select_rectangle(img, rect_color=(0, 255, 255), window_name='Rectangle selection demo')
    print('User confirmed: ', valid, ', rect: ', rect)

    # Select a single point
    valid, pt = ui_basics.select_point(img, point_color=(255, 0, 0), window_name='[Single] Point selection demo')
    print('User confirmed: ', valid, ', pt: ', pt)

    # Select multiple points
    pts = ui_basics.select_points(img, point_color=(255, 0, 0), window_name='[Multiple] Point selection demo')
    print('User selected pts: ', pts)