#!/usr/bin/env python
# coding=utf-8
"""Showcasing Best Effort Streaming (BESt)."""

import os
import sys
import cv2
import numpy as np
from vito import pyutils
# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import bgm
from vcp import colormaps
from vcp import imutils
from vcp import imvis
from vcp import best


def streaming_demo(cfg_file):
    print('\n\n\nLoading streaming configuration: {}'.format(cfg_file)) # todo ensure absolute paths!
    capture = best.Capture()
    capture.load_libconfig(cfg_file)
    if not capture.open():
        raise RuntimeError('Cannot open devices')
    if not capture.start():
        raise RuntimeError('Cannot start streams')
    if not capture.wait_for_initial_frames(5000):
        raise RuntimeError("Didn't receive an initial frameset within 5 seconds")
    
    if not capture.stop():
        raise RuntimeError('Cannot stop streams')
    if not capture.close():
        raise RuntimeError('Cannot close devices')

def demo():
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'data-best')
    cfg_files = [os.path.join(folder, file) for file in os.listdir(folder) if file.endswith(".cfg")]
    
    for cf in cfg_files:
        try:
            streaming_demo(cf)
        except RuntimeError as e:
            print('[ERROR] While streaming: {}'.format(e))


if __name__ == "__main__":
    demo()
