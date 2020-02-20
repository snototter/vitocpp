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


def streaming_demo(cfg_file, folder):
    print('\n\n\nLoading streaming configuration: {}'.format(cfg_file)) # TODO ensure absolute paths!
    capture = best.Capture()
    capture.load_libconfig(os.path.join(folder, cfg_file), rel_path_base_dir=folder)

    print('Starting to stream {} sinks from {} devices'.format(capture.num_streams(), capture.num_devices()))
    print('The configured capture yields the following streams:')
    print('  Labels:             {}'.format(capture.frame_labels()))
    print('  Configuration keys: {}'.format(capture.configuration_keys()))
    print('  Frame types:        {}'.format(capture.frame_types()))

    if not capture.open():
        raise RuntimeError('Cannot open devices')
    if not capture.start():
        raise RuntimeError('Cannot start streams')
    # Some cameras (especially our tested RGBD sensors) take quite long to provide the 
    # initial frameset, so it's recommended to wait a bit longer for the device to finish
    # initialization.
    if not capture.wait_for_frames(5000):
        raise RuntimeError("Didn't receive an initial frameset within 5 seconds")

    while capture.all_devices_available():
        # wait_for_frames returns true, if frames for *all* streams are available.
        # Thus, we don't have to check for None frames afterwards.
        if not capture.wait_for_frames(1000.0):
            print('[WARNING]: wait_for_frames timed out')

        # Query the frames (since we know that all streams are available now)
        frames = capture.next()

        # Colorize depth/infrared images for visualization
        def _col_dir(f):
            # RealSense infrared is provided (by default) in Y8 format
            if f.dtype == np.uint8:
                return f
            # Depth values are usually 16bit, in millimeters
            return imvis.pseudocolor(f, limits=[0, 5000], color_map=colormaps.colormap_turbo_rgb)

        vis_frames = [
                _col_dir(frames[idx])
                if capture.is_depth(idx) or capture.is_infrared(idx)
                else frames[idx]
            for idx in range(len(frames))]

        # Overlay stream labels
        vis_frames = [
            imvis.draw_text_box(vis_frames[idx], capture.frame_label(idx),
                (vis_frames[idx].shape[1]//2, 10), 'north',
                bg_color=(0, 0, 0), font_color=(-1, -1, -1),
                font_scale=1.0, font_thickness=1,
                padding=5, fill_opacity=0.5)
            for idx in range(len(vis_frames))]

        # Display the images (as a single image/collage)
        collage = imvis.make_collage(vis_frames, num_images_per_row=2)
        k = imvis.imshow(collage, title='streams', wait_ms=10)
        if k == 27 or k == ord('q'):
            break

    # Shut down gracefully (would be called upon desctruction anyways)
    if not capture.stop():
        raise RuntimeError('Cannot stop streams')
    if not capture.close():
        raise RuntimeError('Cannot close devices')


def list_devices():
    handles = [('Webcams', best.list_webcams), ('RealSense2', best.list_realsense2_devices),
        ('mvBlueFox3', best.list_mvbluefox3_devices), ('Kinect Azure', best.list_k4a_devices)]
    for h in handles:
        try:
            print('Listing connected {}:\n{}\n'.format(h[0], h[1]()))
        except Exception as e:
            print('[Exception occured]: {}\n'.format(e))


def demo():
    list_devices()

    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'data-best')
    # cfg_files = [file for file in os.listdir(folder) if file.endswith(".cfg")]
    cfg_files = ['realsense.cfg'] #, 'image_sequence.cfg', 'k4a.cfg', 'webcam.cfg']
    cfg_files = ['webcam.cfg']
    
    for cf in cfg_files:
        try:
            streaming_demo(cf, folder)
        except RuntimeError as e:
            print('[ERROR] while streaming: {}'.format(e))


if __name__ == "__main__":
    demo()
