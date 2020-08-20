#!/usr/bin/env python
# coding=utf-8
"""
Showcasing Best Effort Streaming (BESt) - only streaming + some visualization stuff, no calibration, etc.
"""

import os
import sys
import cv2
import logging
import numpy as np
import time
from vito import pyutils
# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import bgm
from vcp import colormaps
from vcp import imutils
from vcp import imvis
from vcp import best

import multiprocessing as mp

#TODO use best.MulticamStreamer/MulticamStepper

def streaming_demo(cfg_file, folder, output_folder=None, output_fps=15):
    # cfg_file = 'replay.cfg'
    # folder='output-demo'
    # output_folder=None
    mp.log_to_stderr(logging.INFO)
    print('\n\n\nLoading streaming configuration: {}'.format(cfg_file)) # TODO ensure absolute paths!
    capture = best.Capture()
    capture.load_libconfig(os.path.join(folder, cfg_file), rel_path_base_dir=folder)

    print('\nStarting to stream {} sinks from {} devices'.format(capture.num_streams(), capture.num_devices()))
    print('The configured capture yields the following streams:')
    print('  Labels:             {}'.format(capture.frame_labels()))
    print('  Configuration keys: {}'.format(capture.configuration_keys()))
    print('  Frame types:        {}\n'.format(capture.frame_types()))

    if not capture.open():
        raise RuntimeError('Cannot open devices')
    if not capture.start():
        raise RuntimeError('Cannot start streams')

    # Some cameras (especially our tested RGBD sensors) take quite long to provide the 
    # initial frameset, so it's recommended to wait a bit longer for the device to finish
    # initialization.
    if not capture.wait_for_frames(10000):
        raise RuntimeError("Didn't receive an initial frameset within 10 seconds")

    # Depending on your setup, some devices may return empty frames (e.g. not synchronized RealSense streams),
    # so we'll wait for the first "complete" frameset.
    while True:
        if not capture.wait_for_frames(1000.0):
            print('[WARNING]: wait_for_frames timed out')
            continue
        frames = capture.next()
        if any([f is None for f in frames]):
            print('[WARNING]: Skipping invalid frameset')
        else:
            break
    
    # Query all intrinsics
    #print('\nStream intrinsics')
    #for i in range(capture.num_streams()):
    #    print('Stream {}\n  {}\n'.format(capture.frame_label(i), capture.intrinsics(i)))

    # Set up the stream storage
    if output_folder is None:
        storage = None
    else:
        storage = dict()
        storage_params = dict() # Used to save the replay config & calibration
        for idx in range(capture.num_streams()):
            lbl = capture.frame_label(idx)
            if capture.is_image(idx):
                # This stream can be stored as a video
                h, w = frames[idx].shape[:2]
                fn = capture.canonic_frame_label(idx) + '.mp4'
                
                storage_params[lbl] = best.StreamStorageParams(
                        best.StreamStorageParams.Type.Video, fn)
                storage[lbl] = best.SingleVideoStorage(
                    os.path.join(output_folder, fn),
                    output_fps, w, h, flip_channels=False, verbose=False)
            else:
                pn = capture.canonic_frame_label(idx)
                storage_params[lbl] = best.StreamStorageParams(
                        best.StreamStorageParams.Type.ImageSequence, pn)
                storage[lbl] = best.ImageSequenceStorage(
                    os.path.join(output_folder, pn),
                    file_extension='.png', flip_channels=False, verbose=False)

        capture.save_replay_config(output_folder, storage_params, False)

    while capture.all_devices_available():
        if not capture.wait_for_frames(1000.0):
            print('[WARNING]: wait_for_frames timed out')
            continue

        # Query the frames (since we know that all streams are available now)
        frames = capture.next()
        if any([f is None for f in frames]):
            print('[WARNING]: Skipping invalid frameset')
            continue

        # Colorize depth/infrared images for visualization
        def _col_rgb(f):
            return f #imutils.transform(f, 'histeq')

        def _col_depth(f):
            return imutils.transform(f, 'depth2surfnorm', 'surfnorm2rgb')
            #return imvis.pseudocolor(f, limits=[0, 5000], color_map=colormaps.colormap_turbo_rgb)

        def _col_ir(f):
            # RealSense infrared is provided (by default) in Y8 format
            if f.dtype == np.uint8:
                return f
            return imvis.pseudocolor(f, limits=None, color_map=colormaps.colormap_turbo_rgb)

        if storage is not None:
            for idx in range(len(frames)):
                storage[capture.frame_label(idx)].put_storage_request(frames[idx])

        vis_frames = [
            _col_depth(frames[idx])
                if capture.is_depth(idx)
                else (_col_ir(frames[idx])
                    if capture.is_infrared(idx)
                    else _col_rgb(frames[idx]))
            for idx in range(len(frames))]
        
        # Resize
        vis_frames = [imutils.fuzzy_resize(f, 0.75) for f in vis_frames]

        # Overlay stream labels
        vis_frames = [
            imvis.draw_text_box(vis_frames[idx], 
                capture.frame_label(idx) + (' [Undist. & Rect.]' if capture.is_rectified(idx) else ''),
                (vis_frames[idx].shape[1]//2, 10), 'north',
                bg_color=(0, 0, 0), font_color=(-1, -1, -1),
                font_scale=1.0, font_thickness=1,
                padding=5, fill_opacity=0.5)
            for idx in range(len(vis_frames))]

        # TODO remove - overlay depth and IR
        # vis_frames.append((vis_frames[1].astype(np.float32) * 0.5 + vis_frames[2].astype(np.float32) * 0.5).astype(np.uint8))

        # Display the images (as a single image/collage)
        collage = imvis.make_collage(vis_frames, num_images_per_row=2)
        k = imvis.imshow(collage, title='streams', wait_ms=100)
        if k == 27 or k == ord('q'):
            break

    # Shut down gracefully (would be called upon desctruction anyways)
    if not capture.stop():
        raise RuntimeError('Cannot stop streams')
    if not capture.close():
        raise RuntimeError('Cannot close devices')


def list_devices():
    handles = [('Webcams', best.list_webcams), ('RealSense2', best.list_realsense2_devices),
        ('mvBlueFox3', best.list_mvbluefox3_devices), ('Kinect Azure', best.list_k4a_devices),
        ('ZED', best.list_zed_devices)]
    for h in handles:
        try:
            print('Listing connected {}:\n{}\n'.format(h[0], h[1]()))
        except Exception as e:
            print('[Exception occured]: {}\n'.format(e))


def demo():
    list_devices()

    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'data-best')
    # cfg_files = [file for file in os.listdir(folder) if file.endswith(".cfg")]
    # cfg_files = ['realsense.cfg'] #, 'image_sequence.cfg', 'k4a.cfg', 'webcam.cfg']
    # # cfg_files = ['zed.cfg', 'realsense.cfg']
    # cfg_files = ['rgbds.cfg']
    # cfg_files = ['kinects.cfg']
    cfg_files = [
        # 'axis-http.cfg', 
        'axis-rtsp.cfg', 
        'mobotix.cfg']
    # cfg_files = ['kinects-old.cfg']
    # print('FOOOOOOOOOO CHECK - OLD means not synced!')
    # cfg_files = ['webcam.cfg']
    output_folder = None #'output-demo'
    output_fps = 15
    
    for cf in cfg_files:
        try:
            streaming_demo(cf, folder, output_folder, output_fps)
        except RuntimeError as e:
            print('[ERROR] while streaming: {}'.format(e))

if __name__ == "__main__":
    demo()
