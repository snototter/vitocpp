#!/usr/bin/env python
# coding=utf-8
"""Command line streaming utility providing the option to store images to disk."""

import argparse
import os
import sys
import logging
import numpy as np
import multiprocessing as mp

from vito import pyutils

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import colormaps
from vcp import imutils
from vcp import imvis
from vcp import best

class LiveStreamDemo(object):
    def __init__(self, args):
        self._args = args
        abs_cfg = os.path.abspath(args.config_file)
        cfg_base_folder = os.path.dirname(abs_cfg)
        is_libconfig = not args.config_file.lower().endswith('.json')

        self._streamer = best.MulticamStreamer(
            cfg_file=abs_cfg,
            is_libconfig=is_libconfig,
            cfg_file_rel_path_base_dir=cfg_base_folder,  # If not None, relative paths within the config file will be prefixed by this relative path
            callback_first_frameset=self._cb_first_frameset, # Will be called after the first valid frameset has been received (params: capture, list of np ndarrays)
            callback_next_frameset=self._cb_next_frameset, # Will be called for each of the following framesets
            wait_ms_first_frameset=args.wait_initial,  # How many milliseconds to wait for the first frameset (some cameras take quite long to setup their internal sensors)
            wait_ms_next_frameset=args.wait_next,  # How many milliseconds to wait for each of the following framesets
            callback_streams_terminated=self._cb_streams_terminated,  # Called if we observe an End-of-Stream, just before closing the devices
            verbose=True
        )
        self._intrinsics = None
        self._extrinsics = None
        self._storage = None
    
    def run(self):
        self._streamer.start()
        # self._streamer.wait()  # Not needed, because the python application will continue running as long as there is the streaming thread active ;-)
    
    def _cb_first_frameset(self, capture, frameset):
        mp.get_logger().info('[Demo] First frameset received')
        if True:
            return
        # Query all intrinsics & extrinsics
        # self._intrinsics = {capture.frame_label(i): capture.intrinsics(i) for i in range(len(frameset))}
        self._intrinsics = {capture.frame_label(i): None for i in range(len(frameset))}
        self._extrinsics = {capture.frame_label(i): capture.extrinsics(i) for i in range(len(frameset))}
        # Tell user which streams are calibrated
        unknown_int = [k for k in self._intrinsics if self._intrinsics[k] is None]
        unknown_ext = [k for k in self._extrinsics if any([t is None for t in self._extrinsics[k]])]
        if len(unknown_int) > 0:
            mp.get_logger().info('[Demo] Intrinsics not calibrated for streams:\n       * {}'.format('\n       * '.join(unknown_int)))
        else:
            mp.get_logger().info('[Demo] All streams have valid intrinsic calibration.')
        if len(unknown_ext) > 0:
            mp.get_logger().info('[Demo] Extrinsics not calibrated for streams:\n       * {}'.format('\n       * '.join(unknown_ext)))
        else:
            mp.get_logger().info('[Demo] All streams have valid extrinsic calibration.')
        # Set up storage handlers
        self._storage = dict()
        storage_params = dict() # Used to save the replay config & calibration
        for idx in range(capture.num_streams()):
            lbl = capture.frame_label(idx)
            # if capture.is_image(idx):
            #     # This stream can be stored as a video
            #     h, w = frames[idx].shape[:2]
            #     fn = capture.canonic_frame_label(idx) + '.mp4'
            #     storage_params[lbl] = best.StreamStorageParams(
            #             best.StreamStorageParams.Type.Video, fn)
            #     storage[lbl] = best.SingleVideoStorage(
            #         os.path.join(output_folder, fn),
            #         output_fps, w, h, flip_channels=False, verbose=False)
            # else:
            pn = capture.canonic_frame_label(idx)
            storage_params[lbl] = best.StreamStorageParams(
                    best.StreamStorageParams.Type.ImageSequence, pn)
            self._storage[lbl] = best.ImageSequenceStorage(
                os.path.join(self._args.output_folder, pn),
                file_extension='.png', flip_channels=False, verbose=False)
        capture.save_replay_config(self._args.output_folder, storage_params, True)
        # Process this frameset the same way as all the others:
        self._cb_next_frameset(capture, frameset)

    def _cb_next_frameset(self, capture, frameset):
        if any([f is None for f in frameset]):
            mp.get_logger().warning('[Demo] Skipping invalid frameset')
            return

        # Colorize depth/infrared images for visualization
        def _col_rgb(f):
            return f #imutils.transform(f, 'histeq')

        def _col_depth(f):
            # return imutils.transform(f, 'depth2surfnorm', 'surfnorm2rgb')
            return imvis.pseudocolor(f, limits=[0, self._args.max_depth], color_map=colormaps.colormap_turbo_rgb)

        def _col_ir(f):
            #TODO 
            # Normalize/stretch, etc. using: self._args.max_ir
            if f.dtype == np.uint8:
                return f
            return imvis.pseudocolor(f, limits=None, color_map=colormaps.colormap_turbo_rgb)

        vis_frames = [
            _col_depth(frameset[idx])
                if capture.is_depth(idx)
                else (_col_ir(frameset[idx])
                    if capture.is_infrared(idx)
                    else _col_rgb(frameset[idx]))
            for idx in range(len(frameset))]
        # Resize
        vis_frames = [imutils.fuzzy_resize(f, self._args.scale) for f in vis_frames]
        # Overlay frame labels
        if self._args.label_overlay:
            vis_frames = [
                imvis.draw_text_box(vis_frames[idx], 
                    capture.frame_label(idx) + (' [Undist. & Rect.]' if capture.is_rectified(idx) else ''),
                    (vis_frames[idx].shape[1]//2, 10), 'north',
                    bg_color=(0, 0, 0), font_color=(-1, -1, -1),
                    font_scale=1.0, font_thickness=1,
                    padding=5, fill_opacity=0.5)
                for idx in range(len(vis_frames))]
        # Show collage
        collage = imvis.make_collage(vis_frames, num_images_per_row=self._args.images_per_row)
        k = imvis.imshow(collage, title='Live Stream', wait_ms=10)
        # Handle user input (if any)
        if k == 27 or k == ord('q'):
            self._streamer.stop()
        elif k == ord('s') or k == 85 or k == 86 or k == 10:
            # 85, 86, 10: PgUp, PgDown, Enter
            if self._storage is not None:
                for idx in range(len(frameset)):
                    self._storage[capture.frame_label(idx)].put_storage_request(frameset[idx])
            mp.get_logger().info('[Demo] Current frameset has been saved.')
        # elif k > 0 and k != 255:
        #     print('[DEV] Unknown key: {:d}'.format(int(k)))
    
    def _cb_streams_terminated(self, capture):
        if not capture.all_devices_available():
            mp.get_logger().info('[Demo] End-of-stream reached, goodbye!')
        else:
            mp.get_logger().info('[Demo] You requested shutdown, goodbye!')
        if self._storage is not None:
            for k in self._storage:
                self._storage[k].stop()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file', action='store',
        help='Path to the stream configuration file (json or libconfig)')
    parser.add_argument('--output', action='store', default='output-snapshots', dest='output_folder',
        help='Folder where to store the snapshots.')

    # Streaming params
    parser.add_argument('--wait-initial', action='store', type=pyutils.check_positive_int,
        default=10000, help='Number of milliseconds to wait for the initial frameset (some sensors take longer to start streaming).')
    parser.add_argument('--wait-next', action='store', type=pyutils.check_positive_int,
        default=1000, help='Maximum wait time (milliseconds) for every other frameset.')

    # General visualization params
    parser.add_argument('--overlay', action='store_true', dest='label_overlay',
        help='Enable camera label overlay.')
    parser.add_argument('--no-overlay', action='store_false', dest='label_overlay',
        help='Disable camera label overlay.')
    parser.set_defaults(label_overlay=True)

    parser.add_argument('--scale', action='store', type=pyutils.check_positive_real, default=0.5,
        help="Scale the image before displaying them (e.g. --scale 0.5 to show at 50%).")
    
    parser.add_argument('--per-row', action='store', type=pyutils.check_positive_int, default=3,
        dest='images_per_row', help='Number of images per row of the displayed collage.')

    parser.add_argument('--max-depth', action='store', type=pyutils.check_positive_int, default=5000,
        help="Expected maximum depth value (typically in Millimeters) to cut off visualization.")
    
    parser.add_argument('--max-ir', action='store', type=pyutils.check_positive_int, default=255,
        help="Expected maximum infrared value (reflectance) to cut off visualization.")

    args = parser.parse_args()
    return args

if __name__ == "__main__":
    mp.log_to_stderr(logging.INFO)
    args = parse_args()
    LiveStreamDemo(args).run()
