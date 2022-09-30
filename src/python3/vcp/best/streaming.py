#!/usr/bin/env python
# coding=utf-8
"""Utilities to simplify multi-cam streaming."""

import datetime
# import os
# import sys
# import threading
# import time
# # import logging # doesn't work with multiprocessing (if you also use storage.py...)

from vcp.best.best_cpp import Capture
from vcp.best.frameset import Frame, Frameset, FrameType


class StreamingError(Exception):
    """Error raised whenever something goes wrong while trying to stream from a multi-cam configuration."""
    pass

### FIXME Test streamer functionality before re-enabling it
# class MulticamStreamer(object):
#     """Live stream (multi-cam setup is queried in a separate thread) with user-defined event callbacks."""

#     def __init__(self,
#             cfg_file,  # Path to the configuration file
#             is_libconfig,  # Set True if the configuration is libconfig (otherwise, it must be a JSON file)
#             cfg_file_rel_path_base_dir=None,  # If not None, relative paths within the config file will be prefixed by this relative path
#             callback_first_frameset=None, # Will be called after the first valid frameset has been received (params: capture, list of np ndarrays)
#             callback_next_frameset=None, # Will be called for each of the following framesets
#             wait_ms_first_frameset=10000,  # How many milliseconds to wait for the first frameset (some cameras take quite long to setup their internal sensors)
#             wait_ms_next_frameset=1000,  # How many milliseconds to wait for each of the following framesets
#             callback_streams_terminated=None,  # Called if we observe an End-of-Stream, just before closing the devices
#             verbose=True):
#         self._cfg_file = cfg_file
#         self._wait_ms_first_frameset = wait_ms_first_frameset
#         self._wait_ms_next_frameset = wait_ms_next_frameset
#         self._callback_first_frameset = callback_first_frameset
#         self._callback_next_frameset = callback_next_frameset
#         self._callback_streams_terminated = callback_streams_terminated
#         self._verbose = verbose

#         self._keep_running = False
#         self._capture_thread = threading.Thread(target=self._receive)

#         self._capture = Capture()
#         if is_libconfig:
#             self._capture.load_libconfig(cfg_file, rel_path_base_dir='' if cfg_file_rel_path_base_dir is None else cfg_file_rel_path_base_dir)
#         else:
#             self._capture.load_json_file(cfg_file)

#     def __del__(self):
#         self.stop()
#         if self._capture_thread.is_alive():
#             self._capture_thread.join()

#     def start(self):
#         if self._verbose:
#             print('[MulticamStreamer] Starting to stream {} sinks from {} devices'.format(self._capture.num_streams(), self._capture.num_devices()))
#             print('The configured capture yields the following streams:')
#             print('  * Labels:             {}'.format(self._capture.frame_labels()))
#             print('  * Configuration keys: {}'.format(self._capture.configuration_keys()))
#             print('  * Frame types:        {}\n'.format(self._capture.frame_types()))

#         if not self._capture.open():
#             raise StreamingError('Cannot open devices')
#         if not self._capture.start():
#             raise StreamingError('Cannot start streams')

#         # Some cameras (especially our tested RGBD sensors) take quite long to provide the 
#         # initial frameset, so it's recommended to wait a bit longer for the device to finish
#         # initialization.
#         if not self._capture.wait_for_frames(self._wait_ms_first_frameset):
#             raise StreamingError("Didn't receive an initial frameset within {} ms".format(self._wait_ms_first_frameset))

#         # Depending on your setup, some devices may return empty frames (e.g. not synchronized RealSense streams),
#         # so we'll wait for the first "complete" frameset.
#         while True:
#             if not self._capture.wait_for_frames(self._wait_ms_next_frameset):
#                 if self._verbose:
#                     print('[MulticamStreamer] capture.wait_for_frames() timed out')
#                 continue
#             frames = self._capture.next()
#             if any([f is None for f in frames]):
#                 if self._verbose:
#                     print('[MulticamStreamer] Skipping invalid frameset')
#             else:
#                 break
        
#         if self._callback_first_frameset is not None:
#             self._callback_first_frameset(self._capture, frames)
#         # Start thread
#         self._keep_running = True
#         self._capture_thread.start()

    
#     def stop(self):
#         if self._keep_running:
#             self._keep_running = False
    
#     def wait(self):
#         if self._capture_thread.is_alive():
#             self._capture_thread.join()
    
#     def _receive(self):
#         while self._capture.all_devices_available() and self._keep_running:
#             if not self._capture.wait_for_frames(self._wait_ms_next_frameset):
#                 if self._verbose:
#                     print('[MulticamStreamer] capture.wait_for_frames() timed out')
#                 continue

#             # Query the frames (since we know that all streams are available now)
#             frames = self._capture.next()
#             if self._callback_next_frameset is not None:
#                 self._callback_next_frameset(self._capture, frames)
#             time.sleep(0.015)

#         # Unless the user invoked "stop()", being here means that at least 1 stream ended, so we cannot stream anymore.
#         if not self._keep_running and self._callback_streams_terminated is not None:
#             self._callback_streams_terminated(self._capture)

#         # Shut down gracefully (would be called upon desctruction anyways)
#         if not self._capture.stop():
#             raise StreamingError('Cannot stop streams')
#         if not self._capture.close():
#             raise StreamingError('Cannot close devices')


class MulticamStepper(object):
    """Retrieve image data of a multi-cam setup one image at a time."""
    def __init__(self,
            cfg_file: str,       # Path to the configuration file
            is_libconfig: bool,   # Set True if the configuration is libconfig (otherwise, it must be a JSON file)
            cfg_file_rel_path_base_dir: str = None,  # If not None, relative paths within the config file will be prefixed by this relative path
            verbose: bool = False):
        self._cfg_file = cfg_file
        self._verbose = verbose
        self._frame_counter = 0
        self._vcp_labels = list()
        self._vcp_types = list()
        self._vcp_rectified = list()

        self._capture = Capture()
        if is_libconfig:
            self._capture.load_libconfig(cfg_file, rel_path_base_dir='' if cfg_file_rel_path_base_dir is None else cfg_file_rel_path_base_dir)
        else:
            self._capture.load_json_file(cfg_file)

    @property
    def capture(self) -> Capture:
        return self._capture
    
    def __del__(self):
        # Shut down gracefully (would be called upon desctruction anyways)
        if not self._capture.stop():
            raise StreamingError('Cannot stop streams')
        if not self._capture.close():
            raise StreamingError('Cannot close devices')

    def start(self, wait_ms_first_frameset: int = 10000) -> Capture:
        if self._verbose:
            print('\nStarting to receive {} streams from {} devices'.format(self._capture.num_streams(), self._capture.num_devices()))
            print('The configured capture yields the following streams:')
            print('  * Labels:             {}'.format(self._capture.frame_labels()))
            print('  * Configuration keys: {}'.format(self._capture.configuration_keys()))
            print('  * Frame types:        {}\n'.format(self._capture.frame_types()))

        if not self._capture.open():
            raise StreamingError('Cannot open devices')
        if not self._capture.start():
            raise StreamingError('Cannot start streams')
        if not self._capture.wait_for_frames(wait_ms_first_frameset):
            raise StreamingError("Didn't receive an initial frameset within {:.2f} seconds".format(wait_ms_first_frameset/1000.0))
        
        self._frame_counter = 0
        # Preload the information we need to populate
        # each frameset:
        num_streams = self._capture.num_streams()
        self._vcp_labels = [self._capture.frame_label(idx) for idx in range(num_streams)]

        self._vcp_types = list()
        for idx in range(num_streams):
            if self._capture.is_depth(idx):
                self._vcp_types.append(FrameType.Depth)
            elif self._capture.is_infrared(idx):
                self._vcp_types.append(FrameType.Intensity)
            elif self._capture.is_pointcloud(idx):
                self._vcp_types.append(FrameType.Pointcloud)
            elif self._capture.is_image(idx):
                self._vcp_types.append(FrameType.Color)
            else:
                print(f'[WARNING]: FrameType for "{self._vcp_labels[idx]}" not yet handled! Interpreting it as standard image.')
                self._vcp_types.append(FrameType.Color)

        self._vcp_rectified = [self._capture.is_rectified(idx) for idx in range(num_streams)]
        return self._capture
    
    def is_available(self) -> bool:
        return self._capture.all_devices_available()

    def next_frameset(self, wait_ms: int = 1000) -> Frameset:
        if not self.is_available():
            return None
        if not self._capture.wait_for_frames(wait_ms):
            if self._verbose:
                print('[WARNING]: capture.wait_for_frames() timed out')
            return None
        else:
            # Query the frames (since we know that all streams are available now)
            timestamp = datetime.datetime.now()
            framelist = self._capture.next()
            frameset = Frameset(self._frame_counter, timestamp)
            self._frame_counter += 1
            for fidx in range(len(framelist)):
                frame = Frame(
                    label=self._vcp_labels[fidx], frame_data=framelist[fidx],
                    frame_type=self._vcp_types[fidx], timestamp=timestamp,
                    is_rectified=self._vcp_rectified[fidx])
                frameset.frames.append(frame)
            return frameset
