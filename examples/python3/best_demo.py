#!/usr/bin/env python
# coding=utf-8
"""
Showcasing Best Effort Streaming (BESt) with uncalibrated streams.
This example:
* lists available devices (except for IP cameras, as we don't support network search)
* shows the livestream corresponding to the specified configuration file
* optionally stores the stream
"""

import os
import sys
import argparse
import numpy as np

# from vito import colormaps, pyutils
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vito import pyutils
from vcp import colormaps, imvis, best, config

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stream-config', '--config', '--stream',
        dest='stream_config_file', action='store',
        default='../data/data-best/webcam.cfg',
        help='Path to the stream configuration file (json or libconfig).')

    parser.add_argument('-v', '--verbose', action='store_true', dest='verbose', default=False,
        help="Be verbose. Make sure to also adjust the camera.verbose flag within the configuration file!")
    
    parser.add_argument('--capture', '--cap', action='store', dest='capture_directory', default=None,
        help="If you specify a capture directory, the streams will be stored there.")
    
    parser.add_argument('--cap-video', '--video', action='store_true', dest='store_videos', default=False,
        help="By default, we store image sequences. Add this flag to save video files.")
    
    parser.add_argument('--cap-output-fps', dest='capture_output_fps', default=15.0,
        help="Output framerate for video captures (if --video is specified).")
    
    parser.add_argument('--cap-split', action='store', type=int,
        default=None, help="If specified (and you record to a video), the output video will be split after X frames.")

    return parser.parse_args()


def save_replay_libconfig(storage_param_dict, filename):
    cfg = config.Config()
    num_stream = 0
    for stream_label, param in storage_param_dict.items():
        sink_id = 'sink{:d}'.format(num_stream)
        skip = False
        if param.type == best.StreamStorageParams.Type.ImageSequence:
            sink_type = 'imagedir'
            cfg.set_string(sink_id + '.directory', param.path)
        elif param.type == best.StreamStorageParams.Type.Video:
            sink_type = 'video'
            cfg.set_string(sink_id + '.video', param.path)
        else:
            skip = True
        if not skip:
            cfg.set_string(sink_id + '.sink_type', sink_type)
            cfg.set_string(sink_id + '.label', stream_label)
            cfg.set_string(sink_id + '.frame_type', "depth" if param.is_depth else "color")
        num_stream += 1
    cfg.save(filename)
    print('Saved replay configuration to {}'.format(filename))
    print(cfg.dumps())


class StorageParam(best.StreamStorageParams):
    """
    Extends best.StreamStorageParams with a flag indicating whether
    the corresponding stream is a depth stream or not.
    """
    def __init__(self, tp, pth, is_depth):
        super().__init__(tp, pth)
        self.is_depth = is_depth


class StreamingDemo(object):
    def __init__(self):
        self._args = parse_args()
        
        abs_cfg = os.path.abspath(self._args.stream_config_file)
        cfg_base_folder = os.path.dirname(abs_cfg)
        is_libconfig = not self._args.stream_config_file.lower().endswith('.json')

        self._streamer = best.MulticamStreamer(
            cfg_file=abs_cfg,
            is_libconfig=is_libconfig,
            cfg_file_rel_path_base_dir=cfg_base_folder,  # If not None, relative paths within the config file will be prefixed by this relative path
            callback_first_frameset=self._process_first_frameset, # Will be called after the first valid frameset has been received (params: capture, list of np ndarrays)
            callback_next_frameset=self._process_next_frameset, # Will be called for each of the following framesets
            wait_ms_first_frameset=5000,  # How many milliseconds to wait for the first frameset (some cameras take quite long to setup their internal sensors)
            wait_ms_next_frameset=1000,  # How many milliseconds to wait for each of the following framesets
            callback_streams_terminated=self._streams_terminated,  # Called if we observe an End-of-Stream, just before closing the devices
            verbose=self._args.verbose
        )

        self._store_capture = self._args.capture_directory is not None
        self._storage = None
        self._mean_ms_between_framesets = None
    
    def run(self):
        try:
            # Start the streaming thread
            self._streamer.start()
            # And block until it's finished
            self._streamer.wait()
        finally:
            self._stop_storage()

    def _process_first_frameset(self, capture, frameset):
        # This is a good place for stream-specific initialization tasks (e.g. allocate decoder/buffers for saving the output videos, initialize detectors, ...)
        num_streams = capture.num_streams()
        print('Started streaming: {}'.format(
            [capture.frame_label(i) for i in range(num_streams)]))
        for idx, frame in enumerate(frameset):
            print('SinkType for frame #{}: {}, shape: {}'.format(idx, capture.sink_type(idx), frame.shape))

    def _process_next_frameset(self, capture, frameset):
        # assert len(frameset) == 1  # Currently, we assume there's only one mobotix camera connected!
        if frameset[0] is None:
            self._streamer.stop()
            return

        if self._args.verbose:
            # Measure time between received framesets
            if self._mean_ms_between_framesets is None:
                ms_between_framesets = 0
            else:
                ms_between_framesets = pyutils.ttoc('[frameset received]')
            self._mean_ms_between_framesets = ms_between_framesets if self._mean_ms_between_framesets is None else 0.9*self._mean_ms_between_framesets + 0.1*ms_between_framesets
            if ms_between_framesets > 0:
                print('Frameset received after {:.2f} ms, avg. framerate {:.1f}'.format(ms_between_framesets, 1000.0/self._mean_ms_between_framesets))
            pyutils.tic('[frameset received]')

        vis_frames = list()
        vis_labels = list()
        for idx, frame in enumerate(frameset):
            frame_lbl = capture.frame_label(idx)
            vis_frame = imvis.pseudocolor(frame, [0, 5000], color_map=colormaps.colormap_turbo_rgb)\
                if capture.is_depth(idx) or capture.is_infrared(idx) else frame
            vis_frames.append(vis_frame)
            vis_labels.append(frame_lbl)
        
        # Overlay labels
        vis_frames = [imvis.draw_text_box(vis_frames[idx],
                vis_labels[idx], (vis_frames[idx].shape[1]//2, 40), text_anchor='north',
                bg_color=(220, 0, 0), font_color=(0, 0, 0))
                for idx in range(len(vis_frames))]

        # Forward to storage processes
        self._store(capture, frameset)
        
        # Display the live stream
        collage = imvis.make_collage(vis_frames, num_images_per_row=2, padding=0, fixed_size_per_image=(640,480))
        k = imvis.imshow(collage, "Live view", wait_ms=10)
        if k == ord('q') or k == 27:
            self._streamer.stop()

    def _streams_terminated(self, capture):
        print('Streams have terminated')
    
    def _store(self, capture, frameset):
        if not self._store_capture:
            return
        # Extract frames to store
        storage_labels = list()
        storage_frames = list()
        storage_is_depth = list()
        for idx, frame in enumerate(frameset):
            storage_frames.append(frame)
            storage_labels.append(capture.frame_label(idx))
            storage_is_depth.append(capture.is_depth(idx))

        # Ensure storage is set up
        if self._storage is None:
            # Ensure stream labels are unique
            unique_labels = list(set(storage_labels))
            if len(unique_labels) != len(storage_labels):
                print('Stream labels are not unique, cannot set up storage.')
                assert False
            # Initialize storage upon first received frameset (so we know image dimensions, etc.
            # as needed for the video codecs).
            self._storage = dict()
            storage_params = dict() # Used to save the replay config & calibration
            for lbl, frame, is_depth in zip(storage_labels, storage_frames, storage_is_depth):
                if self._args.store_videos and not is_depth: # depth can never be stored as video (requires 16bit)
                    # This stream can be stored as a video
                    h, w = frame.shape[:2]
                    filename = pyutils.slugify(lbl) + '.mp4'
                    filepath = os.path.join(self._args.capture_directory, filename)
                    print('Setting up RGB video storage for "{:s}" at "{:s}".'.format(lbl, filepath))
                    
                    storage_params[lbl] = StorageParam(
                            best.StreamStorageParams.Type.Video, filename, is_depth)
                    self._storage[lbl] = best.SingleVideoStorage(
                        filepath, self._args.capture_output_fps, w, h, flip_channels=False,
                        verbose=self._args.verbose, split_output=self._args.cap_split)
                else:
                    dirname = pyutils.slugify(lbl)
                    dirpath = os.path.join(self._args.capture_directory, dirname)
                    print('Setting up depth still frame storage for "{:s}" at "{:s}".'.format(lbl, dirpath))

                    storage_params[lbl] = StorageParam(
                            best.StreamStorageParams.Type.ImageSequence, dirname, is_depth)
                    self._storage[lbl] = best.ImageSequenceStorage(
                        dirpath, file_extension='.png' if is_depth else '.jpg',
                        flip_channels=False, verbose=self._args.verbose)
            # Save libconfig++ style configuration to replay the recorded streams
            save_replay_libconfig(storage_params, os.path.join(self._args.capture_directory, 'replay.cfg'))
        # Store the frames
        for lbl, frame in zip(storage_labels, storage_frames):
            self._storage[lbl].put_storage_request(frame)
    
    def _stop_storage(self):
        if self._storage is None:
            return
        for stream_label, store in self._storage.items():
            print('Finalizing stream storage for "{:s}".'.format(stream_label))
            store.stop()


def list_devices():
    handles = [
        ('Webcams', best.list_webcams),
        ('RealSense2', best.list_realsense2_devices),
        ('mvBlueFox3', best.list_mvbluefox3_devices),
        ('Kinect Azure', best.list_k4a_devices),
        ('ZED', best.list_zed_devices)
        ]
    for h in handles:
        try:
            print('Listing connected {}:\n{}\n'.format(h[0], h[1]()))
        except Exception as e:
            print('  [Exception occured]: {}\n'.format(e))


if __name__ == '__main__':
    # List known devices:
    list_devices()

    # Start streaming:
    StreamingDemo().run()
