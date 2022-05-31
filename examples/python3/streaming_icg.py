"""
Demo on how to use the Q&D ICG extensions, i.e. timestamped framesets


vcp MUST BE INSTALLED in your virtualenv
CAVEAT: check your CMake configuration, vcp uses an outdated FindPython
        module. Changes are that you built the library FOR THE WRONG
        PYTHON VERSION!
        
        Check the `PYTHON3_...` CMake variables, for example via:
        ```bash
        cd PATH/TO/VCP/build
        ccmake ..
        ```


If you did not compile vcp yet:
1. Activate your virtualenv & ensure updated pip:
   ```bash
   source PATH/TO/VENV/bin/activate
   python -m pip install -U pip
   ```
2. Build & install vcp:
   ```bash
   cd PATH/TO/VCP/build
   ccmake ..

   # CHECK THE PYTHON VERSION YOU ARE LINKING AGAINST! See the
   # 'CAVEAT' above. Once verified, continue building:

   make -j
   make install
   ```
   This installs vcp into `PATH/TO/VCP/gen`
3. Copy the generated `vcp.pth` file into your virtualenv's site packages:
   ```bash
   cp PATH/TO/VCP/gen/vcp.pth PATH/TO/VENV/lib/PYTHON/site-packages
   ```


OpenCV MUST BE INSTALLED in your virtualenv
CAVEAT: Ensure that the OpenCV bindings link to the same version you
        used to build the C++ library.
        One tested way:
        * This assumes that you installed the following OS packages, e.g.
          via `apt`:
          * `libopencv-dev`
          * `python3-opencv`
        * Build & install vcp as outlined above
        * Find your system's OpenCV and add a link to it into your virtualenv:
          ```bash
          opencv_lib=$(find /usr -name cv2* | grep python3 | head -n 1)
          
          # CHECK THAT THIS POINTS TO THE CORRECT PYTHON VERSION!
          # If not, run `find...` without limiting via `head` and manually
          # select the correct library!

          opencvlibdir=$(dirname "${opencv_lib}")

          # ADJUST THE FOLLOWING PATH!
          echo ${opencvlibdir} > PATH/TO/VENV/lib/PYTHON/site-packages/cv2.pth
          ```



Run the demo:
1. Prepare a camera configuration file (if you're reading
   this, you know where to find one)
2. Activate your virtualenv
3. cd PATH/TO/VCP/examples/python3
4. python streaming_icg.py CONFIGURATION.cfg
"""

import argparse
import logging
import numpy as np
import os
import time
from vcp import best, imvis, imutils, colormaps
from vito import pyutils


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('stream_config_file', action='store',
                        help='Path to the stream configuration file (must be in libconfig format).')

    # Everything else below is optional (read: not needed if you just want
    # to load some webcam/IP camera/Kinect images quickly)

    # General streaming params
    parser.add_argument('--wait-next', '--wait-for-next-frameset', action='store', type=pyutils.check_positive_int,
                        default=10000,
                        help='Maximum time (milliseconds) to wait for the next frameset. Default: %(default)d')

    # General visualization params
    parser.add_argument('--labels', action='store_true', dest='label_overlay',
                        help='Enable camera label overlay.')
    parser.add_argument('--no-labels', action='store_false', dest='label_overlay',
                        help='Disable camera label overlay.')
    parser.set_defaults(label_overlay=True)

    parser.add_argument('--scale', action='store', type=pyutils.check_positive_real, default=0.8,
                        help="Scale the image before displaying them (e.g. --scale 0.5 to show at 50%%).")

    parser.add_argument('--per-row', action='store', type=pyutils.check_positive_int, default=2,
                        dest='images_per_row', help='Number of images per row of the displayed collage.')

    parser.add_argument('--max-depth', action='store', type=pyutils.check_positive_int, default=5000,
                        help="Expected maximum depth value (typically in millimeters) to cut off visualization.")

    parser.add_argument('--max-ir', action='store', type=pyutils.check_positive_int, default=255,
                        help="Expected maximum infrared value (reflectance) to cut off visualization.")


    args = parser.parse_args()

    return args


def _colorize_depth(f, args):
    # return imutils.transform(f, 'depth2surfnorm', 'surfnorm2rgb')
    return imvis.pseudocolor(f, limits=[0, args.max_depth], color_map=colormaps.colormap_turbo_rgb)


def _colorize_ir(f, args):
    # Normalize/stretch, etc. using: self._args.max_ir
    f = ((f.astype(np.float32) / args.max_ir) * 255).astype(np.uint8)
    # return imvis.pseudocolor(f, limits=[0, 255], color_map=colormaps.colormap_turbo_rgb)
    return imutils.transform(f, 'histeq', 'gray2rgb')


def _create_visualization(frameset, capture, args):
    # A frameset consists of multiple images. These could be
    # color, depth or infrared. Thus, we need to convert them
    # all to a 3-channel uint8 format for display:
    vis_frames = [_colorize_depth(frameset[idx], args)
                  if capture.is_depth(idx)
                  else (_colorize_ir(frameset[idx], args)
                        if capture.is_infrared(idx)
                        else frameset[idx])
                  for idx in range(len(frameset))]

    # Resize for display
    vis_frames = [imutils.fuzzy_resize(f, args.scale) for f in vis_frames]
    # Overlay frame labels
    if args.label_overlay:
        vis_frames = [imvis.draw_text_box(vis_frames[idx],
                                          capture.frame_label(idx) + (
                                              ' [Undistorted & Rectified]' if capture.is_rectified(
                                                  idx) else ' [Not explicitly undistorted]'),
                                          (vis_frames[idx].shape[1] // 2, 10), 'north',
                                          bg_color=(0, 0, 0), font_color=(-1, -1, -1),
                                          font_scale=1.0, font_thickness=1,
                                          padding=5, fill_opacity=0.5)
                      for idx in range(len(vis_frames))]

    # Combine all streams/frames in this frameset into a single collage image
    return imvis.make_collage(vis_frames, num_images_per_row=args.images_per_row)


def main_demo():
    args = parse_args()

    # We need to know the absolute path of the configuration file to
    # adjust relative paths within this config
    abs_cfg_file = os.path.abspath(args.stream_config_file)
    abs_cfg_dirname = os.path.dirname(abs_cfg_file)
    stepper = best.MulticamStepper(abs_cfg_file, is_libconfig=True,
                                   cfg_file_rel_path_base_dir=abs_cfg_dirname,
                                   verbose=True)

    capture = stepper.start()

    # The "capture" object allows querying information about the streams you configured.
    # For example:
    num_frames_per_frameset = capture.num_streams()
    frame_labels = [capture.frame_label(idx) for idx in range(num_frames_per_frameset)]
    logging.info(f'Starting streaming {num_frames_per_frameset} streams: {frame_labels}')

    # Process/display the live stream:
    is_paused = False
    while stepper.is_available():
        capture, frameset = stepper.next_frameset(wait_ms=args.wait_next) #TODO change interface!
        if frameset is None:
            logging.error('No frameset received, aborting now.')
            break
        if any([f is None for f in frameset]):
            logging.warning('Skipping frameset with some empty frames.')
            time.sleep(0.5)
            continue

        vis = _create_visualization(frameset, capture, args)
        k = imvis.imshow(vis, 'Live', wait_ms=-1 if is_paused else 10) & 0xff
        if k == ord('q') or k == 27:
            break
        elif k == ord('p'):
            is_paused = not is_paused

    # The "stepper" will automatically close the connection upon its destruction


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)-15s %(levelname)-8s %(message)s')
    main_demo()
