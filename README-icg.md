## Summary of Q&D Extensions/Changes

* [x] Rectified streams only include valid image regions.
* [x] Timestamped & serializable framesets

## Quickstart

* Check inline documentation of `examples/python3/streaming_icg.py`  
  The new `Frameset` and `Frame` classes provide improved usability over the old `vcp` interface.
  For example, `frame.is_color` vs `capture.is_color(stream_index)`
* In a nutshell:
  ```python
  stepper = ...
  stepper.start()
  while stepper.is_available():
      frameset = stepper.next_frameset()

      # Access a single image (as "Frame" object):
      first_image = frameset.frames[0]

      # Access the image data of the "Frame" above as numpy ndarray:
      frameset.frames[0].frame_data

      # Access all "Frame"s of a specific camera. For example, a
      # Kinect which could have been configured to provide color,
      # depth & intensity:
      kinect_frames = frameset['My Kinect 1']
  ```

