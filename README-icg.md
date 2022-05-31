## Summary of Q&D Extensions/Changes

* [x] Rectified streams only include valid image regions.
* [x] Timestamped & serializable framesets

## Caveats

This is a C++ library with Python bindings which uses several third party library. This means you must pay attention to:
* The C++ library will be linked against shared libraries on your system (for example, OpenCV). If you want to use the Python bindings, your Python environment **must use the same OpenCV version**.
* `vcp` uses an outdated FindPython CMake module. This can cause issues if you want to use a different Python version.

As of May 31, 2022, the most up-to-date documentation (building, installing & using the library + bindings) can be found as inline documentation of `examples/python3/streaming_icg.py`.

## Quickstart

* Check inline documentation of `examples/python3/streaming_icg.py`  
  This describes how to set up the library and how to use the changed `vcp.best` interface (`Frameset` and `Frame` classes).
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

