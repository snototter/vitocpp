# BESt - Best Effort Streaming
This submodule enables "best effort streaming" from multiple cameras.
To start streaming, you simply need a libconfig++ style configuration file (see "streaming examples" below).

Currently supported sensors:
* Standard webcams, videos and image sequences (i.e. all images within a folder).
* IP cameras (MJPEG & H.264 over RTSP & HTTP)
* RGB-D sensors (ZED stereo cam, Intel RealSense, Microsoft Azure Kinect)

TODO add cmd for terminal/ui usage

## Specifics & Caveats
* For USB3 camera setups, you should increase the internal USB memory.<br/>For example, on Unix, append `usbcore.usbfs_memory_mb=1024` (enter an approximate memory consumption according to your setup here) to the parameter `GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`. Then `sudo update-grub` and `reboot`.
* Sensor-specific SDKs must be installed manually, then enable the corresponding `VCP_BEST_WITH_xxx` option of VCP via CMake
  * ZED SDK: https://www.stereolabs.com/developers/release/
  * RealSense: https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
  * Azure Kinect https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md


## Streaming Example: Webcam
This configuration file sets up live streaming from a standard webcam:
```c++
camera-webcam = {
  // Required
  sink_type = "webcam";
  // Required. On Unix, -1 selects the first available webcam.
  device_number = -1;
}
```


## Streaming Example: Image Sequence & Video File
To replay a video + corresponding still frames (e.g. RGB-D sensor recording where RGB is stored as video and depth frames are stored as 16-bit png files), use the image sequence and video sinks:
```c++
camera-video-playback = {
  // Required
  sink_type = "video";
  // Required.
  video = "/path/to/video";
  // Optional
  fps = 20;
}

camera-sequence = {
  // Required
  sink_type = "image-sequence";
  // Required.
  directory = "/path/to/image-directory";
}
```


## Streaming Example: Azure Kinect (K4A)
## Streaming Example: RealSense
## Streaming Example: IP Cameras (Axis)

```c++
//TODO split into 2 examples, 1 mono, 1 stereo
// 4 image streams (1 stereo + 2 monocular) from 2 physical devices.

camera-rtsp-stereo = {
  // Required
  sink_type = "axis-stereo";

  // Optional
  user = "user";
  password = "password";
 
  // Configure the data stream. We support:
  // * http/mjpg over tcp
  // * rtsp/h264 over tcp/udp (tcp is preferred for Axis;
  //   as our cameras shut down the stream after several
  //   seconds (up to several minutes at most) if using UDP)
  // * rtsp/mjpeg over tcp/udp (again tcp preferred for Axis)
  // Defaults to rtsp/h264 over tcp.
  application_protocol = "rtsp";
  encoding = "h264";
  transport_protocol = "tcp";

  // Required, hostname or IP of the two stereo views.
  host_left = "192.168.0.50";
  host_right = "192.168.0.51";

  // Optional frame rate (integer).
  fps = 20;

  // Required for h264 streams (as we need to know the frame
  // resolution to initialize the decoder).
  // Optional for MJPEG streams.
  frame_width = 1024;
  frame_height = 768;

  // Optional.
  color_as_bgr = true;
  verbose = true;
};

camera-mono-rtsp = {
  // Required
  sink_type = "axis";

  // Optional
  user = "user";
  password = "password";
  
  // Configure the data stream:
  application_protocol = "rtsp";
  transport_protocol = "tcp";
  encoding = "h264";

  // Required
  host = "192.168.0.50";

  // Optional
  fps = 20;
  color_as_bgr = true;
  verbose = true;
};

camera-mono-http = {
  // Required
  sink_type = "axis";

  // Optional
  user = "user";
  password = "password";

  // Configure the data stream:  
  application_protocol = "http";
  transport_protocol = "tcp";
  encoding = "mjpeg";

  // Required
  host = "192.168.0.51";

  // Optional
  fps = 20;
  color_as_bgr = true;
  verbose = true;
};
```

