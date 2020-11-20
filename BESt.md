# BESt - Best Effort Streaming
This submodule enables "best effort streaming" from multiple cameras.
To start streaming, you simply need a libconfig++ style configuration file (see "streaming examples" below).
Note that the following examples only show the most minimalistic configuration (mandatory configuration parameters) to be able to start streaming.
For a more detailed overview of available options, please refer to the separate configuration files within `<VCP_ROOT_DIR>/examples/data/data-best/*.cfg`.

Currently supported sensors:
* Standard webcams, videos and image sequences (i.e. all images within a folder).
* IP cameras (MJPEG & H.264 over RTSP & HTTP)
* RGB-D sensors (ZED stereo cam, Intel RealSense, Microsoft Azure Kinect)

<!--TODO add images (setup + capture result)
TOC generator: https://ecotrust-canada.github.io/markdown-toc/
-->

Table-of-Contents:
* [Quickstart](#quickstart)
* [Specifics & Caveats](#specifics---caveats)
* [Common Sink Configurations](#common-sink-configurations)
* [Exemplary Streaming Configurations](#exemplary-streaming-configurations)
  + [Streaming Example: Webcam](#streaming-example--webcam)
  + [Streaming Example: Image Sequence & Video File](#streaming-example--image-sequence---video-file)
  + [Streaming Example: Azure Kinect (K4A)](#streaming-example--azure-kinect--k4a-)
  + [Streaming Example: RealSense](#streaming-example--realsense)
  + [Streaming Example: StereoLabs ZED](#streaming-example--stereolabs-zed)
  + [Streaming Example: IP Cameras](#streaming-example--ip-cameras)
    - [Generic IP Camera Streaming](#generic-ip-camera-streaming)
    - [Axis IP Camera Streaming](#axis-ip-camera-streaming)
    - [Mobotix IP Camera Streaming](#mobotix-ip-camera-streaming)



## Quickstart
If you enable the examples when building `vcp`, the following CLI will work out-of-the-box:
```bash
# 1) Switch to virtual environment
cd <VCP_ROOT_DIR>/examples/python3
source .venv3/bin/activate

# 2) Show the live stream of a webcam
python best_demo.py --stream-config ../data/data-best/webcam.cfg

# 3a) Show the live stream of a webcam and store the stream as an image sequence to 'OUTPUT_FOLDER'
python best_demo.py --stream-config ../data/data-best/webcam.cfg --capture OUTPUT_FOLDER

# 3b) Show the live stream of a webcam and store the stream as a video to 'OUTPUT_FOLDER'
python best_demo.py --stream-config ../data/data-best/webcam.cfg --capture OUTPUT_FOLDER --video

# 4) After recording the data (see steps 3a or 3b), you can replay the recorded images/videos:
python best_demo.py --stream-config OUTPUT_FOLDER/replay.cfg
```

## Specifics & Caveats
* For USB3 camera setups, you should increase the internal USB memory.<br/>For example, on Unix, append `usbcore.usbfs_memory_mb=1024` (enter an approximate memory consumption according to your setup) to the parameter `GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`. Then `sudo update-grub` and `reboot`.
* Sensor-specific SDKs must be installed manually, then enable the corresponding `VCP_BEST_WITH_xxx` option of VCP via CMake
  * ZED SDK: https://www.stereolabs.com/developers/release/
  * RealSense: https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
  * Azure Kinect https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md

## Common Sink Configurations
All implemented image data sinks support the following (optional) configuration parameters (these could be added to any of the sensor-specific examples below):
```c++
...
  // [Optional] Specify a custom label.
  label = "My stream label";

  // [Optional] Work with color channels in RGB or BGR order.
  color_as_rgb = true;

  // [Optional] Basic image transformations (combine by listing them comma-separated).
  // * Mirroring or rotation: fliplr, flipud, rotate-90/180/270
  // * Histogram equalization: hist-eq
  // * Color conversion: {bgr|rgb}2gray, {bgr|rgb}2hsv, {bgr|rgb}2lab
  transform = "rotate-90, hist-eq";

  // [Optional] Path to the intrinsic calibration file.
  calibration_file = "intrinsic-calibration-sensor-X.xml";

  // [Optional] Undistort/rectify this stream (requires a valid "calibration_file").
  rectify = "True";

  // [Optional] Log sink initialization verbosely.
  verbose = false;

  // [Optional, supported by most sinks] Frame rate.
  fps = 15;

  // [Optional, supported by most sinks] Frame resolution.
  resolution = [1080, 720];
  // Alternatively, this can also be configured as:
  // width = 1080;
  // height = 720;
...
```


## Exemplary Streaming Configurations
### Streaming Example: Webcam
This configuration file sets up live streaming from a standard webcam:
```c++
sink-webcam = {
  sink_type = "webcam";

  // On Unix, -1 selects the first available webcam.
  device_number = -1;
};
```


### Streaming Example: Image Sequence & Video File
To replay a video + corresponding still frames (e.g. RGB-D sensor recording where RGB is stored as video and depth frames are stored as 16-bit png files), use the image sequence and video sinks:
```c++
sink-video-playback = {
  sink_type = "video";

  video = "/path/to/videofile";

  // [Optional]
  // If frame rate is set, the video will be played back (in a separate thread), 
  // allowing you to simulate a realistic live video stream. 
  fps = 20;
};

sink-sequence = {
  // Required
  sink_type = "image-sequence";

  // Required.
  directory = "/path/to/image-directory";
};
```


### Streaming Example: Azure Kinect (K4A)
TODO

### Streaming Example: RealSense
The following configuration enables streaming from an Intel RealSense2 sensor:
```c++
sink = {
  sink_type = "realsense2";

  // [Optional] If omitted or empty, the first available RealSense will be opened.
  serial_number = "";

  // [Optional] Select which streams to receive:
  // * Single-stream only: color, depth, ir-left, ir-right
  // * Multiple streams: rgbd (or color-depth), rgbdir (rgbd + left ir), rgbdir2 (rgbd + both ir)
  stream_type = "rgbd"; 

  // [Optional] Do you want to work with RGB or BGR?
  color_as_rgb = true;

  // [Optional] If true, depth stream will be aligned to the color stream.
  align_depth_to_color = true;

  // [Optional] Set the frame rate of the color sensor as integer (check supported values, e.g. 0 [any], 6, 15, 30).
  rgb_frame_rate = 15;

  // [Optional] Set the resolution of the color sensor.
  rgb_resolution = [848, 480];
};
```

### Streaming Example: StereoLabs ZED
TODO

### Streaming Example: IP Cameras
IP camera streaming currently supports:
* MJPEG over HTTP
* MJPEG over RTSP
* H264 over RTSP

If you know the streaming URL for your camera, you can set up streaming from a "generic IP camera".
Some specific camera manufacturers (currently Axis and Mobotix) are supported out-of-the-box.
See the following exemplary configurations:

#### Generic IP Camera Streaming

```c++
sink-generic = {
  sink_type = "axis";
  host = "<IP/HOSTNAME>";
  stream_url = "<VIDEO-URL>";
  
  application_protocol = "<HTTP/RTSP>";
  transport_protocol = "<TCP/UDP>";
  encoding = "<MJPEG/H264>";
};
```

#### Axis IP Camera Streaming
The following configuration sets up streaming from an Axis camera via MJPEG over HTTP (actually not recommended due to the high bandwidth requirements).
Other supported streaming modes are `MJPEG over RTSP` and `H264 over RTSP`, just adjust the corresponding parameters accordingly (`application_protocol`, `transport_protocol` and `encoding`), see also the Mobotix streaming example below.

```c++
sink-example-axis-via-http = {
  sink_type = "axis";
  host = "<IP/HOSTNAME>";
  user = "<USER>";
  password = "<PASSWORD>";
  
  application_protocol = "http";
  transport_protocol = "tcp";
  encoding = "MJPEG";
};
```

#### Mobotix IP Camera Streaming
The following configuration sets up streaming from a Mobotix camera via H264 over RTSP.
To properly intialize the video codec, you have to additionally specify the stream resolution (here, 800x600).

```c++
sink-example-mobotix-via-rtsp = {
  sink_type = "mobotix";
  host = "<IP/HOSTNAME>";
  user = "<USER>";
  password = "<PASSWORD>";
  
  application_protocol = "rtsp";
  encoding = "h264";
  transport_protocol = "tcp";
  resolution = [800, 600];
};
```

