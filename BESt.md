# BESt - Best Effort Streaming
This submodule enables "best effort streaming" from multiple cameras.
To start streaming, you simply need a libconfig++ style configuration file (see "streaming examples" below).

Currently supported sensors:
* Standard webcams, videos and image sequences (i.e. all images within a folder).
* IP cameras (MJPEG & H.264 over RTSP & HTTP)
* RGB-D sensors (ZED stereo cam, Intel RealSense, Microsoft Azure Kinect)

TODO add cmd for terminal/ui usage

TODO only mandatory configuration keys are stated, refer to examples/data for more detailed configuration options

```bash
# Show the live stream of a webcam
python best_demo.py --stream-config ../data/data-best/webcam.cfg

# Show the live stream of a webcam and store the stream as an image sequence to 'OUTPUT_FOLDER'
python best_demo.py --stream-config ../data/data-best/webcam.cfg --capture OUTPUT_FOLDER

# Show the live stream of a webcam and store the stream as a video to 'OUTPUT_FOLDER'
python best_demo.py --stream-config ../data/data-best/webcam.cfg --capture OUTPUT_FOLDER --video

# Replay the recorded images/videos
python best_demo.py --stream-config OUTPUT_FOLDER/replay.cfg
```

## Specifics & Caveats
* For USB3 camera setups, you should increase the internal USB memory.<br/>For example, on Unix, append `usbcore.usbfs_memory_mb=1024` (enter an approximate memory consumption according to your setup) to the parameter `GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`. Then `sudo update-grub` and `reboot`.
* Sensor-specific SDKs must be installed manually, then enable the corresponding `VCP_BEST_WITH_xxx` option of VCP via CMake
  * ZED SDK: https://www.stereolabs.com/developers/release/
  * RealSense: https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
  * Azure Kinect https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md


## Streaming Example: Webcam
This configuration file sets up live streaming from a standard webcam:
```c++
sink-webcam = {
  // Required
  sink_type = "webcam";
  // Required. On Unix, -1 selects the first available webcam.
  device_number = -1;
};
```


## Streaming Example: Image Sequence & Video File
To replay a video + corresponding still frames (e.g. RGB-D sensor recording where RGB is stored as video and depth frames are stored as 16-bit png files), use the image sequence and video sinks:
```c++
sink-video-playback = {
  // Required
  sink_type = "video";
  // Required.
  video = "/path/to/video";
  // Optional
  fps = 20;
};

sink-sequence = {
  // Required
  sink_type = "image-sequence";
  // Required.
  directory = "/path/to/image-directory";
};
```


## Streaming Example: Azure Kinect (K4A)

## Streaming Example: RealSense


## Streaming Example: IP Cameras
IP camera streaming currently supports:
* MJPEG over HTTP
* MJPEG over RTSP
* H264 over RTSP

If you know the streaming URL for your camera, you can use a "generic IP camera":
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

APIs for some specific camera manufacturers are supported out-of-the-box:
* Axis
* Mobotix
To stream from these cameras, just use the corresponding `sink_type`:
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

sink-example-mobotix-via-rtsp = {
  sink_type = "mobotix";
  host = "<IP/HOSTNAME>";
  user = "<USER>";
  password = "<PASSWORD>";
  
  application_protocol = "rtsp";
  encoding = "h264";
  transport_protocol = "tcp";
};
```

