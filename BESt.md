# BESt - Best Effort Streaming
## Streaming Examples: IP Cameras (Axis)
```c++
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

