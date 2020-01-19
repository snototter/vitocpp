#include "k4a_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <limits>
#include <malloc.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//FIXME k4a VCP_BEST_DEBUG_FRAMERATE
#ifdef VCP_BEST_WITH_K4A_MJPG
    #include <opencv2/highgui/highgui.hpp>
#endif

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/timing_utils.h>

//FIXME k4a may need to wait upon opening multiple devices: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/676
//FIXME k4a https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/803  <== aligning two kinects fails (inaccurate factory calib)
//FIXME k4a check official calib doc: https://docs.microsoft.com/en-us/azure/kinect-dk/use-calibration-functions
//FIXME k4a calibration: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/calibration/main.cpp
//FIXME k4a opencv transformation example: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/opencv_compatibility
namespace vcp
{
namespace best
{
const std::string kEmptyK4ASerialNumber = "----";

namespace k4a
{
std::string GetSerialNumber(k4a_device_t dev)
{
  std::string sn = std::string();
  size_t serial_number_length = 0;

  // Query serial number length
  if (k4a_device_get_serialnum(dev, NULL, &serial_number_length) != K4A_BUFFER_RESULT_TOO_SMALL)
  {
    VCP_LOG_FAILURE("Cannot query K4A serial number length");
    return std::string();
  }

  char *serial_number = new (std::nothrow) char[serial_number_length];
  if (!serial_number)
  {
    VCP_LOG_FAILURE("Cannot allocate " << serial_number_length << " bytes for K4A serial number");
    return sn;
  }

  if (k4a_device_get_serialnum(dev, serial_number, &serial_number_length) != K4A_BUFFER_RESULT_SUCCEEDED)
  {
    VCP_LOG_FAILURE("Cannot retrieve K4A serial number length");
  }
  else
  {
    sn = std::string(serial_number);
  }
  delete[] serial_number;
  serial_number = NULL;
  return sn;
}


bool GetDevice(k4a_device_t &device, const K4AParams &params)
{
  if (params.serial_number.empty() || kEmptyK4ASerialNumber.compare(params.serial_number) == 0)
  {
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
      VCP_LOG_FAILURE("Cannot open default/first Kinect Azure!");
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    const std::vector<K4ADeviceInfo> dev_infos = ListK4ADevices(false);
    if (dev_infos.size() == 0)
    {
      VCP_LOG_FAILURE("No Kinect Azure connected!");
      return false;
    }

    for (uint32_t i = 0; i < dev_infos.size(); ++i)
    {
      if (params.serial_number.compare(dev_infos[i].serial_number) == 0)
      {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &device))
        {
          VCP_LOG_FAILURE("Failed to open Kinect Azure device with serial number '" << params.serial_number << "'");
          return false;
        }
        else
        {
          return true;
        }
      }
    }
    VCP_LOG_FAILURE("Cannot find the Kinect Azure device with serial number '" << params.serial_number << "'");
    return false;
  }
}

std::vector<k4a_color_control_command_t> GetAvailableColorControlCommands()
{
  return {
    K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
  //  K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY, deprecated as of k4a 1.2.0
    K4A_COLOR_CONTROL_BRIGHTNESS,
    K4A_COLOR_CONTROL_CONTRAST,
    K4A_COLOR_CONTROL_SATURATION,
    K4A_COLOR_CONTROL_SHARPNESS,
    K4A_COLOR_CONTROL_WHITEBALANCE,
    K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION,
    K4A_COLOR_CONTROL_GAIN,
    K4A_COLOR_CONTROL_POWERLINE_FREQUENCY
  };
}

#define _RETSTR_AeqB(a, b) if (a == b) return #b
std::string ToStr(const k4a_color_control_command_t &cmd)
{
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE);
//  K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY, deprecated as of k4a 1.2.0
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_BRIGHTNESS);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_CONTRAST);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_SATURATION);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_SHARPNESS);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_WHITEBALANCE);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_GAIN);
  _RETSTR_AeqB(cmd, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY);
  VCP_ERROR("k4a_color_control_command_t '" << cmd << "' not yet supported!");
}

std::string ToStr(const k4a_color_control_mode_t &mode)
{
  _RETSTR_AeqB(mode, K4A_COLOR_CONTROL_MODE_AUTO);
  _RETSTR_AeqB(mode, K4A_COLOR_CONTROL_MODE_MANUAL);
  VCP_ERROR("k4a_color_control_mode_t '" << mode << "' not yet supported!");
}

std::vector<K4AColorControlSetting> GetCurrentColorControlSettings(k4a_device_t dev)
{
  std::vector<K4AColorControlSetting> settings;
  for (const auto &cmd : GetAvailableColorControlCommands())
  {
    K4AColorControlSetting s;
    s.command = cmd;
    if (k4a_device_get_color_control(dev, cmd, &s.mode, &s.value) != K4A_RESULT_SUCCEEDED)
    {
      VCP_LOG_FAILURE("Cannot query color control setting for k4a_color_control_command_t '" << cmd << "'");
    }
    settings.push_back(s);
  }
  return settings;
}


void GetSyncJackStatus(k4a_device_t dev, bool &sync_in_connected, bool &sync_out_connected)
{
  if (k4a_device_get_sync_jack(dev, &sync_in_connected, &sync_out_connected) != K4A_RESULT_SUCCEEDED)
    VCP_LOG_FAILURE("Failed to query sync jack status");
}


//cv::Mat KFromIntrinsics(const rs2_intrinsics &intrinsics)
//{
//  cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
//  K.at<double>(0,0) = static_cast<double>(intrinsics.fx);
//  K.at<double>(1,1) = static_cast<double>(intrinsics.fy);
//  K.at<double>(0,2) = static_cast<double>(intrinsics.ppx);
//  K.at<double>(1,2) = static_cast<double>(intrinsics.ppy);
//  return K;
//}


//cv::Mat DFromIntrinsics(const rs2_intrinsics &intrinsics)
//{
//  cv::Mat D = cv::Mat::zeros(5, 1, CV_64FC1);
//  for (size_t i = 0; i < 5; ++i)
//    D.at<double>(i) = static_cast<double>(intrinsics.coeffs[i]);
//  return D;
//}


//cv::Mat RFromExtrinsics(const rs2_extrinsics &extrinsics)
//{
//  cv::Mat R(3, 3, CV_64FC1);
//  // RealSense stores R in column-major order, so we need to fill R:
//  // [0 3 6;
//  //  1 4 7;
//  //  2 5 8]
//  int arr_idx = 0;
//  for (int c = 0; c < 3; ++c)
//  {
//    for (int r = 0; r < 3; ++r)
//    {
//      R.at<double>(r, c) = static_cast<double>(extrinsics.rotation[arr_idx]);
//      ++arr_idx;
//    }
//  }
//  return R;
//}


//cv::Mat TFromExtrinsics(const rs2_extrinsics &extrinsics)
//{
//  cv::Mat T(3, 1, CV_64FC1);
//  for (int i = 0; i < 3; ++i)
//    T.at<double>(i) = static_cast<double>(extrinsics.translation[i]);
//  return T;
//}


//void DumpCalibration(rs2::pipeline_profile &profile, const std::string &filename, const bool align_depth_to_color, const float depth_scale)
//{
//  if (filename.empty())
//  {
//    PVT_ABORT("DumpCalibration() called with empty file name!");
//  }

//  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//  if (!fs.isOpened())
//  {
//    PVT_ABORT("Cannot open '" << filename << "' to store calibration!");
//  }

//  rs2::video_stream_profile depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
//  rs2::video_stream_profile rgb_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

//  const rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();
//  const rs2_extrinsics depth_extrinsics = depth_profile.get_extrinsics_to(rgb_profile);
//  const rs2_intrinsics rgb_intrinsics = rgb_profile.get_intrinsics();
//  const rs2_extrinsics rgb_extrinsics = rgb_profile.get_extrinsics_to(depth_profile);

//  const cv::Mat Krgb = KFromIntrinsics(rgb_intrinsics);
//  const cv::Mat Drgb = DFromIntrinsics(rgb_intrinsics);
//  const int rgb_width = rgb_intrinsics.width;
//  const int rgb_height = rgb_intrinsics.height;

//  const cv::Mat Kdepth = align_depth_to_color ? Krgb : KFromIntrinsics(depth_intrinsics);
//  const cv::Mat Ddepth = align_depth_to_color ? Drgb : DFromIntrinsics(depth_intrinsics);
//  const int depth_width = align_depth_to_color ? rgb_width : depth_intrinsics.width;
//  const int depth_height = align_depth_to_color ? rgb_height : depth_intrinsics.height;

//  const cv::Mat Rdepth2color = align_depth_to_color ? cv::Mat::eye(3, 3, CV_64FC1) : RFromExtrinsics(depth_extrinsics);
//  const cv::Mat Tdepth2color = align_depth_to_color ? cv::Mat::zeros(3, 1, CV_64FC1) : TFromExtrinsics(depth_extrinsics);
//  const cv::Mat Rcolor2depth = align_depth_to_color ? cv::Mat::eye(3, 3, CV_64FC1) : RFromExtrinsics(rgb_extrinsics);
//  const cv::Mat Tcolor2depth = align_depth_to_color ? cv::Mat::zeros(3, 1, CV_64FC1) : TFromExtrinsics(rgb_extrinsics);

//  fs << "K_rgb" << Krgb;
//  fs << "D_rgb" << Drgb;
//  fs << "width_rgb" << rgb_width;
//  fs << "height_rgb" << rgb_height;

//  fs << "K_depth" << Kdepth;
//  fs << "D_depth" << Ddepth;
//  fs << "width_depth" << depth_width;
//  fs << "height_depth" << depth_height;
//  fs << "depth_scale" << depth_scale;

//  fs << "R_depth2rgb" << Rdepth2color;
//  fs << "t_depth2rgb" << Tdepth2color;
//  fs << "R_rgb2depth" << Rcolor2depth;
//  fs << "t_rgb2depth" << Tcolor2depth;

//  fs << "type" << "rgbd";
//  fs.release();

//  PVT_LOG_INFO("RealSense calibration has been saved to '" << filename << "'. Change the camera configuration if you want to prevent overwriting this calibration during the next start.");
//}


std::vector<K4ADeviceInfo> ListK4ADevices(bool warn_if_no_devices)
{
  uint32_t num_devices = k4a_device_get_installed_count();

  std::vector<K4ADeviceInfo> infos;

  if (num_devices == 0)
  {
    if (warn_if_no_devices)
    {
      VCP_LOG_WARNING("No Kinect Azure device connected!");
    }
    return infos;
  }

  k4a_device_t dev = NULL;
  for (uint32_t i = 0; i < num_devices; ++i)
  {
    if (k4a_device_open(i, &dev) != K4A_RESULT_SUCCEEDED)
    {
      VCP_LOG_FAILURE("Cannot open Kinect Azure #" << i);
      continue;
    }

    const std::string serial_number = GetSerialNumber(dev);
    if (serial_number.empty())
    {
      VCP_LOG_FAILURE("Cannot query serial number for Kinect Azur #" << i);
    }
    else
    {
      K4ADeviceInfo info;
      info.serial_number = serial_number;
      info.name = "Kinect #" + vcp::utils::string::ToStr(i) + ": " + serial_number;
      infos.push_back(info);
    }

    k4a_device_close(dev);
    dev = NULL;
  }

  return infos;
}


class K4ARGBDSink : public StreamSink
{
public:
  K4ARGBDSink(const K4AParams &params, std::unique_ptr<SinkBuffer> rgb_buffer, std::unique_ptr<SinkBuffer> depth_buffer) : StreamSink(),
    continue_capture_(false),
    rgb_queue_(std::move(rgb_buffer)),
    depth_queue_(std::move(depth_buffer)),
    params_(params),
    rgb_stream_enabled_(false),
    depth_stream_enabled_(false),
    ir_stream_enabled_(false),
    k4a_device_(nullptr)
  {
    available_ = 0;
  }

  virtual ~K4ARGBDSink()
  {
    StopStreaming();
    CloseDevice();
  }

  bool OpenDevice() override
  {
    if (k4a_device_)
    {
      VCP_LOG_FAILURE("Device already opened!");
      return false;
    }
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening Kinect Azure device");

    if (!GetDevice(k4a_device_, params_))
    {
      VCP_LOG_FAILURE("Cannot open Kinect Azure device");
      return false;
    }

    serial_number_ = GetSerialNumber(k4a_device_);
    if (serial_number_.empty())
      return false;

    // Check if we need to dump the calibration:
    calibration_file_ = params_.write_calibration ? params_.calibration_file : "";
    if (params_.write_calibration && calibration_file_.empty())
    {
      VCP_LOG_FAILURE("If you want to dump the K4A calibration, you must specify the filename as calibration_file parameter!");
      return false;
    }

    // The user could configure the sensor such that
    // color/depth streams are disabled. In such cases, we
    // will return empty matrices for the corresponding stream.
    rgb_stream_enabled_ = params_.IsColorStreamEnabled();
    depth_stream_enabled_ = params_.IsDepthStreamEnabled();
    ir_stream_enabled_ = params_.IsInfraredStreamEnabled();
    return true;
  }

  bool CloseDevice() override
  {
    if (k4a_device_)
    {
      k4a_device_close(k4a_device_);
      k4a_device_ = nullptr;
    }
    return true;
  }


  bool StartStreaming() override
  {
    if (continue_capture_)
    {
      VCP_LOG_FAILURE("K4A stream already running - ignoring StartStreaming() call.");
      return false;
    }

    continue_capture_ = true;
    stream_thread_ = std::thread(&K4ARGBDSink::Receive, this);
    return true;
  }


  bool StopStreaming() override
  {
    if (continue_capture_)
    {
      continue_capture_ = false;
      stream_thread_.join();
    }
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    //FIXME k4a IR16 ?
    cv::Mat rgb, depth;
    image_queue_mutex_.lock();
    if (rgb_queue_->Empty() || depth_queue_->Empty())
    {
      rgb = cv::Mat();
      depth = cv::Mat();
    }
    else
    {
      // Retrieve oldest image in queue.
      rgb = rgb_queue_->Front().clone();
      rgb_queue_->PopFront();
      depth = depth_queue_->Front().clone();
      depth_queue_->PopFront();
    }
    image_queue_mutex_.unlock();
    std::vector<cv::Mat> res;
    res.push_back(rgb);
    res.push_back(depth);
    return res;
  }


  int IsDeviceAvailable() const override
  {
    return available_;
  }

  size_t NumStreams() const override
  {
    return 2;
    //FIXME k4a IR?
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    switch (stream_index)
    {
      case 0:
        return params_.sink_label + "-rgb";
      case 1:
        return params_.sink_label + "-depth";
      case 2:
      default:
        VCP_ERROR("Stream #" << stream_index << " does not exist for K4A '" << params_.sink_label << "', S/N " << params_.serial_number);
    //FIXME k4a IR?
    }
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    switch (stream_index)
    {
      case 0:
        return FrameType::RGBD_IMAGE;
      case 1:
        return FrameType::RGBD_DEPTH;
      case 2:
      default:
        VCP_ERROR("Stream #" << stream_index << " does not exist for K4A '" << params_.sink_label << "', S/N " << params_.serial_number);
    //FIXME k4a IR?
    }
  }


  int IsFrameAvailable() const override
  {
    //FIXME k4a IR?  a) add IR, b) true if ALL or ANY are available?
    image_queue_mutex_.lock();
    const bool empty = rgb_queue_->Empty() || depth_queue_->Empty();
    image_queue_mutex_.unlock();
    if (empty)
      return 0;
    return 1;
  }

private:
  std::atomic<bool> continue_capture_;
  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> rgb_queue_;
  std::unique_ptr<SinkBuffer> depth_queue_;
  K4AParams params_;
  std::string serial_number_;
  std::string calibration_file_;
  std::atomic<int> available_;
  bool rgb_stream_enabled_;
  bool depth_stream_enabled_;
  bool ir_stream_enabled_;

  k4a_device_t k4a_device_;

  void Receive()
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting K4A stream from device [" << serial_number_ << "]");

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = params_.camera_fps;
#ifdef VCP_BEST_WITH_K4A_MJPG
    // Save bandwidth, but need to decode JPGs on-the-fly.
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Configuring K4A color stream as MJPG.");
#else // VCP_BEST_WITH_K4A_MJPG
    // Use already decoded image data.
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Configuring K4A color stream as BGRA32.");
#endif // VCP_BEST_WITH_K4A_MJPG
    config.color_resolution = params_.color_resolution;
    config.depth_delay_off_color_usec = params_.depth_delay_off_color_usec;
    config.depth_mode = params_.depth_mode;
    config.disable_streaming_indicator = params_.disable_streaming_indicator;
    config.subordinate_delay_off_master_usec = params_.subordinate_delay_off_master_usec;
    config.synchronized_images_only = params_.synchronized_images_only;
    config.wired_sync_mode = params_.wired_sync_mode;


    // After setting the configuration, we can already query sensor information (since
    // the device is already opened).
    // Query device calibration
    k4a_calibration_t sensor_calibration;
    if (k4a_device_get_calibration(k4a_device_, config.depth_mode, config.color_resolution, &sensor_calibration) != K4A_RESULT_SUCCEEDED)
      VCP_ERROR("Failed to retrieve sensor calibration!");

    // Prepare transformation for image alignment if needed.
    k4a_transformation_t transformation = nullptr;
    if (params_.align_depth_to_color)
      transformation = k4a_transformation_create(&sensor_calibration);

    // Save calibration if requested.
    if (params_.write_calibration)
    {
      //TODO FIXME!!
      VCP_LOG_FAILURE("TODO FIXME: need to save calibration!");
      //TODO look into:
      // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/undistort/main.cpp
      // or even better:
      // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/main.cpp
      // calibration_to_color_camera_matrix
    }

    // Set color camera configuration:
    SetColorControl();

    // Check synchronisation status
    //TODO only relevant if we're in a multi-camera setup
    if (params_.verbose)
    {
      bool sync_in_connected, sync_out_connected;
      GetSyncJackStatus(k4a_device_, sync_in_connected, sync_out_connected);
      VCP_LOG_INFO_DEFAULT("K4A sync jack status" << std::endl
                          << "  IN:  " << (sync_in_connected ? "connected" : "not connected") << std::endl
                          << "  OUT: " << (sync_out_connected ? "connected" : "not connected"));
    }

    // In contrast to realsense devices, the camera streams can be started *after*
    // querying sensor calibration, etc.
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(k4a_device_, &config))
    {
      if (k4a_device_)
      {
        k4a_device_close(k4a_device_);
        k4a_device_ = NULL;
      }
      VCP_ERROR("Failed to start k4a device with S/N '" << serial_number_ << "'");
    }

    // From my K4A tests it seems that the color stream takes notably longer to start
    // than the depth/IR stream. Hence the first k4a_device_get_capture() calls return
    // only depth/IR, but no color data.
    // The naive solution: skip "k4a captures" during start up until we get data for both
    // color and depth.
    // FIXME remove and just deliver empty frames - synced depth + color can be enforced via
    // config
    bool skip_initially_missing = true;
    // TODO: Alternatively, we might replace the missing color stream by IR16 instead (but
    // this would probably lead to exceptions on the pvt user's side (as s/he might expect
    // rgb/uint8 color images or None instead of changing between uint16 and uint8 types).

    // Now this sink is ready to publish images.
    available_ = 1;

    k4a_capture_t k4a_capture;
    while(continue_capture_)
    {
      switch (k4a_device_get_capture(k4a_device_, &k4a_capture, params_.capture_timeout_ms))
      {
      case K4A_WAIT_RESULT_SUCCEEDED:
          break;
      case K4A_WAIT_RESULT_TIMEOUT:
          VCP_LOG_WARNING("Capture request to K4A S/N '" << serial_number_ << "' timed out.");
          continue;
          break;
      case K4A_WAIT_RESULT_FAILED:
          VCP_LOG_FAILURE("Failed to read a capture from K4A S/N '" << serial_number_ << "', closing stream.");
          continue_capture_ = false;
          continue;
          break;
      }

      // Probe for color image
      k4a_image_t image = k4a_capture_get_color_image(k4a_capture);
      cv::Mat cvrgb;
      if (image)
      {
        // Image conversion based on https://stackoverflow.com/a/57222191/400948
        // Get image buffer and size
        uint8_t* buffer = k4a_image_get_buffer(image);
        const int rows = k4a_image_get_height_pixels(image);
        const int cols = k4a_image_get_width_pixels(image);

        // Create OpenCV Mat header pointing to the buffer (no copy yet!)
        cv::Mat buf(rows, cols, CV_8UC4, static_cast<void*>(buffer), cv::Mat::AUTO_STEP);
#ifdef VCP_BEST_WITH_K4A_MJPG
        // Stream is JPG encoded.
        // Note that decoding a stream into a 3840x2160x3 image takes ~40 ms!
        cvrgb = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
#else
        // Stream is BGRA32.
        // Converting a 3840x2160x4 image to 3-channels only takes ~2 ms.
        // We need to drop the alpha channel anyways, so use cvtColor to make
        // the deep buffer copy:
        if (params_.color_as_bgr)
          cv::cvtColor(buf, cvrgb, CV_BGRA2BGR);
        else
          cv::cvtColor(buf, cvrgb, CV_BGRA2RGB);
#endif

        // Now it's safe to free the memory
        k4a_image_release(image);
        image = NULL;
      }
      else
      {
        if (rgb_stream_enabled_)
        {
          VCP_LOG_WARNING("No color image received!");
        }
      }

      //FIXME k4a IR16 if depth is enabled
//      // probe for a IR16 image
//      image = k4a_capture_get_ir_image(k4a_capture);
//      if (image != NULL)
//      {
//        PVT_LOG_INFO_NOFILE("IR16 received: " << k4a_image_get_width_pixels(image) << "x" << k4a_image_get_height_pixels(image));
//        k4a_image_release(image);
//      }

      // Probe for a depth16 image
      image = k4a_capture_get_depth_image(k4a_capture);
      cv::Mat cvdepth;
      if (image != NULL)
      {
        // OpenCV matrix header to point to the raw or warped depth data.
        cv::Mat tmp;

        k4a_image_t aligned_depth_image = NULL;
        if (params_.align_depth_to_color)
        {
          // Official warping example:
          // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/transformation/main.cpp
          if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, cvrgb.cols, cvrgb.rows,
                cvrgb.cols * static_cast<int>(sizeof(uint16_t)), &aligned_depth_image)
                != K4A_RESULT_SUCCEEDED)
          {
            VCP_LOG_FAILURE("Cannot allocate k4a image buffer to warp depth to color!");
          }
          else
          {
            if (k4a_transformation_depth_image_to_color_camera(transformation, image, aligned_depth_image)
                != K4A_RESULT_SUCCEEDED)
            {
              VCP_LOG_FAILURE("Cannot align k4a depth image to color image!");
            }
            else
            {
              // Get image buffer and size
              uint8_t* buffer = k4a_image_get_buffer(aligned_depth_image);
              const int rows = k4a_image_get_height_pixels(aligned_depth_image);
              const int cols = k4a_image_get_width_pixels(aligned_depth_image);
              // Create OpenCV Mat header pointing to the buffer (no copy yet!)
              tmp = cv::Mat(rows, cols, CV_16U, static_cast<void*>(buffer), k4a_image_get_stride_bytes(aligned_depth_image));
            }
          }
        }
        else
        {
          // Official depth conversion:
          // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/main.cpp

          // Get image buffer and size
          uint8_t* buffer = k4a_image_get_buffer(image);
          const int rows = k4a_image_get_height_pixels(image);
          const int cols = k4a_image_get_width_pixels(image);
          // Create OpenCV Mat header pointing to the buffer (no copy yet!)
          tmp = cv::Mat(rows, cols, CV_16U, static_cast<void*>(buffer), k4a_image_get_stride_bytes(image));
        }

        // Deep copy:
        if (params_.depth_in_meters)
        {
          tmp.convertTo(cvdepth, CV_64FC1, 0.001, 0.0);
        }
        else
        {
          cvdepth = tmp.clone();
        }
        // Now it's safe to free the memory
        k4a_image_release(image);
        image = NULL;
        if (aligned_depth_image)
        {
          k4a_image_release(aligned_depth_image);
          aligned_depth_image = NULL;
        }
      }
      else
      {
        if (depth_stream_enabled_)
        {
          VCP_LOG_WARNING("No depth data received!");
        }
      }

      // Release capture
      k4a_capture_release(k4a_capture);
      k4a_capture = NULL;

      if ((rgb_stream_enabled_&& cvrgb.empty()) ||
          (depth_stream_enabled_ && cvdepth.empty()))
      {
        // Only skip incomplete captures during startup
        if (!skip_initially_missing)
        {
          VCP_LOG_WARNING("TODO RGB or depth is empty - increase error counter and abort if we cannot get a valid capture!");
          //TODO FIXME
        }
      }
      else
      {
        skip_initially_missing = false;

        //TODO convert depth readings to meters, if sensor has a depth-scale!
        //            if (rgbd_params_.depth_in_meters)
        //              cvdepth = FrameToDepth(depth, depth_size, depth_scale);
        //            else
        //              cvdepth = FrameToMat(depth, depth_size);
        image_queue_mutex_.lock();
        if (!cvrgb.empty() || !rgb_stream_enabled_)
          rgb_queue_->PushBack(cvrgb.clone());
        if (!cvdepth.empty() || !depth_stream_enabled_)
          depth_queue_->PushBack(cvdepth.clone());
        image_queue_mutex_.unlock();
      }
    }
    // Clean up
    if (k4a_capture)
      k4a_capture_release(k4a_capture);

    if (transformation)
      k4a_transformation_destroy(transformation);

    k4a_device_stop_cameras(k4a_device_);
  }

  void SetColorControl()
  {
    // Set color camera configuration:
    // - first, all which should be set to AUTO
    for (const auto &p : params_.color_control_auto)
    {
      if (k4a_device_set_color_control(k4a_device_, p.command, K4A_COLOR_CONTROL_MODE_AUTO, 0) != K4A_RESULT_SUCCEEDED)
      {
        VCP_LOG_FAILURE("Cannot adjust K4A color control setting: " << p);
      }
      else
      {
        if (params_.verbose)
          VCP_LOG_INFO_DEFAULT("Changed K4A color control: " << p);
      }
    }
    // - second, all which should be set to MANUAL
    for (const auto &p : params_.color_control_manual)
    {
      if (k4a_device_set_color_control(k4a_device_, p.command, K4A_COLOR_CONTROL_MODE_MANUAL, p.value) != K4A_RESULT_SUCCEEDED)
      {
        VCP_LOG_FAILURE("Cannot adjust K4A color control setting: " << p);
      }
      else
      {
        if (params_.verbose)
          VCP_LOG_INFO_DEFAULT("Changed K4A color control: " << p);
      }
    }

    // Finally, query current color camera configuration:
    if (params_.verbose)
    {
      for (const auto &p : GetCurrentColorControlSettings(k4a_device_))
        VCP_LOG_INFO_DEFAULT("K4A current color control settings: " << p);
    }
  }
};
} // namespace k4a

bool K4AParams::IsColorStreamEnabled() const
{
  return this->color_resolution != K4A_COLOR_RESOLUTION_OFF;
}

bool K4AParams::IsDepthStreamEnabled() const
{
  return this->depth_mode != K4A_DEPTH_MODE_OFF;
}

bool K4AParams::IsInfraredStreamEnabled() const
{
  return this->enable_infrared_stream && IsDepthStreamEnabled();
}


std::ostream &operator<< (std::ostream &out, const K4AColorControlSetting &s)
{
  if (s.mode == K4A_COLOR_CONTROL_MODE_AUTO)
    out << k4a::ToStr(s.command) << ", " << k4a::ToStr(s.mode);
  else
    out << k4a::ToStr(s.command) << ", " << k4a::ToStr(s.mode) << ", " << s.value;
  return out;
}


std::unique_ptr<StreamSink> CreateBufferedK4ASink(const K4AParams &params, std::unique_ptr<SinkBuffer> rgb_buffer, std::unique_ptr<SinkBuffer> depth_buffer)
{
  return std::unique_ptr<k4a::K4ARGBDSink>(new k4a::K4ARGBDSink(params, std::move(rgb_buffer), std::move(depth_buffer)));
}


std::vector<K4ADeviceInfo> ListK4ADevices(bool warn_if_no_devices)
{
  return k4a::ListK4ADevices(warn_if_no_devices);
}


bool IsK4A(const std::string &type_param)
{
  std::string type(type_param);
  vcp::utils::string::ToLower(type);
  if (type.compare("k4a") == 0
    || type.compare("kinect-azure") == 0
    || type.compare("azure-kinect") == 0
    || type.compare("kinect_azure") == 0
    || type.compare("azure_kinect") == 0
    || type.compare("kinectazure") == 0
    || type.compare("azurekinect") == 0)
  {
    return true;
  }
  return false;
}

} // namespace best
} // namespace vcp
