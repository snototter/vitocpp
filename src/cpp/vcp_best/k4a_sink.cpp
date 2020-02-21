#include "k4a_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <limits>
#include <malloc.h>


#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

#ifdef VCP_BEST_WITH_K4A_MJPG
    #if CV_VERSION_MAJOR < 3
        #include <opencv2/highgui/highgui.hpp>
    #else
        #include <opencv2/highgui.hpp>
    #endif
#endif

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/timing_utils.h>

//Calibration: https://github.com/microsoft/Azure_Kinect_ROS_Driver/search?q=K4ACalibrationTransformData&unscoped_q=K4ACalibrationTransformData
/////FIXME k4a may need to wait upon opening multiple devices: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/676 ==> start sequentially, use parallel
//FIXME k4a https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/803  <== aligning two kinects fails (inaccurate factory calib)
//FIXME k4a check official calib doc: https://docs.microsoft.com/en-us/azure/kinect-dk/use-calibration-functions
//////FIXME k4a calibration: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/calibration/main.cpp
//FIXME k4a opencv transformation example: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/opencv_compatibility
namespace vcp
{
namespace best
{
namespace k4a
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::k4a"

const std::string kEmptyK4ASerialNumber = "----";


#define VCP_K4A_STRING_TO_ENUM(str, O) if (str.compare(#O) == 0) { return O; }

k4a_wired_sync_mode_t StringToWiredSyncMode(const std::string &name)
{
  VCP_K4A_STRING_TO_ENUM(name, K4A_WIRED_SYNC_MODE_STANDALONE);
  VCP_K4A_STRING_TO_ENUM(name, K4A_WIRED_SYNC_MODE_MASTER);
  VCP_K4A_STRING_TO_ENUM(name, K4A_WIRED_SYNC_MODE_SUBORDINATE);
  VCP_ERROR("k4a_wired_sync_mode_t '" << name << "' is not yet mapped");
}

k4a_color_resolution_t StringToColorResolution(const std::string &name)
{
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_OFF);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_720P);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_1080P);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_1440P);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_1536P);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_2160P);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_RESOLUTION_3072P);
  VCP_ERROR("k4a_color_resolution_t '" << name << "' is not yet mapped");
}

k4a_depth_mode_t StringToDepthMode(const std::string &name)
{
  VCP_K4A_STRING_TO_ENUM(name, K4A_DEPTH_MODE_OFF);
  VCP_K4A_STRING_TO_ENUM(name, K4A_DEPTH_MODE_NFOV_2X2BINNED);
  VCP_K4A_STRING_TO_ENUM(name, K4A_DEPTH_MODE_NFOV_UNBINNED);
  VCP_K4A_STRING_TO_ENUM(name, K4A_DEPTH_MODE_WFOV_2X2BINNED);
  VCP_K4A_STRING_TO_ENUM(name, K4A_DEPTH_MODE_WFOV_UNBINNED);
  VCP_K4A_STRING_TO_ENUM(name, K4A_DEPTH_MODE_PASSIVE_IR);
  VCP_ERROR("k4a_depth_mode_t '" << name << "' is not yet mapped");
}

k4a_fps_t StringToFps(const std::string &name)
{
  VCP_K4A_STRING_TO_ENUM(name, K4A_FRAMES_PER_SECOND_5);
  VCP_K4A_STRING_TO_ENUM(name, K4A_FRAMES_PER_SECOND_15);
  VCP_K4A_STRING_TO_ENUM(name, K4A_FRAMES_PER_SECOND_30);
  VCP_ERROR("k4a_fps_t '" << name << "' is not yet mapped");
}

k4a_color_control_command_t StringToColorControlCommand(const std::string &name)
{
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE);
//  K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY // deprecated as of k4a 1.2.0
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_BRIGHTNESS);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_CONTRAST);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_SATURATION);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_SHARPNESS);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_WHITEBALANCE);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_GAIN);
  VCP_K4A_STRING_TO_ENUM(name, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY);
  VCP_ERROR("k4a_color_control_command_t '" << name << "' is not yet mapped");
}


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


bool GetDevice(k4a_device_t &device, const K4ASinkParams &params)
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
  }
  return false;
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
  K4ARGBDSink(const K4ASinkParams &params, std::unique_ptr<SinkBuffer> rgb_buffer,
              std::unique_ptr<SinkBuffer> depth_buffer,
              std::unique_ptr<SinkBuffer> ir_buffer) : StreamSink(),
    continue_capture_(false),
    rgb_queue_(std::move(rgb_buffer)),
    depth_queue_(std::move(depth_buffer)),
    ir_queue_(std::move(ir_buffer)),
    params_(params),
    rgb_stream_enabled_(false),
    depth_stream_enabled_(false),
    ir_stream_enabled_(false),
    k4a_device_(nullptr)
  {
    VCP_LOG_DEBUG("K4ARGBDSink::K4ARGBDSink()");
    available_ = 0;

    // The user could configure the sensor such that
    // color/depth streams are disabled. In such cases, we
    // will return empty matrices for the corresponding stream.
    rgb_stream_enabled_ = params_.IsColorStreamEnabled();
    depth_stream_enabled_ = params_.IsDepthStreamEnabled();
    ir_stream_enabled_ = params_.IsInfraredStreamEnabled();
  }

  virtual ~K4ARGBDSink()
  {
    VCP_LOG_DEBUG("K4ARGBDSink::~K4ARGBDSink()");
    CloseDevice();
  }

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("K4ARGBDSink::OpenDevice()");
    if (k4a_device_)
    {
      VCP_LOG_FAILURE("Device already opened!");
      return false;
    }
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening Kinect Azure device");

    if (!GetDevice(k4a_device_, params_)) // This call already issues a warning if no device is available
      return false;

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
    return true;
  }

  bool CloseDevice() override
  {
    StopStreaming();
    if (k4a_device_)
    {
      VCP_LOG_DEBUG("K4ARGBDSink::CloseDevice()");
      k4a_device_close(k4a_device_);
      k4a_device_ = nullptr;
    }
    return true;
  }


  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("K4ARGBDSink::StartStreaming()");
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
      VCP_LOG_DEBUG("K4ARGBDSink::StopStreaming()");
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Closing k4a receiver thread.");
      continue_capture_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("k4a receiver thread has terminated.");
    }
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> res;
    image_queue_mutex_.lock();
    if (rgb_stream_enabled_)
    {
      if (rgb_queue_->Empty())
      {
        res.push_back(cv::Mat());
      }
      else
      {
        res.push_back(rgb_queue_->Front().clone());
        rgb_queue_->PopFront();
      }
    }
    if (depth_stream_enabled_)
    {
      if (depth_queue_->Empty())
      {
        res.push_back(cv::Mat());
      }
      else
      {
        res.push_back(depth_queue_->Front().clone());
        depth_queue_->PopFront();
      }
    }
    if (ir_stream_enabled_)
    {
      if (ir_queue_->Empty())
      {
        res.push_back(cv::Mat());
      }
      else
      {
        res.push_back(ir_queue_->Front().clone());
        ir_queue_->PopFront();
      }
    }
    image_queue_mutex_.unlock();
    return res;
  }


  int IsDeviceAvailable() const override
  {
    return available_;
  }

  size_t NumStreams() const override
  {
    size_t num = 0;
    if (rgb_stream_enabled_)
      ++num;
    if (depth_stream_enabled_)
      ++num;
    if (ir_stream_enabled_)
      ++num;
    return num;
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    std::vector<std::string> labels;
    if (rgb_stream_enabled_)
      labels.push_back(params_.sink_label + "-rgb");
    if (depth_stream_enabled_)
      labels.push_back(params_.sink_label + "-depth");
    if (ir_stream_enabled_)
      labels.push_back(params_.sink_label + "-ir");
    return labels[stream_index];
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    std::vector<FrameType> types;
    if (rgb_stream_enabled_)
      types.push_back(FrameType::RGBD_IMAGE);
    if (depth_stream_enabled_)
      types.push_back(FrameType::RGBD_DEPTH);
    if (ir_stream_enabled_)
      types.push_back(FrameType::INFRARED);
    return types[stream_index];
  }


  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_;
  }


  size_t NumDevices() const override
  {
    return 1;
  }


  int IsFrameAvailable() const override
  {
    // Should only return true, if all enabled streams have a frame available.
    bool available = true;
    image_queue_mutex_.lock();
    if (rgb_stream_enabled_ && rgb_queue_->Empty())
      available = false;
    if (depth_stream_enabled_ && depth_queue_->Empty())
      available = false;
    if (ir_stream_enabled_ && ir_queue_->Empty())
      available = false;
    image_queue_mutex_.unlock();
    if (available)
      return 1;
    return 0;
  }

  size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    image_queue_mutex_.lock();
    if (rgb_stream_enabled_ && !rgb_queue_->Empty())
        ++num;
    if (depth_stream_enabled_ && !depth_queue_->Empty())
        ++num;
    if (ir_stream_enabled_ && !ir_queue_->Empty())
        ++num;
    image_queue_mutex_.unlock();
    return num;
  }

private:
  std::atomic<bool> continue_capture_;
  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> rgb_queue_;
  std::unique_ptr<SinkBuffer> depth_queue_;
  std::unique_ptr<SinkBuffer> ir_queue_;
  K4ASinkParams params_;
  std::string serial_number_;
  std::string calibration_file_;
  std::atomic<int> available_;
  bool rgb_stream_enabled_;
  bool depth_stream_enabled_;
  bool ir_stream_enabled_;

  k4a_device_t k4a_device_;

  void GetCalibration(k4a_calibration_t sensor_calibration)
  {
    //FIXME if depth or color is disabled, this won't work
    if (sensor_calibration.color_camera_calibration.intrinsics.type != K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY
        || sensor_calibration.depth_camera_calibration.intrinsics.type != K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY)
      VCP_ERROR("Currently, we only support the Brown/Conrady lens distortion model.");
// TODO see https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/8c6964fcc30827b476d6c18076e291fc22daa702/src/k4a_calibration_transform_data.cpp
    //getDepthCameraInfo(sensor_msgs::CameraInfo& camera_info)
    k4a_calibration_intrinsic_parameters_t ic = sensor_calibration.color_camera_calibration.intrinsics.parameters;
    const cv::Mat K_c = (cv::Mat_<double>(3, 3)
                   << ic.param.fx, 0.0, ic.param.cx,
                   0.0, ic.param.fy, ic.param.cy,
                   0.0, 0.0, 1.0);
    const cv::Mat D_c = (cv::Mat_<double>(8, 1)
                   << ic.param.k1, ic.param.k2, ic.param.p1, ic.param.p2, ic.param.k3, ic.param.k4, ic.param.k5, ic.param.k6);

    k4a_calibration_intrinsic_parameters_t id = sensor_calibration.depth_camera_calibration.intrinsics.parameters;
    const cv::Mat K_d = (cv::Mat_<double>(3, 3)
                   << id.param.fx, 0.0, id.param.cx,
                   0.0, id.param.fy, id.param.cy,
                   0.0, 0.0, 1.0);
    const cv::Mat D_d = (cv::Mat_<double>(8, 1)
                   << id.param.k1, id.param.k2, id.param.p1, id.param.p2, id.param.k3, id.param.k4, id.param.k5, id.param.k6);
    // To transform from a source to a target 3D coordinate system, use the parameters stored
    // under extrinsics[source][target].
    k4a_calibration_extrinsics_t e = sensor_calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
    cv::Mat R_d2c;
    if (params_.align_depth_to_color)
      R_d2c = (cv::Mat_<double>(3, 3) << e.rotation[0], e.rotation[1], e.rotation[2], e.rotation[3], e.rotation[4], e.rotation[5], e.rotation[6], e.rotation[7], e.rotation[8]);
    else
      R_d2c = cv::Mat::eye(3, 3, CV_64FC1);

    cv::Mat t_d2c;
    if (params_.align_depth_to_color)
      t_d2c = (cv::Mat_<double>(3, 1) << e.translation[0], e.translation[1], e.translation[2]);
    else
      t_d2c = cv::Mat::zeros(3, 1, CV_64FC1);

    VCP_LOG_FAILURE("TODO store calibration, test calibration:" << std::endl << "Rd2c: " << R_d2c << std::endl << "td2c: " << t_d2c);

    VCP_LOG_FAILURE("Distortion coefficients: " << std::endl << "C: " << D_c << std::endl << "D: " << D_d);
  }

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
      VCP_LOG_FAILURE("TODO FIXME: need to save calibration!");
      GetCalibration(sensor_calibration);
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
      cv::Mat cvrgb, cvdepth, cvir;
      k4a_image_t image = nullptr;

      // Probe for color image
      if (rgb_stream_enabled_)
      {
        image = k4a_capture_get_color_image(k4a_capture);
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
          VCP_LOG_WARNING("No color image received!");
        }
      }

      // Probe for a depth16 image
      if (depth_stream_enabled_)
      {
        image = k4a_capture_get_depth_image(k4a_capture);
        cvdepth = Extract16U(image, transformation, true, cvrgb);
      }

      // Probe for a IR16 image
      if (ir_stream_enabled_)
      {
        image = k4a_capture_get_ir_image(k4a_capture);
        cvir = Extract16U(image, transformation, false, cvrgb);
      }

      // Release capture
      k4a_capture_release(k4a_capture);
      k4a_capture = nullptr;

      const cv::Mat trgb = imutils::ApplyImageTransformations(cvrgb, params_.transforms);
      const cv::Mat tdepth = imutils::ApplyImageTransformations(cvdepth, params_.transforms);
      const cv::Mat tir = imutils::ApplyImageTransformations(cvir, params_.transforms);

      image_queue_mutex_.lock();
      if (rgb_stream_enabled_)
        rgb_queue_->PushBack(trgb.clone());
      if (depth_stream_enabled_)
        depth_queue_->PushBack(tdepth.clone());
      if (ir_stream_enabled_)
        ir_queue_->PushBack(tir.clone());
      image_queue_mutex_.unlock();
    }
    // Clean up
    if (k4a_capture)
      k4a_capture_release(k4a_capture);

    if (transformation)
      k4a_transformation_destroy(transformation);

    k4a_device_stop_cameras(k4a_device_);
  }


  // Currently as of libk4a-1.3, only depth can be warped to color (not the IR stream, although it's the
  // same uint16 pixel format...
  cv::Mat Extract16U(k4a_image_t &image, k4a_transformation_t &transformation, bool is_depth, const cv::Mat &cvrgb)
  {
    cv::Mat extracted;
    if (image != nullptr)
    {
      // OpenCV matrix header to point to the raw or warped depth data.
      cv::Mat tmp;

      k4a_image_t aligned_depth_image = NULL;
      if (is_depth && params_.align_depth_to_color) // Alignment only works for depth imag (libk4a-1.3)
      {
        // Official warping example:
        // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/transformation/main.cpp
        if (k4a_image_create(
                K4A_IMAGE_FORMAT_DEPTH16,
                cvrgb.cols, cvrgb.rows,
                cvrgb.cols * static_cast<int>(sizeof(uint16_t)), &aligned_depth_image)
              != K4A_RESULT_SUCCEEDED)
        {
          VCP_LOG_FAILURE("Cannot allocate k4a image buffer to warp " << (is_depth ? "depth" : "infrared") << " to color!");
        }
        else
        {
          if (k4a_transformation_depth_image_to_color_camera(transformation, image, aligned_depth_image)
              != K4A_RESULT_SUCCEEDED)
          {
            VCP_LOG_FAILURE("Cannot align k4a " << (is_depth ? "depth" : "infrared") << " image to color image!");
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
        tmp.convertTo(extracted, CV_64FC1, 0.001, 0.0);
      }
      else
      {
        extracted = tmp.clone();
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
      VCP_LOG_WARNING("No " << (is_depth ? "depth" : "infrared") << " data received!");
    }
    return extracted;
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


bool K4ASinkParams::IsColorStreamEnabled() const
{
  return this->color_resolution != K4A_COLOR_RESOLUTION_OFF;
}

bool K4ASinkParams::IsDepthStreamEnabled() const
{
  return this->depth_mode != K4A_DEPTH_MODE_OFF;
}

bool K4ASinkParams::IsInfraredStreamEnabled() const
{
  return this->enable_infrared_stream && IsDepthStreamEnabled();
}

bool K4ASinkParams::RequiresWiredSync() const
{
  return this->wired_sync_mode != K4A_WIRED_SYNC_MODE_STANDALONE;
}


std::ostream &operator<< (std::ostream &out, const K4AColorControlSetting &s)
{
  if (s.mode == K4A_COLOR_CONTROL_MODE_AUTO)
    out << k4a::ToStr(s.command) << ", " << k4a::ToStr(s.mode);
  else
    out << k4a::ToStr(s.command) << ", " << k4a::ToStr(s.mode) << ", " << s.value;
  return out;
}


K4ASinkParams K4ASinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  K4ASinkParams params(sink_params);
  // Set to kEmptyK4ASerialNumber (or empty string) to address the first connected RealSense device.
  params.serial_number = GetOptionalStringFromConfig(config, cam_param, "serial_number", kEmptyK4ASerialNumber);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "serial_number"), configured_keys.end());

  // Store path to calibration file (needed such that the sink knows where to save the calibration data).
  params.write_calibration = GetOptionalBoolFromConfig(config, cam_param, "write_calibration", false);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "write_calibration"), configured_keys.end());

  // Align depth to color stream?
  params.align_depth_to_color = GetOptionalBoolFromConfig(config, cam_param, "align_depth_to_color", true);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "align_depth_to_color"), configured_keys.end());

  // Timeout in milliseconds for a single k4a_device_get_capture() call
  params.capture_timeout_ms = static_cast<int32_t>(GetOptionalIntFromConfig(config, cam_param, "capture_timeout_ms", 0));
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "capture_timeout_ms"), configured_keys.end());

  // Color camera resolution mode
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "color_resolution"), configured_keys.end());
  const std::string cres = GetOptionalStringFromConfig(config, cam_param, "color_resolution", std::string());
  if (!cres.empty())
    params.color_resolution = k4a::StringToColorResolution(cres);
  // Sanity check: if RGB stream is disabled, we cannot align depth to RGB, obviously:
  if (!params.IsColorStreamEnabled() && params.align_depth_to_color)
  {
    VCP_LOG_FAILURE("Invalid K4A configuration: align_depth_to_color=true, but RGB stream is OFF - disabling alignment to continue.");
    params.align_depth_to_color = false;
  }

  // Depth mode
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_mode"), configured_keys.end());
  const std::string dm = GetOptionalStringFromConfig(config, cam_param, "depth_mode", std::string());
  if (!dm.empty())
    params.depth_mode = k4a::StringToDepthMode(dm);

  // Enable IR stream?
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "enable_infrared_stream"), configured_keys.end());
  params.enable_infrared_stream = GetOptionalBoolFromConfig(config, cam_param, "enable_infrared_stream", false);
  // Sanity check - IR requires enabled depth:
  if (!params.IsDepthStreamEnabled() && params.enable_infrared_stream)
  {
    VCP_LOG_FAILURE("Invalid K4A configuration: enable_infrared_stream=true, but depth stream is OFF - enabling depth stream to continue.");
    params.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  }

  // Target/desired frame rate
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());
  const std::string fps = GetOptionalStringFromConfig(config, cam_param, "fps", std::string());
  if (!fps.empty())
    params.camera_fps = k4a::StringToFps(fps);

  // Set true if you want to turn off the streaming LED
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "disable_streaming_indicator"), configured_keys.end());
  params.disable_streaming_indicator = GetOptionalBoolFromConfig(config, cam_param, "disable_streaming_indicator", false);

  // Desired delay between color and depth
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_delay_off_color_usec"), configured_keys.end());
  params.depth_delay_off_color_usec = GetOptionalIntFromConfig(config, cam_param, "depth_delay_off_color_usec", params.depth_delay_off_color_usec);

  // Return depth as CV_64F (in meters) or CV_16U (in millimeters).
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_in_meters"), configured_keys.end());
  params.depth_in_meters = GetOptionalBoolFromConfig(config, cam_param, "depth_in_meters", false);

  // External synchronization timing
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "subordinate_delay_off_master_usec"), configured_keys.end());
  params.subordinate_delay_off_master_usec = GetOptionalUnsignedIntFromConfig(config, cam_param, "subordinate_delay_off_master_usec", params.subordinate_delay_off_master_usec);

  // If true, k4a capture will skip readings with invalid color or depth
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "synchronized_images_only"), configured_keys.end());
  params.synchronized_images_only = GetOptionalBoolFromConfig(config, cam_param, "synchronized_images_only", true);

  // External sync mode: Standalone, master or subordinate
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "wired_sync_mode"), configured_keys.end());
  const std::string wsm = GetOptionalStringFromConfig(config, cam_param, "wired_sync_mode", std::string());
  if (!wsm.empty())
    params.wired_sync_mode = k4a::StringToWiredSyncMode(wsm);

  // Check, if there are color control commands to set:
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "color_control_auto"), configured_keys.end());
  const std::string k4a_cc_auto = cam_param + ".color_control_auto";
  if (config.SettingExists(k4a_cc_auto))
  {
    const std::vector<std::string> opts_auto = config.GetStringArray(k4a_cc_auto);
    for (const auto &oa : opts_auto)
    {
      K4AColorControlSetting s;
      s.command = k4a::StringToColorControlCommand(oa);
      s.mode = K4A_COLOR_CONTROL_MODE_AUTO;
      params.color_control_auto.push_back(s);
    }
  }
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "color_control_manual"), configured_keys.end());
  const std::string k4a_cc_manual = cam_param + ".color_control_manual";
  if (config.SettingExists(k4a_cc_manual))
  {
    std::vector<std::pair<std::string, int>> opts_manual = config.GetIntegerKeyValueList(k4a_cc_manual);
    for (const auto &om : opts_manual)
    {
      K4AColorControlSetting s;
      s.command = k4a::StringToColorControlCommand(om.first);
      s.mode = K4A_COLOR_CONTROL_MODE_MANUAL;
      s.value = static_cast<int32_t>(om.second);
      params.color_control_manual.push_back(s);
    }
  }

  // There's no "depth control" yet (except for the "depth mode" which we already set).

  WarnOfUnusedParameters(cam_param, configured_keys);

  return params;
}



std::unique_ptr<StreamSink> CreateBufferedK4ASink(const K4ASinkParams &params,
                                                  std::unique_ptr<SinkBuffer> rgb_buffer,
                                                  std::unique_ptr<SinkBuffer> depth_buffer,
                                                  std::unique_ptr<SinkBuffer> ir_buffer)
{
  return std::unique_ptr<k4a::K4ARGBDSink>(new k4a::K4ARGBDSink(params, std::move(rgb_buffer), std::move(depth_buffer), std::move(ir_buffer)));
}


bool IsK4A(const std::string &type_param)
{
  const std::string type = vcp::utils::string::Lower(type_param);

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

} // namespace k4a
} // namespace best
} // namespace vcp
