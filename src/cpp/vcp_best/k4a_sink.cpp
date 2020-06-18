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

#ifdef VCP_BEST_DEBUG_FRAMERATE
    #include <chrono>
    #include <iomanip>
#endif // VCP_BEST_DEBUG_FRAMERATE

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_utils/timing_utils.h>

// Documents you should be aware of before blindly refactoring this ball of mud:
// Official k4a calibration doc:
//  https://docs.microsoft.com/en-us/azure/kinect-dk/use-calibration-functions
// Calib & transformation in ROS:
//  https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/8c6964fcc30827b476d6c18076e291fc22daa702/src/k4a_calibration_transform_data.cpp
// Check https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/803  <== Issues when aligning two kinects (inaccurate factory calib)
namespace vcp
{
namespace best
{
namespace k4a
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::k4a"

//TODO Nice-to-have would be either CMake options (spams the build system) or libconfig (way more complex - how do we introduce
// parameters that belong to a multi-k4a sink without introducing another sink?). I'd prefer the CMake way.
/** @brief Minimum time (in microseconds) between two subsequent depth image captures (in a multi-k4a setting). */
constexpr int32_t TIME_BETWEEN_DEPTH_CAPTURES_USEC = 160;

/** @brief Maximum time (in milliseconds) to wait for a synchronized capture (in a multi-k4a setting). */
constexpr int64_t WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT = 60000;

/** @brief Maximum tolerated difference between an image's expected timestamp and the time it actually occured. */
constexpr std::chrono::microseconds MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP(100);



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
  const std::vector<K4ADeviceInfo> dev_infos = ListK4ADevices(false);
  if (dev_infos.size() == 0)
  {
    VCP_LOG_FAILURE("No Kinect Azure connected!");
    return false;
  }
  // If no serial number is provided, open the first available K4A
  if (params.serial_number.empty() || kEmptyK4ASerialNumber.compare(params.serial_number) == 0)
  {
    // K4A_DEVICE_DEFAULT would always try to open device #0 (which
    // obviously works only once).
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(dev_infos[0].device_number, &device))
    {
      VCP_LOG_FAILURE("Cannot open first available Kinect Azure (device number " << dev_infos[0].device_number << ")!");
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    for (uint32_t i = 0; i < dev_infos.size(); ++i)
    {
      if (params.serial_number.compare(dev_infos[i].serial_number) == 0)
      {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(dev_infos[i].device_number, &device))
        {
          VCP_LOG_FAILURE("Failed to open Kinect Azure device with serial number '" << params.serial_number << "', device number " << dev_infos[i].device_number << ".");
          return false;
        }
        else
        {
          return true;
        }
      }
    }
    VCP_LOG_FAILURE("Cannot find the Kinect Azure device with serial number '" << params.serial_number << "'.");
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


std::vector<K4ADeviceInfo> ListK4ADevices(bool warn_if_no_devices)
{
  uint32_t num_devices = k4a_device_get_installed_count();

  std::vector<K4ADeviceInfo> infos;

  if (num_devices == 0)
  {
    if (warn_if_no_devices)
      VCP_LOG_WARNING("No Kinect Azure device connected!");
    return infos;
  }

  k4a_device_t dev = NULL;
  for (uint32_t i = 0; i < num_devices; ++i)
  {
    if (k4a_device_open(i, &dev) != K4A_RESULT_SUCCEEDED)
    {
      VCP_LOG_WARNING("Kinect Azure #" << i << " is busy.");
      continue;
    }
    const std::string serial_number = GetSerialNumber(dev);
    if (serial_number.empty())
    {
      VCP_LOG_FAILURE("Cannot query serial number for Kinect Azure #" << i);
    }
    else
    {
      K4ADeviceInfo info;
      info.device_number = i;
      info.serial_number = serial_number;
      info.name = "Kinect #" + vcp::utils::string::ToStr(i) + ": " + serial_number;
      infos.push_back(info);
    }
    k4a_device_close(dev);
    dev = NULL;
  }

  return infos;
}


cv::Mat KFromIntrinsics(const k4a_calibration_intrinsic_parameters_t &intrinsics)
{
  cv::Mat K = (cv::Mat_<double>(3, 3)
                       << intrinsics.param.fx, 0.0, intrinsics.param.cx,
                       0.0, intrinsics.param.fy, intrinsics.param.cy,
                       0.0, 0.0, 1.0);
  return K;
}

cv::Mat DFromIntrinsics(const k4a_calibration_intrinsic_parameters_t &intrinsics)
{
  cv::Mat D = (cv::Mat_<double>(8, 1)
               << intrinsics.param.k1, intrinsics.param.k2,
               intrinsics.param.p1, intrinsics.param.p2,
               intrinsics.param.k3, intrinsics.param.k4,
               intrinsics.param.k5, intrinsics.param.k6);
  return D;
}


void DumpCalibration(const K4ASinkParams &params, k4a_calibration_t sensor_calibration)
{
  if (sensor_calibration.color_camera_calibration.intrinsics.type != K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY
      || sensor_calibration.depth_camera_calibration.intrinsics.type != K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY)
    VCP_ERROR("Currently, we only support the Brown/Conrady lens distortion model.");

  const bool align_d2c = params.IsColorStreamEnabled() && params.IsDepthStreamEnabled() && params.align_depth_to_color;

  cv::Mat Krgb, Drgb;
  int width_rgb, height_rgb;

  // Write to file:
  if (params.calibration_file.empty())
    VCP_ERROR("DumpCalibration() called with empty file name!");

  cv::FileStorage fs(params.calibration_file, cv::FileStorage::WRITE);
  if (!fs.isOpened())
    VCP_ERROR("Cannot open '" << params.calibration_file << "' to store K4A calibration!");

  if (!params.serial_number.empty() && kEmptyK4ASerialNumber.compare(params.serial_number) != 0)
    fs << "serial_number" << params.serial_number;

  if (params.IsColorStreamEnabled())
  {
    const k4a_calibration_camera_t calib_color = sensor_calibration.color_camera_calibration;
    const k4a_calibration_intrinsic_parameters_t ic = sensor_calibration.color_camera_calibration.intrinsics.parameters;
    Krgb = KFromIntrinsics(ic);
    Drgb = DFromIntrinsics(ic);
    width_rgb = calib_color.resolution_width;
    height_rgb = calib_color.resolution_height;
    fs << "M_color" << Krgb;
    fs << "D_color" << Drgb;
    fs << "width_color" << width_rgb;
    fs << "height_color" << height_rgb;

//      if (params_.IsDepthStreamEnabled())
//      {
//        rs2::video_stream_profile depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
//        const rs2_extrinsics rgb_extrinsics = rgb_profile.get_extrinsics_to(depth_profile);
//        const cv::Mat Rcolor2depth = align_d2c ? cv::Mat::eye(3, 3, CV_64FC1) : RFromExtrinsics(rgb_extrinsics);
//        const cv::Mat Tcolor2depth = align_d2c ? cv::Mat::zeros(3, 1, CV_64FC1) : TFromExtrinsics(rgb_extrinsics);
//        fs << "R_rgb2depth" << Rcolor2depth;
//        fs << "t_rgb2depth" << Tcolor2depth;
//      }
  }

  if (params.IsDepthStreamEnabled())
  {
    // Depth intrinsics
    const k4a_calibration_intrinsic_parameters_t id = sensor_calibration.depth_camera_calibration.intrinsics.parameters;
    const cv::Mat Kdepth = align_d2c ? Krgb : KFromIntrinsics(id);
    const cv::Mat Ddepth = align_d2c ? Drgb : DFromIntrinsics(id);
    const int width_depth = align_d2c ? width_rgb : sensor_calibration.depth_camera_calibration.resolution_width;
    const int height_depth = align_d2c ? height_rgb : sensor_calibration.depth_camera_calibration.resolution_height;

    fs << "M_depth" << Kdepth;
    fs << "D_depth" << Ddepth;
    fs << "width_depth" << width_depth;
    fs << "height_depth" << height_depth;

    if (params.IsColorStreamEnabled())
    {
      // To transform from a source to a target 3D coordinate system, use the parameters stored
      // at extrinsics[source][target].
      k4a_calibration_extrinsics_t e = sensor_calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
      cv::Mat R_d2c;
      if (params.align_depth_to_color)
        R_d2c = cv::Mat::eye(3, 3, CV_64FC1);
      else
        R_d2c = (cv::Mat_<double>(3, 3) << e.rotation[0], e.rotation[1], e.rotation[2], e.rotation[3], e.rotation[4], e.rotation[5], e.rotation[6], e.rotation[7], e.rotation[8]);

      cv::Mat t_d2c;
      if (params.align_depth_to_color)
        t_d2c = cv::Mat::zeros(3, 1, CV_64FC1);
      else
        t_d2c = (cv::Mat_<double>(3, 1) << e.translation[0], e.translation[1], e.translation[2]);

      fs << "R_depth2color" << R_d2c;
      fs << "t_depth2color" << t_d2c;
    }
  }

  if (params.IsInfraredStreamEnabled())
  {
    // IR stream has the depth intrinsics (but cannot be aligned to color camera, at least with k4a-v1.3)
    const k4a_calibration_intrinsic_parameters_t id = sensor_calibration.depth_camera_calibration.intrinsics.parameters;
    const cv::Mat Kir = KFromIntrinsics(id);
    const cv::Mat Dir = DFromIntrinsics(id);
    const int width_ir = sensor_calibration.depth_camera_calibration.resolution_width;
    const int height_ir = sensor_calibration.depth_camera_calibration.resolution_height;

    fs << "K_ir" << Kir;
    fs << "D_ir" << Dir;
    fs << "width_ir" << width_ir;
    fs << "height_ir" << height_ir;

    if (params.IsColorStreamEnabled())
    {
      // To transform from a source to a target 3D coordinate system, use the parameters stored
      // at extrinsics[source][target].
      k4a_calibration_extrinsics_t e = sensor_calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
      cv::Mat R_i2c = (cv::Mat_<double>(3, 3) << e.rotation[0], e.rotation[1], e.rotation[2], e.rotation[3], e.rotation[4], e.rotation[5], e.rotation[6], e.rotation[7], e.rotation[8]);
      cv::Mat t_i2c = (cv::Mat_<double>(3, 1) << e.translation[0], e.translation[1], e.translation[2]);
      fs << "R_ir2color" << R_i2c;
      fs << "t_ir2color" << t_i2c;
    }
  }

  fs << "sink_type" << SinkTypeToString(params.sink_type);
  fs << "label" << params.sink_label;

  fs << "type" << "rgbd";
  fs.release();

  if (params.verbose)
    VCP_LOG_INFO("Calibration for K4A '" << params.serial_number << "', sink '" << params.sink_label << "' has been saved to '" << params.calibration_file << "'."
                 << std::endl << "          Change the camera's 'calibration_file' parameter if you want to prevent overwriting it upon the next start.");
}


void SetColorControl(const K4ASinkParams &params, k4a_device_t k4a_device)
{
  // Set color camera configuration:
  // - first, all which should be set to AUTO
  for (const auto &p : params.color_control_auto)
  {
    if (k4a_device_set_color_control(k4a_device, p.command, K4A_COLOR_CONTROL_MODE_AUTO, 0) != K4A_RESULT_SUCCEEDED)
    {
      VCP_LOG_FAILURE("Cannot adjust K4A '" << params.serial_number << "', sink '" << params.sink_label << "' color control setting: " << p);
    }
    else
    {
      if (params.verbose)
        VCP_LOG_INFO_DEFAULT("Changed K4A '" << params.serial_number << "', sink '" << params.sink_label << "' color control: " << p);
    }
  }
  // - second, all which should be set to MANUAL
  for (const auto &p : params.color_control_manual)
  {
    if (k4a_device_set_color_control(k4a_device, p.command, K4A_COLOR_CONTROL_MODE_MANUAL, p.value) != K4A_RESULT_SUCCEEDED)
    {
      VCP_LOG_FAILURE("Cannot adjust K4A '" << params.serial_number << "', sink '" << params.sink_label << "' color control setting: " << p);
    }
    else
    {
      if (params.verbose)
        VCP_LOG_INFO_DEFAULT("Changed K4A '" << params.serial_number << "', sink '" << params.sink_label << "' color control: " << p);
    }
  }

  // Finally, query current color camera configuration:
  if (params.verbose)
  {
    for (const auto &p : GetCurrentColorControlSettings(k4a_device))
      VCP_LOG_INFO_DEFAULT("k4a current color control settings: " << p);
  }
}


// Currently as of libk4a-1.3, only depth can be warped to color (not the IR stream, although it's the
// same uint16 pixel format...
cv::Mat Extract16U(const K4ASinkParams &params, k4a_image_t &image, k4a_transformation_t &transformation, bool is_depth, const cv::Mat &cvrgb)
{
  cv::Mat extracted;
  if (image != nullptr)
  {
    // OpenCV matrix header to point to the raw or warped depth data.
    cv::Mat tmp;

    k4a_image_t aligned_depth_image = NULL;
    if (is_depth && params.align_depth_to_color) // Alignment only works for depth image (libk4a-1.3)
    {
      // Official warping example:
      // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/transformation/main.cpp
      if (k4a_image_create(
              K4A_IMAGE_FORMAT_DEPTH16,
              cvrgb.cols, cvrgb.rows,
              cvrgb.cols * static_cast<int>(sizeof(uint16_t)), &aligned_depth_image)
            != K4A_RESULT_SUCCEEDED)
      {
        VCP_LOG_FAILURE("Cannot allocate K4A image buffer to warp " << (is_depth ? "depth" : "infrared") << " to color!");
      }
      else
      {
        if (k4a_transformation_depth_image_to_color_camera(transformation, image, aligned_depth_image)
            != K4A_RESULT_SUCCEEDED)
        {
          VCP_LOG_FAILURE("Cannot align K4A " << (is_depth ? "depth" : "infrared") << " image to color image!");
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
    if (params.depth_in_meters)
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
    VCP_LOG_WARNING("k4a '" << params.serial_number << "', sink '" << params.sink_label << "' received no " << (is_depth ? "depth" : "infrared") << " data!");
  }
  return extracted;
}


void ExtractK4AStreams(const K4ASinkParams &params,
                       k4a_capture_t &k4a_capture,
                       k4a_transformation_t &transformation,
                       cv::Mat &out_rgb,
                       cv::Mat &out_depth,
                       cv::Mat &out_ir,
                       bool rgb_stream_enabled,
                       bool depth_stream_enabled,
                       bool ir_stream_enabled,
                       calibration::StreamIntrinsics &rgb_intrinsics,
                       calibration::StreamIntrinsics &depth_intrinsics,
                       calibration::StreamIntrinsics &ir_intrinsics)
{
  cv::Mat cvrgb, cvdepth, cvir;
  cv::Mat rrgb, rdepth, rir;
  k4a_image_t image = nullptr;

  // Probe for color image
  if (rgb_stream_enabled)
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
      if (params.color_as_bgr)
        cv::cvtColor(buf, cvrgb, CV_BGRA2BGR);
      else
        cv::cvtColor(buf, cvrgb, CV_BGRA2RGB);
#endif
      SetIntrinsicsResolution(rgb_intrinsics, cvrgb);
      if (params.rectify)
        rrgb = rgb_intrinsics.UndistortRectify(cvrgb);
      else
        rrgb = cvrgb;

      // Now it's safe to free the memory
      k4a_image_release(image);
      image = NULL;
    }
    else
    {
      VCP_LOG_WARNING("k4a '" << params.serial_number << "', sink '" << params.sink_label << "' received no color image!");
    }
  }

  // Probe for a depth16 image
  if (depth_stream_enabled)
  {
    image = k4a_capture_get_depth_image(k4a_capture);
    cvdepth = Extract16U(params, image, transformation, true, cvrgb);

    SetIntrinsicsResolution(depth_intrinsics, cvdepth);
    if (params.rectify)
      rdepth = depth_intrinsics.UndistortRectify(cvdepth);
    else
      rdepth = cvdepth;
  }

  // Probe for the IR16 image
  if (ir_stream_enabled)
  {
    image = k4a_capture_get_ir_image(k4a_capture);
    cvir = Extract16U(params, image, transformation, false, cvrgb);

    SetIntrinsicsResolution(ir_intrinsics, cvir);
    if (params.rectify)
      rir = ir_intrinsics.UndistortRectify(cvir);
    else
      rir = cvir;
  }

  out_rgb = imutils::ApplyImageTransformations(rrgb, params.transforms);
  out_depth = imutils::ApplyImageTransformations(rdepth, params.transforms);
  out_ir = imutils::ApplyImageTransformations(rir, params.transforms);
}



bool MapIntrinsicsHelper(const std::vector<calibration::StreamIntrinsics> &intrinsics,
                         calibration::StreamIntrinsics &to_set, const std::string &expected_label)
{
  for (const auto &calib : intrinsics)
  {
    if (expected_label.compare(calib.StreamLabel()) == 0)
    {
      to_set = calib;
      return true;
    }
  }
  return false;
}

bool MapIntrinsics(const K4ASinkParams &params, const std::vector<calibration::StreamIntrinsics> &intrinsics,
                   calibration::StreamIntrinsics &rgb_intrinsics,
                   calibration::StreamIntrinsics &depth_intrinsics,
                   calibration::StreamIntrinsics &ir_intrinsics)
{
  if (params.IsColorStreamEnabled())
  {
    if (!MapIntrinsicsHelper(intrinsics, rgb_intrinsics, "color"))
    {
      VCP_LOG_FAILURE("Color stream of K4A '" << params.serial_number << "', sink '" << params.sink_label << "' is enabled but not calibrated.");
      return false;
    }
  }
  if (params.IsDepthStreamEnabled())
  {
    if (!MapIntrinsicsHelper(intrinsics, depth_intrinsics, "depth"))
    {
      VCP_LOG_FAILURE("Depth stream of K4A '" << params.serial_number << "', sink '" << params.sink_label << "' is enabled but not calibrated.");
      return false;
    }
  }
  if (params.IsInfraredStreamEnabled())
  {
    if (!MapIntrinsicsHelper(intrinsics, ir_intrinsics, "ir"))
    {
      VCP_LOG_FAILURE("Infrared stream of K4A '" << params.serial_number << "', sink '" << params.sink_label << "' is enabled but not calibrated.");
      return false;
    }
  }
  return true;
}

/** @brief Streams from a single Azure Kinect. */
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
#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE

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

    params_.serial_number = GetSerialNumber(k4a_device_);
    if (params_.serial_number.empty())
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
      VCP_LOG_FAILURE("K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "' stream already running - ignoring StartStreaming() call.");
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
        VCP_LOG_INFO_DEFAULT("Closing K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "' receiver thread.");
      continue_capture_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "' receiver thread has terminated.");
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
      labels.push_back(params_.sink_label + "-color");
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
      types.push_back(FrameType::MONOCULAR);
    if (depth_stream_enabled_)
      types.push_back(FrameType::DEPTH);
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

  void SetVerbose(bool verbose) override
  {
    params_.verbose = verbose;
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    std::vector<const calibration::StreamExtrinsics*> extrinsics;
    if (rgb_stream_enabled_)
      extrinsics.push_back(&rgb_extrinsics_);
    if (depth_stream_enabled_)
      extrinsics.push_back(&depth_extrinsics_);
    if (ir_stream_enabled_)
      extrinsics.push_back(&ir_extrinsics_);

    R = extrinsics[stream_index]->R().clone();
    t = extrinsics[stream_index]->t().clone();
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    calibration::StreamExtrinsics* ext = ExtrinsicsPtrAt(stream_index);

    if (rgb_stream_enabled_)
    {
      const auto &intr = available_ ? IntrinsicsAt(stream_index) : calibration::StreamIntrinsics();
      //const bool retval = ext->SetExtrinsics(R, t, intr, rgb_extrinsics_.R(), rgb_extrinsics_.t());
      const cv::Mat &ref_R = FrameTypeAt(stream_index) == vcp::best::FrameType::MONOCULAR ? cv::Mat() : rgb_extrinsics_.R();
      const cv::Mat &ref_t = FrameTypeAt(stream_index) == vcp::best::FrameType::MONOCULAR ? cv::Mat() : rgb_extrinsics_.t();
      const bool retval = ext->SetExtrinsics(R, t, intr, ref_R, ref_t);
      if (retval && stream_index == 0)
      {
        // If RGB is enabled AND we just set the extrinsics of the color stream successfully,
        // then we can adjust the remaining extrinsics too (via the known view-to-reference
        // transformations).
        for (size_t idx = 1; idx < NumStreams(); ++idx)
        {
          if (ExtrinsicsPtrAt(idx)->Empty())
            SetExtrinsicsAt(idx, cv::Mat(), cv::Mat());
        }
      }
      return retval;
    }
    return ext->SetExtrinsics(R, t);
  }

  SinkType GetSinkType() const override
  {
    return SinkType::K4A;
  }

  vcp::best::calibration::StreamIntrinsics IntrinsicsAt(size_t stream_index) const override
  {
    if (!available_)
    {
      VCP_LOG_FAILURE("Intrinsics for K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "' cannot be queried before the sensor is available! Use IsDeviceAvailable() to check when the sensor is ready.");
      return calibration::StreamIntrinsics();
    }

    std::vector<const calibration::StreamIntrinsics*> intrinsics;
    if (rgb_stream_enabled_)
      intrinsics.push_back(&rgb_intrinsics_);
    if (depth_stream_enabled_)
      intrinsics.push_back(&depth_intrinsics_);
    if (ir_stream_enabled_)
      intrinsics.push_back(&ir_intrinsics_);
    return *(intrinsics[stream_index]);
  }

private:
  std::atomic<bool> continue_capture_;
  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> rgb_queue_;
  std::unique_ptr<SinkBuffer> depth_queue_;
  std::unique_ptr<SinkBuffer> ir_queue_;
  K4ASinkParams params_;
  std::string calibration_file_;
  std::atomic<int> available_;
  bool rgb_stream_enabled_;
  bool depth_stream_enabled_;
  bool ir_stream_enabled_;
  calibration::StreamIntrinsics rgb_intrinsics_;
  calibration::StreamIntrinsics depth_intrinsics_;
  calibration::StreamIntrinsics ir_intrinsics_;
  calibration::StreamExtrinsics rgb_extrinsics_;
  calibration::StreamExtrinsics depth_extrinsics_;
  calibration::StreamExtrinsics ir_extrinsics_;
  k4a_device_t k4a_device_;
#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  void Receive()
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting K4A stream '" << params_.sink_label << "' from device '" << params_.serial_number << "'");

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = params_.camera_fps;
#ifdef VCP_BEST_WITH_K4A_MJPG
    // Save bandwidth, but need to decode JPGs on-the-fly.
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    if (params_.verbose && params_.IsColorStreamEnabled())
      VCP_LOG_INFO_DEFAULT("Configuring K4A color stream as MJPG.");
#else // VCP_BEST_WITH_K4A_MJPG
    // Use already decoded image data.
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    if (params_.verbose && params_.IsColorStreamEnabled())
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
      VCP_ERROR("Failed to retrieve sensor calibration for K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "'.");

    // Prepare transformation for image alignment if needed.
    k4a_transformation_t transformation = nullptr;
    if (params_.align_depth_to_color)
      transformation = k4a_transformation_create(&sensor_calibration);

    // Save calibration if requested.
    if (params_.write_calibration)
      DumpCalibration(params_, sensor_calibration);

    // Load calibration if needed
    // TODO A nice-to-have extension would be to refactor DumpCalibration into something like
    // ReadCalibration() which returns vector<StreamIntrinsics>. This can then be saved
    // (if write_calibration is true) and used (if rectify is true).
    // However, for now it's perfectly fine (i.e. way less effort) to throw an exception
    // and tell the user to adjust the configuration (set a valid calibration file path).
    if (params_.rectify || vcp::utils::file::Exists(params_.calibration_file))
    {
      if (params_.calibration_file.empty() || !vcp::utils::file::Exists(params_.calibration_file))
        VCP_ERROR("To undistort & rectify the K4A '" << params_.serial_number << "' (sink '" << params_.sink_label << "') streams, the calibration file '" << params_.calibration_file << "' must exist!");

      std::vector<calibration::StreamIntrinsics> intrinsics = calibration::LoadIntrinsicsFromFile(params_.calibration_file);
      if (!MapIntrinsics(params_, intrinsics, rgb_intrinsics_, depth_intrinsics_, ir_intrinsics_))
        VCP_ERROR("Cannot load all intrinsics for K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "'.");

      if (!intrinsics.empty() && !intrinsics[0].Identifier().empty() && intrinsics[0].Identifier().compare(params_.serial_number) != 0)
        VCP_ERROR("Calibration file '" << params_.calibration_file << "' provides intrinsics for K4A '" << intrinsics[0].Identifier() << "', but this sensor is '" << params_.serial_number << "'!");

      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Loaded intrinsic calibration for K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "'.");
    }

    // Set color camera configuration:
    SetColorControl(params_, k4a_device_);

    // Check synchronisation status (only relevant if we're in a multi-camera setup)
    if (params_.verbose)
    {
      bool sync_in_connected, sync_out_connected;
      GetSyncJackStatus(k4a_device_, sync_in_connected, sync_out_connected);
      VCP_LOG_INFO_DEFAULT("K4A sync jack status for '" << params_.serial_number << "', sink '" << params_.sink_label << "'" << std::endl
                          << "          IN:  " << (sync_in_connected ? "connected" : "not connected") << std::endl
                          << "          OUT: " << (sync_out_connected ? "connected" : "not connected"));
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
      VCP_ERROR("Failed to start K4A device '" << params_.serial_number << "', sink '" << params_.sink_label << "'.");
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
          VCP_LOG_WARNING("Capture request to K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "' timed out.");
          continue;
          break;
      case K4A_WAIT_RESULT_FAILED:
          VCP_LOG_FAILURE("Failed to read a capture from K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "': closing stream.");
          continue_capture_ = false;
          continue;
      }

      cv::Mat rgb, depth, ir;
      ExtractK4AStreams(params_, k4a_capture, transformation,
                        rgb, depth, ir,
                        rgb_stream_enabled_, depth_stream_enabled_, ir_stream_enabled_,
                        rgb_intrinsics_, depth_intrinsics_, ir_intrinsics_);

      // Store requested streams
      image_queue_mutex_.lock();
      if (rgb_stream_enabled_)
        rgb_queue_->PushBack(rgb.clone());
      if (depth_stream_enabled_)
        depth_queue_->PushBack(depth.clone());
      if (ir_stream_enabled_)
        ir_queue_->PushBack(ir.clone());
      image_queue_mutex_.unlock();

      // Release capture
      k4a_capture_release(k4a_capture);
      k4a_capture = nullptr;

#ifdef VCP_BEST_DEBUG_FRAMERATE
      const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_frame_timepoint_);
      previous_frame_timepoint_ = now;
      const double ms_ema_alpha = 0.1;

      if (ms_between_frames_ < 0.0)
        ms_between_frames_ = duration.count();
      else
        ms_between_frames_ = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_;

      VCP_LOG_DEBUG_DEFAULT("K4A '" << params_.serial_number << "', sink '" << params_.sink_label << "' received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                          << std::setw(5) << (1000.0 / ms_between_frames_) << " fps");
#endif // VCP_BEST_DEBUG_FRAMERATE
    }
    // Clean up
    if (k4a_capture)
      k4a_capture_release(k4a_capture);

    if (transformation)
      k4a_transformation_destroy(transformation);

    if (k4a_device_)
      k4a_device_stop_cameras(k4a_device_);
  }

  calibration::StreamExtrinsics* ExtrinsicsPtrAt(size_t stream_index)
  {
    const auto ft = FrameTypeAt(stream_index);
    if (ft == FrameType::MONOCULAR)
      return &rgb_extrinsics_;
    else if (ft == FrameType::DEPTH)
      return &depth_extrinsics_;
    else if (ft == FrameType::INFRARED)
      return &ir_extrinsics_;
    VCP_ERROR("Invalid frame type '" << ft << "' at K4A stream index " << stream_index << ".");
  }
};

//FIXME extrinsics for synced sink!

/** @brief Streams from multiple wire-synced Azure Kinects. */
class K4ASyncedRGBDSink : public StreamSink
{
public:
  K4ASyncedRGBDSink(const std::vector<K4ASinkParams> &params,
              std::vector<std::unique_ptr<SinkBuffer>> buffers) : StreamSink(),
              continue_capture_(false),
              stream_queues_(std::move(buffers)),
              params_(SortSyncedParams(params)),
              verbose_(false), // Will be set true if any param.verbose is true.
              capture_timeout_ms_(5000) // Will be replaced by the minimum param.capture_timeout_ms
  {
    VCP_LOG_DEBUG("K4ASyncedRGBDSink::K4ASyncedRGBDSink()");
    available_ = 0;
    SanityCheckConfiguration();
    for (size_t idx = 0; idx < params_.size(); ++idx)
      k4a_devices_.push_back(nullptr);

#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
  }

  virtual ~K4ASyncedRGBDSink()
  {
    VCP_LOG_DEBUG("K4ASyncedRGBDSink::~K4ASyncedRGBDSink()");
    CloseDevice();
  }

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("K4ASyncedRGBDSink::OpenDevice()");
    for (size_t idx = 0; idx < k4a_devices_.size(); ++idx)
    {
      if (k4a_devices_[idx])
      {
        VCP_LOG_FAILURE("Device #" << idx << " already opened!");
        return false;
      }
    }
    if (verbose_)
    {
      std::vector<std::string> lbls;
      for (const auto &p : params_)
        lbls.push_back(p.sink_label);
      VCP_LOG_INFO_DEFAULT("Opening Kinect Azure devices " << lbls << " in wired sync mode");
    }

    for (size_t idx = 0; idx < params_.size(); ++idx)
    {
      if (!GetDevice(k4a_devices_[idx], params_[idx])) // This call already issues a warning if no device is available
        return false;

      params_[idx].serial_number = GetSerialNumber(k4a_devices_[idx]);
      if (params_[idx].serial_number.empty())
        return false;

      // Check if we need to dump the calibration:
      if (params_[idx].write_calibration && params_[idx].calibration_file.empty())
      {
        VCP_LOG_FAILURE("If you want to dump the K4A calibration, you must specify the filename as calibration_file parameter! Check configuration of sink '" << params_[idx].sink_label << "'.");
        return false;
      }
    }
    SanityCheckDevices();
    return true;
  }


  bool CloseDevice() override
  {
    StopStreaming();
    for (size_t idx = 0; idx < k4a_devices_.size(); ++idx)
    {
      if (k4a_devices_[idx])
      {
        VCP_LOG_DEBUG("K4ASyncedRGBDSink::CloseDevice() - Closing device #" << idx);
        k4a_device_close(k4a_devices_[idx]);
        k4a_devices_[idx] = nullptr;
      }
    }
    return true;
  }


  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("K4ASyncedRGBDSink::StartStreaming()");
    if (continue_capture_)
    {
      std::vector<std::string> lbls;
      for (const auto &p : params_)
        lbls.push_back(p.sink_label);
      VCP_LOG_FAILURE("Streaming thread for K4A " << lbls << " is already running - ignoring StartStreaming() call.");
      return false;
    }

    continue_capture_ = true;
    stream_thread_ = std::thread(&K4ASyncedRGBDSink::Receive, this);
    return true;
  }


  bool StopStreaming() override
  {
    if (continue_capture_)
    {
      VCP_LOG_DEBUG("K4ASyncedRGBDSink::StopStreaming()");
      std::vector<std::string> lbls;
      if (verbose_)
      {
        for (const auto &p : params_)
          lbls.push_back(p.sink_label);
        VCP_LOG_INFO_DEFAULT("Closing receiver thread for K4ASyncedRGBDSink: " << lbls << ".");
      }
      continue_capture_ = false;
      stream_thread_.join();
      if (verbose_)
        VCP_LOG_INFO_DEFAULT("Receiver thread for K4ASyncedRGBDSink: " << lbls << " has terminated.");
    }
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> res;
    image_queue_mutex_.lock();
    for (size_t idx = 0; idx < enabled_streams_.size(); ++idx)
    {
      if (enabled_streams_[idx])
      {
        if (stream_queues_[idx]->Empty())
        {
          res.push_back(cv::Mat());
        }
        else
        {
          res.push_back(stream_queues_[idx]->Front().clone());
          stream_queues_[idx]->PopFront();
        }
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
//    size_t num = 0;
//    for (const auto e : enabled_streams_)
//    {
//      if (e)
//        ++num;
//    }
//    return num;
    return stream2device_.size();
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    const auto ft = FrameTypeAt(stream_index);
    const auto &p = params_[stream2device_[stream_index]];
    if (ft == FrameType::MONOCULAR)
      return p.sink_label + "-color";
    else if (ft == FrameType::DEPTH)
      return p.sink_label + "-depth";
    else if (ft == FrameType::INFRARED)
      return p.sink_label + "-ir";
    VCP_ERROR("Invalid frame type '" << ft << "' at stream index " << stream_index << " for K4A.");
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    return frame_types_[stream_index];
  }


  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    return params_[stream2device_[stream_index]];
  }


  size_t NumDevices() const override
  {
    return k4a_devices_.size();
  }


  int IsFrameAvailable() const override
  {
    // Should only return true, if all enabled streams have a frame available.
    bool available = true;
    image_queue_mutex_.lock();
    for (size_t idx = 0; idx < enabled_streams_.size(); ++idx)
    {
      if (enabled_streams_[idx] && stream_queues_[idx]->Empty())
        available = false;
    }
    image_queue_mutex_.unlock();
    if (available)
      return 1;
    return 0;
  }

  size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    image_queue_mutex_.lock();
    for (size_t idx = 0; idx < enabled_streams_.size(); ++idx)
    {
      if (enabled_streams_[idx] && !stream_queues_[idx]->Empty())
        ++num;
    }
    image_queue_mutex_.unlock();
    return num;
  }

  void SetVerbose(bool verbose) override
  {
    for (auto &p : params_)
      p.verbose = verbose;
    verbose_ = verbose;
  }

  SinkType GetSinkType() const override
  {
    return SinkType::K4A;
  }

  vcp::best::calibration::StreamIntrinsics IntrinsicsAt(size_t stream_index) const override
  {
    if (!available_)
    {
      VCP_LOG_FAILURE("Intrinsics for K4ASyncedRGBDSink cannot be queried before the sensors are available! Use IsDeviceAvailable() to check when the sensors become ready.");
      return calibration::StreamIntrinsics();
    }
    return *(intrinsics_ptrs_[stream_index]);
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    R = extrinsics_ptrs_[stream_index]->R().clone();
    t = extrinsics_ptrs_[stream_index]->t().clone();
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    // Extrinsics of this stream.
    calibration::StreamExtrinsics* ext = extrinsics_ptrs_[stream_index];
    // Which sensor/sink/device this stream belongs to.
    const auto dev_lookup = stream2device_[stream_index];
    // Configuration of this stream (device, actually).
    const auto &sink_params = params_[dev_lookup];
    if (sink_params.IsColorStreamEnabled())
    {
      const auto &intr = available_ ? IntrinsicsAt(stream_index) : calibration::StreamIntrinsics();
      const cv::Mat &ref_R = FrameTypeAt(stream_index) == vcp::best::FrameType::MONOCULAR ? cv::Mat() : rgb_extrinsics_[dev_lookup].R();
      const cv::Mat &ref_t = FrameTypeAt(stream_index) == vcp::best::FrameType::MONOCULAR ? cv::Mat() : rgb_extrinsics_[dev_lookup].t();
      const bool retval = ext->SetExtrinsics(R, t, intr, ref_R, ref_t);
      //const bool retval = ext->SetExtrinsics(R, t, intr, rgb_extrinsics_[dev_lookup].R(), rgb_extrinsics_[dev_lookup].t());
      if (retval && FrameTypeAt(stream_index) == FrameType::MONOCULAR)
      {
        // If RGB is enabled AND we just set the extrinsics of the color stream successfully,
        // then we can adjust the remaining extrinsics too (via the known view-to-reference
        // transformations).
        if (depth_extrinsics_[dev_lookup].Empty())
        {
          depth_extrinsics_[dev_lookup].SetExtrinsics(cv::Mat(), cv::Mat(),
                                                      intr,
                                                      rgb_extrinsics_[dev_lookup].R(),
                                                      rgb_extrinsics_[dev_lookup].t());
        }

        if (ir_extrinsics_[dev_lookup].Empty())
        {
          ir_extrinsics_[dev_lookup].SetExtrinsics(cv::Mat(), cv::Mat(),
                                                   intr,
                                                   rgb_extrinsics_[dev_lookup].R(),
                                                   rgb_extrinsics_[dev_lookup].t());
        }
      }
      return retval;
    }
    else
      return ext->SetExtrinsics(R, t);
  }

private:
  std::atomic<bool> continue_capture_;
  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;
  std::vector<std::unique_ptr<SinkBuffer>> stream_queues_;
  std::vector<K4ASinkParams> params_;
  std::vector<bool> enabled_streams_;
  std::vector<FrameType> frame_types_;
  std::vector<size_t> stream2device_;
  bool verbose_;
  int32_t capture_timeout_ms_;
  std::atomic<int> available_;
  std::vector<calibration::StreamIntrinsics> rgb_intrinsics_;
  std::vector<calibration::StreamIntrinsics> depth_intrinsics_;
  std::vector<calibration::StreamIntrinsics> ir_intrinsics_;
  std::vector<calibration::StreamIntrinsics*> intrinsics_ptrs_;
  std::vector<calibration::StreamExtrinsics> rgb_extrinsics_;
  std::vector<calibration::StreamExtrinsics> depth_extrinsics_;
  std::vector<calibration::StreamExtrinsics> ir_extrinsics_;
  std::vector<calibration::StreamExtrinsics*> extrinsics_ptrs_;
  //std::vector<size_t> frame2sink_;
  std::vector<k4a_device_t> k4a_devices_;
#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  void Receive()
  {
    std::vector<std::string> lbls;
    for (const auto &p : params_)
      lbls.push_back(p.sink_label);

    if (verbose_)
      VCP_LOG_INFO_DEFAULT("Starting K4ASyncedRGBDSink " << lbls << ".");
    // Prepare sensor transformations (needed, because we mix up the indices since
    // subordinates (k4a_devices_ indices 1+) must be set up before the master (index 0).
    std::vector<k4a_transformation_t> transformations;
    for (size_t cnt = 0; cnt < k4a_devices_.size(); ++cnt)
      transformations.push_back(nullptr);

    for (size_t cnt = 0; cnt < k4a_devices_.size(); ++cnt)
    {
      // We must start the subordinates BEFORE the master!
      const size_t idx = (cnt + 1) % k4a_devices_.size();
      if (verbose_)
        VCP_LOG_INFO_DEFAULT("* Starting K4A stream '" << params_[idx].sink_label << "' from device '" << params_[idx].serial_number << "' (" << (idx > 0 ? "subordinate" : "master") << ").");

      k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
      config.camera_fps = params_[idx].camera_fps;
#ifdef VCP_BEST_WITH_K4A_MJPG
      // Save bandwidth, but need to decode JPGs on-the-fly.
      config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
      if (verbose_ && params_[idx].IsColorStreamEnabled())
        VCP_LOG_INFO_DEFAULT("* Configuring K4A color stream as MJPG.");
#else // VCP_BEST_WITH_K4A_MJPG
      // Use already decoded image data.
      config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
      if (params_[idx].verbose && params_[idx].IsColorStreamEnabled())
        VCP_LOG_INFO_DEFAULT("* Configuring K4A color stream as BGRA32.");
#endif // VCP_BEST_WITH_K4A_MJPG
      config.color_resolution = params_[idx].color_resolution;
      config.depth_delay_off_color_usec = params_[idx].depth_delay_off_color_usec;
      config.depth_mode = params_[idx].depth_mode;
      config.disable_streaming_indicator = params_[idx].disable_streaming_indicator;
      config.subordinate_delay_off_master_usec = params_[idx].subordinate_delay_off_master_usec;
      config.synchronized_images_only = true;
      config.wired_sync_mode = params_[idx].wired_sync_mode;


      // After setting the configuration, we can already query sensor information (since
      // the device is already opened).
      // Query device calibration
      k4a_calibration_t sensor_calibration;
      if (k4a_device_get_calibration(k4a_devices_[idx], config.depth_mode,
                                     config.color_resolution, &sensor_calibration) != K4A_RESULT_SUCCEEDED)
        VCP_ERROR("Failed to retrieve sensor calibration for K4A '" << params_[idx].serial_number << "', sink '" << params_[idx].sink_label << "'.");

      // Prepare transformation for image alignment if needed.

      if (params_[idx].align_depth_to_color)
        transformations[idx] = k4a_transformation_create(&sensor_calibration);


      // Save calibration if requested.
      if (params_[idx].write_calibration)
        DumpCalibration(params_[idx], sensor_calibration);

      // Load calibration if needed
      if (params_[idx].rectify || vcp::utils::file::Exists(params_[idx].calibration_file))
      {
        if (params_[idx].calibration_file.empty() || !vcp::utils::file::Exists(params_[idx].calibration_file))
          VCP_ERROR("To undistort & rectify the K4A '" << params_[idx].serial_number
                    << "' (sink '" << params_[idx].sink_label << "') streams, the calibration file '"
                    << params_[idx].calibration_file << "' must exist!");

        std::vector<calibration::StreamIntrinsics> intrinsics = calibration::LoadIntrinsicsFromFile(params_[idx].calibration_file);
        if (!MapIntrinsics(params_[idx], intrinsics,
                           rgb_intrinsics_[idx], depth_intrinsics_[idx],
                           ir_intrinsics_[idx]))
          VCP_ERROR("Cannot load all intrinsics for K4A '" << params_[idx].serial_number
                    << "', sink '" << params_[idx].sink_label << "'.");

        if (!intrinsics.empty() && !intrinsics[0].Identifier().empty() && intrinsics[0].Identifier().compare(params_[idx].serial_number) != 0)
          VCP_ERROR("Calibration file '" << params_[idx].calibration_file
                    << "' provides intrinsics for K4A '" << intrinsics[0].Identifier()
                    << "', but this sensor is '" << params_[idx].serial_number << "'!");

        if (verbose_)
          VCP_LOG_INFO_DEFAULT("* Loaded intrinsic calibration for K4A '"
                               << params_[idx].serial_number << "', sink '" << params_[idx].sink_label << "'.");
      }

      // Set color camera configuration:
      SetColorControl(params_[idx], k4a_devices_[idx]);

      // In contrast to realsense devices, the camera streams can be started *after*
      // querying sensor calibration, etc.
      if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(k4a_devices_[idx], &config))
      {
        if (k4a_devices_[idx])
        {
          k4a_device_close(k4a_devices_[idx]);
          k4a_devices_[idx] = NULL;
        }
        VCP_ERROR("Failed to start K4A device '" << params_[idx].serial_number << "', sink '" << params_[idx].sink_label << "'.");
      }
    }

    // Now this multi-sink is ready to publish images.
    available_ = 1;

    std::vector<k4a_capture_t> captures(k4a_devices_.size());
    while(continue_capture_)
    {
      switch(GetSynchronizedCaptures(captures, false))
      {
        case K4A_WAIT_RESULT_SUCCEEDED:
          break;
      case K4A_WAIT_RESULT_TIMEOUT:
          VCP_LOG_WARNING("Capture request for K4ASyncedRGBDSink " << lbls << " timed out.");
          continue;
          break;
        case K4A_WAIT_RESULT_FAILED:
          VCP_LOG_FAILURE("Failed to read a synchronized capture from wire-synced K4A " << lbls << ", closing streams.");
          continue_capture_ = false;
          continue;
      }

      // Extract OpenCV images from the K4A captures
      std::vector<cv::Mat> frames(enabled_streams_.size());
      for (size_t idx = 0; idx < captures.size(); ++idx)
      {
        const bool want_rgb = enabled_streams_[idx * 3];
        const bool want_depth = enabled_streams_[idx * 3 + 1];
        const bool want_ir = enabled_streams_[idx * 3 + 2];
        ExtractK4AStreams(params_[idx], captures[idx], transformations[idx],
                          frames[idx * 3], frames[idx * 3 + 1], frames[idx * 3 +2],
                          want_rgb, want_depth, want_ir,
                          rgb_intrinsics_[idx], depth_intrinsics_[idx], ir_intrinsics_[idx]);

      }

      // Store
      image_queue_mutex_.lock();
      for (size_t idx = 0; idx < frames.size(); ++idx)
      {
        if (enabled_streams_[idx])
          stream_queues_[idx]->PushBack(frames[idx].clone());
      }
      image_queue_mutex_.unlock();

      // Release memory
      for (size_t idx = 0; idx < captures.size(); ++idx)
      {
        if (captures[idx])
        {
          k4a_capture_release(captures[idx]);
          captures[idx] = nullptr;
        }
      }

#ifdef VCP_BEST_DEBUG_FRAMERATE
      const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_frame_timepoint_);
      previous_frame_timepoint_ = now;
      const double ms_ema_alpha = 0.1;

      if (ms_between_frames_ < 0.0)
        ms_between_frames_ = duration.count();
      else
        ms_between_frames_ = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_;

      VCP_LOG_DEBUG_DEFAULT("K4ASyncedRGBDSink " << lbls << "' received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                          << std::setw(5) << (1000.0 / ms_between_frames_) << " fps");
#endif // VCP_BEST_DEBUG_FRAMERATE
    }

    // Clean up
    for (size_t idx = 0; idx < k4a_devices_.size(); ++idx)
    {
      if (captures[idx])
        k4a_capture_release(captures[idx]);

      if (transformations[idx])
        k4a_transformation_destroy(transformations[idx]);

      if (k4a_devices_[idx])
        k4a_device_stop_cameras(k4a_devices_[idx]);
    }
  }

  std::vector<K4ASinkParams> SortSyncedParams(const std::vector<K4ASinkParams> &params)
  {
    std::vector<K4ASinkParams> sorted;
    size_t idx_master = 0;
    size_t num_masters = 0;
    for (size_t idx = 0; idx < params.size(); ++idx)
    {
      if (!params[idx].RequiresWiredSync())
        VCP_ERROR("You tried to set up a K4ASyncedRGBDSink with wired_sync_mode STANDALONE! Check configuration of sink '"
                  << params[idx].sink_label << "'!");
      if (params[idx].wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
      {
        ++num_masters;
        idx_master = idx;
      }
    }

    if (num_masters != 1)
      VCP_ERROR("To set up a K4ASyncedRGBDSink you need to specify exactly 1 master sink, you configured "
                << num_masters << ".");
    // First param belongs to the master sink.
    sorted.push_back(params[idx_master]);
    // All others are subordinates.
    for (size_t idx = 0; idx < params.size(); ++idx)
    {
      if (idx != idx_master)
        sorted.push_back(params[idx]);
    }
    return sorted;
  }

  int32_t GetExposureTime(const K4ASinkParams &p)
  {
    for (const auto &ccm : p.color_control_manual)
    {
      if (ccm.command == K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE)
        return ccm.value;
    }
    return -1;
  }

  void SanityCheckConfiguration()
  {
    if (params_.empty())
      VCP_ERROR("You need to configure at least a single K4A sink in order to set up a K4ASyncedRGBDSink!");

    // Compute the depth delay.
    int32_t depth_offset = -1 * static_cast<int32_t>((params_.size() - 1) * TIME_BETWEEN_DEPTH_CAPTURES_USEC / 2);

    const int32_t master_exposure = GetExposureTime(params_[0]);

    // Prepare empty intrinsics
    rgb_intrinsics_.resize(params_.size());
    depth_intrinsics_.resize(params_.size());
    ir_intrinsics_.resize(params_.size());

    rgb_extrinsics_.resize(params_.size());
    depth_extrinsics_.resize(params_.size());
    ir_extrinsics_.resize(params_.size());

    for (size_t idx = 0; idx < params_.size(); ++idx)
    {
      auto &p = params_[idx];

      // For synced K4A capture, each sink must have a manual exposure time set:
      bool exposure_auto = true;
      for (const auto &ccs : p.color_control_manual)
      {
        if (ccs.mode == K4A_COLOR_CONTROL_MODE_MANUAL && ccs.command == K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE)
          exposure_auto = false;
      }
      if (exposure_auto)
        VCP_ERROR("You tried to set up a K4ASyncedRGBDSink without configuring manual exposure time, add K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE to 'color_control_manual' configuration of sink '"
                << p.sink_label << "'!");

      // Master cannot have a subordinate delay
      if (idx == 0 && p.subordinate_delay_off_master_usec > 0)
        VCP_ERROR("The K4A master cannot have a subordinate_delay_off_master_usec setting > 0! Fix configuration of sink '"
                << p.sink_label << "'!");

      if (params_.size() > 1 && !p.synchronized_images_only)
        VCP_ERROR("Synchronized K4A sinks require synchronized_images_only to be true. Fix configuration of sink '"
                << p.sink_label << "'!");

      // Make multi-sink verbose if any sink is verbose.
      verbose_ |= p.verbose;

      // Keep shortest capture timeout configuration:
      capture_timeout_ms_ = std::min(capture_timeout_ms_, p.capture_timeout_ms);

      const int32_t dev_exposure = GetExposureTime(params_[idx]);
      if (dev_exposure != master_exposure)
      {
        // According to the SDK's green_screen/main.cpp: To synchronize a master and
        // subordinate with different exposures the user should set
        // `subordinate_delay_off_master_usec = ((subordinate exposure time) - (master exposure time))/2`.
        p.subordinate_delay_off_master_usec = static_cast<uint32_t>(std::abs(dev_exposure - master_exposure) / 2);
      }
      else
        p.subordinate_delay_off_master_usec = 0;

      // Set depth delay
      p.depth_delay_off_color_usec = depth_offset;
      depth_offset += TIME_BETWEEN_DEPTH_CAPTURES_USEC;

      // Check which streams are enabled for each device
      const bool is_color = p.IsColorStreamEnabled();
      enabled_streams_.push_back(is_color);
      if (is_color)
      {
        stream2device_.push_back(idx);
        frame_types_.push_back(FrameType::MONOCULAR);
        intrinsics_ptrs_.push_back(&rgb_intrinsics_[idx]);
        extrinsics_ptrs_.push_back(&rgb_extrinsics_[idx]);
      }

      const bool is_depth = p.IsDepthStreamEnabled();
      enabled_streams_.push_back(is_depth);
      if (is_depth)
      {
        stream2device_.push_back(idx);
        frame_types_.push_back(FrameType::DEPTH);
        intrinsics_ptrs_.push_back(&depth_intrinsics_[idx]);
        extrinsics_ptrs_.push_back(&depth_extrinsics_[idx]);
      }

      const bool is_ir = p.IsInfraredStreamEnabled();
      enabled_streams_.push_back(is_ir);
      if (is_ir)
      {
        stream2device_.push_back(idx);
        frame_types_.push_back(FrameType::INFRARED);
        intrinsics_ptrs_.push_back(&ir_intrinsics_[idx]);
        extrinsics_ptrs_.push_back(&ir_extrinsics_[idx]);
      }
    }
  }

  void SanityCheckDevices()
  {
    std::vector<std::string> lbls;
    for (const auto &p : params_)
      lbls.push_back(p.sink_label);

    // Subordinates must have SYNC IN connected, master SYNC OUT.
    bool sync_in_connected, sync_out_connected;
    GetSyncJackStatus(k4a_devices_[0], sync_in_connected, sync_out_connected);
    if (k4a_devices_.size() > 1 && !sync_out_connected)
      VCP_ERROR("SYNC OUT of master '" << params_[0].serial_number << "', sink '" << params_[0].sink_label << "' is not connected. Cannot start streaming from K4A " << lbls << ".");

    for (size_t idx = 1; idx < k4a_devices_.size(); ++idx)
    {
      GetSyncJackStatus(k4a_devices_[idx], sync_in_connected, sync_out_connected);
      if (!sync_in_connected)
        VCP_ERROR("SYNC IN of subordinate '" << params_[idx].serial_number << "', sink '" << params_[idx].sink_label << "' is not connected. Cannot start streaming from K4A " << lbls << ".");
    }
  }

  k4a_wait_result_t GetSynchronizedCaptures(std::vector<k4a_capture_t> &captures, const bool compare_sub_depth_instead_of_color)
  {
    // According to the official Azure-Kinect-Sensor-SDK, "dealing with the synchronized cameras is complex" ;-)
    // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/MultiDeviceCapturer.h
    // This blocking call is based on their green screen example.

    // Potential TODOs:
    // * The SDK example uses K4A_WAIT_INFINITE upon k4a_device_get_capture. We prefer not to
    //   wait forever.
    // * Check known "Multiple Device Sync Issues": https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/530

    // Get a frameset from each sink:
    k4a_wait_result_t res = K4A_WAIT_RESULT_SUCCEEDED;
    for (size_t idx = 0; idx < k4a_devices_.size(); ++idx)
    {
      switch (k4a_device_get_capture(k4a_devices_[idx], &captures[idx], capture_timeout_ms_))
      {
        case K4A_WAIT_RESULT_SUCCEEDED:
          break;
        case K4A_WAIT_RESULT_TIMEOUT:
          VCP_LOG_WARNING("Capture request to K4A '" << params_[idx].serial_number << "', sink '" << params_[idx].sink_label << "' timed out.");
          continue;
          break;
        case K4A_WAIT_RESULT_FAILED:
          VCP_LOG_FAILURE("Failed to read a synchronized capture from wire-synced K4A '"
                          << params_[idx].serial_number << "', sink '" << params_[idx].sink_label << "': closing streams.");
          break;
      }
    }

    // Clean up if we had troubles querying this frameset.
    if (res != K4A_WAIT_RESULT_SUCCEEDED)
    {
      for (size_t idx = 0; idx < captures.size(); ++idx)
      {
        if (captures[idx])
        {
          k4a_capture_release(captures[idx]);
          captures[idx] = nullptr;
        }
      }
      return res;
    }

    // Otherwise, we have to check if the images are synced.
    if (captures.size() < 2)
      return res;

    bool have_synced_images = false;
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    while (!have_synced_images)
    {
      // Timeout if this is taking too long
      int64_t duration_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
      if (duration_ms > WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT)
      {
        VCP_LOG_FAILURE("K4A wire-synced sink is taking too long to synchronize captures");
        res = K4A_WAIT_RESULT_TIMEOUT;
        break;
      }

      k4a_image_t master_color_image = k4a_capture_get_color_image(captures[0]);
      const std::chrono::microseconds master_color_image_time(k4a_image_get_device_timestamp_usec(master_color_image)); // Function returns 0 for invalid image handles

      for (size_t i = 1; i < captures.size(); ++i)
      {
        k4a_image_t sub_image = nullptr;
        if (compare_sub_depth_instead_of_color)
          sub_image = k4a_capture_get_depth_image(captures[i]);
        else
          sub_image = k4a_capture_get_color_image(captures[i]);

        if (master_color_image && sub_image)
        {
          const std::chrono::microseconds sub_image_time(k4a_image_get_device_timestamp_usec(sub_image));
          if (sub_image)
            k4a_image_release(sub_image);
          // The subordinate's color image timestamp, ideally, is the master's color image timestamp plus the
          // delay we configured between the master device color camera and subordinate device color camera
          std::chrono::microseconds expected_sub_image_time =
            master_color_image_time +
            std::chrono::microseconds{ params_[i].subordinate_delay_off_master_usec } +
            std::chrono::microseconds{ params_[i].depth_delay_off_color_usec };
          std::chrono::microseconds sub_image_time_error = sub_image_time - expected_sub_image_time;
          // The time error's absolute value must be within the permissible range. So, for example, if
          // MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 2, offsets of -2, -1, 0, 1, and -2 are
          // permitted
          if (sub_image_time_error < -MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP)
          {
            // Example, where MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 1
            // time                    t=1  t=2  t=3
            // actual timestamp        x    .    .
            // expected timestamp      .    .    x
            // error: 1 - 3 = -2, which is less than the worst-case-allowable offset of -1
            // the subordinate camera image timestamp was earlier than it is allowed to be. This means the
            // subordinate is lagging and we need to update the subordinate to get the subordinate caught up
            VCP_LOG_WARNING("Subordinate is lagging - master " << master_color_image_time.count() << " usec, sub "
                            << sub_image_time.count() << " usec: diff = " << sub_image_time_error.count() << " usec.");
            k4a_device_get_capture(k4a_devices_[i], &captures[i], capture_timeout_ms_);
            break;
          }
          else if (sub_image_time_error > MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP)
          {
            // Example, where MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 1
            // time                    t=1  t=2  t=3
            // actual timestamp        .    .    x
            // expected timestamp      x    .    .
            // error: 3 - 1 = 2, which is more than the worst-case-allowable offset of 1
            // the subordinate camera image timestamp was later than it is allowed to be. This means the
            // subordinate is ahead and we need to update the master to get the master caught up
            VCP_LOG_WARNING("Master device is lagging - master " << master_color_image_time.count() << " usec, sub "
                            << sub_image_time.count() << " usec: diff = " << sub_image_time_error.count() << " usec.");
            k4a_device_get_capture(k4a_devices_[0], &captures[0], capture_timeout_ms_);
//              master_device.get_capture(&captures[0], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
            break;
          }
          else
          {
            // These captures are sufficiently synchronized. If we've gotten to the end, then all are
            // synchronized.
            if (i == captures.size() - 1)
            {
              VCP_LOG_DEBUG("Synchronized a capture - master " << master_color_image_time.count() << " usec, sub "
                           << sub_image_time.count() << " usec: diff = " << sub_image_time_error.count() << " usec.");
              have_synced_images = true; // now we'll finish the for loop and then exit the while loop
            }
          }
        }
        else if (!master_color_image)
        {
          k4a_image_release(sub_image);
          VCP_LOG_WARNING("Received bad master image.");
          k4a_device_get_capture(k4a_devices_[0], &captures[0], capture_timeout_ms_);
          break;
        }
        else if (!sub_image)
        {
          VCP_LOG_WARNING("Received bad subordinate image.");
          k4a_device_get_capture(k4a_devices_[i], &captures[i], capture_timeout_ms_);
          break;
        }
      }
      if (master_color_image)
        k4a_image_release(master_color_image);
    }
    return res;
  }
};


bool K4ASinkParams::IsColorStreamEnabled() const
{
  return this->color_resolution != K4A_COLOR_RESOLUTION_OFF;
}

bool K4ASinkParams::IsDepthStreamEnabled() const
{
  return this->depth_mode != K4A_DEPTH_MODE_OFF && this->depth_mode != K4A_DEPTH_MODE_PASSIVE_IR;
  //return this->depth_mode != K4A_DEPTH_MODE_OFF;
}

bool K4ASinkParams::IsInfraredStreamEnabled() const
{
  return this->enable_infrared_stream &&
      this->depth_mode != K4A_DEPTH_MODE_OFF;
  //return this->enable_infrared_stream && IsDepthStreamEnabled();
}

bool K4ASinkParams::RequiresWiredSync() const
{
  return this->wired_sync_mode != K4A_WIRED_SYNC_MODE_STANDALONE;
}

std::ostream &operator<<(std::ostream &stream, const K4ASinkParams &p)
{
  stream << p.sink_label << "(" << p.serial_number << ") " << std::endl
         << "FPS:   " << p.camera_fps << std::endl
         << "Color: " << p.color_resolution << std::endl
         << "Depth: " << p.depth_mode << std::endl
         << "Align depth to color: " << p.align_depth_to_color << std::endl
         << "Rectify streams: " << p.rectify << std::endl
         << "Depth delay off color: " << p.depth_delay_off_color_usec << std::endl
         << "Subordinate delay off master: " << p.subordinate_delay_off_master_usec << std::endl
         << "Synchronized images only: " << p.synchronized_images_only;
  return stream;
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
  if (!params.IsInfraredStreamEnabled() && params.enable_infrared_stream)
  {
    VCP_LOG_FAILURE("Invalid K4A configuration: enable_infrared_stream=true, but depth mode is 'K4A_DEPTH_MODE_OFF' - adjusting it to 'K4A_DEPTH_MODE_PASSIVE_IR' to continue.");
    params.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
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


void GroupK4ASinkParams(const std::vector<K4ASinkParams> &configured, std::vector<K4ASinkParams> &params_single, std::vector<std::vector<K4ASinkParams> > &params_multi)
{
  params_single.clear();
  params_multi.clear();

  std::vector<K4ASinkParams> need_sync;

  size_t num_masters = 0;

  for (const auto &p : configured)
  {
    if (p.RequiresWiredSync())
    {
      if (p.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
        ++num_masters;
      need_sync.push_back(p);
    }
    else
    {
      params_single.push_back(p);
    }
  }

  if (num_masters > 0)
  {
    if (num_masters > 1)
    {
      VCP_ERROR("Currently, we only support a single master for synchronized K4A captures!");
      // If we should ever need multiple masters, we need to "group" the
      // parameters somehow. Either add another configuration parameter,
      // or add all subordinates to the previously configured master.
    }
    params_multi.push_back(need_sync);
  }
}



std::unique_ptr<StreamSink> CreateBufferedK4ASink(const K4ASinkParams &params,
                                                  std::unique_ptr<SinkBuffer> rgb_buffer,
                                                  std::unique_ptr<SinkBuffer> depth_buffer,
                                                  std::unique_ptr<SinkBuffer> ir_buffer)
{
  return std::unique_ptr<K4ARGBDSink>(new K4ARGBDSink(params, std::move(rgb_buffer), std::move(depth_buffer), std::move(ir_buffer)));
}


std::unique_ptr<StreamSink> CreateBufferedK4ASyncedSink(const std::vector<K4ASinkParams> &params,
                                                        std::vector<std::unique_ptr<SinkBuffer>> buffers)
{
  return std::unique_ptr<K4ASyncedRGBDSink>(new K4ASyncedRGBDSink(params, std::move(buffers)));
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
