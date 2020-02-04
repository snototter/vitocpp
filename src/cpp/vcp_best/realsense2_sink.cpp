#include "realsense2_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <limits>

#include <librealsense2/rs.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>


namespace vcp
{
namespace best
{
namespace realsense2
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::realsense2"

#define MAKE_STRING_TO_RS2OPTIONS_IF(str, O) if (str.compare(#O) == 0) { return O; }
rs2_option StringToOption(const std::string &option_name)
{
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_BACKLIGHT_COMPENSATION)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_BRIGHTNESS)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_CONTRAST)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_EXPOSURE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_GAIN)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_GAMMA)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_HUE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_SATURATION)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_SHARPNESS)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_WHITE_BALANCE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_ENABLE_AUTO_EXPOSURE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_VISUAL_PRESET)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_LASER_POWER)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_ACCURACY)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_MOTION_RANGE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_FILTER_OPTION)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_CONFIDENCE_THRESHOLD)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_EMITTER_ENABLED)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_FRAMES_QUEUE_SIZE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_TOTAL_FRAME_DROPS)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_AUTO_EXPOSURE_MODE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_POWER_LINE_FREQUENCY)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_ASIC_TEMPERATURE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_ERROR_POLLING_ENABLED)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_PROJECTOR_TEMPERATURE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_OUTPUT_TRIGGER_ENABLED)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_MOTION_MODULE_TEMPERATURE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_DEPTH_UNITS)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_ENABLE_MOTION_CORRECTION)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_AUTO_EXPOSURE_PRIORITY)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_COLOR_SCHEME)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_MIN_DISTANCE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_MAX_DISTANCE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_TEXTURE_SOURCE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_FILTER_MAGNITUDE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_FILTER_SMOOTH_ALPHA)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_FILTER_SMOOTH_DELTA)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_HOLES_FILL)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_STEREO_BASELINE)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP)
  MAKE_STRING_TO_RS2OPTIONS_IF(option_name, RS2_OPTION_INTER_CAM_SYNC_MODE)
  VCP_ERROR("RS2 option name '" << option_name << "' is not yet mapped");
}

void SetOptionsFromConfig(const config::ConfigParams &config, const std::string &setting_name, std::vector<std::pair<rs2_option, float>> &options)
{
  if (!config.SettingExists(setting_name))
    return;

  std::vector<std::pair<std::string, double>> opts = config.GetDoubleKeyValueList(setting_name);
  for (const auto &p : opts)
    options.push_back(std::make_pair<rs2_option, float>(StringToOption(p.first), static_cast<float>(p.second)));
}

  //FIXME ip cam/webcam height/width vs resolution - support both
RealSense2SinkParams RealSense2SinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  RealSense2SinkParams params(sink_params);
  params.serial_number = GetOptionalStringFromConfig(config, cam_param, "serial_number", kEmptyRealSense2SerialNumber);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "serial_number"), configured_keys.end());

  const cv::Size rgb_res = ParseResolutionFromConfig(config, cam_param, "rgb_", configured_keys);
  if (rgb_res.width > 0 && rgb_res.height > 0)
  {
    params.rgb_width = rgb_res.width;
    params.rgb_height = rgb_res.height;
  }
  else
  {
    VCP_LOG_WARNING("No RGB resolution specified for '" << cam_param << "', using the default setting of " << params.rgb_width << "x" << params.rgb_height);
  }

  const cv::Size depth_res = ParseResolutionFromConfig(config, cam_param, "depth_", configured_keys);
  if (depth_res.width > 0 && depth_res.height > 0)
  {
    params.depth_width = depth_res.width;
    params.depth_height = depth_res.height;
  }
  else
  {
    VCP_LOG_WARNING("No depth resolution specified for '" << cam_param << "', using the default setting of " << params.depth_width << "x" << params.depth_height);
  }

  params.rgb_frame_rate = GetOptionalIntFromConfig(config, cam_param, "rgb_frame_rate", 30);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "rgb_frame_rate"), configured_keys.end());

  const std::string key_frame_rate_depth = cam_param + ".depth_frame_rate";
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_frame_rate"), configured_keys.end());
  if (config.SettingExists(key_frame_rate_depth))
    params.depth_frame_rate = config.GetInteger(key_frame_rate_depth);
  else
  {
    VCP_LOG_WARNING("No frame rate for depth sensor specified, using the same setting (" << params.rgb_frame_rate << " fps) as for the RGB sensor.");
    params.depth_frame_rate = params.rgb_frame_rate;
  }

  params.align_depth_to_color = GetOptionalBoolFromConfig(config, cam_param, "align_depth_to_color", false);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "align_depth_to_color"), configured_keys.end());

  const std::string key_rgb_options = cam_param + ".rgb_options";
  realsense2::SetOptionsFromConfig(config, key_rgb_options, params.rgb_options);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "rgb_options"), configured_keys.end());

  const std::string key_depth_options = cam_param + ".depth_options";
  realsense2::SetOptionsFromConfig(config, key_depth_options, params.depth_options);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_options"), configured_keys.end());

  params.depth_in_meters = GetOptionalBoolFromConfig(config, cam_param, "depth_in_meters", true);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_in_meters"), configured_keys.end());

  params.write_calibration = GetOptionalBoolFromConfig(config, cam_param, "write_calibration", false);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "write_calibration"), configured_keys.end());

  const std::string key_spatial_filter = cam_param + ".spatial_filter_options";
  realsense2::SetOptionsFromConfig(config, key_spatial_filter, params.spatial_filter_options);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "spatial_filter_options"), configured_keys.end());

  const std::string key_temporal_filter = cam_param + ".temporal_filter_options";
  realsense2::SetOptionsFromConfig(config, key_temporal_filter, params.temporal_filter_options);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "temporal_filter_options"), configured_keys.end());

  WarnOfUnusedParameters(cam_param, configured_keys);
  return params;
}


const std::string kEmptyRealSense2SerialNumber = "----";

inline cv::Mat FrameToMat(const rs2::frame &f, const cv::Size &size)
{
  switch(f.get_profile().format())
  {
  case RS2_FORMAT_BGR8:
  case RS2_FORMAT_RGB8:
    return cv::Mat(size, CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);

  case RS2_FORMAT_Z16:
    return cv::Mat(size, CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);

  case RS2_FORMAT_Y8:
    return cv::Mat(size, CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);

  default:
    VCP_ERROR("RealSense to OpenCV conversion for " << f.get_profile().format() << " is not yet supported!");
  }
  return cv::Mat();
}


/** @brief Convert RealSense depth frame to depth values in meters. */
inline cv::Mat FrameToDepth(const rs2::frame &f, const cv::Size &size, float depth_scale)
{
  if (f.get_profile().format() != RS2_FORMAT_Z16)
    VCP_ERROR("Cannot convert frame '" << f.get_profile().format() << "' to depth values!");

  const cv::Mat depth(size, CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
  cv::Mat depth_meters;
  depth.convertTo(depth_meters, CV_64FC1, static_cast<double>(depth_scale), 0.0);

  return depth_meters;
}


std::string GetSensorName(const rs2::sensor &sensor)
{
  return (sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");
}


bool IsDepthSensor(const rs2::sensor &sensor)
{
  return sensor.is<rs2::depth_sensor>();
}


bool IsRgbSensor(const rs2::sensor &sensor)
{
  const std::string sensor_name = GetSensorName(sensor);
  return sensor_name.compare("RGB Camera") == 0;
}


// Scale the depth frame values to meters.
float GetDepthUnits(const rs2::sensor &sensor)
{
  if (rs2::depth_sensor ds = sensor.as<rs2::depth_sensor>())
    return ds.get_depth_scale();
  VCP_ERROR("Sensor '" << GetSensorName(sensor) << "' is not a depth sensor!");
  return 0.0f;
}


bool GetRgbDepthSensors(const rs2::device &device, rs2::sensor &rgb, rs2::sensor &depth)
{
  // https://github.com/IntelRealSense/librealsense/blob/master/examples/sensor-control/api_how_to.h
  std::vector<rs2::sensor> sensors = device.query_sensors();
  bool rgb_set = false;
  bool depth_set = false;
  for (size_t i = 0; i < sensors.size(); ++i)
  {
    if (IsDepthSensor(sensors[i]))
    {
      depth_set = true;
      depth = sensors[i];
    }
    else if (IsRgbSensor(sensors[i]))
    {
      rgb_set = true;
      rgb = sensors[i];
    }
  }
  return rgb_set && depth_set;
}


std::string GetDeviceName(const rs2::device &device)
{
  return (device.supports(RS2_CAMERA_INFO_NAME) ? device.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");
}


std::string GetDeviceSerialNumber(const rs2::device &device)
{
  return (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : kEmptyRealSense2SerialNumber);
}


bool IsPlatformCamera(const rs2::device &device)
{
  // TODO what are "platform cameras"? (the Intel code sample ignores them, so we probably want to do the same)
  const std::string camera_name = GetDeviceName(device);
  return camera_name.compare("Platform Camera") == 0;
}


std::string GetStringRepresentation(const rs2::device &device)
{
  const std::string camera_name = GetDeviceName(device);
  const std::string serial_number = GetDeviceSerialNumber(device);
  return camera_name + " (" + serial_number + ")";
}


bool GetDevice(rs2::device &device, const RealSense2SinkParams &params)
{
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  if (devices.size() == 0)
    return false;

  if (params.serial_number.empty() || kEmptyRealSense2SerialNumber.compare(params.serial_number) == 0)
  {
    for (uint32_t i = 0; i < devices.size(); ++i)
    {
      if (!IsPlatformCamera(devices[i]))
      {
        device = devices[i];
        return true;
      }
    }
    VCP_LOG_FAILURE("Cannot find a non-platform camera RealSense device!");
  }
  else
  {
    for (uint32_t i = 0; i < devices.size(); ++i)
    {
      if (params.serial_number.compare(GetDeviceSerialNumber(devices[i])) == 0)
      {
        device = devices[i];
        return true;
      }
    }
    VCP_LOG_FAILURE("Cannot find the RealSense device with serial number '" << params.serial_number << "'");
  }
  return false;
}


bool SetOption(const rs2::sensor &sensor, rs2_option option, float value, const bool verbose)
{
  if (!sensor.supports(option))
  {
    VCP_LOG_FAILURE("Sensor '" << GetSensorName(sensor) << "' does not support option '" << rs2_option_to_string(option) << "'");
    return false;
  }
  try
  {
    if (option == RS2_OPTION_VISUAL_PRESET)
    {
      if (verbose)
        VCP_LOG_INFO_DEFAULT("'" << GetSensorName(sensor) << "' set '" << rs2_option_to_string(option) << "' = " << value
                          << " (RS400: " << rs2_rs400_visual_preset_to_string(static_cast<rs2_rs400_visual_preset>(value))
                          << ", RS300: " << rs2_sr300_visual_preset_to_string(static_cast<rs2_sr300_visual_preset>(value)) << ")");
    }
    else
    {
      if (verbose)
        VCP_LOG_INFO_DEFAULT("'" << GetSensorName(sensor) << "' set '" << rs2_option_to_string(option) << "' = " << value);
    }
    sensor.set_option(option, value);
    return true;
  }
  catch (const rs2::error &e)
  {
    VCP_LOG_FAILURE("RealSense error occured when trying to set '" << rs2_option_to_string(option) << "' = " << value << ": " << e.what());
  }
  catch (const std::exception &e)
  {
    VCP_LOG_FAILURE("Standard exception occured when trying to set '" << rs2_option_to_string(option) << "' = " << value << ": " << e.what());
  }
  return false;
}

bool SetFilterOption(rs2::processing_block &filter, rs2_option option, float value, const std::string &filter_name, const bool verbose)
{
  if (!filter.supports(option))
  {
    VCP_LOG_FAILURE("Filter '" << filter_name << "' does not support option '" << rs2_option_to_string(option) << "'");
    return false;
  }
  try
  {
    if (verbose)
      VCP_LOG_INFO_DEFAULT("'" << filter_name << "' set '" << rs2_option_to_string(option) << "' = " << value);
    filter.set_option(option, value);
    return true;
  }
  catch (const rs2::error &e)
  {
    VCP_LOG_FAILURE("RealSense error occured when trying to set '" << rs2_option_to_string(option) << "' = " << value << ": " << e.what());
  }
  catch (const std::exception &e)
  {
    VCP_LOG_FAILURE("Standard exception occured when trying to set '" << rs2_option_to_string(option) << "' = " << value << ": " << e.what());
  }
  return false;
}


cv::Mat KFromIntrinsics(const rs2_intrinsics &intrinsics)
{
  cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
  K.at<double>(0,0) = static_cast<double>(intrinsics.fx);
  K.at<double>(1,1) = static_cast<double>(intrinsics.fy);
  K.at<double>(0,2) = static_cast<double>(intrinsics.ppx);
  K.at<double>(1,2) = static_cast<double>(intrinsics.ppy);
  return K;
}


cv::Mat DFromIntrinsics(const rs2_intrinsics &intrinsics)
{
  cv::Mat D = cv::Mat::zeros(5, 1, CV_64FC1);
  for (size_t i = 0; i < 5; ++i)
    D.at<double>(i) = static_cast<double>(intrinsics.coeffs[i]);
  return D;
}


cv::Mat RFromExtrinsics(const rs2_extrinsics &extrinsics)
{
  cv::Mat R(3, 3, CV_64FC1);
  // RealSense stores R in column-major order, so we need to fill R:
  // [0 3 6;
  //  1 4 7;
  //  2 5 8]
  int arr_idx = 0;
  for (int c = 0; c < 3; ++c)
  {
    for (int r = 0; r < 3; ++r)
    {
      R.at<double>(r, c) = static_cast<double>(extrinsics.rotation[arr_idx]);
      ++arr_idx;
    }
  }
  return R;
}


cv::Mat TFromExtrinsics(const rs2_extrinsics &extrinsics)
{
  cv::Mat T(3, 1, CV_64FC1);
  for (int i = 0; i < 3; ++i)
    T.at<double>(i) = static_cast<double>(extrinsics.translation[i]);
  return T;
}


void DumpCalibration(rs2::pipeline_profile &profile, const std::string &filename,
                     const bool align_depth_to_color, const float depth_scale, const bool verbose)
{
  if (filename.empty())
    VCP_ERROR("DumpCalibration() called with empty file name!");

  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (!fs.isOpened())
    VCP_ABORT("Cannot open '" << filename << "' to store calibration!");

  rs2::video_stream_profile depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  rs2::video_stream_profile rgb_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

  const rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();
  const rs2_extrinsics depth_extrinsics = depth_profile.get_extrinsics_to(rgb_profile);
  const rs2_intrinsics rgb_intrinsics = rgb_profile.get_intrinsics();
  const rs2_extrinsics rgb_extrinsics = rgb_profile.get_extrinsics_to(depth_profile);

  const cv::Mat Krgb = KFromIntrinsics(rgb_intrinsics);
  const cv::Mat Drgb = DFromIntrinsics(rgb_intrinsics);
  const int rgb_width = rgb_intrinsics.width;
  const int rgb_height = rgb_intrinsics.height;

  const cv::Mat Kdepth = align_depth_to_color ? Krgb : KFromIntrinsics(depth_intrinsics);
  const cv::Mat Ddepth = align_depth_to_color ? Drgb : DFromIntrinsics(depth_intrinsics);
  const int depth_width = align_depth_to_color ? rgb_width : depth_intrinsics.width;
  const int depth_height = align_depth_to_color ? rgb_height : depth_intrinsics.height;

  const cv::Mat Rdepth2color = align_depth_to_color ? cv::Mat::eye(3, 3, CV_64FC1) : RFromExtrinsics(depth_extrinsics);
  const cv::Mat Tdepth2color = align_depth_to_color ? cv::Mat::zeros(3, 1, CV_64FC1) : TFromExtrinsics(depth_extrinsics);
  const cv::Mat Rcolor2depth = align_depth_to_color ? cv::Mat::eye(3, 3, CV_64FC1) : RFromExtrinsics(rgb_extrinsics);
  const cv::Mat Tcolor2depth = align_depth_to_color ? cv::Mat::zeros(3, 1, CV_64FC1) : TFromExtrinsics(rgb_extrinsics);

  fs << "K_rgb" << Krgb;
  fs << "D_rgb" << Drgb;
  fs << "width_rgb" << rgb_width;
  fs << "height_rgb" << rgb_height;

  fs << "K_depth" << Kdepth;
  fs << "D_depth" << Ddepth;
  fs << "width_depth" << depth_width;
  fs << "height_depth" << depth_height;
  fs << "depth_scale" << depth_scale;

  fs << "R_depth2rgb" << Rdepth2color;
  fs << "t_depth2rgb" << Tdepth2color;
  fs << "R_rgb2depth" << Rcolor2depth;
  fs << "t_rgb2depth" << Tcolor2depth;

  fs << "type" << "rgbd";
  fs.release();

  if (verbose)
    VCP_LOG_INFO("RealSense calibration has been saved to '" << filename << "'. Change the camera configuration if you want to prevent overwriting this calibration during the next start.");
}


std::vector<RealSense2DeviceInfo> ListRealSense2Devices(bool warn_if_no_devices)
{
  rs2::context context;

  const rs2::device_list device_list = context.query_devices();
  uint32_t num_devices = device_list.size();

  std::vector<RealSense2DeviceInfo> infos;

  if (num_devices == 0)
  {
    if (warn_if_no_devices)
    {
      VCP_LOG_WARNING("No RealSense2 capable devices connected!");
    }
    return infos;
  }

  infos.reserve(num_devices);

  for (const rs2::device &device : device_list)
  {
    if (IsPlatformCamera(device))
    {
      if (warn_if_no_devices)
      {
        VCP_LOG_INFO("Skipping RealSense2 platform camera");
      }
      continue;
    }
    RealSense2DeviceInfo info;
    info.serial_number = GetDeviceSerialNumber(device);
    info.name = GetStringRepresentation(device);
    infos.push_back(info);
  }

  return infos;
}



void DebugFramesetMetadata(const std::string &serial_number, const rs2::frameset &frameset)
{
  const std::vector<rs2_frame_metadata_value> attributes_to_query = {
    RS2_FRAME_METADATA_FRAME_COUNTER, // useless! freezes at 6 (color) or 8/9 (depth), won't increase afterwards
    RS2_FRAME_METADATA_FRAME_TIMESTAMP, // usec
    RS2_FRAME_METADATA_SENSOR_TIMESTAMP, // usec
    RS2_FRAME_METADATA_TIME_OF_ARRIVAL, // "system clock"
    RS2_FRAME_METADATA_BACKEND_TIMESTAMP, // uvc driver timestamp, usec
    RS2_FRAME_METADATA_ACTUAL_FPS
  };

  std::stringstream metadata_str;
  metadata_str << "RealSense [" << serial_number << "] frame metadata:";

//  auto format_usec = [](long long usec) {
//    long ms = static_cast<long>(usec / 1000);
//    const long hours = ms / 3600000;
//    ms -= hours * 3600000;
//    const long mins = ms / 60000;
//    ms -= mins * 60000;
//    const long secs = ms / 1000;
//    ms -= secs * 1000;
//    std::stringstream s;
//    s << std::fixed << std::setw(2) << hours << "h " << mins << "min " << secs << "sec " << ms << "ms";
//    return s.str();
//  };

  for (const auto &&frame : frameset)
  {
    const std::string stream_type = frame.get_profile().stream_type() == RS2_STREAM_DEPTH ? "depth" : "color";
    metadata_str << std::endl << "  " << stream_type;
    for (const auto &att : attributes_to_query)
    {
      if (frame.supports_frame_metadata(att))
      {
        const auto val = frame.get_frame_metadata(att);
        metadata_str << std::endl << "    " << rs2_frame_metadata_value_to_string(att) << ": ";
//        if (att == RS2_FRAME_METADATA_FRAME_TIMESTAMP || att == RS2_FRAME_METADATA_SENSOR_TIMESTAMP
//            || att == RS2_FRAME_METADATA_TIME_OF_ARRIVAL || att == RS2_FRAME_METADATA_BACKEND_TIMESTAMP)
//        {
//          metadata_str << format_usec(val);
//        }
//        else
//        {
          metadata_str << val;
//        }
      }
    }
  }

  // Explicitly add frame# to compare against metadata (since md frame count gets stuck after a few seconds), weird behavior as of March/2019
  metadata_str << std::endl << "Frame numbers: " << frameset.get_color_frame().get_frame_number() << " vs " << frameset.get_depth_frame().get_frame_number();

  VCP_LOG_DEBUG_DEFAULT(metadata_str.str());
}


class RealSense2RGBDSink : public StreamSink
{
public:
  RealSense2RGBDSink(const RealSense2SinkParams &params,
                     std::unique_ptr<SinkBuffer> rgb_buffer,
                     std::unique_ptr<SinkBuffer> depth_buffer)
    : StreamSink(),
    continue_capture_(false),
    rgb_queue_(std::move(rgb_buffer)),
    depth_queue_(std::move(depth_buffer)),
    rgb_prev_fnr_(std::numeric_limits<unsigned long long>::max()),
    depth_prev_fnr_(std::numeric_limits<unsigned long long>::max()),
    rgbd_params_(params),
    dev_opened_(false),
    serial_number_(kEmptyRealSense2SerialNumber),
    available_(0)
  {
    VCP_LOG_DEBUG("RealSense2RGBDSink::RealSense2RGBDSink()");
  }

  virtual ~RealSense2RGBDSink()
  {
    VCP_LOG_DEBUG("RealSense2RGBDSink::~RealSense2RGBDSink()");
    CloseDevice();
  }

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("RealSense2RGBDSink::OpenDevice()");

    if (dev_opened_)
    {
      VCP_LOG_FAILURE("Device already opened!");
      return false;
    }

    if (rgbd_params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening RealSense device");

    // Parameter sanity check.
    if (rgbd_params_.write_calibration && rgbd_params_.calibration_file.empty())
    {
      VCP_LOG_FAILURE("If you want to dump the RealSense calibration, you must specify the filename as calibration_file parameter!");
      return false;
    }

    rs2::device dev;
    if (!GetDevice(dev, rgbd_params_))
    {
      VCP_LOG_FAILURE("Cannot open RealSense device");
      return false;
    }

    rs2::sensor rgb_sensor, depth_sensor;
    if (!GetRgbDepthSensors(dev, rgb_sensor, depth_sensor))
    {
      VCP_LOG_FAILURE("Cannot access RGB or depth sensor of device '" << GetStringRepresentation(dev) << "'");
      return false;
    }

    config_.enable_device(GetDeviceSerialNumber(dev));
    rgbd_params_.serial_number = GetDeviceSerialNumber(dev);

    if (rgbd_params_.verbose)
      VCP_LOG_INFO("Configuring streams of RealSense device [" << rgbd_params_.serial_number << "]");
    try
    {
      config_.enable_stream(RS2_STREAM_COLOR, rgbd_params_.rgb_width, rgbd_params_.rgb_height,
                           rgbd_params_.color_as_bgr ? RS2_FORMAT_BGR8 : RS2_FORMAT_RGB8,
                           rgbd_params_.rgb_frame_rate);
      config_.enable_stream(RS2_STREAM_DEPTH, rgbd_params_.depth_width, rgbd_params_.depth_height,
                           RS2_FORMAT_Z16, rgbd_params_.depth_frame_rate);
      return true;
    }
    catch (const rs2::error &e)
    {
      VCP_LOG_FAILURE("Could not enable streams for RealSense [" << rgbd_params_.serial_number << "]. SDK message: " << e.what());
    }
    return false;
  }

  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("RealSense2RGBDSink::StartStreaming()");
    if (continue_capture_)
    {
      VCP_LOG_FAILURE("RealSense stream already running - ignoring StartStreaming() call.");
      return false;
    }
    continue_capture_ = true;
    stream_thread_ = std::thread(&RealSense2RGBDSink::Receive, this);
    return true;
  }

  bool StopStreaming() override
  {
    if (continue_capture_)
    {
      VCP_LOG_DEBUG("RealSense2RGBDSink::StopStreaming()");
      continue_capture_ = false;
      if (rgbd_params_.verbose)
        VCP_LOG_INFO_DEFAULT("Closing RealSense receiver thread.");
      stream_thread_.join();
      if (rgbd_params_.verbose)
        VCP_LOG_INFO_DEFAULT("RealSense receiver thread has terminated.");
    }
    return true;
  }

  bool CloseDevice() override
  {
    VCP_LOG_DEBUG("RealSense2RGBDSink::CloseDevice()");
    return StopStreaming();
  }

  std::vector<cv::Mat> Next() override
  {
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

  size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    image_queue_mutex_.lock();
    if (!rgb_queue_->Empty())
        ++num;
    if (!depth_queue_->Empty())
        ++num;
    image_queue_mutex_.unlock();
    return num;
  }

  int IsFrameAvailable() const override
  {
    image_queue_mutex_.lock();
    const bool empty = rgb_queue_->Empty() || depth_queue_->Empty();
    image_queue_mutex_.unlock();
    if (empty)
      return 0;
    return 1;
  }

  size_t NumStreams() const override
  {
    return 2;
  }

  size_t NumDevices() const override
  {
    return 1;
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    switch(stream_index)
    {
      case 0:
        return FrameType::RGBD_IMAGE;
      case 1:
        return FrameType::RGBD_DEPTH;
      default:
        VCP_ERROR("stream_index (" << stream_index << ") out of bounds.");
    }
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    switch(stream_index)
    {
      case 0:
        return rgbd_params_.sink_label + "-rgb";
      case 1:
        return rgbd_params_.sink_label + "-depth";
      default:
        VCP_ERROR("stream_index (" << stream_index << ") out of bounds.");
    }
  }

  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return rgbd_params_;
  }


private:
  std::atomic<bool> continue_capture_;
  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> rgb_queue_;
  std::unique_ptr<SinkBuffer> depth_queue_;
  std::atomic<unsigned long long> rgb_prev_fnr_; // since frames might be duplicated by the SDK, we need to filter them manually by frame number :-(
  std::atomic<unsigned long long> depth_prev_fnr_;
  RealSense2SinkParams rgbd_params_;
  bool dev_opened_;
  std::string serial_number_;
  std::atomic<int> available_;
  rs2::config config_;

  void Receive()
  {
    if (rgbd_params_.verbose)
      VCP_LOG_INFO("Starting RealSense streaming pipeline for device [" << serial_number_ << "]");
    // Start the librealsense2 pipeline before configuring the sensors (except for configuring stream resolution/frame rate; this has already been done)!
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;
    try
    {
      profile = pipe.start(config_);
    }
    catch (const rs2::error &e)
    {
      VCP_ERROR("Could not start capturing pipeline for RealSense [" << serial_number_ << "]. If you get 'non-descriptive' error messages below, check your configuration file/sensor parameters carefully." << std::endl << "librealsense error: " << e.what());
    }
VCP_LOG_FAILURE("A");
    // Get sensors corresponding to color and depth
    rs2::sensor rgb_sensor, depth_sensor;
    GetRgbDepthSensors(profile.get_device(), rgb_sensor, depth_sensor);

    // Configure the streams
    for (const std::pair<rs2_option, float> &option : rgbd_params_.rgb_options)
      SetOption(rgb_sensor, option.first, option.second, rgbd_params_.verbose);

    for (const std::pair<rs2_option, float> &option : rgbd_params_.depth_options)
      SetOption(depth_sensor, option.first, option.second, rgbd_params_.verbose);
VCP_LOG_FAILURE("B configured");
    // Check if we need to align the depth to the color reference view
    const cv::Size rgb_size(rgbd_params_.rgb_width, rgbd_params_.rgb_height);
    const cv::Size depth_size = rgbd_params_.align_depth_to_color
        ? rgb_size : cv::Size(rgbd_params_.depth_width, rgbd_params_.depth_height);
    rs2::align align_to(RS2_STREAM_COLOR);
    size_t consecutive_error_count = 0;

    // Set up filters
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    const bool filter_spatially = !rgbd_params_.spatial_filter_options.empty();
    for (const auto &p : rgbd_params_.spatial_filter_options)
      SetFilterOption(spatial_filter, p.first, p.second, "Spatial Filter", rgbd_params_.verbose);

    const bool filter_temporally = !rgbd_params_.temporal_filter_options.empty();
    for (const auto &p : rgbd_params_.temporal_filter_options)
      SetFilterOption(temporal_filter, p.first, p.second, "Temporal Filter", rgbd_params_.verbose);
VCP_LOG_FAILURE("C filter set");

    // Now that the pipeline started, we can query the depth scale of the sensor.
    const float depth_scale = GetDepthUnits(depth_sensor);

    // Dump calibration (now that the stream profile should know the intrinsics and extrinsics ;-)
    if (rgbd_params_.write_calibration)
      DumpCalibration(profile, rgbd_params_.calibration_file, rgbd_params_.align_depth_to_color,
                      depth_scale, rgbd_params_.verbose);
VCP_LOG_FAILURE("D calib written");
#ifdef VCP_BEST_DEBUG_FRAMERATE
    std::chrono::high_resolution_clock::time_point previous_time_point_frameset = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point previous_time_point_rgb = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point previous_time_point_depth = std::chrono::high_resolution_clock::now();
    double ms_between_frameset = -1.0;
    double ms_between_rgb = -1.0;
    double ms_between_depth = -1.0;
#endif // DEBUG_RS2_FRAMERATE

    available_ = 1;
    while(continue_capture_)
    {
      try
      {
        VCP_LOG_FAILURE("xxx wait for frames");
        rs2::frameset frameset = pipe.wait_for_frames();
        VCP_LOG_FAILURE("xxx frameset received");
        if (frameset)
        {
          // Reset error count
          consecutive_error_count = 0;

          if (rgbd_params_.align_depth_to_color)
            frameset = align_to.process(frameset);

          rs2::frame color = frameset.get_color_frame();
          rs2::frame depth = frameset.get_depth_frame();

          // Check frame numbers to skip duplicate frames!
          const auto color_fnr = color.get_frame_number();
          const auto depth_fnr = depth.get_frame_number();

          const bool is_new_color = rgb_prev_fnr_ != color_fnr;
          const bool is_new_depth = depth_prev_fnr_ != depth_fnr;

          if (!is_new_color && !is_new_depth)
          {
            VCP_LOG_FAILURE("RealSense [" << serial_number_ << "] received a frameset with duplicated color AND depth stream!");
            continue;
          }

#ifdef VCP_BEST_DEBUG_FRAMERATE
          // Measure time
          const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
          const auto duration_frameset = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_time_point_frameset);
          previous_time_point_frameset = now;
          const double ms_ema_alpha = 0.1;

          if (ms_between_frameset < 0.0)
            ms_between_frameset = duration_frameset.count();
          else
            ms_between_frameset = ms_ema_alpha * duration_frameset.count() + (1.0 - ms_ema_alpha) * ms_between_frameset;


          if (is_new_color)
          {
            const auto duration_rgb = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_time_point_rgb);
            previous_time_point_rgb = now;

            if (ms_between_rgb < 0.0)
              ms_between_rgb = duration_rgb.count();
            else
              ms_between_rgb = ms_ema_alpha * duration_rgb.count() + (1.0 - ms_ema_alpha) * ms_between_rgb;
          }

          if (is_new_depth)
          {
            const auto duration_depth = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_time_point_depth);
            previous_time_point_depth = now;

            if (ms_between_depth < 0.0)
              ms_between_depth = duration_depth.count();
            else
              ms_between_depth = ms_ema_alpha * duration_depth.count() + (1.0 - ms_ema_alpha) * ms_between_depth;
          }
          DebugFramesetMetadata(serial_number_, frameset);
#endif // VCP_BEST_DEBUG_FRAMERATE

          if (filter_spatially)
            depth = spatial_filter.process(depth);

          if (filter_temporally)
            depth = temporal_filter.process(depth);

          cv::Mat cvrgb;
          if (is_new_color)
          {
            cvrgb = FrameToMat(color, rgb_size);
            rgb_prev_fnr_ = color_fnr;
          }

          cv::Mat cvdepth;
          if (is_new_depth)
          {
            if (rgbd_params_.depth_in_meters)
              cvdepth = FrameToDepth(depth, depth_size, depth_scale);
            else
              cvdepth = FrameToMat(depth, depth_size);

            depth_prev_fnr_ = depth_fnr;
          }

          image_queue_mutex_.lock();
          if (!cvrgb.empty())
            rgb_queue_->PushBack(cvrgb.clone());
          if (!cvdepth.empty())
            depth_queue_->PushBack(cvdepth.clone());
          image_queue_mutex_.unlock();

#ifdef VCP_BEST_DEBUG_FRAMERATE
          VCP_LOG_DEBUG_DEFAULT("RealSense [" << serial_number_
              << "] frameset with new (" << (is_new_color ? "color" : "     ") << "|" << (is_new_depth ? "depth" : "     ") << ") after "
              << std::fixed << std::setprecision(1) << std::setw(6) << duration_frameset.count() << " ms, fps (frameset, col, dep): "
              << std::setw(5) << (1000.0 / ms_between_frameset) << ", "
              << std::setw(5) << (1000.0 / ms_between_rgb) << ", "
              << std::setw(5) << (1000.0 / ms_between_depth));
#endif // VCP_BEST_DEBUG_FRAMERATE
        }
        else
        {
          VCP_LOG_WARNING("Skipping empty RealSense frame!");
        }
      }
      catch (const rs2::error &e)
      {
        consecutive_error_count++;
        if (consecutive_error_count > 2)
        {
          // librealsense has a default frame timeout of 5sec, so terminate after 15 seconds (3 failed image polls)
          throw;
        }
        VCP_LOG_FAILURE("RealSense image request failed: " << e.what());
      }
    }
    pipe.stop();
  }
};


std::unique_ptr<StreamSink> CreateBufferedRealSense2Sink(const RealSense2SinkParams &params, std::unique_ptr<SinkBuffer> rgb_buffer, std::unique_ptr<SinkBuffer> depth_buffer)
{
  return std::unique_ptr<realsense2::RealSense2RGBDSink>(new realsense2::RealSense2RGBDSink(params, std::move(rgb_buffer), std::move(depth_buffer)));
}


bool IsRealSense2(const std::string &camera_type)
{
  const std::string type = vcp::utils::string::Lower(camera_type);
  return type.compare("realsense2") == 0;
}

} // namespace realsense2
} // namespace best
} // namespace vcp
