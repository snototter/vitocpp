#include "sink.h"

#include <vcp_config/config_params.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <sstream>

#include "file_sink.h"
#include "webcam_sink.h"
#ifdef VCP_BEST_WITH_K4A
    #include "k4a_sink.h"
#endif

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::sink"

namespace vcp
{
namespace best
{
#define MAKE_STREAMTYPE_TO_STRING_CASE(st) case FrameType::st: rep = std::string(#st); break
std::string FrameTypeToString(const FrameType &s)
{
  std::string rep;
  switch (s)
  {
  MAKE_STREAMTYPE_TO_STRING_CASE(MONOCULAR);
  MAKE_STREAMTYPE_TO_STRING_CASE(STEREO);
  MAKE_STREAMTYPE_TO_STRING_CASE(RGBD_IMAGE);
  MAKE_STREAMTYPE_TO_STRING_CASE(RGBD_DEPTH);
  MAKE_STREAMTYPE_TO_STRING_CASE(INFRARED);
  MAKE_STREAMTYPE_TO_STRING_CASE(UNKNOWN);
  default:
    std::stringstream str;
    str << "(" << static_cast<int>(s) << ")";
    rep = str.str();
    break;
  }

  vcp::utils::string::ToLower(rep);
  return vcp::utils::string::Replace(rep, "_", "-");
}


#define MAKE_STRING_TO_STREAMTYPE_IF(st, rep) if (rep.compare(FrameTypeToString(FrameType::st)) == 0) return FrameType::st
FrameType FrameTypeFromString(const std::string &s)
{
  std::string lower(s);
  vcp::utils::string::ToLower(lower);
  if (lower.empty()
      || lower.compare("unknown") == 0)
    return FrameType::UNKNOWN;

  if (lower.compare("monocular") == 0
      || lower.compare("mono") == 0
      || lower.compare("color") == 0
      || lower.compare("image") == 0)
    return FrameType::MONOCULAR;

  if (lower.compare("stereo") == 0)
    return FrameType::STEREO;

  if (lower.compare("rgbd-color") == 0
      || lower.compare("rgbd-image") == 0)
    return FrameType::RGBD_IMAGE;

  if (lower.compare("depth") == 0
      || lower.compare("rgbd-depth") == 0
      || lower.compare("rgbd_depth") == 0)
    return FrameType::RGBD_DEPTH;

  if (lower.compare("infrared") == 0
      || lower.compare("ir") == 0
      || lower.compare("rgbd-infrared") == 0
      || lower.compare("rgb-ir") == 0)
    return FrameType::INFRARED;

  VCP_ERROR("FrameTypeFromString(): Cannot convert '" << s << "' to FrameType");
}


std::ostream &operator<<(std::ostream &stream, const FrameType &s)
{
  stream << "FrameType::" << FrameTypeToString(s);
  return stream;
}


#define MAKE_SINKTYPE_TO_STRING_CASE(st)  case SinkType::st: rep = std::string(#st); break
std::string SinkTypeToString(const SinkType &s)
{
  std::string rep;
  switch (s)
  {
  MAKE_SINKTYPE_TO_STRING_CASE(IMAGE_DIR);
#ifdef VCP_BEST_WITH_IPCAMERA
  MAKE_SINKTYPE_TO_STRING_CASE(IPCAM_MONOCULAR);
  MAKE_SINKTYPE_TO_STRING_CASE(IPCAM_STEREO);
#endif
#ifdef VCP_BEST_WITH_K4A
  MAKE_SINKTYPE_TO_STRING_CASE(K4A);
#endif
#ifdef VCP_WITH_MATRIXVISION
  MAKE_SINKTYPE_TO_STRING_CASE(MVBLUEFOX3);
#endif
#ifdef VCP_WITH_REALSENSE2
  MAKE_SINKTYPE_TO_STRING_CASE(REALSENSE);
#endif
  MAKE_SINKTYPE_TO_STRING_CASE(VIDEO_FILE);
  MAKE_SINKTYPE_TO_STRING_CASE(WEBCAM);
  default:
    std::stringstream str;
    str << "(" << static_cast<int>(s) << ")";
    rep = str.str();
    break;
  }

  return rep;
}

#define MAKE_STRING_TO_SINKTYPE_IF(st, rep)  if (rep.compare(SinkTypeToString(SinkType::st)) == 0) return SinkType::st
SinkType SinkTypeFromString(const std::string &s)
{
  if (file::IsImageDirectorySink(s))
    return SinkType::IMAGE_DIR;
  else if (file::IsVideoFileSink(s))
    return SinkType::VIDEO_FILE;
  else if (webcam::IsWebcamSink(s))
    return SinkType::WEBCAM;
  else if (k4a::IsK4A(s))
    return SinkType::K4A;
//  else if (ipcam::IsIpCamMono())//FIXME
//  std::string upper(s);
//  vcp::utils::string::ToUpper(upper);
//  MAKE_STRING_TO_SINKTYPE_IF(IMAGE_DIR, upper);
//#ifdef VCP_WITH_IPCAMERA
//  MAKE_STRING_TO_SINKTYPE_IF(IPCAM_MONOCULAR, upper);
//  MAKE_STRING_TO_SINKTYPE_IF(IPCAM_STEREO, upper);
//#endif
//#ifdef VCP_BEST_WITH_K4A
//  MAKE_STRING_TO_SINKTYPE_IF(K4A, upper);
//#endif
//#ifdef VCP_WITH_MATRIXVISION
//  MAKE_STRING_TO_SINKTYPE_IF(MVBLUEFOX3, upper);
//#endif
//#ifdef VCP_WITH_REALSENSE2
//  MAKE_STRING_TO_SINKTYPE_IF(REALSENSE, upper);
//#endif
//  MAKE_STRING_TO_SINKTYPE_IF(VIDEO_FILE, upper);
//  MAKE_STRING_TO_SINKTYPE_IF(WEBCAM, upper);
  VCP_ERROR("SinkTypeFromString(): Representation '" << s << "' not yet handled.");
}

std::ostream &operator<<(std::ostream &stream, const SinkType &s)
{
  stream << "SinkType::" << SinkTypeToString(s);
  return stream;
}


inline std::string GetConfigKey(const std::string &cam_group, const std::string &key)
{
  if (key.empty())
    VCP_ERROR("GetConfigKey(): Key cannot be empty!");
  if (cam_group.empty())
    return key;
  return cam_group + "." + key;
}

std::string GetOptionalStringFromConfig(
    const vcp::config::ConfigParams &config,
    const std::string &cam_group,
    const std::string &key,
    const std::string &default_value)
{
  const std::string lookup = GetConfigKey(cam_group, key);
  if (config.SettingExists(lookup))
    return config.GetString(lookup);
  return default_value;
}


int GetOptionalIntFromConfig(const config::ConfigParams &config, const std::string &cam_group, const std::string &key, int default_value)
{
  const std::string lookup = GetConfigKey(cam_group, key);
  if (config.SettingExists(lookup))
    return config.GetInteger(lookup);
  return default_value;
}


unsigned int GetOptionalUnsignedIntFromConfig(const config::ConfigParams &config,
    const std::string &cam_group, const std::string &key, unsigned int default_value)
{
  const std::string lookup = GetConfigKey(cam_group, key);
  if (config.SettingExists(lookup))
    return config.GetUnsignedInteger(lookup);
  return default_value;
}


double GetOptionalDoubleFromConfig(const config::ConfigParams &config, const std::string &cam_group, const std::string &key, double default_value)
{
  const std::string lookup = GetConfigKey(cam_group, key);
  if (config.SettingExists(lookup))
    return config.GetDouble(lookup);
  return default_value;
}

bool GetOptionalBoolFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, bool default_value)
{
  const std::string lookup = GetConfigKey(cam_group, key);
  if (config.SettingExists(lookup))
    return config.GetBoolean(lookup);
  return default_value;
}


std::string GetSinkLabelFromConfig(const vcp::config::ConfigParams &config,
                                   const std::string &cam_group,
                                   std::vector<std::string> &configured_keys)
{
  // "cam_group" must have a unique name, or the config cannot be loaded
  // by libconfig++, so use it as the default/fallback value.
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "label"), configured_keys.end());
  return GetOptionalStringFromConfig(config, cam_group, "label", cam_group);
}


std::string GetCalibrationFileFromConfig(const vcp::config::ConfigParams &config,
                                         const std::string &cam_group,
                                         std::vector<std::string> &configured_keys)
{
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "calibration_file"), configured_keys.end());
  return GetOptionalStringFromConfig(config, cam_group, "calibration_file", std::string());
}


bool GetColorAsBgrFromConfig(const vcp::config::ConfigParams &config,
                             const std::string &cam_group,
                             std::vector<std::string> &configured_keys)
{
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "color_as_bgr"), configured_keys.end());
  return GetOptionalBoolFromConfig(config, cam_group, "color_as_bgr", false);
}


bool GetVerbosityFlagFromConfig(const vcp::config::ConfigParams &config,
                                const std::string &cam_group,
                                std::vector<std::string> &configured_keys)
{
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "verbose"), configured_keys.end());
  return GetOptionalBoolFromConfig(config, cam_group, "verbose", false);
}



FrameType GetFrameTypeFromConfig(const vcp::config::ConfigParams &config,
                                 const std::string &cam_group,
                                 std::vector<std::string> &configured_keys)
{
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_type"), configured_keys.end());
  return FrameTypeFromString(GetOptionalStringFromConfig(config, cam_group, "frame_type", FrameTypeToString(FrameType::UNKNOWN)));
}

SinkType GetSinkTypeFromConfig(const vcp::config::ConfigParams &config,
                               const std::string &cam_group,
                               std::vector<std::string> *configured_keys)
{
  if (configured_keys)
    configured_keys->erase(std::remove(configured_keys->begin(), configured_keys->end(), "sink_type"), configured_keys->end());
  if (config.SettingExists(cam_group + ".sink_type"))
    return SinkTypeFromString(config.GetString(cam_group + ".sink_type"));
  if (config.SettingExists(cam_group + ".camera_type"))
    return SinkTypeFromString(config.GetString(cam_group + ".camera_type"));
  VCP_LOG_WARNING("Mandatory configuration parameter '" << cam_group << ".sink_type' (or '.camera_type') is not specified, looking up the obsolete '" << cam_group << ".type'.");

  if (configured_keys)
    configured_keys->erase(std::remove(configured_keys->begin(), configured_keys->end(), "type"), configured_keys->end());
  if (config.SettingExists(cam_group + ".type"))
    return SinkTypeFromString(config.GetString(cam_group + ".type"));

  VCP_ERROR("Invalid configuration: Neither '" << cam_group << ".sink_type' nor the (obsolete) '" << cam_group << ".type' has been set. Cannot deduce sink type.");
}


void WarnOfUnusedParameters(const std::string &cam_group, const std::vector<std::string> &unused_parameters)
{
  if (!unused_parameters.empty())
  {
    std::stringstream s;
    s << "'" << unused_parameters[0] << "'";
    for (size_t i = 1; i < unused_parameters.size(); ++i)
      s << ", '" << unused_parameters[i] << "'";
    VCP_LOG_WARNING("There are unused parameters for '" << cam_group << "': " << s.str());
  }
}

// TODO captures needed for multiple realsenses, multiple rtsp streams, etc.

SinkParams ParseBaseSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, std::vector<std::string> &configured_keys)
{
  const SinkType sink_type = GetSinkTypeFromConfig(config, cam_group, &configured_keys);
  const FrameType frame_type = GetFrameTypeFromConfig(config, cam_group, configured_keys);
  const std::string sink_label = GetSinkLabelFromConfig(config, cam_group, configured_keys);
  const std::string calibration_file = GetCalibrationFileFromConfig(config, cam_group, configured_keys);
  const bool color_as_bgr = GetColorAsBgrFromConfig(config, cam_group, configured_keys);
  const bool verbose = GetVerbosityFlagFromConfig(config, cam_group, configured_keys);

  return SinkParams(sink_type, frame_type, sink_label, calibration_file, cam_group, color_as_bgr, verbose);
}

size_t GetNumCamerasFromConfig(const vcp::config::ConfigParams &config)
{
  if (config.SettingExists("num_cameras"))
    return static_cast<size_t>(config.GetInteger("num_cameras"));

  const std::vector<std::string> params = GetCameraConfigParameterNames(config);
  return params.size();
}

std::vector<std::string> GetCameraConfigParameterNames(const vcp::config::ConfigParams &config)
{
  std::vector<std::string> camera_parameters;
  // Get all first-level parameters:
  std::vector<std::string> params = config.ListConfigGroupParameters(std::string());
  // Sort the parameter names:
  std::sort(params.begin(), params.end(), vcp::utils::file::filename_filter::CompareFileLengthsAndNames);

  for (const auto &p : params)
  {
    if (vcp::utils::string::StartsWith(p, "camera")
        || vcp::utils::string::StartsWith(p, "sink"))
      camera_parameters.push_back(p);
  }
  return camera_parameters;
}
} // namespace best
} // namespace vcp
