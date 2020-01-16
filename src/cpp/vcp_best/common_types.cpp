#include "common_types.h"

#include <vcp_config/config_params.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>
#include <sstream>

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
  MAKE_STREAMTYPE_TO_STRING_CASE(DEPTH);
  MAKE_STREAMTYPE_TO_STRING_CASE(UNKNOWN);
  default:
    std::stringstream str;
    str << "(" << static_cast<int>(s) << ")";
    rep = str.str();
    break;
  }

  return rep;
}


#define MAKE_STRING_TO_STREAMTYPE_IF(st, rep) if (rep.compare(FrameTypeToString(FrameType::st)) == 0) return FrameType::st
FrameType FrameTypeFromString(const std::string &s)
{
  std::string upper(s);
  vcp::utils::string::ToUpper(upper);
  MAKE_STRING_TO_STREAMTYPE_IF(MONOCULAR, upper);
  MAKE_STRING_TO_STREAMTYPE_IF(STEREO, upper);
  MAKE_STRING_TO_STREAMTYPE_IF(DEPTH, upper);
  MAKE_STRING_TO_STREAMTYPE_IF(UNKNOWN, upper);
  VCP_ERROR("FrameTypeFromString(): Cannot convert '" << s << "' to FrameType");
}


std::ostream &operator<<(std::ostream &stream, const FrameType &s)
{
  stream << "FrameType::" << FrameTypeToString(s);
  return stream;
}


std::string GetDefaultSinkLabel()
{
  static size_t counter = 0;
  std::stringstream str;
  str << "camera-" << counter;
  ++counter;
  return str.str();
}


#define MAKE_SINKTYPE_TO_STRING_CASE(st)  case SinkType::st: rep = std::string(#st); break
std::string SinkTypeToString(const SinkType &s)
{
  std::string rep;
  switch (s)
  {
  MAKE_SINKTYPE_TO_STRING_CASE(IMAGE_DIRECTORY);
#ifdef VCP_WITH_IPCAMERA
  MAKE_SINKTYPE_TO_STRING_CASE(IPCAM_MONOCULAR);
  MAKE_SINKTYPE_TO_STRING_CASE(IPCAM_STEREO);
#endif
#ifdef VCP_WITH_K4A
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
  std::string upper(s);
  vcp::utils::string::ToUpper(upper);
  MAKE_STRING_TO_SINKTYPE_IF(IMAGE_DIRECTORY, upper);
#ifdef VCP_WITH_IPCAMERA
  MAKE_STRING_TO_SINKTYPE_IF(IPCAM_MONOCULAR, upper);
  MAKE_STRING_TO_SINKTYPE_IF(IPCAM_STEREO, upper);
#endif
#ifdef VCP_WITH_K4A
  MAKE_STRING_TO_SINKTYPE_IF(K4A, upper);
#endif
#ifdef VCP_WITH_MATRIXVISION
  MAKE_STRING_TO_SINKTYPE_IF(MVBLUEFOX3, upper);
#endif
#ifdef VCP_WITH_REALSENSE2
  MAKE_STRING_TO_SINKTYPE_IF(REALSENSE, upper);
#endif
  MAKE_STRING_TO_SINKTYPE_IF(VIDEO_FILE, upper);
  MAKE_STRING_TO_SINKTYPE_IF(WEBCAM, upper);
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


std::string GetSinkLabelFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
{
  return GetOptionalStringFromConfig(config, cam_group, "label", cam_group);
}


std::string GetCalibrationFileFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
{
  return GetOptionalStringFromConfig(config, cam_group, "calibration_file", std::string());
}


std::string GetCameraTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
{
  return config.GetString(cam_group + ".type"); // mandatory field, must be set
}

FrameType GetFrameTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
{
  return FrameTypeFromString(GetOptionalStringFromConfig(config, cam_group, "frame_type", FrameTypeToString(FrameType::UNKNOWN)));
}

SinkType GetSinkTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
{
  return SinkTypeFromString(config.GetString(cam_group + ".sink_type"));
}



// TODO sink params in sink_XY.h
// TODO capture in sink_XY.h if needed (not for webcams, video files, etc.)
// TODO captures needed for multiple realsenses, multiple rtsp streams, etc.


SinkParams ParseSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
{
  const SinkType sink_type = GetSinkTypeFromConfig(config, cam_group);
  switch (sink_type)
  {
    case SinkType::IMAGE_DIRECTORY:
      //TODOreturn vcp::best::WebcamSinkFromConfig(config, cam_group);
//  #ifdef VCP_WITH_IPCAMERA
//    MAKE_STRING_TO_STREAMTYPE_IF(IPCAM_MONOCULAR, upper);
//    MAKE_STRING_TO_STREAMTYPE_IF(IPCAM_STEREO, upper);
//  #endif
//  #ifdef VCP_WITH_K4A
//    MAKE_STRING_TO_STREAMTYPE_IF(K4A, upper);
//  #endif
//  #ifdef VCP_WITH_MATRIXVISION
//    MAKE_STRING_TO_STREAMTYPE_IF(MVBLUEFOX3, upper);
//  #endif
//  #ifdef VCP_WITH_REALSENSE2
//    MAKE_STRING_TO_STREAMTYPE_IF(REALSENSE, upper);
//  #endif
    case SinkType::VIDEO_FILE:
    case SinkType::WEBCAM:
    default:
      VCP_ERROR("SinkType '" << sink_type << "' is not yet supported.");
  }
}

size_t GetNumCamerasFromConfig(const vcp::config::ConfigParams &config)
{
  if (config.SettingExists("num_cameras"))
    return static_cast<size_t>(config.GetInteger("num_cameras"));

  std::vector<std::string> params = config.ListConfigGroupParameters(std::string());
  size_t num_cameras = 0;
  for (const auto &p : params)
  {
    if (vcp::utils::string::StartsWith(p, "camera"))
      ++num_cameras;
  }
  return num_cameras;
}
} // namespace best
} // namespace vcp
