#ifndef __VCP_BEST_COMMON_TYPES_H__
#define __VCP_BEST_COMMON_TYPES_H__

#include <iostream>
#include <memory>
#include <string>
#include <vector>


#include <opencv2/core/core.hpp>
#include <pvt_config/config_params.h>
#include "stream_sink.h"

namespace vcp
{
namespace best
{
/** @brief Strongly typed enum to denote what kind of data a sink (i.e. device) returns.
 * Note that:
 * - some SINKs may add multiple STREAMS (e.g. 1 RGB-D sink ==> 2 streams: one color, one depth). 
 * - watch out, some SINKS concatenate multiple STREAMS to a single one (e.g. ZED stereo camera 
 *   returns a 2*WxH image if you request the color stream; similarly, dual-optics Mobotix send
 *   images horizontally concatenated.
 */
enum class FrameType
{
  UNKNOWN,     /**< The sink doesn't know (e.g. if you load a video file and don't configure the "frame_type" parameter). */
  MONOCULAR,   /**< Most often, we deal with monocular image streams. */
  STEREO,      /**< Horizontal stereo (a single, concatenated frame consisting of the left and right image) */
  DEPTH        /**< Depth stream from an RGBD device. */
};


/** @brief String representation for FrameType. */
std::string FrameTypeToString(const FrameType &s);


/** @brief Get the FrameType based on its string representation. */
FrameType FrameTypeFromString(const std::string &s);


/** @brief Print the FrameType. */
std::ostream &operator<<(std::ostream &stream, const FrameType &s);


/** @brief Strongly typed enum to look up a sink's type. */
enum class SinkType
{
  IMAGE_DIRECTORY,
#ifdef VCP_WITH_IPCAMERA
  IPCAM_MONOCULAR,
  IPCAM_STEREO,
#endif
#ifdef VCP_WITH_K4A
  K4A,
#endif
#ifdef VCP_WITH_MATRIXVISION
  MVBLUEFOX3,
#endif
#ifdef VCP_WITH_REALSENSE2
  REALSENSE,
#endif
  VIDEO_FILE,
  WEBCAM
};

std::string SinkTypeToString(const SinkType &s);
std::ostream &operator<<(std::ostream &stream, const SinkType &s);


/** @brief Returns a default sink label if none is provided. */
std::string GetDefaultSinkLabel();

/** @brief Base class to contain the parameters for a sink (see AxisSinkParams, RealsenseSinkParams, WebcamSinkParams, etc.) */
class SinkParams
{
public:
  virtual ~SinkParams() {}

  FrameType frame_type;
  std::string label;
  std::string calibration_file;
  bool color_as_bgr;

protected:
  SinkParams()
    : stream_type(StreamType::UNKNOWN), label(GetDefaultSinkLabel()), calibration_file(), color_as_bgr(false) {}
  SinkParams(const FrameType &type, const std::string &lbl, const std::string &calib_file, bool return_bgr)
    : frame_type(type), label(lbl), calibration_file(calib_file), color_as_bgr(return_bgr) {}
};


//---------------------------------------------------------------------------------------------------------------------------
// Utils to parse a configuration file
std::string GetOptionalStringFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group, const std::string &key, const std::string &default_value);
int GetOptionalIntFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group, const std::string &key, int default_value);
unsigned int GetOptionalUnsignedIntFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group, const std::string &key, unsigned int default_value);
double GetOptionalDoubleFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group, const std::string &key, double default_value);
bool GetOptionalBoolFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group, const std::string &key, bool default_value);
std::string GetSinkLabelFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group);
std::string GetCalibrationFileFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group);
std::string GetCameraTypeFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group);
StreamType GetStreamTypeFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group);



} // namespace best
} // namespace vcp

#endif // __VCP_BEST_COMMON_TYPES_H__

