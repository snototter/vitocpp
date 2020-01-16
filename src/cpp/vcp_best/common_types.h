#ifndef __VCP_BEST_COMMON_TYPES_H__
#define __VCP_BEST_COMMON_TYPES_H__

#include <iostream>
#include <memory>
#include <string>
#include <vector>


#include <opencv2/core/core.hpp>
#include <vcp_config/config_params.h>
//#include "stream_sink.h"

namespace vcp
{
/** @brief BESt (Best Effort Streaming) module.
 *
 * Configuration can easily be done via:
 * libconfig++
 * TODO refer to examples/data/best/*.cfg
 * or python bindings/demo
 */
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
//TODO change generic IPCAM_MONO to AXIS, etc.
enum class SinkType
{
  IMAGE_DIR,
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

  SinkType sink_type;
  FrameType frame_type;
  std::string label;
  std::string calibration_file;
  bool color_as_bgr;

protected:
  SinkParams(const SinkType &stype,
      const FrameType &ftype,
      const std::string &lbl,
      const std::string &calib_file=std::string(),
      bool return_bgr=false)
    : sink_type(stype), frame_type(ftype),
      label(lbl), calibration_file(calib_file),
      color_as_bgr(return_bgr)
  {}
};


//TODO hide from public interface
////---------------------------------------------------------------------------------------------------------------------------
//// Utils to parse a configuration file
///** @brief Look up an optional string with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
//std::string GetOptionalStringFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, const std::string &default_value);

///** @brief Look up an optional integer with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
//int GetOptionalIntFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, int default_value);

///** @brief Look up an optional unsigned integer with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
//unsigned int GetOptionalUnsignedIntFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, unsigned int default_value);

///** @brief Look up an optional double with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
//double GetOptionalDoubleFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, double default_value);

///** @brief Look up an optional boolean with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
//bool GetOptionalBoolFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, bool default_value);

//std::string GetSinkLabelFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);
//std::string GetCalibrationFileFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);
//std::string GetCameraTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);
//FrameType GetFrameTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);

SinkParams ParseSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);


/** @brief Returns the num_cameras parameter if configured. Otherwise, counts the "cameraX" entries. */
size_t GetNumCamerasFromConfig(const vcp::config::ConfigParams &config);
} // namespace best
} // namespace vcp

#endif // __VCP_BEST_COMMON_TYPES_H__

