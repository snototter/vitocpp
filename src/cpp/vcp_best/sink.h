#ifndef __VCP_BEST_SINK_H__
#define __VCP_BEST_SINK_H__

#include <string>
#include <memory>
#include <exception>
#include <opencv2/core/core.hpp>
#include <vcp_config/config_params.h>
#include "sink_buffer.h"

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
  RGBD_IMAGE,  /**< Image stream from an RGBD device. */
  RGBD_DEPTH,  /**< Depth stream from an RGBD device, usually uint16. */
  INFRARED     /**< Infrared stream (intensity values, usually uint16). */
};


/** @brief String representation for FrameType. */
std::string FrameTypeToString(const FrameType &s);


/** @brief Get the FrameType based on its string representation. */
FrameType FrameTypeFromString(const std::string &s);


/** @brief Print the FrameType. */
std::ostream &operator<<(std::ostream &stream, const FrameType &s);

//FIXME move to sink
/** @brief Strongly typed enum to look up a sink's type. */
//TODO change generic IPCAM_MONO to AXIS, etc.
enum class SinkType
{
  IMAGE_DIR,
#ifdef VCP_BEST_WITH_IPCAMERA
  IPCAM_MONOCULAR,
  IPCAM_STEREO,
#endif
#ifdef VCP_BEST_WITH_K4A
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


///** @brief Returns a default sink label if none is provided. */
//std::string GetDefaultSinkLabel();

/** @brief Base class to contain the parameters for a sink (see AxisSinkParams, RealsenseSinkParams, WebcamSinkParams, etc.) */
struct SinkParams
{
  /** @brief Type of this device (or how it is used, e.g. IP cameras may be used as monochrome or stereo setups). */
  SinkType sink_type;

  /** @brief Type of the return frame(s). This parameter is used to tell such IP mono/stereo setups apart.
   * Other devices, e.g. RGBD sensors won't need it actually (as they will usually provide both
   * a depth and RGB stream).
   * Thus, you should always use Sink::Type(stream_index) instead of trusting this field!
   */
  FrameType frame_type;

  /** @brief Label assigned to this sink. */
  std::string sink_label;

  /** @brief Path to the intrinsic calibration file. */
  std::string calibration_file;

  /** @brief Name of the configuration file parameter (where this sink was configured). */
  std::string configuration_key;

  /** @brief Should color images be returned as BGR (OpenCV) or RGB channels? */
  bool color_as_bgr;

  /** @brief Should sink/device wrapper log verbosely? */
  bool verbose;


  SinkParams(const SinkParams &other)
    : sink_type(other.sink_type), frame_type(other.frame_type),
      sink_label(other.sink_label), calibration_file(other.calibration_file),
      configuration_key(other.configuration_key), color_as_bgr(other.color_as_bgr),
      verbose(other.verbose)
  {}

  SinkParams(const SinkType &stype,
      const FrameType &ftype,
      const std::string &lbl,
      const std::string &calib_file=std::string(),
      const std::string &config_key=std::string(),
      bool return_bgr=false, bool verbose=false)
    : sink_type(stype), frame_type(ftype),
      sink_label(lbl), calibration_file(calib_file),
      configuration_key(config_key),
      color_as_bgr(return_bgr), verbose(verbose)
  {}
};

/**
 * @brief Abstract base class for all device sinks.
 *
 FIXME adapt doc* Intended use:
 * * Instantiate (via factory, pass configuration parameters).
 * * StartStream() starts a separate std::thread to receive camera stream.
 * * GetNextFrame() retrieves the next unprocessed frame (or empty if no frame
 *   is available). <tt>Note</tt> that the actual sink might use a limited
 *   image queue (i.e. the next frame is only guaranteed to be the oldest
 *   within the sink's buffer).
 * * GetMostRecentFrame() retrieves the most recent frame (or empty if no frame
 *   is available).
 * * Terminate() stops the stream and cleans up.
 */
class StreamSink
{
protected:
  StreamSink() {}

public:
  virtual ~StreamSink() {}

  virtual bool OpenDevice() = 0;
  virtual bool StartStreaming() = 0;
  virtual bool StopStreaming() = 0;
  virtual bool CloseDevice() = 0;

  virtual std::vector<cv::Mat> Next() = 0;

  /** @brief Classes which support backwards seeking must overwrite this method. */
  virtual std::vector<cv::Mat> Previous()
  {
    throw std::runtime_error("Sink does not support stream seeking!");
  }


  /** @brief Classes which support skipping frames must overwrite this method. */
  virtual std::vector<cv::Mat> FastForward(size_t /*num_frames*/)
  {
    throw std::runtime_error("Sink does not support stream seeking!");
  }

  /** @brief Returns 1 (true) if the physical device is ready/can be used to stream from.
   *
   * Note that the return value must be an int, because std::atomic uses stdbool.h (which
   * redefines "bool" and thus, would break the build). As some sinks use std::atomic, we
   * use this C-style workaround.
   */
  virtual int IsDeviceAvailable() const = 0;

  /** @brief Returns the number of currently available frames (i.e. number of "streams" that
   * are available). For most sinks, this will be 0 or 1. For RGBD or stereo sensors this
   * number may be higher.
   * Note that this does not return the size of the current frame queue for a single "stream".
   * For example, if 15 RGB frames are enqueued and 10 depth frames (you should be worried about
   * such a "totally off"-scenario, btw), this method returns 2 (there's both RGB and depth available).
   */
  virtual size_t NumAvailableFrames() const = 0;

  /** @brief Informs you whether the image queue is filled or empty. */
  virtual int IsFrameAvailable() const = 0;

  /** @brief Returns the number of streams/frames this sink delivers upon Next() - or, if supported, Previous() and FastForward(). */
  virtual size_t NumStreams() const = 0;

  //TODO doc
  //virtual SinkType Type(size_t stream_index) const = 0;
  virtual FrameType FrameTypeAt(size_t stream_index) const = 0;

  virtual std::string StreamLabel(size_t stream_index) const = 0;
};



//---------------------------------------------------------------------------------------------------------------------------
// Utils to parse a configuration file
/** @brief Look up an optional string with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
std::string GetOptionalStringFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, const std::string &default_value);

/** @brief Look up an optional integer with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
int GetOptionalIntFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, int default_value);

/** @brief Look up an optional unsigned integer with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
unsigned int GetOptionalUnsignedIntFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, unsigned int default_value);

/** @brief Look up an optional double with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
double GetOptionalDoubleFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, double default_value);

/** @brief Look up an optional boolean with parameter name [cam_group].[key]. To look up only [key], [cam_group] can be empty. */
bool GetOptionalBoolFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, const std::string &key, bool default_value);


SinkType GetSinkTypeFromConfig(const vcp::config::ConfigParams &config,
                               const std::string &cam_group,
                               std::vector<std::string> *configured_keys=nullptr);

void WarnOfUnusedParameters(const std::string &cam_group, const std::vector<std::string> &unused_parameters);

// Should only be available to the base sinkparams/abstract sink class
//std::string GetSinkLabelFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);
//std::string GetCalibrationFileFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);
//std::string GetCameraTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);
//bool GetColorAsBgrFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group)
//FrameType GetFrameTypeFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group);

/** @brief Extracts the common SinkParams from the given configuration.
 *
 * Only extracts the common camera/device params contained within SinkParams from
 * the config group named "cam_group", e.g. "camera3".
 * All found keys/parameter names will be ERASED from the input "configured_keys" (so
 * you can check if the user forgot some parameters/made typos/etc.)
 */
SinkParams ParseBaseSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_group, std::vector<std::string> &configured_keys);


/** @brief Returns the num_cameras parameter if configured. Otherwise, counts the "cameraX" entries. */
size_t GetNumCamerasFromConfig(const vcp::config::ConfigParams &config);

/** @brief Returns all first-level config parameters which start which start with "camera" prefix. */
std::vector<std::string> GetCameraConfigParameterNames(const vcp::config::ConfigParams &config);

} // namespace best
} // namespace vcp
#endif // __VCP_BEST_SINK_H__
