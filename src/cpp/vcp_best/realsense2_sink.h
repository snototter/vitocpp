#ifndef __VCP_BEST_REALSENSE2_SINK_H__
#define __VCP_BEST_REALSENSE2_SINK_H__

#include "sink.h"
#include "sink_buffer.h"

#include <librealsense2/rs.hpp>
#include <vector>
#include <string>
#include <utility>

namespace vcp
{
namespace best
{
/** @brief RealSense RGBD streaming. */
namespace realsense2
{
/** @brief String representation of an empty/invalid/unset serial number. Use this if you don't want to specify a particular device. */
extern const std::string kEmptyRealSense2SerialNumber;

/** @brief Strongly typed enum to select which RealSense streams to enable. */
enum class RSStreamType
{
  COLOR_DEPTH,      /**< Both RGB and depth. */
  COLOR_DEPTH_IRL,  /**< RGB, depth and left IR. */
  COLOR_DEPTH_IR2,  /**< RGB, depth and both IR. */
  COLOR,            /**< Only RGB. */
  DEPTH,            /**< Only depth. */
  DEPTH_IRL,        /**< Only depth and left IR. */
  DEPTH_IR2,        /**< Only depth and both IR. */
  INFRARED_LEFT,    /**< Only left IR. */
  INFRARED_RIGHT,   /**< Only left IR. */
  INFRARED2         /**< Both infrared images only. */
};


/** @brief String representation for RSStreamType. */
std::string RSStreamTypeToString(const RSStreamType &s);


/** @brief Get the RSStreamType based on its string representation. */
RSStreamType RSStreamTypeFromString(const std::string &s);


/** @brief Print the RSStreamType. */
std::ostream &operator<<(std::ostream &stream, const RSStreamType &s);


/** @brief Parametrization of a RealSense device. */
struct RealSense2SinkParams : SinkParams
{
  std::string serial_number; /**< Set to kEmptyRealSense2SerialNumber to apply these params to the first connected RealSense device. */
  RSStreamType stream_type;
  int rgb_frame_rate;
  int rgb_width;
  int rgb_height;
  int depth_frame_rate;
  int depth_width;
  int depth_height;

  bool align_depth_to_color; /**< Align depth frame to the RGB frame if set. */
  bool depth_in_meters;      /**< Convert depth to meters (otherwise the plain Z16 values will be returned). */

  bool write_calibration;       /**< If set, the (factory set) calibration will be written to the calibration_file. */

  // We use a vector to load the user-defined sensor options, since the order
  // of changing these options is important (e.g. if you mistakenly set
  // auto_exposure and later on define an explicit exposure time)
  std::vector<std::pair<rs2_option, float>> rgb_options;
  std::vector<std::pair<rs2_option, float>> depth_options;

  // Check rs-post-processing example (binary in librealsense2) for suitable filter settings
  std::vector<std::pair<rs2_option, float>> spatial_filter_options; // Leave empty to disable filter
  std::vector<std::pair<rs2_option, float>> temporal_filter_options; // Leave empty to disable filter

  RealSense2SinkParams(const SinkParams &params,
                   const std::string serial_number=kEmptyRealSense2SerialNumber,
                   const RSStreamType &stream_type=RSStreamType::COLOR_DEPTH,
                   const int rgb_frame_rate=30,
                   const int rgb_width=1920,
                   const int rgb_height=1080,
                   const int depth_frame_rate=30,
                   const int depth_width=1280,
                   const int depth_height=720,
                   const bool align_depth_to_color=true,
                   const bool depth_in_meters=false,
                   const bool write_calibration=false)
    : SinkParams(params),
      serial_number(serial_number),
      stream_type(stream_type),
      rgb_frame_rate(rgb_frame_rate), rgb_width(rgb_width), rgb_height(rgb_height),
      depth_frame_rate(depth_frame_rate), depth_width(depth_width), depth_height(depth_height),
      align_depth_to_color(align_depth_to_color), depth_in_meters(depth_in_meters),
      write_calibration(write_calibration),
      rgb_options(), depth_options(),
      spatial_filter_options(), temporal_filter_options()
  {}

  bool IsColorStreamEnabled() const;
  bool IsDepthStreamEnabled() const;
  bool IsInfrared1StreamEnabled() const;
  bool IsInfrared2StreamEnabled() const;
};


/** @brief Brief device info returned from \c ListRealSense2Devices(). */
struct RealSense2DeviceInfo
{
  std::string serial_number;
  std::string name; // Human readable string

  RealSense2DeviceInfo() : serial_number(kEmptyRealSense2SerialNumber), name("Unknown") {}
};


/** @brief Parses the configuration group "cam_param" into a RealSense2SinkParams configuration. */
RealSense2SinkParams RealSense2SinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);


/**
 * @brief Creates a StreamSink to capture from an Intel RealSense device using librealsense2.
 * @param sink_buffer A sink buffer which will be used as image queue.
 */
std::unique_ptr<StreamSink> CreateBufferedRealSense2Sink(const RealSense2SinkParams &params,
        std::unique_ptr<SinkBuffer> rgb_buffer, std::unique_ptr<SinkBuffer> depth_buffer,
        std::unique_ptr<SinkBuffer> ir1_buffer, std::unique_ptr<SinkBuffer> ir2_buffer);

/**
 * @brief Creates a StreamSink to capture from an Intel RealSense device, specify size of the image queue as template parameter.
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateRealSense2Sink(const RealSense2SinkParams &params)
{
  return CreateBufferedRealSense2Sink(params,
          std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
          std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
          std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
          std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}


/** @brief Returns a list of connected RealSense devices. */
std::vector<RealSense2DeviceInfo> ListRealSense2Devices(bool warn_if_no_devices=true);


/** @brief Checks if the configuration belongs to a RealSense2 device (using the configuration parameter "cameraX.type"). */
bool IsRealSense2(const std::string &camera_type);

} // namespace realsense2
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_REALSENSE2_SINK_H__
