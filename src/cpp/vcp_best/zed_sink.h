#ifndef __VCP_BEST_ZED_SINK_H__
#define __VCP_BEST_ZED_SINK_H__

#include "sink.h"
#include <vcp_utils/enum_utils.h>
#include <sl/Camera.hpp>

namespace vcp
{
namespace best
{
/** @brief ZED stereo camera streaming. */
namespace zed
{

enum class ZedStreams : int
{
  NONE = 0,
  LEFT = 1,
  RIGHT = 2,
  DEPTH = 4
};

/** @brief Configuration parameters to stream from a webcam. */
struct ZedSinkParams : public SinkParams
{
  ZedStreams enabled_streams;
  sl::RESOLUTION resolution;
  sl::DEPTH_MODE depth_mode;
  bool depth_in_meters;
  bool depth_stabilization;

  int fps;
  bool flip_image;
  bool disable_self_calibration;

  sl::SENSING_MODE sensing_mode;
  int confidence_threshold;
  int textureness_threshold;
  bool enable_image_enhancement;

  bool write_calibration;

  int gpu_id;
  unsigned int serial_number;
  int device_id;
  std::string model_name;

  ZedSinkParams(
      const SinkParams &sink_params,
      const ZedStreams &streams=ZedStreams::LEFT | ZedStreams::DEPTH,
      const sl::RESOLUTION &resolution=sl::RESOLUTION::HD720,
      const sl::DEPTH_MODE &depth_mode=sl::DEPTH_MODE::ULTRA,
      const bool depth_in_meters=false,
      const bool depth_stabilization=true,
      const int frame_rate=0,
      const bool flip_image=false,
      const bool disable_self_calibration=false,
      const sl::SENSING_MODE sensing_mode=sl::SENSING_MODE::STANDARD,
      const int confidence_threshold=100,
      const int textureness_threshold=100,
      const bool enable_image_enhancement=true,
      const bool write_calibration=false,
      const int gpu_id=-1,
      unsigned int serial_number=std::numeric_limits<unsigned int>::max(),
      int device_id=-1)
    : SinkParams(sink_params),
      enabled_streams(streams),
      resolution(resolution),
      depth_mode(depth_mode),
      depth_in_meters(depth_in_meters),
      depth_stabilization(depth_stabilization),
      fps(frame_rate),
      flip_image(flip_image),
      disable_self_calibration(disable_self_calibration),
      sensing_mode(sensing_mode),
      confidence_threshold(confidence_threshold),
      textureness_threshold(textureness_threshold),
      enable_image_enhancement(enable_image_enhancement),
      write_calibration(write_calibration),
      gpu_id(gpu_id),
      serial_number(serial_number),
      device_id(device_id),
      model_name(std::string())
  {}

  bool IsLeftStreamEnabled() const;
  bool IsRightStreamEnabled() const;
  bool IsDepthStreamEnabled() const;
};

std::ostream &operator<< (std::ostream &out, const ZedSinkParams &p);


/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a ZED stereo camera. */
bool IsZedSink(const std::string &type_param);


/** @brief Parse a ZED stereo camera configuration from the config group "cam_param". */
ZedSinkParams ZedSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);


struct ZedDeviceInfo
{
  unsigned int serial_number; /**< Serial number of the device. */
  bool available;           /**< True if available, false if already in use. */
  std::string device_path;  /**< The system device path. */
  std::string model_name;   /**< The model name (e.g. ZED2). */
};


/** @brief Returns a vector of connected ZED stereo cameras.
 *
 * If warn_if_no_devices is true and there are no webcams connected, a warning message will be displayed upon stderr.
 */
std::vector<ZedDeviceInfo> ListZedDevices(bool warn_if_no_devices=true, bool list_unavailable_devices=false);

/** @brief Returns a StreamSink to access a ZED stereo cam. Use the templated @see CreateZedStereoSink(). */
std::unique_ptr<StreamSink> CreateBufferedZedSink(const ZedSinkParams &params,
                    std::unique_ptr<SinkBuffer> sink_buffer_left, std::unique_ptr<SinkBuffer> sink_buffer_right,
                    std::unique_ptr<SinkBuffer> sink_buffer_depth);


/** @brief Returns a StreamSink to access a ZED stereo cam. */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateZedSink(const ZedSinkParams &params)
{
  return CreateBufferedZedSink(params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
                               std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
                               std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace zed
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_ZED_SINK_H__
