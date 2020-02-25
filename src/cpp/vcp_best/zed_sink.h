#ifndef __VCP_BEST_ZED_SINK_H__
#define __VCP_BEST_ZED_SINK_H__

#include "sink.h"
#include <vcp_utils/enum_utils.h>
#include <sl/Camera.hpp>

namespace vcp
{
namespace best
{
/** @brief TODO doc. */
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

  //TODO init by dev-id
  //TODO init by serial-nr
  unsigned int serial_number;
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
      unsigned int serial_number=std::numeric_limits<unsigned int>::max()
      )
    : SinkParams(sink_params),
      enabled_streams(streams),
      resolution(resolution),
      depth_mode(depth_mode),
      depth_in_meters(depth_in_meters),
      depth_stabilization(depth_stabilization),
      fps(frame_rate),
      flip_image(flip_image),
      disable_self_calibration(disable_self_calibration),

      serial_number(serial_number),
      model_name(std::string())
  {}

  bool IsLeftStreamEnabled() const;
  bool IsRightStreamEnabled() const;
  bool IsDepthStreamEnabled() const;
};

std::ostream &operator<< (std::ostream &out, const ZedSinkParams &p);


/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a WebcamSink. */
bool IsZedSink(const std::string &type_param);


//TODO doc
ZedSinkParams ZedSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);


//struct WebcamDeviceInfo
//{
//  std::string dev_name;  // Device identifier, e.g. /dev/video1
//  int device_nr;         // Device number as used by OpenCV, e.g. 1
//  std::string name;      // Name from the device's metadata (queried via v4l on linux)

//  /** @brief Default c'tor. */
//  WebcamDeviceInfo() : dev_name("Unknown"), device_nr(-1), name("Unknown") {}

//  /** @brief Comparison operator. */
//  bool operator<(const WebcamDeviceInfo &other) { return device_nr < other.device_nr; }

//  /** @brief Comparison operator. */
//  bool operator>(const WebcamDeviceInfo &other) { return device_nr > other.device_nr; }
//};

///** @brief Returns a vector of connected webcams.
// *
// * If warn_if_no_devices is true and there are no webcams connected, a warning message will be displayed upon stderr.
// * If include_incompatible_devices is true, unsupported devices will be listed too - these are USB devices which
// * couldn't be opened via OpenCV's VideoCapture (e.g. RealSense or Kinect).
// */
//std::vector<WebcamDeviceInfo> ListZedSensors(bool warn_if_no_devices=true);

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
