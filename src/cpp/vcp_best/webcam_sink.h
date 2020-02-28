#ifndef __VCP_BEST_WEBCAM_SINK_H__
#define __VCP_BEST_WEBCAM_SINK_H__

#include "sink.h"

namespace vcp
{
namespace best
{
/** @brief Webcam streaming. */
namespace webcam
{
/** @brief Configuration parameters to stream from a webcam. */
struct WebcamSinkParams : public SinkParams
{
  int device_number;
  cv::Size resolution;
  double fps;

  WebcamSinkParams(
      const SinkParams &sink_params,
      int device=-1,
      const cv::Size &frame_size=cv::Size(),
      double frame_rate=-1.0)
    : SinkParams(sink_params),
      device_number(device),
      resolution(frame_size),
      fps(frame_rate)
  {}
};

std::ostream &operator<< (std::ostream &out, const WebcamSinkParams &p);


/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a WebcamSink. */
bool IsWebcamSink(const std::string &type_param);


//TODO doc
WebcamSinkParams WebcamSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);


struct WebcamDeviceInfo
{
  std::string dev_name;  // Device identifier, e.g. /dev/video1
  int device_nr;         // Device number as used by OpenCV, e.g. 1
  std::string name;      // Name from the device's metadata (queried via v4l on linux)

  /** @brief Default c'tor. */
  WebcamDeviceInfo() : dev_name("Unknown"), device_nr(-1), name("Unknown") {}

  /** @brief Comparison operator. */
  bool operator<(const WebcamDeviceInfo &other) { return device_nr < other.device_nr; }

  /** @brief Comparison operator. */
  bool operator>(const WebcamDeviceInfo &other) { return device_nr > other.device_nr; }
};

/** @brief Returns a vector of connected webcams.
 *
 * If warn_if_no_devices is true and there are no webcams connected, a warning message will be displayed upon stderr.
 * If include_incompatible_devices is true, unsupported devices will be listed too - these are USB devices which
 * couldn't be opened via OpenCV's VideoCapture (e.g. RealSense or Kinect).
 */
std::vector<WebcamDeviceInfo> ListWebcams(bool warn_if_no_devices=true, bool include_incompatible_devices=false);

/** @brief Returns a StreamSink wrapper to access a webcam using OpenCV's VideoCapture capabilities. Use the templated @see CreateWebcamSink(). */
std::unique_ptr<StreamSink> CreateBufferedWebcamSink(const WebcamSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer);

/** @brief Returns a StreamSink wrapper to access a webcam using OpenCV's VideoCapture. */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateWebcamSink(const WebcamSinkParams &params)
{
  return CreateBufferedWebcamSink(params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace webcam
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_WEBCAM_SINK_H__
