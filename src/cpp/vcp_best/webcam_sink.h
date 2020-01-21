#ifndef __VCP_BEST_WEBCAM_SINK_H__
#define __VCP_BEST_WEBCAM_SINK_H__

#include "sink.h"

namespace vcp
{
namespace best
{
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


/** @brief Returns a StreamSink wrapper to access a webcam using OpenCV's Capture capabilities. Use the templated @see CreateWebcamSink(). */
std::unique_ptr<StreamSink> CreateBufferedWebcamSink(const WebcamSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer);


template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateWebcamSink(const WebcamSinkParams &params)
{
  return CreateBufferedWebcamSink(params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace webcam
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_WEBCAM_SINK_H__
