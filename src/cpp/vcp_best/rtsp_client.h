#ifndef __VCP_BEST_RTSP_CLIENT_H__
#define __VCP_BEST_RTSP_CLIENT_H__

#include <memory>
#include "rtsp_sink.h"
#include "rtsp_media_sink.h"
#include <opencv2/core/core.hpp>

namespace vcp
{
namespace best
{
namespace ipcam
{
/**
 * @brief Wraps live555 environment setup, URL opening and event loop.
 */
class RtspClientEnvironment
{
public:
  virtual ~RtspClientEnvironment() {}

  // Opens an RTSP url (specified within params), received frames will be
  // decoded by the given media_sink (which calls back to pvt::icc::StreamSink)
  // The underlying live555 code takes ownership of media_sink, so do NOT
  // delete it!
  virtual void OpenUrl(const RtspStreamParams &params, void (*frame_callback)(const cv::Mat &, void *), void *callback_param) = 0;

  // Opens the SDP file (params.stream_url) and initializes the video stream.
//  virtual void OpenSdpFile(const RtspStreamParams &params, void (*frame_callback)(const cv::Mat &, void *), void *callback_param) = 0;

  // After all streams are opened, call DoEventLoop() which will block until
  // the streams are finished or shut down via TerminateEventLoop();
  virtual void DoEventLoop() = 0;

  virtual void TerminateEventLoop() = 0;


protected:
  RtspClientEnvironment() {}
};

std::unique_ptr<RtspClientEnvironment> CreateRtspClientEnvironment();

} // namespace ipcam
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_RTSP_CLIENT_H__
