#ifndef __VCP_BEST_RTSP_CLIENT_H__
#define __VCP_BEST_RTSP_CLIENT_H__

#include <memory>
#include "rtsp_sink.h"
#include "rtsp_media_sink.h"

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace rtsp
{
/**
 * @brief Wraps the live555 environment setup, URL opening and event loop.
 */
class RtspClientEnvironment
{
public:
  virtual ~RtspClientEnvironment() {}

  /** Opens an RTSP url (specified within params), received frames will be
   * decoded by the given media_sink (which calls back to vcp::best::rtsp::RtspStreamSink)
   * The underlying live555 code takes ownership of media_sink, so do NOT
   * delete it!
   *
   * To open an SDP (session description protocol) file, pass in a "file://<path>/<to>/<file>" URL.
   */
  virtual bool OpenUrl(const IpCameraSinkParams &params,
                       void (*frame_callback)(const cv::Mat &, void *),
                       void *callback_param) = 0;

  /** @brief After you opened (@see OpenUrl) all streams, call DoEventLoop() which will block until
   * the streams are finished or shut down via TerminateEventLoop();
   */
  virtual void DoEventLoop() = 0;

  /** @brief Initiates shutdown/clean up of the RTSP client environment. Not that
   * this is a non-blocking call.
   */
  virtual void TerminateEventLoop() = 0;

protected:
  RtspClientEnvironment() {}
};

std::unique_ptr<RtspClientEnvironment> CreateRtspClientEnvironment();

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_RTSP_CLIENT_H__
