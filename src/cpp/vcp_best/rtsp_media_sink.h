#ifndef __VCP_BEST_RTSP_LIVE555_MEDIA_SINK_H__
#define __VCP_BEST_RTSP_LIVE555_MEDIA_SINK_H__

#include "rtsp_sink.h"

#include <string>

#include <liveMedia.hh>

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace rtsp
{
/**
 * @brief Base class to receive media streams via RTSP. Subclasses must provide the decoding functionality.
 */
class RtspMediaSink : public MediaSink
{
public:
  virtual ~RtspMediaSink();

protected:
  RtspMediaSink(UsageEnvironment &env, MediaSubsession &subsession, const IpCameraSinkParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data);

  virtual void DecodeFrame(unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time, unsigned duration_in_microseconds) = 0;

  u_int8_t *receive_buffer_;
  MediaSubsession& subsession_;
  size_t num_received_frames_;
  bool has_been_rtcp_synchronized_;

  // Image frame dimensions (required to decode the data packages).
  int frame_width_;
  int frame_height_;

  // Optional identifier.
  std::string stream_id_;

  // IP sinks usually stream color as BGR, the user, however, may want RGB frames.
  bool color_as_rgb_;

  // Callback which will be invoked after the subclasses have decoded a frame.
  void (*callback_frame_received_)(const cv::Mat &, void *);
  // User data to be passed (e.g. a pointer to the class instance ;-)
  void *callback_user_data_;

  Boolean continuePlaying() override;

  // Hook to be invoked once a data frame arrived. Must be static to match the
  // callback signature. Handling instance is passed as client_data.
  static void afterGettingFrame(void *client_data, unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time, unsigned duration_in_microseconds);
};

RtspMediaSink *CreateRtspMjpegMediaSink(UsageEnvironment &env,
                                        MediaSubsession &subsession,
                                        const IpCameraSinkParams &params,
                                        void (*callback_frame_received)(const cv::Mat &, void *),
                                        void *callback_user_data);
RtspMediaSink *CreateRtspH264MediaSink(UsageEnvironment &env,
                                       MediaSubsession &subsession,
                                       const IpCameraSinkParams &params,
                                       void (*callback_frame_received)(const cv::Mat &, void *),
                                       void *callback_user_data);

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp

#endif // __VCP_BEST_RTSP_LIVE555_MEDIA_SINK_H__
