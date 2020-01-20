#ifndef __PVT_ICC_RTSP_LIVE555_MEDIA_SINK_H__
#define __PVT_ICC_RTSP_LIVE555_MEDIA_SINK_H__

#include "rtsp_sink.h"

#include <string>

#include <liveMedia.hh>

namespace pvt
{
namespace icc
{
/**
 * @brief Base class to receive media streams via RTSP. Subclasses must provide the decoding functionality.
 */
class RtspMediaSink : public MediaSink
{
public:
  virtual ~RtspMediaSink();

protected:
  RtspMediaSink(UsageEnvironment &env, MediaSubsession &subsession, const RtspStreamParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data);

  //virtual void DecodeFrame(unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time, unsigned duration_in_microseconds, cv::Mat &decoded_frame) = 0;
  virtual void DecodeFrame(unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time, unsigned duration_in_microseconds) = 0;

  u_int8_t *receive_buffer_;
  MediaSubsession& subsession_;
//  AxisCameraParams camera_params;
  size_t num_received_frames_;
//  size_t num_snapshots;
//  std::string wnd_name;
//  std::ofstream fs_timestamps;
//  cv::VideoWriter video_writer;
//  cv::Mat latest_frame;
  bool has_been_rtcp_synchronized_;

  // Image frame dimensions (required to decode the packages).
  int frame_width_;
  int frame_height_;

  // Optional identifier.
  std::string stream_id_;

  // Callback which will be invoked after the subclasses have decoded a frame.
  void (*callback_frame_received_)(const cv::Mat &, void *);
  // User data to be passed (e.g. a pointer to the class instance ;-)
  void *callback_user_data_;

  Boolean continuePlaying() override;

  // Hook to be invoked once a data frame arrived. Must be static to match the
  // callback signature. Handling instance is passed as client_data.
  static void afterGettingFrame(void *client_data, unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time, unsigned duration_in_microseconds);

  // Invoked by realizing classes to log stats for an incoming frame (upon console)
//  void logReceivedFrame(unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time);

  // Provides a live view (and the ability to terminate by hitting ESC)
//  void showLiveStream(const cv::Mat &frame);

// Save frames:
  // Member function pointer invoked by realizations
//  bool (AxisRTSPSink::*handleOutput)(const cv::Mat &, size_t, struct timeval, bool);
//  bool outputSingleFrame(const cv::Mat &frame, size_t frame_nr, struct timeval presentation_time, bool valid_frame);
//  bool appendToVideo(const cv::Mat &frame, size_t frame_nr, struct timeval presentation_time, bool valid_frame);
//  bool outputNothing(const cv::Mat &frame, size_t frame_nr, struct timeval presentation_time, bool valid_frame);

//  inline void logTimeStamp(size_t frame_nr, struct timeval presentation_time)
//  {
//    fs_timestamps << frame_nr << ";" << presentation_time.tv_sec << ";" << presentation_time.tv_usec << ";";
//    if (!has_been_rtcp_synchronized)
//    {
//      fs_timestamps << "0";
//      if (fSubsession.rtpSource() != NULL && fSubsession.rtpSource()->hasBeenSynchronizedUsingRTCP())
//        has_been_rtcp_synchronized = true;
//    }
//    fs_timestamps << std::endl;
//  }
//  inline void logInvalidFrame(struct timeval presentation_time)
//  {
//    fs_timestamps << "-1;" << presentation_time.tv_sec << ";" << presentation_time.tv_usec << ";" << std::endl;
//  }
};

RtspMediaSink *CreateRtspMjpegMediaSink(UsageEnvironment &env, MediaSubsession &subsession, const RtspStreamParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data);
RtspMediaSink *CreateRtspH264MediaSink(UsageEnvironment &env, MediaSubsession &subsession, const RtspStreamParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data);

} // namespace icc
} // namespace pvt

#endif // __PVT_ICC_RTSP_LIVE555_MEDIA_SINK_H__
