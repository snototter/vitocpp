#ifndef __VCP_BEST_RTSP_SINK_H__
#define __VCP_BEST_RTSP_SINK_H__

#include "sink.h"
#include "sink_buffer.h"
#include <vector>

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace rtsp
{
// Supported stream types.
enum class RtspStreamType
{
  MJPEG,
  H264
};

// Supported protocols, i.e. RTP over X.
// Note that our Axis P1365 cameras couldn't stream over UDP (the
// stream was always interrupted after a few seconds/minutes).
enum class RtspProtocol
{
  UDP,
  TCP
};

/**
 * @brief Parameters required to set up a RTSP stream.
 */ //FIXME change to RtspSinkParams : SinkParams!
struct RtspStreamParams
{
  // We almost exclusively receive unicast streams, RTSP over TCP is de-facto standard, see e.g. https://cardinalpeak.com/blog/the-many-ways-to-stream-video-using-rtp-and-rtsp/
  RtspStreamParams() : stream_url(""), frame_width(0), frame_height(0),
    stream_type(RtspStreamType::MJPEG), protocol(RtspProtocol::TCP),
    verbosity_level(0) {}

  // A valid rtsp:// url
  std::string stream_url;

  // Image dimension to properly initialize decoders.
  int frame_width;
  int frame_height;

  // The stream type and protocol
  RtspStreamType stream_type;
  RtspProtocol protocol;

  // Verbosity level of live555 RTSPClient (0: quiet, 1: be verbose).
  int verbosity_level;
};


/**
 * @brief Creates a StreamSink to receive an RTSP stream.
 * @param stream_url Full URL (including parameters, authentication, etc.)
 * @param sink_buffer A sink buffer which will be used as image queue.
 */
std::unique_ptr<StreamSink> CreateBufferedRtspStreamSink(const RtspStreamParams &params, std::unique_ptr<SinkBuffer> sink_buffer);


/**
 * @brief Creates a StreamSink to receive RTSP streams, specify size of the image queue as template parameter.
 * @param stream_url Full URL (including parameters, authentication, etc.)
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateRtspStreamSink(const RtspStreamParams &params)
{
  return CreateBufferedRtspStreamSink(params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}


/**
 * @brief Creates a StreamSink to receive multiple streams over RTSP (e.g. for our IP cam stereo setup).
 * @param stream_url Full URL (including parameters, authentication, etc.)
 * @param sink_buffer A sink buffer which will be used as image queue.
 */
std::unique_ptr<StreamSink> CreateBufferedMultiRtspStreamSink(const std::vector<RtspStreamParams> &params, std::vector<std::unique_ptr<SinkBuffer>> sink_buffers);


/**
 * @brief Creates a StreamSink to receive multiple RTSP streams, specify size of the image queue as template parameter.
 * @param stream_url Full URL (including parameters, authentication, etc.)
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateMultiRtspStreamSink(const std::vector<RtspStreamParams> &params)
{
  std::vector<std::unique_ptr<SinkBuffer>> sink_buffers;
  for (size_t i = 0; i < params.size(); ++i)
    sink_buffers.push_back(std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
  return CreateBufferedMultiRtspStreamSink(params, std::move(sink_buffers));
}

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp

#endif // __VCP_BEST_RTSP_SINK_H__
