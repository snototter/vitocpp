#ifndef __VCP_BEST_RTSP_SINK_H__
#define __VCP_BEST_RTSP_SINK_H__

#include "ipcam_sink.h"
#include "sink_buffer.h"
#include <vector>

namespace vcp
{
namespace best
{
namespace ipcam
{
/** @brief TODO doc. */
namespace rtsp
{
/**
 * @brief Creates a StreamSink to receive multiple streams over RTSP (e.g. for our IP cam stereo setup).
 */
std::unique_ptr<StreamSink> CreateBufferedMultiRtspStreamSink(const std::vector<IpCameraSinkParams> &params,
                                                              std::vector<std::unique_ptr<SinkBuffer>> sink_buffers);


/**
 * @brief Creates a StreamSink to receive multiple RTSP streams, specify size of the image queue as template parameter.
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateMultiRtspStreamSink(const std::vector<IpCameraSinkParams> &params)
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
