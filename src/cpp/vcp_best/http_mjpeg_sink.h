#ifndef __VCP_BEST_HTTP_MJPEG_SINK_H__
#define __VCP_BEST_HTTP_MJPEG_SINK_H__

#include "sink.h"
#include "sink_buffer.h"

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace http
{
/**
 * @brief Creates a StreamSink to receive MJPEG over HTTP.
 * @param stream_url Full URL (including parameters, authentication, etc.)
 * @param sink_buffer A sink buffer which will be used as image queue.
 */
std::unique_ptr<StreamSink> CreateBufferedHttpMjpegSink(const std::string &stream_url, std::unique_ptr<SinkBuffer> sink_buffer);

/**
 * @brief Creates a StreamSink to receive MJPEG over HTTP, specify size of the image queue as template parameter.
 * @param stream_url Full URL (including parameters, authentication, etc.)
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateHttpMjpegSink(const std::string &stream_url)
{
  return CreateBufferedHttpMjpegSink(stream_url, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace http
} // namespace ipcam
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_HTTP_MJPEG_SINK_H__
