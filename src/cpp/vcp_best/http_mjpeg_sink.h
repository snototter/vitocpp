#ifndef __VCP_BEST_HTTP_MJPEG_SINK_H__
#define __VCP_BEST_HTTP_MJPEG_SINK_H__

#include "sink.h"
#include "sink_buffer.h"
#include "ipcam_sink.h"

namespace vcp
{
namespace best
{
namespace ipcam
{
/** @brief TODO doc. */
namespace http
{
/**
 * @brief Creates a StreamSink to receive MJPEG over HTTP.
 */
std::unique_ptr<StreamSink> CreateBufferedHttpMjpegSink(const IpCameraSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer);

/**
 * @brief Creates a StreamSink to receive MJPEG over HTTP, specify size of the image queue as template parameter.
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateHttpMjpegSink(const IpCameraSinkParams &params)
{
  return CreateBufferedHttpMjpegSink(params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace http
} // namespace ipcam
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_HTTP_MJPEG_SINK_H__
