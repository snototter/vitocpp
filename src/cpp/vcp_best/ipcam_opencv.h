#ifndef __VCP_BEST_IPCAM_OPENCV_H__
#define __VCP_BEST_IPCAM_OPENCV_H__

#include "sink.h"
#include "sink_buffer.h"
#include "ipcam_sink.h"

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace videocap
{
std::unique_ptr<StreamSink> CreateBufferedOpenCVIpCamSink(
    const IpCameraSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer);

template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateOpenCVIpCamSink(
    const IpCameraSinkParams &params)
{
  return CreateBufferedOpenCVIpCamSink(
        params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace videocap
} // namespace http
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_IPCAM_OPENCV_H__
