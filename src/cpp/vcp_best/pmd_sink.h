#ifndef __VCP_BEST_PMD_SINK_H__
#define __VCP_BEST_PMD_SINK_H__

#include "sink.h"
#include <vcp_utils/enum_utils.h>

namespace vcp
{
namespace best
{
/** @brief PMD sensor streaming. */
namespace pmd
{


/** @brief Configuration parameters. */
struct PmdSinkParams : public SinkParams
{
  std::string serial_number;
  float gray_divisor;

  PmdSinkParams(
      const SinkParams &sink_params,
      const std::string &sn = std::string(),
      float gray_div = 180.0f)
    : SinkParams(sink_params),
      serial_number(sn),
      gray_divisor(gray_div)
  {}
};

std::ostream &operator<< (std::ostream &out, const PmdSinkParams &p);


/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a PMD camera. */
bool IsPmdSink(const std::string &type_param);


/** @brief Parse a PMD camera configuration from the config group "cam_param". */
PmdSinkParams PmdSinkParamsFromConfig(
    const vcp::config::ConfigParams &config, const std::string &cam_param);


struct PmdDeviceInfo
{
  std::string serial_number; /**< Serial number of the device. */
};


std::vector<PmdDeviceInfo> ListPmdDevices(bool warn_if_no_devices=true);

/** @brief Returns a StreamSink to access a PMD cam. Use the templated @see CreatePMDStereoSink(). */
std::unique_ptr<StreamSink> CreateBufferedPmdSink(
    const PmdSinkParams &params,
    std::unique_ptr<SinkBuffer> sink_buffer_gray,
    std::unique_ptr<SinkBuffer> sink_buffer_depth,
    std::unique_ptr<SinkBuffer> sink_buffer_xyz);


/** @brief Returns a StreamSink to access a PMD cam. */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreatePmdSink(const PmdSinkParams &params)
{
  return CreateBufferedPmdSink(
        params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
        std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
        std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // namespace pmd
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_PMD_SINK_H__
