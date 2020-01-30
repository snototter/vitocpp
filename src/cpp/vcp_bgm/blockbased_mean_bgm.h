#ifndef __VCP_BLOCKBASED_MEAN_BGM_H__
#define __VCP_BLOCKBASED_MEAN_BGM_H__

#include "background_model.h"
#include <memory>

namespace vcp
{
namespace bgm
{
/** @brief Supported channels by the block-based mean background model. */
enum class BlockBasedMeanBgmChannel
{
  GRAYSCALE, /**< Compute the bgm on grayscale-converted input. */
  SATURATION /**< Compute the bgm on saturation channel (HSV). */
};

/** @brief Parametrization for the block-based mean background model. */
struct BlockBasedMeanBgmParams : BgmParams
{
  cv::Size block_size;              /**< @brief Block/patch size. */
  float shift;                      /**< @brief Shift between patches, i.e. this defines the patch overlap. */
  float learning_rate;              /**< @brief Learning rate \f$\lambda\f$, how fast the bgm should adapt to the update image, \f$ \lambda \in [0,1]\f$. */
  float report_threshold;           /**< @brief Threshold s.t. patches will only be reported if their (absolute) difference to the background (mean) model is above this threshold. */
  BlockBasedMeanBgmChannel channel; /**< @brief On which channel the bgm will be computed. */

  BlockBasedMeanBgmParams(
      cv::Size block_size=cv::Size(32, 32),
      float shift=0.25f,
      float learning_rate=0.05f,
      float report_threshold=5.0f,
      BlockBasedMeanBgmChannel channel=BlockBasedMeanBgmChannel::GRAYSCALE)
    : BgmParams(),
      block_size(block_size),
      shift(shift),
      learning_rate(learning_rate),
      report_threshold(report_threshold),
      channel(channel)
  {}
};


/** @brief Create a normalized RGB background model. */
std::unique_ptr<BackgroundModel> CreateBlockBasedMeanBgm(const BlockBasedMeanBgmParams &params);

} // namespace bgm
} // namespace vcp

#endif // __VCP_BLOCKBASED_MEAN_BGM_H__
