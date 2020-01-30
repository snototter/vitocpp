#ifndef __VCP_BGM_APPROX_MEDIAN_BGM_H__
#define __VCP_BGM_APPROX_MEDIAN_BGM_H__

#include "background_model.h"
#include <memory>

namespace vcp
{
namespace bgm
{
/**
 * @brief Parametrization for an adaptive approximate median background model.
 *
 * Based on "Segmentation and Tracking of Piglets in Images". McFarlane and
 * Schofield, Machine Vision and Applications Vol. 8(3), pp. 187--193, 1995.
 */
struct ApproxMedianBgmParams : BgmParams
{
  float adaption_step;
  float threshold;

  ApproxMedianBgmParams(
      float adaption_step = 5.0f,
      float threshold = 20.0f)
    : BgmParams(),
      adaption_step(adaption_step),
      threshold(threshold)
  {}
};


//TODO parse from config

//TODO doc, #1 converts to gray, #2 works on rgb and then reduces the mask
std::unique_ptr<BackgroundModel> CreateApproxMedianBgmGrayscale(const ApproxMedianBgmParams &params);
std::unique_ptr<BackgroundModel> CreateApproxMedianBgmColor(const ApproxMedianBgmParams &params);
} // namespace bgm
} // namespace vcp

#endif // __VCP_BGM_APPROX_MEDIAN_BGM_H__
