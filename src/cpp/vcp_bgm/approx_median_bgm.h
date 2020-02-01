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
  float adaption_step; /**< The "learning rate" when updating the approximate median. */
  float threshold;     /**< Changes of magnitude > threshold will be considered foreground. */

  ApproxMedianBgmParams(
      float adaption_step = 5.0f,
      float threshold = 20.0f)
    : BgmParams("ApproxMedian"),
      adaption_step(adaption_step),
      threshold(threshold)
  {}
};



//TODO parse from config --> nice-to-have, maybe later.


/**
 * @brief Create an approximate median background model with channel reduction.
 *
 * Multi-channel input images are first reduced to grayscale before approximating
 * the median.
 */
std::unique_ptr<BackgroundModel> CreateApproxMedianBgmGrayscale(const ApproxMedianBgmParams &params);


/**
 * @brief Create an approximate median background model with separate channel handling.
 *
 * The median is approximated for each image channel separately and then reduced
 * upon ReportChanges().
 */
std::unique_ptr<BackgroundModel> CreateApproxMedianBgmColor(const ApproxMedianBgmParams &params);
} // namespace bgm
} // namespace vcp

#endif // __VCP_BGM_APPROX_MEDIAN_BGM_H__
