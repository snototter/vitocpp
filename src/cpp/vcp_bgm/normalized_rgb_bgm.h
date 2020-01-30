#ifndef __VCP_BGM_NORMALIZED_RGB_BGM_H__
#define __VCP_BGM_NORMALIZED_RGB_BGM_H__

#include "background_model.h"
#include <vector>
#include <memory>

namespace vcp
{
namespace bgm
{
/**
  * @brief Background subtraction based on intensity normalized RGB values.
  *
  * This class is the C++ port of the Matlab implementation by Straka
  * and Reinbacher. This CPU port is pretty slow - if required for a real
  * application, implement on the GPU.
  * See also "Fast variational multi-view segmentation through backprojection
  * of spatial constraints". Reinbacher, Rüther, Bischof. Image and Vision
  * Computing 30 (2012), pp. 797--807.
  */
struct NormalizedRgbBgmParams : BgmParams
{
  bool apply_threshold; /**< Flag indicating, whether the reported foreground shall be thresholded (binary segmentation) or not (floating point matrix). */
  float threshold;      /**< The threshold to use, if a binary segmentation result is preferred. */
  float learning_rate;  /**< Update rate of the background model, ie <code>bgm = (1-learning_rate) * old_bgm + learning_rate * update_frame</code>. */
  float alpha;          /**< Prevents division by zero. */
  float beta;           /**< Weighting factor for the intensity normalizer term. */

  NormalizedRgbBgmParams(
      bool apply_threshold = false,
      float threshold = 0.15f,
      float learning_rate = 0.05f,
      float alpha = 0.1f,
      float beta = 1.0f)
    : BgmParams(),
      apply_threshold(apply_threshold),
      threshold(threshold),
      learning_rate(learning_rate),
      alpha(alpha),
      beta(beta)
  {}
};

//TODO parse from config

std::unique_ptr<BackgroundModel> CreateNormalizedRgbBgm(const NormalizedRgbBgmParams &params);
} // namespace bgm
} // namespace vcp

#endif // __VCP_BGM_NORMALIZED_RGB_BGM_H__
