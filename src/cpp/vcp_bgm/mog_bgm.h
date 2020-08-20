#ifndef __VCP_BGM_MOG_BGM_H__
#define __VCP_BGM_MOG_BGM_H__

#include "background_model.h"
#include <memory>

namespace vcp
{
namespace bgm
{
/**
 * @brief Parametrization for a Gaussian Mixture-based background subtraction model.
 *
 * Uses OpenCV's BackgroundSubtractorMOG2, which implements the approach proposed
 * in "Efficient adaptive density estimation per image pixel for the task of
 * background subtraction". Zivkovic and van der Heijden. Pattern Recognition Letters,
 * Vol. 27(7), pp. 773--780, 2006.
 */
struct MixtureOfGaussiansBgmParams : BgmParams
{
  int history; /**< Number of previous frames that affect the model. */
  bool detect_shadows; /**< Should shadows be detected? */
  double variance_threshold; /**< Variance threshold for the pixel-model match. */
  double complexity_reduction_threshold; /**< Complexity reduction (CT). */

  MixtureOfGaussiansBgmParams(
      int history=500,
      bool detect_shadows=true,
      double var_threshold=16.0,
      double comp_red_threshold=0.05)
    : BgmParams("GaussianMixture"),
      history(history),
      detect_shadows(detect_shadows),
      variance_threshold(var_threshold),
      complexity_reduction_threshold(comp_red_threshold)
  {}
};

/** @brief Create GMM-based background model, uses cv::BackgroundSubtractorMOG2. */
std::unique_ptr<BackgroundModel> CreateMixtureOfGaussiansBgm(const MixtureOfGaussiansBgmParams &params);
} // namespace bgm
} // namespace vcp

#endif // __VCP_BGM_MOG_BGM_H__
