#ifndef __VCP_IMVIS_PSEUDOCOLOR_H__
#define __VCP_IMVIS_PSEUDOCOLOR_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

#include <iostream>

namespace vcp
{
namespace imvis
{
/** @brief Colormaps and utilities for pseudo-coloring. */
namespace pseudocolor
{
/**
 * @brief Available color maps for <code>PseudoColor</code> and <code>PseudoColorGPU</code> classes.
 */
enum class ColorMap
{
  Autumn,       /**< Red-yellow colormap, similar to MATLAB's autumn. */
  Bone,         /**< Black-blue-white colormap, similar to MATLAB's bone. */
  Cold,         /**< Black-blue-cyan-white colormap. */
  Disparity,    /**< High contrast colormap with subtle gradient discontinuities (for depth/disparity images). */
  Earth,        /**< Black-green-white colormap, linear grayscale changes when printed black/white. */
  Gray,         /**< Convert to grayscale. */
  Hot,          /**< Black-red-yellow-white colormap, similar to MATLAB's hot. */
  HSV,          /**< Red-yellow-green-cyan-blue-magenta-red colormap. */
  Inferno,      /**< Perceptually uniform. */
  Jet,          /**< MATLAB's default color map up until R2015, high contrast but otherwise bad. */
  Magma,        /**< Perceptually uniform. */
  Parula,       /**< MATLAB's default color map as of R2015, perceptually uniform. */
  Pastel,       /**< Black-pastel-white colormap, linear grayscale changes when printed black/white. */
  Plasma,       /**< Perceptually uniform. */
  Sepia,        /**< Black-brown-white colormap, perceptually uniform. */
  Temperature,  /**< Blue-pale-dark red colormap, for visualizing data like temperature (good contrast for colorblind viewers). */
  Thermal,      /**< Black-purple-red-yellow-white colormap. */
  Turbo,        /**< An improved rainbow colormap, similar to (but smoother than) Jet. */
  Viridis      /**< Perceptually uniform, default of matplotlib. */
};

/** Overload the stream operator '<<' for the colormap enumeration. */
std::ostream& operator<<(std::ostream & os, const ColorMap &cm);

/**
 * @brief Creates a color mapped image of the given (single channel) value matrix.
 * @param values single channel data matrix used for color mapping
 * @param colormap color map to use
 * @param colored output image, will be of the same depth with 3 channels
 * @param limit_from [optional] values less than limit_from will be set to this limit
 * @param limit_to [optional] values larger than limit_to will be set to this limit
 */
void Colorize(const cv::Mat &values, const ColorMap &colormap, cv::Mat &colored, double limit_from = 0.0, double limit_to = 0.0);

/** @brief Look up a single color from the color map. */
cv::Scalar GetPseudocolor(double value, const ColorMap &colormap, double limit_from = 0.0, double limit_to = 1.0);

/** @brief Highlight the masked region. If highlight color is negative, show unmasked area in grayscale; otherwise, overlay the color with color_opacity. */
cv::Mat Highlight(const cv::Mat &image, const cv::Mat &mask, const cv::Scalar &highlight = cv::Scalar::all(-1.0), double color_opacity=0.5);
} // namespace pseudocolor
} // namespace imvis
} // namespace vcp

#endif // __VCP_IMVIS_PSEUDOCOLOR_H__
