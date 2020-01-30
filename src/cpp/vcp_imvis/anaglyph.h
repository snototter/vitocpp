#ifndef __VCP_IMVIS_ANAGLYPH_H__
#define __VCP_IMVIS_ANAGLYPH_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
namespace imvis
{
/** @brief TODO doc. */
namespace anaglyph
{
/** @brief Generates a red-cyan anaglyph.
 * Optionally flips the color channels (if the input is RGB, as OpenCV uses BGR).
 * Optionally provide a mask indicating invalid image regions (will be set to 0). */
void GenerateAnaglyph(const cv::Mat &left, const cv::Mat &right, cv::Mat &anaglyph, bool flip_color_channels=false, cv::Mat *invalid=nullptr);


/** @brief Use to adjust the stereo baseline and reduce any double vision issues you might be experiencing.
 * Negative offsets shift to the left/top, positive to the right/bottom.
 * If you provide an output mask (padded), it will highlight which image regions have been padded/are invalid after shifting.
 */
cv::Mat ShiftImage(const cv::Mat &img, int offset_x, int offset_y, cv::Mat *padded);
} // namespace anaglyph
} // namespace imvis
} // namespace vcp
#endif // __VCP_IMVIS_ANAGLYPH_H__
