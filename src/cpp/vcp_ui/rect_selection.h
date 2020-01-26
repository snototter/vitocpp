#ifndef __VCP_UI_RECT_SELECTION_H__
#define __VCP_UI_RECT_SELECTION_H__

#include <opencv2/core/core.hpp>

namespace vcp
{
/**
* @brief Utilities for user interaction.
*/
namespace ui
{
/**
 * @brief Provides a highgui window to let the user select a rectangular region.
 *
 * User interaction:
 *   - Left mouse button down: start drawing the rectangle from the current position
 *   - Mouse move (while LMB pressed): draw/resize the rectangle
 *   - Left mouse button up: finish rectangle selection
 *   - Middle mouse button up: abort selection
 *   - Right mouse button: confirm selection, i.e. close the UI
 *   - Keyboard:
 *     -# <code>'q'</code>/<code>'Q'</code>/<code>'c'</code>/<code>'C'</code>/'return': quit and confirm selection, i.e. close the UI
 *     -# ESC: abort selection, i.e. close UI and return invalid rectangle
 *     -# <code>'h'</code>/<code>'H'</code>: displays a help dialog at the console
 *     -# <code>'r'</code>/<code>'R'</code>: Resets the rectangle
 *
 *
 * @param rectangle Rectangle to store the user selection
 * @param image Input image
 * @param color Color of the visualized rectangle
 * @param window_name Window title
 * @return bool indicating whether the user selected (and confirmed) a valid rectangle (<tt>true</tt>), or <tt>false</tt>
 *   if an invalid rectangle has been selected (resp. the user aborted the selection)
 */
bool SelectRectangle(cv::Rect &rectangle, const cv::Mat &image, const cv::Scalar color = cv::Scalar::all(255), const std::string &window_name = std::string("Select rectangular region"));

/** @brief Returns a short help message on how to use this tool. */
std::string RectSelectionUsage();
} // namespace ui
} // namespace vcp
#endif //__VCP_UI_RECT_SELECTION_H__

