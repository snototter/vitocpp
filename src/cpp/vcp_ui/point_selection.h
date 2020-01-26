#ifndef __VCP_UI_POINT_SELECTION_H__
#define __VCP_UI_POINT_SELECTION_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace vcp
{
namespace ui
{
/**
 * @brief Provides a highgui window to let the user select a single point.
 *
 * User interaction:
 *   - Left mouse button: selects the current mouse position
 *   - Middle mouse button: aborts (and discards) the selection
 *   - Right mouse button: confirms the selection
 *   - Keyboard:
 *     -# ESC Abort (and discard) the selection and close window
 *     -# <code>'q'</code>/<code>'Q'</code>/<code>'c'</code>/<code>'C'</code>/'return': confirm selection and close window
 *     -# <code>'h'</code>/<code>'H'</code>: display help dialog
 *
 * @param point Point to store the user selection
 * @param image Input image
 * @param window_name Window title
 * @return bool indicating whether the user selected (and confirmed) a valid point (<tt>true</tt>) or not.
 */
bool SelectPoint(cv::Point &point, const cv::Mat &image, const cv::Scalar &point_color=cv::Scalar(255, 0, 255), int point_radius=5, const std::string &window_name = std::string("Select point"));

/**
 * @brief Provides a highgui window to let the user select multiple points.
 *
 * User interaction:
 *   - Left mouse button: adds the current mouse position to the output list
 *   - Middle mouse button: discards the point closest to the cursor
 *   - Right mouse button: confirm selection and exit
 *   - Keyboard:
 *     -# ESC: discard all points and exit
 *     -# Return, <code>'q'</code>/<code>'Q'</code>/<code>'c'</code>/<code>'C'</code>: confirm selection and exit
 *     -# <code>'r'</code>: discards the last stored point
 *     -# <code>'h'</code>/<code>'H'</code>: displays a help dialog at the console
 *
 * @param image Input image
 * @param point_color Color to draw selected points
 * @param point_radius Radius of drawn points
 * @param window_name Window title
 * @return Vector of selected points
 */
std::vector<cv::Point> SelectPoints(const cv::Mat &image, const cv::Scalar &point_color=cv::Scalar(255, 0, 255), int point_radius=5, const std::string &window_name = std::string("Select points"));


/** @brief Returns a short help message on how to use the single point selection tool. */
std::string PointSelectionUsage();

/** @brief Returns a short help message on how to use the multiple point selection tool. */
std::string MultiplePointsSelectionUsage();

} // namespace ui
} // namespace vcp
#endif //__VCP_UI_POINT_SELECTION_H__
