#ifndef __VCP_IMUTILS_MATUTILS_H__
#define __VCP_IMUTILS_MATUTILS_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif
#include <vector>

namespace vcp
{
namespace imutils
{
/** @brief Horizontally concatenate two matrices. */
cv::Mat ColumnStack(const cv::Mat &A, const cv::Mat &B);


/** @brief Vertically concatenate two matrices. */
cv::Mat RowStack(const cv::Mat &A, const cv::Mat &B);


/** @brief Resize the given matrix and copy the original data (unless the new matrix is a shrinked version....).
 *
 * Computes:
 * * tmp = cv::Mat(new size)
 * * tmp[:] = value
 * * copy top-left region of 'mat' into top-left region of 'tmp'
 * * update reference: mat = tmp
 */
void ResizeMatrix(cv::Mat &mat, int rows, int cols, double value = 0.0);


/** @brief Utility to get a readable representation (e.g. "CV_8U", "CV_8UC2") of the OpenCV matrix depth and channels (to debug OpenCV matrices).
 * If you don't care about the number of channels, pass -1.
 */
std::string CVMatDepthToString(int depth, int channels=-1);


/** @brief Utility to get a readable representation (e.g. "CV_8UC1") of the OpenCV matrix type. */
std::string CVMatTypeToString(const cv::Mat &mat);


/** @brief Save a mat to a binary file. */
bool DumpMat(const cv::Mat &mat, const std::string &filename, bool zip);


/** @brief Load a mat from a binary file. */
cv::Mat LoadMat(const std::string &filename);

} // namespace imutils
} // namespace vcp

#endif // __VCP_IMUTILS_MATUTILS_H__
