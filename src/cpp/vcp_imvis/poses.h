#ifndef __VCP_IMVIS_POSES_H__
#define __VCP_IMVIS_POSES_H__

#include <vector>
#include <string>
#include <vcp_imutils/opencv_compatibility.h>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

namespace vcp
{
/** @brief Visualization utilities. */
namespace imvis
{
/** @brief Drawing poses. */
namespace poses
{
/** Supported pose estimation models. **/
enum class PoseType : unsigned char
{
  COCO17=0
};

extern const std::vector<std::string> kCOCO17Keypoints;
extern const std::vector<std::pair<size_t, size_t>> kCOCO17Skeleton;
extern const std::vector<cv::Scalar> kCOCO17Colors;


struct PoseModel
{
  PoseType type;
  std::vector<cv::Vec2f> keypoints;
  std::vector<float> scores;

  /** @brief Convert the model's keypoints to pixel coordinates (truncates!). */
  std::vector<cv::Point> KeypointPixelCoords() const;
};

//TODO how to represent a pose?
// vector<vector<float>>, each entry: x, y, score
// enum for supported pose models
// GetPoseColor(model)
// Flag fast (lines) vs beautification (ellipses)
// ellipse angle computation: https://stackoverflow.com/questions/2676719/calculating-the-angle-between-the-line-defined-by-two-points
//void DrawPose(cv::Mat &image, const std::vector<cv::Point> &polygon, const cv::Scalar &color, int line_width=1, int dash_length=-1, double fill_opacity=0.0);
void DrawPose();//cv::Mat &image);

} // namespace poses
} // namespace imvis
} // namespace vcp
#endif // __VCP_IMVIS_POSES_H__
